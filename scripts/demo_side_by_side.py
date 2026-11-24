#!/usr/bin/env python3
"""Synchronized replay for presentations: RViz next to the phi-r and phi-S plots.

Runs inside the iwalk_dwal image, started by scripts/demo_side_by_side.sh.

  compare: two recorded runs, each with its RViz and its phi-r / phi-S plots. Time 0 is the
           first reference command of each run. After that the robots are at different
           places, so the two plots do not show the same scene.
  cuts:    one recorded run. The first bar row is what the run published; the other rows
           re-cluster the same frame offline (open loop, they did not drive the robot).

Keys (click the plot window first): Space play/pause, left/right 2 s, Home restart, q quit.
"""
import argparse
import atexit
import bisect
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import types

import numpy as np
import yaml

SCRIPTS = '/opt/dwal_ws/install/crowd_aware_simulation/share/crowd_aware_simulation/scripts'
sys.path.insert(0, SCRIPTS)
import dwal_bag  # noqa: E402
from offline_metrics import chord, curv2phi, free_variable  # noqa: E402

RVIZ_BASE = '/replay_rviz/dwal_dynamic.rviz'
INTENT, REFERENCE, ODOM = '/shared_controller/intent_diagnostics', '/reference_cmd', '/odom'
REPLAY_TOPICS = ['/tf', '/tf_static', '/scan', '/local_costmap/costmap', '/local_costmap/costmap_updates',
                 '/local_costmap/published_footprint', '/dwal_planner/sampled_pathMarkers',
                 '/dwal_planner/cluster_markers_far', '/shared_controller/path_shared', '/experiment/path',
                 '/experiment/actual_path', '/human_states', '/odom', '/dwal_planner/sampled_paths']
# Paths are traced by base_link. The robot front is this far ahead of it (front_extent 0.865 +
# footprint_padding 0.02, experiment.yaml), so a path ends this far before what stopped it.
FRONT_REACH = 0.885
# Same eight colors as the upstream cluster markers (setColor(id % 8)), so plot and RViz match.
PALETTE = [(1, 0, 0), (0, 1, 0), (.25, .45, 1), (1, 1, 0), (0, 1, 1), (1, 0, 1), (.5, .5, .2), (.8, .1, .4)]
PASS, BRAKE, MIXED = '#1D9E75', '#888780', '#EF9F27'
SEPARATION, MIN_SPAN, MARGIN, PASS_R = 5, 0.2, 0.05, 1.77  # experiment.yaml; pass label as in the offline screen
children = []


def phi_at(phi, level, radius):
    """The same path's phi at another radius (inverse of curv2phi at `level`)."""
    k = 2 * math.sin(phi) / level if abs(phi) < math.pi / 4 else math.copysign(1 / (level * math.cos(phi)), phi)
    return curv2phi(k, radius)


def front_segments(msg, reach=FRONT_REACH):
    """For every sampled path: (x, y) of its end and of the robot front at that end."""
    out = []
    for path in msg.paths:
        if path.poses:
            end = path.poses[-1]
            out.append((end.x, end.y, end.x + reach * math.cos(end.theta), end.y + reach * math.sin(end.theta)))
    return out


def load_run(path, label):
    _, cfg = dwal_bag.locate(path)
    names = cfg['topics']
    radius = float(cfg['reference_radius_m'])
    wanted = {names['sampled_paths'], names['far_diagnostics'], INTENT, REFERENCE, ODOM}
    frames, intents, odom = [], [], []
    start = None
    threshold = float(cfg.get('reference_motion_threshold_mps', .02))
    for now, topic, msg in dwal_bag.events(path, wanted):
        if topic == names['sampled_paths']:
            rows = sorted(dwal_bag.sample_geometry(msg, radius), key=lambda row: row['phi'])
            if not rows or len(msg.pose0) < 3:
                continue
            ends = [msg.paths[row['index']].poses[-1] for row in rows]
            frames.append(dict(t=now, index=np.array([row['index'] for row in rows]),
                               k=np.array([row['curvature'] for row in rows]),
                               phi=np.array([row['phi'] for row in rows]), r=np.array([row['r'] for row in rows]),
                               s=np.array([row['length'] for row in rows]), x=np.array([p.x for p in ends]),
                               y=np.array([p.y for p in ends]), codes=None, required=None))
        elif topic == names['far_diagnostics']:
            if frames and frames[-1]['codes'] is None:
                try:
                    data = json.loads(msg.data)
                except (ValueError, AttributeError):
                    continue
                codes = data.get('codes') or []
                frames[-1]['codes'] = np.array([codes[i] if i < len(codes) else -1 for i in frames[-1]['index']])
                required = data.get('required_length')
                frames[-1]['required'] = required if isinstance(required, (int, float)) and math.isfinite(required) else None
        elif topic == INTENT:
            try:
                intents.append((now, json.loads(msg.data)))
            except (ValueError, AttributeError):
                pass
        elif topic == REFERENCE:
            twist = dwal_bag.twist(msg)
            if start is None and math.hypot(twist.linear.x, twist.angular.z) > threshold:
                start = now
        else:
            odom.append((now, msg.pose.pose.position.x, msg.pose.pose.position.y))
    if not frames:
        raise SystemExit(f'No sampled paths in {path}')
    # A few frames have no diagnostics message. Hold the whole previous frame, because the set of
    # valid paths changes between frames and its labels cannot be applied to the new one.
    for i in range(1, len(frames)):
        if frames[i]['codes'] is None and frames[i - 1]['codes'] is not None:
            frames[i] = {**frames[i - 1], 't': frames[i]['t']}
    start = frames[0]['t'] if start is None else start
    goal = (cfg.get('reference_path_xy') or [None])[-1]
    reached = None
    if goal is not None:
        tolerance = float(cfg.get('goal_tolerance_m', .2))
        reached = next((t - start for t, x, y in odom if t >= start and math.hypot(x - goal[0], y - goal[1]) <= tolerance), None)
    points = [(x, y) for _, x, y in odom] + [tuple(p) for p in cfg.get('reference_path_xy') or []]
    return dict(path=path, label=label, cfg=cfg, radius=radius, frames=frames, times=[f['t'] for f in frames],
                intents=intents, intent_times=[x[0] for x in intents], t0=start, end=frames[-1]['t'] - start,
                reached=reached, bounds=(min(p[0] for p in points), max(p[0] for p in points),
                                         min(p[1] for p in points), max(p[1] for p in points)))


# --- Offline re-clustering of one frame (cuts view). Clusters are lists of positions in phi order.

def admitted(frame):
    if frame['required'] is None:
        return []
    paths = [{'curvature': float(k), 'length': float(s)} for k, s in zip(frame['k'], frame['s'])]
    return free_variable(paths, [p['length'] for p in paths], frame['required'], MARGIN)


def gap_runs(free):
    runs, current = [], []
    for k in free:
        if current and k - current[-1] > SEPARATION:
            runs.append(current)
            current = []
        current.append(k)
    return runs + ([current] if current else [])


def wide_enough(frame, part, radius):
    return chord(radius, frame['phi'][part[-1]] - frame['phi'][part[0]]) >= MIN_SPAN


def tau_parts(frame, tau):
    out, current = [], []
    for k in admitted(frame):
        if current and (k - current[-1] > SEPARATION or abs(frame['s'][k] - frame['s'][current[-1]]) > tau):
            out.append(current)
            current = []
        current.append(k)
    return out + ([current] if current else [])


def dbscan_parts(frame, eps, min_pts=3):
    free = admitted(frame)
    if not free:
        return []
    xy = np.c_[frame['x'][free], frame['y'][free]]
    near = np.sqrt(((xy[:, None] - xy[None]) ** 2).sum(-1)) <= eps
    core = near.sum(1) >= min_pts
    labels = [-1] * len(free)
    cluster = 0
    for i in range(len(free)):
        if labels[i] != -1 or not core[i]:
            continue
        labels[i], stack = cluster, [i]
        while stack:
            p = stack.pop()
            if core[p]:
                for q in np.flatnonzero(near[p]):
                    if labels[q] == -1:
                        labels[q] = cluster
                        stack.append(int(q))
        cluster += 1
    out, current, label = [], [], None
    for i, k in enumerate(free):  # the controller needs clusters contiguous in phi
        if current and (labels[i] != label or k - current[-1] > SEPARATION):
            if label != -1:
                out.append(current)
            current = []
        current.append(k)
        label = labels[i]
    if current and label != -1:
        out.append(current)
    return out


def segment_costs(y):
    """Least-squares error of the best line on every segment y[a:b]."""
    n = len(y)
    x = np.arange(n, dtype=float)
    sums = {m: np.r_[0, np.cumsum(v)] for m, v in dict(x=x, y=y, xx=x * x, xy=x * y, yy=y * y).items()}
    diff = {m: v[None, :] - v[:, None] for m, v in sums.items()}
    count = np.arange(n + 1)[None, :] - np.arange(n + 1)[:, None]
    with np.errstate(divide='ignore', invalid='ignore'):
        sxx = diff['xx'] - diff['x'] ** 2 / count
        sxy = diff['xy'] - diff['x'] * diff['y'] / count
        syy = diff['yy'] - diff['y'] ** 2 / count
        cost = syy - np.where(sxx > 1e-9, sxy ** 2 / sxx, 0.0)
    return np.nan_to_num(np.maximum(cost, 0.0))


def change_point_parts(frame, beta, min_len=3):
    """Optimal partitioning on r: minimize the sum of line errors + beta per cut."""
    out = []
    for run in gap_runs(admitted(frame)):
        n = len(run)
        if n < 2 * min_len:
            out.append(run)
            continue
        cost = segment_costs(frame['r'][run])
        best, last = np.full(n + 1, np.inf), np.zeros(n + 1, int)
        best[0] = -beta
        for end in range(min_len, n + 1):
            values = best[:end - min_len + 1] + cost[:end - min_len + 1, end] + beta
            i = int(np.argmin(values))
            best[end], last[end] = values[i], i
        cuts, end = [], n
        while last[end] > 0:
            end = last[end]
            cuts.append(int(end))
        starts = [0] + sorted(cuts)
        out += [run[a:b] for a, b in zip(starts, starts[1:] + [n])]
    return out


def recorded_parts(frame):
    """Published clusters by ID. A cluster may span a small gap of rejected paths."""
    if frame['codes'] is None:
        return []
    clusters = {}
    for position, value in enumerate(frame['codes']):
        if value >= 0:
            clusters.setdefault(int(value), []).append(position)
    return sorted(clusters.values(), key=lambda part: part[0])


# --- Processes and transport

def spawn(command, env, log):
    process = subprocess.Popen(command, env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    children.append(process)
    return process


def stop_children():
    for process in reversed(children):
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    deadline = time.monotonic() + 3
    for process in children:
        try:
            process.wait(max(.05, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def camera(left, right, bottom, top, width, height, mode, pad=1.7):
    """The same RViz camera for every window, framing the whole scene."""
    x, y = (left + right) / 2, (bottom + top) / 2
    span_x, span_y = right - left + 2 * pad, top - bottom + 2 * pad
    if mode == '2d':
        return {'Class': 'rviz_default_plugins/TopDownOrtho', 'Angle': 0.0, 'X': x, 'Y': y,
                'Scale': float(min(width / span_x, height / span_y)), 'Target Frame': '<Fixed Frame>'}
    pitch = 1.0
    half = math.tan(math.pi / 8)  # RViz vertical field of view: 45 degrees
    distance = 1.2 * max(span_y * math.sin(pitch) / 2 / half, span_x / 2 / (half * width / height))
    return {'Class': 'rviz_default_plugins/Orbit', 'Distance': float(distance), 'Pitch': pitch,
            'Yaw': 3 * math.pi / 2, 'Focal Point': {'X': float(x), 'Y': float(y), 'Z': 0.0},
            'Target Frame': '<Fixed Frame>'}


def rviz_config(run, slot, geometry, view, folder):
    x, y, width, height = geometry
    config = yaml.safe_load(Path(RVIZ_BASE).read_text())
    config['Panels'] = []
    manager = config['Visualization Manager']
    dynamic = str(run['cfg'].get('variant', '')).startswith('dynamic')
    for display in manager['Displays']:
        if (display.get('Topic') or {}).get('Value') == '/dwal_planner/sampled_pathMarkers':
            display['Enabled'] = not dynamic  # gray, not clustered: hidden for the dynamic method
    manager['Displays'].append({'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Humans', 'Enabled': True,
                                'Topic': {'Value': '/demo/humans'}})
    manager['Displays'].append({'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Robot front reach',
                                'Enabled': True, 'Topic': {'Value': '/demo/front_reach'}})
    manager['Views'] = {'Current': view}
    config['Window Geometry'] = {'X': x, 'Y': y, 'Width': width, 'Height': height, 'Hide Left Dock': True,
                                 'Hide Right Dock': True}
    name = ''.join(c if c.isalnum() else '_' for c in f'{slot}_{run["label"]}')
    target = Path(folder) / f'{name}.rviz'
    target.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=True))
    return target


class Domain:
    """One replay: its own ROS domain, player, robot model, RViz and control clients."""

    def __init__(self, run, domain_id, rviz, rate, folder):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from rosbag2_interfaces.srv import Pause, Resume, Seek
        from rosidl_runtime_py.utilities import get_message
        from visualization_msgs.msg import MarkerArray
        self.run, self.clock, self.domain_id = run, None, domain_id
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id))
        log = open(Path(folder) / f'domain_{domain_id}.log', 'wb')
        urdf = Path(run['path']) / 'run' / 'robot.urdf'
        if urdf.exists():
            spawn(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '-p',
                   'use_sim_time:=true', '-p', f'robot_description:={urdf.read_text()}'], env, log)
        metadata = yaml.safe_load((Path(run['path']) / 'bag' / 'metadata.yaml').read_text())
        present = {t['topic_metadata']['name'] for t in metadata['rosbag2_bagfile_information']['topics_with_message_count']}
        topics = [t for t in REPLAY_TOPICS if t in present]
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain_id)
        self.node = rclpy.create_node(f'demo_control_{domain_id}', context=self.context)
        self.node.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.humans = self.node.create_publisher(MarkerArray, '/demo/humans', 10)
        self.node.create_subscription(get_message('hunav_msgs/msg/Agents'), '/human_states', self.on_humans, 10)
        self.front_reach = self.node.create_publisher(MarkerArray, '/demo/front_reach', 10)
        self.node.create_subscription(get_message('dwal_planner/msg/SampledCluster'), '/dwal_planner/sampled_paths',
                                      self.on_sampled_paths, 10)
        self.pause_client = self.node.create_client(Pause, '/rosbag2_player/pause')
        self.resume_client = self.node.create_client(Resume, '/rosbag2_player/resume')
        self.seek_client = self.node.create_client(Seek, '/rosbag2_player/seek')
        self.types = dict(pause=Pause, resume=Resume, seek=Seek)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        spawn(['rviz2', '-d', str(rviz), '--ros-args', '-p', 'use_sim_time:=true'], env, log)
        deadline = time.monotonic() + 40
        while self.node.count_subscribers('/scan') == 0 and time.monotonic() < deadline:
            time.sleep(.2)
        spawn(['ros2', 'bag', 'play', str(Path(run['path']) / 'bag'), '--clock', '100', '--rate', str(rate), '-p',
               '--disable-keyboard-controls', '--topics', *topics], env, log)
        for client in (self.pause_client, self.resume_client, self.seek_client):
            if not client.wait_for_service(timeout_sec=25):
                raise SystemExit(f'Bag player of {run["label"]} did not start, see {log.name}')

    def on_clock(self, msg):
        self.clock = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_humans(self, msg):
        from visualization_msgs.msg import Marker, MarkerArray
        out = MarkerArray()
        for agent in msg.agents:
            marker = Marker()
            marker.header = msg.header
            marker.ns, marker.id, marker.type, marker.action = 'humans', int(agent.id), Marker.CYLINDER, Marker.ADD
            marker.pose.position.x, marker.pose.position.y = agent.position.position.x, agent.position.position.y
            # Low and see-through: a full-height cylinder hides the robot and the paths in 3D.
            marker.pose.position.z, marker.pose.orientation.w = .3, 1.0
            marker.scale.x = marker.scale.y = 2 * min(max(float(agent.radius), .2), .3)
            marker.scale.z = .6
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = .95, .45, .25, .6
            out.markers.append(marker)
        self.humans.publish(out)

    def on_sampled_paths(self, msg):
        """Display only: a light line from each path end to where the robot front would be."""
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker, MarkerArray
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.ns, marker.id, marker.type, marker.action = 'front_reach', 0, Marker.LINE_LIST, Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = .008
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = .85, .85, .85, .3
        for x0, y0, x1, y1 in front_segments(msg):
            marker.points.extend([Point(x=float(x0), y=float(y0), z=.02), Point(x=float(x1), y=float(y1), z=.02)])
        self.front_reach.publish(MarkerArray(markers=[marker]))

    def now(self):
        return None if self.clock is None else self.clock - self.run['t0']

    def pause(self):
        self.pause_client.call_async(self.types['pause'].Request())

    def resume(self):
        self.resume_client.call_async(self.types['resume'].Request())

    def seek(self, relative):
        request = self.types['seek'].Request()
        stamp = max(self.run['t0'] + relative, 0.0)
        request.time.sec, request.time.nanosec = int(stamp), int((stamp - int(stamp)) * 1e9)
        self.seek_client.call_async(request)


# --- Plots

def style_axis(ax, ylabel, top):
    ax.set_facecolor('#282828')
    ax.set_xlim(-90, 90)
    ax.set_ylim(0, top)
    ax.set_ylabel(ylabel, color='#d8d8d0', fontsize=9)
    ax.tick_params(colors='#b8b8b0', labelsize=8)
    ax.grid(color='#444444', lw=.5)
    for spine in ax.spines.values():
        spine.set_color('#555555')


class Panel:
    def __init__(self, figure, cell_r, cell_s, run):
        self.run = run
        self.ax_r = figure.add_subplot(cell_r)
        self.ax_s = figure.add_subplot(cell_s, sharex=self.ax_r)
        style_axis(self.ax_r, 'r [m]', 2.05)
        style_axis(self.ax_s, 'S [m]', 2.25)
        self.ax_r.tick_params(labelbottom=False)
        self.ax_s.set_xlabel('φ [deg]  (+ = αριστερά)', color='#d8d8d0', fontsize=9)
        self.parts = {}
        for name, ax in (('r', self.ax_r), ('s', self.ax_s)):
            line, = ax.plot([], [], color='#777777', lw=.8, zorder=1)
            band = ax.axvspan(0, 0, color='white', alpha=.13, zorder=0, visible=False)
            pick = ax.axvline(0, color='white', lw=1.6, zorder=4, visible=False)
            user = ax.axvline(0, color='#00C8FF', lw=1.3, ls='--', zorder=4, visible=False)
            clustered = ax.scatter([], [], s=13, zorder=3)
            rejected = ax.scatter([], [], s=15, marker='x', c='#E24B4A', linewidths=.8, zorder=2)
            other = ax.scatter([], [], s=11, marker='x', c='#999999', linewidths=.6, zorder=2)
            self.parts[name] = dict(line=line, band=band, pick=pick, user=user, clustered=clustered, rejected=rejected,
                                    other=other)
        self.required = self.ax_s.axhline(0, color='#E24B4A', ls='--', lw=1, visible=False)
        self.title = self.ax_r.set_title('', color='white', fontsize=10, loc='left')

    def intent(self, absolute):
        i = bisect.bisect_right(self.run['intent_times'], absolute) - 1
        if i < 0 or absolute - self.run['intent_times'][i] > .6:
            return None
        return self.run['intents'][i][1]

    def update(self, relative):
        run = self.run
        absolute = run['t0'] + relative
        i = max(0, bisect.bisect_right(run['times'], absolute) - 1)
        frame = run['frames'][i]
        phi = np.degrees(frame['phi'])
        codes = frame['codes'] if frame['codes'] is not None else -np.ones(len(phi), int)
        on, braking = codes >= 0, codes == -2
        rest = ~(on | braking)
        colors = np.array([PALETTE[int(c) % len(PALETTE)] + (1.0,) for c in codes[on]]).reshape(-1, 4)
        for name, values in (('r', frame['r']), ('s', frame['s'])):
            part = self.parts[name]
            part['line'].set_data(phi, values)
            part['clustered'].set_offsets(np.c_[phi[on], values[on]])
            part['clustered'].set_facecolor(colors)
            part['clustered'].set_edgecolor('none')
            part['rejected'].set_offsets(np.c_[phi[braking], values[braking]])
            part['other'].set_offsets(np.c_[phi[rest], values[rest]])
        self.required.set_visible(frame['required'] is not None)
        if frame['required'] is not None:
            self.required.set_ydata([frame['required']] * 2)
        intent, chosen, commanded = self.intent(absolute), None, None
        low = high = None
        if intent and isinstance(intent.get('k_star'), (int, float)) and math.isfinite(intent['k_star']):
            level = float(intent.get('level') or run['radius'])
            chosen = math.degrees(curv2phi(float(intent['k_star']), run['radius']))
            if isinstance(intent.get('phi_user'), (int, float)) and math.isfinite(intent['phi_user']):
                commanded = math.degrees(phi_at(float(intent['phi_user']), level, run['radius']))
            clusters, best = intent.get('clusters') or [], intent.get('map')
            if isinstance(best, int) and 0 <= best < len(clusters):
                low = math.degrees(phi_at(clusters[best]['phi_min'], level, run['radius']))
                high = math.degrees(phi_at(clusters[best]['phi_max'], level, run['radius']))
        for part in self.parts.values():
            part['pick'].set_visible(chosen is not None)
            part['user'].set_visible(commanded is not None)
            part['band'].set_visible(low is not None)
            if chosen is not None:
                part['pick'].set_xdata([chosen, chosen])
            if commanded is not None:
                part['user'].set_xdata([commanded, commanded])
            if low is not None:
                part['band'].set_xy([[low, 0], [low, 1], [high, 1], [high, 0], [low, 0]])
        count = len(set(int(c) for c in codes[on]))
        group = f' · ομάδα {intent["group"]}' if intent and intent.get('group') else ' · χωρίς επιλογή cluster'
        speed = f' · {intent["speed"]:.2f} m/s' if intent and isinstance(intent.get('speed'), (int, float)) else ''
        arrived = '  · ΣΤΟΧΟΣ' if run['reached'] is not None and relative >= run['reached'] else ''
        self.title.set_text(f'{run["label"]} · {count} far clusters{group}{speed}{arrived}')
        return frame


class CutBars:
    """One row per method on the same frame: green = only pass, gray = only brake, orange = mixed."""

    def __init__(self, figure, cell, run, shared, args):
        from matplotlib.collections import PolyCollection
        self.run = run
        self.rows = [(f'το run (τ={run["cfg"].get("length_jump_threshold")})', recorded_parts),
                     (f'τ = {args.tau}', lambda f: tau_parts(f, args.tau)),
                     (f'DBSCAN ε = {args.eps}', lambda f: dbscan_parts(f, args.eps)),
                     (f'change-point β = {args.beta}', lambda f: change_point_parts(f, args.beta))]
        self.ax = figure.add_subplot(cell, sharex=shared)
        self.ax.set_facecolor('#282828')
        self.ax.set_ylim(len(self.rows), 0)
        self.ax.set_yticks([i + .5 for i in range(len(self.rows))])
        self.ax.set_yticklabels([name for name, _ in self.rows], color='#e8e8e0', fontsize=8.5)
        self.ax.tick_params(axis='x', colors='#b8b8b0', labelsize=8)
        self.ax.set_xlabel('φ [deg]  ·  πράσινο: pass, γκρι: brake, πορτοκαλί: ανάμεικτο  (offline, open loop)',
                           color='#d8d8d0', fontsize=9)
        for spine in self.ax.spines.values():
            spine.set_color('#555555')
        self.bars = PolyCollection([], edgecolors='#282828', linewidths=1)
        self.ax.add_collection(self.bars)
        self.cache = {}

    def update(self, frame):
        key = frame['t']
        if key not in self.cache:
            verts, colors = [], []
            phi = np.degrees(frame['phi'])
            for row, (_, method) in enumerate(self.rows):
                for part in method(frame):
                    if method is not recorded_parts and not wide_enough(frame, part, self.run['radius']):
                        continue
                    a, b = phi[part[0]], phi[part[-1]]
                    if b - a < .4:
                        b = a + .4
                    verts.append([(a, row + .16), (b, row + .16), (b, row + .84), (a, row + .84)])
                    passing = frame['r'][part] >= PASS_R
                    colors.append(MIXED if passing.any() and (~passing).any() else (PASS if passing.all() else BRAKE))
            self.cache[key] = (verts, colors)
        verts, colors = self.cache[key]
        self.bars.set_verts(verts)
        self.bars.set_facecolor(colors)


class Demo:
    def __init__(self, args, runs, domains, geometry):
        import matplotlib
        matplotlib.use('TkAgg', force=True)
        for key in list(matplotlib.rcParams):
            if key.startswith('keymap.'):
                matplotlib.rcParams[key] = []
        matplotlib.rcParams['toolbar'] = 'None'
        import matplotlib.pyplot as plt
        from matplotlib.gridspec import GridSpec
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        from matplotlib.widgets import Button, Slider
        self.plt, self.args, self.runs, self.domains = plt, args, runs, domains
        x, y, width, height = geometry
        self.figure = plt.figure(figsize=(width / 100, height / 100), dpi=100, facecolor='#202020')
        window = self.figure.canvas.manager.window
        window.geometry(f'{width}x{height}+{x}+{y}')
        self.figure.canvas.manager.set_window_title('φ–r και φ–S')
        if args.mode == 'compare':
            grid = GridSpec(5, 1, height_ratios=[1, 1, .12, 1, 1], left=.075, right=.985, top=.92, bottom=.115, hspace=.11)
            self.panels = [Panel(self.figure, grid[0], grid[1], runs[0]), Panel(self.figure, grid[3], grid[4], runs[1])]
            self.cuts = None
        else:
            grid = GridSpec(3, 1, height_ratios=[1, 1, .8], left=.2, right=.985, top=.92, bottom=.135, hspace=.2)
            self.panels = [Panel(self.figure, grid[0], grid[1], runs[0])]
            self.panels[0].ax_s.set_xlabel('')
            self.panels[0].ax_s.tick_params(labelbottom=False)
            self.cuts = CutBars(self.figure, grid[2], runs[0], self.panels[0].ax_r, args)
        self.figure.legend(handles=[Line2D([], [], color='white', lw=1.6, label='επιλεγμένο path'),
                                    Patch(color='white', alpha=.2, label='επιλεγμένο cluster'),
                                    Line2D([], [], color='#00C8FF', lw=1.3, ls='--', label='εντολή χρήστη'),
                                    Line2D([], [], color='#E24B4A', lw=1, ls='--', label='απαιτούμενο S'),
                                    Line2D([], [], color='#E24B4A', marker='x', ls='none', label='απόρριψη (πέδηση)')],
                           loc='upper center', ncol=5, frameon=False, fontsize=8.5, labelcolor='#e8e8e0')
        self.t_min = max(0.0, args.start)
        self.t_max = max(self.t_min + .5, min(r['end'] for r in runs) - .3)
        if args.end is not None:
            self.t_max = max(self.t_min + .5, min(self.t_max, args.end))
        self.slider = Slider(self.figure.add_axes([.14, .035, .68, .022], facecolor='#333333'), 't [s]', 0, self.t_max,
                             valinit=self.t_min, color='#378ADD')
        self.slider.label.set_color('white')
        self.slider.valtext.set_color('white')
        self.slider.vline.set_visible(False)
        self.button = Button(self.figure.add_axes([.875, .025, .11, .04]), 'Play', color='#3a3a3a', hovercolor='#555555')
        self.button.label.set_color('white')
        self.playing, self.internal, self.hold, self.recorder, self.drawn = False, False, None, None, None
        self.slider.on_changed(self.on_slider)
        self.button.on_clicked(lambda _: self.toggle())
        self.figure.canvas.mpl_connect('key_press_event', self.on_key)
        self.figure.canvas.mpl_connect('close_event', lambda _: self.finish())
        self.timer = self.figure.canvas.new_timer(interval=80)
        self.timer.add_callback(self.tick)

    def seek(self, relative):
        relative = min(max(relative, 0.0), self.t_max)
        self.hold = (relative, time.monotonic())
        for domain in self.domains:
            domain.seek(relative)
        self.show(relative)

    def toggle(self):
        if self.playing:
            self.set_playing(False)
        else:
            if self.current() >= self.t_max - .05:
                self.seek(self.t_min)
            self.set_playing(True)

    def set_playing(self, value):
        self.playing = value
        for domain in self.domains:
            (domain.resume if value else domain.pause)()
        self.button.label.set_text('Pause' if value else 'Play')

    def current(self):
        now = self.domains[0].now()
        if self.hold is not None:
            if now is not None and abs(now - self.hold[0]) < .15 or time.monotonic() - self.hold[1] > 1.0:
                self.hold = None
            else:
                return self.hold[0]
        return self.slider.val if now is None else now

    def on_slider(self, value):
        if not self.internal:
            self.seek(value)

    def on_key(self, event):
        if event.key == ' ':
            self.toggle()
        elif event.key == 'left':
            self.seek(self.current() - 2)
        elif event.key == 'right':
            self.seek(self.current() + 2)
        elif event.key == 'home':
            self.seek(self.t_min)
        elif event.key == 'q':
            self.plt.close(self.figure)

    def show(self, relative):
        relative = min(max(relative, 0.0), self.t_max)
        if self.drawn is not None and abs(relative - self.drawn) < .02:
            return
        self.drawn = relative
        for index, panel in enumerate(self.panels):
            own = self.domains[index].now() if self.playing and self.hold is None else None
            frame = panel.update(relative if own is None else own)
            if self.cuts is not None:
                self.cuts.update(frame)
        self.internal = True
        self.slider.set_val(relative)
        self.internal = False
        self.figure.canvas.draw_idle()

    def tick(self):
        relative = self.current()
        if self.playing and len(self.domains) == 2 and self.hold is None:
            a, b = self.domains[0].now(), self.domains[1].now()
            if a is not None and b is not None and abs(a - b) > .2:
                self.domains[1].seek(a)
        self.show(relative)
        if self.playing and relative >= self.t_max:
            self.set_playing(False)
            if self.args.record:
                self.figure.canvas.get_tk_widget().after(600, lambda: self.plt.close(self.figure))
        return True

    def start_recording(self):
        window = self.figure.canvas.manager.window
        width, height = window.winfo_screenwidth(), window.winfo_screenheight()
        width, height = width - width % 2, height - height % 2
        target = Path('/demo_out') / self.args.record
        # fatal: x11grab reports one error when shared memory is unavailable in the container,
        # then continues without it.
        self.recorder = subprocess.Popen(['ffmpeg', '-y', '-loglevel', 'fatal', '-f', 'x11grab', '-framerate', '25',
                                          '-video_size', f'{width}x{height}', '-i', os.environ['DISPLAY'],
                                          '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '23', '-pix_fmt', 'yuv420p',
                                          str(target)], stdin=subprocess.PIPE)

    def finish(self):
        if self.recorder is not None and self.recorder.poll() is None:
            try:
                self.recorder.communicate(b'q', timeout=15)
            except subprocess.TimeoutExpired:
                self.recorder.kill()
            target = Path('/demo_out') / self.args.record
            owner = os.environ.get('HOST_UID_GID')
            if owner and target.exists():
                subprocess.run(['chown', owner, str(target)], check=False)

    def fit_canvas(self):
        # Matplotlib scales the figure with the Tk DPI after the window got its fixed size, and
        # no resize event follows, so the right edge would fall outside the window.
        widget = self.figure.canvas.get_tk_widget()
        width, height = widget.winfo_width(), widget.winfo_height()
        if width <= 1 or height <= 1:
            widget.after(100, self.fit_canvas)
            return
        self.figure.canvas.resize(types.SimpleNamespace(width=width, height=height))

    def run(self):
        self.seek(self.t_min)
        self.timer.start()
        self.figure.canvas.get_tk_widget().after(200, self.fit_canvas)
        if self.args.record:
            def begin():
                self.start_recording()
                self.figure.canvas.get_tk_widget().after(1200, lambda: self.set_playing(True))
            self.figure.canvas.get_tk_widget().after(2500, begin)
        self.plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mode', choices=('compare', 'cuts'))
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--labels', nargs='+')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--start', type=float, default=0.0)
    parser.add_argument('--end', type=float)
    parser.add_argument('--record')
    parser.add_argument('--view', choices=('3d', '2d'), default='3d')
    parser.add_argument('--tau', type=float, default=0.2)
    parser.add_argument('--eps', type=float, default=0.2)
    parser.add_argument('--beta', type=float, default=0.2)
    parser.add_argument('--top', type=int, default=32, help='height of the desktop top bar in pixels')
    parser.add_argument('--title-bar', type=int, default=38, help='height of a window title bar in pixels')
    args = parser.parse_args()
    if len(args.runs) != (2 if args.mode == 'compare' else 1):
        parser.error('compare needs two runs, cuts needs one')
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error('rate must be positive')
    labels = args.labels or [Path(p).name for p in args.runs]
    print('Loading the recorded runs ...', flush=True)
    runs = [load_run(p, label) for p, label in zip(args.runs, labels)]

    # A separate process: a Tk root destroyed here would leave errors in the plot window's Tk.
    size = 'import tkinter as t; r = t.Tk(); print(r.winfo_screenwidth(), r.winfo_screenheight())'
    screen_w, screen_h = map(int, subprocess.check_output([sys.executable, '-c', size]).split())
    usable = screen_h - args.top
    half = screen_w // 2
    if args.mode == 'compare':
        slot = usable // 2
        rviz = [(0, args.top + i * slot, half, slot - args.title_bar) for i in range(2)]
    else:
        rviz = [(0, args.top, half, usable - args.title_bar)]
    plot = (half, args.top, screen_w - half, usable - args.title_bar)
    left = min(r['bounds'][0] for r in runs)
    right = max(r['bounds'][1] for r in runs)
    bottom = min(r['bounds'][2] for r in runs)
    top = max(r['bounds'][3] for r in runs)
    # The RViz render area is smaller than its window (menu, tool bar, status bar).
    view = camera(left, right, bottom, top, rviz[0][2] - 60, rviz[0][3] - 112, args.view)

    folder = '/tmp/demo'
    os.makedirs(folder, exist_ok=True)
    atexit.register(stop_children)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    print('Starting RViz and the bag players ...', flush=True)
    domains = []
    for i, run in enumerate(runs):
        config = rviz_config(run, 'AB'[i], rviz[i], view, folder)
        domains.append(Domain(run, 77 + i, config, args.rate, folder))
    demo = Demo(args, runs, domains, plot)
    print('Ready. Click the plot window: Space play/pause, left/right 2 s, Home restart, q quit.', flush=True)
    demo.run()
    demo.finish()


if __name__ == '__main__':
    main()
