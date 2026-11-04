#!/usr/bin/env python3
"""Run metrics with explicit definitions and per-metric validity.

Collisions (obstacle / human), standstill while motion is requested, personal-zone
time and path length, human and obstacle distance (min / mean), motion smoothness
(mean / max / min absolute acceleration and jerk) and DWAL path diagnostic codes.
Used for crowd_aware simulation bags and scand_ws runs (evaluation.json decides the
motion source and the window).
"""
import argparse
import csv
import json
import math
from pathlib import Path
from dwal_bag import events, locate, sample_geometry, twist
from offline_metrics import Collector


class Metrics:
    def __init__(self, cfg, duration, start=None):
        self.cfg, self.duration = cfg, duration
        self.start = self.end = start
        self.last = None
        self.ref = self.odom = self.contact = None
        self.requested = self.stopped = self.known_motion = self.contact_coverage = 0.0
        self.reference_active = self.requested_observed = 0.0
        self.pairs = {}; self.episode_pairs = []
        self.reference_messages = self.motion_messages = self.contact_messages = 0

    def advance(self, now):
        if self.last is not None and now < self.last:
            raise ValueError('Simulation clock moved backwards inside this bag; split the runs')
        if self.start is not None and self.last is not None:
            lo, hi = max(self.last, self.start), min(now, self.start+self.duration)
            if hi > lo:
                # Odometry coverage is independent of whether a command is active.
                odom_until = (min(hi, self.odom[0]+self.cfg['odom_timeout_s'])
                              if self.odom is not None else lo)
                self.known_motion += max(0.0, odom_until-lo)
                if self.ref is not None:
                    ref_until = min(hi, self.ref[0]+self.cfg['reference_timeout_s'])
                    active_dt = max(0.0, ref_until-lo)
                    self.reference_active += active_dt
                    if abs(self.ref[1]) > self.cfg['reference_motion_threshold_mps']:
                        self.requested += active_dt
                        observed_dt = max(0.0, min(ref_until, odom_until)-lo)
                        self.requested_observed += observed_dt
                        if self.odom is not None and abs(self.odom[1]) < self.cfg['stationary_threshold_mps']:
                            self.stopped += observed_dt
                if self.contact is not None:
                    self.contact_coverage += max(0., min(hi,self.contact+self.cfg['contact_max_gap_s'])-lo)
        self.last = now
        if self.start is not None: self.end = max(self.start, min(now, self.start+self.duration))

    def reference(self, now, speed):
        self.reference_messages += 1
        self.ref = (now, speed)
        if self.start is None and abs(speed) > self.cfg['reference_motion_threshold_mps']:
            if self.odom is not None and now-self.odom[0] <= self.cfg['odom_timeout_s']:
                self.start = self.end = now

    def motion(self, now, speed):
        """Linear speed of the evaluated motion (odometry, or a command; see motion_source)."""
        self.motion_messages += 1
        if not math.isfinite(speed): return
        self.odom = (now, speed)

    def contacts(self, now, pairs):
        self.contact_messages += 1
        self.contact = now
        if self.start is None or now < self.start or now > self.start+self.duration: return
        robot = self.cfg.get('robot_model_name', 'iwalk')
        # Merge body/wheel/manifold contacts into one robot-object encounter.
        model_pairs = {tuple(sorted(name.split('::')[0] for name in pair)) for pair in pairs}
        for pair in model_pairs:
            if robot not in pair or pair[0] == pair[1]:
                continue
            if any(fragment in name for fragment in self.cfg['ignored_contact_name_fragments'] for name in pair):
                continue
            if pair not in self.pairs or now-self.pairs[pair] > self.cfg['contact_release_s']:
                self.episode_pairs.append(pair)
            self.pairs[pair] = now

    def collisions(self, human_names):
        """(total, obstacle, human) episodes. HuNav human collision models are '<agent name>_body'."""
        robot = self.cfg.get('robot_model_name', 'iwalk')
        bodies = {name+'_body' for name in human_names}
        human = sum(any(name in bodies for name in pair if name != robot) for pair in self.episode_pairs)
        return len(self.episode_pairs), len(self.episode_pairs)-human, human

    def result(self, human_names=()):
        elapsed = 0. if self.start is None else max(0., self.end-self.start)
        total, obstacle, human = self.collisions(human_names)
        contact_ok = elapsed > 0 and self.contact_coverage/elapsed >= self.cfg.get('minimum_coverage_fraction', .95)
        coverage_min = self.cfg.get('minimum_coverage_fraction', .95)
        request_coverage = self.requested_observed/self.requested if self.requested > 0 else None
        motion_ok = (elapsed > 0 and self.known_motion/elapsed >= coverage_min and
                     (request_coverage is None or request_coverage >= coverage_min))
        return {
            'condition': {k:self.cfg.get(k) for k in ('controller','variant','semantics','scenario','seed')},
            'duration_requested_s': self.duration, 'duration_observed_s': elapsed,
            'complete_window': elapsed >= self.duration-1e-6,
            'collision_episodes': total if contact_ok else None,
            'obstacle_collision_episodes': obstacle if contact_ok else None,
            'human_collision_episodes': human if contact_ok else None,
            'collision_definition': 'physical robot/object MODEL-pair episodes; ground-name filter; NOT inflated costmap overlap; human = HuNav <agent>_body model',
            'collision_status': 'contact stream coverage OK; geometry validation is a separate prerequisite' if contact_ok else 'unavailable/incomplete contact stream',
            'standstill_while_requested_s': self.stopped if motion_ok else None,
            'requested_motion_s': self.requested,
            'requested_motion_coverage_fraction': request_coverage,
            'reference_active_fraction': self.reference_active/elapsed if elapsed else 0.,
            'standstill_definition': 'Time stationary during unexpired nonzero linear reference; expired commands are inactive',
            'motion_coverage_fraction': self.known_motion/elapsed if elapsed else 0.,
            'contact_coverage_fraction': self.contact_coverage/elapsed if elapsed else 0.,
            'counts': {'reference':self.reference_messages,'motion':self.motion_messages,'contacts':self.contact_messages},
            'limits': ['Two metrics evaluate navigation outcomes, not clustering ground-truth accuracy.',
                       'Missing/stale odometry is excluded and coverage is reported separately.',
                       'Reference expires after reference_timeout_s; expired intervals are not motion requests.',
                       'Replay completion/count/timing checks remain required to validate reference delivery.',
                       'Compare only equal complete windows and identical reference schedules.']}


SAFETY_NOTE = ('Not computed for SCAND: the pose replays the Jackal recording (up to ~2 m/s) while the '
               'controller state is scaled to iWalk limits (0.3 m/s), so braking/safety quantities do not '
               'describe a feasible iWalk motion and no physical contacts exist. Only clustering, '
               'correspondence and selection metrics are meaningful.')
SAFETY_KEYS = ('personal_zone_time_s', 'personal_zone_fraction', 'personal_zone_path_m', 'personal_zone_path_fraction',
               'minimum_human_center_distance_m', 'mean_human_center_distance_m',
               'minimum_obstacle_distance_m', 'mean_obstacle_distance_m')


def suppress_safety(result):
    for key in ('collision_episodes', 'obstacle_collision_episodes', 'human_collision_episodes'):
        result[key] = None
    result['collision_status'] = SAFETY_NOTE
    extra = result.get('additional_metrics') or {}
    for key in SAFETY_KEYS:
        if key in extra: extra[key] = None
    proposed = extra.get('proposed_path') or {}
    for key in ('shared_clearance_min_m', 'shared_clearance_mean_m', 'human_clearance_min_m', 'human_clearance_mean_m'):
        if key in proposed: proposed[key] = None
    result['safety_metrics'] = SAFETY_NOTE


def evaluate_main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag');p.add_argument('--duration',type=float)
    p.add_argument('--output');p.add_argument('--metrics-config');a=p.parse_args();bag,cfg=locate(a.bag)
    # duration_s null (scand_ws): evaluate from the first motion request to the end of the bag.
    duration=a.duration if a.duration is not None else cfg['duration_s']
    if duration is not None and (not math.isfinite(duration) or duration<=0):p.error('duration must be positive')
    dest=Path(a.output) if a.output else bag.parent/'analysis_v3';dest.mkdir(parents=True,exist_ok=True)
    provenance=bag.parent/'reference_schedule.json'
    replay=json.loads(provenance.read_text()) if provenance.exists() else None
    start = replay.get('start_sim_s') if replay else None
    t=cfg['topics'];metric=Metrics(cfg,duration if duration is not None else math.inf,start=start)
    extra=AdditionalMetrics(cfg,load_settings(a.metrics_config))
    collector=Collector(cfg,bag.parent)
    # Motion for standstill/smoothness: executed odometry (simulation) or a command topic
    # (scand_ws: the robot replays the recording, so the controller output is evaluated).
    motion_source=cfg.get('motion_source','odom')
    motion_topic=t[motion_source]
    diagnostics={t[k] for k in ('far_diagnostics','near_diagnostics')}
    human_topic=t.get('human_states','/human_states')
    selected={t[k] for k in ('reference_cmd','odom','contacts','sampled_paths','clusters_far','clusters_near','scan')}
    proposals={t['shared_path']:'shared',t['human_path']:'human'}
    selected|={motion_topic,human_topic,*diagnostics,*proposals,t['selected_cmd'],'/tf_static'}
    selected|=collector.topics()
    from rosidl_runtime_py.convert import message_to_ordereddict
    with (dest/'samples.csv').open('w',newline='') as sf, (dest/'clusters.jsonl').open('w') as cf:
        fields=['time_s','index','curvature','phi','r','length'];writer=csv.DictWriter(sf,fieldnames=fields);writer.writeheader()
        for now,topic,msg in events(a.bag,selected):
            metric.advance(now)
            collector.event(now,topic,msg)
            if topic==t['odom']:
                if msg.header.frame_id != cfg.get('global_frame','odom'):
                    raise ValueError('Odometry frame differs from the configured global frame')
                extra.odometry(now,msg)
            if topic==motion_topic:
                velocity=msg.twist.twist if topic==t['odom'] else twist(msg)
                metric.motion(now,velocity.linear.x);extra.motion(now,velocity.linear.x,velocity.angular.z)
            if topic==t['selected_cmd']: extra.selected_command(now,msg)
            if topic==t['reference_cmd']:
                extra.command(now,msg)
                v=twist(msg).linear.x
                if math.isfinite(v):metric.reference(now,v)
            elif topic==human_topic: extra.human_states(now,msg)
            elif topic==t['scan']: extra.scan(now,msg)
            elif topic=='/tf_static': extra.tf_static(msg)
            elif topic in proposals: extra.proposal(now,proposals[topic],msg)
            elif topic==t['contacts']:
                metric.contacts(now,[(s.collision1_name,s.collision2_name) for s in msg.states])
            elif topic==t['sampled_paths']:
                for row in sample_geometry(msg,cfg['reference_radius_m']):writer.writerow({'time_s':now,**row})
            elif topic in diagnostics or topic in (t['clusters_far'],t['clusters_near']):
                if topic in diagnostics: extra.diagnostics(now,topic,msg)
                cf.write(json.dumps({'time_s':now,'topic':topic,'message':message_to_ordereddict(msg)})+'\n')
    if duration is None:
        duration=max(0.,metric.last-metric.start) if metric.start is not None else 0.
        metric.duration=duration
        if metric.start is not None: metric.end=metric.last
    result=metric.result(extra.human_names)
    result['motion_source']=motion_topic
    result['reference_schedule']=replay
    result['window_start_sim_s']=metric.start
    result['window_source']='replay start' if start is not None else 'first nonzero reference (legacy/pilot)'
    result['reference_count_matches_replay']=bool(replay and metric.reference_messages == replay.get('messages', -2)+1)
    result['comparison_ready']=bool(metric.requested > 0 and result['reference_count_matches_replay'] and replay and replay.get('completed') and replay.get('timing_valid') and
        replay.get('absolute_start') and result['complete_window'] and
        result['collision_episodes'] is not None and result['standstill_while_requested_s'] is not None)
    result['schema_version']=6
    result['additional_metrics'] = (extra.result(metric.start,duration,dest)
        if metric.start is not None and duration > 0 else {'all_metrics_available':False,'error':'No evaluation window'})
    if metric.start is not None and duration > 0:
        step=extra.settings['grid_step_s']
        grid=[metric.start+i*step for i in range(int(math.ceil(duration/step)))]
        result['offline']=collector.results(metric.start,duration,grid,extra)
    if str(cfg.get('scenario','')).startswith('scand:'):
        suppress_safety(result)
    result['all_metrics_comparison_ready'] = bool(result['comparison_ready'] and
        result['additional_metrics']['all_metrics_available'])
    result['limits'][0] = 'Navigation outcome metrics do not establish clustering ground-truth accuracy.'
    (dest/'metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps(result,indent=2,allow_nan=False))


"""Goal-free offline metrics. Uses bag receipt times, not exact sensor synchronization.

All thresholds and preprocessing settings are exported with every result.
"""
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path


def load_settings(path=None):
    if path:
        path = Path(path)
    else:
        here = Path(__file__).resolve()
        candidates = (
            here.parents[1] / 'config' / 'additional_metrics.json',
            here.parents[2] / 'share' / 'crowd_aware_simulation' / 'config' / 'additional_metrics.json',
        )
        path = next((candidate for candidate in candidates if candidate.exists()), candidates[0])
    cfg = json.loads(path.read_text())
    for key, value in cfg.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid additional metric setting: {key}')
    if int(cfg['smoothing_samples']) != cfg['smoothing_samples'] or cfg['smoothing_samples'] % 2 != 1:
        raise ValueError('smoothing_samples must be a positive odd integer')
    return cfg


def finite(*values):
    return all(math.isfinite(v) for v in values)


def frame(name):
    return name.lstrip('/')


class Series:
    def __init__(self, rows):
        self.rows = rows
        self.times = [r[0] for r in rows]

    def at(self, t, timeout):
        i = bisect.bisect_right(self.times, t+1e-9)-1
        return self.rows[i] if i >= 0 and t-self.rows[i][0] <= timeout+1e-9 else None


def smooth(values, width):
    half = width//2
    out = [None]*len(values)
    for i in range(half, len(values)-half):
        window = values[i-half:i+half+1]
        if all(x is not None for x in window): out[i] = sum(window)/width
    return out


def derivative(values, step):
    return [None if a is None or b is None else (b-a)/step
            for a,b in zip(values, values[1:])]


def absolute_stats(values, expected, minimum):
    """(mean, max, min) of |x| over valid samples, or Nones below minimum coverage."""
    valid = [abs(x) for x in values if x is not None and math.isfinite(x)]
    coverage = len(valid)/expected if expected > 0 else 0.
    if not valid or coverage < minimum: return None, None, None, coverage
    return sum(valid)/len(valid), max(valid), min(valid), coverage


def weighted_mean(values, weights):
    pairs = [(v,w) for v,w in zip(values, weights) if v is not None]
    total = sum(w for _,w in pairs)
    return sum(v*w for v,w in pairs)/total if total > 0 else None


def nearest(xy, centers):
    return min(math.hypot(xy[0]-x, xy[1]-y) for x,y in centers)


# DWAL per-path diagnostic codes (clustering diagnostics topics; see plot.py).
DIAGNOSTIC_CODES = {-1: 'invalid_or_filtered', -2: 'braking_rejected', -3: 'outside_slice', -4: 'min_span_rejected'}


def diagnostic_summary(rows, start, duration):
    """Mean share of sampled paths per code over diagnostics messages in the window."""
    rows = [codes for t,codes in rows if start <= t < start+duration and codes]
    shares = {name: 0. for name in ('clustered', *DIAGNOSTIC_CODES.values())}
    for codes in rows:
        shares['clustered'] += sum(c >= 0 for c in codes)/len(codes)
        for code, name in DIAGNOSTIC_CODES.items():
            shares[name] += sum(c == code for c in codes)/len(codes)
    return {
        'messages': len(rows),
        'mean_path_fraction': {k: v/len(rows) for k,v in shares.items()} if rows else None,
        'no_cluster_message_fraction': sum(not any(c >= 0 for c in codes) for codes in rows)/len(rows) if rows else None,
        'definition': 'Per diagnostics message: share of sampled paths with each code (>=0 clustered, -1 invalid/filtered, -2 braking L<S_required, -3 outside slice, -4 accepted but cluster dropped by min_cluster_span), averaged over messages.'}


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def compose(a, b):
    """Planar pose a∘b; poses are (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1], a[2]+b[2])


def clearance(path, points):
    """Smallest distance from any path point to any obstacle point (numpy arrays Nx2, Mx2)."""
    import numpy as np
    return float(np.sqrt(((path[:,None,:]-points[None,:,:])**2).sum(axis=2).min()))


def curvature(v, w, threshold):
    return w/v if abs(v) > threshold else None


class AdditionalMetrics:
    def __init__(self, cfg, settings):
        self.cfg, self.settings = cfg, settings
        self.odom = []; self.motion_rows = []; self.reference = []; self.humans = []
        self.scans = []; self.scan_points = []; self.diagnostic_rows = {}
        self.selected = []; self.proposals = {'shared': [], 'human': []}
        self.static = {}; self.base_frame = None
        self.human_names = set(); self.rejected_frames = {}

    def odometry(self, now, msg):
        p = msg.pose.pose.position
        if frame(msg.header.frame_id) != frame(self.cfg.get('global_frame','odom')):
            self.rejected_frames['odom'] = self.rejected_frames.get('odom',0)+1
            return
        self.base_frame = frame(msg.child_frame_id)
        yaw = yaw_of(msg.pose.pose.orientation)
        if finite(p.x,p.y,yaw): self.odom.append((now,p.x,p.y,yaw))

    def motion(self, now, v, w):
        if finite(v,w): self.motion_rows.append((now,v,w))

    def selected_command(self, now, msg):
        from dwal_bag import twist
        msg = twist(msg)
        if finite(msg.linear.x,msg.angular.z): self.selected.append((now,msg.linear.x,msg.angular.z))

    def tf_static(self, msg):
        for tr in msg.transforms:
            t = tr.transform.translation
            self.static[frame(tr.child_frame_id)] = (frame(tr.header.frame_id), (t.x, t.y, yaw_of(tr.transform.rotation)))

    def sensor_offset(self, sensor):
        """Planar pose of a sensor frame in the odometry child (base) frame, from /tf_static."""
        pose, current = (0., 0., 0.), sensor
        while current != self.base_frame:
            if current not in self.static: return None
            parent, edge = self.static[current]
            pose, current = compose(edge, pose), parent
        return pose

    def proposal(self, now, kind, msg):
        """Shared controller's chosen path ('shared') or the path of the user command ('human')."""
        import numpy as np
        if frame(msg.header.frame_id) != frame(self.cfg.get('global_frame','odom')): return
        points = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses], dtype=float).reshape(-1, 2)
        self.proposals[kind].append((now, points))

    def command(self, now, msg):
        from dwal_bag import twist
        msg = twist(msg)
        if finite(msg.linear.x,msg.angular.z):
            self.reference.append((now,msg.linear.x,msg.angular.z))

    def human_states(self, now, msg):
        if frame(msg.header.frame_id) != frame(self.cfg.get('global_frame','odom')):
            self.rejected_frames['human_states'] = self.rejected_frames.get('human_states',0)+1
            return
        centers = []
        for agent in msg.agents:
            p = agent.position.position
            if not finite(p.x,p.y): return  # invalid human is not an empty crowd
            centers.append((p.x,p.y))
            self.human_names.add(agent.name)
        self.humans.append((now,centers))

    def scan(self, now, msg):
        import numpy as np
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min+msg.angle_increment*np.arange(len(ranges))
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not valid.any(): return
        self.scans.append((now, float(ranges[valid].min())))
        r, a = ranges[valid], angles[valid]
        self.scan_points.append((now, frame(msg.header.frame_id), np.stack([r*np.cos(a), r*np.sin(a)], axis=1)))

    def points_at(self, t):
        """Latest scan (within scan_timeout) as obstacle points in the odometry frame."""
        import numpy as np
        scans, odom = Series(self.scan_points), Series(self.odom)
        b = scans.at(t, self.cfg.get('scan_timeout_s', 0.5))
        o = odom.at(b[0], self.cfg['odom_timeout_s']) if b is not None else None
        if o is None: return None
        offset = self.sensor_offset(b[1])
        if offset is None: return None
        x, y, yaw = compose(o[1:4], offset)
        c, s = math.cos(yaw), math.sin(yaw)
        return b[2] @ np.array([[c, s], [-s, c]]) + [x, y]

    def false_stop(self, grid):
        """Share of grid time where the selected command stops (< 0.01 m/s) while the recorded
        robot moves faster than 0.1 m/s (speed from consecutive recorded positions)."""
        selected, odom = Series(self.selected), self.odom
        ot = [o[0] for o in odom]
        hits = total = 0
        for t in grid:
            i = bisect.bisect_right(ot, t) - 1
            if i < 1 or t - ot[i] > self.cfg['odom_timeout_s']: continue
            a, b = odom[i-1], odom[i]
            if b[0] - a[0] <= 0: continue
            moving = math.hypot(b[1]-a[1], b[2]-a[2])/(b[0]-a[0]) > 0.1
            c = selected.at(t, self.cfg['reference_timeout_s'])
            if c is None or not moving: continue
            total += 1; hits += abs(c[1]) < 0.01
        return {'fraction': hits/total if total else None, 'moving_samples': total}

    def diagnostics(self, now, topic, msg):
        try: codes = [int(c) for c in json.loads(msg.data).get('codes', [])]
        except (ValueError, TypeError): return
        self.diagnostic_rows.setdefault(topic,[]).append((now,codes))

    def proposed_paths(self, times, start, duration, odom_timeout, scan_timeout):
        """Metrics of what the controller proposes rather than of the executed motion
        (the only controller-dependent ones when the robot replays a recording, scand_ws)."""
        cfg = self.cfg
        ref_timeout, threshold = cfg['reference_timeout_s'], cfg['reference_motion_threshold_mps']
        odom, scans = Series(self.odom), Series(self.scan_points)
        paths = {kind: Series(rows) for kind,rows in self.proposals.items()}
        offsets = {}
        clear = {'shared': [], 'human': []}; lengths = []; evaluated = 0
        for t in times:
            b = scans.at(t, scan_timeout)
            o = odom.at(b[0], odom_timeout) if b is not None else None
            if o is None: continue
            if b[1] not in offsets: offsets[b[1]] = self.sensor_offset(b[1])
            if offsets[b[1]] is None: continue
            # Scan points into the odometry frame, with the robot pose at the scan time.
            x, y, yaw = compose(o[1:4], offsets[b[1]])
            c, s = math.cos(yaw), math.sin(yaw)
            points = b[2] @ [[c, s], [-s, c]] + [x, y]
            sample = False
            for kind, series in paths.items():
                p = series.at(t, ref_timeout)
                if p is None or len(p[1]) == 0: continue
                sample = True
                clear[kind].append(clearance(p[1], points))
                if kind == 'shared':
                    lengths.append(float(sum(math.dist(u, v) for u, v in zip(p[1], p[1][1:]))))
            evaluated += sample
        reference, selected = Series(self.reference), Series(self.selected)
        k_diff = []; ratios = []
        for t in times:
            r, c = reference.at(t, ref_timeout), selected.at(t, ref_timeout)
            if r is None or c is None: continue
            kr, kc = curvature(r[1], r[2], threshold), curvature(c[1], c[2], threshold)
            if kr is not None and kc is not None: k_diff.append(abs(kc-kr))
            if r[1] > threshold: ratios.append(c[1]/r[1])
        stats = lambda values, f: f(values) if values else None
        mean = lambda values: sum(values)/len(values)
        return {
            'proposed_path': {
                'shared_clearance_min_m': stats(clear['shared'], min),
                'shared_clearance_mean_m': stats(clear['shared'], mean),
                'human_clearance_min_m': stats(clear['human'], min),
                'human_clearance_mean_m': stats(clear['human'], mean),
                'shared_length_mean_m': stats(lengths, mean),
                'shared_length_min_m': stats(lengths, min),
                'evaluated_grid_fraction': evaluated/len(times) if times else 0.,
                'definition': 'On the grid: latest /shared_controller/path_shared (controller choice) and path_human (user command arc), clearance = smallest distance from the path centerline to /scan points (laser pose from odometry + /tf_static). Length = drawn length of the chosen path (checked free length). Empty paths are excluded.'},
            'intervention': {
                'curvature_difference_mean_1pm': stats(k_diff, mean),
                'curvature_difference_max_1pm': stats(k_diff, max),
                'speed_ratio_mean': stats(ratios, mean),
                'samples': len(k_diff),
                'definition': 'Selected command vs reference command on the grid: |w/v selected - w/v reference| when both |v| exceed reference_motion_threshold_mps; speed ratio v_selected/v_reference when the reference moves forward.'}}

    def result(self, start, duration, dest):
        cfg,s = self.cfg,self.settings
        minimum = cfg.get('minimum_coverage_fraction',.95)
        odom_timeout, scan_timeout = cfg['odom_timeout_s'], cfg.get('scan_timeout_s', 0.5)
        zone_radius = s['personal_zone_center_distance_m']
        dt = s['grid_step_s']
        times = []; weights = []
        n = int(math.ceil(duration/dt))
        for i in range(n):
            times.append(start+i*dt)
            weights.append(min(dt,start+duration-times[-1]))
        odom,motion,human,ref,scan = (Series(rows) for rows in
            (self.odom,self.motion_rows,self.humans,self.reference,self.scans))
        velocities = []; angular = []; references = []; distances = []; obstacles = []
        human_coverage = active_human = zone = 0.
        for t,weight in zip(times,weights):
            o = odom.at(t,odom_timeout)
            m = motion.at(t,odom_timeout)
            h = human.at(t,s['human_timeout_s'])
            r = ref.at(t,cfg['reference_timeout_s'])
            b = scan.at(t,scan_timeout)
            velocities.append(m[1] if m else None)
            angular.append(m[2] if m else None)
            references.append(r)
            obstacles.append(b[1] if b else None)
            d = None
            if o is not None and h is not None:
                human_coverage += weight
                if h[1]:
                    active_human += weight
                    d = nearest(o[1:3],h[1])
                    if d < zone_radius: zone += weight
            distances.append(d)
        width = s['smoothing_samples']
        sv,sw = smooth(velocities,width),smooth(angular,width)
        av,aw = derivative(sv,dt),derivative(sw,dt)
        jv,jw = derivative(av,dt),derivative(aw,dt)
        smooth_results = {}
        for name,unit,values,expected in (
            ('linear_acceleration','mps2',av,n-width),
            ('angular_acceleration','radps2',aw,n-width),
            ('linear_jerk','mps3',jv,n-width-1),
            ('angular_jerk','radps3',jw,n-width-1)):
            mean,high,low,coverage = absolute_stats(values,expected,minimum)
            smooth_results[f'{name}_mean_abs_{unit}'] = mean
            smooth_results[f'{name}_max_abs_{unit}'] = high
            smooth_results[f'{name}_min_abs_{unit}'] = low
            smooth_results[f'{name}_mean_abs_{unit}_coverage_fraction'] = coverage
        exposure_coverage = human_coverage/duration
        human_ok = exposure_coverage >= minimum
        # Minimum over all observed human/odom update times, not only the grid.
        known_distances = []
        for t in sorted(set(odom.times+human.times)):
            if not start <= t < start+duration: continue
            o,h = odom.at(t,odom_timeout),human.at(t,s['human_timeout_s'])
            if o is not None and h is not None and h[1]:
                known_distances.append(nearest(o[1:3],h[1]))
        # Path length: consecutive odometry positions in the window (gaps > odom timeout skipped);
        # a segment is inside the personal zone when its end point is.
        path_length = zone_path = 0.
        window = [row for row in self.odom if start <= row[0] < start+duration]
        for a,b in zip(window,window[1:]):
            if b[0]-a[0] > odom_timeout: continue
            step = math.hypot(b[1]-a[1],b[2]-a[2])
            path_length += step
            h = human.at(b[0],s['human_timeout_s'])
            if h is not None and h[1] and nearest(b[1:3],h[1]) < zone_radius: zone_path += step
        scan_coverage = sum(w for w,b in zip(weights,obstacles) if b is not None)/duration
        scan_ok = scan_coverage >= minimum
        scan_minimum = [d for t,d in self.scans if start <= t < start+duration]
        proposed = self.proposed_paths(times, start, duration, odom_timeout, scan_timeout)
        diagnostics = {topic: diagnostic_summary(rows,start,duration)
                       for topic,rows in self.diagnostic_rows.items()}
        result = {
            'settings': s,
            'settings_sha256': hashlib.sha256(json.dumps(s,sort_keys=True).encode()).hexdigest(),
            'time_alignment': 'Uniform simulation-time grid; latest bag-received sample held only until timeout. No TF inference.',
            'personal_zone_time_s': zone if human_ok else None,
            'personal_zone_fraction': zone/human_coverage if human_ok and human_coverage else None,
            'path_length_m': path_length,
            'personal_zone_path_m': zone_path if human_ok else None,
            'personal_zone_path_fraction': zone_path/path_length if human_ok and path_length > 0 else None,
            'minimum_human_center_distance_m': min(known_distances) if human_ok and known_distances else None,
            'mean_human_center_distance_m': weighted_mean(distances,weights) if human_ok else None,
            'human_geometry_definition': 'Planar odometry-origin to HuNav-agent-center distance (nearest human). Minimum at all observed odom/human update times, mean over the grid while humans exist; NOT footprint clearance.',
            'human_joint_coverage_fraction': exposure_coverage,
            'observed_time_with_humans_s': active_human,
            'minimum_obstacle_distance_m': min(scan_minimum) if scan_ok and scan_minimum else None,
            'mean_obstacle_distance_m': weighted_mean(obstacles,weights) if scan_ok else None,
            'obstacle_distance_definition': 'Nearest valid /scan return from the laser origin (any object, humans included). Minimum over all scans, mean over the grid; scans with no return in range are excluded.',
            'scan_coverage_fraction': scan_coverage,
            'rejected_frame_messages': self.rejected_frames,
            'smoothness': smooth_results,
            'dwal_diagnostics': diagnostics,
            'proposed_path': proposed['proposed_path'],
            'intervention': proposed['intervention'],
            'limits': [
                'Higher clearance/lower proximity exposure is not automatically better in dense crowds.',
                'Lower acceleration/jerk alone can reward standing still; interpret with standstill and safety.',
                'Common downstream smoothing and collision monitoring can mask controller differences.',
                'No human samples means unavailable; valid empty crowd means zero exposure and undefined minimum distance.'
            ]}
        with (Path(dest)/'additional_timeline.csv').open('w',newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['time_s','dt_s','linear_velocity_mps','angular_velocity_radps','human_center_distance_m',
                             'obstacle_distance_m','reference_linear_mps','reference_angular_radps'])
            for t,w,v,a,d,b,r in zip(times,weights,velocities,angular,distances,obstacles,references):
                writer.writerow([t,w,v,a,d,b,r[1] if r else None,r[2] if r else None])
        result['all_metrics_available'] = bool(human_ok and known_distances and scan_ok and
            all(v is not None for k,v in smooth_results.items() if not k.endswith('coverage_fraction')))
        return result
#!/usr/bin/env python3
"""Export all metrics and validity flags without treating unavailable values as zero."""
import argparse
import csv
import json
from pathlib import Path


def flatten(obj, prefix=''):
    result = {}
    for key,value in obj.items():
        name = prefix+key
        if isinstance(value,dict): result.update(flatten(value,name+'.'))
        elif not isinstance(value,list): result[name]=value
    return result


def compare_main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('results',nargs='+',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    rows=[]
    for path in a.results:
        if path.is_dir():
            path = path/'analysis_v3'/'metrics.json'
        if path.name != 'metrics.json':
            raise SystemExit(f'comparison input must be metrics.json or a run directory: {path}')
        d=json.loads(path.read_text())
        row=flatten(d)
        row['source']=str(path)
        rows.append(row)
    fields=list(dict.fromkeys(k for row in rows for k in row))
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields)
        writer.writeheader();writer.writerows(rows)
    print(f'{len(rows)} runs -> {a.output}. Pair equal seed, reference SHA, window and additional settings SHA. Check per-metric coverage; ID continuity is a proxy.')

def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    evaluate = subparsers.add_parser('evaluate')
    evaluate.add_argument('bag')
    evaluate.add_argument('--duration', type=float)
    evaluate.add_argument('--output')
    evaluate.add_argument('--metrics-config')
    compare = subparsers.add_parser('compare')
    compare.add_argument('results', nargs='+', type=Path)
    compare.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'evaluate':
        import sys
        sys.argv = [sys.argv[0], args.bag]
        if args.duration is not None: sys.argv += ['--duration', str(args.duration)]
        if args.output is not None: sys.argv += ['--output', args.output]
        if args.metrics_config is not None: sys.argv += ['--metrics-config', args.metrics_config]
        evaluate_main()
    else:
        import sys
        sys.argv = [sys.argv[0], *[str(path) for path in args.results], '--output', str(args.output)]
        compare_main()

if __name__ == '__main__':
    main()
