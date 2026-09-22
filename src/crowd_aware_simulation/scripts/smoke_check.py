#!/usr/bin/env python3
"""Read-only runtime gate shared by all eight local experiment conditions."""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from cohan_msgs.msg import TrackedAgents
from crowd_aware_interfaces.msg import ContextState, DynamicRadiusState
from dwal_planner.msg import ClusterGroup, SampledCluster


def finite_twist(msg):
    return all(math.isfinite(value) for value in (
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z))


parser = argparse.ArgumentParser()
parser.add_argument('--controller',
                    choices=('fixed_dwal', 'dwb', 'hateb', 'dynamic_dwal'),
                    required=True)
parser.add_argument('--semantics', choices=('off', 'on'), required=True)
parser.add_argument('--timeout', type=float, default=45.0)
args = parser.parse_args()

rclpy.init()
node = Node('crowd_aware_smoke_check')
seen = {}
subscriptions = []
positions = []
selected_nonzero = False
controller_tracks_received = False
radius_values = []
sampled_outer_levels = []
task_status = ''


def odom_cb(msg):
    valid = msg.header.frame_id == 'odom' and msg.child_frame_id == 'sim_base'
    seen['odometry'] = seen.get('odometry', False) or valid
    if valid:
        positions.append((float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)))


def scan_cb(msg):
    valid = (msg.header.frame_id == 'laser_frame' and
             any(math.isfinite(value) and msg.range_min < value < msg.range_max
                 for value in msg.ranges))
    seen['lidar'] = seen.get('lidar', False) or valid


def costmap_cb(msg):
    valid = (msg.header.frame_id == 'odom' and msg.info.width > 0 and
             msg.info.height > 0 and any(value >= 100 for value in msg.data))
    seen['costmap'] = seen.get('costmap', False) or valid


def context_cb(msg):
    required = {'OPEN_AREA', 'NARROW_CORRIDOR', 'DOORWAY', 'JUNCTION',
                'DENSE_CROWD', 'OCCLUSION', 'UNKNOWN', 'STALE'}
    valid = (msg.header.frame_id == 'odom' and msg.valid and
             required.issubset(set(msg.labels)) and
             len(msg.labels) == len(msg.scores))
    seen['context'] = seen.get('context', False) or valid


def logging_tracks_cb(msg):
    seen['ground_truth_tracks'] = (
        seen.get('ground_truth_tracks', False) or
        (msg.header.frame_id == 'odom' and bool(msg.agents)))


def controller_tracks_cb(msg):
    global controller_tracks_received
    controller_tracks_received = (
        controller_tracks_received or
        (msg.header.frame_id == 'odom' and bool(msg.agents)))


def command_cb(key):
    def callback(msg):
        global selected_nonzero
        seen[key] = seen.get(key, False) or finite_twist(msg)
        if key == 'selected_command' and finite_twist(msg):
            selected_nonzero = selected_nonzero or (
                abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3)
    return callback


def task_cb(msg):
    global task_status
    task_status = msg.data
    seen['task_status'] = bool(msg.data)


def sampled_cb(msg):
    valid = bool(msg.paths) and any(path.poses for path in msg.paths)
    seen['sampled_paths'] = seen.get('sampled_paths', False) or valid
    if msg.levels:
        sampled_outer_levels.append(float(msg.levels[-1]))


def clusters_cb(_msg):
    # A blocked scene can legitimately yield zero clusters; fresh reception is the gate.
    seen['clusters'] = True


def radius_cb(msg):
    expected_modes = ('fixed',) if args.controller == 'fixed_dwal' else ('continuous', 'discrete')
    valid = (msg.header.frame_id == 'odom' and msg.valid and
             msg.adaptation_mode in expected_modes and
             math.isfinite(msg.applied_radius) and
             math.isfinite(msg.speed_limit))
    seen['radius_policy'] = seen.get('radius_policy', False) or valid
    if valid:
        radius_values.append(float(msg.applied_radius))


subscriptions += [
    node.create_subscription(Odometry, '/odom', odom_cb, qos_profile_sensor_data),
    node.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data),
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                             costmap_cb, qos_profile_sensor_data),
    node.create_subscription(ContextState, '/crowd_context',
                             context_cb, qos_profile_sensor_data),
    node.create_subscription(TrackedAgents, '/tracked_agents_logging',
                             logging_tracks_cb, qos_profile_sensor_data),
]
for key, topic in (
        ('selected_command', '/cmd_vel_selected'),
        ('guarded_command', '/cmd_vel_guarded'),
        ('smoothed_command', '/cmd_vel_smoothed'),
        ('collision_checked_command', '/cmd_vel_safe'),
        ('final_command', '/cmd_vel')):
    subscriptions.append(
        node.create_subscription(Twist, topic, command_cb(key), qos_profile_sensor_data))

latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscriptions.append(
    node.create_subscription(String, '/experiment/task_result', task_cb, latched))

if args.semantics == 'on':
    subscriptions.append(
        node.create_subscription(TrackedAgents, '/tracked_agents',
                                 controller_tracks_cb, qos_profile_sensor_data))

if args.controller in ('fixed_dwal', 'dynamic_dwal'):
    subscriptions += [
        node.create_subscription(SampledCluster, '/dwal_planner/sampled_paths',
                                 sampled_cb, qos_profile_sensor_data),
        node.create_subscription(ClusterGroup, '/dwal_planner/clusters_near',
                                 clusters_cb, qos_profile_sensor_data),
        node.create_subscription(DynamicRadiusState,
                                 '/experiment/dynamic_radius_state',
                                 radius_cb, qos_profile_sensor_data),
    ]

end = time.monotonic() + max(1.0, args.timeout)
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
    common_ready = all(seen.get(key, False) for key in (
        'odometry', 'lidar', 'costmap', 'context', 'ground_truth_tracks',
        'selected_command', 'guarded_command', 'smoothed_command',
        'collision_checked_command', 'final_command', 'task_status'))
    dwal_ready = (args.controller not in ('fixed_dwal', 'dynamic_dwal') or
                  all(seen.get(key, False) for key in
                      ('sampled_paths', 'clusters', 'radius_policy')))
    if common_ready and dwal_ready and selected_nonzero and len(positions) >= 2:
        moved = any(math.hypot(x, y) > 0.03 for x, y in positions)
        if moved:
            break


def lifecycle_state(service):
    client = node.create_client(GetState, service)
    if not client.wait_for_service(timeout_sec=2.0):
        return 'service unavailable'
    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done() or future.exception() is not None:
        return 'query failed'
    return future.result().current_state.label


checks = dict(seen)
checks['nonzero_selected_command'] = selected_nonzero
checks['robot_motion'] = any(math.hypot(x, y) > 0.03 for x, y in positions)
checks['velocity_smoother_active'] = (
    lifecycle_state('/velocity_smoother/get_state') == 'active')
checks['collision_monitor_active'] = (
    lifecycle_state('/collision_monitor/get_state') == 'active')
if args.controller in ('fixed_dwal', 'dynamic_dwal'):
    checks['costmap_active'] = (
        lifecycle_state('/costmap/costmap/get_state') == 'active')
    checks['reference_controller_active'] = (
        lifecycle_state('/reference/controller_server/get_state') == 'active')
else:
    checks['controller_active'] = (
        lifecycle_state('/controller_server/get_state') == 'active')

final_publishers = node.get_publishers_info_by_topic('/cmd_vel')
checks['single_final_publisher'] = (
    len(final_publishers) == 1 and
    final_publishers[0].node_name == 'final_cmd_watchdog')

track_publishers = node.get_publishers_info_by_topic('/tracked_agents')
track_subscribers = node.get_subscriptions_info_by_topic('/tracked_agents')
if args.controller in ('fixed_dwal', 'dynamic_dwal'):
    expected_consumers = {'shared_controller'}
elif args.controller == 'dwb':
    # CoHAN social layers are hosted by Nav2's local costmap node.
    expected_consumers = {'local_costmap'}
else:
    expected_consumers = {'controller_server'}
if args.semantics == 'on':
    checks['semantic_track_output'] = controller_tracks_received
    checks['semantic_controller_consumer'] = any(
        endpoint.node_name in expected_consumers
        for endpoint in track_subscribers)
else:
    checks['semantic_isolation_no_track_publisher'] = len(track_publishers) == 0

if args.controller == 'fixed_dwal':
    checks['fixed_radius_preserved'] = (
        bool(radius_values) and bool(sampled_outer_levels) and
        all(abs(value - 1.8) <= 0.03 for value in radius_values[-5:]) and
        all(abs(value - 1.8) <= 0.03 for value in sampled_outer_levels[-5:]))
elif args.controller == 'dynamic_dwal':
    checks['dynamic_radius_changed'] = (
        bool(radius_values) and any(abs(value - 1.8) > 0.03 for value in radius_values))
    checks['dynamic_radius_propagated'] = (
        bool(radius_values) and bool(sampled_outer_levels) and
        any(abs(radius - level) <= 0.06
            for radius in radius_values for level in sampled_outer_levels))

print(f'condition: {args.controller}/{args.semantics}')
print(f'last_task_status: {task_status or "none"}')
if radius_values:
    print(f'observed_applied_radius_range: {min(radius_values):.3f}..{max(radius_values):.3f}')
if sampled_outer_levels:
    print('observed_generator_outer_levels: ' +
          ','.join(f'{value:.3f}' for value in sorted(set(sampled_outer_levels))))
for key in sorted(checks):
    print(f'{key}: {"PASS" if checks[key] else "FAIL"}')

passed = bool(checks) and all(checks.values())
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if passed else 1)
