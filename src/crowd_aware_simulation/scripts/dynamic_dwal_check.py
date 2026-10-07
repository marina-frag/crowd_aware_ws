#!/usr/bin/env python3
"""Dense-crowd runtime acceptance gate for dynamic_dwal OFF/ON."""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from crowd_aware_interfaces.msg import DynamicDwalState


parser = argparse.ArgumentParser()
parser.add_argument('--semantics', choices=('off', 'on'), required=True)
parser.add_argument('--scenario', default='dense_crowd')
parser.add_argument('--timeout', type=float, default=32.0)
args = parser.parse_args()
rclpy.init()
node = Node('dynamic_dwal_dense_check')
request_pub = node.create_publisher(Twist, '/reference_cmd', 10)
states, positions, selected_commands, final_commands, markers = [], [], [], [], []


def clusters_are_contiguous(msg):
    runs, current = [], []
    for path in msg.paths:
        if path.admissible:
            if current and path.index != current[-1] + 1:
                runs.append(current)
                current = []
            current.append(path.index)
        elif current:
            runs.append(current)
            current = []
        if not path.admissible and path.cluster_id != -1:
            return False
    if current:
        runs.append(current)
    groups = [list(cluster.path_indices) for cluster in msg.clusters]
    return (
        runs == groups and
        all(msg.paths[index].cluster_id == cluster_id
            for cluster_id, run in enumerate(runs) for index in run))


node.create_subscription(
    DynamicDwalState, '/dynamic_dwal/diagnostics',
    lambda msg: states.append(msg), 20)
node.create_subscription(
    Odometry, '/odom',
    lambda msg: positions.append(
        (msg.pose.pose.position.x, msg.pose.pose.position.y,
         msg.twist.twist.linear.x)), 20)
node.create_subscription(
    Twist, '/cmd_vel_selected',
    lambda msg: selected_commands.append(
        (time.monotonic(), msg.linear.x, msg.angular.z)), 20)
node.create_subscription(
    Twist, '/cmd_vel',
    lambda msg: final_commands.append(
        (time.monotonic(), msg.linear.x, msg.angular.z)), 20)
node.create_subscription(
    MarkerArray, '/dynamic_dwal/markers',
    lambda msg: markers.append(msg), 20)

start = time.monotonic()
end = start + max(27.0, args.timeout)
while time.monotonic() < end:
    elapsed = time.monotonic() - start
    command = Twist()
    publish = True
    if elapsed < 8.0:
        command.linear.x = 0.12
    elif elapsed < 14.0:
        command.linear.x = 0.12
        command.angular.z = 0.15
    elif elapsed < 17.0:
        pass
    elif elapsed < 22.0:
        command.angular.z = 0.25
    elif elapsed < 24.0:
        command.linear.x = 0.12
    else:
        publish = False
    if publish:
        request_pub.publish(command)
    rclpy.spin_once(node, timeout_sec=0.05)

valid_paths = [msg for msg in states if msg.data_valid and msg.paths]
translation_outputs = [
    msg for msg in states
    if msg.data_valid and msg.selected_path_index >= 0 and
    msg.selected_linear > 1.0e-4]
checks = {}
checks['fresh_translation_inputs'] = any(
    msg.odom_fresh and msg.costmap_fresh and msg.scan_fresh and
    msg.reference_fresh and
    (args.semantics == 'off' or msg.tracks_fresh)
    for msg in valid_paths)
checks['many_left_straight_right_samples'] = any(
    msg.sampled_curvature_count >= 81 and
    msg.sampled_curvature_min <= -3.99 and
    msg.sampled_curvature_max >= 3.99 and
    any(path.curvature < -1.0e-6 for path in msg.paths) and
    any(abs(path.curvature) <= 1.0e-9 for path in msg.paths) and
    any(path.curvature > 1.0e-6 for path in msg.paths)
    for msg in valid_paths)
checks['symmetric_full_configured_family'] = any(
    len(msg.paths) >= 3 and
    all(abs(msg.paths[index].curvature +
            msg.paths[-1-index].curvature) <= 1.0e-9
        for index in range(len(msg.paths)))
    for msg in valid_paths)
checks['outside_dynamic_window_never_admissible'] = all(
    (path.test_angular_speed >= msg.dynamic_w_min - 1.0e-9 and
     path.test_angular_speed <= msg.dynamic_w_max + 1.0e-9) or
    (not path.admissible and path.cluster_id == -1)
    for msg in valid_paths for path in msg.paths)
checks['certified_length_is_conservative'] = all(
    path.arc_lengths and
    abs(path.arc_lengths[-1] - path.free_length) <= 1.0e-9 and
    (not math.isfinite(path.first_failure_length) or
     path.free_length <= path.first_failure_length + 1.0e-12)
    for msg in valid_paths for path in msg.paths)
checks['failure_causes_are_separated'] = all(
    path.termination_reason in {
        'costmap_boundary', 'unknown_costmap_cell',
        'lethal_footprint_collision', 'laser_outside_fov',
        'invalid_laser_ray', 'insufficient_laser_clearance',
        'maximum_search_length'}
    for msg in valid_paths for path in msg.paths)
checks['exact_certified_arc_line_transition'] = any(
    abs(path.curvature) > 1.0e-12 and
    (math.pi / (2.0 * abs(path.curvature))) <= path.free_length + 1.0e-9 and
    any(abs(s - math.pi / (2.0 * abs(path.curvature))) <= 1.0e-9
        for s in path.arc_lengths)
    for msg in valid_paths for path in msg.paths)
checks['visible_arc_line_family'] = any(
    sum(len(path.poses) >= 3 for path in msg.paths) >= 3 and
    any(path.poses and abs(path.poses[-1].y) > 0.02
        for path in msg.paths)
    for msg in valid_paths)
checks['variable_per_path_lengths'] = any(
    len({round(path.free_length, 3) for path in msg.paths}) > 1
    for msg in valid_paths)
checks['contiguous_real_clusters'] = any(
    msg.clusters and clusters_are_contiguous(msg) for msg in valid_paths)
checks['selected_is_actual_cluster_member'] = any(
    msg.selected_cluster_id >= 0 and
    msg.selected_path_index in
    list(msg.clusters[msg.selected_cluster_id].path_indices)
    for msg in translation_outputs)
checks['final_motion_rechecked'] = bool(translation_outputs) and all(
    msg.final_motion_admissible for msg in translation_outputs)
checks['standstill_start'] = any(
    abs(msg.measured_linear) < 0.01 and msg.selected_linear > 0.0
    for msg in translation_outputs)
checks['zero_command_handled'] = any(
    msg.data_valid and msg.state_reason == 'teleop_zero' and
    abs(msg.selected_linear) < 1.0e-9 and
    abs(msg.selected_angular) < 1.0e-9
    for msg in states)
checks['rotation_handled_explicitly'] = any(
    msg.data_valid and msg.in_place_rotation and
    ('rotation' in msg.controller_reason)
    for msg in states)
checks['rotation_and_stop_have_swept_poses'] = any(
    msg.data_valid and msg.in_place_rotation and len(msg.braking_poses) >= 2
    for msg in states) and any(
    msg.data_valid and msg.state_reason == 'teleop_zero' and msg.braking_poses
    for msg in states)
checks['stale_reference_stops'] = any(
    not msg.data_valid and msg.state_reason == 'missing_or_stale_reference' and
    abs(msg.selected_linear) < 1.0e-9 and
    abs(msg.selected_angular) < 1.0e-9
    for msg in states)

line_families = []
selected_markers = False
transition_markers = False
rotation_sweep_markers = False
braking_sweep_markers = False
for marker_array in markers:
    path_lines = [
        marker for marker in marker_array.markers
        if marker.type == Marker.LINE_STRIP and
        marker.ns == 'dynamic_dwal_paths']
    if path_lines:
        line_families.append(path_lines)
    selected_markers |= any(
        marker.ns == 'dynamic_dwal_selected' and
        marker.type == Marker.LINE_STRIP and len(marker.points) >= 2
        for marker in marker_array.markers)
    transition_markers |= any(
        marker.ns == 'dynamic_dwal_arc_line_transition'
        for marker in marker_array.markers)
    rotation_sweep_markers |= any(
        marker.ns == 'dynamic_dwal_rotation_swept_footprint'
        for marker in marker_array.markers)
    braking_sweep_markers |= any(
        marker.ns == 'dynamic_dwal_braking_swept_footprint'
        for marker in marker_array.markers)
checks['rviz_all_members_and_selected'] = (
    any(len(lines) >= 81 and
        all(abs(line.scale.x - 0.005) <= 1.0e-6 for line in lines) and
        sum(len(line.points) >= 3 for line in lines) >= 3
        for lines in line_families) and selected_markers)
checks['rviz_transition_and_swept_footprints'] = (
    transition_markers and rotation_sweep_markers and braking_sweep_markers)

checks['selected_command_nonzero'] = any(
    abs(v) + abs(w) > 1.0e-4 for _, v, w in selected_commands)
checks['final_command_nonzero'] = any(
    abs(v) + abs(w) > 1.0e-4 for _, v, w in final_commands)
if positions:
    x0, y0, _ = positions[0]
    checks['robot_moved_through_shared_control'] = max(
        math.hypot(x - x0, y - y0) for x, y, _ in positions) > 0.02
else:
    checks['robot_moved_through_shared_control'] = False

node_names = set(node.get_node_names())
selected_publishers = node.get_publishers_info_by_topic('/cmd_vel_selected')
final_publishers = node.get_publishers_info_by_topic('/cmd_vel')
track_publishers = node.get_publishers_info_by_topic('/tracked_agents')
track_subscribers = node.get_subscriptions_info_by_topic('/tracked_agents')
checks['dynamic_is_only_selector'] = (
    len(selected_publishers) == 1 and
    selected_publishers[0].node_name == 'dynamic_dwal' and
    'shared_controller' not in node_names and
    'dwal_generator' not in node_names and
    'dwal_clustering' not in node_names)
checks['single_final_publisher'] = (
    len(final_publishers) == 1 and
    final_publishers[0].node_name == 'final_cmd_watchdog')
if args.semantics == 'off':
    checks['off_uses_no_structured_tracks'] = (
        not track_publishers and not any(
            endpoint.node_name == 'dynamic_dwal'
            for endpoint in track_subscribers))
    checks['prediction_mode_behavior'] = all(
        path.prediction_admissible
        for msg in valid_paths for path in msg.paths)
else:
    checks['on_structured_tracks_connected'] = (
        bool(track_publishers) and any(
            endpoint.node_name == 'dynamic_dwal'
            for endpoint in track_subscribers))
    checks['prediction_mode_behavior'] = bool(valid_paths) and all(
        (not path.admissible or path.prediction_admissible)
        for msg in valid_paths for path in msg.paths)

evidence = max(
    valid_paths,
    key=lambda msg: (
        msg.admissible_count,
        sum(len(path.poses) for path in msg.paths)),
    default=None)
print(f'condition: dynamic_dwal/{args.semantics}/{args.scenario}')
print(f'diagnostic_cycles: {len(states)}, valid_path_cycles: {len(valid_paths)}')
if evidence is not None:
    print(
        f'evidence_cycle={evidence.cycle} test_speed={evidence.test_linear:.3f} '
        f'samples={evidence.sampled_curvature_count} '
        f'k=[{evidence.sampled_curvature_min:.3f},'
        f'{evidence.sampled_curvature_max:.3f}] '
        f'admissible={evidence.admissible_count} '
        f'clusters={[list(c.path_indices) for c in evidence.clusters]} '
        f'selected={evidence.selected_path_index}')
    for path in evidence.paths:
        print(
            f'path[{path.index}] k={path.curvature:.3f} '
            f'points={len(path.poses)} L={path.free_length:.3f} '
            f'fail_s={path.first_failure_length:.3f} '
            f'term={path.termination_reason} detail={path.failure_detail!r} '
            f'cell=({path.costmap_cell_x},{path.costmap_cell_y},'
            f'{path.costmap_cell_value}) ray=({path.laser_ray_index},'
            f'{path.laser_ray_angle:.3f},{path.laser_ray_range:.3f},'
            f'{path.laser_required_range:.3f}) reject={path.rejection_reason} '
            f'admissible={path.admissible} cluster={path.cluster_id}')
for key in sorted(checks):
    print(f'{key}: {"PASS" if checks[key] else "FAIL"}')
passed = bool(checks) and all(checks.values())
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if passed else 1)
