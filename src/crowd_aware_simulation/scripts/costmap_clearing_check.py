#!/usr/bin/env python3
"""Verify that a previously lethal lidar cell clears after a free ray."""
import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


parser = argparse.ArgumentParser()
parser.add_argument('--timeout', type=float, default=45.0)
args = parser.parse_args()
rclpy.init()
node = Node('costmap_clearing_check')
tf_buffer = Buffer()
tf_listener = TransformListener(tf_buffer, node)
latest_scan = None
marked = {}
map_count = 0
cleared = None


def scan_callback(msg):
    global latest_scan
    latest_scan = msg


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def grid_value(msg, wx, wy):
    origin = msg.info.origin
    yaw = yaw_of(origin.orientation)
    dx, dy = wx - origin.position.x, wy - origin.position.y
    mx = math.cos(yaw) * dx + math.sin(yaw) * dy
    my = -math.sin(yaw) * dx + math.cos(yaw) * dy
    ix = math.floor(mx / msg.info.resolution)
    iy = math.floor(my / msg.info.resolution)
    if ix < 0 or iy < 0 or ix >= msg.info.width or iy >= msg.info.height:
        return None
    return int(msg.data[iy * msg.info.width + ix])


def free_ray_reaches(msg, wx, wy):
    if latest_scan is None or not latest_scan.ranges:
        return False
    try:
        transform = tf_buffer.lookup_transform(
            msg.header.frame_id, latest_scan.header.frame_id, Time(),
            timeout=Duration(seconds=0.05))
    except TransformException:
        return False
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y
    yaw = yaw_of(transform.transform.rotation)
    dx, dy = wx - tx, wy - ty
    distance = math.hypot(dx, dy)
    relative = math.atan2(dy, dx) - yaw
    while relative < latest_scan.angle_min:
        relative += 2.0 * math.pi
    while relative > latest_scan.angle_max:
        relative -= 2.0 * math.pi
    if relative < latest_scan.angle_min or relative > latest_scan.angle_max:
        return False
    index = int(round(
        (relative - latest_scan.angle_min) / latest_scan.angle_increment))
    if index < 0 or index >= len(latest_scan.ranges):
        return False
    measured = latest_scan.ranges[index]
    ray_end = latest_scan.range_max if math.isinf(measured) else measured
    return math.isfinite(ray_end) and (
        ray_end > distance + 2.0 * msg.info.resolution)


def map_callback(msg):
    global map_count, cleared
    map_count += 1
    resolution = msg.info.resolution
    origin = msg.info.origin
    yaw = yaw_of(origin.orientation)
    c, s = math.cos(yaw), math.sin(yaw)
    for iy in range(msg.info.height):
        for ix in range(msg.info.width):
            value = int(msg.data[iy * msg.info.width + ix])
            if value < 100:
                continue
            mx = (ix + 0.5) * resolution
            my = (iy + 0.5) * resolution
            wx = origin.position.x + c * mx - s * my
            wy = origin.position.y + s * mx + c * my
            key = (round(wx / resolution), round(wy / resolution))
            marked.setdefault(key, (wx, wy, map_count))
    for key, (wx, wy, first_map) in list(marked.items()):
        if map_count - first_map < 2:
            continue
        value = grid_value(msg, wx, wy)
        if value is not None and 0 <= value < 100 and free_ray_reaches(
                msg, wx, wy):
            cleared = (wx, wy, value, first_map, map_count)
            return


costmap_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(LaserScan, '/scan', scan_callback, qos_profile_sensor_data)
node.create_subscription(
    OccupancyGrid, '/local_costmap/costmap', map_callback, costmap_qos)

deadline = time.monotonic() + args.timeout
while rclpy.ok() and time.monotonic() < deadline and cleared is None:
    rclpy.spin_once(node, timeout_sec=0.10)

if cleared is None:
    print(
        f'FAIL: no marked-then-free observed cell; maps={map_count}, '
        f'marked_world_cells={len(marked)}')
    result = 1
else:
    wx, wy, value, first_map, last_map = cleared
    print(
        f'PASS: lethal lidar cell ({wx:.3f}, {wy:.3f}) cleared to {value} '
        f'after a free ray; map_updates={first_map}->{last_map}')
    result = 0
node.destroy_node()
rclpy.shutdown()
sys.exit(result)
