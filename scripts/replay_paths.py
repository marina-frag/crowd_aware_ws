#!/usr/bin/env python3
"""Replay helper: desired reference path and executed trail for bags recorded before
/experiment/path and /experiment/actual_path were recorded.

  python3 replay_paths.py <run directory>

Publishes (latched) /experiment/path from evaluation.json reference_path_xy or
run/task.yaml path_xy, and /experiment/actual_path accumulated from /odom.
Does nothing for a topic the bag already contains.
"""
import json
import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def desired(run):
    cfg = json.loads((run/'evaluation.json').read_text())
    if cfg.get('reference_path_xy'):
        return cfg['reference_path_xy']
    try:
        flat = next(iter(yaml.safe_load((run/'run'/'task.yaml').read_text()).values()))['ros__parameters']['path_xy']
        return [flat[i:i+2] for i in range(0, len(flat), 2)]
    except (OSError, KeyError, StopIteration, TypeError):
        return None


class ReplayPaths(Node):
    def __init__(self, run):
        super().__init__('replay_paths', parameter_overrides=[])
        recorded = json.loads((run/'recording_topics.json').read_text()) if (run/'recording_topics.json').exists() else []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.frame = json.loads((run/'evaluation.json').read_text()).get('global_frame', 'odom')
        if '/experiment/path' not in recorded:
            points = desired(run)
            if points:
                self.desired_pub = self.create_publisher(PathMsg, '/experiment/path', latched)
                msg = PathMsg(); msg.header.frame_id = self.frame
                for x, y in points:
                    pose = PoseStamped(); pose.header.frame_id = self.frame
                    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
                    pose.pose.orientation.w = 1.0
                    msg.poses.append(pose)
                self.desired_pub.publish(msg)
                self.get_logger().info(f'desired path: {len(points)} points')
        if '/experiment/actual_path' not in recorded:
            self.trail = PathMsg(); self.trail.header.frame_id = self.frame
            self.trail_pub = self.create_publisher(PathMsg, '/experiment/actual_path', latched)
            self.create_subscription(Odometry, '/odom', self.odom, 20)

    def odom(self, msg):
        p = msg.pose.pose.position
        if self.trail.poses:
            last = self.trail.poses[-1].pose.position
            if (p.x - last.x) ** 2 + (p.y - last.y) ** 2 < 0.05 ** 2:
                return
        pose = PoseStamped(); pose.header = msg.header; pose.pose = msg.pose.pose
        self.trail.poses.append(pose); self.trail.header.stamp = msg.header.stamp
        self.trail_pub.publish(self.trail)


def main():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = ReplayPaths(Path(sys.argv[1]))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
