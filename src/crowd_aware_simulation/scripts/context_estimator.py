#!/usr/bin/env python3
"""Explainable local-geometry/crowd context estimator for HuNav ground truth."""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String
from cohan_msgs.msg import TrackedAgents, TrackedSegmentType
from crowd_aware_interfaces.msg import ContextState

from policy_core import CONTEXT_LABELS, choose_profile


def clamp01(value):
    return max(0.0, min(1.0, float(value)))


class ContextEstimator(Node):
    def __init__(self):
        super().__init__('context_estimator')
        defaults = {
            'tracks_topic': '/tracked_agents_logging',
            'costmap_topic': '/local_costmap/costmap',
            'odom_topic': '/odom',
            'output_topic': '/crowd_context',
            'profile_topic': '/crowd_context/profile',
            'output_rate': 8.0,
            'track_stale_after': 0.5,
            'costmap_stale_after': 0.5,
            'odom_stale_after': 0.5,
            'minimum_dwell': 1.5,
            'switch_margin': 0.12,
            'crowd_radius': 3.0,
            'dense_count': 4,
            'narrow_width': 1.5,
            'doorway_width': 1.15,
            'open_clearance': 2.25,
            'junction_clearance': 1.8,
            'lethal_cost': 90,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.values = {key: self.get_parameter(key).value for key in defaults}
        self.tracks = self.costmap = self.odom = None
        self.received = {}
        self.current_profile = 'UNKNOWN'
        self.transition_time = self.get_clock().now()
        self.previous_now = self.transition_time
        self.pub = self.create_publisher(ContextState, self.values['output_topic'], 10)
        self.profile_pub = self.create_publisher(String, self.values['profile_topic'], 10)
        self.create_subscription(TrackedAgents, self.values['tracks_topic'], self._tracks, 10)
        self.create_subscription(OccupancyGrid, self.values['costmap_topic'], self._costmap, 10)
        self.create_subscription(Odometry, self.values['odom_topic'], self._odom, 10)
        self.create_timer(1.0 / float(self.values['output_rate']), self.tick)

    def _remember(self, key, msg):
        setattr(self, key, msg)
        self.received[key] = self.get_clock().now()

    def _tracks(self, msg): self._remember('tracks', msg)
    def _costmap(self, msg): self._remember('costmap', msg)
    def _odom(self, msg): self._remember('odom', msg)

    def _ray_clearance(self, angle, maximum=4.0):
        grid = self.costmap
        info = grid.info
        x0 = self.odom.pose.pose.position.x
        y0 = self.odom.pose.pose.position.y
        step = max(float(info.resolution), 0.05)
        distance = 0.0
        while distance <= maximum:
            x = x0 + distance * math.cos(angle)
            y = y0 + distance * math.sin(angle)
            mx = int((x - info.origin.position.x) / info.resolution)
            my = int((y - info.origin.position.y) / info.resolution)
            if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
                return distance
            cost = grid.data[my * info.width + mx]
            if cost < 0 or cost >= int(self.values['lethal_cost']):
                return distance
            distance += step
        return maximum

    @staticmethod
    def _yaw(quaternion):
        return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                          1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2))

    def _scores(self):
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        people = []
        for agent in self.tracks.agents:
            for segment in agent.segments:
                if segment.type == TrackedSegmentType.TORSO:
                    p = segment.pose.pose.position
                    people.append((p.x, p.y))
                    break
        nearby = sum(math.hypot(px - x, py - y) <= float(self.values['crowd_radius'])
                     for px, py in people)
        dense = clamp01(nearby / max(1.0, float(self.values['dense_count'])))

        yaw = self._yaw(self.odom.pose.pose.orientation)
        front = self._ray_clearance(yaw)
        back = self._ray_clearance(yaw + math.pi)
        left = self._ray_clearance(yaw + math.pi / 2.0)
        right = self._ray_clearance(yaw - math.pi / 2.0)
        side_width = left + right
        narrow = clamp01((float(self.values['narrow_width']) - side_width) /
                         max(float(self.values['narrow_width']) - 0.65, 0.1))
        fore_aft_open = clamp01((min(front, back) - 0.8) / 1.5)
        doorway_width = float(self.values['doorway_width'])
        doorway = clamp01((doorway_width - side_width) / max(doorway_width - 0.65, 0.1)) * fore_aft_open

        directions = [self._ray_clearance(yaw + i * math.pi / 4.0) for i in range(8)]
        open_branches = sum(d >= float(self.values['junction_clearance']) for d in directions)
        junction = clamp01((open_branches - 2.0) / 3.0) * (1.0 - doorway)
        clearance = min(front, back, left, right)
        open_area = clamp01(clearance / float(self.values['open_clearance'])) * (1.0 - dense)

        # This is an occlusion-risk heuristic, never an assertion that an absent track is hidden.
        blocked_fraction = sum(d < 1.2 for d in directions) / len(directions)
        occlusion = clamp01(blocked_fraction * 1.5) if people else 0.0
        return {
            'OPEN_AREA': open_area,
            'NARROW_CORRIDOR': narrow * fore_aft_open,
            'DOORWAY': doorway,
            'JUNCTION': junction,
            'DENSE_CROWD': dense,
            'OCCLUSION': occlusion,
            'UNKNOWN': 0.0,
            'STALE': 0.0,
        }, nearby

    def tick(self):
        now = self.get_clock().now()
        if now < self.previous_now:  # Gazebo reset/time jump.
            self.received.clear()
            self.current_profile = 'UNKNOWN'
            self.transition_time = now
        self.previous_now = now
        thresholds = {
            'tracks': float(self.values['track_stale_after']),
            'costmap': float(self.values['costmap_stale_after']),
            'odom': float(self.values['odom_stale_after']),
        }
        missing = [key for key in thresholds if key not in self.received]
        ages = {key: (now - stamp).nanoseconds / 1e9 for key, stamp in self.received.items()}
        stale = [key for key, limit in thresholds.items() if ages.get(key, math.inf) > limit]
        msg = ContextState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.input_age = float(max(ages.values(), default=math.inf))

        if missing:
            scores = {label: 0.0 for label in CONTEXT_LABELS}
            scores['UNKNOWN'] = 1.0
            candidate = 'UNKNOWN'
            msg.valid = False
            msg.confidence = 0.0
            msg.reason = 'missing inputs: ' + ','.join(missing)
        elif stale:
            scores = {label: 0.0 for label in CONTEXT_LABELS}
            scores['STALE'] = 1.0
            candidate = 'STALE'
            msg.valid = False
            msg.confidence = 0.0
            msg.reason = 'stale inputs: ' + ','.join(stale)
        elif not self.costmap.data or self.costmap.header.frame_id != 'odom':
            scores = {label: 0.0 for label in CONTEXT_LABELS}
            scores['UNKNOWN'] = 1.0
            candidate = 'UNKNOWN'
            msg.valid = False
            msg.confidence = 0.0
            msg.reason = 'invalid or non-odom costmap'
        else:
            scores, nearby = self._scores()
            candidate = max((label for label in CONTEXT_LABELS[:-2]), key=scores.get)
            msg.valid = True
            freshness = clamp01(1.0 - max(ages.values()) / max(thresholds.values()))
            msg.confidence = float(scores[candidate] * freshness)
            msg.reason = f'heuristic geometry and {nearby} ground-truth tracks nearby'

        elapsed = (now - self.transition_time).nanoseconds / 1e9
        selected = choose_profile(self.current_profile, candidate, scores, elapsed,
                                  float(self.values['minimum_dwell']),
                                  float(self.values['switch_margin']))
        if selected != self.current_profile:
            self.current_profile = selected
            self.transition_time = now
        msg.labels = list(CONTEXT_LABELS)
        msg.scores = [float(scores[label]) for label in CONTEXT_LABELS]
        msg.dominant_profile = self.current_profile
        self.pub.publish(msg)
        self.profile_pub.publish(String(data=self.current_profile))


def main():
    rclpy.init()
    node = ContextEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
