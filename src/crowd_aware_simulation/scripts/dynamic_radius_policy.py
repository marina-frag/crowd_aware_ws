#!/usr/bin/env python3
"""Continuous/discrete DWAL horizon policy and common context speed policy."""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from crowd_aware_interfaces.msg import ContextState, DynamicRadiusState

from policy_core import horizon_policy, snap_radius


class DynamicRadiusPolicy(Node):
    def __init__(self):
        super().__init__('dynamic_radius_policy')
        defaults = {
            'odom_topic': '/odom', 'context_topic': '/crowd_context',
            'radius_topic': '/dwal/dynamic_radius',
            'speed_limit_topic': '/experiment/speed_limit',
            'state_topic': '/experiment/dynamic_radius_state',
            'semantic_mode': False, 'adaptation_mode': 'fixed', 'output_rate': 10.0,
            'odom_timeout': 0.5, 'context_timeout': 0.5,
            'fixed_radius': 1.8, 'radius_min': 1.2,
            'sensor_reliable_radius': 8.0, 'costmap_reliable_radius': 4.0,
            'front_extent': 0.865, 'brake_deceleration': 0.5,
            'base_margin': 0.2, 'base_reaction_time': 0.35,
            'base_preview_time': 2.5, 'max_speed': 0.3,
            'radius_decrease_rate': 0.5, 'minimum_radius_delta': 0.03,
            'discrete_levels': [1.2, 1.8, 2.6, 4.0],
        }
        for profile in ('OPEN_AREA', 'NARROW_CORRIDOR', 'DOORWAY', 'JUNCTION',
                        'DENSE_CROWD', 'OCCLUSION', 'UNKNOWN', 'STALE'):
            defaults[f'profiles.{profile}.margin'] = 0.2
            defaults[f'profiles.{profile}.reaction_time'] = 0.35
            defaults[f'profiles.{profile}.preview_time'] = 2.5
            defaults[f'profiles.{profile}.max_speed'] = 0.3
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if self.p['adaptation_mode'] not in ('fixed', 'discrete', 'continuous'):
            raise ValueError('adaptation_mode must be fixed, discrete, or continuous')
        self.odom = self.context_state = None
        self.odom_rx = self.context_rx = None
        self.applied = float(self.p['fixed_radius'])
        self.last_tick = self.get_clock().now()
        self.radius_pub = self.create_publisher(Float64, self.p['radius_topic'], 10)
        self.limit_pub = self.create_publisher(Float64, self.p['speed_limit_topic'], 10)
        self.state_pub = self.create_publisher(DynamicRadiusState, self.p['state_topic'], 10)
        self.create_subscription(Odometry, self.p['odom_topic'], self.odom_cb, 10)
        self.create_subscription(ContextState, self.p['context_topic'], self.context_cb, 10)
        self.create_timer(1.0 / float(self.p['output_rate']), self.tick)

    def odom_cb(self, msg):
        self.odom, self.odom_rx = msg, self.get_clock().now()

    def context_cb(self, msg):
        self.context_state, self.context_rx = msg, self.get_clock().now()

    def profile_values(self, profile):
        if not self.p['semantic_mode']:
            return (float(self.p['base_margin']), float(self.p['base_reaction_time']),
                    float(self.p['base_preview_time']), float(self.p['max_speed']))
        return tuple(float(self.p[f'profiles.{profile}.{field}'])
                     for field in ('margin', 'reaction_time', 'preview_time', 'max_speed'))

    def stop_state(self, now, reason, profile='STALE'):
        state = DynamicRadiusState()
        state.header.stamp = now.to_msg()
        state.header.frame_id = 'odom'
        state.valid = False
        state.applied_radius = float(self.applied)
        state.speed_limit = 0.0
        state.active_profile = profile
        state.adaptation_mode = self.p['adaptation_mode']
        state.reason = reason
        self.limit_pub.publish(Float64(data=0.0))
        self.state_pub.publish(state)

    def tick(self):
        now = self.get_clock().now()
        if now < self.last_tick:
            self.odom_rx = self.context_rx = None
        dt = max(0.0, (now - self.last_tick).nanoseconds / 1e9)
        self.last_tick = now
        if self.odom_rx is None or (now - self.odom_rx).nanoseconds / 1e9 > float(self.p['odom_timeout']):
            self.stop_state(now, 'missing or stale odometry')
            return
        profile = 'OPEN_AREA'
        if self.p['semantic_mode']:
            if (self.context_rx is None or
                    (now - self.context_rx).nanoseconds / 1e9 > float(self.p['context_timeout']) or
                    not self.context_state.valid):
                self.stop_state(now, 'semantic mode requires valid fresh context')
                return
            profile = self.context_state.dominant_profile
        margin, reaction, preview, profile_speed = self.profile_values(profile)
        speed = max(0.0, float(self.odom.twist.twist.linear.x))
        mode = self.p['adaptation_mode']
        if mode == 'fixed':
            result = horizon_policy(speed, float(self.p['front_extent']), margin, reaction,
                                    float(self.p['brake_deceleration']), preview,
                                    float(self.p['radius_min']),
                                    float(self.p['sensor_reliable_radius']),
                                    float(self.p['costmap_reliable_radius']), profile_speed)
            requested = applied = float(self.p['fixed_radius'])
            speed_limit = min(profile_speed, result.speed_limit) if result.valid else 0.0
            reason = 'fixed DWAL radius; ' + result.reason
        else:
            result = horizon_policy(speed, float(self.p['front_extent']), margin, reaction,
                                    float(self.p['brake_deceleration']), preview,
                                    float(self.p['radius_min']),
                                    float(self.p['sensor_reliable_radius']),
                                    float(self.p['costmap_reliable_radius']), profile_speed)
            if not result.valid:
                self.stop_state(now, result.reason, profile)
                return
            requested = result.requested
            target = (snap_radius(requested, self.p['discrete_levels'])
                      if mode == 'discrete' else result.applied)
            # Safety-driven increases are immediate; decreases are rate-limited.
            if target >= self.applied:
                applied = target
            else:
                applied = max(target, self.applied - float(self.p['radius_decrease_rate']) * dt)
            if abs(applied - self.applied) < float(self.p['minimum_radius_delta']):
                applied = self.applied
            self.applied = applied
            speed_limit = min(profile_speed, result.speed_limit)
            reason = result.reason
            self.radius_pub.publish(Float64(data=float(applied)))

        self.limit_pub.publish(Float64(data=float(speed_limit)))
        state = DynamicRadiusState()
        state.header.stamp = now.to_msg()
        state.header.frame_id = 'odom'
        state.valid = bool(result.valid)
        state.speed = float(speed)
        state.requested_radius = float(requested)
        state.applied_radius = float(applied)
        state.radius_upper_bound = float(result.upper)
        state.speed_limit = float(speed_limit)
        state.active_profile = profile
        state.adaptation_mode = mode
        state.reason = reason
        self.state_pub.publish(state)


def main():
    rclpy.init()
    node = DynamicRadiusPolicy()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
