#!/usr/bin/env python3
"""Context speed policy retained after removal of the scalar DWAL-radius policy."""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from crowd_aware_interfaces.msg import ContextState


PROFILES = (
    'OPEN_AREA', 'NARROW_CORRIDOR', 'DOORWAY', 'JUNCTION',
    'DENSE_CROWD', 'OCCLUSION', 'UNKNOWN', 'STALE')


class ContextSpeedLimit(Node):
    def __init__(self):
        super().__init__('context_speed_limit')
        defaults = {
            'context_topic': '/crowd_context',
            'output_topic': '/experiment/speed_limit',
            'semantic_mode': False,
            'context_timeout': 0.5,
            'output_rate': 10.0,
            'max_speed': 0.30,
        }
        for profile in PROFILES:
            defaults[f'profiles.{profile}.max_speed'] = 0.30
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.values = {
            key: self.get_parameter(key).value for key in defaults}
        self.context = None
        self.context_received = None
        self.publisher = self.create_publisher(
            Float64, self.values['output_topic'], 10)
        self.create_subscription(
            ContextState, self.values['context_topic'],
            self.context_callback, 10)
        rate = float(self.values['output_rate'])
        if rate <= 0.0:
            raise ValueError('output_rate must be positive')
        self.create_timer(1.0 / rate, self.tick)

    def context_callback(self, msg):
        self.context = msg
        self.context_received = self.get_clock().now()

    def tick(self):
        limit = float(self.values['max_speed'])
        if self.values['semantic_mode']:
            now = self.get_clock().now()
            age = math.inf if self.context_received is None else (
                now - self.context_received).nanoseconds / 1e9
            valid = (
                self.context is not None and self.context.valid and
                0.0 <= age <= float(self.values['context_timeout']) and
                self.context.dominant_profile in PROFILES)
            if not valid:
                limit = 0.0
            else:
                key = f'profiles.{self.context.dominant_profile}.max_speed'
                limit = min(limit, float(self.values[key]))
        self.publisher.publish(Float64(data=max(0.0, limit)))


def main():
    rclpy.init()
    node = ContextSpeedLimit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
