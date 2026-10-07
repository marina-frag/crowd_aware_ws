#!/usr/bin/env python3
"""Publish a constant experiment speed cap without context-dependent policy."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ConstantSpeedLimit(Node):
    def __init__(self):
        super().__init__('constant_speed_limit')
        self.declare_parameter('output_topic', '/experiment/speed_limit')
        self.declare_parameter('speed_limit', 0.30)
        self.declare_parameter('output_rate', 10.0)
        topic = self.get_parameter('output_topic').value
        self.speed_limit = max(0.0, float(self.get_parameter('speed_limit').value))
        rate = float(self.get_parameter('output_rate').value)
        if rate <= 0.0:
            raise ValueError('output_rate must be positive')
        self.publisher = self.create_publisher(Float64, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        self.publisher.publish(Float64(data=self.speed_limit))


def main():
    rclpy.init()
    node = ConstantSpeedLimit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
