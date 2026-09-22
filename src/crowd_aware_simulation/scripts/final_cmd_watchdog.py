#!/usr/bin/env python3
"""Single publisher to Gazebo; emits zero when collision-monitor output stops."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class FinalCommandWatchdog(Node):
    def __init__(self):
        super().__init__('final_cmd_watchdog')
        self.declare_parameter('input_topic', '/cmd_vel_safe')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('input_timeout', 0.3)
        self.declare_parameter('output_rate', 20.0)
        self.command = Twist()
        self.received = None
        self.publisher = self.create_publisher(Twist, self.get_parameter('output_topic').value, 10)
        self.create_subscription(Twist, self.get_parameter('input_topic').value, self.callback, 10)
        self.create_timer(1.0 / float(self.get_parameter('output_rate').value), self.tick)

    def callback(self, msg):
        self.command, self.received = msg, self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        stale = self.received is None or (now - self.received).nanoseconds / 1e9 > float(self.get_parameter('input_timeout').value)
        finite = all(math.isfinite(v) for v in (self.command.linear.x, self.command.angular.z))
        self.publisher.publish(self.command if not stale and finite else Twist())


def main():
    rclpy.init()
    node = FinalCommandWatchdog()
    try: rclpy.spin(node)
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
