#!/usr/bin/env python3
"""Command heartbeat, stale-input stop, experiment limit, and forward-only guard."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')
        defaults = {
            'input_topic': '/cmd_vel_selected', 'output_topic': '/cmd_vel_guarded',
            'speed_limit_topic': '/experiment/speed_limit',
            'status_topic': '/experiment/command_guard_status',
            'input_timeout': 0.35, 'speed_limit_timeout': 0.5,
            'output_rate': 20.0, 'max_linear': 0.3, 'max_angular': 0.8,
            'allow_reverse': False,
            'reference_gate_enabled': False,
            'reference_gate_topic': '/experiment/teleop_reference_active',
            'reference_gate_timeout': 0.35,
        }
        for key, value in defaults.items(): self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.command = None
        self.command_rx = None
        self.limit = 0.0
        self.limit_rx = None
        self.reference_gate = False
        self.reference_gate_rx = None
        self.publisher = self.create_publisher(Twist, self.p['output_topic'], 10)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 10)
        self.create_subscription(Twist, self.p['input_topic'], self.command_cb, 10)
        self.create_subscription(Float64, self.p['speed_limit_topic'], self.limit_cb, 10)
        if self.p['reference_gate_enabled']:
            self.create_subscription(
                Bool, self.p['reference_gate_topic'], self.reference_gate_cb, 10)
        self.create_timer(1.0 / float(self.p['output_rate']), self.tick)

    def command_cb(self, msg):
        self.command, self.command_rx = msg, self.get_clock().now()

    def limit_cb(self, msg):
        self.limit, self.limit_rx = max(0.0, float(msg.data)), self.get_clock().now()

    def reference_gate_cb(self, msg):
        self.reference_gate = bool(msg.data)
        self.reference_gate_rx = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        reason = 'forwarded'
        out = Twist()
        command_age = math.inf if self.command_rx is None else (now - self.command_rx).nanoseconds / 1e9
        limit_age = math.inf if self.limit_rx is None else (now - self.limit_rx).nanoseconds / 1e9
        gate_age = (math.inf if self.reference_gate_rx is None else
                    (now - self.reference_gate_rx).nanoseconds / 1e9)
        if (self.p['reference_gate_enabled'] and
                (gate_age > float(self.p['reference_gate_timeout']) or
                 not self.reference_gate)):
            reason = 'STOP inactive teleop reference'
        elif command_age > float(self.p['input_timeout']):
            reason = 'STOP stale selected command'
        elif limit_age > float(self.p['speed_limit_timeout']):
            reason = 'STOP stale speed limit'
        elif self.command.linear.x < 0.0 and not self.p['allow_reverse']:
            reason = 'STOP reverse motion outside experiment scope'
        elif not all(math.isfinite(v) for v in (self.command.linear.x, self.command.angular.z, self.limit)):
            reason = 'STOP non-finite command'
        else:
            out.linear.x = max(0.0, min(float(self.command.linear.x), self.limit,
                                        float(self.p['max_linear'])))
            out.angular.z = max(-float(self.p['max_angular']),
                                min(float(self.command.angular.z), float(self.p['max_angular'])))
            if self.limit <= 0.0:
                out = Twist()
                reason = 'STOP zero admissible speed'
        self.publisher.publish(out)
        self.status_pub.publish(String(data=reason))


def main():
    rclpy.init()
    node = CommandGuard()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
