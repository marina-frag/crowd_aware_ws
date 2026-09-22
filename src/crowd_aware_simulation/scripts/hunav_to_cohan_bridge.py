#!/usr/bin/env python3
"""HuNav ground-truth adapter derived from pmb2_hunav_simulation@770c5af."""
import math

import rclpy
from rclpy.node import Node
from hunav_msgs.msg import Agents
from cohan_msgs.msg import (
    AgentType, TrackedAgent, TrackedAgents, TrackedSegment, TrackedSegmentType)


class HunavToCohanBridge(Node):
    def __init__(self):
        super().__init__('hunav_to_cohan_bridge')
        self.declare_parameter('input_topic', '/human_states')
        self.declare_parameter('logging_output_topic', '/tracked_agents_logging')
        self.declare_parameter('controller_output_topic', '/tracked_agents')
        self.declare_parameter('publish_controller_output', True)
        self.declare_parameter('moving_speed_threshold', 0.05)
        self.declare_parameter('output_frame', 'odom')
        input_topic = self.get_parameter('input_topic').value
        logging_topic = self.get_parameter('logging_output_topic').value
        controller_topic = self.get_parameter('controller_output_topic').value
        self.threshold = float(self.get_parameter('moving_speed_threshold').value)
        self.output_frame = self.get_parameter('output_frame').value
        self.logging_publisher = self.create_publisher(TrackedAgents, logging_topic, 10)
        self.controller_publisher = None
        if self.get_parameter('publish_controller_output').value:
            self.controller_publisher = self.create_publisher(TrackedAgents, controller_topic, 10)
        self.subscription = self.create_subscription(Agents, input_topic, self.callback, 10)
        self.get_logger().info(
            f'HuNav ground truth: {input_topic} -> {logging_topic}' +
            (f' and {controller_topic}' if self.controller_publisher else '') +
            f' in {self.output_frame}')

    def callback(self, msg):
        out = TrackedAgents()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self.output_frame
        for agent in msg.agents:
            tracked = TrackedAgent()
            tracked.track_id = max(int(agent.id), 0)
            tracked.type = AgentType.ROBOT if agent.type == agent.ROBOT else AgentType.HUMAN
            tracked.name = agent.name
            speed = math.hypot(agent.velocity.linear.x, agent.velocity.linear.y)
            tracked.state = TrackedAgent.MOVING if speed > self.threshold else TrackedAgent.STOPPED
            segment = TrackedSegment()
            segment.type = TrackedSegmentType.TORSO
            segment.pose.pose = agent.position
            segment.twist.twist = agent.velocity
            tracked.segments = [segment]
            out.agents.append(tracked)
        self.logging_publisher.publish(out)
        if self.controller_publisher:
            self.controller_publisher.publish(out)


def main():
    rclpy.init()
    node = HunavToCohanBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
