#!/usr/bin/env python3
"""Supply the CoHAN agent-state side channel when DWB hosts social layers.

HATEB normally publishes ``agent_path_prediction/AgentsInfo`` from its internal
Agents helper. DWB does not instantiate that helper, while the reusable CoHAN
costmap layers require the message to keep tracked-agent data valid. This
adapter derives only the fields those layers need from the canonical HuNav
track stream; it is not an intention predictor.
"""
import math

import rclpy
from agent_path_prediction.msg import AgentsInfo, HumanInfo
from cohan_msgs.msg import AgentType, TrackedAgent, TrackedAgents, TrackedSegmentType
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


HATEB_NO_STATE = 0
HATEB_STATIC = 1
HATEB_MOVING = 2
HATEB_STOPPED = 3
HATEB_BLOCKED = 4


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def hateb_state(state):
    return {
        TrackedAgent.STATIC: HATEB_STATIC,
        TrackedAgent.MOVING: HATEB_MOVING,
        TrackedAgent.STOPPED: HATEB_STOPPED,
        TrackedAgent.BLOCKED: HATEB_BLOCKED,
    }.get(state, HATEB_NO_STATE)


class CohanAgentsInfoAdapter(Node):
    def __init__(self):
        super().__init__('cohan_agents_info_adapter')
        self.declare_parameter('tracks_topic', '/tracked_agents')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('output_topic', '/agents_info')
        self.odom = None
        self.publisher = self.create_publisher(
            AgentsInfo, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(
            TrackedAgents, self.get_parameter('tracks_topic').value,
            self.tracks_callback, qos_profile_sensor_data)

    def odom_callback(self, msg):
        self.odom = msg

    def tracks_callback(self, msg):
        if self.odom is None or msg.header.frame_id != self.odom.header.frame_id:
            return
        output = AgentsInfo()
        robot = self.odom.pose.pose
        quaternion = robot.orientation
        output.robot_pose.x = robot.position.x
        output.robot_pose.y = robot.position.y
        output.robot_pose.theta = yaw_from_quaternion(
            quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        by_distance = []
        for agent in msg.agents:
            if agent.type != AgentType.HUMAN:
                continue
            segment = next((part for part in agent.segments
                            if part.type == TrackedSegmentType.TORSO), None)
            if segment is None:
                continue
            pose = segment.pose.pose
            info = HumanInfo()
            info.id = int(agent.track_id)
            info.name = agent.name
            info.state = hateb_state(agent.state)
            info.pose.x = pose.position.x
            info.pose.y = pose.position.y
            orientation = pose.orientation
            info.pose.theta = yaw_from_quaternion(
                orientation.x, orientation.y,
                orientation.z, orientation.w)
            info.dist = math.hypot(
                pose.position.x - robot.position.x,
                pose.position.y - robot.position.y)
            by_distance.append((info.dist, info))
        for _, info in sorted(by_distance, key=lambda item: item[0]):
            output.humans.append(info)
            output.visible.append(info.id)
            if info.state == HATEB_MOVING:
                output.moving.append(info.id)
            else:
                output.still.append(info.id)
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = CohanAgentsInfoAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
