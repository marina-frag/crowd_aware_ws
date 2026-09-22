#!/usr/bin/env python3
"""Submit the same odom-frame local path to any Nav2 Controller Server."""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
from lifecycle_msgs.srv import GetState
from hunav_msgs.srv import StartEvaluation
from std_msgs.msg import String
from std_srvs.srv import Empty


class LocalTask(Node):
    def __init__(self):
        super().__init__('local_task')
        self.declare_parameter('action_name', '/follow_path')
        self.declare_parameter('path_topic', '/experiment/path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_xy', [0.0, 0.0, 1.0, 0.0])
        self.declare_parameter('timeout', 90.0)
        self.declare_parameter('controller_id', 'FollowPath')
        self.declare_parameter('goal_checker_id', 'general_goal_checker')
        self.declare_parameter('controller_lifecycle_node', '/controller_server')
        self.declare_parameter('autostart', True)
        self.declare_parameter('evaluator_enabled', True)
        self.declare_parameter('experiment_tag', 'experiment')
        self.declare_parameter('run_id', 0)
        xy = [float(v) for v in self.get_parameter('path_xy').value]
        if len(xy) < 4 or len(xy) % 2:
            raise ValueError('path_xy must contain at least two x,y pairs')
        self.timeout = float(self.get_parameter('timeout').value)
        if not math.isfinite(self.timeout) or self.timeout <= 0.0:
            raise ValueError('timeout must be positive and finite')
        self.points = list(zip(xy[0::2], xy[1::2]))
        self.client = ActionClient(self, FollowPath, self.get_parameter('action_name').value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, self.get_parameter('path_topic').value, qos)
        self.result_pub = self.create_publisher(String, '/experiment/task_result', qos)
        lifecycle_service = self.get_parameter('controller_lifecycle_node').value + '/get_state'
        self.state_client = self.create_client(GetState, lifecycle_service)
        self.state_future = None
        self.evaluation_start_client = self.create_client(
            StartEvaluation, '/hunav_start_recording')
        self.evaluation_stop_client = self.create_client(
            Empty, '/hunav_stop_recording')
        self.evaluation_started = False
        self.evaluation_start_future = None
        self.evaluation_stop_future = None
        self.controller_active = False
        self.odom = None
        self.sent = False
        self.finished = False
        self.goal_handle = None
        self.started_at = None
        self.result_pub.publish(String(data='STARTING waiting for active controller and odometry'))
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 10)
        self.create_timer(0.5, self.tick)

    def odom_cb(self, msg): self.odom = msg

    def make_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'odom'
        for index, (x, y) in enumerate(self.points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(self.points):
                nx, ny = self.points[index + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                px, py = self.points[index - 1]
                yaw = math.atan2(y - py, x - px)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def tick(self):
        path = self.make_path()
        self.path_pub.publish(path)
        if (self.sent and not self.finished and self.started_at is not None and
                (self.get_clock().now() - self.started_at).nanoseconds/1e9 >= self.timeout):
            self.finished = True
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.result_pub.publish(String(data=f'FAILED timeout={self.timeout:.3f}s'))
            self.stop_evaluation()
            return
        if self.sent or not self.get_parameter('autostart').value or self.odom is None:
            return
        if not self.controller_active:
            if self.state_future is None and self.state_client.service_is_ready():
                self.state_future = self.state_client.call_async(GetState.Request())
                self.state_future.add_done_callback(self.state_response)
            self.get_logger().info('Waiting for active Controller Server', throttle_duration_sec=2.0)
            return
        if not self.client.server_is_ready():
            self.get_logger().info('Waiting for FollowPath action server', throttle_duration_sec=2.0)
            return
        if (self.get_parameter('evaluator_enabled').value and
                not self.evaluation_started):
            if self.evaluation_start_future is None:
                if not self.evaluation_start_client.service_is_ready():
                    self.get_logger().info('Waiting for HuNav evaluator service', throttle_duration_sec=2.0)
                    return
                request = StartEvaluation.Request()
                request.robot_goal = path.poses[-1]
                request.experiment_tag = self.get_parameter('experiment_tag').value
                request.run_id = self.get_parameter('run_id').value
                self.evaluation_start_future = self.evaluation_start_client.call_async(request)
                self.evaluation_start_future.add_done_callback(self.evaluation_start_response)
            return
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.get_parameter('controller_id').value
        goal.goal_checker_id = self.get_parameter('goal_checker_id').value
        if hasattr(goal, 'progress_checker_id'):
            goal.progress_checker_id = 'progress_checker'
        self.sent = True
        self.started_at = self.get_clock().now()
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        future.add_done_callback(self.goal_response)

    def evaluation_start_response(self, future):
        try:
            response = future.result()
            if not response.success:
                self.result_pub.publish(String(data='FAILED evaluator refused recording'))
                self.sent = True
                return
            self.evaluation_started = True
            self.get_logger().info('HuNav evaluator recording started')
        except Exception as exc:
            self.get_logger().warn(f'Evaluator start failed: {exc}')
        finally:
            self.evaluation_start_future = None


    def state_response(self, future):
        try:
            state = future.result().current_state
            self.controller_active = state.label == 'active'
        except Exception as exc:
            self.get_logger().warn(f'Controller lifecycle query failed: {exc}')
            self.controller_active = False
        finally:
            self.state_future = None

    def stop_evaluation(self):
        if not self.evaluation_started or self.evaluation_stop_future is not None:
            return
        if not self.evaluation_stop_client.service_is_ready():
            self.get_logger().warn('HuNav evaluator stop service is unavailable')
            return
        self.evaluation_stop_future = self.evaluation_stop_client.call_async(Empty.Request())
        self.evaluation_stop_future.add_done_callback(self.evaluation_stop_response)

    def evaluation_stop_response(self, future):
        try:
            future.result()
            self.get_logger().info('HuNav evaluator metrics stored')
        except Exception as exc:
            self.get_logger().error(f'Evaluator stop failed: {exc}')
        finally:
            self.evaluation_started = False
            self.evaluation_stop_future = None


    def feedback(self, feedback):
        distance = getattr(feedback.feedback, 'distance_to_goal', math.nan)
        self.result_pub.publish(String(data=f'RUNNING distance_to_goal={distance:.3f}'))

    def goal_response(self, future):
        handle = future.result()
        self.goal_handle = handle
        if self.finished:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.result_pub.publish(String(data='FAILED goal rejected'))
            self.finished = True
            self.stop_evaluation()
            return
        result = handle.get_result_async()
        result.add_done_callback(self.done)

    def done(self, future):
        if self.finished:
            return
        wrapped = future.result()
        self.finished = True
        self.result_pub.publish(String(data=f'FINISHED action_status={wrapped.status}'))
        self.stop_evaluation()


def main():
    rclpy.init()
    node = LocalTask()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
