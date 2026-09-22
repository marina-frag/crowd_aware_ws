#!/usr/bin/env python3
"""Turn a live differential-drive Twist intention into serialized local paths."""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String


def integrate_twist(x, y, yaw, linear, angular, horizon, sample_period):
    """Return an odom-frame path for a constant differential-drive intention."""
    steps = max(2, int(math.ceil(horizon / sample_period)))
    samples = []
    for index in range(steps + 1):
        elapsed = horizon * index / steps
        heading = yaw + angular * elapsed
        if abs(angular) < 1.0e-6:
            px = x + linear * elapsed * math.cos(yaw)
            py = y + linear * elapsed * math.sin(yaw)
        else:
            radius = linear / angular
            px = x + radius * (math.sin(heading) - math.sin(yaw))
            py = y - radius * (math.cos(heading) - math.cos(yaw))
        samples.append((px, py, heading))
    return samples


class TeleopPathAdapter(Node):
    def __init__(self):
        super().__init__('teleop_path_adapter')
        defaults = {
            'action_name': '/follow_path',
            'reference_topic': '/reference_cmd',
            'odom_topic': '/odom',
            'path_topic': '/experiment/path',
            'gate_topic': '/experiment/teleop_reference_active',
            'status_topic': '/experiment/teleop_reference_status',
            'controller_lifecycle_node': '/controller_server',
            'controller_id': 'FollowPath',
            'goal_checker_id': 'general_goal_checker',
            'global_frame': 'odom',
            'base_frame': 'sim_base',
            'input_timeout': 0.35,
            'odom_timeout': 0.50,
            'controller_state_timeout': 1.50,
            'feedback_timeout': 0.75,
            'update_rate': 10.0,
            'goal_refresh_period': 0.75,
            'projection_horizon': 3.0,
            'sample_period': 0.10,
            'max_linear': 0.30,
            'max_angular': 0.80,
            'command_epsilon': 0.001,
            'allow_reverse': False,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        for key in ('input_timeout', 'odom_timeout', 'controller_state_timeout',
                    'feedback_timeout', 'update_rate', 'goal_refresh_period',
                    'projection_horizon', 'sample_period', 'max_linear',
                    'max_angular'):
            if float(self.p[key]) <= 0.0:
                raise ValueError(f'{key} must be positive')

        self.client = ActionClient(self, FollowPath, self.p['action_name'])
        lifecycle_service = self.p['controller_lifecycle_node'] + '/get_state'
        self.state_client = self.create_client(GetState, lifecycle_service)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, self.p['path_topic'], qos)
        self.gate_pub = self.create_publisher(Bool, self.p['gate_topic'], 10)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], qos)
        self.create_subscription(Twist, self.p['reference_topic'], self.reference_cb, 10)
        self.create_subscription(Odometry, self.p['odom_topic'], self.odom_cb, 10)

        self.reference = None
        self.reference_rx = None
        self.odom = None
        self.odom_rx = None
        self.controller_active = False
        self.controller_state_rx = None
        self.state_future = None
        self.state_query_at = None
        self.goal_handle = None
        self.goal_send_future = None
        self.goal_result_future = None
        self.cancel_future = None
        self.cancel_acknowledged = False
        self.cancel_attempt_at = None
        self.goal_started = None
        self.goal_command = None
        self.feedback_rx = None
        self.last_status = None
        self.status('STARTING waiting for teleop, odometry, and active controller')
        self.create_timer(1.0 / float(self.p['update_rate']), self.tick)

    def reference_cb(self, msg):
        self.reference = msg
        self.reference_rx = self.get_clock().now()

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_rx = self.get_clock().now()

    @staticmethod
    def age(now, received):
        if received is None:
            return math.inf
        age = (now - received).nanoseconds / 1.0e9
        return age if age >= 0.0 else math.inf

    def status(self, value):
        if value != self.last_status:
            self.last_status = value
            self.status_pub.publish(String(data=value))
            self.get_logger().info(value)

    def query_controller_state(self, now):
        if self.state_future is not None:
            return
        if (self.state_query_at is not None and
                self.age(now, self.state_query_at) < 0.5):
            return
        self.state_query_at = now
        if not self.state_client.service_is_ready():
            self.controller_active = False
            return
        self.state_future = self.state_client.call_async(GetState.Request())
        self.state_future.add_done_callback(self.state_response)

    def state_response(self, future):
        try:
            state = future.result().current_state
            self.controller_active = state.label == 'active'
            self.controller_state_rx = self.get_clock().now()
        except Exception as exc:
            self.get_logger().warn(f'Controller lifecycle query failed: {exc}')
            self.controller_active = False
        finally:
            self.state_future = None

    def current_intent(self, now):
        if self.age(now, self.reference_rx) > float(self.p['input_timeout']):
            return None, 'STOP stale teleop input'
        if self.reference is None:
            return None, 'STOP missing teleop input'
        linear = float(self.reference.linear.x)
        angular = float(self.reference.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            return None, 'STOP non-finite teleop input'
        if linear < 0.0 and not self.p['allow_reverse']:
            return None, 'STOP reverse teleop input rejected'
        linear = max(-float(self.p['max_linear']), min(linear, float(self.p['max_linear'])))
        angular = max(-float(self.p['max_angular']),
                      min(angular, float(self.p['max_angular'])))
        epsilon = float(self.p['command_epsilon'])
        if abs(linear) <= epsilon and abs(angular) <= epsilon:
            return None, 'STOP zero teleop input'
        return (linear, angular), 'RUNNING valid teleop intention'

    def odom_pose(self, now):
        if self.age(now, self.odom_rx) > float(self.p['odom_timeout']):
            return None, 'STOP stale odometry'
        if self.odom is None:
            return None, 'STOP missing odometry'
        if (self.odom.header.frame_id != self.p['global_frame'] or
                self.odom.child_frame_id != self.p['base_frame']):
            return None, 'STOP unexpected odometry frame'
        pose = self.odom.pose.pose
        values = (pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            return None, 'STOP non-finite odometry'
        q = pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if norm < 1.0e-6:
            return None, 'STOP invalid odometry orientation'
        qx, qy, qz, qw = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2.0 * (qw*qz + qx*qy),
                         1.0 - 2.0 * (qy*qy + qz*qz))
        return (float(pose.position.x), float(pose.position.y),
                float(pose.position.z), yaw), None

    def controller_ready(self, now):
        return (self.controller_active and
                self.age(now, self.controller_state_rx) <=
                float(self.p['controller_state_timeout']))

    def make_path(self, now, intent, odom_pose):
        x, y, z, yaw = odom_pose
        linear, angular = intent
        path = Path()
        path.header.stamp = now.to_msg()
        path.header.frame_id = self.p['global_frame']
        samples = integrate_twist(
            x, y, yaw, linear, angular,
            float(self.p['projection_horizon']), float(self.p['sample_period']))
        for px, py, heading in samples:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = px
            pose.pose.position.y = py
            pose.pose.position.z = z
            pose.pose.orientation.z = math.sin(heading / 2.0)
            pose.pose.orientation.w = math.cos(heading / 2.0)
            path.poses.append(pose)
        return path

    def command_changed(self, intent):
        if self.goal_command is None:
            return True
        epsilon = float(self.p['command_epsilon'])
        return (abs(intent[0] - self.goal_command[0]) > epsilon or
                abs(intent[1] - self.goal_command[1]) > epsilon)

    def send_goal(self, path, intent, now):
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = self.p['controller_id']
        goal.goal_checker_id = self.p['goal_checker_id']
        if hasattr(goal, 'progress_checker_id'):
            goal.progress_checker_id = 'progress_checker'
        self.path_pub.publish(path)
        self.goal_command = intent
        self.goal_started = now
        self.feedback_rx = None
        self.goal_send_future = self.client.send_goal_async(
            goal, feedback_callback=self.feedback)
        self.goal_send_future.add_done_callback(self.goal_response)
        self.status('STARTING serialized FollowPath goal')

    def goal_response(self, future):
        self.goal_send_future = None
        try:
            handle = future.result()
            if not handle.accepted:
                self.status('STOP FollowPath goal rejected')
                return
            self.goal_handle = handle
            self.cancel_acknowledged = False
            self.cancel_attempt_at = None
            self.goal_result_future = handle.get_result_async()
            self.goal_result_future.add_done_callback(
                lambda result, accepted=handle: self.goal_result(result, accepted))
        except Exception as exc:
            self.status(f'STOP FollowPath send failed: {exc}')

    def feedback(self, _feedback):
        self.feedback_rx = self.get_clock().now()

    def goal_result(self, future, handle):
        try:
            wrapped = future.result()
            detail = f'action status {wrapped.status}'
        except Exception as exc:
            detail = f'action result failed: {exc}'
        if handle is self.goal_handle:
            self.goal_handle = None
            self.goal_result_future = None
            self.cancel_future = None
            self.cancel_acknowledged = False
            self.goal_command = None
            self.feedback_rx = None
        self.status(f'STOP {detail}')

    def request_cancel(self, now):
        if self.goal_handle is None or self.cancel_future is not None:
            return
        if (self.cancel_acknowledged or
                (self.cancel_attempt_at is not None and
                 self.age(now, self.cancel_attempt_at) < 0.5)):
            return
        self.cancel_attempt_at = now
        self.cancel_future = self.goal_handle.cancel_goal_async()
        self.cancel_future.add_done_callback(self.cancel_response)

    def cancel_response(self, future):
        try:
            response = future.result()
            self.cancel_acknowledged = bool(response.goals_canceling)
            if not self.cancel_acknowledged:
                self.get_logger().warn('FollowPath cancel was not acknowledged')
        except Exception as exc:
            self.get_logger().warn(f'FollowPath cancel failed: {exc}')
            self.cancel_acknowledged = False
        finally:
            self.cancel_future = None

    def tick(self):
        now = self.get_clock().now()
        self.query_controller_state(now)
        intent, reason = self.current_intent(now)
        odom_pose, odom_reason = self.odom_pose(now)
        ready = intent is not None and odom_pose is not None and self.controller_ready(now)
        if intent is not None and odom_pose is not None and not self.controller_ready(now):
            reason = 'STOP controller inactive or lifecycle state stale'
        elif intent is not None and odom_pose is None:
            reason = odom_reason

        feedback_fresh = (
            self.goal_handle is not None and self.feedback_rx is not None and
            self.age(now, self.feedback_rx) <= float(self.p['feedback_timeout']))
        feedback_stale = (
            self.goal_handle is not None and not feedback_fresh and
            self.age(now, self.goal_started) > float(self.p['feedback_timeout']))
        replacing = False
        if ready and self.goal_handle is not None:
            refresh_due = self.age(now, self.goal_started) >= float(
                self.p['goal_refresh_period'])
            replacing = self.command_changed(intent) or refresh_due or feedback_stale

        if not ready or replacing:
            self.request_cancel(now)
        if ready and self.goal_handle is None and self.goal_send_future is None:
            if self.client.server_is_ready():
                self.send_goal(self.make_path(now, intent, odom_pose), intent, now)
            else:
                reason = 'STOP FollowPath action server unavailable'

        gate_active = (
            ready and self.goal_handle is not None and feedback_fresh and
            not replacing and not self.cancel_acknowledged and self.cancel_future is None)
        self.gate_pub.publish(Bool(data=gate_active))
        if gate_active:
            self.status('RUNNING controller tracking teleop reference')
        elif self.goal_send_future is None:
            self.status(reason)

    def stop(self):
        self.gate_pub.publish(Bool(data=False))
        if self.goal_handle is not None and self.cancel_future is None:
            self.cancel_future = self.goal_handle.cancel_goal_async()


def main():
    rclpy.init()
    node = TeleopPathAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
