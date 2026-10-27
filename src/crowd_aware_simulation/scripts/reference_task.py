#!/usr/bin/env python3
"""Autonomous reference path only; no legacy evaluator or metrics."""
import math
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus


class ReferenceTask(Node):
    def __init__(self):
        super().__init__('local_task')
        self.xy=self.declare_parameter('path_xy',Parameter.Type.DOUBLE_ARRAY).value or []
        self.declare_parameter('timeout',120.)
        self.spacing=0.05
        self.frame=self.declare_parameter('global_frame','odom').value
        odom=self.declare_parameter('odom_topic','/odom').value
        topic=self.declare_parameter('path_topic','/experiment/path').value
        self.pub=self.create_publisher(Path,topic,1)
        self.client=ActionClient(self,FollowPath,'/reference/follow_path')
        self.have_odom=False;self.sent=False;self.retry_at=None
        self.sub=self.create_subscription(Odometry,odom,self.odom,10)
        self.timer=self.create_timer(.2,self.tick)
    def odom(self,msg):self.have_odom=msg.header.frame_id==self.frame
    def tick(self):
        if self.sent or not self.have_odom or not self.client.server_is_ready():return
        if self.retry_at is not None and self.get_clock().now()<self.retry_at:return
        if len(self.xy)<4 or len(self.xy)%2:raise ValueError('path_xy requires at least two points')
        path=Path();path.header.frame_id=self.frame;path.header.stamp=self.get_clock().now().to_msg()
        # RPP needs a dense path: with sparse waypoints the pruned start point
        # can sit behind the robot yet beyond the lookahead, and RPP then turns
        # around toward it.
        pts=[(float(self.xy[i]),float(self.xy[i+1])) for i in range(0,len(self.xy),2)]
        for (x0,y0),(x1,y1) in zip(pts,pts[1:]):
            yaw=math.atan2(y1-y0,x1-x0);steps=max(1,math.ceil(math.hypot(x1-x0,y1-y0)/self.spacing))
            for s in range(steps):
                self.add_pose(path,x0+(x1-x0)*s/steps,y0+(y1-y0)*s/steps,yaw)
        self.add_pose(path,*pts[-1],yaw)
        goal=FollowPath.Goal();goal.path=path;goal.controller_id='FollowPath';goal.goal_checker_id='general_goal_checker'
        self.pub.publish(path);self.sent=True
        future=self.client.send_goal_async(goal);future.add_done_callback(self.accepted)
    @staticmethod
    def add_pose(path,x,y,yaw):
        pose=PoseStamped();pose.header=path.header
        pose.pose.position.x=x;pose.pose.position.y=y
        pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);path.poses.append(pose)
    def accepted(self,future):
        handle=future.result()
        if not handle.accepted:self.sent=False;self.get_logger().warning('Reference goal rejected; retrying')
        else:
            self.result_future=handle.get_result_async()
            self.result_future.add_done_callback(self.finished)
    def finished(self,future):
        # An abort (e.g. a transient costmap obstacle while actors spawn) would
        # otherwise leave the reference stopped for the whole run.
        if future.result().status==GoalStatus.STATUS_SUCCEEDED:return
        self.get_logger().warning('Reference goal ended without success; resending in 1 s')
        self.retry_at=self.get_clock().now()+rclpy.duration.Duration(seconds=1.0);self.sent=False


def main():
    rclpy.init();node=ReferenceTask()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
