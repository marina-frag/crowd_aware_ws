#!/usr/bin/env python3
"""Autonomous reference path only; no legacy evaluator or metrics."""
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class ReferenceTask(Node):
    def __init__(self):
        super().__init__('local_task')
        self.xy=self.declare_parameter('path_xy',[]).value
        self.declare_parameter('timeout',120.)
        self.frame=self.declare_parameter('global_frame','odom').value
        odom=self.declare_parameter('odom_topic','/odom').value
        topic=self.declare_parameter('path_topic','/experiment/path').value
        self.pub=self.create_publisher(Path,topic,1)
        self.client=ActionClient(self,FollowPath,'/reference/follow_path')
        self.have_odom=False;self.sent=False
        self.sub=self.create_subscription(Odometry,odom,self.odom,10)
        self.timer=self.create_timer(.2,self.tick)
    def odom(self,msg):self.have_odom=msg.header.frame_id==self.frame
    def tick(self):
        if self.sent or not self.have_odom or not self.client.server_is_ready():return
        if len(self.xy)<4 or len(self.xy)%2:raise ValueError('path_xy requires at least two points')
        path=Path();path.header.frame_id=self.frame;path.header.stamp=self.get_clock().now().to_msg()
        for i in range(0,len(self.xy),2):
            pose=PoseStamped();pose.header=path.header
            pose.pose.position.x=float(self.xy[i]);pose.pose.position.y=float(self.xy[i+1])
            j=min(i+2,len(self.xy)-2);k=i if j!=i else max(0,i-2)
            yaw=math.atan2(self.xy[j+1]-self.xy[k+1],self.xy[j]-self.xy[k])
            pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);path.poses.append(pose)
        goal=FollowPath.Goal();goal.path=path;goal.controller_id='FollowPath';goal.goal_checker_id='general_goal_checker'
        self.pub.publish(path);self.sent=True
        future=self.client.send_goal_async(goal);future.add_done_callback(self.accepted)
    def accepted(self,future):
        handle=future.result()
        if not handle.accepted:self.sent=False;self.get_logger().warning('Reference goal rejected; retrying')
        else:self.result_future=handle.get_result_async()


def main():
    rclpy.init();node=ReferenceTask()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
