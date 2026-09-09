"""Read-only runtime gate; run in the same container after launch."""
import math,time,sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from dwal_planner.msg import SampledCluster,ClusterGroup

rclpy.init()
node=Node('dwal_smoke_check')
seen={}
subs=[]
def odom(msg):
    seen['odometry']=msg.header.frame_id=='odom' and msg.child_frame_id=='sim_base'
def scan(msg):
    seen['lidar']=msg.header.frame_id=='laser_frame' and any(math.isfinite(v) and msg.range_min<v<msg.range_max for v in msg.ranges)
def costmap(msg):
    seen['costmap']=msg.header.frame_id=='odom' and msg.info.width>0 and any(v>=100 for v in msg.data)
def paths(msg):
    seen['paths']=bool(msg.paths) and any(p.poses for p in msg.paths)
def clusters(msg):
    # Empty clusters can be legitimate in blocked scenes; reception is the gate.
    seen['clusters']=True
for typ,topic,fn in [(Odometry,'/odom',odom),(LaserScan,'/scan',scan),(OccupancyGrid,'/local_costmap/costmap',costmap),(SampledCluster,'/dwal_planner/sampled_paths',paths),(ClusterGroup,'/dwal_planner/clusters_near',clusters)]:
    subs.append(node.create_subscription(typ,topic,fn,qos_profile_sensor_data))
end=time.monotonic()+45
while time.monotonic()<end and not (len(seen)==5 and all(seen.values())):
    rclpy.spin_once(node,timeout_sec=0.2)
for key in ['odometry','lidar','costmap','paths','clusters']:
    print(f'{key}: {"PASS" if seen.get(key,False) else "FAIL / no valid data"}')
passed=len(seen)==5 and all(seen.values())
node.destroy_node();rclpy.shutdown()
sys.exit(0 if passed else 1)
