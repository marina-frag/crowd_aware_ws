#!/usr/bin/env python3
"""Read-only DWAL/pipeline report. Never publishes a motion command."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time
from datetime import datetime, timezone
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, PolygonStamped
from cohan_msgs.msg import TrackedAgents
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from dwal_planner.msg import ClusterGroup, SampledCluster, Path as DwalPath


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller',choices=['fixed_dwal','dynamic_dwal'],required=True)
    parser.add_argument('--semantics',choices=['on','off'],required=True)
    parser.add_argument('--seconds',type=float,default=15.0)
    args=parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds<=0:
        parser.error('--seconds must be positive and finite')
    if not hasattr(DwalPath(),'available_length'):
        parser.error('Old DWAL message installation: rebuild the image with the upstream patch')
    rclpy.init()
    node=Node('dwal_runtime_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
    stats={};subscriptions=[];odom_first=None;odom_last=None
    def callback(topic,kind):
        def receive(msg):
            nonlocal odom_first,odom_last
            now=time.monotonic()
            row=stats.setdefault(topic,{'messages':0,'max_gap_s':0.0})
            if 'last_received' in row:row['max_gap_s']=max(row['max_gap_s'],now-row['last_received'])
            row['last_received']=now;row['messages']+=1
            if kind=='command':
                v,w=msg.linear.x,msg.angular.z
                row['nonzero']=row.get('nonzero',0)+int(abs(v)>1e-5 or abs(w)>1e-5)
                row['rotation_requests']=row.get('rotation_requests',0)+int(abs(w)>1e-5)
                row['last_v_w']=[v,w]
            elif kind=='status':row['last_status']=msg.data
            elif kind=='clusters':
                lengths=[p.available_length for c in msg.clusters for p in c.paths]
                row.update(clusters=len(msg.clusters),paths=len(lengths),radii=[c.r for c in msg.clusters],
                           length_min=min(lengths,default=0),length_max=max(lengths,default=0))
                row['nonempty_messages']=row.get('nonempty_messages',0)+int(bool(lengths))
            elif kind=='samples':
                row.update(samples=len(msg.paths),free_prefixes=sum(len(p.poses)>=2 for p in msg.paths),levels=list(msg.levels))
            elif kind=='markers':
                # DELETE/DELETEALL aren't rendered paths and don't count as duplicates.
                active=[m for m in msg.markers if m.action==m.ADD]
                ids=[(m.ns,m.id) for m in active]
                row.update(active_markers=len(active),duplicate_active_ids=len(ids)-len(set(ids)),
                           short_line_strips=sum(m.type==m.LINE_STRIP and len(m.points)<2 for m in active))
            elif kind=='odom':
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                odom_last=[p.x,p.y,yaw]
                if odom_first is None:odom_first=odom_last
                row['last_pose']=odom_last
            if hasattr(msg,'header'):
                row['frame']=msg.header.frame_id
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                row['source_age_sim_s']=node.get_clock().now().nanoseconds*1e-9-stamp
        return receive
    chain=['/reference_cmd','/cmd_vel_selected','/cmd_vel_guarded','/cmd_vel_smoothed','/cmd_vel_safe','/cmd_vel']
    topics=[(t,Twist,'command') for t in chain]+[
        ('/experiment/command_guard_status',String,'status'),('/odom',Odometry,'odom'),
        ('/scan',LaserScan,'scan'),('/local_costmap/costmap',OccupancyGrid,'map'),
        ('/local_costmap/published_footprint',PolygonStamped,'footprint'),
        ('/dwal_planner/sampled_paths',SampledCluster,'samples'),
        ('/dwal_planner/clusters_far',ClusterGroup,'clusters'),
        ('/dwal_planner/sampled_pathMarkers',MarkerArray,'markers'),
        ('/dwal_planner/cluster_markers_far',MarkerArray,'markers')]
    if args.controller=='fixed_dwal':topics.append(('/dwal_planner/clusters_near',ClusterGroup,'clusters'))
    if args.semantics=='on':topics.append(('/tracked_agents',TrackedAgents,'tracks'))
    for topic,cls,kind in topics:
        subscriptions.append(node.create_subscription(cls,topic,callback(topic,kind),qos_profile_sensor_data))
    print(f'Recording {args.seconds:g}s without sending commands. In the teleop terminal hold j, then l, then i in clear space.',flush=True)
    started=time.monotonic()
    try:
        while rclpy.ok() and time.monotonic()-started<args.seconds:rclpy.spin_once(node,timeout_sec=.1)
        ended=time.monotonic()
        for topic,_,_ in topics:
            row=stats.setdefault(topic,{'messages':0})
            if 'last_received' in row:row['receive_age_s']=ended-row.pop('last_received')
            row['publishers']=[{'node':x.node_namespace.rstrip('/')+'/'+x.node_name,'type':x.topic_type}
                               for x in node.get_publishers_info_by_topic(topic)]
        nodes=[ns.rstrip('/')+'/'+name for name,ns in node.get_node_names_and_namespaces()]
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    process_commands=[]
    for proc in Path('/proc').glob('[0-9]*/cmdline'):
        try:cmd=proc.read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        if any(name in cmd for name in ('/dwal_generator','/dwal_clustering','/shared_control_node','/dynamic_dwal_node')):process_commands.append(cmd)
    parameters={}
    for name in ('/dwal_planner/dwal_generator','/dwal_planner/dwal_clustering','/shared_controller'):
        try:
            result=subprocess.run(['ros2','param','dump',name],capture_output=True,text=True,timeout=4)
            parameters[name]=result.stdout or result.stderr
        except subprocess.TimeoutExpired:parameters[name]='parameter service timeout'
    observations=[]
    if not stats['/reference_cmd'].get('nonzero',0):observations.append('No nonzero teleop reference observed; inspect input/remapping first.')
    if not stats['/reference_cmd'].get('rotation_requests',0):observations.append('No angular reference observed; this report cannot test in-place rotation.')
    for before,after in zip(chain,chain[1:]):
        if stats[before].get('nonzero',0) and not stats[after].get('nonzero',0):
            observations.append(f'Nonzero input on {before}, none observed on {after}; inspect this stage and its logs (window-level evidence).')
    if not stats['/dwal_planner/clusters_far'].get('nonempty_messages',0):observations.append('No nonempty far clusters observed; inspect generator inputs, free prefixes and min_cluster_span.')
    report={'version':'DWAL_UPSTREAM_V4','expected_controller':args.controller,'expected_semantics':args.semantics,
            'duration_wall_s':ended-started,'topics':stats,'nodes':nodes,'process_commands':process_commands,
            'parameters':parameters,'observations':observations}
    if odom_first is not None:
        report['motion']={'translation_net_m':math.hypot(odom_last[0]-odom_first[0],odom_last[1]-odom_first[1]),
                          'yaw_net_rad':math.atan2(math.sin(odom_last[2]-odom_first[2]),math.cos(odom_last[2]-odom_first[2]))}
    destination=Path('/rosbags')/('dwal_check_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.json')
    destination.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print('Topic                              messages nonzero last v,w / status')
    for topic in chain+['/experiment/command_guard_status']:
        row=stats[topic];print(f'{topic:36} {row["messages"]:6} {str(row.get("nonzero","-")):>7} {row.get("last_v_w",row.get("last_status","-"))}')
    for observation in observations:print(observation)
    print(f'Report: {destination} (host: crowd_aware_ws/rosbags/{destination.name})')
    print('Counts alone do not establish causality. Check the recorded parameters and recent node logs.')

if __name__=='__main__':main()
