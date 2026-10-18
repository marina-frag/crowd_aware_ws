#!/usr/bin/env python3
"""Read-only readiness check for the active DWAL condition; no old metrics."""
import argparse
import json
from pathlib import Path
import time
from evaluation_config import recording_topics


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=float,default=8.);a=p.parse_args()
    if a.seconds<=0:p.error('seconds must be positive')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    import yaml
    active=json.loads(Path('/tmp/dwal_active_run.json').read_text());run=Path(active['run_directory'])
    cfg=json.loads((run/'evaluation.json').read_text());exp=yaml.safe_load((run/'resolved_config.yaml').read_text())['experiment'];t=cfg['topics']
    rclpy.init();node=Node('dwal_runtime_check');counts={};subscriptions={};errors={}
    topics=recording_topics(exp,cfg,active['controller'])
    deadline=time.monotonic()+a.seconds
    try:
        while rclpy.ok() and time.monotonic()<deadline:
            known=dict(node.get_topic_names_and_types())
            for topic in topics:
                if topic in subscriptions or topic not in known:continue
                try:
                    cls=get_message(known[topic][0])
                    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if topic in ('/tf_static',t['costmap']) else qos_profile_sensor_data
                    subscriptions[topic]=node.create_subscription(cls,topic,lambda _,name=topic:counts.update({name:counts.get(name,0)+1}),qos)
                except Exception as e:errors[topic]=str(e)
            rclpy.spin_once(node,timeout_sec=.1)
        required=[t[k] for k in ('odom','scan','costmap','sampled_paths','clusters_far','contacts')]
        if active['controller']=='fixed_dwal':required.append(t['clusters_near'])
        else:required.append(t['braking_requirement'])
        ok=all(counts.get(k,0)>0 for k in required)
        for topic in topics:print(f'{topic:55} {counts.get(topic,0):6}'+(' REQUIRED' if topic in required else ''))
        print(json.dumps({'active':active,'ready':ok,'type_errors':errors},indent=2))
        return 0 if ok else 1
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
