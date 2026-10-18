#!/usr/bin/env python3
"""Replay ONLY a recorded reference schedule against the running simulator clock."""
import argparse
import math
import json
import hashlib
from pathlib import Path
from dwal_bag import events, locate, twist


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bag');p.add_argument('--lead-in',type=float,default=2.)
    a=p.parse_args()
    if not math.isfinite(a.lead_in) or a.lead_in<0:p.error('lead-in must be nonnegative')
    _,source_cfg=locate(a.bag)
    schedule=[(now,twist(msg)) for now,topic,msg in events(a.bag,{source_cfg['topics']['reference_cmd']})]
    if not schedule:p.error('No reference messages in bag')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    active=json.loads(Path('/tmp/dwal_active_run.json').read_text())
    target_cfg=json.loads((Path(active['run_directory'])/'evaluation.json').read_text())
    if active['reference_mode']!='teleop':p.error('Reference replay requires REFERENCE_MODE=teleop')
    canonical = [(round(now-schedule[0][0],9), float(msg.linear.x), float(msg.angular.z)) for now,msg in schedule]
    if not all(math.isfinite(value) for row in canonical for value in row):p.error('Nonfinite reference schedule')
    provenance = {'source':str(Path(a.bag).resolve()), 'sha256':hashlib.sha256(json.dumps(canonical).encode()).hexdigest(),
                  'messages':len(schedule), 'completed':False}
    provenance_path = Path(active['run_directory'])/'reference_schedule.json'
    provenance_path.write_text(json.dumps(provenance,indent=2))
    rclpy.init();node=Node('reference_replay',parameter_overrides=[Parameter('use_sim_time',value=True)])
    pub=node.create_publisher(Twist,target_cfg['topics']['reference_cmd'],10)
    ready={'odom':False}
    sub=node.create_subscription(Odometry,target_cfg['topics']['odom'],lambda _:ready.update(odom=True),qos_profile_sensor_data)
    i=0;start=None;previous=None
    try:
        print('Waiting for simulator clock and odometry; keep the keyboard teleop stopped.',flush=True)
        while rclpy.ok() and i<len(schedule):
            rclpy.spin_once(node,timeout_sec=.01);now=node.get_clock().now().nanoseconds*1e-9
            if previous is not None and now<previous:raise RuntimeError('Clock reset during replay')
            previous=now
            if start is None:
                if now>0 and ready['odom']:start=now+a.lead_in
                else:continue
            while i<len(schedule) and now>=start+schedule[i][0]-schedule[0][0]:
                pub.publish(schedule[i][1]);i+=1
        # A zero is an explicit end of the schedule, not a background teleop source.
    except KeyboardInterrupt:pass
    finally:
        provenance['completed'] = i == len(schedule)
        provenance['sent_messages'] = i
        provenance_path.write_text(json.dumps(provenance,indent=2))
        if rclpy.ok():
            pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
