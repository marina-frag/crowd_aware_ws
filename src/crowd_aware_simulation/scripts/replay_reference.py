#!/usr/bin/env python3
"""Replay only reference commands at a fixed simulator time, with timing checks."""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from dwal_bag import events, locate, twist


def canonical_schedule(schedule):
    if not schedule:
        raise ValueError('No reference messages in bag')
    result = [(round(t-schedule[0][0], 9), float(m.linear.x), float(m.angular.z))
              for t, m in schedule]
    if not all(math.isfinite(v) for row in result for v in row):
        raise ValueError('Nonfinite reference schedule')
    if any(b[0] < a[0] for a,b in zip(result,result[1:])):
        raise ValueError('Reference times moved backwards')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--start-at', type=float, help='Absolute simulator seconds; default from evaluation.json')
    args=parser.parse_args()
    _, source_cfg=locate(args.bag)
    schedule=[(t,twist(m)) for t,topic,m in events(args.bag,{source_cfg['topics']['reference_cmd']})]
    canonical=canonical_schedule(schedule)
    active=json.loads(Path('/tmp/dwal_active_run.json').read_text())
    run=Path(active['run_directory']); cfg=json.loads((run/'evaluation.json').read_text())
    if active['reference_mode']!='teleop':parser.error('Requires REFERENCE_MODE=teleop; do not start keyboard teleop')
    start=args.start_at if args.start_at is not None else cfg['replay_start_sim_s']
    tolerance=cfg['replay_max_lateness_s']
    if not math.isfinite(start) or start<=0:parser.error('Invalid absolute start time')
    if canonical[-1][0] < cfg['duration_s']:
        parser.error('Reference shorter than the configured evaluation window')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    rclpy.init(); node=Node('reference_replay',parameter_overrides=[Parameter('use_sim_time',value=True)])
    topic=cfg['topics']['reference_cmd'];pub=node.create_publisher(Twist,topic,100)
    ready={'odom':None}
    def odom(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if math.isfinite(msg.twist.twist.linear.x):ready['odom']=stamp
    sub=node.create_subscription(Odometry,cfg['topics']['odom'],odom,qos_profile_sensor_data)
    provenance={'source':str(Path(args.bag).resolve()),
        'sha256':hashlib.sha256(json.dumps(canonical).encode()).hexdigest(),
        'messages':len(schedule),'completed':False,'sent_messages':0,
        'absolute_start':True,'start_sim_s':start,'timing_valid':False,
        'max_lateness_s':0.,'lateness_tolerance_s':tolerance,
        'reference_duration_s':canonical[-1][0],'evaluation_duration_s':cfg['duration_s']}
    file=run/'reference_schedule.json'
    def save():
        temp=file.with_suffix('.tmp');temp.write_text(json.dumps(provenance,indent=2));temp.replace(file)
    save();i=0;previous=None;armed=False;last_clock_change=time.monotonic();failure=None
    try:
        print(f'Replay target: simulator t={start:.3f}s. Waiting for fresh odometry, controller and recorder.',flush=True)
        while rclpy.ok() and i<len(schedule):
            rclpy.spin_once(node,timeout_sec=.002)
            now=node.get_clock().now().nanoseconds*1e-9
            if previous is not None and now<previous:raise RuntimeError('Simulator clock reset')
            if previous is None or now!=previous:last_clock_change=time.monotonic()
            if time.monotonic()-last_clock_change>30:raise RuntimeError('Simulator clock stalled for 30 wall seconds')
            previous=now
            if not armed:
                subscribers={s.node_name for s in node.get_subscriptions_info_by_topic(topic)}
                fresh=ready['odom'] is not None and -.1<=now-ready['odom']<=cfg['odom_timeout_s']
                armed=(fresh and 'shared_controller' in subscribers and
                       any('rosbag2_recorder' in name for name in subscribers) and now>0)
                if now>=start:raise RuntimeError('Start deadline missed; restart a clean run')
                if armed:print(f'Armed at t={now:.3f}s; reference starts at {start:.3f}s.',flush=True)
                continue
            if now<start:continue
            while i<len(schedule) and now>=start+canonical[i][0]:
                late=now-(start+canonical[i][0])
                provenance['max_lateness_s']=max(provenance['max_lateness_s'],late)
                if late>tolerance:raise RuntimeError(f'Replay late by {late:.3f}s; run invalid')
                pub.publish(schedule[i][1]);i+=1
        provenance['completed']=i==len(schedule)
        provenance['timing_valid']=provenance['completed']
    except (KeyboardInterrupt, Exception) as error:
        failure=f'{type(error).__name__}: {error}'
        provenance['error']=failure
    finally:
        provenance['sent_messages']=i
        if rclpy.ok():
            pub.publish(Twist());rclpy.spin_once(node,timeout_sec=.05)
        save();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps(provenance,indent=2),flush=True)
    if failure or not provenance['completed']:raise SystemExit(1)

if __name__=='__main__':main()
