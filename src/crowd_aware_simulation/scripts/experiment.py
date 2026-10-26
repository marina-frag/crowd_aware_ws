#!/usr/bin/env python3
"""Record the ACTIVE run, snapshot parameters, and keep labels with the bag."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import yaml
from configuration import recording_topics


def record_main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['record'])
    p.add_argument('scenario'); p.add_argument('controller', choices=['fixed_dwal', 'dynamic_dwal'])
    p.add_argument('semantics', choices=['on', 'off']); p.add_argument('seed', type=int)
    a = p.parse_args()
    active = json.loads(Path('/tmp/dwal_active_run.json').read_text())
    for key in ('scenario', 'controller', 'semantics', 'seed'):
        if active[key] != getattr(a, key):
            p.error(f'Record labels disagree with active run: {key}={active[key]!r}')
    run = Path(active['run_directory'])
    cfg = json.loads((run/'evaluation.json').read_text())
    resolved = yaml.safe_load((run/'resolved_config.yaml').read_text())
    topics = recording_topics(resolved['experiment'], cfg, a.controller)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    name = f'{a.scenario}_{active["variant"]}_{a.semantics}_seed{a.seed}_{stamp}'
    root = Path(resolved['runtime']['result_directory'])/name
    root.mkdir(parents=True, exist_ok=False)
    shutil.copytree(run, root/'run')
    cfg.update({k: active[k] for k in ('controller', 'variant', 'semantics', 'scenario', 'seed', 'reference_mode')})
    (root/'evaluation.json').write_text(json.dumps(cfg, indent=2))
    (root/'recording_topics.json').write_text(json.dumps(topics, indent=2))
    for node in ('/dwal_planner/dwal_generator', '/dwal_planner/dwal_clustering', '/shared_controller'):
        try:
            r = subprocess.run(['ros2', 'param', 'dump', node], capture_output=True, text=True, timeout=5)
            data = r.stdout if r.returncode == 0 else r.stderr
        except subprocess.TimeoutExpired:
            data = 'ERROR: parameter dump timed out\n'
        (root/(node.strip('/').replace('/', '_')+'_parameters.yaml')).write_text(data)
    transient = {'history': 'keep_last', 'depth': 1, 'reliability': 'reliable', 'durability': 'transient_local'}
    qos = {'/tf_static': transient, cfg['topics']['costmap']: transient}
    qos_file = root/'recorder_qos.yaml'; qos_file.write_text(yaml.safe_dump(qos))
    cmd = ['ros2', 'bag', 'record', '--use-sim-time', '--qos-profile-overrides-path',
           str(qos_file), '-o', str(root/'bag'), *topics]
    (root/'recorder_command.json').write_text(json.dumps(cmd, indent=2))
    print(f'Recording {root}; stop with Ctrl+C. Start before the reference replay.', flush=True)
    child = subprocess.Popen(cmd)
    def stop(_sig, _frame):
        if child.poll() is None: child.send_signal(signal.SIGINT)
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    code = child.wait()
    if (run/'reference_schedule.json').exists():
        shutil.copy2(run/'reference_schedule.json',root/'reference_schedule.json')
    # Final snapshot lists recorder metadata; missing topics are checked by the evaluator/checker.
    (root/'recording_result.json').write_text(json.dumps({'returncode': code}, indent=2))
    return code if code >= 0 else 128-code


    sys.exit(main())
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


def replay_main():
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

#!/usr/bin/env python3
"""Own recording/replay lifetime for one freshly launched comparison run."""
import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path


def experiment_main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('reference_bag');a=p.parse_args()
    active=json.loads(Path('/tmp/dwal_active_run.json').read_text())
    run=Path(active['run_directory']);here=Path(__file__).resolve().parent
    if (run/'reference_schedule.json').exists():p.error('Replay already attempted in this run; start a fresh simulation')
    if active['reference_mode']!='teleop':p.error('Requires REFERENCE_MODE=teleop')
    # Load/validate source before starting a new recording.
    from dwal_bag import locate
    locate(a.reference_bag)
    # Lifecycle state is required even when idle nodes have no velocity output.
    subprocess.run([sys.executable,str(here/'dwal_runtime_check.py'),
                    '--pipeline-only','--seconds','8'],check=True)
    children=[]
    def interrupted(sig, frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    recorder=None;replay=None;code=1
    try:
        recorder=subprocess.Popen([sys.executable,str(here/'experiment.py'),'record',
            active['scenario'],active['controller'],active['semantics'],str(active['seed'])],
            start_new_session=True);children.append(recorder)
        replay=subprocess.Popen([sys.executable,str(here/'experiment.py'),'replay',a.reference_bag],
            start_new_session=True);children.append(replay)
        while replay.poll() is None:
            if recorder.poll() is not None:raise RuntimeError('Recorder exited before replay completed')
            time.sleep(.1)
        code=replay.returncode
    except KeyboardInterrupt:
        print('Experiment interrupted; recording will close, replay marked incomplete.',flush=True)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    try:child.wait(timeout=5)
                    except subprocess.TimeoutExpired:child.kill();child.wait()
                    code=1
        if recorder is not None and recorder.returncode not in (0,130):code=1
    print(f'Experiment finished; status={code}. Recorder closed before simulator shutdown.',flush=True)
    return code

def main():
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == 'record':
        return record_main()
    if len(sys.argv) > 1 and sys.argv[1] == 'replay':
        sys.argv.pop(1)
        return replay_main()
    if len(sys.argv) > 1 and sys.argv[1] == 'run':
        sys.argv.pop(1)
    return experiment_main()

if __name__ == '__main__':
    sys.exit(main())
