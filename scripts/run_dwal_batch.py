#!/usr/bin/env python3
"""Sequential host runner. Keeps simulation and metric implementations unchanged."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import sqlite3
from datetime import datetime, timezone

REFERENCE='dense_crowd_dynamic_split_off_seed1_20261019T133051612349Z'
REFERENCE_SHA='4e32e403b58b3f4ad76d1a692944f552efc044504645ac386e4c7dc5b1464578'
SETTINGS_SHA='f8c03de8f47c2bb467280ffb74eadbef9b53b5e1966fd043d06d9d71586ed23e'
PILOT='dense_crowd_dynamic_split_off_seed2_20261020T133318455014Z'
EVALUATOR='/opt/dwal_ws/install/crowd_aware_simulation/share/crowd_aware_simulation/scripts/evaluation.py'
CONDITIONS=[('fixed','fixed_dwal','off'),('dynamic_no_split','dynamic_dwal','off'),
 ('dynamic_split','dynamic_dwal','off'),('fixed','fixed_dwal','on'),
 ('dynamic_no_split','dynamic_dwal','on'),('dynamic_split','dynamic_dwal','on')]


def read_json(path):
    return json.loads(path.read_text())


def validate(data,variant,controller,semantics,seed):
    expected=dict(variant=variant,controller=controller,semantics=semantics,seed=seed,scenario='dense_crowd')
    if any(data['condition'].get(k)!=v for k,v in expected.items()):raise ValueError('Condition labels differ')
    replay=data.get('reference_schedule') or {}
    if not (replay.get('completed') and replay.get('timing_valid') and replay.get('absolute_start') and
            replay.get('sha256')==REFERENCE_SHA and replay.get('messages')==1705 and replay.get('sent_messages')==1705 and
            data.get('reference_count_matches_replay') and data.get('comparison_ready') and data.get('complete_window') and
            data.get('window_start_sim_s')==120 and data.get('duration_requested_s')==60 and data.get('duration_observed_s')==60):
        raise ValueError('Incomplete/mismatched replay or evaluation window')
    extra=data.get('additional_metrics') or {}
    if extra.get('settings_sha256')!=SETTINGS_SHA:raise ValueError('Additional metric settings differ from pilot')
    values=[data.get('collision_episodes'),data.get('standstill_while_requested_s'),extra.get('personal_zone_time_s'),extra.get('minimum_human_center_distance_m')]
    for key in ['linear_acceleration_mean_abs_mps2','angular_acceleration_mean_abs_radps2','linear_jerk_mean_abs_mps3','angular_jerk_mean_abs_radps3']:
        values.append(extra.get('smoothness',{}).get(key))
        if extra.get('smoothness',{}).get(key+'_coverage_fraction',0)<.95:raise ValueError('Insufficient smoothness coverage')
    if any(v is None for v in values):raise ValueError('A retained metric is unavailable')
    if extra.get('human_joint_coverage_fraction',0)<.95:raise ValueError('Insufficient human coverage')
    # ID/oscillation metrics deliberately do not gate this batch.


def pipeline_check(run):
    """Check recorded output presence; zero velocities are legitimate outputs."""
    totals={topic:0 for topic in ('/cmd_vel_guarded','/cmd_vel_smoothed','/cmd_vel_safe','/cmd_vel')}
    files=sorted((run/'bag').glob('*.db3'))
    if not files:raise ValueError('No SQLite bag shards for pipeline check')
    for file in files:
        with sqlite3.connect(file.resolve().as_uri()+'?mode=ro',uri=True) as db:
            for topic,count in db.execute(
                'SELECT topics.name, COUNT(*) FROM messages JOIN topics ON messages.topic_id=topics.id '
                'WHERE timestamp>=? AND timestamp<? GROUP BY topics.name',
                (120_000_000_000,180_000_000_000)):
                if topic in totals:totals[topic]+=count
    if any(count==0 for count in totals.values()):
        raise ValueError('Pipeline outputs missing in evaluation window: '+str(totals))
    return totals


def logged_run(command, log, **kwargs):
    """Inherit terminal stdin for Docker -it; mirror combined output to a log."""
    log.parent.mkdir(parents=True,exist_ok=True)
    with log.open('w') as out:
        child=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,
                               text=True,errors='replace',**kwargs)
        try:
            for line in child.stdout:
                out.write(line);out.flush();print(line,end='',flush=True)
            code=child.wait()
        except BaseException:
            if child.poll() is None:
                child.send_signal(2)
                try:child.wait(timeout=45)
                except subprocess.TimeoutExpired:child.terminate();child.wait(timeout=10)
            raise
    if code:raise subprocess.CalledProcessError(code,command)


def inspect_run(root,name,condition):
    run=root/'rosbags'/name
    data=read_json(run/'analysis_v3/metrics.json')
    validate(data,*condition)
    recording=read_json(run/'recording_result.json')
    if recording.get('returncode')!=0:raise ValueError('Recorder did not finish successfully')
    pipeline_check(run)
    return run,data


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace',type=Path,default=Path.home()/'crowd_aware_ws')
    p.add_argument('--seeds',type=int,nargs='+',default=[2])
    p.add_argument('--rviz',choices=['true','false'],default='true')
    p.add_argument('--resume',type=Path,help='Resume this batch manifest; does not select arbitrary previous runs')
    p.add_argument('--execute',action='store_true',help='Without this option only print the plan')
    a=p.parse_args();root=a.workspace.expanduser().resolve()
    if len(set(a.seeds))!=len(a.seeds) or any(s<2 for s in a.seeds):p.error('Use unique seeds >=2; seed1 is the existing baseline')
    if not (root/'scripts/run_dwal_cafe.sh').is_file():p.error('Workspace launcher not found')
    source=root/'rosbags'/REFERENCE
    if not (source/'bag').is_dir():p.error('Common reference bag not found')
    manifest_path=a.resume.expanduser().resolve() if a.resume else None
    state=read_json(manifest_path) if manifest_path else {'seeds':a.seeds,'runs':{}}
    if state.get('seeds')!=a.seeds:p.error('Resume seeds must match the manifest')
    jobs=[]
    for seed in a.seeds:
        for variant,controller,semantics in CONDITIONS:
            key=f'{seed}:{variant}:{semantics}'
            name=state['runs'].get(key)
            if seed==2 and variant=='dynamic_split' and semantics=='off':
                name=name or PILOT
            if name:
                inspect_run(root,name,(variant,controller,semantics,seed))
                print('KEEP',key,name,flush=True)
            else:print('RUN ',key,flush=True)
            jobs.append((key,seed,variant,controller,semantics,name))
    if not a.execute:
        print('Plan only. Add --execute to run.');return
    if not sys.stdin.isatty():p.error('Run from an interactive terminal; existing Docker launcher uses -it')
    subprocess.run(['docker','image','inspect','iwalk_dwal:latest'],check=True,stdout=subprocess.DEVNULL)
    image=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}','iwalk_dwal:latest'],text=True).strip()
    if state.get('image_id') not in (None,image):p.error('Image changed since batch started; do not mix versions')
    state['image_id']=image
    active=subprocess.check_output(['docker','ps','-aq','--filter','name=^/iwalk_dwal$'],text=True).strip()
    if active:p.error('Container iwalk_dwal already exists; close the previous simulation first')
    if a.rviz=='true' and not os.environ.get('DISPLAY'):p.error('RVIZ=true requires DISPLAY')
    if manifest_path is None:
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        manifest_path=root/'rosbags'/'results'/'batches'/f'batch_{stamp}'/'batch.json'
    manifest_path.parent.mkdir(parents=True,exist_ok=True)
    def save():
        tmp=manifest_path.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2));tmp.replace(manifest_path)
    save();print('BATCH:',manifest_path,flush=True)
    retained=[]
    for key,seed,variant,controller,semantics,name in jobs:
        if name:
            state['runs'][key]=name;save();retained.append(root/'rosbags'/name);continue
        current_image=subprocess.check_output(['docker','image','inspect','--format','{{.Id}}','iwalk_dwal:latest'],text=True).strip()
        if current_image!=image:raise RuntimeError('Docker image changed during batch; stop and investigate')
        print(f'\nSTART {key}; replay starts at simulation time 120 s.',flush=True)
        prefix=f'dense_crowd_{variant}_{semantics}_seed{seed}_'
        before={f.name for f in (root/'rosbags').glob(prefix+'*') if f.is_dir()}
        env={**os.environ,'SCENARIO':'dense_crowd','REFERENCE_MODE':'teleop','REFERENCE_BAG':'/rosbags/'+REFERENCE,
             'DWAL_VARIANT':variant,'GUI':'false','HEADLESS':'true','RVIZ':a.rviz}
        logged_run(['bash','scripts/run_dwal_cafe.sh','run',controller,semantics,str(seed)],
                   manifest_path.parent/'logs'/f'{seed}_{variant}_{semantics}.log',cwd=root,env=env)
        # Check exact newly created directory, never guess the latest old recording.
        created=[f for f in (root/'rosbags').glob(prefix+'*') if f.is_dir() and f.name not in before]
        if len(created)!=1:raise RuntimeError(f'Expected exactly one new bag, found {len(created)}. Inspect before resuming.')
        run=created[0]
        subprocess.run(['docker','run','--rm','-v',f'{root}/rosbags:/rosbags','iwalk_dwal:latest','python3',
                        EVALUATOR,'evaluate','/rosbags/'+run.name],check=True)
        inspect_run(root,run.name,(variant,controller,semantics,seed))
        state['runs'][key]=run.name;save();retained.append(run)
        print('VALIDATED',key,run.name,flush=True)
    # Verify same actual agent definitions within every seed, independent of controller.
    check_script='''import sys,yaml,json,hashlib
from pathlib import Path
per_seed={}
for folder in sys.argv[1:]:
 r=Path(folder);c=json.loads((r/'evaluation.json').read_text())
 agents=yaml.safe_load((r/'run/agents.yaml').read_text())
 digest=hashlib.sha256(json.dumps(agents,sort_keys=True).encode()).hexdigest()
 per_seed.setdefault(c['seed'],set()).add(digest)
if any(len(v)!=1 for v in per_seed.values()):raise SystemExit('Agent definitions differ within a seed')
if len({next(iter(v)) for v in per_seed.values()})!=len(per_seed):raise SystemExit('Different seeds generated identical agents')
print('PASS: same agents within each seed; distinct agent definitions across seeds.')
'''
    container_paths=['/rosbags/'+run.name for run in retained]
    subprocess.run(['docker','run','--rm','-v',f'{root}/rosbags:/rosbags:ro','iwalk_dwal:latest','python3','-c',check_script,*container_paths],check=True)
    for seed in a.seeds:
        selected={key:name for key,name in state['runs'].items() if key.startswith(str(seed)+':')}
        destination=root/'rosbags'/'results'/f'seed_{seed}'
        destination.mkdir(parents=True,exist_ok=True)
        # Preserve previous selections. Never silently replace an existing seed summary.
        selection={'seed':seed,'runs':selected,'batch_manifest':str(manifest_path),
                   'pipeline_check':'All four recorded pipeline outputs present in 120-180 s; not a continuous health guarantee.'}
        if (destination/'selection.json').exists():
            old=read_json(destination/'selection.json')
            if old.get('runs')!=selected:raise ValueError(f'Existing selection differs: {destination}; archive explicitly first')
        output='/rosbags/'+str(destination.relative_to(root/'rosbags'))+'/results.csv'
        subprocess.run(['docker','run','--rm','-v',f'{root}/rosbags:/rosbags','iwalk_dwal:latest','python3',EVALUATOR,
                        'compare',*['/rosbags/'+name+'/analysis_v3/metrics.json' for name in selected.values()],
                        '--output',output],check=True)
        (destination/'selection.json').write_text(json.dumps(selection,indent=2))
        print('RESULTS:',destination/'results.csv',flush=True)
    state['completed']=True;save()
    print('COMPLETE:',manifest_path)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:
        print('\nBatch interrupted. No next run will start. Check that recorder/container closed before resuming.',file=sys.stderr)
        sys.exit(130)
    except (OSError,ValueError,KeyError,RuntimeError,subprocess.CalledProcessError) as error:
        print(f'BATCH STOPPED: {error}',file=sys.stderr);sys.exit(1)
