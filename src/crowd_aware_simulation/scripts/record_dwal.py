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
from evaluation_config import recording_topics


def main():
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


if __name__ == '__main__':
    sys.exit(main())
