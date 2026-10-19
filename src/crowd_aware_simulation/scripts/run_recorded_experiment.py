#!/usr/bin/env python3
"""Own recording/replay lifetime for one freshly launched comparison run."""
import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('reference_bag');a=p.parse_args()
    active=json.loads(Path('/tmp/dwal_active_run.json').read_text())
    run=Path(active['run_directory']);here=Path(__file__).resolve().parent
    if (run/'reference_schedule.json').exists():p.error('Replay already attempted in this run; start a fresh simulation')
    if active['reference_mode']!='teleop':p.error('Requires REFERENCE_MODE=teleop')
    # Load/validate source before starting a new recording.
    from dwal_bag import locate
    locate(a.reference_bag)
    children=[]
    def interrupted(sig, frame):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    recorder=None;replay=None;code=1
    try:
        recorder=subprocess.Popen([sys.executable,str(here/'record_dwal.py'),'record',
            active['scenario'],active['controller'],active['semantics'],str(active['seed'])],
            start_new_session=True);children.append(recorder)
        replay=subprocess.Popen([sys.executable,str(here/'replay_reference.py'),a.reference_bag],
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

if __name__=='__main__':sys.exit(main())
