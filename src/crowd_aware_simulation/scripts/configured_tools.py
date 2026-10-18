"""Run teleoperation/recording with the installed, image-matched configuration."""
from datetime import datetime, timezone
import os
from pathlib import Path
import sys
from setup_config import load_configuration


def command(arguments, cfg):
    runtime, exp = cfg['runtime'], cfg['experiment']
    if arguments == ['teleop']:
        return ['ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                '--ros-args', '--remap', 'cmd_vel:='+exp['topics']['reference_cmd'],
                '-p', 'speed:='+str(runtime['teleop']['speed']),
                '-p', 'turn:='+str(runtime['teleop']['turn'])]
    if len(arguments) == 5 and arguments[0] == 'record':
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        name = '_'.join([*arguments[1:], stamp])
        topics = [exp['topics'][entry['key']] if 'key' in entry else entry['literal']
                  for entry in cfg['recording']['topics']]
        return ['ros2', 'bag', 'record', '-o', str(Path(runtime['result_directory'])/name),
                *dict.fromkeys(topics)]
    raise ValueError('Expected teleop or record SCENARIO CONTROLLER SEMANTICS SEED')


if __name__ == '__main__':
    cfg = load_configuration(Path(__file__).resolve().parents[1]/'config/experiment.yaml')
    args = command(sys.argv[1:], cfg)
    os.execvp(args[0], args)
