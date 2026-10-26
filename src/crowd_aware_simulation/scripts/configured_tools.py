"""Configured keyboard teleop; recording is owned by experiment.py."""
import os
from pathlib import Path
import sys
from configuration import load_configuration


def command(arguments,cfg):
    if arguments!=['teleop']:raise ValueError('Use experiment.py record for recording')
    return ['ros2','run','teleop_twist_keyboard','teleop_twist_keyboard','--ros-args',
            '--remap','cmd_vel:='+cfg['experiment']['topics']['reference_cmd'],
            '-p','speed:='+str(cfg['runtime']['teleop']['speed']),
            '-p','turn:='+str(cfg['runtime']['teleop']['turn'])]

if __name__=='__main__':
    cfg=load_configuration(Path(__file__).resolve().parents[1]/'config/experiment.yaml')
    args=command(sys.argv[1:],cfg);os.execvp(args[0],args)
