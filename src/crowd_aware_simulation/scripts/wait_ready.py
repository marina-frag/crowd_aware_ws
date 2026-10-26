#!/usr/bin/env python3
"""Wait for either the generated world or the DWAL clustering service."""
import argparse
import subprocess
import sys
import time
import xml.etree.ElementTree as E
from pathlib import Path


def wait_world(path):
    path = Path(path)
    deadline = time.monotonic() + 120
    while time.monotonic() < deadline:
        try:
            root = E.parse(path).getroot()
            world = root.find('world')
            if (world is not None and world.findall('actor') and
                    any(x.get('filename') == 'libHuNavPlugin.so' for x in world.findall('plugin'))):
                print('HuNav world ready:', path, flush=True)
                return 0
        except (OSError, E.ParseError):
            pass
        time.sleep(0.2)
    print('HuNav world generation timed out. Inspect hunav_loader and world_generator logs.', file=sys.stderr)
    return 1


def wait_clustering():
    end = time.monotonic() + 90
    while time.monotonic() < end:
        try:
            result = subprocess.run(
                ['ros2', 'param', 'get', '/dwal_planner/dwal_clustering', 'dwal_clustering/Marker_num'],
                capture_output=True, timeout=5)
            if result.returncode == 0 and b'Integer value' in result.stdout:
                return 0
        except subprocess.TimeoutExpired:
            pass
        time.sleep(0.2)
    print('DWAL clustering parameter service did not become ready', file=sys.stderr)
    return 1


def main():
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest='command', required=True)
    world = subparsers.add_parser('world')
    world.add_argument('path')
    subparsers.add_parser('clustering')
    args = parser.parse_args()
    return wait_world(args.path) if args.command == 'world' else wait_clustering()


if __name__ == '__main__':
    raise SystemExit(main())
