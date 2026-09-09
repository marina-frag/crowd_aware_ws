#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/kiro_ws/install/setup.bash
source /home/hunav_gz_ws/install/setup.bash
source /opt/dwal_ws/install/setup.bash
export XDG_RUNTIME_DIR=/tmp/dwal-runtime
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
exec "$@"
