#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-run}
case "$mode" in run|build|shell|teleop|check) ;; *) echo 'Usage: run_dwal_cafe.sh [run|build|shell|teleop|check]'; exit 2;; esac
if [[ "$mode" == check ]]; then
  exec docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh python3 /opt/dwal_ws/install/crowd_aware_simulation/share/crowd_aware_simulation/scripts/smoke_check.py
fi
if [[ "$mode" == teleop ]]; then
  exec docker exec -it iwalk_dwal /bin/bash /dwal_entrypoint.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.4
fi
if [[ "$mode" == build ]]; then
  if ! docker image inspect pmb2_hunav_simulation:latest >/dev/null 2>&1; then
    cache_dir="${XDG_CACHE_HOME:-$HOME/.cache}/iwalk-dwal"
    mkdir -p "$cache_dir"
    checkout="$cache_dir/pmb2_hunav_simulation"
    if [[ ! -d "$checkout/.git" ]]; then
      git clone https://github.com/andvatistas/pmb2_hunav_simulation.git "$checkout"
    fi
    git -C "$checkout" fetch origin
    git -C "$checkout" checkout --detach 770c5af36e860edb471fe2431eedcf6c9c3bb5d4
    docker build -t pmb2_hunav_simulation:latest "$checkout"
  fi
  exec docker build -t iwalk_dwal:latest -f "$repo_dir/docker/dwal/Dockerfile" "$repo_dir"
fi
docker image inspect iwalk_dwal:latest >/dev/null 2>&1 || { echo 'First run: bash scripts/run_dwal_cafe.sh build'; exit 1; }
: "${DISPLAY:?An X11/XWayland DISPLAY is required for Gazebo GPU lidar}"
# Grant only local root for this container, restoring that grant on exit.
granted=false
if ! xhost | grep -q 'SI:localuser:root'; then xhost +si:localuser:root; granted=true; fi
cleanup() { if [[ "$granted" == true ]]; then xhost -si:localuser:root >/dev/null; fi; }
trap cleanup EXIT
args=()
if [[ "$mode" == shell ]]; then args=(bash); fi
docker run --rm -it --name iwalk_dwal \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e ROS_DOMAIN_ID=73 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  iwalk_dwal:latest "${args[@]}"
