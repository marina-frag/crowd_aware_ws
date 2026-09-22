#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-run}
controller=${2:-fixed_dwal}
semantics=${3:-off}
seed=${4:-1}
case "$mode" in run|build|shell|teleop|check|record) ;;
  *) echo 'Usage: run_dwal_cafe.sh [run|build|shell|teleop|check|record] [fixed_dwal|dwb|hateb|dynamic_dwal] [off|on] [seed]'; exit 2;;
esac
case "$controller" in fixed_dwal|dwb|hateb|dynamic_dwal) ;;
  *) echo "Unknown controller: $controller"; exit 2;;
esac
case "$semantics" in off|on) ;; *) echo "Semantic mode must be off or on"; exit 2;; esac
if [[ "$mode" == check ]]; then
  exec docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh python3 \
    /opt/dwal_ws/install/crowd_aware_simulation/share/crowd_aware_simulation/scripts/smoke_check.py \
    --controller "$controller" --semantics "$semantics"
fi
if [[ "$mode" == teleop ]]; then
  exec docker exec -it iwalk_dwal /bin/bash /dwal_entrypoint.sh \
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    --remap cmd_vel:=/reference_cmd -p speed:=0.15 -p turn:=0.4
fi
if [[ "$mode" == record ]]; then
  stamp=$(date -u +%Y%m%dT%H%M%SZ)
  exec docker exec -it iwalk_dwal /bin/bash /dwal_entrypoint.sh ros2 bag record \
    -o "/rosbags/${controller}_${semantics}_${seed}_${stamp}" \
    /odom /scan /human_states /robot_states /tracked_agents_logging /tracked_agents /agents_info \
    /crowd_context /crowd_context/profile /experiment/dynamic_radius_state /experiment/speed_limit \
    /experiment/task_result /experiment/command_guard_status \
    /experiment/path /dwal_planner/sampled_paths /dwal_planner/clusters_near /dwal_planner/clusters_far \
    /plan_time /traj_time /teb_feedback /reference_cmd /cmd_vel_selected /cmd_vel_guarded \
    /cmd_vel_smoothed /cmd_vel_safe /cmd_vel
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
headless=${HEADLESS:-true}
gui=${GUI:-false}
rviz=${RVIZ:-false}
reference_mode=${REFERENCE_MODE:-autonomous}
radius_mode=${RADIUS_MODE:-continuous}
for value in "$headless" "$gui" "$rviz"; do
  case "$value" in true|false) ;; *) echo 'HEADLESS, GUI, and RVIZ must be true or false'; exit 2;; esac
done

case "$reference_mode" in autonomous|teleop) ;; *) echo 'REFERENCE_MODE must be autonomous or teleop'; exit 2;; esac
case "$radius_mode" in discrete|continuous) ;; *) echo 'RADIUS_MODE must be discrete or continuous'; exit 2;; esac
mkdir -p "$repo_dir/rosbags"

docker_args=(--rm -it --name iwalk_dwal -e ROS_DOMAIN_ID=73 -v "$repo_dir/rosbags:/rosbags")
granted=false
if [[ "$headless" == false || "$gui" == true || "$rviz" == true ]]; then
  : "${DISPLAY:?DISPLAY is required when graphical mode is enabled}"
  if ! xhost | grep -q 'SI:localuser:root'; then xhost +si:localuser:root; granted=true; fi
  cleanup() { if [[ "$granted" == true ]]; then xhost -si:localuser:root >/dev/null; fi; }
  trap cleanup EXIT
  docker_args+=(-e DISPLAY -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
fi

args=(
  ros2 launch crowd_aware_simulation dwal_cafe.launch.py
  "controller:=$controller"
  "semantics:=$semantics"
  "reference_mode:=$reference_mode"
  "radius_mode:=$radius_mode"
  "seed:=$seed"
  "headless:=$headless"
  "gui:=$gui"
  "rviz:=$rviz"
  "result_file:=/rosbags/metrics_${controller}_${semantics}_seed${seed}"
)

if [[ "$mode" == shell ]]; then
  args=(bash)
fi
docker run "${docker_args[@]}" iwalk_dwal:latest "${args[@]}"
