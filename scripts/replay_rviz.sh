#!/usr/bin/env bash
# Replay a recorded experiment bag in RViz.
#
#   bash scripts/replay_rviz.sh                      # list the recorded runs
#   bash scripts/replay_rviz.sh <run dir or pattern> [rate]
#   e.g. bash scripts/replay_rviz.sh dense_crowd_fixed_off_seed1
#
# Uses dwal_dynamic.rviz for dynamic_* runs and dwal.rviz otherwise (host copies, no build
# needed). Desired path (cyan) and executed path (orange) are shown for autonomous runs;
# bags recorded before those topics existed get them from scripts/replay_paths.py.
# Space in this terminal pauses/resumes playback (ros2 bag play keyboard controls).
set -euo pipefail
WS=$HOME/crowd_aware_ws; BAGS=$WS/rosbags
if [[ $# -lt 1 ]]; then
  echo "Recorded runs (for the same seed, dynamic_split: earlier = upstream, later = tracking):"
  for d in "$BAGS"/*/bag; do d=${d%/bag}; echo "  $(basename "$d")  [$(grep -ho 'correspondence: [a-z]*' "$d"/dwal_planner_dwal_clustering_parameters.yaml 2>/dev/null || echo -)] [$(python3 -c "import json,sys;c=json.load(open(sys.argv[1]));print(c.get('density') or 'medium', c.get('path_shape') or 'line', c.get('reference_mode','-'))" "$d/evaluation.json" 2>/dev/null)]"; done
  exit 0
fi
run=$1; rate=${2:-1.0}
if [[ ! -d "$BAGS/$run/bag" ]]; then
  matches=("$BAGS"/$run*/bag)
  [[ -d "${matches[0]}" ]] || { echo "No run matches: $run"; exit 1; }
  (( ${#matches[@]} > 1 )) && { echo "Several runs match, using the first:"; printf '  %s\n' "${matches[@]%/bag}"; }
  run=$(basename "${matches[0]%/bag}")
fi
rviz=dwal.rviz; [[ $run == *dynamic* ]] && rviz=dwal_dynamic.rviz
echo "Replaying $run with $rviz at rate $rate"

: "${DISPLAY:?DISPLAY is required}"
granted=false
if ! xhost | grep -q 'SI:localuser:root'; then xhost +si:localuser:root >/dev/null; granted=true; fi
trap '[[ $granted == true ]] && xhost -si:localuser:root >/dev/null' EXIT
gl=${GL:-$([[ "${XDG_SESSION_TYPE:-}" == wayland ]] && echo software || echo nvidia)}
gl_args=(-e LIBGL_ALWAYS_SOFTWARE=1)
[[ $gl == nvidia ]] && gl_args=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute
  -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia)

# Own container name and domain, so it does not clash with a running simulation.
tty=(-i); [[ -t 0 ]] && tty=(-it)
exec docker run --rm "${tty[@]}" --name iwalk_replay -e ROS_DOMAIN_ID=${REPLAY_DOMAIN_ID:-77} \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:ro "${gl_args[@]}" \
  -v "$BAGS:/rosbags:ro" -v "$WS/src/crowd_aware_simulation/rviz:/replay_rviz:ro" \
  -v "$WS/scripts/replay_paths.py:/replay_paths.py:ro" iwalk_dwal:latest /bin/bash /dwal_entrypoint.sh bash -c "
    share=\$(ros2 pkg prefix crowd_aware_simulation)/share/crowd_aware_simulation
    # /robot_description is not in the bag: publish it from the run's own URDF.
    urdf=/rosbags/$run/run/robot.urdf
    if [[ -f \$urdf ]]; then
      ros2 run robot_state_publisher robot_state_publisher --ros-args \
        -p use_sim_time:=true -p robot_description:=\"\$(cat \$urdf)\" >/dev/null 2>&1 &
    else echo 'No run/robot.urdf: the robot model is not shown.'; fi
    python3 /replay_paths.py /rosbags/$run >/dev/null 2>&1 &
    rviz2 -d /replay_rviz/$rviz --ros-args -p use_sim_time:=true >/dev/null 2>&1 &
    sleep 4
    # The bag also holds the recorded /clock (10 Hz). Played back next to the player's own
    # --clock, the two disagree slightly, time jumps back several times a second and RViz
    # resets (\"Detected jump back in time\"), so the view freezes. Play every topic except
    # /clock (Humble has no exclude option) and let --clock be the only time source.
    topics=\$(grep -oE 'name: /[^ ]+' /rosbags/$run/bag/metadata.yaml | cut -d' ' -f2 | grep -vx /clock | sort -u)
    ros2 bag play /rosbags/$run/bag --clock 100 --rate $rate --topics \$topics
    echo 'Playback finished. Close RViz or press Ctrl-C.'; wait"
