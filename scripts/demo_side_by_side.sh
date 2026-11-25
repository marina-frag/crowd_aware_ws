#!/usr/bin/env bash
# Presentation demo: synchronized replay of recorded runs, RViz next to the phi-r and phi-S plots.
#
#   bash scripts/demo_side_by_side.sh compare junction:left_turn 6              # fixed vs tracking
#   bash scripts/demo_side_by_side.sh compare junction:left_turn 6 tracking changepoint
#   bash scripts/demo_side_by_side.sh cuts    junction:left_turn 6              # same frame: tau, DBSCAN, change-point
#   bash scripts/demo_side_by_side.sh runs    <run dir A> <run dir B>           # any two runs under rosbags/
#
# A method is fixed, tracking, upstream, or any text that appears in the run directory name.
# Environment: RATE=0.5  FROM=5  TO=40 (seconds after the first command)  VIEW=2d (default 3d)
#              RECORD=junction.mp4 (plays FROM..TO once, writes rosbags/results/demo/junction.mp4)
#              JUMP=0.3  TAU=0.2  EPS=0.2  BETA=0.2  (cuts view: values of the offline rows)
# Keys, after clicking the plot window: Space play/pause, left/right 2 s, Home restart, q quit.
# Read-only on the bags. Do not run it during a simulation batch: it refuses, because the
# replay load could change the timing of the runs.
set -euo pipefail
WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd); BAGS=$WS/rosbags; STEPS=$BAGS/results/steps; OUT=$BAGS/results/demo

usage(){ sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-2}"; }

pattern(){  # method -> fragment of the run directory name
  case "$1" in
    fixed)    echo "fixed";;
    tracking) echo "dynamic_split_tracking_j${JUMP:-0.3}";;
    upstream) echo "dynamic_split_upstream_j${JUMP:-0.3}";;
    *)        echo "$1";;
  esac
}

find_run(){  # scenario path seed method
  local fragment matches; fragment=$(pattern "$4")
  # the variant must follow the path directly, so hard_curve does not also match hard_curve_1m
  matches=$(cat "$STEPS"/*.txt 2>/dev/null | grep -E "^$1_medium_$2_(fixed|dynamic)" | grep -E "${fragment}.*_seed$3_" | sort -u || true)
  [[ -n $matches ]] || { echo "No run for $1:$2 seed $3 method $4" >&2; exit 1; }
  (( $(wc -l <<<"$matches") == 1 )) || echo "Several runs match $4, using the latest" >&2
  tail -1 <<<"$matches"
}

[[ $# -ge 1 ]] || usage
mode=$1; shift
labels=()
case "$mode" in
  compare|cuts)
    [[ $# -ge 2 && $1 == *:* ]] || usage
    scenario=${1%%:*}; path=${1#*:}; seed=$2; shift 2
    if [[ $mode == compare ]]; then a=${1:-fixed}; b=${2:-tracking}; methods=("$a" "$b"); else methods=("${1:-tracking}"); fi
    runs=()
    for m in "${methods[@]}"; do run=$(find_run "$scenario" "$path" "$seed" "$m"); runs+=("$run"); labels+=("$m"); done;;
  runs)
    [[ $# -eq 2 ]] || usage
    runs=("$(basename "$1")" "$(basename "$2")"); labels=(A B); mode=compare;;
  -h|--help) usage 0;;
  *) usage;;
esac
for run in "${runs[@]}"; do [[ -d $BAGS/$run/bag ]] || { echo "No bag: $BAGS/$run"; exit 1; }; done
echo "Runs:"; printf '  %s\n' "${runs[@]}"

: "${DISPLAY:?DISPLAY is required}"
if docker ps --format '{{.Names}}' | grep -qx iwalk_dwal; then
  echo "A simulation is running (container iwalk_dwal). Run the demo after the batch ends."; exit 1
fi
docker rm -f iwalk_demo >/dev/null 2>&1 || true

# Root in the container authenticates to the desktop with the session's X cookie, so the
# X access list of the machine is not changed.
xauth_file=${XAUTHORITY:-$HOME/.Xauthority}
[[ -f $xauth_file ]] || { echo "No X authority file at $xauth_file"; exit 1; }
gl=${GL:-$([[ "${XDG_SESSION_TYPE:-}" == wayland ]] && echo software || echo nvidia)}
gl_args=(-e LIBGL_ALWAYS_SOFTWARE=1)
[[ $gl == nvidia ]] && gl_args=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute
  -e __NV_PRIME_RENDER_OFFLOAD=1 -e __GLX_VENDOR_LIBRARY_NAME=nvidia)

container_runs=(); for run in "${runs[@]}"; do container_runs+=("/rosbags/$run"); done
options=(--labels "${labels[@]}" --rate "${RATE:-1.0}" --start "${FROM:-0}" --view "${VIEW:-3d}" --tau "${TAU:-0.2}" --eps "${EPS:-0.2}" --beta "${BETA:-0.2}")
[[ -n ${TO:-} ]] && options+=(--end "$TO")
out_mount=()
if [[ -n ${RECORD:-} ]]; then
  [[ $RECORD == */* ]] && { echo "RECORD is a file name, it is written to $OUT"; exit 2; }
  mkdir -p "$OUT"; out_mount=(-v "$OUT:/demo_out"); options+=(--record "$RECORD")
fi
tty=(-i); [[ -t 0 ]] && tty=(-it)
docker run --rm "${tty[@]}" --name iwalk_demo -h "$(hostname)" -e PYTHONUNBUFFERED=1 \
  -e DISPLAY -e XAUTHORITY=/tmp/demo.xauth -e QT_X11_NO_MITSHM=1 -e HOST_UID_GID="$(id -u):$(id -g)" \
  -v "$xauth_file:/tmp/demo.xauth:ro" -v /tmp/.X11-unix:/tmp/.X11-unix:ro "${gl_args[@]}" \
  -v "$BAGS:/rosbags:ro" -v "$WS/src/crowd_aware_simulation/rviz:/replay_rviz:ro" \
  -v "$WS/scripts/demo_side_by_side.py:/demo_side_by_side.py:ro" ${out_mount[@]+"${out_mount[@]}"} \
  iwalk_dwal:latest /bin/bash /dwal_entrypoint.sh python3 /demo_side_by_side.py "$mode" "${container_runs[@]}" "${options[@]}"
[[ -n ${RECORD:-} && -f $OUT/$RECORD ]] && echo "Video: $OUT/$RECORD"
exit 0
