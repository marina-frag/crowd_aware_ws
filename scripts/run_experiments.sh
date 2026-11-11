#!/usr/bin/env bash
# Experiment runner, one mode per protocol step (doc/input2.md), then metrics.
#
#   bash ~/crowd_aware_ws/scripts/run_experiments.sh deviation [seeds...]  # step 1: path deviation (auto)
#   bash ~/crowd_aware_ws/scripts/run_experiments.sh jump      [seeds...]  # step 2: jump sweep, dynamic_split tracking
#   JUMP=0.3 bash ~/crowd_aware_ws/scripts/run_experiments.sh compare [seeds...]  # step 3: 3 methods, sim
#   JUMP=0.3 bash ~/crowd_aware_ws/scripts/run_experiments.sh scand                # step 4: 3 methods, SCAND
#   JUMP=0.3 bash ~/crowd_aware_ws/scripts/run_experiments.sh broad   [seeds...]  # step 6: all scenarios
#
# Defaults (override with the environment variables):
#   deviation: SCENARIOS="open_area" (no obstacles near the path) DENSITIES="medium"
#              PATHS="line s_curve hard_curve", seeds 1 2 3
#   jump:      SCENARIOS="dense_crowd dense_static" DENSITIES="sparse medium dense" PATHS="line"
#              JUMPS="0.1 0.2 0.3 0.5 0.7 1.0", seeds 101..105 (tuning seeds, never reused in compare)
#   compare:   every scenario in config/scenarios, seeds 1..10; JUMP required (the step-2 result).
#              dense_crowd, dense_static: DENSITIES="sparse medium dense" x PATHS="line s_curve hard_curve";
#              the other (hand-written) scenarios have no density or path shapes: medium, line only
#              (input2.md decision M)
#   broad:     every scenario in config/scenarios, medium, line, seeds 11..60 (1..10 are in compare);
#              JUMP required
# Methods (deviation, compare, broad): METHODS="fixed tracking" (add "upstream" for
#   dynamic_split upstream); scand always runs all three.
#   fixed = fixed_dwal | upstream = dynamic_dwal dynamic_split upstream | tracking = ... tracking
# VIEW=1 opens RViz for every simulation run (desired path cyan, executed path orange).
# Each simulation run: start headless, start the recorder (the autonomous reference waits
# for it, so the bag holds the whole run), record $RECORD_S s (default 110), stop.
# A run whose reference never started, or whose lifecycle nodes did not all activate, is
# moved to rosbags_failed/ and repeated (up to 3x);
# the batch stops if the iwalk_dwal image changes while it runs.
# Logs: ~/crowd_aware_ws/rosbags/results/experiment_logs/. Every successful run is listed in
# rosbags/results/steps/<mode>.txt (metrics_summary.py --step <mode> summarises only those). Ctrl-C stops (the yaml is restored).
set -uo pipefail
WS=$HOME/crowd_aware_ws; SCAND=$HOME/scand_ws
YAML=$WS/src/crowd_aware_simulation/config/experiment.yaml
LOGS=$WS/rosbags/results/experiment_logs; mkdir -p "$LOGS"
STEPS=$WS/rosbags/results/steps  # one list of run directories per mode
SEMANTICS=${SEMANTICS:-off}; RECORD_S=${RECORD_S:-110}
RVIZ=false; [[ ${VIEW:-0} == 1 ]] && RVIZ=true
mode=${1:-}; shift || true
case "$mode" in
  deviation) d_sc="open_area"; d_den="medium"; d_path="line s_curve hard_curve"; d_seeds="1 2 3";;
  jump)      d_sc="dense_crowd dense_static"; d_den="sparse medium dense"; d_path="line"; d_seeds="101 102 103 104 105";;
  compare)   d_sc="$(ls "$WS"/src/crowd_aware_simulation/config/scenarios/*.yaml | xargs -n1 basename | sed 's/\.yaml$//' | tr '\n' ' ')"
             d_den="sparse medium dense"; d_path="line s_curve hard_curve"; d_seeds="$(seq -s ' ' 1 10)";;
  broad)     d_sc="$(ls "$WS"/src/crowd_aware_simulation/config/scenarios/*.yaml | xargs -n1 basename | sed 's/\.yaml$//' | tr '\n' ' ')"
             d_den="medium"; d_path="line"; d_seeds="$(seq -s ' ' 11 60)";;
  scand) ;;
  *) sed -n '2,26p' "$0"; exit 2;;
esac
SCENARIOS=${SCENARIOS:-${d_sc:-}}; DENSITIES=${DENSITIES:-${d_den:-}}; PATHS=${PATHS:-${d_path:-}}
seeds=(${@:-${d_seeds:-}})
if [[ $mode == compare || $mode == broad || $mode == scand ]] && [[ -z ${JUMP:-} ]]; then
  echo "JUMP is required for $mode (the value chosen in step 2)"; exit 2
fi
original=$(grep -E '^    (correspondence|length_jump_threshold): ' "$YAML")
restore(){
  while IFS= read -r line; do key=${line%%:*}; sed -i "s/^$key: .*/$line/" "$YAML"; done <<< "$original"
  docker stop -t 15 iwalk_dwal scand_dwal >/dev/null 2>&1; }
trap 'echo; echo "interrupted"; restore; exit 130' INT TERM
# One image for the whole batch (a rebuild mid-batch would mix code versions).
IMAGE=$(docker image inspect --format '{{.Id}}' iwalk_dwal:latest 2>/dev/null) || { echo 'no iwalk_dwal image: bash scripts/run_dwal_cafe.sh build'; exit 2; }
echo "$(date -Is) $mode image=$IMAGE" >> "$LOGS/batches.log"

sim_run(){  # scenario density path controller variant correspondence jump seed
  # Retries a run whose reference never started (infrastructure failure, not a controller
  # result): DDS can drop a lifecycle response at startup, so controller_server stays
  # inactive, every reference goal is rejected and the robot never moves. The failed run
  # goes to rosbags_failed/ (kept, excluded from the analysis) and is listed in failed_runs.log.
  local attempt marker bag rejected started active inactive
  for attempt in 1 2 3; do
    marker=$(mktemp); sim_run_once "$@"
    local tag="${1}_${2}_${3}_${5}_${6}${7:+_j$7}_${SEMANTICS}_seed${8}"
    rejected=$(grep -a -c 'Reference goal rejected' "$LOGS/sim_$tag.log" 2>/dev/null); rejected=${rejected:-0}
    # Every lifecycle manager that starts a bringup must report its nodes active. If one does
    # not (e.g. velocity_smoother / collision_monitor never activated), the controller computes
    # commands but the robot never moves: 4 of the 180 step-2 runs, caught by nothing else.
    started=$(grep -a -c 'Starting managed nodes bringup' "$LOGS/sim_$tag.log" 2>/dev/null); started=${started:-0}
    active=$(grep -a -c 'Managed nodes are active' "$LOGS/sim_$tag.log" 2>/dev/null); active=${active:-0}
    inactive=$(( started > active ? started - active : 0 ))
    bag=$(find "$WS/rosbags" -maxdepth 1 -mindepth 1 -type d -newer "$marker" -name '*_seed*' | head -1); rm -f "$marker"
    if (( rejected <= 25 && inactive == 0 && started > 0 )) && [[ -n $bag ]]; then
      # step manifest: metrics_summary.py --step $mode reads only the runs listed here
      mkdir -p "$STEPS"; basename "$bag" >> "$STEPS/$mode.txt"; return; fi
    echo "  attempt $attempt failed (reference goal rejected ${rejected}x, lifecycle managers not active: $inactive of $started, bag: ${bag:-none}); retrying"
    echo "$(date -Is) $tag attempt=$attempt rejected=$rejected inactive=$inactive/$started bag=$(basename "${bag:-none}")" >> "$LOGS/failed_runs.log"
    cp "$LOGS/sim_$tag.log" "$LOGS/sim_${tag}_failed$attempt.log" 2>/dev/null
    if [[ -n $bag ]]; then
      mkdir -p "$WS/rosbags_failed"
      docker run --rm -v "$WS:/ws" iwalk_dwal:latest bash -c "mv '/ws/rosbags/$(basename "$bag")' /ws/rosbags_failed/" >/dev/null 2>&1
    fi
  done
  echo "  giving up after 3 attempts: $tag"
}

sim_run_once(){  # scenario density path controller variant correspondence jump seed
  local sc=$1 den=$2 path=$3 ctl=$4 var=$5 corr=$6 jump=$7 seed=$8
  local tag="${sc}_${den}_${path}_${var}_${corr}${jump:+_j$jump}_${SEMANTICS}_seed${seed}"
  # stop cleanly before the disk fills up (a run is ~50 MB; a full disk breaks the recorder)
  local free_gb; free_gb=$(df -BG --output=avail "$WS/rosbags" | tail -1 | tr -dc 0-9)
  if (( free_gb < ${MIN_FREE_GB:-3} )); then
    echo "only ${free_gb} GB free on the disk; stopping before $tag (free space and rerun the missing runs)"
    restore; exit 1
  fi
  if [[ $(docker image inspect --format '{{.Id}}' iwalk_dwal:latest 2>/dev/null) != "$IMAGE" ]]; then
    echo "iwalk_dwal image changed during the batch; stopping (do not mix code versions)"; restore; exit 1
  fi
  echo "$(date +%T) SIM $tag"
  (cd "$WS" && SCENARIO=$sc DENSITY=$den PATH_SHAPE=$path REFERENCE_MODE=autonomous WAIT_FOR_RECORDER=true \
     HEADLESS=true GUI=false RVIZ=$RVIZ DWAL_VARIANT=$var CORRESPONDENCE=$corr JUMP=$jump \
     script -qec "bash scripts/run_dwal_cafe.sh run $ctl $SEMANTICS $seed" "$LOGS/sim_$tag.log" >/dev/null 2>&1) &
  local i; for i in $(seq 1 120); do docker exec iwalk_dwal true 2>/dev/null && break; sleep 1; done
  if ! docker exec iwalk_dwal true 2>/dev/null; then echo "  simulation did not start, see $LOGS/sim_$tag.log"; wait; return; fi
  sleep 4
  (cd "$WS" && script -qec "bash scripts/run_dwal_cafe.sh record $ctl $SEMANTICS $seed --scenario $sc" \
     "$LOGS/rec_$tag.log" >/dev/null 2>&1) &
  sleep "$RECORD_S"
  docker exec iwalk_dwal pkill -INT -f experiment.py 2>/dev/null; sleep 10
  docker stop -t 20 iwalk_dwal >/dev/null 2>&1; wait
}

methods(){  # scenario density path seed: the compared methods ($METHODS)
  local m; for m in ${METHODS:-fixed tracking}; do case "$m" in
    fixed)    sim_run "$1" "$2" "$3" fixed_dwal fixed upstream "" "$4";;
    upstream) sim_run "$1" "$2" "$3" dynamic_dwal dynamic_split upstream "${JUMP:-}" "$4";;
    tracking) sim_run "$1" "$2" "$3" dynamic_dwal dynamic_split tracking "${JUMP:-}" "$4";;
    *) echo "unknown method $m (fixed|upstream|tracking)"; exit 2;;
  esac; done
}

scand_run(){  # controller correspondence (SCAND reads config/experiment.yaml)
  local tag="scand_$1_$2_j${JUMP}"
  echo "$(date +%T) SCAND $tag"
  sed -i "s/^    correspondence: .*/    correspondence: $2/; s/^    length_jump_threshold: .*/    length_jump_threshold: $JUMP/" "$YAML"
  (cd "$SCAND" && script -qec "./scand/scripts/scand.sh run $1" "$LOGS/$tag.log" >/dev/null 2>&1) &
  local i; for i in $(seq 1 720); do grep -q 'SCAND recording finished' "$LOGS/$tag.log" 2>/dev/null && break; sleep 5; done
  docker kill -s INT scand_dwal >/dev/null 2>&1; sleep 12
  docker stop -t 10 scand_dwal >/dev/null 2>&1; wait
}

# Scenarios with a density layout and path shapes; the others run medium / line only.
LAYOUT_SCENARIOS="dense_crowd dense_static"
for seed in "${seeds[@]}"; do
  for sc in $SCENARIOS; do for den in $DENSITIES; do for path in $PATHS; do
    if [[ " $LAYOUT_SCENARIOS " != *" $sc "* ]] && [[ $den != medium || $path != line ]]; then continue; fi
    case "$mode" in
      deviation|compare|broad) methods "$sc" "$den" "$path" "$seed";;
      jump) for j in ${JUMPS:-0.1 0.2 0.3 0.5 0.7 1.0}; do
              sim_run "$sc" "$den" "$path" dynamic_dwal dynamic_split tracking "$j" "$seed"; done;;
    esac
  done; done; done
done
if [[ $mode == scand ]]; then
  scand_run fixed_dwal upstream
  scand_run dynamic_dwal upstream
  scand_run dynamic_dwal tracking
fi
restore
echo "$(date +%T) done. Metrics:"
if [[ $mode == deviation ]]; then
  echo "  python3 $WS/scripts/metrics_summary.py --metrics path_deviation --evaluate"
else
  echo "  python3 $WS/scripts/metrics_summary.py $WS/rosbags $SCAND/results --step $mode --evaluate --markdown $WS/rosbags/results/summary_$mode.md --csv $WS/rosbags/results/summary_$mode.csv"
fi
