#!/usr/bin/env bash
# Experiment runner: crowd_aware simulation (autonomous reference) and SCAND, then metrics.
#
#   bash ~/crowd_aware_ws/scripts/run_experiments.sh sim   [seeds...]   # default seeds 1 2 3 4 5 6
#   bash ~/crowd_aware_ws/scripts/run_experiments.sh scand
#   bash ~/crowd_aware_ws/scripts/run_experiments.sh all   [seeds...]   # sim, then scand
#   bash ~/crowd_aware_ws/scripts/run_experiments.sh nosplit [seeds...] # only dynamic_no_split (split ablation)
#
# Simulation conditions (scenario $SCENARIO, default dense_crowd; semantics $SEMANTICS, default off):
#   fixed_dwal                          (baseline, upstream)
#   dynamic_dwal dynamic_split upstream (greedy correspondence)
#   dynamic_dwal dynamic_split tracking (correspondence steps 1-6)
#   + dynamic_dwal dynamic_no_split upstream if NO_SPLIT=1
# Each run: start the simulation headless, start the recorder, record $RECORD_S seconds
# (default 110; the evaluation window is 60 s from the first motion request), stop.
# SCAND conditions: fixed_dwal, dynamic_dwal upstream, dynamic_dwal tracking; each replays the
# whole recording and stops when "SCAND recording finished" is printed.
# The correspondence mode is switched in config/experiment.yaml and restored at the end.
# Logs: ~/crowd_aware_ws/rosbags/results/experiment_logs/. Stop with Ctrl-C (the yaml is restored).
set -uo pipefail
WS=$HOME/crowd_aware_ws; SCAND=$HOME/scand_ws
YAML=$WS/src/crowd_aware_simulation/config/experiment.yaml
LOGS=$WS/rosbags/results/experiment_logs; mkdir -p "$LOGS"
SCENARIO=${SCENARIO:-dense_crowd}; SEMANTICS=${SEMANTICS:-off}; RECORD_S=${RECORD_S:-110}
mode=${1:-all}; shift || true; seeds=("${@:-1 2 3 4 5 6}"); seeds=(${seeds[@]})
original=$(grep -E '^    correspondence: ' "$YAML")
restore(){ sed -i "s/^    correspondence: .*/$original/" "$YAML"; docker stop -t 15 iwalk_dwal scand_dwal >/dev/null 2>&1; }
trap 'echo; echo "interrupted"; restore; exit 130' INT TERM
set_corr(){ sed -i "s/^    correspondence: .*/    correspondence: $1/" "$YAML"; }

sim_run(){  # controller variant correspondence seed
  local tag="${SCENARIO}_$2_$3_${SEMANTICS}_seed$4"
  echo "$(date +%T) SIM $tag"; set_corr "$3"
  (cd "$WS" && SCENARIO=$SCENARIO REFERENCE_MODE=autonomous HEADLESS=true GUI=false RVIZ=false DWAL_VARIANT=$2 \
     script -qec "bash scripts/run_dwal_cafe.sh run $1 $SEMANTICS $4" "$LOGS/sim_$tag.log" >/dev/null 2>&1) &
  local i; for i in $(seq 1 120); do docker exec iwalk_dwal true 2>/dev/null && break; sleep 1; done
  if ! docker exec iwalk_dwal true 2>/dev/null; then echo "  simulation did not start, see $LOGS/sim_$tag.log"; wait; return; fi
  sleep 4
  (cd "$WS" && script -qec "bash scripts/run_dwal_cafe.sh record $1 $SEMANTICS $4 --scenario $SCENARIO" \
     "$LOGS/rec_$tag.log" >/dev/null 2>&1) &
  sleep "$RECORD_S"
  docker exec iwalk_dwal pkill -INT -f experiment.py 2>/dev/null; sleep 10
  docker stop -t 20 iwalk_dwal >/dev/null 2>&1; wait
}

scand_run(){  # controller correspondence
  local tag="scand_$1_$2"
  echo "$(date +%T) SCAND $tag"; set_corr "$2"
  (cd "$SCAND" && script -qec "./scand/scripts/scand.sh run $1" "$LOGS/$tag.log" >/dev/null 2>&1) &
  local i; for i in $(seq 1 120); do grep -q 'SCAND recording finished' "$LOGS/$tag.log" 2>/dev/null && break; sleep 5; done
  docker kill -s INT scand_dwal >/dev/null 2>&1; sleep 12
  docker stop -t 10 scand_dwal >/dev/null 2>&1; wait
}

if [[ $mode == sim || $mode == all ]]; then
  for seed in "${seeds[@]}"; do
    sim_run fixed_dwal fixed upstream "$seed"
    sim_run dynamic_dwal dynamic_split upstream "$seed"
    sim_run dynamic_dwal dynamic_split tracking "$seed"
    [[ ${NO_SPLIT:-0} == 1 ]] && sim_run dynamic_dwal dynamic_no_split upstream "$seed"
  done
fi
if [[ $mode == nosplit ]]; then
  for seed in "${seeds[@]}"; do sim_run dynamic_dwal dynamic_no_split upstream "$seed"; done
fi
if [[ $mode == scand || $mode == all ]]; then
  scand_run fixed_dwal upstream
  scand_run dynamic_dwal upstream
  scand_run dynamic_dwal tracking
fi
restore
echo "$(date +%T) done. Metrics:"
echo "  python3 $WS/scripts/metrics_summary.py $WS/rosbags $SCAND/results --evaluate --markdown $WS/rosbags/results/summary.md --csv $WS/rosbags/results/summary.csv"
