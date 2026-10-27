#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
runtime_reader="$repo_dir/src/crowd_aware_simulation/scripts/configuration.py"
runtime_default() { python3 "$runtime_reader" "$1"; }
mode=${1:-run}
controller=${2:-$(runtime_default controller)}
semantics=${3:-$(runtime_default semantics)}
seed=${4:-$(runtime_default seed)}
scenario=${SCENARIO:-$(runtime_default scenario)}
[[ "$seed" =~ ^[0-9]+$ ]] || { echo 'Seed must be a nonnegative integer'; exit 2; }
# Include untracked source files too; a successful old build must not run silently.
source_fingerprint() {
  python3 - "$repo_dir" <<'PYCODE'
import hashlib, pathlib, sys
root=pathlib.Path(sys.argv[1]); digest=hashlib.sha256()
for relative in ['docker/dwal','patches','scripts/run_dwal_cafe.sh',
                 'src/iwalk_description','src/crowd_aware_interfaces','src/crowd_aware_simulation']:
    base=root/relative
    files=[base] if base.is_file() else sorted(base.rglob('*'))
    for path in files:
        if not path.is_file() or '__pycache__' in path.parts or path.suffix=='.pyc': continue
        digest.update(str(path.relative_to(root)).encode()+b'\0')
        digest.update(path.read_bytes()+b'\0')
print(digest.hexdigest())
PYCODE
}
extra_args=("${@:5}")
while (( ${#extra_args[@]} )); do
  case "${extra_args[0]}" in
    --scenario)
      (( ${#extra_args[@]} >= 2 )) || { echo '--scenario requires a value'; exit 2; }
      scenario=${extra_args[1]}
      extra_args=("${extra_args[@]:2}")
      ;;
    *) echo "Unknown argument: ${extra_args[0]}"; exit 2;;
  esac
done
case "$mode" in run|build|shell|teleop|check|record) ;;
  *) echo 'Usage: run_dwal_cafe.sh [run|build|shell|teleop|check|record] [fixed_dwal|dynamic_dwal] [off|on] [seed] [--scenario NAME]'; exit 2;;
esac
contains_runtime_value() {
  local allowed
  while IFS= read -r allowed; do [[ "$allowed" == "$2" ]] && return 0; done < <(runtime_default "$1")
  return 1
}
[[ "$controller" == fixed_dwal || "$controller" == dynamic_dwal ]] || { echo "Unknown controller: $controller"; exit 2; }
contains_runtime_value semantic_modes "$semantics" || { echo "Unknown semantics: $semantics"; exit 2; }
[[ -f "$repo_dir/src/crowd_aware_simulation/config/scenarios/$scenario.yaml" && "$scenario" != *'/'* && "$scenario" != *'..'* ]] || { echo "Unknown scenario: $scenario"; exit 2; }
if [[ "$mode" == check ]]; then
  exec docker exec iwalk_dwal /bin/bash /dwal_entrypoint.sh python3 \
    /opt/dwal_ws/install/crowd_aware_simulation/share/crowd_aware_simulation/scripts/dwal_runtime_check.py
fi
if [[ "$mode" == teleop ]]; then
  exec docker exec -it iwalk_dwal /bin/bash /dwal_entrypoint.sh python3 \
    /opt/dwal_ws/install/crowd_aware_simulation/share/crowd_aware_simulation/scripts/configured_tools.py teleop
fi
if [[ "$mode" == record ]]; then
  exec docker exec -it iwalk_dwal /bin/bash /dwal_entrypoint.sh python3 \
    /opt/dwal_ws/install/crowd_aware_simulation/share/crowd_aware_simulation/scripts/experiment.py \
    record "$scenario" "$controller" "$semantics" "$seed"
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
  exec docker build --build-arg "SOURCE_SHA=$(source_fingerprint)" -t iwalk_dwal:latest -f "$repo_dir/docker/dwal/Dockerfile" "$repo_dir"
fi
docker image inspect iwalk_dwal:latest >/dev/null 2>&1 || { echo 'First run: bash scripts/run_dwal_cafe.sh build'; exit 1; }
if [[ "$mode" == run ]]; then
  expected=$(source_fingerprint)
  built=$(docker image inspect --format '{{ index .Config.Labels "org.iwalk.source-sha256" }}' iwalk_dwal:latest)
  if [[ "$expected" != "$built" ]]; then
    echo 'The image does not match the current sources/patches. Run: bash scripts/run_dwal_cafe.sh build'
    exit 1
  fi
  echo "DWAL_UPSTREAM_V4: controller=$controller semantics=$semantics source=$built"
fi
headless=${HEADLESS:-$(runtime_default headless)}
gui=${GUI:-$(runtime_default gui)}
rviz=${RVIZ:-$(runtime_default rviz)}
reference_mode=${REFERENCE_MODE:-$(runtime_default reference_mode)}
teleop_input_timeout=${TELEOP_INPUT_TIMEOUT:-}
variant=${DWAL_VARIANT:-}
for value in "$headless" "$gui" "$rviz"; do
  case "$value" in true|false) ;; *) echo 'HEADLESS, GUI, and RVIZ must be true or false'; exit 2;; esac
done

contains_runtime_value reference_modes "$reference_mode" || { echo "Invalid REFERENCE_MODE: $reference_mode"; exit 2; }
mkdir -p "$repo_dir/rosbags"

docker_args=(--rm -it --name iwalk_dwal -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$(runtime_default ros_domain_id)}" -v "$repo_dir/rosbags:/rosbags")
granted=false
if [[ "$headless" == false || "$gui" == true || "$rviz" == true ]]; then
  : "${DISPLAY:?DISPLAY is required when graphical mode is enabled}"
  if ! xhost | grep -q 'SI:localuser:root'; then xhost +si:localuser:root; granted=true; fi
  cleanup() { if [[ "$granted" == true ]]; then xhost -si:localuser:root >/dev/null; fi; }
  trap cleanup EXIT
  # NVIDIA GLX fails through XWayland (BadValue creating the context), so a
  # Wayland session falls back to software OpenGL. Override with GL=nvidia|software.
  gl=${GL:-$([[ "${XDG_SESSION_TYPE:-}" == wayland ]] && echo software || echo nvidia)}
  docker_args+=(-e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
  case "$gl" in
    nvidia) docker_args+=(--gpus all
      -e NVIDIA_DRIVER_CAPABILITIES=graphics,display,utility,compute
      -e __NV_PRIME_RENDER_OFFLOAD=1
      -e __GLX_VENDOR_LIBRARY_NAME=nvidia);;
    software) docker_args+=(-e LIBGL_ALWAYS_SOFTWARE=1);;
    *) echo 'GL must be nvidia or software'; exit 2;;
  esac
fi

args=(
  ros2 launch crowd_aware_simulation dwal_cafe.launch.py
  "controller:=$controller"
  "semantics:=$semantics"
  "reference_mode:=$reference_mode"
  "scenario:=$scenario"
  "seed:=$seed"
  "headless:=$headless"
  "gui:=$gui"
  "rviz:=$rviz"
)

if [[ -n "$variant" ]]; then
  args+=("variant:=$variant")
fi

if [[ -n "${REFERENCE_BAG:-}" ]]; then
  args+=("reference_bag:=$REFERENCE_BAG")
fi

if [[ -n "$teleop_input_timeout" ]]; then
  args+=("teleop_input_timeout:=$teleop_input_timeout")
fi

if [[ "$mode" == shell ]]; then
  docker_args+=(-v "$repo_dir:/workspace")
  args=(bash)
fi

docker run "${docker_args[@]}" iwalk_dwal:latest "${args[@]}"
