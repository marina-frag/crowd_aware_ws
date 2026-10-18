#!/usr/bin/env bash
set -euo pipefail
container=iwalk_dwal
image=iwalk_dwal:latest
running=$(docker inspect --format '{{.State.Running}}' "$container" 2>/dev/null || true)
[[ "$running" == true ]] || { echo 'Start the simulation first with run_dwal_cafe.sh run ...'; exit 1; }
current=$(docker inspect --format '{{.Image}}' "$container")
latest=$(docker image inspect --format '{{.Id}}' "$image")
[[ "$current" == "$latest" ]] || { echo 'The simulation container uses an older image. Stop it and restart the simulation after rebuilding.'; exit 1; }
: "${DISPLAY:?DISPLAY is required for the live plot}"
granted=false
if ! xhost | grep -q 'SI:localuser:root'; then xhost +si:localuser:root; granted=true; fi
cleanup() { if [[ "$granted" == true ]]; then xhost -si:localuser:root >/dev/null; fi; }
trap cleanup EXIT
# An explicit import verifies the running container, rather than assuming the build succeeded.
docker exec "$container" /bin/bash /dwal_entrypoint.sh python3 -c \
 'import numpy; import matplotlib; matplotlib.use("TkAgg"); import matplotlib.pyplot; import tkinter; print("Live plot stack: NumPy", numpy.__version__)'
docker exec -it -e DISPLAY="$DISPLAY" "$container" /bin/bash /dwal_entrypoint.sh python3 \
 /opt/dwal_ws/install/crowd_aware_simulation/share/crowd_aware_simulation/scripts/plot_dwal_live.py "$@"
