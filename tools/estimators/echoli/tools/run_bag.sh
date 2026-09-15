#!/usr/bin/env bash
# ECHO-LI offline replay of one flight bag, scored against mocap, with the echo-li
# repo's tools/mocap_eval harness. Run in a ROS 2 Humble environment:
#   BAG_DIR=<bags> tools/run_bag.sh <HHMMSS> [internal_id=1]
# Steps, each skipped if its output exists: prepare (patched for the missing
# VPS/SPS/PPS) -> frontend tracks -> filter replay -> evaluate -> TUM. Only internal
# ID 1 writes <OUT>/<HHMMSS>.tum; other IDs (calibration checks) write
# <CACHE>/runs/tuned_id<N>.tum.
#   ECHO_LI  the echo-li checkout (default ~/Documents/repos/echo-li)
#   CFG      filter config (default: tools/eqvio_voxl2.used.yaml, the tuned one used here)
#   PY       python with PyAV and matplotlib (prepare, evaluate)
#   RTPY     echo-li's runtime python (default ~/.cache/echo-li/ros2-humble-py3.10/venv/bin/python)
set -euo pipefail
B=$1
ID=${2:-1}
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-92}"

BAG_DIR="${BAG_DIR:?set BAG_DIR to the folder holding the flight bags}"
BAG=$(ls -d "$BAG_DIR"/*_"$B" | head -1)
OUT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS=$OUT/tools
ECHO_LI="${ECHO_LI:-$HOME/Documents/repos/echo-li}"
HARNESS=$ECHO_LI/tools/mocap_eval
CFG="${CFG:-$TOOLS/eqvio_voxl2.used.yaml}"
CAL=$ECHO_LI/echo-li-ros2/config/voxl2_internal_id_$ID.yaml
CACHE="${CACHE_ROOT:-$HOME/.cache/echo-li/eval}/flight0914_$B"
PY="${PY:-python3}"
RTPY="${RTPY:-$HOME/.cache/echo-li/ros2-humble-py3.10/venv/bin/python}"

mkdir -p "$CACHE/runs"
if [ ! -f "$CACHE/sensors.npz" ]; then
    set +u; source /opt/ros/humble/setup.bash; set -u
    "$PY" "$TOOLS/prepare_bag_fallback.py" "$BAG" "$CACHE" 2>&1 | tee "$CACHE/prepare.log"
fi
if [ ! -f "$CACHE/tracks_id$ID.npz" ]; then
    "$RTPY" "$HARNESS/run_offline.py" frontend "$CACHE" --calib "$CAL" --config "$CFG" \
        -o "$CACHE/tracks_id$ID.npz" 2>&1 | tee "$CACHE/frontend_id$ID.log"
fi
RUN=$CACHE/runs/tuned_id$ID
"$RTPY" "$HARNESS/run_offline.py" filter "$CACHE" "$CACHE/tracks_id$ID.npz" \
    --calib "$CAL" --config "$CFG" -o "$RUN.npz" 2>&1 | tee "$RUN.filter.log"
# evaluate.py is numpy-only; PY also has a working matplotlib for the plot
"$PY" "$HARNESS/evaluate.py" "$CACHE" "$RUN.npz" --plot "$RUN.png" 2>&1 | tee "$RUN.eval.json"
if [ "$ID" = 1 ]; then
    "$RTPY" "$TOOLS/traj_to_tum.py" "$RUN.npz" "$OUT/$B.tum"
else   # calibration check only: keep it out of the estimator output directory
    "$RTPY" "$TOOLS/traj_to_tum.py" "$RUN.npz" "$RUN.tum"
fi
