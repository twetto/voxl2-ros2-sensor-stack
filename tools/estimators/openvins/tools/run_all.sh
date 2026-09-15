#!/usr/bin/env bash
# Reproduce the OpenVINS runs on the 2026-09-14 flight bags, in a ROS 2 Humble environment.
#   tools/run_all.sh decode [HHMMSS ...]            decode the bags into _work/<HHMMSS>/
#   tools/run_all.sh run <variant> [HHMMSS ...]     config/<variant> -> runs/<variant>/
#   tools/run_all.sh eval <variant> [HHMMSS ...]    mocap sanity check -> runs/<variant>/eval.jsonl
# BAG_DIR holds the bags, one folder per flight named ..._<HHMMSS>. PY must have PyAV.
# OV_WS / OV_INSTALL as for build.sh. Build tools/ov_serial first (tools/build.sh).
set -eo pipefail
O="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_DIR="${BAG_DIR:?set BAG_DIR to the folder holding the flight bags}"
PY="${PY:-python3}"
OV_INSTALL="${OV_INSTALL:-${OV_WS:-$HOME/openvins_ws_foxy}/install_humble}"
ALL="150826 152018 152903 153521 154136 161901 163912"
bag() { ls -d "$BAG_DIR"/*_"$1" | head -1; }
# mocap clock = VOXL clock + offset; the offset changes at every VOXL reboot
offset() { case $1 in 161901) echo 1201950.9;; 163912) echo 1203115.7;; *) echo 1196764.8;; esac; }
cmd="${1:-}"; shift || true
source /opt/ros/humble/setup.bash
cd "$O"
case "$cmd" in
  decode)
    for b in ${*:-$ALL}; do "$PY" tools/decode_bag.py "$(bag "$b")" "_work/$b"; done ;;
  run)
    V="$1"; shift
    export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
    source "$OV_INSTALL/setup.bash"
    mkdir -p "runs/$V"
    for b in ${*:-$ALL}; do echo "$b"; done | xargs -P 4 -I{} sh -c \
      "tools/ov_serial config/$V/estimator_config.yaml _work/{} runs/$V/{}.tum > runs/$V/{}.log 2>&1" ;;
  eval)
    V="$1"; shift
    for b in ${*:-$ALL}; do
      [ -s "runs/$V/$b.tum" ] && "$PY" tools/eval_mocap.py "$(bag "$b")" "runs/$V/$b.tum" "$(offset "$b")"
    done > "runs/$V/eval.jsonl"
    cat "runs/$V/eval.jsonl" ;;
  *) echo "usage: $0 decode|run <variant>|eval <variant> [HHMMSS ...]" >&2; exit 2 ;;
esac
