#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OPENVINS_WS="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
DECODER_WS="/home/twetto/voxl_h265_decoder_ws"
SENSOR_STACK_REPO="${DECODER_WS}/src/voxl_h265_decoder"
BASE_CONFIG_DIR="${SCRIPT_DIR}/config/voxl2_tracking_front"
VOXL_BAG_DOMAIN_ID=42

BAG_PATH=""
INTERNAL_ID=""
PLAYBACK_RATE="1.0"
CAMERA_IMU_TIMESHIFT="-0.0248"
DECODER_ELEMENT=""   # auto-detect by default

usage()
{
  cat <<EOF
Usage: $(basename "$0") --bag PATH --internal-id 1|2 [OPTIONS]

Required:
  --bag PATH         ROS 2 bag directory containing metadata.yaml
  --internal-id ID   Sticker ID of the VOXL2 that recorded the bag (1 or 2)

Optional:
  --rate RATE          Playback rate (default: 1.0)
  --timeshift SECONDS  OpenVINS camera-to-IMU time shift (default: -0.0264)
  --decoder ELEMENT    GStreamer H.265 decoder element (default: auto-detect
                       vah265dec → avdec_h265)
  -h, --help           Show this help
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --bag)
      if (( $# < 2 )); then
        echo "Missing value for --bag" >&2
        usage >&2
        exit 2
      fi
      BAG_PATH="$2"
      shift 2
      ;;
    --internal-id)
      if (( $# < 2 )); then
        echo "Missing value for --internal-id" >&2
        usage >&2
        exit 2
      fi
      INTERNAL_ID="$2"
      shift 2
      ;;
    --rate)
      if (( $# < 2 )); then
        echo "Missing value for --rate" >&2
        usage >&2
        exit 2
      fi
      PLAYBACK_RATE="$2"
      shift 2
      ;;
    --timeshift)
      if (( $# < 2 )); then
        echo "Missing value for --timeshift" >&2
        usage >&2
        exit 2
      fi
      CAMERA_IMU_TIMESHIFT="$2"
      shift 2
      ;;
    --decoder)
      if (( $# < 2 )); then
        echo "Missing value for --decoder" >&2
        usage >&2
        exit 2
      fi
      DECODER_ELEMENT="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${BAG_PATH}" ]]; then
  echo "--bag is required" >&2
  usage >&2
  exit 2
fi

case "${INTERNAL_ID}" in
  1|2)
    ;;
  *)
    echo "--internal-id must be 1 or 2" >&2
    usage >&2
    exit 2
    ;;
esac

if [[ ! "${PLAYBACK_RATE}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] ||
   ! awk -v rate="${PLAYBACK_RATE}" 'BEGIN { exit !(rate > 0) }'; then
  echo "--rate must be a positive number" >&2
  exit 2
fi

if [[ ! "${CAMERA_IMU_TIMESHIFT}" =~ ^[+-]?([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
  echo "--timeshift must be a signed number of seconds" >&2
  exit 2
fi

FACTORY_CONFIG_PATH="${SENSOR_STACK_REPO}/config/openvins/internal_id_${INTERNAL_ID}_tracking_front/kalibr_imucam_chain.yaml"

require_file()
{
  if [[ ! -f "$1" ]]; then
    echo "Required file not found: $1" >&2
    exit 1
  fi
}

require_directory()
{
  if [[ ! -d "$1" ]]; then
    echo "Required directory not found: $1" >&2
    exit 1
  fi
}

require_file "${BASE_CONFIG_DIR}/estimator_config.yaml"
require_file "${BASE_CONFIG_DIR}/kalibr_imu_chain.yaml"
require_file "${FACTORY_CONFIG_PATH}"
require_directory "${BAG_PATH}"
require_file "${BAG_PATH}/metadata.yaml"

# Detect which ROS 2 distro is available (Humble preferred, Foxy fallback).
if [[ -f /opt/ros/humble/setup.bash ]]; then
  ROS_SETUP=/opt/ros/humble/setup.bash
  OPENVINS_INSTALL="${OPENVINS_WS}/install_humble/setup.bash"
  DECODER_INSTALL="${DECODER_WS}/install_humble/setup.bash"
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  ROS_SETUP=/opt/ros/foxy/setup.bash
  OPENVINS_INSTALL="${OPENVINS_WS}/install/setup.bash"
  DECODER_INSTALL="${DECODER_WS}/install_foxy/setup.bash"
else
  echo "No supported ROS 2 distro found (need Humble or Foxy under /opt/ros/)" >&2
  exit 1
fi

require_file "${ROS_SETUP}"
require_file "${OPENVINS_INSTALL}"
require_file "${DECODER_INSTALL}"

# Prevent a host Jazzy/other environment from contaminating the process.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
unset PYTHONPATH LD_LIBRARY_PATH

# shellcheck disable=SC1091
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${OPENVINS_INSTALL}"
# shellcheck disable=SC1091
source "${DECODER_INSTALL}"

set -u

export ROS_DOMAIN_ID="${VOXL_BAG_DOMAIN_ID}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset ROS_LOCALHOST_ONLY

CHILD_PIDS=()
RUN_CONFIG_DIR=""

cleanup()
{
  trap - EXIT INT TERM

  # Give every child a short, bounded opportunity to stop cleanly. Some ROS 2
  # launchers spawn another process, so signal each child's entire process
  # group. Do not let cleanup wait forever if a ROS process ignores SIGTERM.
  for pid in "${CHILD_PIDS[@]}"; do
    if kill -0 -- "-${pid}" 2>/dev/null; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
    fi
  done

  local deadline=$((SECONDS + 3))
  while (( SECONDS < deadline )); do
    local children_running=false
    for pid in "${CHILD_PIDS[@]}"; do
      if kill -0 -- "-${pid}" 2>/dev/null; then
        children_running=true
        break
      fi
    done
    if [[ "${children_running}" == false ]]; then
      break
    fi
    sleep 0.1
  done

  for pid in "${CHILD_PIDS[@]}"; do
    if kill -0 -- "-${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${CHILD_PIDS[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done

  if [[ -n "${RUN_CONFIG_DIR}" && -d "${RUN_CONFIG_DIR}" ]]; then
    rm -f -- \
      "${RUN_CONFIG_DIR}/estimator_config.yaml" \
      "${RUN_CONFIG_DIR}/kalibr_imu_chain.yaml" \
      "${RUN_CONFIG_DIR}/kalibr_imucam_chain.yaml"
    rmdir -- "${RUN_CONFIG_DIR}" 2>/dev/null || true
  fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

RUN_CONFIG_DIR="$(mktemp -d "/tmp/openvins-voxl2-id-${INTERNAL_ID}.XXXXXX")"
cp -- "${BASE_CONFIG_DIR}/estimator_config.yaml" \
  "${RUN_CONFIG_DIR}/estimator_config.yaml"
cp -- "${BASE_CONFIG_DIR}/kalibr_imu_chain.yaml" \
  "${RUN_CONFIG_DIR}/kalibr_imu_chain.yaml"
cp -- "${FACTORY_CONFIG_PATH}" \
  "${RUN_CONFIG_DIR}/kalibr_imucam_chain.yaml"

TIMESHIFT_MATCHES="$(grep -Ec '^[[:space:]]*timeshift_cam_imu:' \
  "${RUN_CONFIG_DIR}/kalibr_imucam_chain.yaml" || true)"
if [[ "${TIMESHIFT_MATCHES}" != 1 ]]; then
  echo "Expected exactly one timeshift_cam_imu entry in ${FACTORY_CONFIG_PATH}" >&2
  exit 1
fi
sed -i -E \
  "s/^([[:space:]]*timeshift_cam_imu:).*/\1 ${CAMERA_IMU_TIMESHIFT}/" \
  "${RUN_CONFIG_DIR}/kalibr_imucam_chain.yaml"

CONFIG_PATH="${RUN_CONFIG_DIR}/estimator_config.yaml"

check_child()
{
  local name="$1"
  local pid="$2"
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "${name} exited during startup." >&2
    wait "${pid}" || true
    exit 1
  fi
}

echo "ROS distribution : ${ROS_DISTRO}"
echo "ROS domain       : ${ROS_DOMAIN_ID}"
echo "Bag              : ${BAG_PATH}"
echo "VOXL2 internal ID: ${INTERNAL_ID}"
echo "Playback rate    : ${PLAYBACK_RATE}x"
echo "Camera-IMU shift : ${CAMERA_IMU_TIMESHIFT} s"
echo "Factory camera   : ${FACTORY_CONFIG_PATH}"
echo "OpenVINS config  : ${CONFIG_PATH}"
echo "H.265 decoder    : (auto-detect at launch)"

# Default to "auto" — the decoder node itself tries vah265dec → avdec_h265.
DECODER_ELEMENT="${DECODER_ELEMENT:-auto}"

echo "Starting the H.265 decoder (element: ${DECODER_ELEMENT})..."
setsid ros2 run voxl_h265_decoder h265_decoder_node \
  --ros-args \
  -p use_sim_time:=true \
  -p "decoder:=${DECODER_ELEMENT}" &
DECODER_PID=$!
CHILD_PIDS+=("${DECODER_PID}")

sleep 2
check_child "H.265 decoder" "${DECODER_PID}"

EST_FILE="/home/twetto/ov_estimate.txt"
STD_FILE="/home/twetto/ov_estimate_std.txt"

echo "Starting OpenVINS..."
setsid ros2 run ov_msckf run_subscribe_msckf \
  --ros-args \
  -p "config_path:=${CONFIG_PATH}" \
  -p use_sim_time:=true \
  -p save_total_state:=true \
  -p "filepath_est:=${EST_FILE}" \
  -p "filepath_std:=${STD_FILE}" &
OPENVINS_PID=$!
CHILD_PIDS+=("${OPENVINS_PID}")

sleep 2
check_child "OpenVINS" "${OPENVINS_PID}"

echo "Playing the bag. Press Ctrl-C to stop."
ros2 bag play "${BAG_PATH}" \
  --rate "${PLAYBACK_RATE}" \
  --clock 5
