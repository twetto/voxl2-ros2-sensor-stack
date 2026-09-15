#!/usr/bin/env bash
# Build ov_serial against a ROS 2 Humble OpenVINS build, with the same defines,
# include paths and flags run_subscribe_msckf was built with (read from its flags.make).
# Nothing in the OpenVINS workspace is modified.
#   OV_WS=<open_vins colcon workspace> tools/build.sh [output]
# The default layout is the lab laptop's: ~/openvins_ws_foxy with install_humble/ and
# build_humble/ (override with OV_INSTALL / OV_BUILD).
set -eo pipefail
OV_WS="${OV_WS:-$HOME/openvins_ws_foxy}"
OV_INSTALL="${OV_INSTALL:-$OV_WS/install_humble}"
OV_BUILD="${OV_BUILD:-$OV_WS/build_humble}"
source /opt/ros/humble/setup.bash
source "$OV_INSTALL/setup.bash"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-$HERE/ov_serial}"
FL="$OV_BUILD/ov_msckf/CMakeFiles/run_subscribe_msckf.dir/flags.make"
I="$OV_INSTALL"
DEFS="$(sed -n 's/^CXX_DEFINES = //p' "$FL")"
INCS="$(sed -n 's/^CXX_INCLUDES = //p' "$FL" | sed "s#-I$OV_WS/src/open_vins/ov_msckf/src#-I$I/ov_msckf/include#")"
FLAGS="$(sed -n 's/^CXX_FLAGS = //p' "$FL")"
eval g++ $FLAGS $DEFS $INCS -o "$OUT" "$HERE/ov_serial.cpp" \
  -Wl,-rpath,$I/ov_core/lib:$I/ov_init/lib:$I/ov_msckf/lib:/opt/ros/humble/lib \
  $I/ov_msckf/lib/libov_msckf_lib.so $I/ov_init/lib/libov_init_lib.so $I/ov_core/lib/libov_core_lib.so \
  -L/opt/ros/humble/lib -lrclcpp -lrcutils \
  $(pkg-config --libs opencv4) -lboost_system -lboost_filesystem
echo "built $OUT"
