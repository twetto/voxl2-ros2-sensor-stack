# VOXL H.265 decoder

ROS 2 node that decodes the VOXL2 H.265 tracking-camera topic and publishes
CPU-resident `sensor_msgs/msg/Image` messages in `bgr8` format.

## Supported decoder backends

The `decoder` parameter defaults to `auto` and selects the first available
backend in this order:

1. NVIDIA Jetson: `nvv4l2decoder` with `nvvidconv`
2. VA-API (including supported AMD Radeon systems): `vah265dec`
3. Software fallback: `avdec_h265`

The Jetson path uses full-frame Annex-B input, NVDEC, and the VIC converter.
The VA-API and software paths negotiate CPU-visible output through
`videoconvert`.

## Dependencies

Common Ubuntu build and runtime dependencies:

```bash
sudo apt update
sudo apt install \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  pkg-config
```

Jetson systems also need the NVIDIA GStreamer plugins supplied by JetPack. If
they are missing:

```bash
sudo apt install nvidia-l4t-gstreamer
```

Verify hardware elements with one of:

```bash
gst-inspect-1.0 nvv4l2decoder nvvidconv
gst-inspect-1.0 vah265dec
```

## Build

Place this package under a ROS 2 workspace's `src` directory, source the ROS
distribution, and build it:

```bash
cd ~/voxl_h265_decoder_ws
source /opt/ros/$ROS_DISTRO/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --executor sequential \
  --packages-select voxl_h265_decoder --symlink-install
source install/setup.bash
```

## Run

Automatic backend selection:

```bash
ros2 run voxl_h265_decoder h265_decoder_node
```

Select a backend explicitly:

```bash
# NVIDIA Jetson hardware decode
ros2 run voxl_h265_decoder h265_decoder_node \
  --ros-args -p decoder:=nvv4l2decoder

# AMD/VA-API hardware decode
ros2 run voxl_h265_decoder h265_decoder_node \
  --ros-args -p decoder:=vah265dec

# Software decode
ros2 run voxl_h265_decoder h265_decoder_node \
  --ros-args -p decoder:=avdec_h265
```

Default topics:

- Input: `/tracking_front_misp_encoded` (`sensor_msgs/msg/CompressedImage`)
- Output: `/tracking_front/decoded` (`sensor_msgs/msg/Image`, `bgr8`)

Override topics and frame ID with parameters:

```bash
ros2 run voxl_h265_decoder h265_decoder_node --ros-args \
  -p input_topic:=/tracking_front_misp_encoded \
  -p output_topic:=/tracking_front/decoded \
  -p frame_id:=tracking_front
```

The node reports received, pushed, decoded, published, and dropped totals every
five seconds. Check the output rate with:

```bash
ros2 topic hz /tracking_front/decoded
```
