# VOXL2 ROS 2 sensor stack

Startup and decoding components for streaming a VOXL2 IMU and H.265 tracking
camera into ROS 2 on an NVIDIA Jetson or AMD Radeon computer.

```text
VOXL2 (ROS 2 Foxy)                         Companion computer

imu_apps -> test2.py -> /voxl/raw_imu ----- Ethernet ---------->
tracking camera -> voxl_mpa_to_ros2
                -> /tracking_front_misp_encoded --------------->
                                      H.265 decoder -> /tracking_front/decoded
```

The repository contains the `voxl_h265_decoder` ROS package plus systemd units
for both sides of the link.

## H.265 decoder backends

The `decoder` parameter defaults to `auto` and selects the first available
backend:

1. NVIDIA Jetson: `nvv4l2decoder` with `nvvidconv`
2. VA-API, including supported AMD Radeon systems: `vah265dec`
3. Software fallback: `avdec_h265`

The Jetson path uses full-frame Annex-B input, NVDEC, and the VIC converter.
The other paths negotiate CPU-visible output through `videoconvert`.

### Dependencies

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

### Build

Place this repository under a ROS 2 workspace's `src` directory, source ROS,
and build it:

```bash
cd ~/voxl_h265_decoder_ws
source /opt/ros/$ROS_DISTRO/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 \
  colcon build --executor sequential \
  --packages-select voxl_h265_decoder --symlink-install
source install/setup.bash
```

### Run

Automatic backend selection:

```bash
ros2 run voxl_h265_decoder h265_decoder_node
```

Explicit backend selection:

```bash
# NVIDIA Jetson
ros2 run voxl_h265_decoder h265_decoder_node \
  --ros-args -p decoder:=nvv4l2decoder

# AMD/VA-API
ros2 run voxl_h265_decoder h265_decoder_node \
  --ros-args -p decoder:=vah265dec

# Software
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
five seconds.

## Orin automatic startup

The Orin unit runs as user `nvidia`, sources ROS 2 Humble and the decoder
workspace, explicitly selects NVDEC, and restarts after failures.

Build the workspace first, then install the unit:

```bash
cd ~/voxl_h265_decoder_ws/src/voxl_h265_decoder
sudo install -m 0644 systemd/orin/voxl-h265-decoder.service \
  /etc/systemd/system/voxl-h265-decoder.service
sudo systemctl daemon-reload
sudo systemctl enable --now voxl-h265-decoder.service
```

Inspect or restart it:

```bash
systemctl status voxl-h265-decoder.service
journalctl -u voxl-h265-decoder.service -f
sudo systemctl restart voxl-h265-decoder.service
```

## VOXL2 automatic startup

The VOXL2 units assume:

- ROS 2 Foxy is installed under `/opt/ros/foxy`.
- The MPA overlay is `/opt/ros/foxy/mpa_to_ros2/install`.
- The custom IMU publisher is `/home/root/test2.py`.
- The IMU and camera pipes are `imu_apps` and
  `tracking_front_misp_encoded`.
- ROS domain 0 and Fast DDS are used on both computers.

The readiness helper waits for a real IMU sample. The camera bridge performs
ModalAI's camera pass/fail test before launching. This avoids cold-boot races
where MPA pipe files exist before sensor data is ready.

Copy the VOXL2 files to the target and install them as root:

```bash
install -m 0755 systemd/voxl2/voxl-wait-for-imu \
  /usr/local/bin/voxl-wait-for-imu
install -m 0644 systemd/voxl2/voxl-test2.service \
  /etc/systemd/system/voxl-test2.service
install -m 0644 systemd/voxl2/voxl-mpa-to-ros2.service \
  /etc/systemd/system/voxl-mpa-to-ros2.service
install -m 0644 systemd/voxl2/voxl-sensors.target \
  /etc/systemd/system/voxl-sensors.target

systemctl daemon-reload
systemctl enable --now voxl-sensors.target
```

Inspect the services:

```bash
systemctl status voxl-test2.service voxl-mpa-to-ros2.service
journalctl -u voxl-test2.service -u voxl-mpa-to-ros2.service -f
```

When updating the units, reload systemd and restart both in dependency order:

```bash
systemctl stop voxl-mpa-to-ros2.service voxl-test2.service
systemctl daemon-reload
systemctl start voxl-test2.service
systemctl start voxl-mpa-to-ros2.service
```

## ROS graph checks

On the companion computer:

```bash
ros2 topic info -v /voxl/raw_imu
ros2 topic info -v /tracking_front_misp_encoded
ros2 topic hz /voxl/raw_imu
ros2 topic hz /tracking_front/decoded
```

A topic name alone does not prove that VOXL2 is publishing it: the decoder's
subscription can keep `/tracking_front_misp_encoded` visible in the ROS graph.
