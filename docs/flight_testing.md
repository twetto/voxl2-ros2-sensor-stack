# Flight-testing notes: VOXL2 + Orin + ArduPilot

These are lessons from the 2026-09-14 NFU drone flights. On each flight a VOXL2 streamed its tracking camera and IMU to the Orin, and the FC (ArduPilot EKF3 through MAVROS) flew on mocap.

## Clocks and reboots

- **The VOXL clock can be days off, and it jumps at every VOXL reboot** (a battery swap, say). On 2026-09-14 the recording clock was the VOXL clock plus +1196764.8 s (15:08–15:41), +1201950.9 s (16:19) and +1203115.7 s (16:31–16:39).
- **Align each bag on its own.** `scripts/eval/mocap_tools.py` takes a coarse offset from receive times and refines it by cross-correlating the gyro with mocap body rates, axis by axis. Signed axes correlate at 0.85–0.94 even on smooth flights, where |ω| alone gives about 0.4.
- **After a VOXL reboot the Orin's H.265 decoder may go silent, so restart it.** Its 5 s stats line (`received=`, `decoded=`, `published=`) shows where it stops.
- **Anything buffering VOXL stamps must accept the clock coming back earlier.** `scripts/orin/imu_buffer.py` starts over when stamps go back by more than 1 s. A node that rejected every newer-but-earlier stamp as out of order stayed silent after a reboot.

## Recorded data

- **Bags recorded mid-stream have no H.265 VPS/SPS/PPS**, because the VOXL encoder sends them only once, at stream start. Decode from the first IDR frame with the decoder's fallback parameter sets prepended. `scripts/eval/h265.py` does this without ROS (PyAV), as the decoder node does.
- **VRPN sends every pose twice.** The copies are interleaved with the next poses and stamped about 3 ms later, so dropping only back-to-back repeats removes just 9–22% of them. Keep the first copy of each identical pose (`mocap_tools.mocap_track`) before differentiating for velocity.
- **MAVROS streams are slow by default:** FC attitude at 4 Hz and local position at 2 Hz. Raise `SRn_EXTRA1` and `SRn_POSITION` for the Orin's serial port if something on the Orin needs them.
- **The downward rangefinder** (MAVROS `Range`, 2 Hz) tracked mocap height with correlation 0.999 and 2 cm spread. It reads 0.105 m below the mocap origin.
- `scripts/orin/record_flight` records all of the above in one bag.

## VIO on the flights

Horizontal ATE (m) against mocap, SE(3)-aligned from arming to landing. OpenVINS and ECHO-LI ran offline on the recorded camera and IMU (`tools/estimators/`); qvio is as recorded.

| flight | qvio | OpenVINS | ECHO-LI |
|---|---|---|---|
| 150826 3 m hop at 0.65 m | 0.26 | 0.25 | 0.05 |
| 152018 forward/back, heading +1° | 36 | 0.15 | 0.05 |
| 152903 right/left, −91° | 17 | 0.15 | 0.08 |
| 153521 back/forth, −178° | 17 | 0.18 | 0.13 |
| 154136 left/right, +86° | 9.01 | 0.22 | 0.06 |
| 161901 rectangle, −96° | 1.01 | 0.07 | 0.10 |
| 163912 rectangle, yaw turns | 4.11 | 0.53 | 0.40 |
| **median** | 9.01 | 0.18 | 0.08 |

- **qvio** diverged 7–15 s after arming on every flight at 1.6 m.
- **OpenVINS** needs `init_dyn_use: true` when the drone is armed straight after recording starts (150826); its 30 Hz `track_frequency` gate drops about 36% of frames because of stamp jitter.
- **ECHO-LI** used the tuned `eqvio_voxl2.yaml`, saved in `tools/estimators/echoli/tools/`.
- **The yaw-turning rectangle is hardest for everyone.**
- Both offline runs used the internal ID 1 calibration. ID 2 scored the same or worse, but which unit is on the drone isn't confirmed.

## ArduPilot EKF3 and an external velocity source

Read from the Copter 4.3.8 and 4.6.3 source; the plan is now to run the estimator on the Orin and send a pose.

- **`VISION_SPEED_ESTIMATE` alone isn't usable** (`EK3_SRCn_POSXY=0`, `VELXY=6`). The velocity is fused, but the EKF never reports a valid relative position, so Loiter and PosHold refuse and an in-flight switch trips the EKF failsafe after about 10 s.
- **Velocity-only aiding means `VISION_POSITION_DELTA`** (body-frame odometry, like optical flow). It keeps `horiz_pos_rel` valid. MAVROS 2.x has no plugin for it: pack it with pymavlink and publish it on the MAVROS router's `/uasN/mavlink_sink`.
- **`EK3_SRC_OPTIONS` bit 0 (FuseAllVelocities) defaults to on in 4.3–4.6.** With it on, a velocity listed in any source set is fused in every set.
- **Switching source sets:** `RCx_OPTION` 90 (low/mid/high = sets 1/2/3), or `MAV_CMD_SET_EKF_SOURCE_SET` (42007).
- **Sending a pose instead:** `scripts/orin/orin_ekf.py` is an error-state EKF on the VOXL IMU that takes a body-frame velocity, the rangefinder, and zero velocity and rate on the ground. It outputs a pose the FC takes through `vision_pose`, like mocap, so the FC's own EKF needs nothing new.
  - `test_orin_ekf.py` checks it on a simulated flight.
  - It has run offline only so far.
  - Its yaw has no measurement and drifted 4–25° per test flight.
