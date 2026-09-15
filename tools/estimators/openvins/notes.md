# OpenVINS on the 2026-09-14 VOXL2 flight bags

Run 2026-09-15. Offline and deterministic (no bag playback). Calibration: VOXL2 internal ID 1.

## Outputs

- `<HHMMSS>.tum` for 150826, 152018, 152903, 153521, 154136, 161901, 163912.
  - Line format: `t x y z qx qy qz qw`.
  - Pose: the IMU pose in the OpenVINS world frame. The IMU frame is the `/voxl/raw_imu` frame (FRD: x forward, y right, z down). The world frame is gravity-aligned with z up; origin and yaw are set by the initialisation.
  - Quaternion: Hamilton q(IMU→world). OpenVINS's JPL q_GtoI has the same numbers.
  - `t`: seconds on the VOXL header clock, the same clock as the `/voxl/raw_imu` header stamps. It is the camera header stamp plus `calib_dt_CAMtoIMU`, which is 0 and not estimated.
  - One line per camera update after initialisation. No lines before init. The files include the ground segments before takeoff and after landing.
  - Update rate is ~19 Hz, not 30 Hz: see `track_frequency` below.
- `runs/id1_dyn/`: the final runs. The `.tum` files are identical to the top-level ones.
  - `.tum.state` holds `t p q v bg ba n_slam dt_cam_imu`.
  - `.log` is the OpenVINS INFO log.
  - `eval.jsonl` holds the mocap sanity metrics.
- `runs/id1`, `runs/id2`, `runs/id1_tf31`: comparison runs (see below). `runs/decode/`: decode statistics per bag.
- `config/<variant>/`: the configs used. `tools/`: every script and the runner.

## Pipeline

1. **Decode** with `tools/decode_bag.py` (PyAV).
   - Source: `/tracking_front_misp_encoded`. No bag contains a VPS, so decoding starts at the first IDR packet with voxl_h265_decoder's `fallback_codec_params` prepended. The first IDR is packet 4–27, 0.13–0.92 s into the bag. This is the method used in `flow_video.py` and `bag_to_nfu.py`.
   - Each frame is the luma plane, which is what the decoder node publishes as mono8. It keeps its packet's original header stamp; nothing is restamped.
   - All 7 bags: 0 decode errors and 0 corrupt-flagged frames, 571–1766 frames per bag.
   - The IMU is `/voxl/raw_imu` as recorded, not flipped to `/imu_apps`. The ID 1 config expects this frame:
     - `kalibr_imu_chain.yaml` sets rostopic `/voxl/raw_imu`;
     - `T_imu_cam` maps camera z→IMU x, x→y, y→z (FRD);
     - the accelerometer reads z ≈ −9.7 at rest.
   - Output is raw files, not a new ROS 2 bag: `frames.u8`, `cam_t.txt` with sec/nsec stamps, and `imu.csv`. The data is the same; this just keeps rosbag2 out of the runner.
2. **Run** with `tools/ov_serial` (`ov_serial.cpp`, built by `tools/build.sh`).
   - A standalone serial runner linked against `~/openvins_ws_foxy/install_humble`, the same `libov_msckf_lib` as `run_subscribe_msckf`. It is compiled with the defines, includes and flags from `build_humble`. Nothing in the workspace was modified.
   - It copies ROS2Visualizer's logic. `callback_inertial` feeds the IMU, then processes queued frames whose stamp is below the IMU stamp minus `calib_dt`. `callback_monocular` applies the `track_frequency` gate and an all-zero mask.
   - Events are ordered by header stamp, with the IMU first on ties.
   - The initialiser runs inline (`use_multi_threading_subs=false`, as in `ros1_serial_msckf`).
   - There is no ROS node, sim clock or decoder relay. The environment still sets `ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=91`.
   - **Deterministic:** a second run of 152018 was bit-identical in both the TUM and state files.
   - Alternatives not used:
     - `ros2 bag play` + `run_subscribe_msckf` depends on timing.
     - The Humble build has no serial mode.
     - `~/catkin_ws` `ros1_serial_msckf` is a June 2025 build of older source, and the ubuntu-22-04 box has no ROS 1.
3. **Sanity check** with `tools/eval_mocap.py` (method below).

Commands, run from this directory. Only the build step runs inside the box; `run_all.sh` enters the distrobox itself.

```
distrobox enter ubuntu-22-04 -- bash -lc "$PWD/tools/build.sh"   # once
tools/run_all.sh decode              # -> _work/<HHMMSS>/, ~8.8 GB, ~1 min
tools/run_all.sh run  id1_dyn        # -> runs/id1_dyn/<HHMMSS>.tum(.state/.log), 10-20 s per bag, 4 in parallel
tools/run_all.sh eval id1_dyn        # -> runs/id1_dyn/eval.jsonl
for b in 150826 152018 152903 153521 154136 161901 163912; do cp runs/id1_dyn/$b.tum $b.tum; done
rm -rf _work
```

## Config

**Final config: `config/id1_dyn`.** It is the repo copy of `internal_id_1_tracking_front/` (`estimator_config.yaml`, `kalibr_imucam_chain.yaml`, `kalibr_imu_chain.yaml`) with one change: `init_dyn_use: false → true`.

- **Why the change.** 150826 was armed 0.2 s into the recording and the props spin from the start (|a| std 1.3–4.8 m/s² in 1 s windows). The static initialiser never gets a still window. The repo config logs "failed static init: platform moving too much" until landing and produces no pose at all.
- **Effect on the other 6 bags: none.** The static initialiser still fires first, at spin-up, and the output is bit-identical to the unmodified repo config (compared with `cmp` on the TUM and state files).
- The comment for this change had to go on its own line: OpenCV's YAML reader rejects a boolean with an inline comment.

Repo settings left unchanged but worth knowing:

- **`track_frequency: 30.0` with a 30 Hz camera.** ROS2Visualizer drops any frame that arrives less than 1/30 s after the last accepted one. Stamp jitter puts about half of all intervals under 33.33 ms, so ~36% of frames are dropped (510 of 1443 on 152018). Updates end up at ~19 Hz with 33/67 ms spacing. The live `run_subscribe_msckf` does the same thing.
  - A copy with 31.0, which feeds every frame (`id1_tf31`), was not better (table below), so 30.0 stays.
- **`timeshift_cam_imu: 0.0`**, with `calib_cam_timeoffset: false`. The −0.0248 default in `run_voxl2_bag.sh` belongs to its live relay pipeline and was not used.
- **`save_total_state` / `filepath_*`** belong to the ROS visualiser and are unused here; nothing was written to `~/ov_estimate*.txt`.
- **IMU noise.** `accelerometer_noise_density` is 2.3e-3 m/s²/√Hz, about 0.07 m/s² per 1 kHz sample. In flight the IMU sees |a| std 3–5 m/s² from prop vibration. OpenVINS tracked anyway.

### Variants compared (ATE SE(3) RMSE in m, `runs/*/eval.jsonl`)

| bag | **id1_dyn** (final) | id1 (repo, unchanged) | id2 | id1_tf31 |
|---|---|---|---|---|
| 150826 | **0.229** | no init | not run | no init |
| 152018 | **0.172** | 0.172 | 0.219 | 0.378 |
| 152903 | **0.170** | 0.170 | 0.153 | 0.188 |
| 153521 | **0.266** | 0.266 | 0.264 | 0.239 |
| 154136 | **0.297** | 0.297 | 0.224 | 0.216 |
| 161901 | **0.093** | 0.093 | 0.094 | 0.252 |
| 163912 | **0.541** | 0.541 | 0.640 | 2.350 |

- **id2** pairs the repo `internal_id_2_tracking_front/kalibr_imucam_chain.yaml` with ID 1's estimator and IMU files, as `run_voxl2_bag.sh` does.
  - Mean over the 6 bags: 0.266 m for ID 2 against 0.257 m for ID 1. Per-bag differences go both ways and are at most 0.1 m.
  - ID 1 isn't clearly poor and ID 2 isn't better, so **ID 1 is kept**.
  - ATE cannot identify the unit here, even though the repo ID 2 factory focal length (424 px) is 9% below ID 1's (462 px). Which VOXL is on the drone remains unconfirmed.
- **id1_tf31** is ID 1 with `track_frequency: 31.0`.
  - Feeding every frame changes results erratically: 163912 reads its path 2× too long.
  - So single-run differences of ~0.1 m between configs mean little (the ECHO-LI tuning found the same "rugged objective").

## Per bag (final, id1_dyn)

Times are seconds after the first `/voxl/raw_imu` sample.
- Arming comes from `/mocap_drone_01/state`.
- Takeoff and landing are when the mocap height first and last rises 0.10 m above the height on the ground.
- ATE is SE(3)-aligned over the whole trajectory; h/v is the horizontal/vertical split.
- Sim(3) columns give the ATE after scale-aligning, and the scale factor.
- End is the aligned error at the last pose.

| bag | flight | armed | init | takeoff | landing | last pose | poses | ATE (h / v) | max err | Sim(3) ATE / scale | path OV / mocap | end |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 150826 | 3 m hop, 0.65 m | 0.2 | 2.30 (dynamic, airborne) | 1.7 | 10.6 | 19.1 | 320 | 0.229 (0.220 / 0.062) | 0.60 | 0.152 / 0.86 | 5.2 / 3.8 m | 0.05 |
| 152018 | route, +1° | 2.9 | 4.00 | 6.0 | 42.8 | 48.9 | 871 | 0.172 (0.150 / 0.084) | 0.43 | 0.136 / 0.93 | 12.7 / 11.7 m | 0.18 |
| 152903 | right–left | 5.5 | 8.00 | 10.2 | 45.7 | 49.2 | 793 | 0.170 (0.147 / 0.085) | 0.34 | 0.123 / 1.11 | 10.8 / 11.0 m | 0.09 |
| 153521 | back–forth, −178° | 2.6 | 5.00 | 6.3 | 31.2 | 34.6 | 569 | 0.266 (0.177 / 0.199) | 0.74 | 0.220 / 1.15 | 8.9 / 9.6 m | 0.10 |
| 154136 | left–right, +86° | 7.1 | 9.86 | 11.2 | 37.3 | 40.4 | 580 | 0.297 (0.216 / 0.204) | 0.75 | 0.229 / 0.86 | 11.9 / 10.0 m | 0.25 |
| 161901 | rectangle, −96° | 2.8 | 5.20 | 6.2 | 53.4 | 57.9 | 1007 | 0.093 (0.072 / 0.059) | 0.34 | 0.093 / 0.99 | 17.0 / 16.3 m | 0.08 |
| 163912 | yaw rectangle | 5.2 | 7.53 | 9.2 | 55.4 | 59.1 | 996 | 0.541 (0.525 / 0.128) | 0.98 | 0.230 / 0.72 | 22.7 / 17.0 m | 0.28 |

**Divergence: none.** All 7 track to the end of the recording, through landing and disarm.
- Final aligned error is 0.05–0.28 m.
- Estimated speed after landing is 0.015–0.075 m/s.
- The gyro bias converges to ≈ [0.0063–0.0070, −0.0019…−0.0034, 0.0012–0.0022] rad/s in every bag.
- For comparison, qvio diverged 7–15 s after arming on flights 152018–163912.
- The 6 statically initialised bags start at motor spin-up, 1.1–2.8 s after arming and 1.0–2.2 s before takeoff.

Odd things:
- **150826** is the only bag that needs the dynamic initialiser, and it initialised 0.6 s after takeoff. Treat it as lower quality.
  - The largest errors fall in the first 10 s (0.60 m).
  - The path is read 37% long (Sim(3) scale 0.86).
  - Peak |v| is 1.32 m/s, an initialisation transient.
  - Accel bias at the end is [−0.57, 0.61, 0.11] m/s², against ≈0.1 in the other bags, so it is poorly converged in a 17 s run.
- **163912** (yaw rectangle, heading −8° → +143°) is the worst.
  - The path is read 34% long (22.7 vs 17.0 m, Sim(3) scale 0.72).
  - Horizontal errors reach 0.8–1.0 m mid-flight.
  - There is a 0.275 m single-step jump at +10.1 s, about 1 s after takeoff.
  - ID 2 behaves the same (0.64 m).
- **152903** has a 0.155 m single-step jump at +11.3 s, about 1 s after takeoff. No other bag has a step above 0.1 m.
- **153521 and 154136** have their largest errors (~0.7 m) in the first 10–20 s after takeoff, then settle to 0.2–0.3 m. Their vertical RMSE is 0.20 m, against 0.06–0.13 m elsewhere.
- **152018** is missing one camera frame (a 67 ms gap in the bag itself).
- **Frames before the first IDR** (0.13–0.92 s) are lost. There is no effect, because init comes later.
- **Bootstrap frame.** The live decoder node drops its first decoded frame, while this pipeline keeps the first IDR frame. That frame falls before init.

## Sanity-check method (`tools/eval_mocap.py`)

- **Mocap cleaning** (`/vrpn_mocap/drone_01/pose`).
  - Every pose is sent twice: exactly 50% of messages are unique (4595 of 9188 on 150826).
  - The copies are interleaved with other poses, not only consecutive: consecutive repeats are just 9–22% of messages.
  - Each copy carries its own header stamp, a median 2.6–3.4 ms after the original (99th percentile < 10 ms).
  - So the eval keeps the earliest-stamped copy of each bit-identical pose, which also drops dropout freezes. Dropping only consecutive repeats would have left 30–40% of the messages as repeats.
  - Poses falling in a gap > 60 ms are not scored (at most 3 per bag).
- **Clock.** mocap stamp = VOXL stamp + 1196764.8 s (15xxxx bags), + 1201950.9 s (161901), + 1203115.7 s (163912). These match the VOXL topics' header-minus-receive offsets.
  - Cross-correlating |gyro| with the mocap |body rate| was too weak to refine the offset (peak 0.14–0.38 after cleaning), so the rough offsets are used.
  - With the earlier consecutive-only cleaning, 4 bags gave peaks > 0.5 at −22…+30 ms from the rough offset. The change of cleaning and offset moved ATE by at most 2 mm, so the offset doesn't limit these numbers.
- **Alignment.** SE(3) Umeyama on positions: OpenVINS IMU against the mocap rigid-body origin, with no lever arm, over the whole trajectory including ground segments.
  - ATE is the RMSE after alignment; the h/v split uses mocap world axes (Y up).
  - Sim(3) is reported for scale only.
  - Path lengths sum the steps of the OpenVINS track, and of mocap interpolated at the same stamps.

## Cleanup

- The decoded data (`_work/`, 8.8 GB: `frames.u8`, `imu.csv`, `cam_t.txt`) was deleted. `tools/run_all.sh decode` recreates it in ~1 min.
- Kept: `runs/` (11 MB) and the `tools/ov_serial` binary (27 MB; `tools/build.sh` rebuilds it).
