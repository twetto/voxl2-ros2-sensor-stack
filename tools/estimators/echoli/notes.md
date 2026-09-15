# ECHO-LI offline on the 2026-09-14 VOXL2 flights

Run 2026-09-15 with the deterministic mocap_eval harness
(`~/Documents/repos/echo-li/tools/mocap_eval/`, untracked), tuned filter config,
VOXL internal ID 1 calibration. All 7 bags initialised and none diverged.

## Outputs

`<HHMMSS>.tum` for 150826, 152018, 152903, 153521, 154136, 161901 and 163912.

- Lines are `t x y z qx qy qz qw`, preceded by one `#` header line.
- The pose is T_world_imu from `VIOFilter.get_pose()`, where get_pose gives `as_xyzw()`.
  - The body is the `/voxl/raw_imu` frame.
  - The world is ECHO-LI's: gravity-aligned with z up, origin at the IMU's position at initialisation, arbitrary yaw.
  - Compare the files with an SE(3) alignment.
- There is one pose per decoded camera frame, about 30 Hz.
- **Time base.** `t` = the camera header stamp + `camera_offset` 0.004 s, the harness default taken from `run_voxl2_ros2.sh`.
  - This is the time the filter state refers to, on the **`/voxl/raw_imu` header (VOXL) clock**, so no conversion is needed.
  - Camera and IMU headers share that clock: their header−receive offsets differ by only 4–12 ms of transport latency.
  - Every file's time range lies inside its bag's IMU header range.
  - It is written from the int64 ns, so there is no float rounding.
- **VOXL clock caveat.** The VOXL clock offset changes at every VOXL reboot, so the TUM times are not one continuous timeline across bags.
  - The offset of the recording (Orin) clock over the VOXL clock is +1196764.8 s for 150826–154136, +1201950.9 s for 161901, and +1203115.7 s for 163912 (from gyro↔mocap xcorr, `sync_vrpn.json`).
  - 161901 (…71992–…72049 s) and 163912 (…72037–…72096 s) **overlap** on the VOXL clock. Don't concatenate them.
  - For a bag's mocap time, use `t_mocap = t + offset` with that bag's offset.

## Config and calibration

- **Filter and frontend config: `~/Documents/repos/echo-li/echo-li-ros2/config/eqvio_voxl2.yaml`.** This is the working-tree version: modified and uncommitted, on HEAD ec73feb, sha256 `b1530f3b…c9c`.
  - It has the values marked "tuned": eqf.maxFeatures 21, featureOutlierAbs 3.3, initialVariance.point 17, processVariance attitude 1.6e-5 / point 7.2e-5 / position 2.5e-4, velocityNoise acc 4.2e-3 / accBias 4.1e-3 / gyr 1.7e-4 / gyrBias 1.2e-4.
  - It has the camelCase RudolfV CLAHE keys (claheTileSize 256, claheClipLimit 4.0).
  - A snapshot is in `tools/eqvio_voxl2.used.yaml`.
  - The copy in `~/voxl_h265_decoder_ws/src/voxl_h265_decoder/config/echo-li/` is the older **untuned** file, with snake_case CLAHE keys that are silently ignored. It was not used.
- **Calibration: `echo-li-ros2/config/voxl2_internal_id_1.yaml`, the harness default.** It is equidistant, fx 462.46, with t_bs lever 0.037 m.
- **run_offline defaults:** `--camera-offset 0.004`, `--n-init 100`.

## What was changed / added (no tracked echo-li files touched)

These files are in `tools/` here:
- `prepare_bag_fallback.py` is a copy of `prepare_bag.py`. Only `decode_camera()` changed.
  - These bags have no VPS/SPS/PPS because the recorder started mid-stream.
  - Decoding starts at the first IDR, with voxl_h265_decoder's `fallback_codec_params` prepended. This is the same logic as `scripts/eval/h265.py`.
  - Frames before that IDR stay `cam_decoded=False`, and run_offline skips them.
  - Packets PyAV rejects are skipped, but none were rejected.
- `traj_to_tum.py` converts run_offline's traj.npz to TUM.
- `run_bag.sh` is the per-bag driver: prepare → frontend → filter → evaluate → TUM.
- `summarize.py` prints the per-bag table below. Its output is in `summary_tuned_id1.jsonl`.
- `run_<HHMMSS>_id1.log` are the driver logs. Log files were not kept for 150826, which was run in the foreground.

## Commands

```bash
# per bag, inside the ubuntu-22-04 distrobox (sets ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=92)
distrobox enter ubuntu-22-04 -- bash tools/run_bag.sh <HHMMSS> 1
# which runs:
#  prepare  : source /opt/ros/humble/setup.bash
#             python3 tools/prepare_bag_fallback.py \
#               <bag_dir> ~/.cache/echo-li/eval/flight0914_<HHMMSS>
#  frontend : ~/.cache/echo-li/ros2-humble-py3.10/venv/bin/python run_offline.py frontend CACHE \
#               --calib voxl2_internal_id_1.yaml --config eqvio_voxl2.yaml -o CACHE/tracks_id1.npz
#  filter   : ... run_offline.py filter CACHE CACHE/tracks_id1.npz --calib ... --config ... \
#               -o CACHE/runs/tuned_id1.npz
#  evaluate : python3 evaluate.py CACHE CACHE/runs/tuned_id1.npz \
#               --plot CACHE/runs/tuned_id1.png  > CACHE/runs/tuned_id1.eval.json
#  tum      : ... tools/traj_to_tum.py CACHE/runs/tuned_id1.npz <HHMMSS>.tum
python3 tools/summarize.py tuned_id1          # host, table below
```

- evaluate.py is numpy-only. It was run with a venv with PyAV and matplotlib because the runtime venv's system matplotlib fails to import against its numpy 2.x.
- The caches are `~/.cache/echo-li/eval/flight0914_<HHMMSS>/`. Each holds `info.json`, `sync_vrpn.json`, `tracks_id1.npz`, and `runs/tuned_id1.{npz,png,eval.json}`.

## Per-bag results (evaluate.py against `/vrpn_mocap/drone_01/pose`)

evaluate.py cleans the mocap, syncs the clocks by |ω| cross-correlation, fits an SE(3) alignment over the whole run, and reports ATE. The ATE includes the pre-arm and post-landing ground segments.

In the table, h/v are the horizontal and vertical ATE (VRPN is y-up), "end" is the aligned end-point error, and "corr" is the clock-sync correlation.

| bag | flight | frames decoded (first IDR) | airborne / total s | path m | ATE m | h / v | sim3 scale | rot ° | v_body m/s | end m | corr |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 150826 | 3 m hop, 0.65 m, fixed heading | 571/575 (0.13 s) | 8.8 / 19.0 | 3.4 | **0.061** | 0.050 / 0.036 | 1.02 | 0.8 | 0.063 | 0.03 | 0.68 |
| 152018 | route fwd/back, hdg +1°, 1.5 m | 1443/1466 (0.77 s) | 36.7 / 47.8 | 9.7 | **0.090** | 0.050 / 0.075 | 1.02 | 1.0 | 0.056 | 0.02 | 0.26 |
| 152903 | same route, lateral | 1465/1480 (0.50 s) | 35.5 / 48.5 | 9.0 | **0.157** | 0.080 / 0.134 | 1.06 | 1.2 | 0.051 | 0.15 | 0.57 |
| 153521 | back/forth, hdg −178° | 1033/1040 (0.23 s) | 24.7 / 34.1 | 7.4 | **0.194** | 0.138 / 0.136 | 1.12 | 1.1 | 0.087 | 0.16 | 0.45 |
| 154136 | left/right, hdg +86° | 1185/1211 (0.87 s) | 26.0 / 39.4 | 7.7 | **0.114** | 0.057 / 0.099 | 1.06 | 0.9 | 0.063 | 0.18 | 0.58 |
| 161901 | rectangle, hdg −96°, people | 1713/1740 (0.90 s) | 47.0 / 57.1 | 14.3 | **0.132** | 0.098 / 0.089 | 1.00 | 0.8 | 0.046 | 0.16 | 0.57 |
| 163912 | yaw rectangle, −8°→+143° | 1766/1777 (0.37 s) | 46.1 / 56.9 | 14.2 | **0.396** | 0.390 / 0.067 | 0.83 | 2.2 | 0.062 | 0.40 | 0.65 |

Status is the same for every bag:
- Filter output starts at the first decoded frame. Initialisation falls on the static pre-arm segment, with more than 100 IMU samples before the first IDR.
- `diverged: false` and 0 dropped images.
- No position jumps: the largest step between frames is 0.047 m.
- The frontend returned its full 300 tracks on every frame.
- ECHO-LI stayed bounded on all six flights where `/qvio/odom` diverged (7–15 s after arming).

Per-bag notes:
- **150826.** Clean; the velocity overlays match mocap closely.
- **152018.** Clean.
  - The clock-sync correlation is low (0.26) on this smooth fixed-heading flight, but the refined offset is only +7.5 ms from the bag-stamp estimate.
  - The velocity edges line up in the plot.
- **152903 / 154136.** The error is mostly vertical, 0.10–0.13 m.
- **153521.** ECHO-LI under-reads the lateral velocity during the takeoff drift (0.2 vs 0.4 m/s) and the backward legs by about 20% (−0.4 vs −0.5 m/s). It also over-reads the forward legs slightly. The sim3 scale is 1.12.
- **163912, the weakest.** The velocity is good (v_body 0.062 m/s), but the aligned rectangle is about 20% oversized (scale 0.83). This happens on the yawing loop only.
  - The takeoff lateral velocity is under-read (0.4 vs 0.55 m/s at t ≈ 9–12 s).
  - Together these leave a 0.3–0.5 m offset for the whole run and 2.2° attitude RMSE.
  - It is still bounded and qualitatively right.
- **Clock sync.** Across bags, every refinement is within ±17 ms of the bag-stamp estimate. The resulting offsets match the VOXL clock offsets known for each boot.

## Internal ID 2 check (163912 only)

ID 1 results were not clearly poor. One check was still run on the weakest bag, because this drone's unit is unconfirmed. The outputs are in the cache as `runs/tuned_id2.*`, and no TUM file was written here.

| 163912 | ATE m | ATE max | scale | rot ° | v_body m/s | RPE 5 s m |
|---|---|---|---|---|---|---|
| ID 1 | 0.396 | 0.49 | 0.83 | 2.2 | **0.062** | **0.23** |
| ID 2 | 0.341 | 0.59 | 1.03 | 3.5 | 0.105 | 0.40 |

ID 2's better global scale comes from errors that cancel. It under-reads the 20–25 s leg (0.35 vs 0.57 m/s) and distorts the rectangle. Its local velocity, RPE and attitude are clearly worse. **ID 1 is favoured, and the TUM files use ID 1.** The unit is still not confirmed: an ID 2 run on more bags would settle it (`run_bag.sh <bag> 2`).

## Disk

- The caches total about 9 GB, mostly `frames.npy`.
- There was 67 GB free after the runs, so the `frames.npy` files were kept for reruns.
- To free the space: `rm ~/.cache/echo-li/eval/flight0914_*/frames.npy`. The frontend step needs them again only if the RudolfV config or calibration changes.
