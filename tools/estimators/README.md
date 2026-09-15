# Offline VIO runs on flight bags

How OpenVINS and ECHO-LI were run on the 2026-09-14 flight bags, recorded on the Orin with the VOXL2 tracking camera (H.265), `/voxl/raw_imu` and VRPN mocap. Both runs are deterministic: frames are fed in stamp order with no bag playback, so re-running gives the same trajectory. The results and the lessons are in [`docs/flight_testing.md`](../../docs/flight_testing.md).

| Folder | What |
|---|---|
| `openvins/` | `ov_serial.cpp`, a serial runner linked against an OpenVINS Humble build (`tools/build.sh`); bag decoding; a mocap sanity check; the configs (ID 1, ID 1 with dynamic init, ID 2); `notes.md` |
| `echoli/` | the bag-prep patch for the echo-li `tools/mocap_eval` harness (bags recorded mid-stream have no VPS/SPS/PPS); a per-bag runner; TUM export; the tuned config that was used; `notes.md` |

Both write TUM files (`t x y z qx qy qz qw`), with t on the VOXL header clock, the same as `/voxl/raw_imu`. Set `BAG_DIR` to the folder holding the bags. The scripts' other paths default to the lab laptop's layout and can be overridden with the environment variables listed in each script's header.
