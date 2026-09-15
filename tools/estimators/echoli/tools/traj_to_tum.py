#!/usr/bin/env python3
"""run_offline.py filter output (traj.npz) -> TUM file.

Each line: t x y z qx qy qz qw
  t      traj t_ns = camera header stamp + camera_offset (0.004 s), i.e. the
         time the filter state refers to, on the /voxl/raw_imu header clock
         (VOXL clock). Written from the int64 ns exactly (no float rounding).
  x..qw  T_wb from VIOFilter.get_pose(): the IMU/body (/voxl/raw_imu frame)
         pose in ECHO-LI's world frame, quaternion x y z w (as_xyzw()).

    python traj_to_tum.py traj.npz out.tum
"""
import sys

import numpy as np


def main():
    src, dst = sys.argv[1], sys.argv[2]
    d = np.load(src)
    t, p, q = d["t_ns"].astype(np.int64), d["p"], d["q"]
    if len(t) and np.any(np.diff(t) <= 0):
        raise SystemExit("non-increasing timestamps in " + src)
    with open(dst, "w") as f:
        f.write("# t[s, VOXL /voxl/raw_imu header clock] x y z qx qy qz qw"
                " (ECHO-LI T_world_imu)\n")
        for ti, pi, qi in zip(t.tolist(), p, q):
            f.write("%d.%09d %.6f %.6f %.6f %.9f %.9f %.9f %.9f\n"
                    % (ti // 1_000_000_000, ti % 1_000_000_000, *pi, *qi))
    print("%s: %d poses, %.3f .. %.3f s" % (dst, len(t), t[0] / 1e9, t[-1] / 1e9))


if __name__ == "__main__":
    main()
