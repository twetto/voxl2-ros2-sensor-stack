#!/usr/bin/env python3
"""Sanity check of an OpenVINS TUM trajectory against /vrpn_mocap/drone_01/pose.

  - mocap: VRPN re-sends every pose. In the 0914 bags the copies are interleaved with
    other poses (only 9-22 % are consecutive) and carry their own, later header stamps,
    so only the earliest-stamped copy of each bit-identical pose is kept (this also drops
    dropout freezes). Samples falling in a gap > 60 ms between kept poses are not scored.
  - clock: mocap_stamp = voxl_stamp + offset. Starts from the rough per-session offset,
    refined by cross-correlating |gyro| (/voxl/raw_imu) with the mocap |body rate|
    (rotation-invariant, so the mocap-body<->IMU rotation is not needed); the refined value
    is used only if its correlation peak is > 0.5 and within +-0.3 s of the rough one.
  - SE(3) Umeyama alignment of OpenVINS positions (IMU) onto mocap positions (rigid body,
    no lever arm), ATE RMSE; Sim(3) scale reported for information. Path lengths over the
    scored span. Diagnostics: takeoff/landing from mocap height, max error per 10 s,
    largest pose step, and velocity/biases from <tum>.state if present.

Run in the ubuntu-22-04 distrobox with ROS 2 Humble sourced (rclpy for CDR):
  python eval_mocap.py <bag_dir> <traj.tum> <rough_offset_s>   -> one JSON line
"""
import glob
import json
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read(con, topic):
    tid, typ = con.execute("select id, type from topics where name=?", (topic,)).fetchone()
    cls = get_message(typ)
    return [deserialize_message(d, cls) for (d,) in
            con.execute("select data from messages where topic_id=? order by timestamp", (tid,))]


def st(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def qmul(a, b):
    x1, y1, z1, w1 = a.T
    x2, y2, z2, w2 = b.T
    return np.stack([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2], 1)


def rate_mag(t, q):
    """|body rate| from consecutive quaternions (x y z w)."""
    qc = q * np.array([-1, -1, -1, 1])
    d = qmul(qc[:-1], q[1:])
    d *= np.sign(d[:, 3:4] + 1e-12)
    ang = 2 * np.arctan2(np.linalg.norm(d[:, :3], axis=1), d[:, 3])
    return 0.5 * (t[:-1] + t[1:]), ang / np.diff(t)


def umeyama(src, dst, scale=False):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(xd.T @ xs / len(src))
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(U @ Vt))
    R = U @ D @ Vt
    s = np.trace(np.diag(S) @ D) / (xs ** 2).sum(1).mean() if scale else 1.0
    return s, R, mu_d - s * R @ mu_s


def load_mocap(con):
    mm = read(con, "/vrpn_mocap/drone_01/pose")
    tm = np.array([st(m) for m in mm])
    P = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z, m.pose.orientation.x,
                   m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w] for m in mm])
    n_raw = len(tm)
    n_consec = int(np.all(P[1:] == P[:-1], axis=1).sum())
    o = np.argsort(tm, kind="stable")
    tm, P = tm[o], P[o]
    first, keep, lag = {}, np.zeros(n_raw, bool), []
    for i in range(n_raw):
        key = P[i].tobytes()
        j = first.get(key)
        if j is None:
            first[key] = i
            keep[i] = True
        else:
            lag.append(tm[i] - tm[j])
    tm, P = tm[keep], P[keep]
    good = np.r_[True, np.diff(tm) > 0]
    tm, P = tm[good], P[good]
    lag = np.array(lag) if lag else np.zeros(1)
    stats = dict(mocap_n_raw=n_raw, mocap_n_consecutive_repeats=n_consec, mocap_n_kept=len(tm),
                 mocap_resend_lag_ms_median=float(np.median(lag) * 1e3),
                 mocap_resend_lag_ms_p99=float(np.percentile(lag, 99) * 1e3))
    return tm, P, stats


def main():
    bag, tum, rough = sys.argv[1], sys.argv[2], float(sys.argv[3])
    con = sqlite3.connect("file:%s?immutable=1" % glob.glob(bag + "/*.db3")[0], uri=True)
    tm, P, mstats = load_mocap(con)

    imu = read(con, "/voxl/raw_imu")
    ti = np.array([st(m) for m in imu])
    wi = np.linalg.norm([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for m in imu], axis=1)

    # |omega| cross-correlation on a 100 Hz grid (lightly smoothed), lag search +-0.5 s
    tr, wr = rate_mag(tm, P[:, 3:])
    ok = np.diff(tm) < 0.02
    tr, wr = tr[ok], wr[ok]
    g = np.arange(ti[0] + 0.5, ti[-1] - 0.5, 0.01)
    k = np.ones(5) / 5
    wi_g = np.convolve(np.interp(g, ti, wi), k, "same")
    best = (-2, rough)
    for lag in np.arange(-0.5, 0.5001, 0.002):
        tq = g + rough + lag
        inside = (tq > tr[0]) & (tq < tr[-1])
        if inside.sum() < 300:
            continue
        wm_g = np.convolve(np.interp(tq, tr, wr), k, "same")
        c = np.corrcoef(wi_g[inside], wm_g[inside])[0, 1]
        if c > best[0]:
            best = (c, rough + lag)
    xc, off_fine = best
    use_fine = xc > 0.5 and abs(off_fine - rough) < 0.3
    offset = off_fine if use_fine else rough

    E = np.loadtxt(tum)
    te, pe = E[:, 0], E[:, 1:4]
    tq = te + offset
    j = np.searchsorted(tm, tq)
    valid = (j > 0) & (j < len(tm))
    jj = np.clip(j, 1, len(tm) - 1)
    valid &= (tm[jj] - tm[jj - 1]) < 0.06
    pm = np.column_stack([np.interp(tq, tm, P[:, c]) for c in range(3)])
    a, b = pe[valid], pm[valid]
    _, R, t = umeyama(a, b)
    ea = (R @ a.T).T + t - b
    err = np.linalg.norm(ea, axis=1)
    s_sim, R2, t2 = umeyama(a, b, scale=True)
    err_sim = np.linalg.norm(s_sim * (R2 @ a.T).T + t2 - b, axis=1)
    out = dict(
        tum=tum, n_poses=len(te), n_scored=int(valid.sum()),
        t_first=float(te[0]), t_last=float(te[-1]), span_s=float(te[-1] - te[0]),
        offset_rough=rough, offset_xcorr=float(off_fine), xcorr_peak=float(xc), offset_used=float(offset),
        ate_se3_rmse=float(np.sqrt(np.mean(err ** 2))), ate_se3_max=float(err.max()),
        ate_sim3_rmse=float(np.sqrt(np.mean(err_sim ** 2))), sim3_scale=float(s_sim),
        path_ov=float(np.linalg.norm(np.diff(a, axis=0), axis=1).sum()),
        path_mocap=float(np.linalg.norm(np.diff(b, axis=0), axis=1).sum()),
        ov_extent=float(np.linalg.norm(pe - pe[0], axis=1).max()),
        err_end=float(err[-1]),
        err_h_rmse=float(np.sqrt(np.mean(ea[:, 0] ** 2 + ea[:, 2] ** 2))),   # mocap world is Y-up
        err_v_rmse=float(np.sqrt(np.mean(ea[:, 1] ** 2))))
    out.update(mstats)

    h = P[:, 1]
    h0 = np.median(h[:50])
    air = np.where(h > h0 + 0.10)[0]
    out["t_takeoff_rel"] = float(tm[air[0]] - offset - te[0]) if len(air) else None
    out["t_land_rel"] = float(tm[air[-1]] - offset - te[0]) if len(air) else None
    tvv = te[valid]
    out["err_max_per_10s"] = [round(float(err[(tvv >= s0) & (tvv < s0 + 10.0)].max()), 2)
                              for s0 in np.arange(tvv[0], tvv[-1], 10.0)
                              if ((tvv >= s0) & (tvv < s0 + 10.0)).any()]
    out["max_pose_step_m"] = float(np.linalg.norm(np.diff(pe, axis=0), axis=1).max())
    try:
        S = np.loadtxt(tum + ".state")
        v = np.linalg.norm(S[:, 8:11], axis=1)
        out.update(v_max=float(v.max()), v_end=float(v[-1]),
                   bg_end=[round(float(x), 4) for x in S[-1, 11:14]],
                   ba_end=[round(float(x), 3) for x in S[-1, 14:17]],
                   n_slam_min=int(S[:, 17].min()), n_slam_med=float(np.median(S[:, 17])))
    except OSError:
        pass
    print(json.dumps(out))


if __name__ == "__main__":
    main()
