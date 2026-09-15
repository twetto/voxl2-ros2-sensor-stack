#!/usr/bin/env python3
"""Synthetic check of orin_ekf: a simulated flight with known truth.

A drone stands still for 10 s, climbs to 1.5 m, then wanders for 70 s while
turning back and forth, with body tilt set by the acceleration it needs. From
that truth:
  IMU       specific force and rate, body FLU, with noise (and bias), 200 Hz
  velocity  a velocity source reading the body velocity at scale 0.9 forward and
            0.4 left (as a monocular-flow source does at some headings), with
            noise, 28 Hz
  range     p_z / R_zz with noise, 2 Hz
Checks:
  default filter (IMU biases on): tracks as well as a naive integration of the
    same velocities with the true attitude, holds yaw, finds the gyro bias
  scale states (ideal IMU: no bias, filter noise = true noise): the maths is
    right if they cut the error and find the left scale
Usage: python3 test_orin_ekf.py
"""
import numpy as np

from orin_ekf import GRAVITY, OrinEKF, yaw_of

DT, T_END, T_GROUND = 0.005, 90.0, 10.0
SCALE = np.array([0.9, 0.4])
ACC_BIAS, GYRO_BIAS = np.array([0.08, -0.05, 0.1]), np.array([0.004, -0.003, 0.002])


def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def truth(t):
    """Position and yaw at the times t."""
    air = smoothstep((t - T_GROUND) / 5.0) * (1.0 - smoothstep((t - 85.0) / 4.0))
    fly = np.clip(t - T_GROUND, 0.0, None)
    p = np.column_stack([air * (1.5 * np.sin(0.21 * fly) + 0.6 * np.sin(0.53 * fly)),
                         air * (1.2 * np.sin(0.17 * fly + 0.5) - 0.5 * np.sin(0.47 * fly)),
                         1.5 * air])
    yaw = 0.3 + 1.2 * np.sin(0.12 * fly) * air
    return p, yaw


def simulate(acc_bias, gyro_bias, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, T_END, DT)
    p, yaw = truth(t)
    v = np.gradient(p, DT, axis=0)
    a = np.gradient(v, DT, axis=0)
    R = np.empty((len(t), 3, 3))
    for i in range(len(t)):
        z = a[i] + GRAVITY * np.array([0.0, 0.0, 1.0])
        z /= np.linalg.norm(z)
        y = np.cross(z, [np.cos(yaw[i]), np.sin(yaw[i]), 0.0])
        y /= np.linalg.norm(y)
        R[i] = np.column_stack([np.cross(y, z), y, z])
    f = np.einsum("nji,nj->ni", R, a + GRAVITY * np.array([0.0, 0.0, 1.0]))
    w = np.zeros((len(t), 3))
    for i in range(len(t) - 1):                     # rate over [t_i, t_i+1]
        d = R[i].T @ R[i + 1]
        w[i] = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]]) / (2.0 * DT)
    w[-1] = w[-2]
    imu_f = f + acc_bias + rng.normal(0.0, 0.06 / np.sqrt(DT), f.shape)
    imu_w = w + gyro_bias + rng.normal(0.0, 0.005 / np.sqrt(DT), w.shape)
    vb = np.einsum("nji,nj->ni", R, v)
    return dict(t=t, p=p, R=R, yaw=yaw, imu_f=imu_f, imu_w=imu_w, vb=vb)


def run(sim, **params):
    t, rng = sim["t"], np.random.default_rng(1)
    ekf = OrinEKF(**params)
    k0 = int(1.0 / DT)
    ekf.initialise(t[k0], sim["imu_f"][:k0].mean(0), sim["imu_w"][:k0].mean(0), yaw=sim["yaw"][0])
    vel_every, range_every = int(round(1.0 / 28 / DT)), int(round(0.5 / DT))
    est, yaw_est = np.zeros((len(t), 3)), np.zeros(len(t))
    for i in range(k0 + 1, len(t)):
        ekf.predict(t[i], sim["imu_f"][i], sim["imu_w"][i - 1])
        if i % range_every == 0:
            rz = sim["p"][i, 2] / sim["R"][i][2, 2] + rng.normal(0.0, 0.02)
            ekf.update_range(rz)
            if rz < 0.1:
                ekf.update_ground(sim["imu_w"][i - range_every:i].mean(0))
        if i % vel_every == 0 and sim["p"][i, 2] > 0.1:
            ekf.update_velocity(SCALE * sim["vb"][i, :2] + rng.normal(0.0, 0.1, 2))
        est[i], yaw_est[i] = ekf.p, yaw_of(ekf.R)
    k = k0 + 1
    e = np.linalg.norm(est[k:, :2] - sim["p"][k:, :2], axis=1)
    dyaw = np.degrees(np.angle(np.exp(1j * (yaw_est[k:] - sim["yaw"][k:]))))
    return dict(rms=float(np.sqrt(np.mean(e ** 2))), end=float(e[-1]), yaw=float(np.abs(dyaw).max()), ekf=ekf)


def naive_rms(sim):
    """The scaled source velocity, rotated with the true attitude and integrated."""
    vb = sim["vb"].copy()
    vb[:, :2] *= SCALE
    track = np.cumsum(np.einsum("nij,nj->ni", sim["R"], vb) * DT, axis=0)
    return float(np.sqrt(np.mean(np.sum((track[:, :2] - sim["p"][:, :2]) ** 2, axis=1))))


def main():
    sim = simulate(ACC_BIAS, GYRO_BIAS)
    base = run(sim)
    ideal = simulate(np.zeros(3), np.zeros(3))
    sc = run(ideal, estimate_scale=True, acc_noise=0.06)
    fixed = run(ideal, estimate_scale=False, acc_noise=0.06)
    print("synthetic flight %.0f s, source scale forward %.2f left %.2f" % (T_END, *SCALE))
    print("  naive integration (true attitude)  rms %.2f m" % naive_rms(sim))
    print("  EKF default, biased IMU            rms %.2f m, end %.2f m, yaw error max %.1f deg, gyro bias error %.4f rad/s"
          % (base["rms"], base["end"], base["yaw"], np.abs(base["ekf"].bg - GYRO_BIAS).max()))
    print("  ideal IMU: scale states            rms %.2f m, scale %.2f %.2f" % (sc["rms"], *sc["ekf"].s))
    print("  ideal IMU: fixed scale             rms %.2f m" % fixed["rms"])
    checks = [("default tracks like naive integration (<= 1.1x)", base["rms"] <= 1.1 * naive_rms(sim)),
              # a mis-scaled source pulls yaw (8.8 deg here)
              ("default yaw error under 10 deg", base["yaw"] < 10.0),
              ("default gyro bias within 0.005 rad/s", np.abs(base["ekf"].bg - GYRO_BIAS).max() < 0.005),
              ("ideal IMU: scale states cut the error (< 0.8x)", sc["rms"] < 0.8 * fixed["rms"]),
              ("ideal IMU: left scale within 0.05", abs(sc["ekf"].s[1] - SCALE[1]) < 0.05)]
    ok = True
    for label, passed in checks:
        print("  %-50s %s" % (label, "PASS" if passed else "FAIL"))
        ok &= bool(passed)
    print("ALL PASS" if ok else "SOME FAILED")


if __name__ == "__main__":
    main()
