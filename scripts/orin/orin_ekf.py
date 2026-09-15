"""Error-state EKF for the Orin: VOXL IMU + a body-frame velocity source + the
downward rangefinder -> pose, to send to the FC as vision_pose the way mocap and
qvio are, so the FC's own EKF needs no new input.

  nominal   p, v      position and velocity, world frame, z up
            R         attitude, world <- body (FLU)
            b_a, b_g  accelerometer and gyro biases
            s         optional scale of the velocity source, forward and left
  error     dp, dv, dth, db_a, db_g, ds: 17 states, dth on the right
            (R_true = R Exp(dth))
Prediction runs on the IMU in body FLU (/voxl/raw_imu is FRD: negate y and z).
Updates:
  velocity  the source's body-frame horizontal velocity (forward, left) at the
            body origin: h = s * (R^T v)[:2]
  range     downward rangefinder over a flat floor: h = p_z / R_zz
  ground    zero velocity and zero rate while standing on the ground. The
            zero-rate update is what pins the yaw gyro bias.
Yaw has no measurement: after the ground update it drifts with the residual
gyro bias; on the 2026-09-14 test flights it drifted 4-25 deg per flight.
Measurements must come in time order; the caller buffers them.

The scale states suit a source whose gain drifts; monocular flow, for example,
reads v/Z. They are observable only while the drone accelerates, and a
multicopter's horizontal acceleration reaches the IMU only through tilt, so on
gentle flights they barely help. They're off by default.
test_orin_ekf.py checks the filter on a simulated flight with known truth.
"""
import collections

import numpy as np

GRAVITY = 9.80665
N = 17
P_, V_, TH, BA, BG, S_ = (slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15),
                          slice(15, 17))
E_Z = np.array([0.0, 0.0, 1.0])

DEFAULTS = dict(
    acc_noise=0.3,          # m/s^2/sqrt(Hz), vibration included; 0.1 diverged on test flights
    gyro_noise=0.01,        # rad/s/sqrt(Hz)
    acc_bias_walk=0.02,     # m/s^3/sqrt(Hz)
    gyro_bias_walk=5e-4,    # rad/s^2/sqrt(Hz)
    estimate_scale=False,
    scale_init_sigma=0.3,
    scale_walk=0.02,        # 1/sqrt(s)
    vel_sigma=0.25,         # m/s per axis, the velocity source
    vel_gate=13.8,          # chi^2, 2 dof (99.9 %)
    # let velocity updates turn yaw and the yaw gyro bias. A source that reads each
    # body axis at a different scale gets the direction wrong when the drone moves
    # diagonally, and those updates then rotate yaw; locking yaw out made it worse
    # on test flights, so it stays on
    velocity_yaw=True,
    range_sigma=0.03,       # m
    range_gate=10.8,        # chi^2, 1 dof (99.9 %)
    zero_vel_sigma=0.03,    # m/s
    zero_rate_sigma=0.01,   # rad/s
    yaw_leak=0.0,           # m/s of left velocity the source reads per rad/s of yaw rate
)


def skew(w):
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def exp_so3(phi):
    a = np.linalg.norm(phi)
    if a < 1e-9:
        return np.eye(3) + skew(phi)
    k = skew(phi / a)
    return np.eye(3) + np.sin(a) * k + (1.0 - np.cos(a)) * (k @ k)


def rot_zyx(yaw, pitch, roll):
    cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def yaw_of(R):
    return float(np.arctan2(R[1, 0], R[0, 0]))


class OrinEKF(object):
    def __init__(self, **params):
        unknown = set(params) - set(DEFAULTS)
        if unknown:
            raise ValueError("unknown EKF parameters %s" % sorted(unknown))
        self.c = dict(DEFAULTS, **params)
        self.t = None
        self.p, self.v = np.zeros(3), np.zeros(3)
        self.R = np.eye(3)
        self.ba, self.bg, self.s = np.zeros(3), np.zeros(3), np.ones(2)
        self.P = np.zeros((N, N))
        self.n = collections.Counter()

    def initialise(self, t, acc, gyro, yaw=0.0, height=0.0):
        """At rest: level from the mean specific force, gyro bias from the mean rate."""
        f = np.asarray(acc, float)
        self.R = rot_zyx(yaw, np.arctan2(-f[0], np.hypot(f[1], f[2])), np.arctan2(f[1], f[2]))
        self.p, self.v = np.array([0.0, 0.0, height]), np.zeros(3)
        self.ba, self.bg, self.s = np.zeros(3), np.asarray(gyro, float).copy(), np.ones(2)
        sd = np.r_[[0.01] * 3, [0.05] * 3, 0.02, 0.02, 0.05, [0.2] * 3, [0.005] * 3,
                   [self.c["scale_init_sigma"] if self.c["estimate_scale"] else 0.0] * 2]
        self.P = np.diag(sd ** 2)
        self.t = t

    def predict(self, t, acc, gyro):
        """Propagate to t with the IMU sample (mean over the step), body FLU."""
        dt = t - self.t
        if dt <= 0.0:
            return
        a_b = np.asarray(acc, float) - self.ba
        w_b = np.asarray(gyro, float) - self.bg
        R0 = self.R
        a_w = R0 @ a_b - GRAVITY * E_Z
        self.p = self.p + self.v * dt + 0.5 * a_w * dt * dt
        self.v = self.v + a_w * dt
        self.R = R0 @ exp_so3(w_b * dt)
        F = np.eye(N)
        F[P_, V_] = np.eye(3) * dt
        F[V_, TH] = -R0 @ skew(a_b) * dt
        F[V_, BA] = -R0 * dt
        F[TH, TH] = exp_so3(-w_b * dt)
        F[TH, BG] = -np.eye(3) * dt
        c = self.c
        q = np.zeros(N)
        q[V_] = c["acc_noise"] ** 2 * dt
        q[TH] = c["gyro_noise"] ** 2 * dt
        q[BA] = c["acc_bias_walk"] ** 2 * dt
        q[BG] = c["gyro_bias_walk"] ** 2 * dt
        if c["estimate_scale"]:
            q[S_] = c["scale_walk"] ** 2 * dt
        self.P = F @ self.P @ F.T + np.diag(q)
        self.t = t
        self.n["predict"] += 1
        if self.n["predict"] % 1000 == 0:           # keep R orthonormal
            u, _, vt = np.linalg.svd(self.R)
            self.R = u @ vt

    def _update(self, y, H, Rn, gate, name, lock_yaw=False):
        S = H @ self.P @ H.T + Rn
        Si = np.linalg.inv(S)
        if gate is not None and float(y @ Si @ y) > gate:
            self.n[name + "_rejected"] += 1
            return False
        K = self.P @ H.T @ Si
        if lock_yaw:
            # no correction about world up, to the attitude or the gyro bias; the
            # Joseph form below stays valid for this suboptimal gain
            u = self.R.T @ E_Z
            for blk in (TH, BG):
                K[blk] -= np.outer(u, u @ K[blk])
        dx = K @ y
        IKH = np.eye(N) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ Rn @ K.T
        self.p = self.p + dx[P_]
        self.v = self.v + dx[V_]
        self.R = self.R @ exp_so3(dx[TH])
        self.ba = self.ba + dx[BA]
        self.bg = self.bg + dx[BG]
        self.s = self.s + dx[S_]
        self.n[name] += 1
        return True

    def update_velocity(self, v_fwd_left, yaw_rate=0.0):
        """Body-frame velocity from the source, forward and left (m/s), at the body origin."""
        z = np.asarray(v_fwd_left, float).copy()
        z[1] -= self.c["yaw_leak"] * yaw_rate
        vb = self.R.T @ self.v
        Sd = np.diag(self.s)
        H = np.zeros((2, N))
        H[:, V_] = Sd @ self.R.T[:2]
        H[:, TH] = Sd @ skew(vb)[:2]
        H[:, S_] = np.diag(vb[:2])
        return self._update(z - self.s * vb[:2], H, np.eye(2) * self.c["vel_sigma"] ** 2,
                            self.c["vel_gate"], "velocity", lock_yaw=not self.c["velocity_yaw"])

    def update_range(self, r):
        """Downward rangefinder distance (m) to a flat floor at z = 0."""
        cz = self.R[2, 2]
        if cz < 0.5:                                # tilted over 60 deg: no floor model
            return False
        H = np.zeros((1, N))
        H[0, 2] = 1.0 / cz
        H[0, TH] = self.p[2] / cz ** 2 * (self.R[2] @ skew(E_Z))
        return self._update(np.array([r - self.p[2] / cz]), H,
                            np.eye(1) * self.c["range_sigma"] ** 2, self.c["range_gate"], "range")

    def update_ground(self, gyro_mean):
        """Standing on the ground: zero velocity, and the mean rate is the gyro bias."""
        H = np.zeros((3, N))
        H[:, V_] = np.eye(3)
        self._update(-self.v, H, np.eye(3) * self.c["zero_vel_sigma"] ** 2, None, "zero_velocity")
        H = np.zeros((3, N))
        H[:, BG] = np.eye(3)
        self._update(np.asarray(gyro_mean, float) - self.bg, H,
                     np.eye(3) * self.c["zero_rate_sigma"] ** 2, None, "zero_rate")
