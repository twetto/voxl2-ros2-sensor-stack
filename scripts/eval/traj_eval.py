"""Trajectory scoring against mocap, shared by every estimator compared here.

Mocap's world is Y up; mocap_zup turns it into a z-up world (x, y, z) =
(X, -Z, Y), so estimators with gravity-aligned z-up frames line up without
flips. score() SE(3)-aligns an estimate to mocap over a common window (the
usual ATE: rotation and translation, no scale) and reports the horizontal
error, which is what the FC would fly on.
"""
import numpy as np


def mocap_zup(P):
    return np.column_stack([P[:, 0], -P[:, 2], P[:, 1]])


def umeyama(src, dst):
    """R, t minimising |R src + t - dst| (rotation and translation, no scale)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - mu_s).T @ (dst - mu_d))
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    return R, mu_d - R @ mu_s


def score(t_est, p_est, t_ref, p_ref, window=None):
    """Estimate positions at their own times inside window (reference clock), aligned
    to the reference interpolated at those times."""
    m = np.all(np.isfinite(p_est), axis=1) & (t_est >= t_ref[0]) & (t_est <= t_ref[-1])
    if window is not None:
        m &= (t_est >= window[0]) & (t_est <= window[1])
    te, pe = t_est[m], p_est[m]
    pr = np.column_stack([np.interp(te, t_ref, p_ref[:, k]) for k in range(3)])
    R, t = umeyama(pe, pr)
    pa = pe @ R.T + t
    eh = np.linalg.norm((pa - pr)[:, :2], axis=1)
    return dict(t=te, aligned=pa, ref=pr, R=R, trans=t,
                ate=float(np.sqrt(np.mean(np.sum((pa - pr) ** 2, axis=1)))),
                ate_h=float(np.sqrt(np.mean(eh ** 2))), end_h=float(eh[-1]), max_h=float(eh.max()),
                path=float(np.sum(np.linalg.norm(np.diff(pr[:, :2], axis=0), axis=1))))
