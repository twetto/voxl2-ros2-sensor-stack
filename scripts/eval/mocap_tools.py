"""Motion-capture ground truth for VOXL + VRPN bags, without ROS.

  read_topic   one topic's messages and receive times, via `rosbags`
  mocap_track  VRPN sends every pose twice, interleaved with the next ones and
               restamped ~3 ms later: keep the first copy of each identical pose,
               order by stamp, make the quaternions continuous
  sample       position and attitude at query times
  body_rates   body angular velocity from consecutive attitudes
  align        the VOXL clock onto mocap's: a coarse offset from receive times
               (coarse_offset), refined by cross-correlating the gyro with the
               mocap body rates axis by axis. Signed axes stay sharp on smooth
               flights, where |w| alone correlates at ~0.4.
The rigid body is FUR (x forward, y up, z right) in a Y-up world, as the lab's
Motive setup streams it; /voxl/raw_imu is FRD. The VOXL clock can be days off,
and jumps at every VOXL reboot, so align each bag on its own.
Needs numpy and scipy, plus rosbags for read_topic.
"""
import numpy as np
from scipy.spatial.transform import Rotation

FLIP = np.array([1.0, -1.0, -1.0])                  # /voxl/raw_imu (FRD) -> FLU


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def read_topic(bag, topic):
    """(messages, receive times in s) of one topic of a ROS 2 bag."""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    with Reader(bag) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        rows = [(t_ns * 1e-9, ts.deserialize_cdr(raw, c.msgtype))
                for c, t_ns, raw in reader.messages(connections=conns)]
    return [m for _, m in rows], np.array([t for t, _ in rows])


def mocap_track(msgs):
    """PoseStamped messages -> (t, positions, continuous quaternions xyzw)."""
    t = np.array([stamp(m) for m in msgs])
    X = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z, m.pose.orientation.x,
                   m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w] for m in msgs])
    _, first = np.unique(X, axis=0, return_index=True)
    first.sort()
    t, X = t[first], X[first]
    order = np.argsort(t, kind="stable")
    t, X = t[order], X[order]
    k = [0]
    for i in range(1, len(t)):
        if t[i] - t[k[-1]] >= 5e-4:
            k.append(i)
    t, P, Q = t[k], X[k, :3], X[k, 3:]
    s = np.sign(np.sum(Q[1:] * Q[:-1], axis=1))
    s[s == 0] = 1.0
    Q[1:] *= np.cumprod(s)[:, None]
    return t, P, Q


def sample(t, P, Q, tq):
    p = np.column_stack([np.interp(tq, t, P[:, c]) for c in range(3)])
    q = np.column_stack([np.interp(tq, t, Q[:, c]) for c in range(4)])
    return p, q / np.linalg.norm(q, axis=1, keepdims=True)


def body_rates(q, tq):
    """Body angular velocity from consecutive attitudes."""
    R = Rotation.from_quat(q)
    rel = (R[:-1].inv() * R[1:]).as_rotvec() / np.diff(tq)[:, None]
    w = np.zeros((len(tq), 3))
    w[:-1] += rel
    w[1:] += rel
    w[1:-1] *= 0.5
    return w


def smooth(x, n):
    return np.convolve(x, np.ones(n) / n, mode="same")


def coarse_offset(imu_t, imu_rx, mocap_t, mocap_rx):
    """mocap clock - VOXL clock from receive times: the IMU's least-delay sample
    for the VOXL side, the median delay for mocap."""
    return -np.percentile(imu_t - imu_rx, 99) + np.median(mocap_t - mocap_rx)


def align(imu_t, imu_w, mt, mP, mQ, coarse):
    """(offset, correlation) with mocap time = VOXL time + offset, refined over
    +-0.3 s. imu_w is the /voxl/raw_imu gyro as recorded (FRD)."""
    grid = np.arange(max(imu_t[0] + coarse, mt[0]) + 0.5, min(imu_t[-1] + coarse, mt[-1]) - 0.5, 0.005)
    _, q = sample(mt, mP, mQ, grid)
    wm = body_rates(q, grid)
    wm = np.column_stack([smooth(wm[:, k], 10) for k in range(3)])
    gf = imu_w * FLIP
    g_fur = np.column_stack([gf[:, 0], gf[:, 2], -gf[:, 1]])
    best = (coarse, -2.0)
    for d in np.arange(-0.3, 0.3, 0.0025):
        tv = grid - coarse - d
        c = np.mean([np.corrcoef(smooth(np.interp(tv, imu_t, g_fur[:, k]), 10), wm[:, k])[0, 1]
                     for k in range(3)])
        if c > best[1]:
            best = (coarse + d, c)
    return best
