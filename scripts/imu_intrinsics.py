#!/usr/bin/env python3
"""Extract IMU intrinsics from a static ROS2 bag via Allan variance.

Usage:
    python imu_intrinsics.py <bag_dir> [--topic /voxl/raw_imu] [--save allan.npz]

Outputs per-axis and aggregate noise parameters in SI units (rad, m, s)
suitable for EqVIO (velocityNoise) and OpenVINS (kalibr_imu_chain.yaml).

Requires: pip install rosbags numpy
"""

import argparse
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

GRAVITY = 9.80665  # m/s²


def allan_variance(rate_data, dt, n_points=200):
    """Overlapping Allan variance on rate data.

    Uses the phase (cumulative sum) formulation:
      AVAR(τ) = 1/(2τ²(N-2m)) Σ (θ_{k+2m} - 2θ_{k+m} + θ_k)²
    where θ = cumsum(rate * dt), τ = m*dt, m = cluster size.
    """
    N = len(rate_data)
    data = rate_data - np.mean(rate_data)
    theta = np.cumsum(data) * dt

    max_m = N // 4
    ms = np.unique(np.logspace(0, np.log10(max_m), n_points).astype(int))
    ms = ms[ms >= 1]

    taus = ms * dt
    adevs = np.empty(len(ms))

    for i, m in enumerate(ms):
        d = theta[2*m:] - 2*theta[m:-m] + theta[:-(2*m)]
        tau = m * dt
        adevs[i] = np.sqrt(np.mean(d**2) / (2.0 * tau**2))

    return taus, adevs


def main():
    parser = argparse.ArgumentParser(
        description="Extract IMU intrinsics from a static ROS2 bag via "
                    "overlapping Allan variance.")
    parser.add_argument("bag", type=Path,
                        help="Path to the ROS2 bag directory")
    parser.add_argument("--topic", default="/voxl/raw_imu",
                        help="IMU topic name (default: /voxl/raw_imu)")
    parser.add_argument("--save", type=Path, default=None,
                        help="Save Allan data to .npz file")
    args = parser.parse_args()

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    print(f"Reading {args.bag} ...")
    stamps, gyro, accel = [], [], []

    with AnyReader([args.bag], default_typestore=typestore) as reader:
        for conn, timestamp, rawdata in reader.messages():
            if conn.topic != args.topic:
                continue
            msg = reader.deserialize(rawdata, conn.msgtype)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            stamps.append(t)
            gyro.append([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])
            accel.append([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])

    stamps = np.array(stamps)
    gyro = np.array(gyro)
    accel = np.array(accel)

    N = len(stamps)
    dt = np.mean(np.diff(stamps))
    rate = 1.0 / dt
    duration = stamps[-1] - stamps[0]

    print(f"\nSamples: {N:,}")
    print(f"Duration: {duration:.1f}s ({duration/3600:.2f}h)")
    print(f"Mean rate: {rate:.1f} Hz")
    print(f"Mean dt: {dt*1e3:.3f} ms")

    # ── Bias (mean) ──
    gyro_bias = np.mean(gyro, axis=0)
    accel_bias_raw = np.mean(accel, axis=0)
    accel_norm = np.linalg.norm(accel_bias_raw)
    gravity_dir = accel_bias_raw / accel_norm
    accel_bias = accel_bias_raw - gravity_dir * GRAVITY

    print(f"\n{'='*60}")
    print("BIASES (mean of static data)")
    print(f"{'='*60}")
    print(f"Gyro bias [rad/s]:  {gyro_bias}")
    print(f"Gyro bias [deg/s]:  {np.degrees(gyro_bias)}")
    print(f"Accel raw mean:     {accel_bias_raw}")
    print(f"Accel norm:         {accel_norm:.5f} m/s² (expect {GRAVITY:.5f})")
    print(f"Accel bias [m/s²]:  {accel_bias}")

    # ── White noise (std dev) ──
    gyro_std = np.std(gyro, axis=0)
    accel_std = np.std(accel, axis=0)
    gyro_noise_density = gyro_std / np.sqrt(rate)
    accel_noise_density = accel_std / np.sqrt(rate)

    print(f"\n{'='*60}")
    print("WHITE NOISE (std dev of static data)")
    print(f"{'='*60}")
    print(f"Gyro σ [rad/s]:         {gyro_std}")
    print(f"Gyro σ [deg/s]:         {np.degrees(gyro_std)}")
    print(f"Gyro noise density:     {gyro_noise_density}  [rad/s/√Hz]")
    print(f"Accel σ [m/s²]:         {accel_std}")
    print(f"Accel noise density:    {accel_noise_density}  [m/s²/√Hz]")

    # ── Allan variance ──
    print(f"\n{'='*60}")
    print("ALLAN DEVIATION ANALYSIS")
    print(f"{'='*60}")
    print(f"Computing (overlapping Allan variance on {N:,} samples)...")

    axes = ['x', 'y', 'z']
    gyro_arw = np.empty(3)      # angle random walk  [rad/s/√Hz]
    gyro_bi = np.empty(3)       # bias instability   [rad/s]
    gyro_bi_tau = np.empty(3)
    gyro_rrw = np.empty(3)      # rate random walk   [rad/s²/√Hz]
    accel_vrw = np.empty(3)     # velocity random walk [m/s²/√Hz]
    accel_bi = np.empty(3)      # bias instability   [m/s²]
    accel_bi_tau = np.empty(3)
    accel_arw = np.empty(3)     # accel random walk  [m/s³/√Hz]

    for ax in range(3):
        print(f"  Computing gyro {axes[ax]}...")
        taus_g, adevs_g = allan_variance(gyro[:, ax], dt)
        print(f"  Computing accel {axes[ax]}...")
        taus_a, adevs_a = allan_variance(accel[:, ax], dt)

        # ARW: N = σ_AD(τ) * √τ in the -1/2 slope region (0.5–10s)
        mask = (taus_g >= 0.5) & (taus_g <= 10.0)
        if np.any(mask):
            gyro_arw[ax] = np.median(adevs_g[mask] * np.sqrt(taus_g[mask]))
        else:
            idx = np.argmin(np.abs(taus_g - 1.0))
            gyro_arw[ax] = adevs_g[idx]

        mask = (taus_a >= 0.5) & (taus_a <= 10.0)
        if np.any(mask):
            accel_vrw[ax] = np.median(adevs_a[mask] * np.sqrt(taus_a[mask]))
        else:
            idx = np.argmin(np.abs(taus_a - 1.0))
            accel_vrw[ax] = adevs_a[idx]

        # Bias instability: min of Allan deviation / 0.6642
        imin = np.argmin(adevs_g)
        gyro_bi[ax] = adevs_g[imin] / 0.6642
        gyro_bi_tau[ax] = taus_g[imin]

        imin = np.argmin(adevs_a)
        accel_bi[ax] = adevs_a[imin] / 0.6642
        accel_bi_tau[ax] = taus_a[imin]

        # Rate random walk: K = σ_AD * √3 / √τ from +1/2 slope tail
        mask = taus_g > taus_g[np.argmin(adevs_g)] * 3
        gyro_rrw[ax] = (np.median(adevs_g[mask] * np.sqrt(3)
                                   / np.sqrt(taus_g[mask]))
                         if np.any(mask) else 0.0)

        mask = taus_a > taus_a[np.argmin(adevs_a)] * 3
        accel_arw[ax] = (np.median(adevs_a[mask] * np.sqrt(3)
                                    / np.sqrt(taus_a[mask]))
                          if np.any(mask) else 0.0)

        print(f"    Gyro  {axes[ax]}: ARW={gyro_arw[ax]:.4e} rad/s/√Hz  "
              f"BI={gyro_bi[ax]:.4e} rad/s (τ={gyro_bi_tau[ax]:.1f}s)  "
              f"RRW={gyro_rrw[ax]:.4e} rad/s²/√Hz")
        print(f"    Accel {axes[ax]}: VRW={accel_vrw[ax]:.4e} m/s²/√Hz  "
              f"BI={accel_bi[ax]:.4e} m/s² (τ={accel_bi_tau[ax]:.1f}s)  "
              f"ARW={accel_arw[ax]:.4e} m/s³/√Hz")

    # ── Summary ──
    print(f"\n{'='*60}")
    print("SUMMARY  (all units: rad, m, s)")
    print(f"{'='*60}")

    print(f"\nGyroscope:")
    print(f"  Noise density (white):  {gyro_noise_density}  [rad/s/√Hz]")
    print(f"  Allan ARW:              {gyro_arw}  [rad/s/√Hz]")
    print(f"  Bias instability:       {gyro_bi}  [rad/s]")
    print(f"  Rate random walk:       {gyro_rrw}  [rad/s²/√Hz]")
    print(f"  Bias (mean):            {gyro_bias}  [rad/s]")

    print(f"\nAccelerometer:")
    print(f"  Noise density (white):  {accel_noise_density}  [m/s²/√Hz]")
    print(f"  Allan VRW:              {accel_vrw}  [m/s²/√Hz]")
    print(f"  Bias instability:       {accel_bi}  [m/s²]")
    print(f"  Accel random walk:      {accel_arw}  [m/s³/√Hz]")
    print(f"  Bias (mean):            {accel_bias}  [m/s²]")

    # ── Config snippets (max across axes) ──
    gyr = float(np.max(gyro_arw))
    gyr_bias = float(np.max(gyro_rrw))
    acc = float(np.max(accel_vrw[:2]))   # exclude z outlier if present
    acc_bias = float(np.max(accel_arw[:2]))

    print(f"\n{'='*60}")
    print("SUGGESTED CONFIG VALUES  (max across axes, z-accel excluded)")
    print(f"{'='*60}")

    print(f"\n  # EqVIO velocityNoise:")
    print(f"  gyr:     {gyr:.5e}   # gyro white noise  [rad/s/√Hz]")
    print(f"  gyrBias: {gyr_bias:.5e}   # gyro random walk  [rad/s²/√Hz]")
    print(f"  acc:     {acc:.5e}   # accel white noise  [m/s²/√Hz]")
    print(f"  accBias: {acc_bias:.5e}   # accel random walk  [m/s³/√Hz]")

    print(f"\n  # OpenVINS kalibr_imu_chain.yaml:")
    print(f"  gyroscope_noise_density:     {gyr:.5e}")
    print(f"  gyroscope_random_walk:       {gyr_bias:.5e}")
    print(f"  accelerometer_noise_density: {acc:.5e}")
    print(f"  accelerometer_random_walk:   {acc_bias:.5e}")

    # ── Save ──
    if args.save:
        np.savez(
            args.save,
            gyro_bias=gyro_bias, accel_bias=accel_bias,
            gyro_noise_density=gyro_noise_density,
            accel_noise_density=accel_noise_density,
            gyro_arw=gyro_arw, gyro_bi=gyro_bi, gyro_rrw=gyro_rrw,
            accel_vrw=accel_vrw, accel_bi=accel_bi, accel_arw=accel_arw,
            gyro_std=gyro_std, accel_std=accel_std,
            rate=rate, duration=duration,
        )
        print(f"\nAllan data saved to {args.save}")


if __name__ == "__main__":
    main()
