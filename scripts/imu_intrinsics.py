#!/usr/bin/env python3
"""Extract IMU intrinsics from a static ROS2 bag via Allan variance.

Usage:
    python imu_intrinsics.py <bag_dir> [--topic /voxl/raw_imu] [--save allan.npz]
                             [--accel-axes xyz] [--psd-band LO HI]

Each noise term is fitted only where the Allan deviation has that term's
slope: white noise on the first -1/2 stretch, random walk on a +1/2 stretch.
White noise is cross-checked against the PSD floor, which also stands in when
the curve has no -1/2 stretch. With no +1/2 stretch the random walk is not
observable, and the largest value the curve allows is reported as a bound.

Outputs per-axis and aggregate noise parameters in SI units (rad, m, s)
suitable for EqVIO (velocityNoise) and OpenVINS (kalibr_imu_chain.yaml).

Requires: pip install rosbags numpy
"""

import argparse
import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

GRAVITY = 9.80665     # m/s²
BI_SCALE = 0.6643     # Allan floor of flicker noise = 0.6643 × bias instability
SLOPE_TOL = 0.1       # max |local slope − ideal slope| inside a fit stretch
MIN_SPAN = 0.3        # a fit stretch covers at least this many decades of τ
MIN_POINTS = 5        # ... and at least this many τ values
MIN_CLUSTERS = 10     # σ(τ) is trusted up to τ = duration / MIN_CLUSTERS
PSD_DISAGREE = 0.3    # flag Allan and PSD white noise differing by more
AXES = "xyz"
GYRO_UNITS = ("rad/s/√Hz", "rad/s", "rad/s²/√Hz")
ACCEL_UNITS = ("m/s²/√Hz", "m/s²", "m/s³/√Hz")
SAVE_KEYS = {"N": "noise_density", "N_allan": "noise_density_allan",
             "N_psd": "noise_density_psd", "B": "bias_instability",
             "K": "random_walk"}


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


def local_slope(taus, adevs, half_width=0.1):
    """Least-squares d log σ / d log τ over ±half_width decades of each τ."""
    x, y = np.log10(taus), np.log10(adevs)
    i = np.arange(len(x))
    lo = np.minimum(np.searchsorted(x, x - half_width), np.maximum(i - 1, 0))
    hi = np.maximum(np.searchsorted(x, x + half_width, side="right"),
                    np.minimum(i + 2, len(x)))
    n = hi - lo

    def window_sum(v):
        cs = np.concatenate([[0.0], np.cumsum(v)])
        return cs[hi] - cs[lo]

    sx, sy, sxx, sxy = (window_sum(v) for v in (x, y, x * x, x * y))
    return (n * sxy - sx * sy) / (n * sxx - sx * sx)


def runs(mask):
    """[start, stop) index pairs of the True stretches of a boolean array."""
    edges = np.flatnonzero(np.diff(np.concatenate(
        [[0], mask.astype(np.int8), [0]])))
    return list(zip(edges[::2], edges[1::2]))


def long_enough(taus, start, stop):
    return (stop - start >= MIN_POINTS
            and np.log10(taus[stop - 1] / taus[start]) >= MIN_SPAN)


def psd_floor(rate_data, fs, band):
    """White-noise density sqrt(S/2) from the median one-sided PSD S in band.

    Welch estimate, ~16 s Hann segments, 50% overlap. The median ignores
    narrow vibration lines; the band has to sit above the flicker rise and
    below any on-chip low-pass roll-off.
    """
    x = rate_data - np.mean(rate_data)
    nper = min(1 << int(round(np.log2(16 * fs))), len(x))
    step = nper // 2
    win = np.hanning(nper)
    nseg = (len(x) - nper) // step + 1
    power = np.zeros(nper // 2 + 1)
    for k in range(nseg):
        seg = x[k * step:k * step + nper]
        power += np.abs(np.fft.rfft((seg - seg.mean()) * win)) ** 2
    psd = 2.0 * power / (nseg * fs * np.sum(win ** 2))
    f = np.fft.rfftfreq(nper, 1.0 / fs)
    sel = (f >= band[0]) & (f <= band[1])
    return float(np.sqrt(np.median(psd[sel]) / 2.0))


def analyse_axis(rate_data, dt, psd_band):
    """Allan curve and noise terms of one axis of static rate data.

    Returns a dict; 'notes' lists what the curve could not resolve.
    """
    taus, adevs = allan_variance(rate_data, dt)
    slopes = local_slope(taus, adevs)
    tau_trust = len(rate_data) * dt / MIN_CLUSTERS
    notes = []

    # White noise dominates the short-τ end, so only the first -1/2 stretch
    # is white noise; later ones belong to other processes.
    first = runs(np.abs(slopes + 0.5) <= SLOPE_TOL)[:1]
    white = first[0] if first and long_enough(taus, *first[0]) else None
    n_psd = psd_floor(rate_data, 1.0 / dt, psd_band)
    if white:
        a, b = white
        n_allan = 10 ** np.mean(np.log10(adevs[a:b])
                                + 0.5 * np.log10(taus[a:b]))
        if abs(n_psd / n_allan - 1) > PSD_DISAGREE:
            notes.append(f"PSD floor is {n_psd / n_allan - 1:+.0%} off the "
                         f"Allan fit")
    else:
        n_allan = np.nan
        notes.append("no -1/2 stretch: white noise taken from the PSD floor")

    imin = int(np.argmin(adevs))
    B = adevs[imin] / BI_SCALE
    if taus[imin] > tau_trust:
        notes.append("no bias-instability floor resolved: B is an upper bound")

    rising = (np.abs(slopes - 0.5) <= SLOPE_TOL) & (taus > taus[imin])
    fits = [r for r in runs(rising) if long_enough(taus, *r)]
    if fits:
        a, b = max(fits, key=lambda r: taus[r[1] - 1] / taus[r[0]])
        K = np.sqrt(3) * 10 ** np.mean(np.log10(adevs[a:b])
                                       - 0.5 * np.log10(taus[a:b]))
        k_range = (taus[a], taus[b - 1])
    else:
        # Every noise term only adds to σ(τ), so σ(τ) ≥ K·√(τ/3) caps K.
        cap = taus <= tau_trust
        K = np.min(np.sqrt(3) * adevs[cap] / np.sqrt(taus[cap]))
        k_range = None
        notes.append(f"no +1/2 stretch: random walk not observable, bound "
                     f"from tau <= {tau_trust:.0f} s")

    return dict(taus=taus, adevs=adevs,
                N=n_allan if white else n_psd, N_allan=n_allan, N_psd=n_psd,
                white_range=(taus[white[0]], taus[white[1] - 1])
                if white else None,
                B=B, B_tau=taus[imin], K=K, K_range=k_range, notes=notes)


def report(name, r, units):
    n_unit, b_unit, k_unit = units
    src = ("Allan -1/2 fit {:.3g}-{:.3g} s".format(*r["white_range"])
           if r["white_range"] else "PSD floor")
    print(f"  {name}  N  = {r['N']:.3e} {n_unit}   [{src}; "
          f"PSD floor {r['N_psd']:.3e}]")
    print(f"          B  = {r['B']:.3e} {b_unit}   "
          f"[Allan min at tau = {r['B_tau']:.3g} s]")
    if r["K_range"]:
        print(f"          K  = {r['K']:.3e} {k_unit}   "
              "[+1/2 fit {:.3g}-{:.3g} s]".format(*r["K_range"]))
    else:
        print(f"          K <= {r['K']:.3e} {k_unit}   [upper bound]")
    for note in r["notes"]:
        print(f"          ! {note}")


def worst(res, key, axes):
    """Largest res[axis][key] over axes, and which axis it came from."""
    i = max(axes, key=lambda a: res[a][key])
    bound = key == "K" and res[i]["K_range"] is None
    return res[i][key], AXES[i] + (", upper bound" if bound else "")


def main():
    parser = argparse.ArgumentParser(
        description="Extract IMU intrinsics from a static ROS2 bag via "
                    "overlapping Allan variance.")
    parser.add_argument("bag", type=Path,
                        help="Path to the ROS2 bag directory")
    parser.add_argument("--topic", default="/voxl/raw_imu",
                        help="IMU topic name (default: /voxl/raw_imu)")
    parser.add_argument("--save", type=Path, default=None,
                        help="Save parameters and Allan curves to .npz file")
    parser.add_argument("--accel-axes", default=AXES,
                        help="Accel axes the suggested config values are "
                             "taken over (default: xyz; 'xy' leaves out z)")
    parser.add_argument("--psd-band", type=float, nargs=2, metavar=("LO", "HI"),
                        help="PSD band in Hz for the white-noise floor "
                             "(default: rate/40 to rate/8)")
    args = parser.parse_args()
    if not args.accel_axes or set(args.accel_axes) - set(AXES):
        parser.error("--accel-axes takes letters from 'xyz'")

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
    dts = np.diff(stamps)
    dt = np.mean(dts)
    rate = 1.0 / dt
    duration = stamps[-1] - stamps[0]
    band = tuple(args.psd_band) if args.psd_band else (rate / 40, rate / 8)

    print(f"\nSamples: {N:,}")
    print(f"Duration: {duration:.1f}s ({duration/3600:.2f}h)")
    print(f"Mean rate: {rate:.1f} Hz")
    print(f"Mean dt: {dt*1e3:.3f} ms")
    print(f"Gaps: {np.sum(dts > 1.5 * np.median(dts))} (dt > 1.5x median), "
          f"non-increasing stamps: {np.sum(dts <= 0)}")

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
    if abs(accel_norm / GRAVITY - 1) > 0.01:
        print("  (norm is >1% off g: one static pose cannot tell a bias "
              "from a scale-factor error)")
    print(f"Gyro σ [rad/s]:     {np.std(gyro, axis=0)}")
    print(f"Accel σ [m/s²]:     {np.std(accel, axis=0)}")

    # ── Allan variance ──
    print(f"\n{'='*60}")
    print("ALLAN DEVIATION ANALYSIS")
    print(f"{'='*60}")
    print(f"Overlapping Allan variance on {N:,} samples, "
          f"PSD floor over {band[0]:.3g}-{band[1]:.3g} Hz\n")

    gyro_res, accel_res = [], []
    for label, data, res, units in (("Gyro ", gyro, gyro_res, GYRO_UNITS),
                                    ("Accel", accel, accel_res, ACCEL_UNITS)):
        for ax in range(3):
            res.append(analyse_axis(data[:, ax], dt, band))
            report(f"{label} {AXES[ax]}", res[-1], units)

    # ── Summary ──
    print(f"\n{'='*60}")
    print("SUMMARY  (all units: rad, m, s)")
    print(f"{'='*60}")

    def col(res, key):
        return np.array([r[key] for r in res])

    def bound_axes(res):
        axes = [AXES[i] for i, r in enumerate(res) if r["K_range"] is None]
        return f"  (upper bound: {', '.join(axes)})" if axes else ""

    print(f"\nGyroscope:")
    print(f"  Noise density:          {col(gyro_res, 'N')}  [rad/s/√Hz]")
    print(f"    Allan -1/2 fit:       {col(gyro_res, 'N_allan')}")
    print(f"    PSD floor:            {col(gyro_res, 'N_psd')}")
    print(f"  Bias instability:       {col(gyro_res, 'B')}  [rad/s]")
    print(f"  Rate random walk:       {col(gyro_res, 'K')}  [rad/s²/√Hz]"
          f"{bound_axes(gyro_res)}")
    print(f"  Bias (mean):            {gyro_bias}  [rad/s]")

    print(f"\nAccelerometer:")
    print(f"  Noise density:          {col(accel_res, 'N')}  [m/s²/√Hz]")
    print(f"    Allan -1/2 fit:       {col(accel_res, 'N_allan')}")
    print(f"    PSD floor:            {col(accel_res, 'N_psd')}")
    print(f"  Bias instability:       {col(accel_res, 'B')}  [m/s²]")
    print(f"  Accel random walk:      {col(accel_res, 'K')}  [m/s³/√Hz]"
          f"{bound_axes(accel_res)}")
    print(f"  Bias (mean):            {accel_bias}  [m/s²]")

    # ── Config snippets (max across axes) ──
    accel_axes = [AXES.index(c) for c in args.accel_axes]
    gyr, gyr_from = worst(gyro_res, "N", range(3))
    gyr_bias, gyr_bias_from = worst(gyro_res, "K", range(3))
    acc, acc_from = worst(accel_res, "N", accel_axes)
    acc_bias, acc_bias_from = worst(accel_res, "K", accel_axes)

    print(f"\n{'='*60}")
    print(f"SUGGESTED CONFIG VALUES  (max over gyro xyz, "
          f"accel {args.accel_axes})")
    print(f"{'='*60}")

    print(f"\n  # EqVIO velocityNoise:")
    print(f"  gyr:     {gyr:.5e}   # gyro white noise  [rad/s/√Hz] "
          f"({gyr_from})")
    print(f"  gyrBias: {gyr_bias:.5e}   # gyro random walk  [rad/s²/√Hz] "
          f"({gyr_bias_from})")
    print(f"  acc:     {acc:.5e}   # accel white noise  [m/s²/√Hz] "
          f"({acc_from})")
    print(f"  accBias: {acc_bias:.5e}   # accel random walk  [m/s³/√Hz] "
          f"({acc_bias_from})")

    print(f"\n  # OpenVINS kalibr_imu_chain.yaml:")
    print(f"  gyroscope_noise_density:     {gyr:.5e}")
    print(f"  gyroscope_random_walk:       {gyr_bias:.5e}")
    print(f"  accelerometer_noise_density: {acc:.5e}")
    print(f"  accelerometer_random_walk:   {acc_bias:.5e}")

    # ── Save ──
    if args.save:
        out = dict(taus=gyro_res[0]["taus"], rate=rate, duration=duration,
                   gyro_bias=gyro_bias, accel_bias=accel_bias,
                   accel_mean=accel_bias_raw)
        for sensor, res in (("gyro", gyro_res), ("accel", accel_res)):
            out[f"{sensor}_adev"] = col(res, "adevs")
            for key, name in SAVE_KEYS.items():
                out[f"{sensor}_{name}"] = col(res, key)
            out[f"{sensor}_random_walk_is_bound"] = np.array(
                [r["K_range"] is None for r in res])
        np.savez(args.save, allow_pickle=False, **out)
        print(f"\nParameters and Allan curves saved to {args.save}")


if __name__ == "__main__":
    main()
