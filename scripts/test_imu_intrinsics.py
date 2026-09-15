#!/usr/bin/env python3
"""Synthetic check of imu_intrinsics: 2 h of static rate data at 200 Hz with
known noise terms.
  white + RRW   white noise N plus rate random walk K
  white only    no random walk in the record
  zeroed        white + RRW minus a 200 s trailing-mean bias estimate, as an
                IMU that re-zeroes its own bias while still does
Checks:
  white + RRW: the Allan fit and the PSD floor both find N, the +1/2 fit finds K
  white only, zeroed: N is still found, and K is reported only as an upper
    bound, never as a fit
Usage: python3 test_imu_intrinsics.py
"""
import numpy as np

from imu_intrinsics import analyse_axis

FS, T_END = 200.0, 7200.0
N_TRUE, K_TRUE = 1.5e-4, 2e-5          # rad/s/√Hz, rad/s²/√Hz


def simulate(rrw=True, zero_window=None, seed=0):
    rng = np.random.default_rng(seed)
    n = int(FS * T_END)
    x = N_TRUE * np.sqrt(FS) * rng.standard_normal(n)
    if rrw:
        x += np.cumsum(K_TRUE / np.sqrt(FS) * rng.standard_normal(n))
    if zero_window:
        w = int(zero_window * FS)
        cs = np.concatenate([[0.0], np.cumsum(x)])
        i = np.arange(1, n + 1)
        lo = np.maximum(i - w, 0)
        x = x - (cs[i] - cs[lo]) / (i - lo)
    return x


def rel(value, truth):
    return abs(value / truth - 1.0)


def main():
    cases = {"white + RRW": simulate(),
             "white only": simulate(rrw=False),
             "zeroed": simulate(zero_window=200.0)}
    res = {name: analyse_axis(x, 1.0 / FS, (FS / 40, FS / 8))
           for name, x in cases.items()}
    print("synthetic static IMU %.0f h at %.0f Hz, N %.2e, K %.2e"
          % (T_END / 3600, FS, N_TRUE, K_TRUE))
    for name, r in res.items():
        print("  %-12s N %.3e (Allan %.3e, PSD %.3e)  K %s %.3e"
              % (name, r["N"], r["N_allan"], r["N_psd"],
                 "=" if r["K_range"] else "<=", r["K"]))
    base, white, zeroed = res["white + RRW"], res["white only"], res["zeroed"]
    checks = [("white + RRW: Allan N within 5%", rel(base["N_allan"], N_TRUE) < 0.05),
              ("white + RRW: PSD N within 5%", rel(base["N_psd"], N_TRUE) < 0.05),
              ("white + RRW: K fitted within 30%",
               base["K_range"] is not None and rel(base["K"], K_TRUE) < 0.3),
              ("white only: N within 5%", rel(white["N"], N_TRUE) < 0.05),
              ("white only: K only an upper bound", white["K_range"] is None),
              ("zeroed: N within 5%", rel(zeroed["N"], N_TRUE) < 0.05),
              ("zeroed: K only an upper bound", zeroed["K_range"] is None)]
    ok = True
    for label, passed in checks:
        print("  %-50s %s" % (label, "PASS" if passed else "FAIL"))
        ok &= bool(passed)
    print("ALL PASS" if ok else "SOME FAILED")


if __name__ == "__main__":
    main()
