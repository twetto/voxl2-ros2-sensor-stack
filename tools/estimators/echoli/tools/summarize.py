#!/usr/bin/env python3
"""Per-bag summary of the 0914 ECHO-LI replays (for notes.md).

For each ~/.cache/echo-li/eval/flight0914_<HHMMSS> with runs/<run>.npz:
decode start, frontend feature counts, filter drop/divergence flags, the
evaluate.py metrics, the aligned end-point error, the path length and height
range from mocap, and the time the estimate takes to leave the ground.

    python3 summarize.py [run=tuned_id1] [HHMMSS ...]
"""
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.environ.get("ECHO_LI", os.path.expanduser("~/Documents/repos/echo-li")),
                                "tools/mocap_eval"))
import evaluate as E  # noqa: E402

ROOT = os.path.expanduser("~/.cache/echo-li/eval")


def summarize(cache, run):
    info = json.load(open(os.path.join(cache, "info.json")))
    traj_path = os.path.join(cache, "runs", run + ".npz")
    if not os.path.exists(traj_path):
        return None
    tag = run.split("_")[-1]
    tracks = np.load(os.path.join(cache, f"tracks_{tag}.npz"))
    traj = np.load(traj_path)
    meta = json.loads(str(traj["meta"]))
    sensors = np.load(os.path.join(cache, "sensors.npz"))
    mocap = E.load_mocap(sensors, "vrpn")
    sync = E.cached_clock_offset(cache, sensors, mocap[0], mocap[2], "vrpn")
    m, d = E.evaluate(traj, mocap, sync["offset_ns"], vertical_axis=1)
    cnt = tracks["count"]
    G = d["G"]
    path = float(np.sum(np.linalg.norm(np.diff(G[:, [0, 2]], axis=0), axis=1)))
    h = G[:, 1] - G[:5, 1].mean()
    air = np.flatnonzero(h > 0.10)
    step = np.linalg.norm(np.diff(traj["p"], axis=0), axis=1)
    return dict(
        bag=os.path.basename(cache)[-6:],
        frames=f"{info['frames_decoded']}/{info['clocks']['cam']['n']}",
        decode_start_s=round(info["decode"]["skipped_before_first_decodable_s"], 2),
        fallback=info["decode"]["fallback_params"],
        feat_med=int(np.median(cnt)), feat_p5=int(np.percentile(cnt, 5)),
        frames_lt5=float(np.mean(cnt < 5)),
        poses=len(traj["t_ns"]), dropped=meta["dropped_images"], diverged=meta["diverged"],
        max_step_m=float(step.max()), max_speed=float(np.linalg.norm(traj["v"], axis=1).max()),
        ate=m["ate_rmse"], ate_max=m["ate_max"], ate_h=m["ate_horiz_rmse"], ate_v=m["ate_vert_rmse"],
        end_err=float(np.linalg.norm(d["P_al"][-1] - G[-1])),
        scale=m["sim3_scale"], rot=m["rot_rmse"], vbody=m["vel_body_rmse"],
        rpe5=m["rpe5"], coverage=m["coverage"], dur=m["duration_s"],
        corr=sync["corr"], offset_s=sync["offset_ns"] / 1e9,
        path_m=path, h_max=float(h.max()),
        airborne_s=float((air[-1] - air[0]) / (len(h) / m["duration_s"])) if len(air) else 0.0)


def main():
    run = sys.argv[1] if len(sys.argv) > 1 else "tuned_id1"
    bags = sys.argv[2:] or sorted(os.path.basename(p)[-6:]
                                  for p in glob.glob(os.path.join(ROOT, "flight0914_[0-9]*"))
                                  if os.path.isdir(p))
    rows = [r for b in bags if (r := summarize(os.path.join(ROOT, "flight0914_" + b), run))]
    for r in rows:
        print(json.dumps({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()}))


if __name__ == "__main__":
    main()
