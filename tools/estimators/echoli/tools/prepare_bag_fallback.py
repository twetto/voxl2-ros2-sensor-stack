#!/usr/bin/env python3
"""prepare_bag.py for bags recorded mid-stream (no H.265 VPS/SPS/PPS).

Copy of echo-li/tools/mocap_eval/prepare_bag.py (2026-09-14). The only change
is decode_camera(): the VOXL encoder sends VPS/SPS/PPS once, at stream start,
so a recorder started later (the 2026-09-14 flight bags) has none.
Decoding starts at the first packet that is decodable: one carrying a VPS
(the stream's own start) or, failing that, the first IDR frame with
voxl_h265_decoder's fallback_codec_params prepended, as the decoder node does
(same logic as scripts/eval/h265.py).
Packets before that stay cam_decoded=False, which run_offline.py skips.
Packets PyAV rejects are skipped instead of aborting. info.json records the
start index and whether the fallback parameter sets were used.

Everything else (IMU, mocap, output layout) is unchanged, so run_offline.py and
evaluate.py read the cache as before.

    export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=92
    source /opt/ros/humble/setup.bash
    python3 \\
        prepare_bag_fallback.py <bag_dir> <out_dir>
"""
import argparse
import glob
import json
import os
import sqlite3
import time

import av
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CAM_TOPIC = "/tracking_front_misp_encoded"
IMU_TOPIC = "/voxl/raw_imu"
MOCAP_TOPICS = {"vrpn": "/vrpn_mocap/drone_01/pose",
                "vp": "/mocap_drone_01/vision_pose/pose"}
WIDTH, HEIGHT = 1280, 800
# default of voxl_h265_decoder's fallback_codec_params (h265_decoder_node.cpp)
FALLBACK_CODEC_PARAMS = bytes.fromhex(
    "0000000140010c01ffff016000000300b00000030000030096ac09"
    "00000001420101016000000300b00000030000030096a002808032165aee4c92ea5005da1425"
    "000000014401c0e30f09418f610800")


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class Bag:
    def __init__(self, bag_dir):
        dbs = glob.glob(os.path.join(bag_dir, "*.db3"))
        if len(dbs) != 1:
            raise SystemExit(f"expected one .db3 in {bag_dir}, found {len(dbs)}")
        self.path = os.path.abspath(dbs[0])
        self.con = sqlite3.connect(f"file:{self.path}?immutable=1", uri=True)
        self.topics = {name: (tid, typ) for tid, name, typ in
                       self.con.execute("select id, name, type from topics")}

    def count(self, topic):
        tid, _ = self.topics[topic]
        return self.con.execute("select count(*) from messages where topic_id=?",
                                (tid,)).fetchone()[0]

    def messages(self, topic):
        tid, typ = self.topics[topic]
        cls = get_message(typ)
        cur = self.con.execute("select timestamp, data from messages where topic_id=? "
                               "order by timestamp", (tid,))
        for bag_ns, data in cur:
            yield bag_ns, deserialize_message(data, cls)


def extract_imu(bag):
    bag_ns, hdr, gyr, acc = [], [], [], []
    for t, m in bag.messages(IMU_TOPIC):
        bag_ns.append(t)
        hdr.append(stamp_ns(m))
        g, a = m.angular_velocity, m.linear_acceleration
        gyr.append((g.x, g.y, g.z))
        acc.append((a.x, a.y, a.z))
    return dict(imu_bag=np.array(bag_ns, np.int64), imu_hdr=np.array(hdr, np.int64),
                imu_gyr=np.array(gyr), imu_acc=np.array(acc))


def extract_pose(bag, key, topic):
    bag_ns, hdr, pos, quat, frame_id = [], [], [], [], None
    for t, m in bag.messages(topic):
        bag_ns.append(t)
        hdr.append(stamp_ns(m))
        p, q = m.pose.position, m.pose.orientation
        pos.append((p.x, p.y, p.z))
        quat.append((q.x, q.y, q.z, q.w))
        frame_id = m.header.frame_id
    arrays = {f"{key}_bag": np.array(bag_ns, np.int64), f"{key}_hdr": np.array(hdr, np.int64),
              f"{key}_pos": np.array(pos), f"{key}_quat": np.array(quat)}
    return arrays, frame_id


def nal_types(b):
    types, j = [], 0
    while True:
        k = b.find(b"\x00\x00\x01", j)
        if k < 0 or k + 3 >= len(b):
            return types
        types.append((b[k + 3] >> 1) & 0x3F)
        j = k + 3


def first_decodable(datas):
    """(index to start decoding at, whether it needs the fallback parameter sets)."""
    for i, b in enumerate(datas):
        t = nal_types(b)
        if 32 in t:                                   # VPS: the stream's own start
            return i, False
        if 19 in t or 20 in t:                        # IDR without parameter sets
            return i, True
    raise SystemExit("no decodable H.265 frame in the bag")


def decode_camera(bag, frames_path):
    n = bag.count(CAM_TOPIC)
    cam_bag = np.zeros(n, np.int64)
    cam_hdr = np.zeros(n, np.int64)
    datas = []
    for i, (t, m) in enumerate(bag.messages(CAM_TOPIC)):
        if m.format != "h265":
            raise SystemExit(f"expected h265, got {m.format!r}")
        cam_bag[i] = t
        cam_hdr[i] = stamp_ns(m)
        datas.append(bytes(m.data))
    i0, needs_params = first_decodable(datas)

    frames = np.lib.format.open_memmap(frames_path, mode="w+", dtype=np.uint8,
                                       shape=(n, HEIGHT, WIDTH))
    decoded = np.zeros(n, bool)
    dec = av.CodecContext.create("hevc", "r")
    dec.thread_type = "AUTO"
    rejected = []

    def take(fr):
        if (fr.width, fr.height) != (WIDTH, HEIGHT):
            raise SystemExit(f"unexpected frame size {fr.width}x{fr.height}")
        plane = fr.planes[0]
        y = np.frombuffer(plane, np.uint8).reshape(fr.height, plane.line_size)
        frames[fr.pts] = y[:, :WIDTH]
        decoded[fr.pts] = True

    for i in range(i0, n):
        data = FALLBACK_CODEC_PARAMS + datas[i] if (i == i0 and needs_params) else datas[i]
        pkt = av.Packet(data)
        pkt.pts = i
        try:
            out = dec.decode(pkt)
        except av.error.InvalidDataError:           # a packet the decoder cannot use
            rejected.append(i)
            continue
        for fr in out:
            take(fr)
    for fr in dec.decode(None):
        take(fr)
    frames.flush()
    meta = dict(first_decodable_index=int(i0), fallback_params=bool(needs_params),
                rejected_packets=rejected,
                skipped_before_first_decodable_s=float((cam_hdr[i0] - cam_hdr[0]) * 1e-9))
    return dict(cam_bag=cam_bag, cam_hdr=cam_hdr, cam_decoded=decoded), meta


def clock_stats(arrays, key, t0_bag):
    b, h = arrays[f"{key}_bag"], arrays[f"{key}_hdr"]
    off = (h - b) * 1e-9
    dh = np.diff(h) * 1e-9
    return dict(n=int(len(b)),
                bag_start_s=float((b[0] - t0_bag) * 1e-9),
                bag_end_s=float((b[-1] - t0_bag) * 1e-9),
                hdr_minus_bag_median_s=float(np.median(off)),
                hdr_minus_bag_p1_s=float(np.percentile(off, 1)),
                hdr_minus_bag_p99_s=float(np.percentile(off, 99)),
                hdr_dt_median_ms=float(np.median(dh) * 1e3),
                hdr_dt_max_ms=float(dh.max() * 1e3),
                hdr_nonmonotonic=int((dh <= 0).sum()),
                rate_hz=float((len(b) - 1) / ((b[-1] - b[0]) * 1e-9)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag_dir")
    ap.add_argument("out_dir")
    args = ap.parse_args()

    bag = Bag(args.bag_dir)
    os.makedirs(args.out_dir, exist_ok=True)
    started = time.monotonic()
    arrays = extract_imu(bag)
    frame_ids = {}
    for key, topic in MOCAP_TOPICS.items():
        if topic in bag.topics:
            pose_arrays, frame_ids[key] = extract_pose(bag, key, topic)
            arrays.update(pose_arrays)
    print(f"sensors extracted in {time.monotonic() - started:.1f}s", flush=True)
    cam_arrays, decode_meta = decode_camera(bag, os.path.join(args.out_dir, "frames.npy"))
    arrays.update(cam_arrays)
    print(f"camera decoded in {time.monotonic() - started:.1f}s "
          f"({int(arrays['cam_decoded'].sum())}/{len(arrays['cam_decoded'])} frames, "
          f"start {decode_meta})", flush=True)
    np.savez(os.path.join(args.out_dir, "sensors.npz"), **arrays)

    t0_bag = int(arrays["imu_bag"][0])
    info = dict(bag=bag.path, topics={k: v[1] for k, v in bag.topics.items()},
                mocap_topics={k: t for k, t in MOCAP_TOPICS.items() if t in bag.topics},
                mocap_frame_ids=frame_ids,
                frames_decoded=int(arrays["cam_decoded"].sum()),
                decode=decode_meta,
                clocks={k: clock_stats(arrays, k, t0_bag)
                        for k in ["imu", "cam"] + [k for k in MOCAP_TOPICS
                                                   if f"{k}_bag" in arrays]})
    for key in MOCAP_TOPICS:
        if f"{key}_pos" in arrays:
            pq = np.hstack([arrays[f"{key}_pos"], arrays[f"{key}_quat"]])
            info["clocks"][key]["repeat_fraction"] = float(np.mean(np.all(pq[1:] == pq[:-1], axis=1)))
    with open(os.path.join(args.out_dir, "info.json"), "w") as f:
        json.dump(info, f, indent=2, ensure_ascii=False)
    print(json.dumps(info["clocks"], indent=1))


if __name__ == "__main__":
    main()
