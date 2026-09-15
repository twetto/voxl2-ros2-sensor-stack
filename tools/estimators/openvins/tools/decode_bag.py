#!/usr/bin/env python3
"""Decode one VOXL2 flight bag into the raw inputs of ov_serial.

Writes into <out_dir>:
  frames.u8   N x 800 x 1280 luma bytes, one decoded frame after another
  cam_t.txt   per frame: header stamp sec nanosec (the original encoded-message stamp)
  imu.csv     /voxl/raw_imu, per message: sec,nanosec,gx,gy,gz,ax,ay,az (header stamps, unflipped)
  decode.json counts and the first-IDR / fallback-parameter-set bookkeeping

The recorder starts mid-stream, so there is no VPS/SPS/PPS in the bag. Decoding starts
at the first IDR frame with voxl_h265_decoder's fallback parameter sets prepended
(same as scripts/eval/h265.py).
No restamping: each frame keeps the header stamp of the packet it came from.

Run in the ubuntu-22-04 distrobox, ROS 2 Humble sourced, venv with PyAV (PyAV):
  python decode_bag.py <bag_dir> <out_dir>
"""
import glob
import json
import os
import sqlite3
import sys

import av
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

CAM_TOPIC = "/tracking_front_misp_encoded"
IMU_TOPIC = "/voxl/raw_imu"
W, H = 1280, 800
# default of voxl_h265_decoder's fallback_codec_params (h265_decoder_node.cpp)
FALLBACK_CODEC_PARAMS = bytes.fromhex(
    "0000000140010c01ffff016000000300b00000030000030096ac09"
    "00000001420101016000000300b00000030000030096a002808032165aee4c92ea5005da1425"
    "000000014401c0e30f09418f610800")


def nal_types(b):
    types, j = [], 0
    while True:
        k = b.find(b"\x00\x00\x01", j)
        if k < 0 or k + 3 >= len(b):
            return types
        types.append((b[k + 3] >> 1) & 0x3F)
        j = k + 3


def first_decodable(datas):
    for i, b in enumerate(datas):
        t = nal_types(b)
        if 32 in t:
            return i, False
        if 19 in t or 20 in t:
            return i, True
    raise SystemExit("no decodable H.265 frame in the bag")


def read_topic(con, topic):
    tid, typ = con.execute("select id, type from topics where name=?", (topic,)).fetchone()
    rows = con.execute("select timestamp, data from messages where topic_id=? order by timestamp",
                       (tid,)).fetchall()
    cls = get_message(typ)
    return [deserialize_message(d, cls) for _, d in rows]


def main():
    bag_dir, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)
    db = glob.glob(os.path.join(bag_dir, "*.db3"))[0]
    con = sqlite3.connect("file:%s?immutable=1" % db, uri=True)

    imu = read_topic(con, IMU_TOPIC)
    with open(os.path.join(out_dir, "imu.csv"), "w") as f:
        for m in imu:
            g, a = m.angular_velocity, m.linear_acceleration
            f.write("%d,%d,%r,%r,%r,%r,%r,%r\n" % (m.header.stamp.sec, m.header.stamp.nanosec,
                                                   g.x, g.y, g.z, a.x, a.y, a.z))

    cam = read_topic(con, CAM_TOPIC)
    datas = [bytes(m.data) for m in cam]
    stamps = [(m.header.stamp.sec, m.header.stamp.nanosec) for m in cam]
    i0, needs_params = first_decodable(datas)
    dec = av.CodecContext.create("hevc", "r")
    st = dict(n=0, corrupt=0, bad_size=0, errors=0, last_pts=-1, out_of_order=0)
    fimg = open(os.path.join(out_dir, "frames.u8"), "wb")
    ft = open(os.path.join(out_dir, "cam_t.txt"), "w")

    def take(fr):
        if fr.width != W or fr.height != H:
            st["bad_size"] += 1
            return
        if getattr(fr, "is_corrupt", False):
            st["corrupt"] += 1          # kept (live node would drop it); counted for the notes
        if fr.pts <= st["last_pts"]:
            st["out_of_order"] += 1
        st["last_pts"] = fr.pts
        y = np.frombuffer(fr.planes[0], np.uint8).reshape(fr.height, fr.planes[0].line_size)
        fimg.write(np.ascontiguousarray(y[:, :W]).tobytes())
        ft.write("%d %d\n" % stamps[fr.pts])
        st["n"] += 1

    for i in range(i0, len(datas)):
        data = FALLBACK_CODEC_PARAMS + datas[i] if (i == i0 and needs_params) else datas[i]
        pkt = av.Packet(data)
        pkt.pts = i
        try:
            frames = dec.decode(pkt)
        except av.error.InvalidDataError:
            st["errors"] += 1
            continue
        for fr in frames:
            take(fr)
    for fr in dec.decode(None):
        take(fr)
    fimg.close()
    ft.close()
    info = dict(bag=os.path.abspath(db), n_imu=len(imu), n_cam_msgs=len(cam), first_idr_msg=i0,
                fallback_params=needs_params, n_frames=st["n"], n_corrupt_flagged=st["corrupt"],
                n_bad_size=st["bad_size"], n_decode_errors=st["errors"],
                n_out_of_order=st["out_of_order"],
                t_first_msg=stamps[0][0] + stamps[0][1] * 1e-9,
                t_first_frame=stamps[i0][0] + stamps[i0][1] * 1e-9)
    with open(os.path.join(out_dir, "decode.json"), "w") as f:
        json.dump(info, f, indent=2)
    print(json.dumps(info))


if __name__ == "__main__":
    main()
