"""Decode the VOXL tracking camera's H.265 stream from recorded messages, no ROS.

The VOXL encoder sends VPS/SPS/PPS only once, at stream start. A bag recorded
mid-stream therefore has none, so decoding starts at its first IDR frame with
voxl_h265_decoder's fallback parameter sets prepended, as that node does. A bag
that caught a stream start (e.g. after a VOXL reboot) decodes as it is.
Needs PyAV (pip install av).
"""
import av
import numpy as np

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
    """(index to start decoding at, whether it needs the fallback parameter sets)."""
    for i, b in enumerate(datas):
        t = nal_types(b)
        if 32 in t:                                   # VPS: the stream's own start
            return i, False
        if 19 in t or 20 in t:                        # IDR without parameter sets
            return i, True
    raise ValueError("no decodable H.265 frame")


def _luma(fr):
    y = np.frombuffer(fr.planes[0], np.uint8).reshape(fr.height, fr.planes[0].line_size)
    return np.ascontiguousarray(y[:, :fr.width])


def decode(datas):
    """Yield (message index, 8-bit luma image) per decoded frame, in stream order."""
    i0, needs_params = first_decodable(datas)
    dec = av.CodecContext.create("hevc", "r")
    for i in range(i0, len(datas)):
        data = FALLBACK_CODEC_PARAMS + datas[i] if (i == i0 and needs_params) else datas[i]
        pkt = av.Packet(data)
        pkt.pts = i
        try:
            frames = dec.decode(pkt)
        except av.error.InvalidDataError:              # a packet the decoder can't use yet
            continue
        for fr in frames:
            yield fr.pts, _luma(fr)
    for fr in dec.decode(None):
        yield fr.pts, _luma(fr)
