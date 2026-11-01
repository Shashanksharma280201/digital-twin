"""Wire format shared with the Blender add-on (sim/blender/addon/dt_live/protocol.py).

Frame: [u32 little-endian header length][header JSON, utf-8][payload]
Header: {"type": str, "stamp": float, "frame": str, "dtype": "f32", "shape": [..], "nbytes": int}
Payload: little-endian float32 array, C order. Both copies of this file must stay identical.
"""
import json
import struct

import numpy as np

_LEN = struct.Struct('<I')


def encode(kind, array=None, stamp=0.0, frame='world'):
    payload = b''
    header = {'type': kind, 'stamp': float(stamp), 'frame': frame, 'dtype': 'f32', 'shape': [0]}
    if array is not None:
        data = np.ascontiguousarray(array, dtype='<f4')
        payload = data.tobytes()
        header['shape'] = list(data.shape)
    header['nbytes'] = len(payload)
    head = json.dumps(header, separators=(',', ':')).encode('utf-8')
    return _LEN.pack(len(head)) + head + payload


class Decoder:
    """Incremental decoder: feed() bytes as they arrive, get complete (header, array) frames."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data):
        self._buf += data
        frames = []
        while True:
            if len(self._buf) < _LEN.size:
                break
            (head_len,) = _LEN.unpack_from(self._buf, 0)
            if len(self._buf) < _LEN.size + head_len:
                break
            header = json.loads(bytes(self._buf[_LEN.size:_LEN.size + head_len]))
            total = _LEN.size + head_len + header['nbytes']
            if len(self._buf) < total:
                break
            payload = bytes(self._buf[_LEN.size + head_len:total])
            del self._buf[:total]
            array = None
            if header['nbytes']:
                array = np.frombuffer(payload, dtype='<f4').reshape(header['shape'])
            frames.append((header, array))
        return frames
