import filecmp
import os
import socket
import time

import numpy as np
import pytest

from dt_blender_bridge.node import Server, VoxelMap
from dt_blender_bridge.protocol import Decoder, encode

ADDON_PROTOCOL = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'sim', 'blender',
                              'addon', 'dt_live', 'protocol.py')


def test_roundtrip_split_stream():
    pose = np.array([1, 2, 3, 0, 0, 0, 1], np.float32)
    cloud = np.random.rand(1000, 4).astype(np.float32)
    data = encode('pose', pose, 12.5) + encode('cloud', cloud) + encode('hello')
    dec = Decoder()
    frames = []
    for i in range(0, len(data), 777):  # arbitrary TCP segmentation
        frames += dec.feed(data[i:i + 777])
    assert [h['type'] for h, _ in frames] == ['pose', 'cloud', 'hello']
    assert frames[0][0]['stamp'] == 12.5
    np.testing.assert_array_equal(frames[0][1], pose)
    np.testing.assert_array_equal(frames[1][1], cloud)
    assert frames[2][1] is None


def test_voxel_map_dedup_and_cap():
    m = VoxelMap(voxel=0.1, max_points=150)
    pts = np.zeros((100, 4), np.float32)
    pts[:, 0] = np.arange(100) * 0.1 + 0.05  # 100 distinct voxels
    m.add(pts)
    m.add(pts)  # same voxels again: no change
    assert len(m.points) == 100 and m.version == 1
    pts[:, 1] = 5.0
    m.add(pts)
    assert len(m.points) == 150  # capped


def test_server_delivers_frames():
    class Log:
        def info(self, _):
            pass
    server = Server('127.0.0.1', 0, Log())
    port = server.sock.getsockname()[1]
    client = socket.create_connection(('127.0.0.1', port))
    deadline = time.time() + 2
    while not server.connected and time.time() < deadline:
        time.sleep(0.01)
    server.send(encode('pose', np.arange(7, dtype=np.float32)))
    client.settimeout(2)
    dec, frames = Decoder(), []
    while not frames:
        frames = dec.feed(client.recv(4096))
    assert frames[0][0]['type'] == 'pose'
    client.close()


@pytest.mark.skipif(not os.path.exists(ADDON_PROTOCOL), reason='Blender add-on not present')
def test_addon_protocol_is_identical():
    here = os.path.join(os.path.dirname(__file__), '..', 'dt_blender_bridge', 'protocol.py')
    assert filecmp.cmp(here, ADDON_PROTOCOL, shallow=False)
