import math

import pytest

from dt_bringup.rig import invert, pose, rig_static_transforms, rpy_to_quat


def _rotate(q, v):
    x, y, z, w = q
    # Rotation matrix from quaternion.
    m = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def test_rpy_yaw_90():
    q = rpy_to_quat(0.0, 0.0, math.pi / 2)
    assert _rotate(q, [1, 0, 0]) == pytest.approx([0, 1, 0], abs=1e-12)


def test_invert_roundtrip():
    xyz, q = pose({'xyz': [1.0, 2.0, 3.0], 'rpy': [0.1, -0.2, 0.3]})
    ixyz, iq = invert(xyz, q)
    p = [0.5, -0.4, 2.0]
    world = [a + b for a, b in zip(_rotate(q, p), xyz)]
    back = [a + b for a, b in zip(_rotate(iq, world), ixyz)]
    assert back == pytest.approx(p, abs=1e-12)


def test_mid360_body_to_lidar_matches_fast_lio_extrinsic():
    rig = {
        'T_lidar_imu': {'xyz': [0.011, 0.02329, -0.04412], 'rpy': [0, 0, 0]},
        'T_base_lidar': {'xyz': [0, 0, 0.5], 'rpy': [0, 0, 0]},
        'T_lidar_camera': {'xyz': [0.08, 0, -0.06], 'rpy': [0, 0, 0]},
    }
    tfs = {(p, c): (xyz, q) for p, c, xyz, q in rig_static_transforms(rig)}
    xyz, _ = tfs[('body', 'lidar_link')]
    assert list(xyz) == pytest.approx([-0.011, -0.02329, 0.04412])  # FAST-LIO extrinsic_T
    xyz, _ = tfs[('lidar_link', 'base_link')]
    assert list(xyz) == pytest.approx([0, 0, -0.5])
