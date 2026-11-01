"""Rig geometry helpers shared by the launch files.

rig.yaml stores poses as ``T_a_b: {xyz: [..], rpy: [..]}`` or ``{xyz, quat_xyzw}``,
meaning the pose of frame b in frame a. The same schema is read by
pipeline/dt_pipeline/rig.py.
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory

PROFILES = ('mid360', 'airy')


def profile_dir(profile):
    if profile not in PROFILES:
        raise ValueError(f'unknown profile {profile!r}, expected one of {PROFILES}')
    return os.path.join(get_package_share_directory('dt_bringup'), 'config', 'profiles', profile)


def load_rig(profile):
    with open(os.path.join(profile_dir(profile), 'rig.yaml')) as f:
        return yaml.safe_load(f)


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    qc = (-q[0], -q[1], -q[2], q[3])
    x, y, z, _ = _quat_mul(_quat_mul(q, qv), qc)
    return (x, y, z)


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def pose(entry):
    """Returns (xyz, quat_xyzw) for a rig.yaml pose entry."""
    xyz = tuple(float(v) for v in entry.get('xyz', (0.0, 0.0, 0.0)))
    if 'quat_xyzw' in entry:
        q = tuple(float(v) for v in entry['quat_xyzw'])
    else:
        q = rpy_to_quat(*(float(v) for v in entry.get('rpy', (0.0, 0.0, 0.0))))
    return xyz, q


def invert(xyz, q):
    qi = (-q[0], -q[1], -q[2], q[3])
    t = _rotate(qi, xyz)
    return (-t[0], -t[1], -t[2]), qi


def static_tf_args(parent, child, xyz, q):
    return [
        '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
        '--qx', str(q[0]), '--qy', str(q[1]), '--qz', str(q[2]), '--qw', str(q[3]),
        '--frame-id', parent, '--child-frame-id', child,
    ]


def rig_static_transforms(rig, camera_frame='zed_camera_link'):
    """(parent, child, xyz, quat) tuples for the sensor tree rooted at the IMU frame 'body'.

    body -> lidar_link -> {base_link, camera_frame}; see design §4.4.
    """
    lidar_in_body = invert(*pose(rig['T_lidar_imu']))
    base_in_lidar = invert(*pose(rig['T_base_lidar']))
    camera_in_lidar = pose(rig['T_lidar_camera'])
    return [
        ('body', 'lidar_link', *lidar_in_body),
        ('lidar_link', 'base_link', *base_in_lidar),
        ('lidar_link', camera_frame, *camera_in_lidar),
    ]
