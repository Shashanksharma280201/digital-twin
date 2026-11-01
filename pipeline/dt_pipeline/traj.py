"""Trajectories in the canonical form ``T_world_lidar(t)``.

Sources:
* GLIM dump: ``traj_lidar.txt`` is already ``T_world_lidar`` (TUM).
* FAST-LIO ``/Odometry``: pose of ``body`` (IMU) in ``camera_init``, converted with the rig
  extrinsic ``T_lidar_imu``: ``T_world_lidar = T_world_body @ inv(T_lidar_imu)``.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import Trajectory, inv
from .io import ODOM_TOPIC, Bag
from .io.bag import stamp_to_sec
from .rig import Rig


def from_glim_dump(dump: str | Path) -> Trajectory:
    dump = Path(dump)
    path = dump / "traj_lidar.txt" if dump.is_dir() else dump
    return Trajectory.from_tum(path)


def from_odometry(bag_path: str | Path, rig: Rig, topic: str = ODOM_TOPIC) -> Trajectory:
    stamps, xyz, quat = [], [], []
    with Bag(bag_path) as bag:
        for _, _, msg in bag.messages([topic]):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            stamps.append(stamp_to_sec(msg.header.stamp))
            xyz.append((p.x, p.y, p.z))
            quat.append((q.x, q.y, q.z, q.w))
    if len(stamps) < 2:
        raise LookupError(f"not enough {topic} messages in {bag_path}")
    T = np.tile(np.eye(4), (len(stamps), 1, 1))
    T[:, :3, :3] = Rotation.from_quat(np.asarray(quat)).as_matrix()
    T[:, :3, 3] = xyz
    return Trajectory(np.asarray(stamps), T).compose(inv(rig.T_lidar_imu))
