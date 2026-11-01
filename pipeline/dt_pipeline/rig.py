"""Rig extrinsics (``rig.yaml``), shared with the ROS side (``dt_bringup`` profiles).

Schema (all keys optional; missing transforms default to identity)::

    profile: mid360                 # informational
    T_base_lidar:                   # pose of lidar_link in base_link (the mount)
      xyz: [0.0, 0.0, 0.5]
      rpy: [0.0, 0.0, 0.0]          # radians, ROS fixed-axis roll/pitch/yaw
    T_lidar_imu:                    # pose of the canonical IMU ("body") in lidar_link
      xyz: [0.011, 0.02329, -0.04412]
      quat_xyzw: [0, 0, 0, 1]       # either rpy or quat_xyzw
    T_lidar_camera:                 # pose of zed_camera_link in lidar_link (static TF)
      xyz: [...]
      rpy: [...]
    T_lidar_camera_optical:         # pose of the left rectified optical frame in lidar_link
      xyz: [...]                    # (OpenCV axes: x right, y down, z forward);
      quat_xyzw: [...]              # output of direct_visual_lidar_calibration
    camera_time_offset: 0.0         # seconds added to image stamps to get LiDAR time

``T_a_b`` always maps points from frame b into frame a (``p_a = T_a_b @ p_b``).
If ``T_lidar_camera_optical`` is absent it is derived from ``T_lidar_camera``, the ZED 2i
left-lens offset in zed_camera_link (zed-ros2-wrapper URDF) and the camera-link → optical rotation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .geometry import from_xyz_quat, from_xyz_rpy, make_T

# zed_camera_link (x fwd, y left, z up) -> optical (x right, y down, z fwd)
R_LINK_OPTICAL = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
# zed_camera_link -> zed_left_camera_frame for the ZED 2i (URDF: centre z = height/2, left lens
# x = optical_offset_x, y = baseline/2). The published image is the left rectified view.
T_LINK_LEFT_ZED2I = make_T(t=[-0.01, 0.06, 0.015])


def parse_transform(entry) -> np.ndarray:
    if entry is None:
        return np.eye(4)
    if isinstance(entry, (list, tuple)):
        return np.asarray(entry, dtype=float).reshape(4, 4)
    xyz = entry.get("xyz", [0.0, 0.0, 0.0])
    if "quat_xyzw" in entry:
        return from_xyz_quat(xyz, entry["quat_xyzw"])
    return from_xyz_rpy(xyz, entry.get("rpy", [0.0, 0.0, 0.0]))


@dataclass
class Rig:
    T_base_lidar: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_lidar_imu: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_lidar_camera: np.ndarray = field(default_factory=lambda: np.eye(4))
    T_lidar_camera_optical: np.ndarray | None = None
    camera_time_offset: float = 0.0
    profile: str = ""

    @classmethod
    def load(cls, path: str | Path) -> "Rig":
        data = yaml.safe_load(Path(path).read_text()) or {}
        data = data.get("rig", data)
        optical = data.get("T_lidar_camera_optical")
        return cls(
            T_base_lidar=parse_transform(data.get("T_base_lidar")),
            T_lidar_imu=parse_transform(data.get("T_lidar_imu")),
            T_lidar_camera=parse_transform(data.get("T_lidar_camera")),
            T_lidar_camera_optical=parse_transform(optical) if optical is not None else None,
            camera_time_offset=float(data.get("camera_time_offset", 0.0)),
            profile=str(data.get("profile", "")),
        )

    @property
    def lidar_to_optical(self) -> np.ndarray:
        """``T_lidar_optical``: pose of the image optical frame in lidar_link."""
        if self.T_lidar_camera_optical is not None:
            return self.T_lidar_camera_optical
        return self.T_lidar_camera @ T_LINK_LEFT_ZED2I @ make_T(R_LINK_OPTICAL)
