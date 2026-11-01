"""SE(3) helpers, TUM trajectory I/O and pose interpolation.

Conventions: 4x4 homogeneous matrices, ``T_a_b`` maps points from frame b to frame a
(``p_a = T_a_b @ p_b``). Quaternions are scalar-last ``[qx, qy, qz, qw]`` unless noted.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def make_T(R: np.ndarray | None = None, t: np.ndarray | None = None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def inv(T: np.ndarray) -> np.ndarray:
    """Inverse of one (4x4) or many (Nx4x4) rigid transforms."""
    R = T[..., :3, :3]
    t = T[..., :3, 3]
    out = np.zeros_like(T)
    Rt = np.swapaxes(R, -1, -2)
    out[..., :3, :3] = Rt
    out[..., :3, 3] = -np.einsum("...ij,...j->...i", Rt, t)
    out[..., 3, 3] = 1.0
    return out


def from_xyz_quat(xyz, quat_xyzw) -> np.ndarray:
    return make_T(Rotation.from_quat(quat_xyzw).as_matrix(), np.asarray(xyz, dtype=float))


def from_xyz_rpy(xyz, rpy) -> np.ndarray:
    """ROS convention: fixed-axis roll (x), pitch (y), yaw (z), radians."""
    return make_T(Rotation.from_euler("xyz", rpy).as_matrix(), np.asarray(xyz, dtype=float))


def to_xyz_quat(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return T[..., :3, 3].copy(), Rotation.from_matrix(T[..., :3, :3]).as_quat()


def transform_points(T: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    return xyz @ T[:3, :3].T + T[:3, 3]


def read_tum(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Read ``t x y z qx qy qz qw`` lines. Returns (stamps [s], poses Nx4x4)."""
    data = np.loadtxt(path, comments="#", ndmin=2)
    if data.shape[1] != 8:
        raise ValueError(f"{path}: expected 8 columns (TUM), got {data.shape[1]}")
    T = np.tile(np.eye(4), (len(data), 1, 1))
    T[:, :3, :3] = Rotation.from_quat(data[:, 4:8]).as_matrix()
    T[:, :3, 3] = data[:, 1:4]
    return data[:, 0], T


def write_tum(path: str | Path, stamps: np.ndarray, poses: np.ndarray) -> None:
    xyz, q = to_xyz_quat(poses)
    data = np.column_stack([stamps, xyz, q])
    np.savetxt(path, data, fmt="%.9f %.6f %.6f %.6f %.9f %.9f %.9f %.9f",
               header="timestamp x y z qx qy qz qw")


@dataclass
class Trajectory:
    """Time-stamped poses ``T_world_frame`` with SLERP/linear interpolation."""

    stamps: np.ndarray  # seconds, strictly increasing
    poses: np.ndarray  # Nx4x4

    def __post_init__(self):
        order = np.argsort(self.stamps, kind="stable")
        stamps = np.asarray(self.stamps, dtype=float)[order]
        poses = np.asarray(self.poses, dtype=float)[order]
        keep = np.concatenate([[True], np.diff(stamps) > 0])
        self.stamps, self.poses = stamps[keep], poses[keep]
        if len(self.stamps) < 2:
            raise ValueError("trajectory needs at least two distinct poses")
        self._slerp = Slerp(self.stamps, Rotation.from_matrix(self.poses[:, :3, :3]))

    @classmethod
    def from_tum(cls, path: str | Path) -> "Trajectory":
        return cls(*read_tum(path))

    def to_tum(self, path: str | Path) -> None:
        write_tum(path, self.stamps, self.poses)

    @property
    def start(self) -> float:
        return float(self.stamps[0])

    @property
    def end(self) -> float:
        return float(self.stamps[-1])

    def covers(self, t) -> np.ndarray:
        t = np.asarray(t, dtype=float)
        return (t >= self.start) & (t <= self.end)

    def interpolate(self, t) -> np.ndarray:
        """Poses at times ``t`` (scalar or array). Times must lie inside the trajectory."""
        t = np.atleast_1d(np.asarray(t, dtype=float))
        if not np.all(self.covers(t)):
            raise ValueError("interpolation time outside trajectory span")
        out = np.tile(np.eye(4), (len(t), 1, 1))
        out[:, :3, :3] = self._slerp(t).as_matrix()
        for k in range(3):
            out[:, k, 3] = np.interp(t, self.stamps, self.poses[:, k, 3])
        return out

    def compose(self, T_right: np.ndarray, T_left: np.ndarray | None = None) -> "Trajectory":
        """Return ``T_left @ poses @ T_right`` (e.g. change of tracked frame or world frame)."""
        poses = self.poses @ T_right
        if T_left is not None:
            poses = T_left @ poses
        return Trajectory(self.stamps.copy(), poses)

    def transform_points_at(self, xyz: np.ndarray, t: np.ndarray, resolution: float = 1e-3) -> np.ndarray:
        """Transform each point with the pose at its own time (deskew).

        Point times are quantised to ``resolution`` seconds so only a few hundred
        poses are interpolated per scan instead of one per point.
        """
        q = np.round(np.asarray(t, dtype=float) / resolution).astype(np.int64)
        keys, inverse = np.unique(q, return_inverse=True)
        tk = np.clip(keys * resolution, self.start, self.end)
        P = self.interpolate(tk)
        R = P[inverse, :3, :3]
        return np.einsum("nij,nj->ni", R, xyz) + P[inverse, :3, 3]
