"""Ground-truth device trajectory (trajectory.csv: t,x,y,z,qx,qy,qz,qw = T_world_base).

Poses at arbitrary times use linear position + Slerp over the 1 kHz samples (exact at
samples, cheap for per-point LiDAR timing). Derivatives for the IMU use a quintic
position spline and scipy's RotationSpline fitted on knots thinned to ``knot_rate`` Hz,
which keeps handheld motion (< ~15 Hz) while not amplifying CSV rounding noise.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.spatial.transform import Rotation, RotationSpline, Slerp


def load_csv(path: str | Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    t, xyz, q = data[:, 0], data[:, 1:4], data[:, 4:8]
    order = np.argsort(t, kind="stable")
    t, xyz, q = t[order], xyz[order], q[order]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return t[keep], xyz[keep], q[keep]


def save_csv(path: str | Path, t, xyz, quat_xyzw) -> None:
    data = np.column_stack([t, xyz, quat_xyzw])
    np.savetxt(path, data, delimiter=",", header="t,x,y,z,qx,qy,qz,qw", comments="", fmt="%.9f")


class Trajectory:
    """Poses of a frame in world; optionally re-anchored to a child frame (T_world_child)."""

    def __init__(self, t, xyz, quat_xyzw, T_frame_child: np.ndarray | None = None,
                 knot_rate: float = 200.0):
        t = np.asarray(t, dtype=float)
        rot = Rotation.from_quat(np.asarray(quat_xyzw, dtype=float))
        xyz = np.asarray(xyz, dtype=float)
        if T_frame_child is not None:
            xyz = xyz + rot.apply(T_frame_child[:3, 3])
            rot = rot * Rotation.from_matrix(T_frame_child[:3, :3])
        self.t, self.xyz, self.rot = t, xyz, rot
        self._slerp = Slerp(t, rot)

        step = max(1, int(round(1.0 / (knot_rate * np.median(np.diff(t)))))) if len(t) > 1 else 1
        idx = np.arange(0, len(t), step)
        if idx[-1] != len(t) - 1:
            idx = np.append(idx, len(t) - 1)
        k = 5 if len(idx) > 5 else max(1, len(idx) - 1)
        self._pos_spline = make_interp_spline(t[idx], xyz[idx], k=k)
        self._rot_spline = RotationSpline(t[idx], rot[idx])

    @classmethod
    def from_csv(cls, path, T_frame_child=None, knot_rate: float = 200.0) -> "Trajectory":
        t, xyz, q = load_csv(path)
        return cls(t, xyz, q, T_frame_child, knot_rate)

    @property
    def start(self) -> float:
        return float(self.t[0])

    @property
    def end(self) -> float:
        return float(self.t[-1])

    def _clip(self, times):
        return np.clip(np.asarray(times, dtype=float), self.t[0], self.t[-1])

    def position(self, times) -> np.ndarray:
        times = self._clip(times)
        return np.column_stack([np.interp(times, self.t, self.xyz[:, i]) for i in range(3)])

    def rotation(self, times) -> Rotation:
        return self._slerp(self._clip(times))

    def matrices(self, times) -> np.ndarray:
        times = np.atleast_1d(self._clip(times))
        T = np.tile(np.eye(4), (len(times), 1, 1))
        T[:, :3, :3] = self.rotation(times).as_matrix()
        T[:, :3, 3] = self.position(times)
        return T

    def velocity(self, times) -> np.ndarray:
        return self._pos_spline(self._clip(times), 1)

    def acceleration(self, times) -> np.ndarray:
        return self._pos_spline(self._clip(times), 2)

    def angular_velocity_body(self, times) -> np.ndarray:
        """Angular rate in the moving (body) frame, rad/s."""
        return self._rot_spline(self._clip(times), 1)
