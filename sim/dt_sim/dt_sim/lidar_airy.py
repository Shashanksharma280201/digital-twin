"""RoboSense Airy (96-beam hemispherical) simulation, rslidar_sdk XYZIRT PointCloud2 output.

Assumptions (the real vertical angles come per unit from DIFOP calibration):
- 96 rings uniformly spaced from ``elev_min`` = 0 deg to ``elev_max`` = 90 deg above the
  sensor XY plane (manual: 0-90 deg, ~0.947 deg spacing).
- 0.4 deg horizontal resolution at 10 Hz -> 900 columns x 96 rings = 86,400 points/frame.
- One column every 0.1 s / 900 = 111.1 us (block duration 111.08 us in rs_driver); rings
  inside a column fire evenly across the column period.
Output matches rslidar_sdk built with POINT_TYPE XYZIRT: x, y, z, intensity (f32), ring (u16),
timestamp (f64, absolute seconds), point_step 26, height 1, NaN for no return
(dense_points: false), header stamp = first point (ts_first_point: true).
"""

from __future__ import annotations

import numpy as np

from .lidar import LidarNoise, LidarReturns, cast_scan
from .trajectory import Trajectory
from .world import World

FRAME_RATE = 10.0
FRAME_ID = "rslidar"
RINGS = 96

XYZIRT_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "ring", "timestamp"],
    "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f8"],
    "offsets": [0, 4, 8, 12, 16, 18],
    "itemsize": 26,
})
FIELDS = [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4),
          ("timestamp", 18, 8)]


class Airy:
    def __init__(self, world: World, traj_lidar: Trajectory, noise: LidarNoise | None = None,
                 h_res_deg: float = 0.4, elev_min: float = 0.0, elev_max: float = 90.0, seed: int = 0,
                 ring_bias_mm: float = 0.0):
        self.world = world
        self.traj = traj_lidar
        self.noise = noise or LidarNoise(range_sigma=0.01)
        self.rng = np.random.default_rng(seed)
        self.columns = int(round(360.0 / h_res_deg))
        az = np.radians(np.arange(self.columns) * h_res_deg)
        el = np.radians(np.linspace(elev_min, elev_max, RINGS))
        A, E = np.meshgrid(az, el, indexing="ij")  # column-major firing order
        self.dirs = np.column_stack([
            (np.cos(E) * np.cos(A)).ravel(), (np.cos(E) * np.sin(A)).ravel(), np.sin(E).ravel()])
        self.ring = np.tile(np.arange(RINGS, dtype=np.uint16), self.columns)
        col_period = 1.0 / FRAME_RATE / self.columns
        col = np.repeat(np.arange(self.columns), RINGS)
        self.offsets = col * col_period + self.ring * (col_period / RINGS)
        # Per-ring range error, as real units show (none by default); the truth is kept for scoring.
        self.ring_bias = np.random.default_rng(seed + 1).normal(0.0, ring_bias_mm / 1000.0, RINGS)

    @property
    def points_per_frame(self) -> int:
        return len(self.dirs)

    def frame(self, t_start: float) -> tuple[LidarReturns, np.ndarray]:
        times = t_start + self.offsets
        ret = cast_scan(self.world, self.traj, self.dirs, times, self.noise, self.rng)
        ret.points += self.dirs * self.ring_bias[self.ring, None]
        return ret, times


def pack_points(returns: LidarReturns, ring: np.ndarray, abs_times: np.ndarray) -> np.ndarray:
    pts = np.zeros(len(ring), XYZIRT_DTYPE)
    pts["x"], pts["y"], pts["z"] = (returns.points[:, i].astype(np.float32) for i in range(3))
    pts["intensity"] = returns.intensity.astype(np.float32)
    pts["ring"] = ring
    pts["timestamp"] = abs_times
    return pts
