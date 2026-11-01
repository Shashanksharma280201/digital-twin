"""Livox Mid-360 simulation: official non-repetitive scan pattern, CustomMsg output.

Pattern: Livox-SDK/livox_laser_simulation scan_mode/mid360.csv (MIT), 800k rows of
(index, azimuth deg, zenith deg); stepped at 200 kHz, i.e. 20k points per 10 Hz frame,
``offset_time = j * 5000 ns``, ``line = i % 4``, ``tag = 0x10`` (as the Gazebo plugins do).
"""

from __future__ import annotations

import struct
import urllib.request
from pathlib import Path

import numpy as np

from .lidar import LidarNoise, LidarReturns, cast_scan
from .trajectory import Trajectory
from .world import World

PATTERN_URL = ("https://raw.githubusercontent.com/Livox-SDK/livox_laser_simulation/"
               "master/scan_mode/mid360.csv")
POINT_RATE = 200_000
FRAME_RATE = 10.0
POINTS_PER_FRAME = int(POINT_RATE / FRAME_RATE)
FRAME_ID = "livox_frame"

CUSTOM_POINT_DTYPE = np.dtype({
    "names": ["offset_time", "x", "y", "z", "reflectivity", "tag", "line"],
    "formats": ["<u4", "<f4", "<f4", "<f4", "u1", "u1", "u1"],
    "offsets": [0, 4, 8, 12, 16, 17, 18],
    "itemsize": 20,
})


def pattern_path(cache_dir: Path | None = None) -> Path:
    cache = Path(cache_dir or Path.home() / ".cache" / "dt_sim")
    path = cache / "mid360.csv"
    if not path.exists():
        cache.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".part")
        urllib.request.urlretrieve(PATTERN_URL, tmp)
        tmp.rename(path)
    return path


def load_pattern(path: Path | None = None) -> np.ndarray:
    """Unit ray directions (M, 3) in the Mid-360 frame."""
    src = Path(path) if path else pattern_path()
    npy = src.with_suffix(".dirs.npy")
    if npy.exists() and npy.stat().st_mtime >= src.stat().st_mtime:
        return np.load(npy)
    data = np.loadtxt(src, delimiter=",", skiprows=1, usecols=(1, 2), dtype=np.float64)
    az = np.radians(data[:, 0])
    el = np.radians(90.0 - data[:, 1])
    dirs = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    np.save(npy, dirs)
    return dirs


class Mid360:
    def __init__(self, world: World, traj_lidar: Trajectory, noise: LidarNoise | None = None,
                 pattern: np.ndarray | None = None, seed: int = 0):
        self.world = world
        self.traj = traj_lidar
        self.noise = noise or LidarNoise()
        self.pattern = load_pattern() if pattern is None else pattern
        self.rng = np.random.default_rng(seed)

    def frame(self, k: int, t_start: float) -> tuple[np.ndarray, LidarReturns, np.ndarray]:
        """Frame k starting at run time t_start -> (point indices, returns, offsets_ns)."""
        n = POINTS_PER_FRAME
        idx = (k * n + np.arange(n)) % len(self.pattern)
        offsets_ns = np.arange(n, dtype=np.int64) * (1_000_000_000 // POINT_RATE)
        times = t_start + offsets_ns * 1e-9
        returns = cast_scan(self.world, self.traj, self.pattern[idx], times, self.noise, self.rng)
        return idx, returns, offsets_ns


def pack_points(idx: np.ndarray, returns: LidarReturns, offsets_ns: np.ndarray) -> np.ndarray:
    pts = np.zeros(len(idx), CUSTOM_POINT_DTYPE)
    pts["offset_time"] = offsets_ns.astype(np.uint32)
    xyz = np.where(returns.valid[:, None], returns.points, 0.0)  # driver reports misses as 0,0,0
    pts["x"], pts["y"], pts["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    pts["reflectivity"] = np.round(returns.intensity).astype(np.uint8)
    pts["tag"] = 0x10
    pts["line"] = (idx % 4).astype(np.uint8)
    return pts


def _align(pos: int, n: int) -> int:
    return (pos + n - 1) // n * n


def custom_msg_cdr(stamp_ns: int, timebase_ns: int, points: np.ndarray,
                   frame_id: str = FRAME_ID, lidar_id: int = 0) -> bytes:
    """Serialise livox_ros_driver2/msg/CustomMsg (little-endian CDR) without per-point objects."""
    fid = frame_id.encode() + b"\x00"
    body = bytearray(struct.pack("<iII", stamp_ns // 1_000_000_000, stamp_ns % 1_000_000_000, len(fid)))
    body += fid
    body += b"\x00" * (_align(len(body), 8) - len(body))
    body += struct.pack("<QIB3x", timebase_ns, len(points), lidar_id)
    body += b"\x00" * (_align(len(body), 4) - len(body))
    body += struct.pack("<I", len(points))
    raw = points.tobytes()
    if len(points):
        raw = raw[:-1]  # the last element carries no trailing alignment pad
    return b"\x00\x01\x00\x00" + bytes(body) + raw
