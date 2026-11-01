"""Deskewed map accumulation: canonical scans + ``T_world_lidar(t)`` → voxelised cloud."""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass

import numpy as np

from .geometry import Trajectory
from .io import Scan


def voxel_keys(xyz: np.ndarray, voxel: float) -> np.ndarray:
    """Integer voxel coordinates packed into one int64 per point (21 bits per axis)."""
    ijk = np.floor(xyz / voxel).astype(np.int64) + (1 << 20)
    return (ijk[:, 0] << 42) | (ijk[:, 1] << 21) | ijk[:, 2]


def keys_to_centers(keys: np.ndarray, voxel: float) -> np.ndarray:
    mask = (1 << 21) - 1
    ijk = np.column_stack([(keys >> 42) & mask, (keys >> 21) & mask, keys & mask]) - (1 << 20)
    return (ijk + 0.5) * voxel


@dataclass
class VoxelCloud:
    """Running per-voxel centroid of points and attributes."""

    voxel: float
    keys: np.ndarray = None
    sums: np.ndarray = None  # columns: x, y, z, intensity
    counts: np.ndarray = None

    def __post_init__(self):
        self.keys = np.empty(0, np.int64)
        self.sums = np.empty((0, 4))
        self.counts = np.empty(0, np.int64)

    def add(self, xyz: np.ndarray, intensity: np.ndarray) -> None:
        keys = np.concatenate([self.keys, voxel_keys(xyz, self.voxel)])
        vals = np.concatenate([self.sums, np.column_stack([xyz, intensity])])
        counts = np.concatenate([self.counts, np.ones(len(xyz), np.int64)])
        uniq, inverse = np.unique(keys, return_inverse=True)
        sums = np.zeros((len(uniq), 4))
        np.add.at(sums, inverse, vals)
        self.keys, self.sums = uniq, sums
        self.counts = np.bincount(inverse, weights=counts, minlength=len(uniq)).astype(np.int64)

    def result(self, min_points: int = 1) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        keep = self.counts >= min_points
        mean = self.sums[keep] / self.counts[keep, None]
        return mean[:, :3], mean[:, 3].astype(np.float32), self.counts[keep]


def voxel_downsample(xyz: np.ndarray, voxel: float, attrs: np.ndarray | None = None):
    vc = VoxelCloud(voxel)
    vc.add(xyz, attrs if attrs is not None else np.zeros(len(xyz)))
    pts, att, _ = vc.result()
    return (pts, att) if attrs is not None else pts


@dataclass
class WorldScan:
    stamp: float
    xyz: np.ndarray  # world frame
    intensity: np.ndarray
    origin: np.ndarray  # sensor position at scan midpoint
    pose: np.ndarray  # T_world_lidar at scan midpoint


def scans_to_world(scans: Iterable[Scan], traj: Trajectory, deskew: bool = True,
                   min_range: float = 0.3, max_range: float = 60.0, every: int = 1) -> Iterator[WorldScan]:
    """Transform scans into the world frame; points outside the trajectory span are dropped."""
    for i, scan in enumerate(scans):
        if i % every:
            continue
        r = np.linalg.norm(scan.xyz, axis=1)
        t = scan.abs_time
        keep = (r >= min_range) & (r <= max_range) & traj.covers(t)
        if not keep.any():
            continue
        xyz, t = scan.xyz[keep], t[keep]
        t_mid = float(np.clip(scan.stamp + scan.time[keep].max() / 2, traj.start, traj.end))
        pose = traj.interpolate(t_mid)[0]
        if deskew:
            world = traj.transform_points_at(xyz, t)
        else:
            world = xyz @ pose[:3, :3].T + pose[:3, 3]
        yield WorldScan(t_mid, world, scan.intensity[keep], pose[:3, 3].copy(), pose)


def build_map(world_scans: Iterable[WorldScan], voxel: float = 0.05, min_points: int = 1,
              flush_every: int = 20) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Accumulate world scans into a voxel-centroid map. Returns ``(xyz, intensity, counts)``."""
    vc = VoxelCloud(voxel)
    buf_xyz, buf_int = [], []
    for k, ws in enumerate(world_scans, 1):
        buf_xyz.append(ws.xyz)
        buf_int.append(ws.intensity)
        if k % flush_every == 0:
            vc.add(np.concatenate(buf_xyz), np.concatenate(buf_int))
            buf_xyz, buf_int = [], []
    if buf_xyz:
        vc.add(np.concatenate(buf_xyz), np.concatenate(buf_int))
    return vc.result(min_points)
