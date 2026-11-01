"""Shared LiDAR ray casting with per-point motion distortion and noise (design §7).

Each ray is fired from the sensor pose at its own firing time, and the measured point is
expressed in the sensor frame *at that time*: exactly what a real scanner reports, so the
stack's deskewing is exercised.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .trajectory import Trajectory
from .world import World


@dataclass
class LidarNoise:
    range_sigma: float = 0.02      # m
    blind: float = 0.1             # m
    max_range: float = 70.0        # m (for reflectivity 1.0; scaled down for dark surfaces)
    dropout: float = 0.01          # base probability
    grazing_dropout: float = 0.6   # extra dropout at grazing incidence
    intensity_sigma: float = 4.0   # in 0..255 units

    @classmethod
    def ideal(cls) -> "LidarNoise":
        return cls(range_sigma=0.0, dropout=0.0, grazing_dropout=0.0, intensity_sigma=0.0)


@dataclass
class LidarReturns:
    points: np.ndarray     # (N, 3) sensor frame at firing time; NaN where no return
    intensity: np.ndarray  # (N,) 0..255 float
    valid: np.ndarray      # (N,) bool
    face_class: np.ndarray # (N,) uint8


def cast_scan(world: World, traj_lidar: Trajectory, dirs: np.ndarray, times: np.ndarray,
              noise: LidarNoise, rng: np.random.Generator) -> LidarReturns:
    """dirs: (N,3) unit rays in the sensor frame; times: (N,) run seconds."""
    T = traj_lidar.matrices(times)
    origins = T[:, :3, 3]
    dirs_w = np.einsum("nij,nj->ni", T[:, :3, :3], dirs)
    hits = world.cast(origins, dirs_w)

    dist = hits.distance
    cos_inc = np.abs(np.einsum("ni,ni->n", hits.normal, dirs_w))
    max_r = noise.max_range * np.sqrt(np.clip(hits.reflectivity, 0.05, 1.0))
    valid = np.isfinite(dist) & (dist >= noise.blind) & (dist <= max_r)

    if noise.dropout > 0 or noise.grazing_dropout > 0:
        p = noise.dropout + noise.grazing_dropout * (1.0 - np.clip(cos_inc, 0, 1)) ** 6
        p = p + 0.3 * np.clip(dist / np.maximum(max_r, 1e-6), 0, 1) ** 4
        valid &= rng.random(len(dist)) >= np.nan_to_num(p, nan=1.0)

    r = np.where(valid, dist, np.nan)
    if noise.range_sigma > 0:
        r = r + rng.normal(0.0, noise.range_sigma, len(r))
    points = dirs * r[:, None]

    intensity = 255.0 * hits.reflectivity * np.clip(cos_inc, 0.05, 1.0)
    if noise.intensity_sigma > 0:
        intensity = intensity + rng.normal(0.0, noise.intensity_sigma, len(intensity))
    intensity = np.where(valid, np.clip(intensity, 0, 255), 0.0)
    return LidarReturns(points, intensity, valid, np.where(valid, hits.face_class, 0))
