"""Change detection between two aligned maps.

* Voxel occupancy diff: a voxel is *added* when the current map occupies it and the
  reference has nothing within one voxel (26-neighbourhood), *removed* symmetrically.
  Note: "removed" also covers areas the current capture never observed; restrict
  interpretation to the overlap of both captures.
* Optional M3C2 (py4dgeo) for significance-aware surface displacement.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from .mapping import voxel_keys

_OFFSETS = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)], dtype=np.int64)
_PACK = np.array([1 << 42, 1 << 21, 1], dtype=np.int64)


def dilate_keys(keys: np.ndarray) -> np.ndarray:
    return np.unique((keys[:, None] + (_OFFSETS @ _PACK)[None, :]).ravel())


def _drop_isolated(keys: np.ndarray, min_neighbours: int) -> np.ndarray:
    if min_neighbours <= 0 or len(keys) == 0:
        return keys
    neigh = (keys[:, None] + (_OFFSETS @ _PACK)[None, :]).ravel()
    counts = np.isin(neigh, keys).reshape(len(keys), -1).sum(axis=1) - 1
    return keys[counts >= min_neighbours]


@dataclass
class Cluster:
    centroid: list
    bbox_min: list
    bbox_max: list
    n_points: int


@dataclass
class DiffResult:
    voxel: float
    added_mask: np.ndarray = field(repr=False)  # over current points
    removed_mask: np.ndarray = field(repr=False)  # over reference points
    added_clusters: list[Cluster] = field(default_factory=list)
    removed_clusters: list[Cluster] = field(default_factory=list)

    def summary(self) -> dict:
        v3 = self.voxel ** 3
        return {
            "voxel": self.voxel,
            "added_points": int(self.added_mask.sum()),
            "removed_points": int(self.removed_mask.sum()),
            "added_clusters": [asdict(c) for c in self.added_clusters],
            "removed_clusters": [asdict(c) for c in self.removed_clusters],
            "approx_added_volume_m3_per_voxel": v3,
        }


def clusters(xyz: np.ndarray, eps: float, min_points: int) -> list[Cluster]:
    if len(xyz) < min_points:
        return []
    import open3d as o3d

    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    labels = np.asarray(pc.cluster_dbscan(eps=eps, min_points=min_points))
    out = []
    for lab in range(labels.max() + 1):
        p = xyz[labels == lab]
        out.append(Cluster(p.mean(0).tolist(), p.min(0).tolist(), p.max(0).tolist(), int(len(p))))
    return sorted(out, key=lambda c: -c.n_points)


def voxel_diff(ref: np.ndarray, cur: np.ndarray, voxel: float = 0.05, min_neighbours: int = 2,
               cluster_min_points: int = 20) -> DiffResult:
    rk, ck = voxel_keys(ref, voxel), voxel_keys(cur, voxel)
    ru, cu = np.unique(rk), np.unique(ck)
    added = _drop_isolated(np.setdiff1d(cu, dilate_keys(ru)), min_neighbours)
    removed = _drop_isolated(np.setdiff1d(ru, dilate_keys(cu)), min_neighbours)
    res = DiffResult(voxel, np.isin(ck, added), np.isin(rk, removed))
    res.added_clusters = clusters(cur[res.added_mask], 3 * voxel, cluster_min_points)
    res.removed_clusters = clusters(ref[res.removed_mask], 3 * voxel, cluster_min_points)
    return res


def m3c2(ref: np.ndarray, cur: np.ndarray, core_every: int = 20, cyl_radius: float = 0.2,
         normal_radii=(0.1, 0.2, 0.4)):
    """Return ``(corepoints, distance, lodetection)``. Requires py4dgeo (``pip install .[full]``)."""
    import py4dgeo

    core = ref[::core_every]
    m = py4dgeo.M3C2(epochs=(py4dgeo.Epoch(ref), py4dgeo.Epoch(cur)), corepoints=core,
                     cyl_radius=cyl_radius, normal_radii=list(normal_radii))
    dist, unc = m.run()
    return core, dist, unc["lodetection"]


def write_summary(path: str | Path, summary: dict) -> None:
    Path(path).write_text(json.dumps(summary, indent=2))
