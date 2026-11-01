"""Dynamic-object removal: split a map into static and dynamic parts.

``method="dufomap"`` uses KTH-RPL DUFOMap when installed (x86 wheel; build from source on
Jetson). ``method="visibility"`` is a numpy fallback: a voxel is dynamic when other scans see
through it (range-image visibility) at least as often as they hit it.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from .mapping import WorldScan, keys_to_centers, voxel_keys


def dufomap_available() -> bool:
    try:
        import dufomap  # noqa: F401
    except ImportError:
        return False
    return True


def _window_min(img: np.ndarray, r: int) -> np.ndarray:
    """Min over a (2r+1)^2 window, azimuth wraps; empty pixels (inf) are ignored."""
    out = img.copy()
    padded = np.pad(img, ((r, r), (0, 0)), constant_values=np.inf)
    for di in range(-r, r + 1):
        rows = padded[r + di : r + di + img.shape[0]]
        for dj in range(-r, r + 1):
            out = np.minimum(out, np.roll(rows, dj, axis=1))
    return out


def visibility(scans: Sequence[WorldScan], voxel: float = 0.1, az_res_deg: float = 1.0,
               el_res_deg: float = 2.0, window: int = 1, margin: float = 0.2, margin_ratio: float = 0.05,
               max_range: float = 40.0, free_ratio: float = 1.0, min_free: int = 2) -> tuple[np.ndarray, np.ndarray]:
    """Range-image visibility test (Removert-style). Returns ``(static_keys, dynamic_keys)``.

    An occupied voxel gets a "seen-through" vote from a scan when every return in the
    window around its direction lies clearly behind it; the window minimum keeps
    grazing surfaces such as floors from voting against themselves.
    """
    hit_keys = np.concatenate([np.unique(voxel_keys(s.xyz, voxel)) for s in scans])
    occ, hits = np.unique(hit_keys, return_counts=True)
    centers = keys_to_centers(occ, voxel)
    n_az, n_el = int(round(360 / az_res_deg)), int(round(180 / el_res_deg))
    free = np.zeros(len(occ), np.int64)

    def pixels(v):
        r = np.linalg.norm(v, axis=1)
        az = np.degrees(np.arctan2(v[:, 1], v[:, 0])) % 360
        el = np.degrees(np.arcsin(np.clip(v[:, 2] / np.maximum(r, 1e-9), -1, 1))) + 90
        return r, np.minimum((el / el_res_deg).astype(int), n_el - 1), np.minimum((az / az_res_deg).astype(int), n_az - 1)

    for s in scans:
        R, o = s.pose[:3, :3], s.pose[:3, 3]
        r, i, j = pixels((s.xyz - o) @ R)
        img = np.full((n_el, n_az), np.inf)
        np.minimum.at(img, (i, j), r)
        wmin = _window_min(img, window)
        rc, ic, jc = pixels((centers - o) @ R)
        behind = wmin[ic, jc]
        free += (rc < max_range) & np.isfinite(behind) & (behind > rc + np.maximum(margin, margin_ratio * rc))
    dynamic = (free >= min_free) & (free >= free_ratio * hits)
    return occ[~dynamic], occ[dynamic]


def split_points(xyz: np.ndarray, dynamic_keys: np.ndarray, voxel: float) -> tuple[np.ndarray, np.ndarray]:
    """Boolean masks ``(static, dynamic)`` for map points given dynamic voxel keys."""
    is_dyn = np.isin(voxel_keys(xyz, voxel), dynamic_keys)
    return ~is_dyn, is_dyn


def dufomap_dynamic_keys(scans: Sequence[WorldScan], voxel: float = 0.1, out_file: str = "dufomap_out.pcd",
                         inflate_hits: float = 0.2, inflate_unknown: int = 2) -> np.ndarray:
    """Run DUFOMap and return keys of voxels it removed. [unverified API, see research/07]"""
    from dufomap import dufomap
    from scipy.spatial.transform import Rotation

    m = dufomap(voxel, inflate_hits, inflate_unknown)
    for s in scans:
        qx, qy, qz, qw = Rotation.from_matrix(s.pose[:3, :3]).as_quat()
        pose7 = np.array([*s.pose[:3, 3], qw, qx, qy, qz])
        m.run(s.xyz.astype(np.float32), pose7, cloud_transform=False)
    m.oncePropagateCluster(if_propagate=True, if_cluster=False)
    all_pts = np.concatenate([s.xyz for s in scans]).astype(np.float32)
    m.outputMap(all_pts, voxel_map=False)
    import open3d as o3d

    kept = np.asarray(o3d.io.read_point_cloud(out_file).points)
    all_keys = np.unique(voxel_keys(all_pts.astype(np.float64), voxel))
    return np.setdiff1d(all_keys, voxel_keys(kept, voxel))


def dynamic_voxels(scans: Sequence[WorldScan], voxel: float = 0.1, method: str = "auto") -> np.ndarray:
    if method == "auto":
        method = "dufomap" if dufomap_available() else "visibility"
    if method == "dufomap":
        return dufomap_dynamic_keys(scans, voxel)
    if method == "visibility":
        return visibility(scans, voxel)[1]
    raise ValueError(f"unknown clean method {method!r}")


__all__ = ["visibility", "dufomap_available", "dynamic_voxels", "keys_to_centers", "split_points"]
