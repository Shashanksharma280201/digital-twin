"""Per-beam LiDAR self-calibration: learn each ring's range and elevation error from a recording.

Every laser ring of a multi-beam LiDAR has its own small range and angle error, so each ring draws
the same wall in a slightly different place and the fused map gets thick. No target is needed: the
TSDF mesh averages all rings, so a ring's mean signed distance to the mesh is its error. For ring
``k`` with beam direction ``u`` and elevation tangent ``e`` (both in the sensor frame), a point at
range ``r`` is corrected by ``dr[k] * u + r * de[k] * e``. Both offsets are fitted per ring by
robust least squares against the surface normals.

The mesh only knows the rings relative to each other, so the mean offset is removed: a shared range
or elevation error is not observable here and must not leak into the map's scale.
"""

from __future__ import annotations

import json
import logging
from collections.abc import Iterable
from pathlib import Path

import numpy as np

from .geometry import Trajectory
from .io import Scan

log = logging.getLogger("dtp")

CALIB_NAME = "ring_calib.json"


def beam_axes(xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``(range, unit beam u, unit elevation tangent e)``; ``e`` is 0 where elevation is undefined (zenith)."""
    r = np.linalg.norm(xyz, axis=1)
    u = xyz / np.maximum(r, 1e-9)[:, None]
    h = np.hypot(u[:, 0], u[:, 1])
    safe = np.maximum(h, 1e-9)
    e = np.column_stack([-u[:, 2] * u[:, 0] / safe, -u[:, 2] * u[:, 1] / safe, h])
    e[h < 1e-3] = 0.0
    return r, u, e


def apply(xyz: np.ndarray, ring: np.ndarray, range_m: np.ndarray, elev_rad: np.ndarray) -> np.ndarray:
    """Corrected sensor-frame points. Rings beyond the calibration are left unchanged."""
    r, u, e = beam_axes(xyz)
    n = len(range_m)
    known = ring < n
    k = np.minimum(ring, n - 1).astype(np.int64)
    dr = np.where(known, range_m[k], 0.0)
    de = np.where(known, elev_rad[k], 0.0)
    return xyz + u * dr[:, None] + e * (r * de)[:, None]


def solve(ring: np.ndarray, a: np.ndarray, b: np.ndarray, d: np.ndarray, rings: int, min_points: int = 2000,
          huber: float = 0.02, iterations: int = 4) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Robust per-ring fit of ``a * dr + b * de = -d``; returns ``(dr, de, points per ring)``."""
    dr, de, counts = np.zeros(rings), np.zeros(rings), np.bincount(ring, minlength=rings)[:rings]
    for k in np.flatnonzero(counts >= min_points):
        m = ring == k
        ak, bk, dk = a[m], b[m], d[m]
        x = np.zeros(2)
        for _ in range(iterations):
            res = ak * x[0] + bk * x[1] + dk
            w = huber / np.maximum(np.abs(res), huber)
            A = np.array([[(w * ak * ak).sum(), (w * ak * bk).sum()], [(w * ak * bk).sum(), (w * bk * bk).sum()]])
            # A small ridge keeps rings with one viewing geometry from trading range against elevation.
            x = np.linalg.solve(A + np.eye(2) * 1e-6 * max(np.trace(A), 1e-9), -np.array([(w * ak * dk).sum(), (w * bk * dk).sum()]))
        dr[k], de[k] = x
    used = counts >= min_points
    if used.any():
        dr[used] -= np.average(dr[used], weights=counts[used])
        de[used] -= np.average(de[used], weights=counts[used])
    return dr, de, counts


def estimate(scans: Iterable[Scan], tr: Trajectory, mesh, rings: int = 96, min_range: float = 0.8,
             max_range: float = 15.0, max_residual: float = 0.1, step: int = 3) -> dict:
    """Fit the ring offsets of ``scans`` against the fused open3d ``mesh``."""
    import open3d as o3d

    mesh.compute_triangle_normals()
    normals = np.asarray(mesh.triangle_normals)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    parts = []
    for sc in scans:
        if sc.ring is None:
            raise ValueError("ring calibration needs a ring field in the scans")
        r = np.linalg.norm(sc.xyz, axis=1)
        idx = np.flatnonzero((r > min_range) & (r < max_range) & tr.covers(sc.abs_time))[::step]
        if len(idx) < 100:
            continue
        P = tr.interpolate(sc.abs_time[idx])
        R, t = P[:, :3, :3], P[:, :3, 3]
        world = np.einsum("nij,nj->ni", R, sc.xyz[idx]) + t
        ans = scene.compute_closest_points(o3d.core.Tensor(world.astype(np.float32)))
        n = normals[ans["primitive_ids"].numpy()]
        d = ((world - ans["points"].numpy().astype(np.float64)) * n).sum(1)
        keep = np.abs(d) < max_residual
        n_s = np.einsum("nji,nj->ni", R, n)  # normal in the sensor frame
        rng, u, e = beam_axes(sc.xyz[idx])
        a, b = (n_s * u).sum(1), rng * (n_s * e).sum(1)
        parts.append(np.column_stack([sc.ring[idx], a, b, d])[keep])
    if not parts:
        raise ValueError("no scan points near the mesh")
    data = np.concatenate(parts)
    ring = data[:, 0].astype(np.int64)
    if ring.max() >= rings:
        rings = int(ring.max()) + 1
    dr, de, counts = solve(ring, data[:, 1], data[:, 2], data[:, 3], rings)
    return {"range_m": dr.tolist(), "elev_rad": de.tolist(), "points": counts.tolist()}


def save(path: str | Path, calib: dict) -> None:
    Path(path).write_text(json.dumps(calib))


def load(path: str | Path) -> tuple[np.ndarray, np.ndarray] | None:
    p = Path(path)
    if not p.exists():
        return None
    c = json.loads(p.read_text())
    return np.asarray(c["range_m"]), np.asarray(c["elev_rad"])
