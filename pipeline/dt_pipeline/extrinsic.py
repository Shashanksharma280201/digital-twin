"""Target-free LiDAR->camera extrinsic refinement by photometric consistency.

With the right ``T_lidar_optical`` a map point projects onto the same surface in every frame
that sees it, so its grey value barely changes across views; a wrong extrinsic smears each
point over neighbouring texture. The cost is the mean per-point variance of grey values across
frames, divided by the overall variance of the samples (so pushing points onto blank areas
does not pay). Searched coarse-to-fine over rotation then translation around the current guess.
Needs a good trajectory (``dtp refine``) and textured surfaces seen from several places.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from .colmap import project
from .geometry import make_T

log = logging.getLogger("dtp")


@dataclass
class View:
    grey: np.ndarray  # float32 HxW, lightly blurred
    T_world_lidar: np.ndarray


def perturb(T: np.ndarray, x: np.ndarray) -> np.ndarray:
    """``T`` moved by x = (rx, ry, rz [rad], tx, ty, tz [m]) in its own (optical) frame."""
    return T @ make_T(Rotation.from_rotvec(x[:3]).as_matrix(), x[3:])


def consistency(xyz: np.ndarray, views: list[View], K: np.ndarray, T_lidar_optical: np.ndarray,
                max_depth: float = 8.0, cell: int = 4, min_views: int = 3) -> float:
    n = len(xyz)
    s, ss, c = np.zeros(n), np.zeros(n), np.zeros(n)
    for v in views:
        h, w = v.grey.shape
        uv, z = project(xyz, v.T_world_lidar @ T_lidar_optical, K)
        ok = (z > 0.3) & (z < max_depth) & (uv[:, 0] >= 0) & (uv[:, 0] < w - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < h - 1)
        idx = np.flatnonzero(ok)
        u, vv, d = uv[idx, 0], uv[idx, 1], z[idx]
        gw = (w + cell - 1) // cell
        cells = (vv.astype(int) // cell) * gw + (u.astype(int) // cell)
        zbuf = np.full(gw * ((h + cell - 1) // cell), np.inf)
        np.minimum.at(zbuf, cells, d)
        vis = d <= zbuf[cells] * 1.03 + 0.05
        idx, u, vv = idx[vis], u[vis], vv[vis]
        u0, v0 = u.astype(int), vv.astype(int)
        fu, fv = u - u0, vv - v0
        g = v.grey
        val = ((1 - fu) * (1 - fv) * g[v0, u0] + fu * (1 - fv) * g[v0, u0 + 1]
               + (1 - fu) * fv * g[v0 + 1, u0] + fu * fv * g[v0 + 1, u0 + 1])
        s[idx] += val
        ss[idx] += val * val
        c[idx] += 1
    m = c >= min_views
    if m.sum() < 1000:
        return 1.0
    mean = s[m] / c[m]
    within = np.mean(ss[m] / c[m] - mean ** 2)
    total = np.sum(ss[m]) / np.sum(c[m]) - (np.sum(s[m]) / np.sum(c[m])) ** 2
    return float(within / max(total, 1e-9))


def refine_extrinsic(xyz: np.ndarray, views: list[View], K: np.ndarray, T0: np.ndarray,
                     rot_range_deg: float = 3.0, trans_range_m: float = 0.05) -> tuple[np.ndarray, dict]:
    """Returns (refined T_lidar_optical, report)."""
    def cost(x):
        return consistency(xyz, views, K, perturb(T0, x))

    c0 = cost(np.zeros(6))
    # Coarse grid on rotation (the dominant term), then a joint local search.
    best = (c0, np.zeros(6))
    steps = np.radians(np.linspace(-rot_range_deg, rot_range_deg, 7))
    for axis in range(3):
        for a in steps:
            x = best[1].copy()
            x[axis] = a
            c = cost(x)
            if c < best[0]:
                best = (c, x)
    scale = np.array([np.radians(rot_range_deg)] * 3 + [trans_range_m] * 3)
    res = minimize(lambda y: cost(y * scale), best[1] / scale, method="Powell",
                   options={"xtol": 0.02, "ftol": 1e-4, "maxfev": 400})
    x = res.x * scale
    report = {"cost_before": round(c0, 4), "cost_after": round(float(res.fun), 4),
              "rotation_deg_optical": np.degrees(x[:3]).round(2).tolist(), "translation_m_optical": x[3:].round(3).tolist(),
              "evaluations": int(res.nfev) + 21}
    return perturb(T0, x), report
