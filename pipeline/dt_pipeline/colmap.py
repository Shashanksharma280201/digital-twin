"""COLMAP text-model export for Gaussian splatting with known poses (gsplat / nerfstudio).

Layout written to ``out_dir``::

    images/<name>.jpg
    sparse/0/cameras.txt   one PINHOLE camera from CameraInfo (rectified images)
    sparse/0/images.txt    world->camera (OpenCV axes): QW QX QY QZ TX TY TZ, empty POINTS2D line
    sparse/0/points3D.txt  coloured LiDAR map points with empty tracks

Camera pose: ``T_world_cam(t) = T_world_lidar(t + camera_time_offset) @ T_lidar_optical``.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import Trajectory, inv
from .io import CameraInfo


@dataclass
class ExportedImage:
    image_id: int
    name: str
    stamp: float
    T_world_cam: np.ndarray


def project(xyz_world: np.ndarray, T_world_cam: np.ndarray, K: np.ndarray):
    """Pixel coords and depth of world points in an OpenCV camera."""
    T_cw = inv(T_world_cam)
    pc = xyz_world @ T_cw[:3, :3].T + T_cw[:3, 3]
    z = pc[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = (pc[:, :2] / z[:, None]) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    return uv, z


def select_frames(stamps: np.ndarray, traj: Trajectory, T_lidar_optical: np.ndarray, time_offset: float,
                  every: int = 1, min_translation: float = 0.0, min_rotation_deg: float = 0.0) -> list[int]:
    """Indices of images inside the trajectory, decimated by count and/or motion."""
    t = stamps + time_offset
    idx = [i for i in np.flatnonzero(traj.covers(t)) if i % every == 0]
    if not idx or (min_translation <= 0 and min_rotation_deg <= 0):
        return idx
    poses = traj.interpolate(t[idx]) @ T_lidar_optical
    keep, last = [idx[0]], poses[0]
    for i, P in zip(idx[1:], poses[1:]):
        dt = np.linalg.norm(P[:3, 3] - last[:3, 3])
        dr = np.degrees(Rotation.from_matrix(last[:3, :3].T @ P[:3, :3]).magnitude())
        if dt >= min_translation or dr >= min_rotation_deg:
            keep.append(i)
            last = P
    return keep


class ColorAccumulator:
    """Occlusion-aware point colouring from posed, rectified images.

    For each image a coarse z-buffer is built from the map points themselves (cells of
    ``cell`` pixels); a point takes colour only if it is within ``rel_tol * depth + abs_tol``
    of the nearest point in its cell, so walls behind columns are not painted with the column.
    Views are weighted by ``(sharpness * centrality / depth^2) ** sharpen``: the sharpest, closest,
    most central views dominate instead of being averaged with blurred or grazing ones.
    ``add(..., sharpness=...)`` takes a per-image score relative to the run (1 = typical).
    """

    def __init__(self, xyz: np.ndarray, K: np.ndarray, max_depth: float = 15.0, cell: int = 4,
                 rel_tol: float = 0.03, abs_tol: float = 0.05, sharpen: float = 1.0):
        self.xyz, self.K, self.sharpen = xyz, K, sharpen
        self.max_depth, self.cell, self.rel_tol, self.abs_tol = max_depth, cell, rel_tol, abs_tol
        self.acc = np.zeros((len(xyz), 3))
        self.weight = np.zeros(len(xyz))

    def add(self, bgr: np.ndarray, T_world_cam: np.ndarray, sharpness: float = 1.0) -> int:
        h, w = bgr.shape[:2]
        uv, z = project(self.xyz, T_world_cam, self.K)
        ok = (z > 0.1) & (z < self.max_depth) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        idx = np.flatnonzero(ok)
        if len(idx) == 0:
            return 0
        u, v, d = uv[idx, 0].astype(int), uv[idx, 1].astype(int), z[idx]
        gw = (w + self.cell - 1) // self.cell
        cells = (v // self.cell) * gw + (u // self.cell)
        zbuf = np.full(gw * ((h + self.cell - 1) // self.cell), np.inf)
        np.minimum.at(zbuf, cells, d)
        vis = d <= zbuf[cells] * (1 + self.rel_tol) + self.abs_tol
        idx, u, v, d = idx[vis], u[vis], v[vis], d[vis]
        r2 = ((u - w / 2) / (w / 2)) ** 2 + ((v - h / 2) / (h / 2)) ** 2
        centre = np.clip(1.0 - 0.4 * r2, 0.2, 1.0)
        wgt = (sharpness * centre / np.maximum(d, 0.5) ** 2) ** self.sharpen
        self.acc[idx] += bgr[v, u, ::-1] * wgt[:, None]
        self.weight[idx] += wgt
        return len(idx)

    def result(self, unseen=(128, 128, 128)) -> tuple[np.ndarray, np.ndarray]:
        """(rgb uint8 Nx3, seen mask)."""
        seen = self.weight > 0
        rgb = np.tile(np.asarray(unseen, float), (len(self.xyz), 1))
        rgb[seen] = self.acc[seen] / self.weight[seen, None]
        return np.clip(rgb, 0, 255).astype(np.uint8), seen


def colorize(xyz: np.ndarray, images: list[tuple[np.ndarray, np.ndarray]], K: np.ndarray,
             max_depth: float = 15.0) -> np.ndarray:
    """RGB per point from the images that see it (occlusion-aware). Unseen → grey."""
    acc = ColorAccumulator(xyz, K, max_depth)
    for bgr, T_world_cam in images:
        acc.add(bgr, T_world_cam)
    return acc.result()[0]


def write_model(out_dir: str | Path, cam: CameraInfo, images: Iterable[ExportedImage],
                xyz: np.ndarray, rgb: np.ndarray) -> Path:
    sparse = Path(out_dir) / "sparse" / "0"
    sparse.mkdir(parents=True, exist_ok=True)
    K = cam.K
    (sparse / "cameras.txt").write_text(
        "# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n"
        f"1 PINHOLE {cam.width} {cam.height} {K[0, 0]:.6f} {K[1, 1]:.6f} {K[0, 2]:.6f} {K[1, 2]:.6f}\n"
    )
    lines = ["# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME", "# POINTS2D[] as (X, Y, POINT3D_ID)"]
    for im in images:
        T_cw = inv(im.T_world_cam)
        qx, qy, qz, qw = Rotation.from_matrix(T_cw[:3, :3]).as_quat()
        tx, ty, tz = T_cw[:3, 3]
        lines.append(f"{im.image_id} {qw:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {tx:.6f} {ty:.6f} {tz:.6f} 1 {im.name}")
        lines.append("")
    (sparse / "images.txt").write_text("\n".join(lines) + "\n")
    with open(sparse / "points3D.txt", "w") as f:
        f.write("# POINT3D_ID X Y Z R G B ERROR TRACK[]\n")
        for i, (p, c) in enumerate(zip(xyz, rgb), 1):
            f.write(f"{i} {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {c[0]} {c[1]} {c[2]} 0\n")
    return sparse


def read_images_txt(path: str | Path) -> dict[str, np.ndarray]:
    """Parse images.txt back into ``{name: T_world_cam}`` (used by tests and sanity checks)."""
    out = {}
    lines = [ln for ln in Path(path).read_text().splitlines() if not ln.startswith("#")]
    for ln in lines[::2]:
        parts = ln.split()
        if len(parts) < 10:
            continue
        qw, qx, qy, qz, tx, ty, tz = map(float, parts[1:8])
        T_cw = np.eye(4)
        T_cw[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
        T_cw[:3, 3] = [tx, ty, tz]
        out[parts[9]] = inv(T_cw)
    return out


def sharpness(bgr: np.ndarray, size: int = 480) -> float:
    """Variance of the Laplacian on a downscaled grey image (motion blur lowers it)."""
    import cv2

    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    f = size / max(g.shape)
    if f < 1:
        g = cv2.resize(g, None, fx=f, fy=f, interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(g, cv2.CV_32F).var())
