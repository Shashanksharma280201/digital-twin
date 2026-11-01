"""GPU (PyTorch) kernels for the heavy, per-point steps: deskew, voxel map, TSDF fusion, projection.

Runs on CUDA when available (laptop RTX, Orin), else on the CPU with the same code. Times are
kept relative to the trajectory start so float32 holds them to ~30 us over hours.
"""

from __future__ import annotations

import logging

import numpy as np
import torch

from .geometry import Trajectory

log = logging.getLogger("dtp")


def device() -> torch.device:
    return torch.device("cuda" if torch.cuda.is_available() else "cpu")


def _quat_from_matrix(R: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    q = Rotation.from_matrix(R).as_quat()  # x, y, z, w
    # Keep consecutive quaternions in one hemisphere so linear blending is valid.
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    return q


def quat_to_matrix(q: torch.Tensor) -> torch.Tensor:
    x, y, z, w = q.unbind(-1)
    return torch.stack([
        1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
        2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
        2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
    ], -1).reshape(*q.shape[:-1], 3, 3)


class GpuTrajectory:
    """``T_world_lidar(t)`` on the device: linear position, normalised-linear quaternion blend."""

    def __init__(self, tr: Trajectory, dev: torch.device | None = None):
        self.dev = dev or device()
        self.t0 = float(tr.stamps[0])
        self.start, self.end = tr.start, tr.end
        self.t = torch.as_tensor(tr.stamps - self.t0, dtype=torch.float32, device=self.dev)
        self.p = torch.as_tensor(tr.poses[:, :3, 3], dtype=torch.float32, device=self.dev)
        self.q = torch.as_tensor(_quat_from_matrix(tr.poses[:, :3, :3]), dtype=torch.float32, device=self.dev)

    def rel(self, t_abs: np.ndarray) -> torch.Tensor:
        return torch.as_tensor(np.asarray(t_abs, dtype=np.float64) - self.t0, dtype=torch.float32, device=self.dev)

    def at(self, t: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """(R (N,3,3), p (N,3)) at relative times ``t`` (clamped to the span)."""
        t = t.clamp(float(self.t[0]), float(self.t[-1]))
        i = torch.searchsorted(self.t, t).clamp(1, len(self.t) - 1)
        t0, t1 = self.t[i - 1], self.t[i]
        a = ((t - t0) / (t1 - t0).clamp_min(1e-9)).unsqueeze(-1)
        p = self.p[i - 1] * (1 - a) + self.p[i] * a
        q = self.q[i - 1] * (1 - a) + self.q[i] * a
        q = q / q.norm(dim=-1, keepdim=True)
        return quat_to_matrix(q), p


def to_world(gt: GpuTrajectory, stamps: np.ndarray, pts: np.ndarray, deskew: bool = True):
    """Cache batch -> (world xyz, sensor origin per point, intensity) tensors on the device.

    ``deskew`` uses each point's own time; otherwise the scan's mid time for all its points.
    """
    P = torch.as_tensor(pts, device=gt.dev)
    t = gt.rel(stamps) + P[:, 3]
    if not deskew:
        t = gt.rel(stamps)
    R, p = gt.at(t)
    xyz = torch.einsum("nij,nj->ni", R, P[:, :3]) + p
    return xyz, p, P[:, 4]


def voxel_keys(xyz: torch.Tensor, voxel: float) -> torch.Tensor:
    ijk = torch.floor(xyz / voxel).to(torch.int64) + (1 << 20)
    return (ijk[:, 0] << 42) | (ijk[:, 1] << 21) | ijk[:, 2]


def downsample_per_scan(xyz: torch.Tensor, scan_id: torch.Tensor, voxel: float):
    """Voxel centroids computed separately for each scan of a batch -> (xyz, scan_id) tensors.

    Keys pack 19 bits per axis (+-5 km at 2 cm) and the scan index in the top 6 bits,
    so a batch may hold up to 64 scans.
    """
    ijk = torch.floor(xyz / voxel).to(torch.int64) + (1 << 18)
    key = (scan_id.to(torch.int64) << 57) | (ijk[:, 0] << 38) | (ijk[:, 1] << 19) | ijk[:, 2]
    uniq, inv = torch.unique(key, return_inverse=True)
    sums = torch.zeros((len(uniq), 3), dtype=torch.float32, device=xyz.device).index_add_(0, inv, xyz)
    cnt = torch.zeros(len(uniq), dtype=torch.float32, device=xyz.device).index_add_(
        0, inv, torch.ones(len(xyz), device=xyz.device))
    return sums / cnt[:, None], uniq >> 57


class VoxelMap:
    """Running per-voxel centroid of points and intensity (same result as mapping.VoxelCloud)."""

    def __init__(self, voxel: float, dev: torch.device):
        self.voxel, self.dev = voxel, dev
        self.keys = torch.empty(0, dtype=torch.int64, device=dev)
        self.sums = torch.empty((0, 4), dtype=torch.float64, device=dev)
        self.counts = torch.empty(0, dtype=torch.int64, device=dev)

    def add(self, xyz: torch.Tensor, intensity: torch.Tensor) -> None:
        keys = torch.cat([self.keys, voxel_keys(xyz, self.voxel)])
        vals = torch.cat([self.sums, torch.cat([xyz, intensity[:, None]], 1).double()])
        counts = torch.cat([self.counts, torch.ones(len(xyz), dtype=torch.int64, device=self.dev)])
        uniq, inv = torch.unique(keys, return_inverse=True)
        self.sums = torch.zeros((len(uniq), 4), dtype=torch.float64, device=self.dev).index_add_(0, inv, vals)
        self.counts = torch.zeros(len(uniq), dtype=torch.int64, device=self.dev).index_add_(0, inv, counts)
        self.keys = uniq

    def result(self, min_points: int = 1):
        keep = self.counts >= min_points
        mean = self.sums[keep] / self.counts[keep, None]
        return (mean[:, :3].cpu().numpy(), mean[:, 3].float().cpu().numpy(), self.counts[keep].cpu().numpy())


class Tsdf:
    """Dense truncated signed distance grid filled along LiDAR rays (VDBFusion-style).

    Each return adds samples along its ray within +-``trunc`` of the hit: positive in front of the
    surface (towards the sensor), negative behind. Averaging many passes gives one surface.
    """

    def __init__(self, lo: np.ndarray, hi: np.ndarray, voxel: float, trunc: float, dev: torch.device,
                 max_voxels: int = 400_000_000):
        dims = np.ceil((hi - lo) / voxel).astype(int) + 1
        while np.prod(dims.astype(np.int64)) > max_voxels:
            voxel *= 1.25
            dims = np.ceil((hi - lo) / voxel).astype(int) + 1
            log.warning("tsdf: area too large for the GPU budget, voxel raised to %.3f m", voxel)
        self.lo, self.voxel, self.trunc, self.dims, self.dev = lo.astype(np.float32), voxel, trunc, dims, dev
        n = int(np.prod(dims.astype(np.int64)))
        self.sdf = torch.zeros(n, dtype=torch.float32, device=dev)
        self.w = torch.zeros(n, dtype=torch.float32, device=dev)
        self._lo = torch.as_tensor(self.lo, device=dev)
        self._dims = torch.as_tensor(dims, device=dev)
        self.steps = torch.arange(-trunc, trunc + 1e-6, voxel * 0.75, device=dev, dtype=torch.float32)

    def integrate(self, xyz: torch.Tensor, origin: torch.Tensor) -> None:
        d = xyz - origin
        d = d / d.norm(dim=1, keepdim=True).clamp_min(1e-6)
        for s in self.steps:
            q = xyz + s * d
            ijk = torch.round((q - self._lo) / self.voxel).to(torch.int64)
            ok = ((ijk >= 0) & (ijk < self._dims)).all(1)
            ijk = ijk[ok]
            flat = (ijk[:, 0] * self._dims[1] + ijk[:, 1]) * self._dims[2] + ijk[:, 2]
            # Signed distance of the voxel centre (not the sample) to the hit, along the ray.
            centre = self._lo + ijk.to(torch.float32) * self.voxel
            sdf = ((xyz[ok] - centre) * d[ok]).sum(1).clamp(-self.trunc, self.trunc)
            self.sdf.index_add_(0, flat, sdf)
            self.w.index_add_(0, flat, torch.ones(len(flat), device=self.dev))

    def mesh(self, min_weight: float = 3.0):
        """Marching cubes on the averaged field -> (vertices (V,3) world, triangles (T,3))."""
        from skimage.measure import marching_cubes

        w = self.w.reshape(*self.dims)
        field = torch.where(w > 0, self.sdf.reshape(*self.dims) / w.clamp_min(1e-6),
                            torch.tensor(self.trunc, device=self.dev))
        # A cube is meshed only when all 8 corners were observed; otherwise the edge between the
        # observed band behind a wall and unobserved space becomes a second, ghost surface.
        ok = w >= min_weight
        cube = ok.clone()
        for dx in (0, 1):
            for dy in (0, 1):
                for dz in (0, 1):
                    if dx or dy or dz:
                        shifted = torch.zeros_like(ok)
                        shifted[:ok.shape[0] - dx, :ok.shape[1] - dy, :ok.shape[2] - dz] = ok[dx:, dy:, dz:]
                        cube &= shifted
        mask = cube.cpu().numpy()
        vol = field.cpu().numpy()
        verts, faces, _, _ = marching_cubes(vol, level=0.0, mask=mask, allow_degenerate=False)
        # skimage's mask does not reliably skip cubes at the observed/unobserved border, so drop
        # every triangle with a vertex whose cube has an under-observed corner.
        base = np.clip(np.floor(verts).astype(np.int64), 0, np.asarray(mask.shape) - 1)
        good_v = mask[base[:, 0], base[:, 1], base[:, 2]]
        faces = faces[good_v[faces].all(1)]
        return verts * self.voxel + self.lo, faces


def project(xyz: torch.Tensor, T_world_cam: np.ndarray, K: np.ndarray):
    """Pixel coords and depth of world points in an OpenCV camera (device tensors)."""
    T_cw = np.linalg.inv(T_world_cam)
    R = torch.as_tensor(T_cw[:3, :3], dtype=torch.float32, device=xyz.device)
    t = torch.as_tensor(T_cw[:3, 3], dtype=torch.float32, device=xyz.device)
    pc = xyz @ R.T + t
    z = pc[:, 2]
    u = pc[:, 0] / z * float(K[0, 0]) + float(K[0, 2])
    v = pc[:, 1] / z * float(K[1, 1]) + float(K[1, 2])
    return u, v, z


def visible(u, v, z, w: int, h: int, max_depth: float, cell: int = 4, rel_tol: float = 0.03,
            abs_tol: float = 0.05):
    """Indices of points inside the image and in front (coarse z-buffer over the points themselves)."""
    ok = (z > 0.1) & (z < max_depth) & (u >= 0) & (u < w) & (v >= 0) & (v < h)
    idx = torch.nonzero(ok).squeeze(1)
    ui, vi, d = u[idx].long(), v[idx].long(), z[idx]
    gw = (w + cell - 1) // cell
    cells = (vi // cell) * gw + ui // cell
    zbuf = torch.full((gw * ((h + cell - 1) // cell),), float("inf"), device=z.device)
    zbuf.scatter_reduce_(0, cells, d, reduce="amin")
    vis = d <= zbuf[cells] * (1 + rel_tol) + abs_tol
    return idx[vis], ui[vis], vi[vis], d[vis]
