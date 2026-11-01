"""Session-to-session registration: Open3D FPFH+RANSAC (coarse) then small_gicp GICP (fine)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class AlignResult:
    T_ref_src: np.ndarray
    converged: bool
    fitness: float  # fraction of source points with a reference neighbour within max_corr
    rmse: float
    coarse: np.ndarray | None = None


def coarse_align(ref: np.ndarray, src: np.ndarray, voxel: float = 0.25, seed: int = 0) -> np.ndarray:
    import open3d as o3d

    o3d.utility.random.seed(seed)
    reg = o3d.pipelines.registration

    def prep(xyz):
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz)).voxel_down_sample(voxel)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2, max_nn=30))
        fpfh = reg.compute_fpfh_feature(pc, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5, max_nn=100))
        return pc, fpfh

    pr, fr = prep(ref)
    ps, fs = prep(src)
    dist = voxel * 1.5
    res = reg.registration_ransac_based_on_feature_matching(
        ps, pr, fs, fr, True, dist,
        reg.TransformationEstimationPointToPoint(False), 3,
        [reg.CorrespondenceCheckerBasedOnEdgeLength(0.9), reg.CorrespondenceCheckerBasedOnDistance(dist)],
        reg.RANSACConvergenceCriteria(200000, 0.999),
    )
    return np.asarray(res.transformation)


def fine_align(ref: np.ndarray, src: np.ndarray, init: np.ndarray | None = None,
               downsampling: float = 0.1, max_corr: float = 0.5, num_threads: int = 8):
    import small_gicp

    res = small_gicp.align(
        ref, src,
        init_T_target_source=np.eye(4) if init is None else init,
        registration_type="GICP",
        downsampling_resolution=downsampling,
        max_correspondence_distance=max_corr,
        num_threads=num_threads,
    )
    return np.asarray(res.T_target_source), bool(res.converged)


def evaluate(ref: np.ndarray, src: np.ndarray, T_ref_src: np.ndarray, max_corr: float) -> tuple[float, float]:
    from scipy.spatial import cKDTree

    moved = src @ T_ref_src[:3, :3].T + T_ref_src[:3, 3]
    d, _ = cKDTree(ref).query(moved, distance_upper_bound=max_corr)
    inl = np.isfinite(d)
    return float(inl.mean()), float(np.sqrt(np.mean(d[inl] ** 2))) if inl.any() else float("inf")


def align_maps(ref: np.ndarray, src: np.ndarray, coarse: bool = True, init: np.ndarray | None = None,
               coarse_voxel: float = 0.25, fine_voxel: float = 0.1, max_corr: float = 0.5) -> AlignResult:
    """Estimate ``T_ref_src`` mapping ``src`` map points into the ``ref`` map frame."""
    T0 = coarse_align(ref, src, coarse_voxel) if coarse and init is None else (np.eye(4) if init is None else init)
    T, converged = fine_align(ref, src, T0, fine_voxel, max_corr)
    T, _ = fine_align(ref, src, T, fine_voxel / 2, max_corr / 4)
    fitness, rmse = evaluate(ref, src, T, max_corr / 4)
    return AlignResult(T, converged, fitness, rmse, T0 if coarse else None)
