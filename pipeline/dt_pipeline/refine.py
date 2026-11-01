"""Global trajectory refinement: keyframe pose graph with ICP odometry and loop-closure edges,
plus ``polish``: aligning scans to the fused surface.

FAST-LIO is a filter: once its heading has drifted a few degrees (spinning in place, jerks),
nothing pulls it back, and a wall seen again later lands 10-20 cm beside its first copy.
Here every keyframe gets a local submap (its neighbouring deskewed scans), consecutive
keyframes are re-registered, and keyframes that revisit an earlier place are registered to
it as loop edges. The optimised keyframe poses give a correction ``C_k = T_opt_k @ inv(T_odom_k)``
that is interpolated in time and applied to the full-rate trajectory.

Loop edges must be plausible: good overlap, low residual, and a correction relative to the
odometry no larger than FAST-LIO could have drifted (``max_loop_shift`` m, ``max_loop_turn`` deg).

Registration is point-to-plane ICP with a Tukey kernel on Open3D tensors (CUDA when available).
"""

from __future__ import annotations

import logging
from collections import deque
from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .geometry import Trajectory, inv
from .mapping import WorldScan

log = logging.getLogger("dtp")


def o3d_device():
    import open3d as o3d

    return o3d.core.Device("CUDA:0" if o3d.core.cuda.is_available() else "CPU:0")


@dataclass
class Keyframe:
    stamp: float
    pose: np.ndarray  # odometry T_world_lidar
    cloud: object = None  # open3d tensor PointCloud (with normals) in the keyframe frame


def tensor_cloud(xyz: np.ndarray, voxel: float | None = None, max_nn: int = 20):
    import open3d as o3d

    pcd = o3d.t.geometry.PointCloud(o3d.core.Tensor(np.ascontiguousarray(xyz, np.float32), device=o3d_device()))
    if voxel:
        pcd = pcd.voxel_down_sample(voxel)
    pcd.estimate_normals(max_nn=max_nn, radius=max(3 * (voxel or 0.05), 0.1))
    return pcd


def build_keyframes(scans: Iterable[WorldScan], min_translation: float = 0.25, min_rotation_deg: float = 15.0,
                    window: int = 5, voxel: float = 0.05) -> list[Keyframe]:
    """Keyframes by motion; each submap = deskewed scans within ``window`` of it, in its frame.

    Streams: only ``2 * window + 1`` scans are held at a time; pass scans already downsampled
    (``cached_world_scans(..., downsample=...)``) to keep that small.
    """
    buf: deque = deque(maxlen=2 * window + 1)
    pending: list[tuple[int, WorldScan]] = []  # keyframes waiting for their trailing window
    kfs, last = [], None

    def finish(i_kf, kf_scan, scans_window):
        T_kw = inv(kf_scan.pose)
        pts = np.concatenate([s.xyz for j, s in scans_window if abs(j - i_kf) <= window])
        kfs.append(Keyframe(kf_scan.stamp, kf_scan.pose, tensor_cloud(pts @ T_kw[:3, :3].T + T_kw[:3, 3], voxel)))

    for i, ws in enumerate(scans):
        buf.append((i, ws))
        if last is None or (np.linalg.norm(ws.pose[:3, 3] - last[:3, 3]) >= min_translation or np.degrees(
                Rotation.from_matrix(last[:3, :3].T @ ws.pose[:3, :3]).magnitude()) >= min_rotation_deg):
            pending.append((i, ws))
            last = ws.pose
        while pending and i - pending[0][0] >= window:
            finish(*pending.pop(0), list(buf))
    for p in pending:
        finish(*p, list(buf))
    return kfs


def register(src, dst, init: np.ndarray, max_corr: float, iterations: int = 40):
    """Point-to-plane ICP of src into dst; returns (T_dst_src, fitness, rmse, information 6x6)."""
    import open3d as o3d

    reg = o3d.t.pipelines.registration
    est = reg.TransformationEstimationPointToPlane(reg.robust_kernel.RobustKernel(
        reg.robust_kernel.RobustKernelMethod.TukeyLoss, max_corr / 2))
    res = reg.icp(src, dst, max_corr, o3d.core.Tensor(init), est, reg.ICPConvergenceCriteria(max_iteration=iterations))
    T = res.transformation.cpu().numpy()
    info = reg.get_information_matrix(src, dst, max_corr, res.transformation).cpu().numpy()
    return T, float(res.fitness), float(res.inlier_rmse), info


def _delta(Ta: np.ndarray, Tb: np.ndarray) -> tuple[float, float]:
    D = inv(Ta) @ Tb
    return float(np.linalg.norm(D[:3, 3])), float(np.degrees(Rotation.from_matrix(D[:3, :3]).magnitude()))


def optimise(kfs: list[Keyframe], loop_radius: float = 2.0, loop_min_gap_s: float = 10.0,
             loop_fitness: float = 0.6, loop_rmse: float = 0.03, max_corr: float = 0.15,
             max_loops_per_kf: int = 2, max_loop_shift: float = 0.3, max_loop_turn: float = 5.0) -> tuple[np.ndarray, dict]:
    """Pose-graph optimisation; returns optimised keyframe poses (N,4,4) and edge statistics."""
    import open3d as o3d

    reg = o3d.pipelines.registration
    graph = reg.PoseGraph()
    for k in kfs:
        graph.nodes.append(reg.PoseGraphNode(k.pose))
    stats = {"keyframes": len(kfs), "odometry_edges": 0, "loop_candidates": 0, "loop_edges": 0,
             "loops_rejected_fit": 0, "loops_rejected_implausible": 0}
    for a in range(len(kfs) - 1):
        b = a + 1
        prior = inv(kfs[a].pose) @ kfs[b].pose
        T_ab, fit, rmse, info = register(kfs[b].cloud, kfs[a].cloud, prior, max_corr)
        if fit < 0.3 or _delta(prior, T_ab)[0] > max_loop_shift:  # fall back to the odometry edge
            T_ab = prior
        graph.edges.append(reg.PoseGraphEdge(b, a, T_ab, info, uncertain=False))
        stats["odometry_edges"] += 1
    pos = np.array([k.pose[:3, 3] for k in kfs])
    stamps = np.array([k.stamp for k in kfs])
    for b in range(len(kfs)):
        older = np.flatnonzero((stamps < stamps[b] - loop_min_gap_s)
                               & (np.linalg.norm(pos - pos[b], axis=1) < loop_radius))
        chosen: list[int] = []
        for a in older[np.argsort(np.linalg.norm(pos[older] - pos[b], axis=1))]:
            if len(chosen) >= max_loops_per_kf:
                break
            if any(abs(stamps[a] - stamps[c]) < loop_min_gap_s for c in chosen):
                continue  # one candidate per earlier visit
            chosen.append(int(a))
        for a in chosen:
            stats["loop_candidates"] += 1
            prior = inv(kfs[a].pose) @ kfs[b].pose
            T_ab, fit, rmse, info = register(kfs[b].cloud, kfs[a].cloud, prior, max_corr)
            if fit < loop_fitness or rmse > loop_rmse:
                stats["loops_rejected_fit"] += 1
                continue
            shift, turn = _delta(prior, T_ab)
            if shift > max_loop_shift or turn > max_loop_turn:
                stats["loops_rejected_implausible"] += 1
                continue
            graph.edges.append(reg.PoseGraphEdge(b, a, T_ab, info, uncertain=True))
            stats["loop_edges"] += 1
    reg.global_optimization(graph, reg.GlobalOptimizationLevenbergMarquardt(),
                            reg.GlobalOptimizationConvergenceCriteria(),
                            reg.GlobalOptimizationOption(max_correspondence_distance=max_corr,
                                                         edge_prune_threshold=0.25, reference_node=0))
    return np.array([n.pose for n in graph.nodes]), stats


def apply_correction(tr: Trajectory, kf_stamps: np.ndarray, kf_odom: np.ndarray, kf_opt: np.ndarray) -> Trajectory:
    """``T_new(t) = C(t) @ T_odom(t)`` with ``C`` interpolated between keyframe corrections."""
    C = kf_opt @ np.linalg.inv(kf_odom)
    t = np.clip(tr.stamps, kf_stamps[0], kf_stamps[-1])
    rot = Slerp(kf_stamps, Rotation.from_matrix(C[:, :3, :3]))(t).as_matrix()
    trans = np.column_stack([np.interp(t, kf_stamps, C[:, i, 3]) for i in range(3)])
    Ct = np.tile(np.eye(4), (len(t), 1, 1))
    Ct[:, :3, :3], Ct[:, :3, 3] = rot, trans
    return Trajectory(tr.stamps.copy(), Ct @ tr.poses)


def polish(tr: Trajectory, scans: Iterable[WorldScan], surface_xyz: np.ndarray, max_corr: float = 0.05,
           voxel: float = 0.03, smooth: int = 5) -> tuple[Trajectory, dict]:
    """Align each (sampled) deskewed scan to the fused surface and spread the corrections.

    The TSDF surface averages every pass, so it is less noisy than any single scan; pulling each
    scan onto it removes the small per-scan pose errors that thicken walls in the point map.
    Corrections are median-smoothed over ``smooth`` neighbours so one bad fit cannot kink the path.
    """
    target = tensor_cloud(surface_xyz, None, max_nn=30)
    stamps, odom, corr, fits = [], [], [], []
    for ws in scans:
        src = tensor_cloud(ws.xyz, voxel)
        T, fit, rmse, _ = register(src, target, np.eye(4), max_corr, iterations=30)
        if fit < 0.5 or _delta(np.eye(4), T)[0] > 0.1:
            T = np.eye(4)
        stamps.append(ws.stamp)
        odom.append(ws.pose)
        corr.append(T)
        fits.append(fit)
    corr = np.asarray(corr)
    # Smooth: median translation, chordal-mean rotation over a sliding window.
    sm = corr.copy()
    h = smooth // 2
    for i in range(len(corr)):
        w = corr[max(0, i - h): i + h + 1]
        sm[i, :3, 3] = np.median(w[:, :3, 3], axis=0)
        sm[i, :3, :3] = Rotation.from_matrix(w[:, :3, :3]).mean().as_matrix()
    odom = np.asarray(odom)
    out = apply_correction(tr, np.asarray(stamps), odom, sm @ odom)
    shift = np.linalg.norm(sm[:, :3, 3], axis=1)
    return out, {"scans": len(stamps), "median_fitness": round(float(np.median(fits)), 3),
                 "median_shift_mm": round(float(np.median(shift) * 1000), 1),
                 "p95_shift_mm": round(float(np.percentile(shift, 95) * 1000), 1)}
