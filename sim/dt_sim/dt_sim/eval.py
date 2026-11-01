"""Evaluate a stack run against simulator ground truth (design §8).

Trajectory: estimated LiDAR poses (FAST-LIO /Odometry from a dt_session bag, or a TUM file
such as dtp's products/traj_lidar.tum) vs GT ``T_world_lidar``.
- anchored: the estimate is placed in the world with the GT pose at its first sample
  (FAST-LIO's camera_init is the initial body pose, which the sim knows exactly);
- aligned: SE(3) Umeyama alignment over the whole trajectory.
Map: distance of each map point to the GT mesh, after the aligned transform; completeness is
the share of GT surface (visible from the path) with a map point within 5 cm.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from dt_pipeline.geometry import inv
from dt_pipeline.geometry.transforms import read_tum
from dt_pipeline.io.bag import Bag, stamp_to_sec
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .lidar import LidarNoise
from .lidar_airy import Airy
from .lidar_mid360 import Mid360
from .run import EPOCH0, RunDir
from .trajectory import Trajectory
from .world import World

log = logging.getLogger(__name__)


@dataclass
class EstTrajectory:
    t: np.ndarray        # run seconds
    T: np.ndarray        # (N,4,4) T_est_lidar in the estimator's frame
    source: str


def umeyama_se3(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """T minimising ||dst - T(src)|| (no scale)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = mu_d - R @ mu_s
    return T


def load_estimate(run: RunDir, session: Path | None, traj: Path | None,
                  epoch0: float = EPOCH0) -> EstTrajectory:
    sim = run.load_rig()
    if traj is None and session is not None:
        cand = Path(session) / "products" / "traj_lidar.tum"
        traj = cand if cand.exists() else None
    if traj is not None:
        stamps, poses = read_tum(traj)
        return EstTrajectory(np.asarray(stamps) - epoch0, np.asarray(poses), str(traj))
    if session is None:
        raise ValueError("need --session or --traj")
    T_imu_lidar = inv(sim.rig.T_lidar_imu)
    ts, Ts = [], []
    with Bag(Path(session) / "bag") as bag:
        for _, _, msg in bag.messages(["/Odometry"]):
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            T[:3, 3] = [p.x, p.y, p.z]
            ts.append(stamp_to_sec(msg.header.stamp) - epoch0)
            Ts.append(T @ T_imu_lidar)  # body (IMU) -> lidar_link
    if not ts:
        raise ValueError(f"no /Odometry in {session}/bag")
    return EstTrajectory(np.array(ts), np.array(Ts), f"{session}/bag:/Odometry")


def trajectory_metrics(run: RunDir, est: EstTrajectory) -> tuple[dict, np.ndarray, np.ndarray]:
    sim = run.load_rig()
    gt = Trajectory.from_csv(run.trajectory_path, sim.rig.T_base_lidar)
    keep = (est.t >= gt.start) & (est.t <= gt.end)
    t, T_est = est.t[keep], est.T[keep]
    if len(t) < 3:
        raise ValueError("estimate does not overlap the ground-truth time range (epoch0?)")
    T_gt = gt.matrices(t)

    T_anchor = T_gt[0] @ inv(T_est[0])
    T_aligned = umeyama_se3(T_est[:, :3, 3], T_gt[:, :3, 3])

    def errors(T_world_est):
        p = (T_world_est[:3, :3] @ T_est[:, :3, 3].T).T + T_world_est[:3, 3]
        e = np.linalg.norm(p - T_gt[:, :3, 3], axis=1)
        R_est = np.einsum("ij,njk->nik", T_world_est[:3, :3], T_est[:, :3, :3])
        rel = np.einsum("nji,njk->nik", R_est, T_gt[:, :3, :3])
        ang = np.degrees(np.linalg.norm(Rotation.from_matrix(rel).as_rotvec(), axis=1))
        return {"rmse_m": float(np.sqrt(np.mean(e ** 2))), "max_m": float(e.max()),
                "final_m": float(e[-1]), "rot_mean_deg": float(ang.mean()),
                "rot_max_deg": float(ang.max())}

    path_len = float(np.linalg.norm(np.diff(T_gt[:, :3, 3], axis=0), axis=1).sum())
    metrics = {"source": est.source, "samples": int(len(t)), "duration_s": float(t[-1] - t[0]),
               "path_length_m": path_len, "anchored": errors(T_anchor),
               "aligned": errors(T_aligned)}
    metrics["anchored"]["drift_pct"] = 100.0 * metrics["anchored"]["final_m"] / max(path_len, 1e-6)
    return metrics, T_anchor, T_aligned


def load_points(path: Path) -> np.ndarray:
    path = Path(path)
    if path.suffix in (".laz", ".las"):
        from dt_pipeline.io.las import read_laz
        xyz, _ = read_laz(path)
        return np.asarray(xyz, dtype=np.float64)
    if path.suffix == ".npz":
        return np.load(path)["xyz"].astype(np.float64)
    import open3d as o3d
    return np.asarray(o3d.io.read_point_cloud(str(path)).points, dtype=np.float64)


def map_metrics(run: RunDir, world: World, map_xyz: np.ndarray, T_world_map: np.ndarray,
                completeness_radius: float = 0.05, samples: int = 200_000,
                seed: int = 0) -> tuple[dict, np.ndarray, np.ndarray]:
    xyz = map_xyz @ T_world_map[:3, :3].T + T_world_map[:3, 3]
    err = world.distance(xyz)
    stats = {
        "points": int(len(xyz)),
        "mean_m": float(err.mean()), "p50_m": float(np.percentile(err, 50)),
        "p90_m": float(np.percentile(err, 90)), "p95_m": float(np.percentile(err, 95)),
        "within_2cm_pct": float(100 * np.mean(err < 0.02)),
        "within_5cm_pct": float(100 * np.mean(err < 0.05)),
    }
    stats["completeness_pct"] = completeness(run, world, xyz, completeness_radius, samples, seed)
    return stats, xyz, err


def umeyama_sim3(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray]:
    """``(scale, T)`` minimising ||dst - (scale * R src + t)||."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    a, b = src - mu_s, dst - mu_d
    U, S, Vt = np.linalg.svd(b.T @ a / len(src))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    scale = float((S * np.diag(D)).sum() / (a * a).sum(1).mean())
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = mu_d - scale * R @ mu_s
    return scale, T


def surface_metrics(world: World, xyz: np.ndarray, gate: float = 0.1, iterations: int = 30,
                    pairs: int = 20_000, seed: int = 0) -> dict:
    """Accuracy of the map's own shape: align it to the true surface, then measure.

    Aligning by the trajectory alone leaves any small tilt of the path in the score. Here the map
    is fitted to the surface (rotation + translation, then a separate fit with scale), so what
    remains is shape error. ``tape`` is a virtual tape measure: distances between random pairs
    of map points against the same pairs on the true surface, independent of the alignment.
    """
    rng = np.random.default_rng(seed)
    pts = xyz[rng.choice(len(xyz), min(300_000, len(xyz)), replace=False)]

    def fit(with_scale: bool):
        scale, T = 1.0, np.eye(4)
        for _ in range(iterations):
            cur = scale * pts @ T[:3, :3].T + T[:3, 3]
            near = world.closest(cur)
            ok = np.linalg.norm(cur - near, axis=1) < gate
            s, T = umeyama_sim3(pts[ok], near[ok])
            if with_scale:
                scale = s
            else:
                T = umeyama_se3(pts[ok], near[ok])
        return scale, T

    _, T = fit(False)
    scale, _ = fit(True)
    aligned = xyz @ T[:3, :3].T + T[:3, 3]
    err = world.distance(aligned)
    near = world.closest(aligned)
    inlier = np.flatnonzero(err < gate)
    a, b = rng.choice(inlier, pairs), rng.choice(inlier, pairs)
    true_d = np.linalg.norm(near[a] - near[b], axis=1)
    keep = true_d > 1.0
    tape = np.abs(np.linalg.norm(aligned[a] - aligned[b], axis=1) - true_d)[keep]
    return {"mean_m": float(err.mean()), "p50_m": float(np.percentile(err, 50)),
            "p90_m": float(np.percentile(err, 90)), "within_2cm_pct": float(100 * np.mean(err < 0.02)),
            "scale_error_ppm": float((scale - 1) * 1e6),
            "tape_median_mm": float(np.median(tape) * 1000), "tape_p95_mm": float(np.percentile(tape, 95) * 1000),
            "T_world_map": T.tolist()}


def completeness(run: RunDir, world: World, xyz_world: np.ndarray, radius: float,
                 samples: int, seed: int, max_range: float = 30.0) -> float:
    sim = run.load_rig()
    gt = Trajectory.from_csv(run.trajectory_path, sim.rig.T_base_lidar)
    rng = np.random.default_rng(seed)
    surf = world.sample_surface(samples, rng)
    stations = gt.position(np.arange(gt.start, gt.end, 0.5))
    d, nearest = cKDTree(stations).query(surf)
    near = d < max_range
    origins = stations[nearest[near]]
    vec = surf[near] - origins
    dist = np.linalg.norm(vec, axis=1)
    hits = world.cast(origins, vec / dist[:, None])
    visible = np.zeros(len(surf), bool)
    visible[np.flatnonzero(near)] = hits.distance >= dist - 0.05
    if not visible.any() or len(xyz_world) == 0:
        return 0.0
    dmap, _ = cKDTree(xyz_world).query(surf[visible], distance_upper_bound=radius)
    return float(100 * np.mean(np.isfinite(dmap)))


def evaluate(run: RunDir, session: Path | None = None, map_path: Path | None = None,
             traj: Path | None = None, variant: str = "day1", epoch0: float = EPOCH0,
             diff_dir: Path | None = None) -> dict:
    est = load_estimate(run, session, traj, epoch0)
    traj_m, T_anchor, T_aligned = trajectory_metrics(run, est)
    result = {"run": str(run.root), "variant": variant, "trajectory": traj_m}

    if map_path is None and session is not None:
        cand = Path(session) / "products" / "map.laz"
        map_path = cand if cand.exists() else None
    if map_path is not None:
        world = World.load(run.mesh(variant))
        stats, xyz, err = map_metrics(run, world, load_points(map_path), T_aligned)
        stats["map"] = str(map_path)
        result["map"] = stats
        result["map_surface"] = surface_metrics(world, xyz)
        T = np.array(result["map_surface"]["T_world_map"])
        result["map_surface"]["completeness_pct"] = completeness(run, world, xyz @ T[:3, :3].T + T[:3, 3], 0.05,
                                                                 200_000, 0)
        run.products.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(run.products / "map_eval.npz", x=xyz[:, 0].astype(np.float32),
                            y=xyz[:, 1].astype(np.float32), z=xyz[:, 2].astype(np.float32),
                            error=err.astype(np.float32))
    result["T_world_estimate_aligned"] = T_aligned.tolist()
    result["T_world_estimate_anchored"] = T_anchor.tolist()
    run.products.mkdir(parents=True, exist_ok=True)
    export_world_trajectory(est, T_aligned, run.products / f"traj_est_world_{variant}.tum")
    if diff_dir is not None:
        result["diff_exported"] = export_diff_world(Path(diff_dir), T_aligned,
                                                    run.products / f"diff_{variant}")
    (run.root / f"eval_{variant}.json").write_text(json.dumps(result, indent=2))
    (run.root / "eval.json").write_text(json.dumps(result, indent=2))
    return result


def export_world_trajectory(est: EstTrajectory, T_world_est: np.ndarray, out: Path) -> Path:
    """Estimated lidar_link trajectory in the sim world frame (TUM), for show_result.py."""
    T = np.einsum("ij,njk->nik", T_world_est, est.T)
    q = Rotation.from_matrix(T[:, :3, :3]).as_quat()
    np.savetxt(out, np.column_stack([est.t, T[:, :3, 3], q]), fmt="%.6f")
    return out


def export_diff_world(diff_dir: Path, T_world_ref: np.ndarray, out: Path) -> dict:
    """dtp diff output (in the reference session's map frame) -> world-frame npz for Blender.

    T_world_ref must come from evaluating the *reference* session of that diff.
    """
    out.mkdir(parents=True, exist_ok=True)
    counts = {}
    for name in ("added", "removed"):
        src = diff_dir / f"{name}.laz"
        xyz = load_points(src) if src.exists() else np.empty((0, 3))
        xyz = xyz @ T_world_ref[:3, :3].T + T_world_ref[:3, 3]
        np.savez_compressed(out / f"{name}.npz", points=xyz.astype(np.float32))
        counts[name] = int(len(xyz))
    return counts


def export_gt_map(run: RunDir, profile: str, out: Path, variant: str = "day1",
                  voxel: float = 0.03, frame_step: float = 0.5) -> Path:
    """Noise-free LiDAR returns in the world frame: the best map the device could produce."""
    from dt_pipeline.io.las import write_laz

    sim = run.load_rig()
    world = World.load(run.mesh(variant))
    traj = Trajectory.from_csv(run.trajectory_path, sim.rig.T_base_lidar)
    ideal = LidarNoise.ideal()
    chunks = []
    if profile == "airy":
        sensor = Airy(world, traj, ideal)
    else:
        sensor = Mid360(world, traj, ideal)
    for k, t in enumerate(np.arange(traj.start, traj.end - 0.1, frame_step)):
        if profile == "airy":
            ret, times = sensor.frame(t)
        else:
            idx, ret, offs = sensor.frame(k, t)
            times = t + offs * 1e-9
        T = traj.matrices(times[ret.valid])
        p = ret.points[ret.valid]
        chunks.append(np.einsum("nij,nj->ni", T[:, :3, :3], p) + T[:, :3, 3])
    xyz = np.concatenate(chunks) if chunks else np.zeros((0, 3))
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    xyz = xyz[np.sort(first)]
    return write_laz(out, xyz)
