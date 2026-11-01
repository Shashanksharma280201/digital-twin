"""Session-level steps behind the ``dtp`` CLI (design doc §10). Each writes into ``products/``."""

from __future__ import annotations

import json
import logging
from pathlib import Path

import numpy as np

from . import align as align_mod
from . import clean as clean_mod
from . import colmap as colmap_mod
from . import diff as diff_mod
from .geometry import Trajectory, transform_points
from .io import CAMERA_INFO_TOPIC, IMAGE_TOPIC, ODOM_TOPIC, POINTS_TOPIC, Bag, read_laz, write_copc, write_laz
from .mapping import build_map, scans_to_world
from .session import Session
from .traj import from_glim_dump, from_odometry

log = logging.getLogger("dtp")


def info(path: str | Path) -> dict:
    s = Session(path)
    with Bag(s.bag) as bag:
        return bag.info()


def traj(session: Session, source: str = "fastlio", glim_dump: str | Path | None = None,
         odom_topic: str = ODOM_TOPIC) -> Trajectory:
    if source == "glim":
        dump = Path(glim_dump) if glim_dump else session.path / "glim_dump"
        tr = from_glim_dump(dump)
    elif source == "fastlio":
        tr = from_odometry(session.bag, session.rig, odom_topic)
    else:
        raise ValueError(f"unknown trajectory source {source!r}")
    tr.to_tum(session.traj_path)
    (session.products / "traj_lidar_odom.tum").unlink(missing_ok=True)  # stale `dtp refine` input
    log.info("trajectory: %d poses, %.1f s -> %s", len(tr.stamps), tr.end - tr.start, session.traj_path)
    return tr


def load_traj(session: Session) -> Trajectory:
    if not session.traj_path.exists():
        raise FileNotFoundError(f"{session.traj_path} missing; run `dtp traj` first")
    return Trajectory.from_tum(session.traj_path)


def world_scans(session: Session, topic: str = POINTS_TOPIC, deskew: bool = True, every: int = 1,
                min_range: float = 0.3, max_range: float = 60.0):
    tr = load_traj(session)
    with Bag(session.bag) as bag:
        yield from scans_to_world(bag.scans(topic), tr, deskew=deskew, every=every,
                                  min_range=min_range, max_range=max_range)


def cache_dir(session: Session) -> Path:
    return session.products / "cache"


def ingest(session: Session, topic: str = POINTS_TOPIC, odom_topic: str = ODOM_TOPIC, workers: int = 8,
           drop_mixed: bool = True) -> dict:
    """Read the bag once: scan cache (products/cache) + FAST-LIO trajectory (traj_lidar.tum)."""
    from . import ringcal as ringcal_mod
    from .cache import ingest as ingest_cache

    cache, tr, stats = ingest_cache(session.bag, cache_dir(session), session.rig.T_lidar_imu, topic, odom_topic,
                                    workers, drop_mixed=drop_mixed,
                                    ring_calib=ringcal_mod.load(session.path / ringcal_mod.CALIB_NAME))
    if tr is not None:
        tr.to_tum(session.traj_path)
        (session.products / "traj_lidar_odom.tum").unlink(missing_ok=True)
        stats["trajectory_s"] = round(tr.end - tr.start, 1)
    log.info("ingest: %s -> %s", stats, cache.dir)
    return stats


def cached_world_scans(session: Session, tr: Trajectory, every: int = 1, deskew: bool = True,
                       min_range: float = 0.3, max_range: float = 60.0, downsample: float | None = None):
    """WorldScans from the scan cache, transformed (and optionally voxel-downsampled) on the GPU.

    With ``downsample`` the scans carry no intensity (registration input).
    """
    import torch

    from . import gpu
    from .cache import ScanCache
    from .mapping import WorldScan

    cache = ScanCache(cache_dir(session))
    gt = gpu.GpuTrajectory(tr)
    for sel, st, pts in cache.batches(every=every, max_scans=64):
        r = np.linalg.norm(pts[:, :3], axis=1)
        keep = (r >= min_range) & (r <= max_range) & tr.covers(st)
        scan_of = np.repeat(np.arange(len(sel)), cache.offsets[sel + 1] - cache.offsets[sel])[keep]
        if not keep.any():
            continue
        xyz, _, inten = gpu.to_world(gt, st[keep], pts[keep], deskew)
        if downsample:
            xyz, sid = gpu.downsample_per_scan(xyz, torch.as_tensor(scan_of, device=xyz.device), downsample)
            scan_of = sid.cpu().numpy()
            inten = torch.zeros(len(xyz), device=xyz.device)
        counts = np.bincount(scan_of, minlength=len(sel))
        xyz, inten = xyz.cpu().numpy().astype(np.float64), inten.cpu().numpy()
        mids = np.clip(cache.stamps[sel] + 0.05, tr.start, tr.end)
        poses = tr.interpolate(mids)
        bounds = np.concatenate([[0], np.cumsum(counts)])
        for k in range(len(sel)):
            if counts[k]:
                a, b = bounds[k], bounds[k + 1]
                yield WorldScan(float(mids[k]), xyz[a:b], inten[a:b], poses[k, :3, 3].copy(), poses[k])


def ringcal(session: Session, topic: str = POINTS_TOPIC, every: int = 12) -> dict:
    """Learn per-ring range and elevation offsets against mesh.ply; writes SESSION/ring_calib.json.

    Reads the raw scans from the bag, so re-running it after a calibrated fuse replaces the
    calibration rather than stacking on it. The next ``ingest`` applies it.
    """
    from itertools import islice

    import open3d as o3d

    from . import ringcal as ringcal_mod

    tr = load_traj(session)
    mesh = o3d.io.read_triangle_mesh(str(session.products / "mesh.ply"))
    with Bag(session.bag) as bag:
        calib = ringcal_mod.estimate(islice(bag.scans(topic), 0, None, every), tr, mesh)
    ringcal_mod.save(session.path / ringcal_mod.CALIB_NAME, calib)
    dr, de = np.asarray(calib["range_m"]), np.asarray(calib["elev_rad"])
    stats = {"rings": int(np.count_nonzero(calib["points"])), "points": int(sum(calib["points"])),
             "range_std_mm": round(float(dr.std() * 1000), 2), "range_max_mm": round(float(np.abs(dr).max() * 1000), 2),
             "elev_std_deg": round(float(np.degrees(de.std())), 4)}
    log.info("ringcal: %s", stats)
    return stats


def refine(session: Session, topic: str = POINTS_TOPIC, every: int = 2, loop_radius: float = 2.0) -> dict:
    """Pose-graph refinement with loop closures; keeps the odometry as products/traj_lidar_odom.tum."""
    from . import refine as refine_mod

    odom_path = session.products / "traj_lidar_odom.tum"
    if not odom_path.exists():
        session.traj_path.replace(odom_path)
    tr = Trajectory.from_tum(odom_path)
    from .cache import ScanCache

    if ScanCache.exists(cache_dir(session)):
        kfs = refine_mod.build_keyframes(cached_world_scans(session, tr, every, downsample=0.025))
    else:
        with Bag(session.bag) as bag:
            kfs = refine_mod.build_keyframes(scans_to_world(bag.scans(topic), tr, deskew=True, every=every))
    if len(kfs) < 2:  # the rig barely moved: nothing to optimise, keep the odometry
        tr.to_tum(session.traj_path)
        log.info("refine: skipped, only %d keyframe(s)", len(kfs))
        return {"keyframes": len(kfs), "skipped": True}
    opt, stats = refine_mod.optimise(kfs, loop_radius=loop_radius)
    stamps = np.array([k.stamp for k in kfs])
    refined = refine_mod.apply_correction(tr, stamps, np.array([k.pose for k in kfs]), opt)
    refined.to_tum(session.traj_path)
    shift = np.linalg.norm(opt[:, :3, 3] - np.array([k.pose[:3, 3] for k in kfs]), axis=1)
    stats["max_keyframe_shift_m"] = round(float(shift.max()), 3)
    log.info("refine: %s -> %s", stats, session.traj_path)
    return stats


def build(session: Session, topic: str = POINTS_TOPIC, voxel: float = 0.05, deskew: bool = True,
          min_range: float = 0.3, max_range: float = 60.0, copc: bool = False) -> Path:
    xyz, intensity, counts = build_map(world_scans(session, topic, deskew, 1, min_range, max_range), voxel)
    out = write_laz(session.map_path, xyz, intensity, extra={"hits": counts.astype(np.uint32)})
    log.info("map: %d points -> %s", len(xyz), out)
    if copc and not write_copc(out, out.with_suffix("").with_suffix(".copc.laz")):
        log.warning("PDAL not installed; skipped COPC")
    return out


def fuse(session: Session, voxel: float = 0.02, mesh_voxel: float = 0.02, trunc: float | None = None,
         min_weight: float = 3.0, self_zone: float = 0.8, max_range: float = 30.0, mesh: bool = True,
         min_fragment: int = 2000, min_hits: int = 3) -> dict:
    """GPU: deskew every cached scan once, building the voxel map and the TSDF mesh together.

    Writes map.laz (``voxel`` centroids) and mesh.ply (marching cubes on the averaged TSDF, small
    disconnected fragments removed). ``self_zone`` drops returns from the operator and the rig.

    Each voxel counts once in the map, so a voxel holding one stray return weighs as much as one
    holding a thousand good ones. Voxels with fewer than ``min_hits`` returns go to map_loose.laz,
    so nothing is lost. In the sim this beat a distance-to-mesh filter on every score: a mesh has
    holes, and that filter removed real surfaces there.
    """
    import time

    from . import gpu
    from .cache import ScanCache

    t_start = time.time()
    tr = load_traj(session)
    cache = ScanCache(cache_dir(session))
    gt = gpu.GpuTrajectory(tr)
    dev = gt.dev
    trunc = trunc or 3 * mesh_voxel

    def batches(every=1):
        for sel, st, pts in cache.batches(every=every):
            r = np.linalg.norm(pts[:, :3], axis=1)
            keep = (r >= self_zone) & (r <= max_range) & tr.covers(st)
            if keep.any():
                yield gpu.to_world(gt, st[keep], pts[keep])

    tsdf = None
    if mesh:
        sample = torch_cat_cpu(xyz for xyz, _, _ in batches(every=20))
        lo, hi = np.percentile(sample, 0.05, axis=0) - trunc, np.percentile(sample, 99.95, axis=0) + trunc
        tsdf = gpu.Tsdf(lo, hi, mesh_voxel, trunc, dev)
    vm = gpu.VoxelMap(voxel, dev)
    for xyz, origin, inten in batches():
        vm.add(xyz, inten)
        if tsdf is not None:
            tsdf.integrate(xyz, origin)
    pts, inten, counts = vm.result()
    loose = counts < min_hits
    write_laz(session.products / "map_loose.laz", pts[loose], inten[loose], extra={"hits": counts[loose].astype(np.uint32)})
    write_laz(session.map_path, pts[~loose], inten[~loose], extra={"hits": counts[~loose].astype(np.uint32)})
    stats = {"map_points": int((~loose).sum()), "map_loose_points": int(loose.sum())}
    if tsdf is not None:
        import open3d as o3d

        verts, faces = tsdf.mesh(min_weight)
        m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts), o3d.utility.Vector3iVector(faces))
        clusters, sizes, _ = m.cluster_connected_triangles()
        clusters, sizes = np.asarray(clusters), np.asarray(sizes)
        m.remove_triangles_by_mask(sizes[clusters] < min_fragment)
        m.remove_unreferenced_vertices()
        m.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(session.products / "mesh.ply"), m)
        stats |= {"mesh_vertices": len(m.vertices), "mesh_triangles": len(m.triangles),
                  "fragments_removed": int((sizes < min_fragment).sum()), "tsdf_voxel": tsdf.voxel}
    stats["seconds"] = round(time.time() - t_start, 1)
    log.info("fuse: %s", stats)
    return stats


def torch_cat_cpu(tensors) -> np.ndarray:
    import torch

    return torch.cat(list(tensors)).cpu().numpy()


def polish(session: Session, every: int = 5, max_corr: float = 0.05) -> dict:
    """Align sampled scans to the fused surface (mesh.ply) and update traj_lidar.tum."""
    import open3d as o3d

    from . import refine as refine_mod

    tr = load_traj(session)
    mesh = o3d.io.read_triangle_mesh(str(session.products / "mesh.ply"))
    surface = np.asarray(mesh.sample_points_uniformly(min(3_000_000, 4 * len(mesh.vertices))).points)
    new, stats = refine_mod.polish(tr, cached_world_scans(session, tr, every, min_range=0.8, max_range=30.0,
                                                                downsample=0.03),
                                   surface, max_corr)
    tr.to_tum(session.products / "traj_lidar_prepolish.tum")
    new.to_tum(session.traj_path)
    log.info("polish: %s -> %s", stats, session.traj_path)
    return stats


def clean(session: Session, topic: str = POINTS_TOPIC, voxel: float = 0.1, method: str = "auto",
          every: int = 2) -> tuple[Path, Path]:
    scans = list(world_scans(session, topic, True, every))
    dyn = clean_mod.dynamic_voxels(scans, voxel, method)
    xyz, attrs = read_laz(session.map_path)
    static, dynamic = clean_mod.split_points(xyz, dyn, voxel)
    a = write_laz(session.static_map_path, xyz[static], attrs["intensity"][static])
    b = write_laz(session.products / "map_dynamic.laz", xyz[dynamic], attrs["intensity"][dynamic])
    log.info("clean: %d static / %d dynamic points", static.sum(), dynamic.sum())
    return a, b


def align(ref: Session, cur: Session, coarse: bool = True, init: np.ndarray | None = None,
          max_corr: float = 0.5) -> align_mod.AlignResult:
    ref_xyz, _ = read_laz(ref.best_map())
    cur_xyz, _ = read_laz(cur.best_map())
    res = align_mod.align_maps(ref_xyz, cur_xyz, coarse=coarse, init=init, max_corr=max_corr)
    np.savetxt(cur.products / "T_ref_session.txt", res.T_ref_src, fmt="%.9f",
               header=f"ref={ref.path} fitness={res.fitness:.4f} rmse={res.rmse:.4f} converged={res.converged}")
    log.info("align: fitness %.3f rmse %.4f", res.fitness, res.rmse)
    return res


def diff(ref: Session, cur: Session, voxel: float = 0.05, use_m3c2: bool = False) -> dict:
    T_path = cur.products / "T_ref_session.txt"
    T = np.loadtxt(T_path) if T_path.exists() else np.eye(4)
    ref_xyz, _ = read_laz(ref.best_map())
    cur_xyz, _ = read_laz(cur.best_map())
    cur_xyz = transform_points(T, cur_xyz)
    res = diff_mod.voxel_diff(ref_xyz, cur_xyz, voxel)
    out = cur.products / "diff"
    out.mkdir(exist_ok=True)
    if res.added_mask.any():
        write_laz(out / "added.laz", cur_xyz[res.added_mask])
    if res.removed_mask.any():
        write_laz(out / "removed.laz", ref_xyz[res.removed_mask])
    summary = res.summary() | {"ref": str(ref.path), "session": str(cur.path)}
    if use_m3c2:
        core, dist, lod = diff_mod.m3c2(ref_xyz, cur_xyz)
        sig = np.abs(dist) > lod
        write_laz(out / "m3c2.laz", core, extra={"distance": dist.astype(np.float32),
                                                 "lodetection": lod.astype(np.float32)})
        summary["m3c2_significant_fraction"] = float(np.nanmean(sig))
    diff_mod.write_summary(out / "summary.json", summary)
    return summary


def colorize(session: Session, image_topic: str = IMAGE_TOPIC, info_topic: str = CAMERA_INFO_TOPIC,
             every: int = 1, max_depth: float = 15.0, source: str = "auto", svo_scale: float = 0.5,
             blur_skip: float = 0.6, sharpen: float = 2.0, use_gpu: bool = True, exposure: bool = True) -> Path:
    """LiDAR map (+ mesh) + camera -> true-colour map (products/map_rgb.laz) and coloured mesh.ply.

    ``source``: ``svo`` (the session's zed.svo2, every frame), ``bag`` (the image topic) or ``auto``
    (svo when present). GPU path (PyTorch): keyframes by camera motion, exposure compensation,
    best views win. CPU fallback: every ``every``-th frame sharper than ``blur_skip`` x the median.
    """
    from contextlib import ExitStack

    rig = session.rig
    T_lo = rig.lidar_to_optical
    tr = load_traj(session)
    xyz, attrs = read_laz(session.best_map())
    mesh_path = session.products / "mesh.ply"
    svo_path = session.path / "zed.svo2"
    if source == "auto":  # an SVO holding only its header (AGX wrapper v5.2.2) does not count
        source = "svo" if svo_path.exists() and svo_path.stat().st_size > 1_000_000 else "bag"
    try:
        import torch  # noqa: F401
    except ImportError:
        use_gpu = False
    with ExitStack() as stack:
        if source == "svo":
            from .io.svo import Svo

            svo = stack.enter_context(Svo(svo_path, svo_scale))
            K, frames = svo.K, svo.images(lazy=use_gpu)
        else:
            bag = stack.enter_context(Bag(session.bag))
            K, frames = bag.camera_info(info_topic).K, bag.images(image_topic, lazy=use_gpu)
        if use_gpu:
            import open3d as o3d

            from .colour import GpuColourer, colour_frames

            m = o3d.io.read_triangle_mesh(str(mesh_path)) if mesh_path.exists() else None
            verts = np.asarray(m.vertices) if m is not None else np.empty((0, 3))
            col = GpuColourer(np.concatenate([xyz, verts]), K, max_depth, sharpen=3.0, exposure=exposure)
            stats = colour_frames(col, frames, tr, T_lo, rig.camera_time_offset, blur_skip=blur_skip)
            rgb_all, seen_all = col.result()
            rgb, seen = rgb_all[:len(xyz)], seen_all[:len(xyz)]
            if m is not None:
                m.vertex_colors = o3d.utility.Vector3dVector(rgb_all[len(xyz):] / 255.0)
                o3d.io.write_triangle_mesh(str(mesh_path), m)
                stats["mesh_coloured_pct"] = round(100 * float(seen_all[len(xyz):].mean()), 1)
        else:
            from collections import deque

            acc = colmap_mod.ColorAccumulator(xyz, K, max_depth, sharpen=sharpen)
            recent = deque(maxlen=31)
            stats = {"used": 0, "blurred": 0}
            for i, (stamp, bgr) in enumerate(frames):
                t = stamp + rig.camera_time_offset
                if i % every or not tr.covers(np.array([t]))[0]:
                    continue
                sh = colmap_mod.sharpness(bgr)
                recent.append(sh)
                med = float(np.median(recent))
                if len(recent) > 5 and sh < blur_skip * med:
                    stats["blurred"] += 1
                    continue
                acc.add(bgr, tr.interpolate(t)[0] @ T_lo, sharpness=float(np.clip(sh / max(med, 1e-6), 0.3, 2.0)))
                stats["used"] += 1
            rgb, seen = acc.result()
    out = session.products / "map_rgb.laz"
    write_laz(out, xyz, attrs.get("intensity"), rgb=rgb)
    stats["map_coloured_pct"] = round(100 * float(seen.mean()), 1)
    log.info("colorize (%s, %s): %s -> %s", source, "gpu" if use_gpu else "cpu", stats, out)
    return out


def texture(session: Session, image_topic: str = IMAGE_TOPIC, info_topic: str = CAMERA_INFO_TOPIC,
            source: str = "auto", svo_scale: float = 1.0, cell: int = 7, max_depth: float = 15.0,
            sharpen: float = 4.0, min_move: float = 0.1, min_turn: float = 8.0, blur_skip: float = 0.6,
            exposure: bool = True, holdout: int = 0) -> Path:
    """Image-textured mesh from mesh.ply and the camera -> products/textured/mesh.obj (+ .mtl, .png)."""
    from contextlib import ExitStack

    import open3d as o3d

    from . import texture as texture_mod
    from .colour import GpuColourer, colour_frames

    rig = session.rig
    tr = load_traj(session)
    m = o3d.io.read_triangle_mesh(str(session.products / "mesh.ply"))
    verts, faces = np.asarray(m.vertices), np.asarray(m.triangles)
    side, rc = texture_mod.atlas_layout(len(faces), cell)
    xyz, flat = texture_mod.texel_positions(verts, faces, side, rc, cell)
    svo_path = session.path / "zed.svo2"
    if source == "auto":
        source = "svo" if svo_path.exists() and svo_path.stat().st_size > 1_000_000 else "bag"
    with ExitStack() as stack:
        if source == "svo":
            from .io.svo import Svo

            svo = stack.enter_context(Svo(svo_path, svo_scale))
            K, frames = svo.K, svo.images(lazy=True)
        else:
            bag = stack.enter_context(Bag(session.bag))
            K, frames = bag.camera_info(info_topic).K, bag.images(image_topic, lazy=True)
        col = GpuColourer(xyz, K, max_depth, sharpen=sharpen, exposure=exposure)
        stats = colour_frames(col, frames, tr, rig.lidar_to_optical, rig.camera_time_offset, min_move, min_turn,
                              blur_skip, holdout)
    rgb, seen = col.result()
    atlas = texture_mod.fill_atlas(side, flat, rgb, seen)
    out = session.products / "textured"
    out.mkdir(exist_ok=True)
    m.triangle_uvs = o3d.utility.Vector2dVector(texture_mod.triangle_uvs(len(faces), side, rc, cell))
    m.triangle_material_ids = o3d.utility.IntVector(np.zeros(len(faces), np.int32).tolist())
    m.textures = [o3d.geometry.Image(atlas)]
    path = out / "mesh.obj"
    o3d.io.write_triangle_mesh(str(path), m, write_vertex_colors=False)
    stats |= {"texels": len(xyz), "atlas": side, "texels_seen_pct": round(100 * float(seen.mean()), 1)}
    (out / "texture.json").write_text(json.dumps(stats, indent=2))
    log.info("texture: %s -> %s", stats, path)
    return path


def extrinsic(session: Session, frames: int = 80, points: int = 150_000, apply: bool = False) -> dict:
    """Refine T_lidar_camera_optical by photometric consistency (products/extrinsic.json).

    Frames come from zed.svo2, or from the bag's image topic when the session has no usable SVO.

    ``apply`` writes the result into the session's rig.yaml (copy the line into the profile's
    rig.yaml for future captures).
    """
    import cv2
    import yaml

    from . import extrinsic as ext
    from .io.svo import Svo

    rig = session.rig
    tr = load_traj(session)
    xyz, _ = read_laz(session.map_path)
    xyz = xyz[np.random.default_rng(0).choice(len(xyz), min(points, len(xyz)), replace=False)]
    from contextlib import ExitStack

    svo_path = session.path / "zed.svo2"
    with ExitStack() as stack:
        if svo_path.exists() and svo_path.stat().st_size > 1_000_000:
            svo = stack.enter_context(Svo(svo_path, scale=0.25))
            K, n_frames, frames_it = svo.K, svo.frames, svo.images(lazy=True)
        else:  # no usable SVO (AGX wrapper v5.2.2): the bag's image topic, downscaled to ~480 px
            bag = stack.enter_context(Bag(session.bag))
            info = bag.camera_info(CAMERA_INFO_TOPIC)
            f = 480.0 / info.width
            K = info.K * np.array([[f, 1, f], [1, f, f], [1, 1, 1]])
            n_frames = len(bag.image_stamps(IMAGE_TOPIC))
            frames_it = ((st, lambda g=g: cv2.resize(g(), None, fx=f, fy=f, interpolation=cv2.INTER_AREA))
                         for st, g in bag.images(IMAGE_TOPIC, lazy=True))
        stride = max(1, n_frames // (frames * 3))
        cand = []
        for i, (stamp, get) in enumerate(frames_it):
            t = stamp + rig.camera_time_offset
            if i % stride or not tr.covers(np.array([t]))[0]:
                continue
            bgr = get()
            cand.append((colmap_mod.sharpness(bgr), t, cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)))
    # The sharper half, spread over the capture.
    cand = [c for c in cand if c[0] >= np.percentile([c[0] for c in cand], 50)]
    pick = cand[:: max(1, len(cand) // frames)][:frames]
    views = [ext.View(cv2.GaussianBlur(g.astype(np.float32), (5, 5), 1.0), tr.interpolate(t)[0]) for _, t, g in pick]
    T, report = ext.refine_extrinsic(xyz, views, K, rig.lidar_to_optical)
    from .geometry import to_xyz_quat

    p, q = to_xyz_quat(T)
    report |= {"views": len(views), "T_lidar_camera_optical": {"xyz": p.round(4).tolist(), "quat_xyzw": q.round(6).tolist()}}
    (session.products / "extrinsic.json").write_text(json.dumps(report, indent=2))
    if apply:
        rig_path = session.path / "rig.yaml"
        data = yaml.safe_load(rig_path.read_text()) or {}
        data["T_lidar_camera_optical"] = report["T_lidar_camera_optical"]
        rig_path.write_text(yaml.safe_dump(data, sort_keys=False))
    log.info("extrinsic: %s", report)
    return report


def mesh(session: Session, topic: str = POINTS_TOPIC, voxel: float = 0.03, trunc: float = 0.09,
         min_weight: float = 3.0, every: int = 1, min_range: float = 0.3, max_range: float = 30.0) -> Path:
    """TSDF fusion of every deskewed scan (VDBFusion) -> coloured surface mesh (products/mesh.ply).

    Each voxel averages all the passes that saw it, so noise and small pose jitter cancel instead of
    piling up as thick walls; surfaces seen fewer than ``min_weight`` times are dropped.
    Vertex colours come from map_rgb.laz (run ``dtp colorize`` first), else grey.
    """
    import open3d as o3d
    import vdbfusion
    from scipy.spatial import cKDTree

    vol = vdbfusion.VDBVolume(voxel, trunc, False)
    n = 0
    for ws in world_scans(session, topic, True, every, min_range, max_range):
        vol.integrate(ws.xyz, ws.origin)
        n += 1
    verts, tris = vol.extract_triangle_mesh(True, min_weight)
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(verts), o3d.utility.Vector3iVector(tris))
    rgb_path = session.products / "map_rgb.laz"
    if rgb_path.exists():
        cxyz, cattrs = read_laz(rgb_path)
        _, nn = cKDTree(cxyz).query(np.asarray(verts), k=1, workers=-1)
        m.vertex_colors = o3d.utility.Vector3dVector(cattrs["rgb"][nn] / 255.0)
    m.compute_vertex_normals()
    out = session.products / "mesh.ply"
    o3d.io.write_triangle_mesh(str(out), m)
    log.info("mesh: %d scans -> %d vertices, %d triangles -> %s", n, len(verts), len(tris), out)
    return out


def colmap(session: Session, image_topic: str = IMAGE_TOPIC, info_topic: str = CAMERA_INFO_TOPIC,
           every: int = 1, min_translation: float = 0.0, min_rotation_deg: float = 0.0,
           points_voxel: float = 0.05, max_points: int = 2_000_000, out_dir: str | Path | None = None) -> Path:
    import cv2

    rig = session.rig
    T_lo = rig.lidar_to_optical
    tr = load_traj(session)
    out = Path(out_dir) if out_dir else session.products / "colmap"
    (out / "images").mkdir(parents=True, exist_ok=True)
    with Bag(session.bag) as bag:
        cam = bag.camera_info(info_topic)
        if np.abs(cam.D).max(initial=0) > 1e-6:
            log.warning("camera_info has non-zero distortion; COLMAP export assumes rectified images")
        stamps = bag.image_stamps(image_topic)
        chosen = set(colmap_mod.select_frames(stamps, tr, T_lo, rig.camera_time_offset,
                                              every, min_translation, min_rotation_deg))
        exported, for_color = [], []
        for i, (stamp, bgr) in enumerate(bag.images(image_topic)):
            if i not in chosen:
                continue
            T_wc = tr.interpolate(stamp + rig.camera_time_offset)[0] @ T_lo
            name = f"{stamp:.6f}.jpg"
            cv2.imwrite(str(out / "images" / name), bgr)
            exported.append(colmap_mod.ExportedImage(len(exported) + 1, name, stamp, T_wc))
            if len(for_color) < 200:
                for_color.append((bgr, T_wc))
    xyz, _ = read_laz(session.best_map())
    if points_voxel:
        from .mapping import voxel_downsample

        xyz = voxel_downsample(xyz, points_voxel)
    if len(xyz) > max_points:
        xyz = xyz[np.random.default_rng(0).choice(len(xyz), max_points, replace=False)]
    rgb = colmap_mod.colorize(xyz, for_color[:: max(1, len(for_color) // 50)], cam.K)
    colmap_mod.write_model(out, cam, exported, xyz, rgb)
    (out / "export.json").write_text(json.dumps({"images": len(exported), "points": len(xyz),
                                                 "camera_time_offset": rig.camera_time_offset}, indent=2))
    log.info("colmap: %d images, %d points -> %s", len(exported), len(xyz), out)
    return out
