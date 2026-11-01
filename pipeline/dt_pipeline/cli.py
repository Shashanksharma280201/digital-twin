"""``dtp``: offline digital-twin pipeline CLI."""

from __future__ import annotations

import argparse
import json
import logging
import sys

from . import steps
from .io import CAMERA_INFO_TOPIC, IMAGE_TOPIC, ODOM_TOPIC, POINTS_TOPIC
from .session import Session


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(prog="dtp", description=__doc__)
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("info", help="summarise a session's bag")
    s.add_argument("session")

    s = sub.add_parser("traj", help="write products/traj_lidar.tum (T_world_lidar)")
    s.add_argument("session")
    s.add_argument("--source", choices=["fastlio", "glim"], default="fastlio")
    s.add_argument("--glim-dump", help="GLIM dump dir (default <session>/glim_dump)")
    s.add_argument("--odom-topic", default=ODOM_TOPIC)

    s = sub.add_parser("ingest", help="read the bag once: scan cache + traj_lidar.tum (fast path for every later step)")
    s.add_argument("session")
    s.add_argument("--topic", default=POINTS_TOPIC)
    s.add_argument("--odom-topic", default=ODOM_TOPIC)
    s.add_argument("--workers", type=int, default=8)
    s.add_argument("--keep-mixed", action="store_true", help="keep mixed-pixel points at depth edges")

    s = sub.add_parser("fuse", help="GPU: map.laz + TSDF mesh.ply in one pass over the scan cache")
    s.add_argument("session")
    s.add_argument("--voxel", type=float, default=0.02)
    s.add_argument("--mesh-voxel", type=float, default=0.02)
    s.add_argument("--min-weight", type=float, default=3.0)
    s.add_argument("--self-zone", type=float, default=0.8, help="drop returns closer than this (operator, rig)")
    s.add_argument("--no-mesh", action="store_true")
    s.add_argument("--min-hits", type=int, default=3,
                   help="voxels with fewer returns go to map_loose.laz (1 = keep all in map.laz)")

    s = sub.add_parser("ringcal", help="learn per-ring LiDAR offsets against mesh.ply (after fuse); next ingest applies them")
    s.add_argument("session")
    s.add_argument("--topic", default=POINTS_TOPIC)
    s.add_argument("--every", type=int, default=12, help="use every Nth scan")

    s = sub.add_parser("polish", help="align scans to the fused surface (after fuse), updates traj_lidar.tum")
    s.add_argument("session")
    s.add_argument("--every", type=int, default=5)

    s = sub.add_parser("refine", help="pose-graph + loop closures over traj_lidar.tum (run after traj)")
    s.add_argument("session")
    s.add_argument("--topic", default=POINTS_TOPIC)
    s.add_argument("--every", type=int, default=2, help="use every Nth scan for keyframe submaps")
    s.add_argument("--loop-radius", type=float, default=2.0)

    s = sub.add_parser("map", help="deskewed voxel map -> products/map.laz")
    s.add_argument("session")
    s.add_argument("--topic", default=POINTS_TOPIC)
    s.add_argument("--voxel", type=float, default=0.05)
    s.add_argument("--no-deskew", action="store_true")
    s.add_argument("--min-range", type=float, default=0.3)
    s.add_argument("--max-range", type=float, default=60.0)
    s.add_argument("--copc", action="store_true", help="also write map.copc.laz (needs PDAL)")

    s = sub.add_parser("clean", help="split map into static/dynamic")
    s.add_argument("session")
    s.add_argument("--topic", default=POINTS_TOPIC)
    s.add_argument("--voxel", type=float, default=0.1)
    s.add_argument("--method", choices=["auto", "dufomap", "visibility"], default="auto")
    s.add_argument("--every", type=int, default=2, help="use every Nth scan")

    s = sub.add_parser("align", help="register a session's map to a reference session")
    s.add_argument("ref")
    s.add_argument("session")
    s.add_argument("--no-coarse", action="store_true", help="skip FPFH+RANSAC (sessions already roughly aligned)")
    s.add_argument("--max-corr", type=float, default=0.5)

    s = sub.add_parser("diff", help="change detection against a reference session")
    s.add_argument("ref")
    s.add_argument("session")
    s.add_argument("--voxel", type=float, default=0.05)
    s.add_argument("--m3c2", action="store_true", help="also run py4dgeo M3C2")

    s = sub.add_parser("colorize", help="colour the LiDAR map from the camera images (map_rgb.laz)")
    s.add_argument("session")
    s.add_argument("--image-topic", default=IMAGE_TOPIC)
    s.add_argument("--info-topic", default=CAMERA_INFO_TOPIC)
    s.add_argument("--every", type=int, default=1, help="use every Nth image")
    s.add_argument("--max-depth", type=float, default=15.0)
    s.add_argument("--source", choices=["auto", "svo", "bag"], default="auto",
                   help="auto = the session's zed.svo2 when present (full frame rate), else the bag topic")
    s.add_argument("--svo-scale", type=float, default=0.5, help="image downscale for SVO frames")
    s.add_argument("--blur-skip", type=float, default=0.6, help="skip frames below this x running-median sharpness")
    s.add_argument("--cpu", action="store_true", help="CPU path (no PyTorch)")
    s.add_argument("--no-exposure", action="store_true", help="no per-frame exposure compensation")

    s = sub.add_parser("texture", help="image-textured mesh from mesh.ply -> products/textured/mesh.obj")
    s.add_argument("session")
    s.add_argument("--source", choices=["auto", "svo", "bag"], default="auto")
    s.add_argument("--cell", type=int, default=7, help="texels per triangle cell side, including its border")
    s.add_argument("--sharpen", type=float, default=4.0, help="how strongly the best view wins")
    s.add_argument("--no-exposure", action="store_true")
    s.add_argument("--holdout", type=int, default=0, help="never use every Nth frame (for scoring)")

    s = sub.add_parser("extrinsic", help="target-free LiDAR->camera refinement from zed.svo2 (photometric)")
    s.add_argument("session")
    s.add_argument("--frames", type=int, default=80)
    s.add_argument("--apply", action="store_true", help="write the result into the session's rig.yaml")

    s = sub.add_parser("mesh", help="TSDF-fused coloured surface mesh -> products/mesh.ply")
    s.add_argument("session")
    s.add_argument("--topic", default=POINTS_TOPIC)
    s.add_argument("--voxel", type=float, default=0.03)
    s.add_argument("--trunc", type=float, default=0.09)
    s.add_argument("--min-weight", type=float, default=3.0)
    s.add_argument("--every", type=int, default=1)
    s.add_argument("--min-range", type=float, default=0.3)

    s = sub.add_parser("colmap", help="export COLMAP model for Gaussian splatting")
    s.add_argument("session")
    s.add_argument("--image-topic", default=IMAGE_TOPIC)
    s.add_argument("--info-topic", default=CAMERA_INFO_TOPIC)
    s.add_argument("--every", type=int, default=1)
    s.add_argument("--min-translation", type=float, default=0.2)
    s.add_argument("--min-rotation", type=float, default=10.0, help="degrees")
    s.add_argument("--out")

    a = p.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO, format="%(levelname)s %(message)s")

    if a.cmd == "info":
        print(json.dumps(steps.info(a.session), indent=2))
    elif a.cmd == "traj":
        steps.traj(Session(a.session), a.source, a.glim_dump, a.odom_topic)
    elif a.cmd == "ingest":
        print(json.dumps(steps.ingest(Session(a.session), a.topic, a.odom_topic, a.workers, not a.keep_mixed)))
    elif a.cmd == "fuse":
        print(json.dumps(steps.fuse(Session(a.session), a.voxel, a.mesh_voxel, None, a.min_weight, a.self_zone,
                                    mesh=not a.no_mesh, min_hits=a.min_hits)))
    elif a.cmd == "ringcal":
        print(json.dumps(steps.ringcal(Session(a.session), a.topic, a.every)))
    elif a.cmd == "polish":
        print(json.dumps(steps.polish(Session(a.session), a.every)))
    elif a.cmd == "refine":
        print(json.dumps(steps.refine(Session(a.session), a.topic, a.every, a.loop_radius)))
    elif a.cmd == "map":
        steps.build(Session(a.session), a.topic, a.voxel, not a.no_deskew, a.min_range, a.max_range, a.copc)
    elif a.cmd == "clean":
        steps.clean(Session(a.session), a.topic, a.voxel, a.method, a.every)
    elif a.cmd == "align":
        res = steps.align(Session(a.ref), Session(a.session), coarse=not a.no_coarse, max_corr=a.max_corr)
        print(res.T_ref_src)
    elif a.cmd == "diff":
        print(json.dumps(steps.diff(Session(a.ref), Session(a.session), a.voxel, a.m3c2), indent=2))
    elif a.cmd == "colorize":
        steps.colorize(Session(a.session), a.image_topic, a.info_topic, a.every, a.max_depth, a.source,
                       a.svo_scale, a.blur_skip, use_gpu=not a.cpu, exposure=not a.no_exposure)
    elif a.cmd == "texture":
        print(steps.texture(Session(a.session), source=a.source, cell=a.cell, sharpen=a.sharpen,
                            exposure=not a.no_exposure, holdout=a.holdout))
    elif a.cmd == "extrinsic":
        print(json.dumps(steps.extrinsic(Session(a.session), a.frames, apply=a.apply), indent=2))
    elif a.cmd == "mesh":
        steps.mesh(Session(a.session), a.topic, a.voxel, a.trunc, a.min_weight, a.every, a.min_range)
    elif a.cmd == "colmap":
        steps.colmap(Session(a.session), a.image_topic, a.info_topic, a.every, a.min_translation,
                     a.min_rotation, out_dir=a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
