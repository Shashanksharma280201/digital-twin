"""dt_sim: simulate the DT-1 handheld in a Blender-generated world and evaluate the stack.

dt_sim info RUN
dt_sim bag  RUN --profile mid360|airy [--variant day1] [--no-camera] [--duration S] [--seed N]
dt_sim eval RUN --session S [--map PATH] [--traj PATH] [--variant day1]
dt_sim gtmap RUN --profile mid360|airy [--variant day1]
"""

from __future__ import annotations

import argparse
import json
import logging
import shutil
import sys
import time
from pathlib import Path

import numpy as np

from .run import EPOCH0, RunDir


def _epoch0(run: RunDir, override: float | None) -> float:
    if override is not None:
        return override
    info = run.root / "bag_info.json"
    if info.exists():
        return float(json.loads(info.read_text()).get("epoch0", EPOCH0))
    return EPOCH0


def cmd_info(args) -> int:
    run = RunDir(args.run)
    print(f"run: {run.root}")
    for p in sorted((run.root / "world").glob("mesh_*.npz")):
        d = np.load(p)
        print(f"  {p.name}: {len(d['vertices'])} vertices, {len(d['triangles'])} triangles")
    if run.trajectory_path.exists():
        data = np.loadtxt(run.trajectory_path, delimiter=",", skiprows=1, ndmin=2)
        length = np.linalg.norm(np.diff(data[:, 1:4], axis=0), axis=1).sum()
        print(f"  trajectory: {len(data)} samples, {data[-1, 0] - data[0, 0]:.1f} s, {length:.1f} m")
    if run.rig_path.exists():
        sim = run.load_rig()
        print(f"  rig_sim: profile {sim.profile}, camera {sim.camera.width}x{sim.camera.height} "
              f"fx {sim.camera.fx:.1f}")
    from .camera import load_frames
    print(f"  camera frames: {len(load_frames(run.camera_dir))}")
    for name in ("bag_info.json", "eval.json"):
        if (run.root / name).exists():
            print(f"  {name}: present")
    return 0


def cmd_bag(args) -> int:
    from .bag import BagOptions, write_bag

    run = RunDir(args.run)
    out = Path(args.out) if args.out else run.root / "bag"
    if out.exists():
        if not args.overwrite:
            print(f"{out} exists (use --overwrite)", file=sys.stderr)
            return 1
        shutil.rmtree(out)
    opts = BagOptions(profile=args.profile, variant=args.variant, camera=not args.no_camera,
                      duration=args.duration, seed=args.seed, ring_bias_mm=args.ring_bias_mm,
                      epoch0=args.epoch0 if args.epoch0 is not None else EPOCH0)
    t = time.time()
    info = write_bag(run, out, opts)
    info["wall_time_s"] = round(time.time() - t, 1)
    (run.root / "bag_info.json").write_text(json.dumps(info, indent=2))
    print(json.dumps(info, indent=2))
    return 0


def cmd_eval(args) -> int:
    from .eval import evaluate

    run = RunDir(args.run)
    result = evaluate(run, Path(args.session) if args.session else None,
                      Path(args.map) if args.map else None,
                      Path(args.traj) if args.traj else None,
                      args.variant, _epoch0(run, args.epoch0),
                      Path(args.diff) if args.diff else None)
    tr = result["trajectory"]
    print(f"trajectory ({tr['source']}, {tr['duration_s']:.1f} s, {tr['path_length_m']:.1f} m)")
    for key in ("anchored", "aligned"):
        m = tr[key]
        print(f"  ATE {key:9s} rmse {m['rmse_m']:.3f} m  max {m['max_m']:.3f} m  "
              f"rot {m['rot_mean_deg']:.2f} deg")
    print(f"  drift (anchored, final) {tr['anchored']['drift_pct']:.2f} % of path")
    if "map" in result:
        m = result["map"]
        print(f"map ({m['points']} pts): mean {m['mean_m'] * 100:.1f} cm, p50 {m['p50_m'] * 100:.1f} cm, "
              f"p95 {m['p95_m'] * 100:.1f} cm, <2cm {m['within_2cm_pct']:.0f} %, "
              f"<5cm {m['within_5cm_pct']:.0f} %, completeness {m['completeness_pct']:.0f} %")
    if "map_surface" in result:
        m = result["map_surface"]
        print(f"map shape: mean {m['mean_m'] * 1000:.1f} mm, p50 {m['p50_m'] * 1000:.1f} mm, "
              f"p90 {m['p90_m'] * 1000:.1f} mm, completeness {m['completeness_pct']:.0f} %, scale {m['scale_error_ppm']:+.0f} ppm, "
              f"tape {m['tape_median_mm']:.1f} mm (p95 {m['tape_p95_mm']:.1f} mm)")
    print(f"-> {run.root / 'eval.json'}")
    return 0


def cmd_gtmap(args) -> int:
    from .eval import export_gt_map

    run = RunDir(args.run)
    out = Path(args.out) if args.out else run.products / f"gt_map_{args.variant}.laz"
    path = export_gt_map(run, args.profile, out, args.variant, args.voxel)
    print(path)
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="dt_sim", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-v", "--verbose", action="store_true")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("info", help="summarise a run directory")
    p.add_argument("run")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("bag", help="simulate sensors and write a driver-native MCAP bag")
    p.add_argument("run")
    p.add_argument("--profile", choices=["mid360", "airy"], default="mid360")
    p.add_argument("--variant", default="day1")
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--epoch0", type=float, default=None)
    p.add_argument("--out", default=None, help="bag directory (default RUN/bag)")
    p.add_argument("--ring-bias-mm", type=float, default=0.0,
                   help="Airy: std of a per-ring range error; the truth goes to RUN/ring_bias_truth.json")
    p.add_argument("--overwrite", action="store_true")
    p.set_defaults(func=cmd_bag)

    p = sub.add_parser("eval", help="evaluate a stack run against ground truth")
    p.add_argument("run")
    p.add_argument("--session", default=None, help="dt_session directory (bag with /Odometry)")
    p.add_argument("--map", default=None, help="map file (.laz/.ply/.npz); default SESSION/products/map.laz")
    p.add_argument("--traj", default=None, help="TUM trajectory of lidar_link (e.g. dtp traj output)")
    p.add_argument("--variant", default="day1")
    p.add_argument("--epoch0", type=float, default=None)
    p.add_argument("--diff", default=None,
                   help="dtp diff dir whose reference is this session; exported to world frame")
    p.set_defaults(func=cmd_eval)

    p = sub.add_parser("gtmap", help="export the noise-free map the device would see")
    p.add_argument("run")
    p.add_argument("--profile", choices=["mid360", "airy"], default="mid360")
    p.add_argument("--variant", default="day1")
    p.add_argument("--voxel", type=float, default=0.03)
    p.add_argument("--out", default=None)
    p.set_defaults(func=cmd_gtmap)

    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
