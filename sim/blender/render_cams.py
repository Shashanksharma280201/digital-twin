"""Render the ZED 2i stereo images along the walk (design §5, render_cams).

sim/blender/bl.sh render_cams.py --out RUN --variant day1 [--rate 15] [--hd] [--max-frames N] [--right]

Camera pose = T_world_base(t) · T_base_lidar · T_lidar_camera_optical (rig_sim.json),
converted from OpenCV optical axes to Blender camera axes. Intrinsics are matched
exactly via lens/shift. Resumable: existing PNGs are skipped. camera/frames.csv
lists every frame whose left image exists.
"""
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bpy  # noqa: E402
import numpy as np  # noqa: E402
from mathutils import Matrix  # noqa: E402

import common as C  # noqa: E402


def main():
    args = C.parse_args(lambda p: (
        p.add_argument("--out", required=True),
        p.add_argument("--variant", default="day1", choices=["day1", "day2", "pillar"]),
        p.add_argument("--rate", type=float, default=15.0),
        p.add_argument("--hd", action="store_true", help="1920x1080 (updates rig_sim camera block)"),
        p.add_argument("--max-frames", type=int, default=None),
        p.add_argument("--right", action="store_true", help="also render the right camera"),
        p.add_argument("--samples", type=int, default=8),
    ))
    if "__EGL_VENDOR_LIBRARY_FILENAMES" not in os.environ:
        print("[render_cams] WARNING: NVIDIA EGL not forced; EEVEE may be very slow (use sim/blender/bl.sh)")
    run = C.run_dir(args.out)
    rig = C.load_rig_json(run)
    if args.hd and rig["camera"]["width"] != 1920:
        s = 1920 / rig["camera"]["width"]
        cam = rig["camera"]
        rig["camera"] = {**cam, "width": 1920, "height": 1080, "fx": cam["fx"] * s, "fy": cam["fy"] * s,
                         "cx": cam["cx"] * s, "cy": cam["cy"] * s}
        C.write_rig_yaml(run / "rig_sim.yaml", rig)

    bpy.ops.wm.open_mainfile(filepath=str(run / "world" / f"world_{args.variant}.blend"))
    scene = bpy.context.scene
    C.setup_eevee(scene, rig["camera"]["width"], rig["camera"]["height"], args.samples)
    T_base_opt = C.pose_to_matrix(rig["T_base_lidar"]) @ C.pose_to_matrix(rig["T_lidar_camera_optical"])
    baseline = rig["camera"]["baseline"]

    cams = {}
    for side in (["left", "right"] if args.right else ["left"]):
        cam = bpy.data.objects.new(f"ZED_{side}", bpy.data.cameras.new(f"ZED_{side}"))
        scene.collection.objects.link(cam)
        C.setup_camera(cam, rig)
        cams[side] = cam
        (run / "camera" / side).mkdir(parents=True, exist_ok=True)

    ts, xyz, q = C.load_trajectory(run)
    times = np.arange(0.0, ts[-1] + 1e-9, 1.0 / args.rate)
    if args.max_frames:
        times = times[:args.max_frames]
    rendered, t0, n_new = [], time.time(), 0
    for k, t in enumerate(times):
        T_world_opt = C.interp_pose(t, ts, xyz, q) @ T_base_opt
        for side, cam in cams.items():
            T = T_world_opt if side == "left" else T_world_opt @ Matrix.Translation((baseline, 0, 0))
            path = run / "camera" / side / f"{k:06d}.png"
            if path.exists():
                continue
            cam.matrix_world = T @ C.OPTICAL_TO_BLENDER_CAM
            scene.camera = cam
            scene.render.filepath = str(path)
            bpy.ops.render.render(write_still=True)
            n_new += 1
        if (run / "camera" / "left" / f"{k:06d}.png").exists():
            rendered.append((k, t))
        if n_new and k % 25 == 0:
            print(f"[render_cams] frame {k}/{len(times)}  {(time.time() - t0) / n_new:.2f}s/image")

    with open(run / "camera" / "frames.csv", "w") as f:
        f.write("index,t\n")
        for k, t in rendered:
            f.write(f"{k},{t:.6f}\n")
    per = (time.time() - t0) / max(n_new, 1)
    print(f"[render_cams] {len(rendered)} frames listed, {n_new} images rendered, {per:.2f}s/image "
          f"-> {run / 'camera'}")


if __name__ == "__main__":
    main()
