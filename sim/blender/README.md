# sim/blender: world, device, walk, cameras, results (Blender 5.2)

This folder holds the Blender side of the simulation; see `docs/design/2026-10-18-blender-sim-design.md`. Every script runs headless through **`bl.sh`**, which forces the NVIDIA GPU. Without it, EEVEE on a hybrid laptop renders on the Intel iGPU and is about 6× slower.

```bash
R=~/dt_data/sim/demo
python3 sim/blender/fetch_assets.py                                    # optional CC0 textures -> sim/assets/
sim/blender/bl.sh gen_world.py   --out $R --variant day1               # + --variant day2
sim/blender/bl.sh gen_device.py  --out $R --profile mid360 --render    # or airy; writes rig_sim.yaml/.json
sim/blender/bl.sh walk.py        --out $R --variant day1 --duration 90 # trajectory.csv (1 kHz) + preview .blend
sim/blender/bl.sh render_cams.py --out $R --variant day1 [--right] [--max-frames N] [--hd]
# ... dt_sim bag / stack / dt_sim eval ...
sim/blender/bl.sh show_result.py --out $R --variant day1 --map $R/products/map_eval.npz \
    --est-traj $R/products/traj_est_world.tum --diff $R/products/diff --render
```

## Outputs (design §3 contracts)

| File | From | Notes |
|---|---|---|
| `world/world_<v>.blend`, `world/mesh_<v>.npz` | gen_world | npz keys: `vertices` f32 (V,3), `triangles` i32 (F,3), `face_class` u8, `face_reflectivity` f32, `classes` (JSON id→name) |
| `device/device_<profile>.blend`, `device/hero_<profile>{,_side,_full}.png` | gen_device | Collection `DT_device` with root empty `DT1` (= base_link) and sensor empties. `DT_hero_extras` holds the hip box and tether. |
| `rig_sim.yaml` (+ `rig_sim.json`) | gen_device | `T_base_lidar`, `T_lidar_imu`, `T_lidar_camera`, `T_lidar_camera_optical` (`{xyz, quat_xyzw}`), `camera {width, height, fx, fy, cx, cy, baseline}`, `profile`. Blender has no PyYAML, so the scripts read the `.json` twin. |
| `trajectory.csv` | walk | `t,x,y,z,qx,qy,qz,qw` of base_link at 1 kHz. The first 3 s are static. Clearance is checked against both variants. |
| `world/walk_preview_<v>.blend` | walk | World plus the animated device at 25 fps, with a `FollowCam` |
| `camera/{left,right}/%06d.png`, `camera/frames.csv` | render_cams | Rectified pinhole images, OpenCV optical frame, intrinsics from rig. `frames.csv` lists frames whose left PNG exists. Re-running resumes. |
| `world/result_<v>.blend`, `result_<v>{,_inside}.png` | show_result | Map coloured by `error` (green 0 → yellow 5 cm → red ≥ 10 cm) or by height. GT path green, estimate orange, diff added green / removed red, world in translucent clay. |

## Frames and conventions
- **World:** Z up, metres. **base_link:** grip centre, ROS axes (x forward, y left, z up).
- **lidar_link:** pitched 30° forward on the head wedge.
  - Mid-360 origin is 47 mm above the mounting face.
  - Airy origin is 35 mm above the mounting face (assumed).
  - `T_lidar_imu`: Mid-360 (0.011, 0.02329, −0.04412) with axes equal to the lidar axes; Airy 0.
- **Camera:** ZED 2i HD720, fx = fy = 527 (horizontal FoV about 101°), principal point at the image centre, baseline 0.12 m. The left optical frame is the left lens (+y side). The ZED is pitched 10° down.
- **OpenCV optical → Blender camera:** `diag(1, −1, −1)` (`common.OPTICAL_TO_BLENDER_CAM`). Intrinsics are matched through lens and shift (`common.setup_camera`).

## Site (common.SITE)
- About 60×40 m, fenced, with a south gate gap.
- Three-storey RC frame: column grid x ∈ {−12…12} step 6, y ∈ {−4…14} step 6, storey 3.2 m. Stair core in bay x 6–12, y 8–14.
- Excavation pit at x 16–26, y −16…−6, 2.5 m deep with 1:1 batters.
- Site office at (−25, 14).
- **day1:** roof formwork with two rebar mesh layers and shores below. Scaffold on the north façade x −12.5…0. Excavator at the pit. Two spoil piles.
- **day2:** roof poured with starter bars; shores and formwork struck. More blockwork, scaffold to x 12.5, excavator moved to (−21, 6), one pile reduced and one removed, two new pallet stacks.

## Measured on this machine (RTX 4060 Laptop, Blender 5.2.2)

| Step | Result |
|---|---|
| gen_world | ~2 s. day1 is 42.7k triangles, day2 39.3k. |
| gen_device `--render` | 3 Cycles hero shots ≈ 75 s. |
| walk | ~8 s for 90 s at 1 kHz (90.8 m loop, 1.09 m/s). |
| Walk motion | Acceleration p50 2.4 / max 4.0 m/s². Angular rate p99 42 / max 54 °/s. |
| render_cams (EEVEE 8 spp, 720p) | ~0.85 s per image after a ~5–20 s shader warm-up. A 90 s walk at 15 Hz with stereo (2,700 images) takes about 40 min; left only, about 20 min. |
| show_result `--render` | ~20 s for 2 views with 400k points. |

## Notes
- Blender 5.x specifics used here:
  - `BLENDER_EEVEE` engine id.
  - Sky types `SINGLE_SCATTERING` / `MULTIPLE_SCATTERING`.
  - Principled socket names `Emission Color`, `Coat Weight` and `Specular IOR Level`.
  - Materials and worlds come with node trees by default.
  - Point clouds use Geometry Nodes `Mesh to Points` with a `col` color attribute.
- `show_result` can't read `.laz` (Blender has no laspy). Pass `.npz` or `.ply` instead; `dt_sim eval` is expected to write `map_eval.npz` in the world frame.
- `sim/assets/` holds the CC0 textures (≈6 MB), credited in `CREDITS.md`.
