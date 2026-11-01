# Blender Simulation World: Design

| | |
|---|---|
| Date | 2026-10-18 |
| Status | v1 |
| Research | `research/08-sim-assets-device.md`, `research/09-blender-ros-bridge.md` |
| Builds on | `docs/design/2026-10-18-dt-capture-stack-design.md` (canonical contract, profiles, pipeline) |

## 1. Intent
Before touching hardware, see the **end product working end to end**. A virtual DT-1 handheld scanner (Mid-360 or Airy + ZED 2i) is carried through a procedurally generated **construction site in Blender**. It produces sensor data identical in format to the real drivers. That data runs through the **unchanged** capture stack (bridge → FAST-LIO → recorder → `dtp`), and the resulting digital twin is shown back in Blender next to the ground truth, live as it is built and as a final render.

### Goals
1. **World:** a procedural, seeded construction site with a `progress` parameter. Day 1 and day 2 variants exist so we can test change detection.
2. **Device:** a parametric model of the handheld "DT-1" (what the product looks like). Sensor frames are empties, and this model is the single source of truth for the sim extrinsics.
3. **Sensor sim:** driver-native bags (Livox CustomMsg or RoboSense XYZIRT, raw IMU in g, ZED compressed images + camera_info) with realistic handheld motion, per-point motion distortion and noise.
4. **Bridge (ROS 2 ↔ Blender):** live view in Blender of the device's estimated pose and the map growing while the stack runs.
5. **Evaluation:** trajectory error against ground truth (ATE) and map accuracy against the GT mesh. A day 1 vs day 2 change map.
6. **Showcase:** a rendered image or turntable of the device and of the final twin overlaid on the world.

### Non-goals
- Real-time Blender rendering. The camera is rendered offline, at ~0.3 s per frame.
- Physics or a game engine. Gazebo replacement is limited to sensing a static or keyframed world.
- Photoreal LiDAR effects (multipath, rain). There is range noise, dropout and incidence-dependent intensity only.

### Anti-goals
- Simulated topics that differ from the real drivers. The point is to exercise the real bridge and stack.
- Extrinsics duplicated by hand between Blender and the stack. Blender exports `rig_sim.yaml`.

## 2. Architecture

```
 Blender 5.2 (Python 3.13)                        Python 3.10 + ROS 2 Humble
 ─────────────────────────                        ──────────────────────────
 gen_world.py   ─► world_<v>.blend, mesh_<v>.npz ─┐
 gen_device.py  ─► device.blend, rig_sim.yaml    ─┤
 walk.py        ─► trajectory.csv (1 kHz)        ─┼─► dt_sim bag   ─► run/bag (MCAP, driver-native topics)
 render_cams.py ─► camera/{left,right}/*.png     ─┘                        │ ros2 bag play --clock
                                                                           ▼
                                                   capture stack (driver:=false, use_sim_time) ─► dt_session record ─► dtp traj/map/diff
                                                                           │
 dt_live addon  ◄──── TCP 127.0.0.1:5577 ────────── dt_blender_bridge (rclpy) ◄─ /Odometry, /cloud_registered, /sim/ground_truth/odometry
 show_result.py ◄── products/map.laz, diff/*.laz, eval.json ◄── dt_sim eval
```

**Why this split:**
- Blender does what it is good at: modelling, animation, rendering, display.
- Ray casting runs in Python 3.10 with Open3D Embree at 3.5M rays/s, which is 30× faster than Blender's BVH.
- Sensor data travels as MCAP (deterministic, sim time).
- The live bridge carries only light display data, so a dependency-free TCP protocol is enough.

## 3. File contracts (the interface between parts)

All in a run directory `RUN = $DT_DATA/sim/<name>/`. Units are metres and seconds. The world frame is Z-up, right-handed (Blender world). `t` is in seconds from the start of the run. Bag stamps are `epoch0 + t`, with `epoch0 = 1760000000.0` by default.

| File | Producer | Content |
|---|---|---|
| `world/world_<variant>.blend` | gen_world | Scene. Variants: `day1`, `day2` |
| `world/mesh_<variant>.npz` | gen_world | `vertices (V,3) float32`, `triangles (F,3) int32`, `face_class (F,) uint8`, `face_reflectivity (F,) float32` (0–1), `classes` (JSON string: id → name) |
| `device/device_<profile>.blend` | gen_device | Device model with sensor empties |
| `rig_sim.yaml` | gen_device | rig.yaml schema (`pipeline/dt_pipeline/rig.py`) plus `camera: {width, height, fx, fy, cx, cy, baseline}` and `profile`. `T_base_lidar`, `T_lidar_imu` (IMU position, axes = lidar axes), `T_lidar_camera_optical` (left optical, OpenCV axes) |
| `trajectory.csv` | walk | header `t,x,y,z,qx,qy,qz,qw`, 1 kHz, the pose of `base_link` (device handle) in world |
| `camera/frames.csv` | render_cams | header `index,t`, 15 Hz (configurable) |
| `camera/left/%06d.png`, `camera/right/%06d.png` | render_cams | rectified pinhole images, intrinsics as in `rig_sim.yaml` |
| `bag/` | dt_sim bag | MCAP, topics in §4 |
| `eval.json`, `products/` | dt_sim eval / dtp | metrics and maps |

**Camera convention:** OpenCV optical axes (z forward, x right, y down). Blender camera = optical frame rotated by `diag(1, −1, −1)`.

## 4. Simulated topics (identical to the real drivers)
| Profile | Topic | Type | Notes |
|---|---|---|---|
| mid360 | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 10 Hz, 20k pts. `mid360.csv` pattern at 200 kHz. `offset_time` in ns. `line = i % 4`, `tag = 0x10`. Frame `livox_frame` |
| mid360 | `/livox/imu` | `sensor_msgs/Imu` | 200 Hz, **acceleration in g**, IMU axes = lidar axes |
| airy | `/rslidar_points` | PointCloud2 XYZIRT | x, y, z, intensity f32; ring u16; timestamp f64 absolute s. 96 rings, 0–90° elevation, 10 Hz |
| airy | `/rslidar_imu_data` | `sensor_msgs/Imu` | 200 Hz, g, **raw Airy axes**: `a_raw = Rᵀ a_lidar` with the bridge's R, so the real bridge rotation is exercised |
| both | `/zed/zed_node/rgb/color/rect/image/compressed` + `.../camera_info` | CompressedImage (jpeg), CameraInfo | 15 Hz, frame `zed_left_camera_optical_frame` |
| both | `/sim/ground_truth/odometry` | `nav_msgs/Odometry` | 100 Hz, `world → base_link` |

Ground truth is never published on `/tf`, to avoid fighting FAST-LIO.

## 5. Components
| Path | Runtime | Responsibility |
|---|---|---|
| `sim/blender/common.py` | Blender | materials (Principled, 5.x socket names), seeded RNG, export helpers |
| `sim/blender/gen_world.py` | Blender | Procedural site: terrain, excavation, a 3-storey RC frame (columns, slabs, beams), stairs, blockwork, rebar, formwork + shores, scaffolding, props (cones, barriers, pallets, fencing), parametric excavator. `--progress day1\|day2` changes slabs, walls, scaffold and stockpiles. Optional CC0 textures via `fetch_assets.py` (Poly Haven / ambientCG), with procedural fallback. |
| `sim/blender/gen_device.py` | Blender | DT-1 handheld: handle, head, ZED 2i, LiDAR on a 30° wedge (Mid-360 or Airy), phone clamp, hip box + tether. Empties `base_link`, `lidar_link`, `imu_link`, `zed_left_optical`. Exports `rig_sim.yaml`. Hero render. |
| `sim/blender/walk.py` | Blender | Operator path (waypoint spline through the site, inside the building), walking dynamics, static start, export `trajectory.csv`. Keyframes the device in the world `.blend` for previews. |
| `sim/blender/render_cams.py` | Blender | EEVEE (NVIDIA EGL), stereo, renders at `frames.csv` times |
| `sim/blender/show_result.py` | Blender | Imports GT world + reconstructed map + estimated and GT paths + change map; renders the final output |
| `sim/blender/addon/dt_live/` | Blender add-on | Live bridge client: TCP, timer, updates `DT_Estimated` pose, `DT_Map` points, GT path |
| `sim/dt_sim/` (pip package) | Python 3.10 | `dt_sim bag` (LiDAR ray casting, IMU, images → MCAP), `dt_sim eval` (ATE, map-to-mesh, change detection) |
| `ros2_ws/src/dt_blender_bridge` | ROS 2 | rclpy node: subscribes to odometry, cloud, GT; voxel-accumulates the map; serves TCP frames to Blender |
| `sim/run_demo.sh` | bash | one command: world → device → walk → render → bag → play + stack + record → dtp → eval → show_result |

## 6. Live bridge protocol (TCP, length-prefixed)
`[u32 header_len][header JSON utf-8][payload bytes]`
- Header: `{"type": "pose"|"cloud"|"gt_pose"|"path", "stamp": float, "frame": str, "dtype": "f32", "shape": [N, C]}`. The payload is a little-endian raw array.
- `pose` / `gt_pose`: shape `[7]` = x y z qx qy qz qw, in the **world frame of the run**. The bridge maps FAST-LIO `camera_init` to world using the GT pose at the first odometry message: a sim-only convenience, so the estimate and GT overlay.
- `cloud`: `[N, 4]` = x y z intensity, voxel-downsampled accumulated map, sent at ≤ 2 Hz (full snapshot, capped at ~300k points).
- Blender add-on: a non-blocking socket read inside `bpy.app.timers` at 20 Hz, with an "Connect / Disconnect" panel in the N-sidebar.

## 7. Realism knobs (dt_sim config)
- **LiDAR:**
  - range noise σ = 2 cm (Mid-360) / 1 cm (Airy)
  - dropout probability vs incidence angle and range
  - intensity = reflectivity · cos(incidence) · 255
  - blind range 0.1 m
  - per-point pose interpolation (motion distortion)
- **IMU:** noise density and bias random walk (defaults from typical MEMS), lever arm, gravity 9.80665.
- **Camera:** EEVEE 720p by default (`--hd` for 1080p), JPEG quality 90.

## 8. Evaluation
- **ATE:** estimated `T_world_lidar` vs GT, after SE(3) alignment (Umeyama) and without alignment (the sim knows the start pose). Reports RMSE and max.
- **Map accuracy:** distance from each map point to the GT mesh (Open3D). Reports mean / p50 / p90 / p95, plus the share of points within 2 cm and 5 cm.
- **Completeness:** the share of GT surface area (sampled, visible from the path) within 5 cm of the map.
- **Change:** `dtp align` + `dtp diff` between day 1 and day 2 runs. A summary of added/removed volume, checked against the known edits.

## 9. Decisions (Claude's calls, review welcome)
- Offline MCAP for sensors plus a TCP live-display bridge, instead of Zenoh. No new system dependencies; deterministic.
- Ray casting outside Blender (Open3D, Python 3.10).
- Procedural geometry over downloaded models. CC0 textures are optional.
- Default profile `mid360`. Both profiles are supported.
