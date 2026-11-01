# Blender simulation: the DT-1 handheld scanner on a virtual construction site

The design is in [`docs/design/2026-10-18-blender-sim-design.md`](../docs/design/2026-10-18-blender-sim-design.md). The supporting research is in `research/08` and `research/09`.

```
sim/blender/            Blender 5.2 scripts: world (day1/day2), DT-1 device, walk, camera renders, result scene
sim/blender/addon/      DT Live add-on: shows the twin being built, live, from ROS 2
sim/dt_sim/             Python 3.10: LiDAR/IMU/camera simulation -> driver-native MCAP bags; evaluation vs ground truth
ros2_ws/src/dt_blender_bridge   ROS 2 node streaming pose + growing map + ground truth to Blender (TCP)
sim/run_demo.sh         the whole loop in one command
```

## How the pieces talk
Blender uses Python 3.13 and can't import ROS 2 Humble's rclpy (Python 3.10). There is no maintained ROS 2↔Blender bridge (see `research/09`), so we built our own:

- **Sensor data, Blender to ROS:** files plus deterministic MCAP bags.
  1. Blender exports the world mesh, the operator trajectory, the device extrinsics (`rig_sim.yaml`) and rendered ZED frames.
  2. `dt_sim` ray-casts the LiDAR with Open3D/Embree at about 3.5M rays/s.
  3. `dt_sim` writes the **same topics the real drivers publish**: `/livox/lidar` CustomMsg or `/rslidar_points` XYZIRT, raw IMU in g, and ZED images.
  4. `ros2 bag play --clock` feeds them into the **unchanged** stack.
- **Live view, ROS to Blender:** `dt_blender_bridge` sends the estimated pose, the voxel map and the ground truth over a small TCP protocol. The `DT Live` add-on draws them in the viewport.

## Run it
```bash
# prerequisites: ROS 2 workspace built (dt_* + fast_lio), and a venv with the offline tools
pip install -e pipeline -e sim/dt_sim
DT_WS=$PWD/ros2_ws DT_VENV=$VIRTUAL_ENV ./sim/run_demo.sh --profile mid360          # or airy
DT_WS=$PWD/ros2_ws DT_VENV=$VIRTUAL_ENV ./sim/run_demo.sh --skip-blender --skip-bags --live   # watch it live in Blender
```

**Outputs** (in `~/dt_data/sim/demo_<profile>/`):

| File | What it shows |
|---|---|
| `device/hero_<profile>*.png` | The DT-1 product renders |
| `camera/left/*.png` | What the ZED 2i sees |
| `result_day1.png` | The reconstructed twin on the ground-truth site, coloured by error, with estimated (orange) and true (green) paths |
| `result_day2.png` | The change map from day 1 to day 2: green = added, red = removed |
| `eval_day1.json`, `eval_day2.json` | ATE, map error and completeness |
| `session_day2/products/diff/summary.json` | Change-detection clusters |

`eval_*.json` holds two map scores: `map` (aligned by the path alone) and `map_surface` (fitted to the true surfaces: shape error, completeness, scale, virtual tape measure). Use `map_surface` to judge a map.

## Watch it live in Blender
```bash
ros2 run dt_blender_bridge dt_blender_bridge --ros-args -p rig_path:=<run>/rig_sim.yaml -p use_sim_time:=true
blender <run>/world/walk_preview_day1.blend -P sim/blender/addon/run_live.py -- --connect 127.0.0.1:5577
```
The **DT Live** sidebar tab (N panel) has Connect/Disconnect, Clear and a point-size setting.

On the real rig, run the bridge without `rig_path`. The map then shows in the SLAM frame.

## Known limits
- **Airy at 1× playback** starves FAST-LIO over default FastDDS on a laptop, because each canonical cloud is about 2 MB. Use `--rate 0.25`, or CycloneDDS with large buffers (this is what the Orin setup configures).
- **The Airy model** assumes 96 rings spaced evenly over 0–90°, and the IMU at the LiDAR origin.
- **Camera rendering** takes about 0.9 s per 720p frame on an RTX 4060. The demo renders at 5 Hz by default; use `--cam-rate`.
- **Sensor dimensions:** the LiDAR origin heights on the device and the ZED focal length (527 px at HD720) are nominal values.
