# dt_sim: DT-1 handheld sensor simulator and ground-truth evaluation

`dt_sim` simulates the handheld device in a Blender-generated world, using the run-directory contract in `docs/design/2026-10-18-blender-sim-design.md` §3. It writes a **driver-native** ROS 2 MCAP bag, so the real stack (bridge, FAST-LIO, `dt_session`, `dtp`) runs unchanged. Afterwards it scores the result against ground truth.

```bash
pip install -e pipeline -e sim/dt_sim       # Python 3.10 (same env as dtp); no ROS needed
RUN=~/dt_data/sim/demo                      # world/mesh_day1.npz, trajectory.csv, rig_sim.yaml, [camera/]
dt_sim info RUN
dt_sim bag  RUN --profile mid360            # -> RUN/bag (+ RUN/bag_info.json)
dt_sim bag  RUN --profile airy --variant day2 --out RUN/bag_day2_airy
dt_sim eval RUN --session SESSION           # ATE from SESSION/bag:/Odometry (or products/traj_lidar.tum),
                                            # map accuracy from SESSION/products/map.laz -> RUN/eval.json
dt_sim gtmap RUN --profile mid360           # noise-free map the device could see -> RUN/products/gt_map_day1.laz
```

## Running the stack on a simulated bag
```bash
ros2 launch dt_bringup capture.launch.py profile:=mid360 driver:=false camera:=false use_sim_time:=true
ros2 bag record -s mcap --use-sim-time -o SESSION/bag /Odometry /dt/lidar/points /dt/lidar/imu /sim/ground_truth/odometry
ros2 bag play RUN/bag --clock
cp RUN/rig_sim.yaml SESSION/rig.yaml && dtp traj SESSION --source fastlio && dtp map SESSION
dt_sim eval RUN --session SESSION
```

## What is simulated
| Sensor | Topic | Model |
|---|---|---|
| Mid-360 | `/livox/lidar` (CustomMsg) | Official non-repetitive pattern (`mid360.csv` from Livox-SDK/livox_laser_simulation, MIT, cached in `~/.cache/dt_sim`) at 200 kHz, 20k points per 10 Hz frame. Misses are reported as `0,0,0`. |
| Airy | `/rslidar_points` (XYZIRT) | 96 rings, uniform 0–90° elevation (**assumed**; real units are calibrated per device), 0.4° × 900 columns, 111 µs per column, NaN for misses |
| IMU | `/livox/imu`, `/rslidar_imu_data` | Specific force and body rate from a quintic / RotationSpline fit of the IMU-frame trajectory (lever arm exact), white noise + bias random walk, **in g**. Airy uses raw axes (`a_raw = Rᵀ a_lidar`), so the bridge rotation is exercised. |
| ZED left | `.../image/compressed` + `camera_info` | Rendered PNGs (`camera/frames.csv`) → JPEG q90, pinhole intrinsics from `rig_sim.yaml` |
| Ground truth | `/sim/ground_truth/odometry` | `world → base_link` at 100 Hz |

Every LiDAR ray is cast from the sensor pose at its own firing time (Open3D Embree, about 3.5M rays/s). The point is expressed in the sensor frame at that same time. This reproduces real motion distortion.

The noise model (`LidarNoise`, `ImuNoise`) covers range σ (2 cm Mid-360, 1 cm Airy), dropout increasing with incidence angle and range, reflectivity-dependent maximum range, and intensity = reflectivity · cos(incidence).

## Evaluation
- **ATE** is reported two ways:
  - *anchored*: the estimate is placed using the GT pose at its first sample. FAST-LIO's `camera_init` is the initial body pose.
  - *aligned*: SE(3) Umeyama alignment.
- **Map accuracy** is the point-to-mesh distance (mean, p50/p90/p95, % within 2 and 5 cm). The map is put in the world frame with the aligned transform.
- **Completeness** is the share of GT surface visible from the path that has a map point within 5 cm.

## Tests
```bash
pytest sim/dt_sim/tests      # synthetic box room: CDR round trip, points on surfaces, motion distortion,
                             # Airy fields, IMU at rest and in turns, bag contents, eval with perfect estimates
```

## Results on this machine (x86 laptop, Humble, default FastDDS, 2026-10-18)
| Run | Result |
|---|---|
| Mid-360, 86 s / 88 m walk through the demo site (outside → through the building → back), real time | ATE aligned **3.0 cm**, anchored 5.2 cm (max 30 cm), final drift 0.08 % |
| Map from that run (`dtp map`, FAST-LIO poses) | mean 4.5 cm, p50 3.1 cm, p95 13.4 cm, 69 % < 5 cm, completeness 78 % |
| Same scans with GT poses (sensor-noise floor) | mean 1.0 cm, 100 % < 5 cm, completeness 89 % |
| Airy, 40 s, playback at 0.25× | ATE aligned **4.2 cm**, drift 0.10 % |
| Airy, 40 s, playback at 1× | **Diverges.** FAST-LIO is starved of 2 MB canonical clouds over best-effort FastDDS: output gaps of 7–9 s. |

The Airy real-time result is a transport and throughput limit. The simulated data is fine: it tracks at reduced rate, and the bridge's IMU rotation was checked against gravity. Expect the same on the Orin unless:
- CycloneDDS is used with large socket buffers (`scripts/jetson/05`), and/or
- the driver, bridge and LIO run composed in one process, and/or
- the canonical cloud is decimated.

`dt_sim bag` writes Airy bags at about 0.4× real time (2.2 MB clouds serialised through rosbags), and Mid-360 bags at about 1.5× real time.
