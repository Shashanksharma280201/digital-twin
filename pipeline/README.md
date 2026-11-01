# dt_pipeline: offline digital-twin pipeline

`dt_pipeline` processes capture sessions without ROS. It reads the MCAP bags with `rosbags` and turns them into:

- a trajectory
- a deskewed map
- a static/dynamic split
- session-to-session alignment
- a change map
- a COLMAP export for Gaussian splatting

The steps are specified in §10 of `docs/design/2026-10-18-dt-capture-stack-design.md`.

## Install

```bash
pip install -e pipeline            # core: numpy scipy rosbags laspy[lazrs] small_gicp open3d opencv
pip install -e "pipeline[full]"    # + py4dgeo (M3C2), dufomap (x86 wheels only; build from source on Jetson)
conda install -c conda-forge python-pdal   # optional, enables --copc
```

On a Jetson (JetPack 6, Python 3.10), the core dependencies have aarch64 wheels. Open3D there is CPU-only.

## Session layout

```
<session>/bag/            rosbag2 MCAP (from dt_tools session recorder)
<session>/rig.yaml        extrinsics (schema in dt_pipeline/rig.py)
<session>/glim_dump/      optional, GLIM output (traj_lidar.txt)
<session>/products/       written by dtp
```

## Usage

```bash
scripts/process_session.sh S           # the usual chain: ingest, refine, fuse, colorize, texture, report
dtp info    S                         # topics, rates, duration
dtp ingest  S                         # bag -> scan cache + traj_lidar.tum (applies S/ring_calib.json)
dtp ringcal S                         # learn per-beam LiDAR offsets against mesh.ply -> S/ring_calib.json
dtp fuse    S [--min-hits 3]          # GPU map.laz + mesh.ply; weak voxels -> map_loose.laz
dtp texture S                         # image-textured mesh -> products/textured/mesh.obj
dtp traj    S --source fastlio        # /Odometry -> products/traj_lidar.tum (T_world_lidar)
dtp traj    S --source glim           # or GLIM dump traj_lidar.txt
dtp map     S --voxel 0.05 [--copc]   # per-point deskewed map -> products/map.laz
dtp clean   S                         # -> map_static.laz / map_dynamic.laz (DUFOMap or visibility fallback)
dtp align   REF S [--no-coarse]       # -> S/products/T_ref_session.txt (FPFH+RANSAC, then GICP)
dtp diff    REF S [--m3c2]            # -> S/products/diff/{added,removed}.laz, summary.json
dtp colmap  S --min-translation 0.2   # -> products/colmap/{images,sparse/0} for gsplat
```

LiDAR input can be any of these:

- the canonical `/dt/lidar/points` topic (the default)
- raw driver topics, passed with `--topic`:
  - `/livox/lidar`: Livox CustomMsg or PointCloud2
  - `/rslidar_points`: RoboSense XYZIRT

## Tests

```bash
pip install -e "pipeline[test]" && pytest pipeline/tests
```

The tests build synthetic MCAP sessions: a ray-cast room scanned by a moving spinning LiDAR, plus odometry and images. They then check:

- trajectory conversion
- that the map lies on the room's surfaces, and that deskewing helps
- alignment recovering a known offset
- detection of an added box
- removal of a moving person
- the COLMAP pose convention
