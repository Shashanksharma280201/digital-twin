# Improving map quality: Airy + FAST-LIO2 handheld twin (research)

*2026-10-21. Sources are linked inline. **[U]** marks items that are unverified or inferred.*

## Airy driver, timing and IMU
- **FAST-LIO `lidar_type 2` per-point time.** Times are taken relative to the header, which is assumed to be the scan start. They are trusted only if the last point's time > 0 ([preprocess.cpp](https://github.com/hku-mars/FAST_LIO/blob/ROS2/src/preprocess.cpp)).
  - Needs `ts_first_point: true`.
  - Header must equal the first point.
  - Times must run 0–0.1 s.
  - (Verified for our bridge in the 2026-10-21 analysis.)
- **GLIM [#257](https://github.com/koide3/glim/issues/257).** Airy divergence was fixed by full frames, `ts_first_point=true`, a single clock, and no double re-timestamping.
- **IMU frame and units.** The Airy IMU is in g, mounted upside down, rotated 90° in z, and has the same frame_id as the cloud ([rslidar_sdk #203](https://github.com/RoboSense-LiDAR/rslidar_sdk/issues/203)).
  - Community configs use `R=[0,-1,0,-1,0,0,0,0,-1]`, T=0.
  - **Better: the per-unit DIFOP `imu_calib`** (quaternion + translation). For example, one unit: `T_lidar_imu=[0.00425,0.00418,-0.00446, q(-0.7043,0.7099,0.0013,0.0021)]`. This differs from the generic matrix by about 0.3–0.5° **[U: read our unit]**.
- **Near-range ghosts** (inflated objects, rings) are reported in [#181](https://github.com/RoboSense-LiDAR/rslidar_sdk/issues/181) and are not fixed by return mode or trail filter.
  - Mixed-pixel "trailing" at edges is expected.
  - Use **Strongest** return for mapping.
- **Driver settings.** `dense_points: false` keeps NaNs (our bridge drops them). Use `min_distance` 0.3–0.5 m and `max_distance` about 30 m indoors.
- **Alternative Airy LIO forks:**
  - KiiiLin/fast_lio_robosenseAiry, SaranSundar08/FASTLIO2_robosense_Airy (odometry configs: 0.5/0.5 filters, no dense-map tuning).
  - JunJie1213855/Fast-LIO2_ROS2 (`lidar_type 5`).
  - SuperLIO (blind 1.0).
  - LIO_SC_ROS2 (Point-LIO + ScanContext).

## FAST-LIO2 for a dense map
The FAST-LIVO README uses the same parameter semantics.

| param | ours | dense indoor |
|---|---|---|
| point_filter_num | 6 | 2–3 |
| filter_size_surf | 0.5 | 0.1–0.15 |
| filter_size_map | 0.5 | 0.2–0.3 |
| max_iteration | 3 | 4–5 |
| blind | 0.3 | 0.4–0.6 (operator) |
| det_range | 100 | 30–60 |

Offline pose refinement on top of FAST-LIO poses:
- **HBA / BALM2** ([HBA](https://github.com/hku-mars/HBA)): joint plane bundle adjustment. Use indoor voxel 0.5–1 m; the upstream defaults worsened MME in [aim-bagr/HBA#4](https://github.com/aim-bagr/HBA/pull/4).
- **GLIM** (indoor `voxel_resolution` 0.1–0.25; Airy recipe in #257).
- **Voxel-SLAM** global BA.

## Point-cloud post-processing, in order
1. Per-scan range gate [0.4–0.6, 15–20 m].
2. Operator self-filter: a fixed azimuth sector or box in the sensor frame.
3. Jump/mixed-pixel removal on the ring × azimuth image (drop points between neighbours that differ by more than max(5 cm, 3%·r)), plus an incidence angle limit of 75–80°.
4. DUFOMap ray-casting dynamic removal ([repo](https://github.com/KTH-RPL/dufomap)).
5. Glass and reflections: crop window planes, or dual-return reflection detection ([STAR-Center](https://github.com/STAR-Center/Reflection_detection)).
6. SOR (k 20–30, ratio 1.5–2) followed by a radius filter.
7. Display voxel 3–5 cm, or no voxelising at all: use TSDF meshing.

## A clear visual twin
- **Geometry:** TSDF / [VDBFusion](https://github.com/PRBonn/vdbfusion) (voxel 2–3 cm, truncation 3–4×) averages the 1–1.5 cm sensor noise into one surface. Poisson at depth 10–11 (trim low density). NKSR **[U Orin]**.
- **Colour:**
  - Calibrate the extrinsic (direct_visual_lidar_calibration).
  - Estimate the time offset (photometric sweep).
  - Lock exposure and white balance, with a short exposure (the ZED 2i has a rolling shutter).
  - Pick the best view (cos angle × 1/distance × sharpness) instead of averaging.
  - Drop frames with high angular rate.
  - Use a mesh z-buffer for occlusion.
  - Colour mesh vertices or textures.
- **Splats:** the [GS-SDF](https://github.com/hku-mars/GS-SDF), [Gaussian-LIC2](https://github.com/APRIL-ZJU/Gaussian-LIC) and [GS-LIVM](https://github.com/xieyuser/GS-LIVM) family; our `splat/` path (refined poses + LiDAR init + depth loss).
- **Commercial reference:** XGRIDS Lixel uses 2× 48 MP panoramic cameras plus a separate splat pipeline. Visual quality comes from imagery and splatting, not from point colours.
- **FAST-LIVO2** needs a triggered, global-shutter camera; the ZED 2i is neither.

## Capture technique
- **Tilt the hemispherical head 45–90°** for floor, ceiling and wall coverage, with the operator in the back half ([corridor study PMC9865594](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9865594/); ZEB-style rigs) **[U: angle for Airy]**.
- Start still for 3–5 s.
- Walk at 0.3–0.5 m/s, with rotations under 30–45°/s.
- Cover every room and close loops.
- Hold the rig at a constant offset from your body.
- Pass glass at an angle.
- For the camera, pause 1–2 s per direction.

## Metrics without ground truth
- **MME** (mean map entropy, r ≈ 0.3 m indoors), **MPV** (mean plane variance), and plane-thickness RMS on fitted walls. The target is about 1.5–2 cm; worse than 3 cm means a pose, time or extrinsic error.
- Loop-closure gap.
- A/B test each change on the same bag.
