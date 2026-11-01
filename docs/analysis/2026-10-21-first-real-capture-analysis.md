# First real capture: why the twin looks distorted, and how to make it 1:1

| | |
|---|---|
| Date | 2026-10-21 |
| Session | `~/dt_data/sessions/office/20261021_142710_airy` (AGX), 74 s |
| Rig | RoboSense Airy (upright) resting on top of a ZED 2i, handheld, Jetson AGX Orin |
| Status | **Analysis and plan only. Nothing in the stack was changed.** |
| Research | `research/10-map-quality-improvement.md` |
| Scripts (read-only) | `analyze_session.py`, `analyze_blur.py`, `analyze_refine.py` (run from /tmp on the AGX) |

## 1. Measurements

### 1.1 The sensor data is healthy
| Check | Result | Verdict |
|---|---|---|
| Airy per-point timestamps | span 0.100 s per scan, 99.99999% sorted, ~10k distinct times per scan | ✅ deskew input is correct |
| Points per scan | 78.7k valid, 9.4% no-return (NaN) | ✅ |
| Mixed-pixel ("edge trailing") points | 0.18% | ✅ minor |
| IMU at rest | (−0.21, −0.04, 9.63) m/s², σ 7 mm/s², gyro σ < 0.08 °/s | ✅ (scale reads 1.8% low; minor) |
| IMU rotation vs FAST-LIO rotation | correlation 0.95 / 0.95 / 0.99 (x/y/z), best time lag −5 ms | ✅ IMU orientation and timing agree with the LiDAR |
| LiDAR / IMU / camera clock sync (PTP) | IMU 3 ms, points within one scan period, camera within ~80 ms | ✅ |

### 1.2 How the capture was done
| | |
|---|---|
| Path | 4.1 m in total, but **confined to a 25 × 12 cm box**: the rig was turned in place |
| Rotation | **634° in total**, up to 60°/s (10% of the time above 30°/s) |
| Return-to-start error | 5 mm |
| Range of returns | 54% at 1.5–3 m (mostly the ceiling at ~2.2 m); **16% within 1 m (operator, hand, rig, desk)**; only 5.7% beyond 5 m |

### 1.3 Where the blur comes from (median wall/ceiling thickness, planar 15 cm patches)
| Map built from | Thickness |
|---|---|
| One scan (sensor noise) | 8.8 mm |
| 31 scans while still | 10.3 mm |
| All scans, ignoring < 1.2 m | **22.9 mm** |
| Only slow-rotation scans (< 10°/s) | 22.2 mm |
| Only fast-rotation scans (> 30°/s) | 20.3 mm |
| All scans, **without** per-point motion correction | 32.0 mm |
| All scans after 2 rounds of GICP pose refinement | **18.9 mm** |

GICP had to move FAST-LIO's poses by **a median of 11 mm and 0.15°** (90th percentile: 21 mm and 0.26°).

**Conclusions**
1. **Fast motion is not the main cause.** Slow segments are just as blurred.
2. **About 40% of the extra blur is pose error** from FAST-LIO, which is tuned for speed (0.5 m voxels, every 6th point). Refinement removes it.
3. **About 60% is systematic, sensor-level error.** It survives refinement, and its cause is not identified yet. The most likely candidates are listed below in order, and §4 has a test for each:
   - the same surface measured by different Airy beams or angles disagrees (intrinsic calibration or incidence-angle range bias);
   - residual intra-scan motion from the 1–2 cm pose noise;
   - the IMU↔LiDAR extrinsic being the generic matrix rather than this unit's DIFOP calibration (~0.3–0.5° possible).
4. **The 2 cm voxel map is finer than the sensor's 1–1.5 cm noise.** Raw points can never look "smooth" without surface fusion.

### 1.4 Why it looks unclear, beyond the blur
- **Coverage:** the upright Airy sees only above its horizon. So: ceiling and upper walls, **no floor**, and nothing below about 1.3 m. Standing in one spot gives a single viewpoint.
- **Operator in the data:** 16% of returns are within 1 m. They form a smeared blob around the path as the rig turns.
- **Colour:** only 42% of points coloured (the camera looks forward only). The extrinsic is estimated, not calibrated. Colours are averaged over many views (blurry). The ZED 2i has a rolling shutter.
- **Display:** loose points without surfaces or shading are hard to read even when accurate.

## 2. Do the LiDAR and camera have to be fixed together?
**Yes. Rigidly, in a fixed relative position, for the whole capture.**

- Every LiDAR↔camera fusion step relies on one fixed transform: colouring points, the splat, and calibration. If the camera shifts even 1 mm or 0.1° relative to the LiDAR, colours land on the wrong surfaces. A 0.5° shift is 2.6 cm at 3 m.
- The **Airy's IMU is inside the Airy**, so LiDAR↔IMU is already rigid. Geometry (tracking and map) does not depend on the camera. But the operator should never be able to bump the camera relative to the LiDAR.
- **Resting the Airy on top of the ZED is not enough.** Jerks will shift it. Use a bracket that bolts both to one plate: the Airy has M3 holes, the ZED a ¼"-20 thread. Measure the offsets once, then calibrate.
- Fix the **handle** to the same plate. The rig can then be moved however you like; only the *sensors relative to each other* must never move.
- After any re-mount, re-run the LiDAR↔camera calibration (design §11).

## 3. Target: smooth and 1:1, even with jerks and repeated passes
The aim is a pipeline where **extra passes make the twin better** (more observations averaged) instead of worse (ghosts), and where jerks are absorbed.

| Stage | Today | Proposed | Why it gives robustness |
|---|---|---|---|
| Rig | Airy resting on ZED, upright | **Rigid bracket**; Airy **tilted 45–90°** (or a second, upside-down unit) | Removes relative motion; sees floor, walls and ceiling; operator out of view |
| Calibration | generic IMU matrix; LiDAR↔camera estimated | **Per-unit DIFOP IMU extrinsic**, targetless LiDAR↔camera calibration, measured camera time offset | Removes systematic misregistration |
| Live odometry | FAST-LIO, speed-tuned | FAST-LIO dense-tuned (surf 0.1–0.15, map 0.2–0.3, filter 2–3), or **Point-LIO** for jerky handheld motion (handles IMU saturation; Airy IMU is ±4 g) | Fewer pose errors during jerks |
| **Global back-end (new)** | none | **GLIM offline** (factor graph, loop closure, multi-session merge) → **plane bundle adjustment (HBA/BALM)** on all scans | Revisits are recognised and **fused into one consistent surface**; drift and jerk errors are spread out and corrected |
| Cleaning | none | Self/operator crop → mixed-pixel/jump removal → **DUFOMap** (people, moving objects, operator ghosts) → glass handling → outlier filters | Removes the blobs and ghosts that repeated passes would otherwise multiply |
| Surface | raw points, 2 cm voxels | **TSDF fusion (VDBFusion, 2–3 cm) → mesh**; points kept for measurement | Averages sensor noise across all passes, so **more passes = smoother, more accurate surfaces** |
| Colour | per-point average | Best-view texture on the mesh (sharpness, angle and distance weighting; drop blurred, high-rate frames), then the **LiDAR-supervised Gaussian splat** for the photoreal view | Sharp colours that line up with the edges |
| Quality gate (new) | none | Automatic report per capture: plane thickness, MME, loop gap, coverage, % coloured | Shows whether a capture is good enough, or which area to re-scan |

**Why repeated scanning then helps:**
- With globally consistent poses from the back-end, two passes over a wall are registered to each other within millimetres.
- The TSDF averages them, so noise drops roughly with √(number of passes).
- Dynamic removal deletes anything that was there in one pass but not another.

**Jerks:**
- Short jerks are absorbed by the IMU-driven odometry and corrected by the back-end.
- What a jerk can still break is a blurred camera frame. The colour stage simply skips those frames.

## 4. Experiments to run next (each A/B on this same recording; no new capture needed)
1. **Read the Airy's DIFOP IMU calibration** and compare it with our matrix. Re-run FAST-LIO with it. Metric: thickness and GICP correction size.
2. **Dense FAST-LIO parameters** (filters and point_filter_num) on bag replay. Metric: thickness and GICP correction.
3. **GLIM offline** on the bag. Metric: thickness and MME.
4. **Per-beam consistency test:** thickness of maps built from single ring groups, and from yaw-only versus tilted segments. This isolates an intrinsic/incidence bias (the unexplained ~60%).
5. **Self-filter + DUFOMap + TSDF mesh** on the best poses. Visual check plus thickness on the mesh.
6. Then a **new capture**: rigid bracket, tilted Airy, 2–3 min walk with loops. Same metrics. This is the real 1:1 test.

**Targets:** plane thickness ≤ 12 mm (close to the still-scan 10.3 mm), loop gap < 2 cm, floor + walls + ceiling all present, ≥ 85% of surfaces coloured.
