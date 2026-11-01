# 9. Issues and roadmap

Status words: **Fixed** = solved for good. **Mitigated** = much better, a further step is planned. **Open** = still a problem.

## Issues ledger

### Map quality

| Status | Issue | Cause | What we did / plan |
|---|---|---|---|
| Mitigated | Point map walls thicker than one scan | Weakly supported voxels count as much as solid ones; each LiDAR beam has its own error up to 22 mm. Paths are not the main cause: scans already sit about 3.5 mm from the surface | Beam calibration + loose-point split: 27.4 → 20.6 mm, target 12 mm. `polish` gave no gain, so plane bundle adjustment was not built |
| Mitigated | Doubled walls from drift | FAST-LIO never corrects old heading errors; spinning makes it worse | `dtp refine` with loop closures; capture 3 closes within 4.4 cm. Loop checks need to be stricter (723 of 726 accepted) |
| Mitigated | Loose points hard to read | Points have no surface or shading | `dtp mesh` (TSDF): smooth 2.5 mm surfaces |
| Open | Lumpy furniture, holes | Operator turned on the spot; backs of objects never seen | Walk loops near the walls, twice, sweeping past objects |
| Mitigated | Speckle along edges | LiDAR mixed pixels at depth jumps | `dtp ingest` drops them (about 580,000 on capture 3); check on a walking capture |
| Mitigated | Little floor | Airy only sees from its horizon up | Tilted 21° on the stand; 30–45° would see more |
| Mitigated | Operator in the map | Person holding the rig is seen from every angle | Tilt cut it from 16% to ~5%; points within 0.8 m removed. Next: space carving / DUFOMap |

### Colour

| Status | Issue | Cause | What we did / plan |
|---|---|---|---|
| Fixed | Camera stored at 7.7 Hz | CPU JPEG of full-size images | Colour from the full-rate SVO2; ROS preview at half size (15.2 Hz) |
| Fixed | Colour shifted 6 cm sideways | Pipeline used the ZED mount point instead of its left lens | Lens offset added in `rig.py` |
| Fixed | Camera calibration was a photo estimate | No calibration target | `dtp extrinsic` (target-free): pitch corrected by 3.5° |
| Mitigated | Partial colour (42% → 56%) | Narrow camera view, spinning capture | 97.4% in capture 3 |
| Mitigated | Pale, washed-out colour | Auto-exposure changes; thousands of frames averaged | Per-frame exposure correction added; locking exposure is still the proper fix |
| Mitigated | Blurry colour detail | One colour per mesh corner (~2 cm) | `dtp texture`: fine detail doubled, on real and sim held-out frames |
| Open | Speckle on textures near close objects | Objects inside the 0.8 m self-zone are not in the mesh, so their colour lands on the surface behind | Keep close objects out of the first seconds; use near returns as occluders |
| Open | Operator or close clutter left in the mesh | Returns just beyond 0.8 m survive | Space carving / DUFOMap |

### Hardware and setup

| Status | Issue | Cause | What we did / plan |
|---|---|---|---|
| Mitigated | LiDAR resting loose on the camera | Jerks move one sensor against the other | Metal stand. Next: bolts (M3 + ¼″-20) |
| Mitigated | AGX reboots without warning | Probably power loss; logs lost | Saved journal, 2 s black box (`dt-blackbox`), reboot after panic. Next: check the power supply |
| Mitigated | PTP faults on Orin eth0 | Orin Ethernet driver misses send time stamps | Timeout 100 ms. Moving the LiDAR to an Intel port would remove them |
| Mitigated | Laptop Ethernet has no PTP hardware | Network chip limitation | Software time stamps; still locks (2–3 ms) |
| Fixed | LiDAR stamps 37 s behind | PTP uses TAI, the computer used UTC | `phc2sys -O 37 -S 1`, power-cycle the Airy |
| Fixed | ROS transport would not start | ZED installer's buffer setting loaded after ours | Ours renamed `99-dt-lidar.conf` |
| Fixed | No camera images | ZED wrapper 5.2 renamed topics to `rgb/...` | Stack uses `rgb/...`, old recordings read through an alias |
| Fixed | Bridge silently not loaded | Old nodes survived when only the launch was killed | `stop_stack.sh` stops everything |
| Fixed | Laptop: MRPT and Livox SDK build errors | Broken package update; Livox driver needed newer SDK | Removed old MRPT packages; message-only Livox package on the laptop |
| Fixed | RViz view not movable | RViz configs had no camera tools | Tools added to `calib.rviz` and `dt.rviz` |
| Fixed | SVO kept recording after Ctrl-C | ROS's own Ctrl-C handler shut down before the stop call | Recorder keeps Ctrl-C for itself (installed; verify on next capture) |
| Fixed | 56 min of processing | CPU-only, single-threaded steps | GPU pipeline: 7.9 min first run, 4.9 min repeat run with texture. Full AGX timing still unmeasured |
| Open (AGX offline) | Temporary sudo rule, Blender build (09) and verify (11), artifact sync | AGX not reachable from the laptop | Do when the AGX is back |

### Tested and rejected

| Idea | Result |
|---|---|
| Airy factory IMU calibration (0.69° from nominal) | Walls 25.1 mm vs 22.1 mm without it: worse |
| Finer FAST-LIO grid (0.15 m) | 24.5 mm vs 22.1 mm at 0.2 m: worse |
| `dtp polish` after beam calibration | 21.0 mm vs 20.8 mm: no gain |
| Distance-to-mesh map filter | Lost 7% of true surface in sim; "seen 3+ times" lost 1% and scored better |
| Skipping SVO frames in colour | 60–76 s vs 74 s: no real gain |
| Predicted frames for splats | Lowered PSNR by 2.1–3.1 dB ([research 11](../research/11-pixel-prediction-vs-coverage.md)) |

## Roadmap (options, to be agreed with the team)

Done in software: beam calibration, map cleanup, textured mesh, scale proof in simulation.

Needs the rig:
1. **Walking capture with locked exposure.** Walk loops near the walls, twice, sweep past objects, turn < 30 °/s. Biggest remaining win.
2. **Prove 1:1 on a real room.** Tape-measure 3–4 distances and compare. The sim gives 53 ppm scale error and 6 mm median tape error, which does not prove the real rig.
3. **Bolt the sensors** and check the AGX power supply.
4. **Full walking capture on the AGX**, processed and timed there.
5. **Re-run `--calibrate-rings` on that walking capture.** Capture 3 moved little, so the stored calibration may improve.

Software, after that:
- Close-object handling in textures; space carving for people.
- Live coverage map in the motion coach ([research 11](../research/11-pixel-prediction-vs-coverage.md)).
