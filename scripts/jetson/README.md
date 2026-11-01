# Jetson AGX Orin setup (JetPack 6, ROS 2 Humble)

These scripts take a fresh JetPack 6.x Orin to a built digital-twin workspace. They are safe to re-run: each step skips work that is already done.

## One command

```bash
./setup_everything.sh --iface eth0        # 00-09 + 11, about 3-4 h, asks for sudo once, resumable with --from NN
```

Beyond steps 00–07 below, it also runs:

| Step | Command | Expected result |
|---|---|---|
| 8 | `./08_install_python_tools.sh` | `~/dt_data/venvs/dt` with `dtp` and `dt_sim`. Checks Open3D ray casting on aarch64 and runs 25 tests. |
| 9 | `./09_install_blender.sh` | Blender 5.2 **built from source**. No ARM64 Linux Blender is published anywhere; Blender's ARM64 dependency libraries are. About 1–2 h, about 15 GB. Installs to `/opt/blender-5.2`, with `/usr/local/bin/blender`. |
| 10 | `./10_install_splat.sh` (opt-in: `--with-splat`) | JetPack PyTorch plus gsplat compiled for sm_87. Experimental; training on an x86 GPU is recommended. |
| 11 | `./11_verify.sh [--profile airy] [--camera]` | ROS unit tests plus the full Blender sim on the Orin. Prints `RESULT: PASS/FAIL` against ground truth. |

The test plan that follows the install is in `docs/test-plan/2026-10-19-agx-real-hardware-test-plan.md`.

## Order of operations

| Step | Command | Expected result |
|---|---|---|
| 0 | `./00_check_system.sh` | Prints L4T (36.3/36.4/36.5), CUDA, NVMe, network interface, PTP capability, ZED on USB. Read-only. |
| 1 | `./01_install_ros_humble.sh` | ROS 2 Humble ros-base, colcon, vcstool, rosdep, CycloneDDS, MCAP/zstd, PCL, linuxptp |
| 2 | `./02_install_livox_sdk2.sh` | `/usr/local/lib/liblivox_lidar_sdk_shared.so` (source in `~/dt_deps/Livox-SDK2`) |
| 3 | `./03_install_zed_sdk.sh` | ZED SDK matching L4T (5.2 for 36.3, 5.5 for 36.4/36.5). Prints the wrapper tag. **Log out and back in** if it added you to `video`. |
| 4 | `./04_install_glim.sh` (or `--cpu`) | `ros-humble-glim-ros-cuda12.x` from the koide3 PPA |
| 5 | `./05_setup_network.sh eth0` | Connection `dt-lidar`: 192.168.1.5/24 (Mid-360 host) and 192.168.1.102/24 (Airy host); big UDP buffers; `~/.config/dt/cyclonedds.xml` |
| 6 | `./06_setup_ptp.sh eth0` | `dt-ptp4l` + `dt-phc2sys` systemd services; the Orin is PTP grandmaster |
| 7 | `./07_build_workspace.sh` | Upstream repos in `ros2_ws/src/external`, patched, built. Writes `ros2_ws/dt.lock.repos`. |

To run everything: `./setup_all.sh --iface eth0` (`--skip 03,04` skips steps, `--glim-cpu` installs the CPU GLIM build).

Then, in every shell: `source scripts/jetson/env.sh`

Before capturing: `sudo nvpmodel -m 0 && sudo jetson_clocks`

### Network note
The AGX Orin devkit has one RJ45 port. Step 5 makes that port **static, with no default route**, for the LiDAR. Use Wi-Fi (or a USB-Ethernet adapter) for internet. If you need internet over the RJ45 during setup, run step 5 last, or switch with `nmcli connection up <your-dhcp-connection>`.

### One-time manual step (Airy)
1. Open `http://192.168.1.200`, go to Setting → Time Synchronization Source, and set **PTP-E2E (L4)**, domain 0. Save, then power-cycle the LiDAR.
2. The status page should show **Lock**.

The Mid-360 follows the PTP master on its own.

### Mid-360 IP
The Mid-360's IP is `192.168.1.1XX`, where XX is the last two digits of its serial number. Pass it at launch (`mid360_ip:=192.168.1.1XX`), or `export DT_MID360_IP=192.168.1.1XX` in your shell. The launch file fills it into a copy of the profile's `MID360_config.json`.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Airy publishes 5–8 Hz instead of 10 Hz | DDS throughput on Humble (rslidar_sdk #258) | Check `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (env.sh) and that step 5's sysctl is loaded (`sysctl net.core.rmem_max`). `ros2 topic hz` itself under-reads on Jetson, so check with `dt_monitor`. |
| LiDAR `header.stamp` is a few hundred/thousand seconds (time since power-on) | PTP not locked (the Mid-360 stamps device time) | `journalctl -u dt-ptp4l`, `sudo pmc -u -b 0 'GET PORT_DATA_SET'`. Check the cable goes direct or through a PTP-aware switch. Airy: check the web UI time source. |
| Constant ~37 s offset between LiDAR and system time | UTC vs TAI (PTP carries TAI) | Edit `/etc/systemd/system/dt-phc2sys.service` and change `-O 0` to `-O -37` (or `-O 37` depending on the sign you observe), then `sudo systemctl daemon-reload && sudo systemctl restart dt-phc2sys`. Airy: try its "No Leap Second" option. |
| ptp4l flaps MASTER↔FAULTY, "timed out while polling for tx timestamp" | Orin MGBE tx timestamp latency | `tx_timestamp_timeout` is already 20; raise it to 50 in `/etc/linuxptp/dt-gm.conf` |
| `/rslidar_imu_data` missing, or points have no `ring`/`timestamp` | rslidar_sdk built before the CMake patch (options are cached) | `./07_build_workspace.sh --no-import -- --packages-select rslidar_sdk` after `rm -rf ros2_ws/build/rslidar_sdk` (07 does this automatically when it changes the file). Also set `imu_port: 6688` in the config. |
| Workspace `build/` and `install/` vanished | Someone ran `livox_ros_driver2/build.sh` | **Never run it.** 07 does the ROS 2 file swap itself. Re-run `./07_build_workspace.sh --no-import`. |
| `liblivox_lidar_sdk_shared.so: cannot open shared object` | ldconfig not run | `sudo ldconfig` (02 does it) |
| ZED wrapper fails to build with CUDA_TOOLKIT_ROOT_DIR | Missing JetPack dev package | `sudo apt install nvidia-jetpack-dev` (03 does it) |
| ZED NEURAL depth unavailable | SDK installed with `skip_od_module` | Re-run `./03_install_zed_sdk.sh --force` |
| GLIM: undefined symbols or `kdtreex.hpp` missing | Mixed old/new gtsam_points | `sudo rm -rf /usr/local/lib/cmake/gtsam_points /usr/local/include/gtsam_points` (stale source installs), then re-run 04 |
