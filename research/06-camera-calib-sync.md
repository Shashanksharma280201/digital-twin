# ZED 2i, Calibration, PTP, nvblox, Recording: Verified Facts

*Verified 2026-10-18.*

## ZED SDK / wrapper
| JetPack | L4T | ZED SDK | Installer |
|---|---|---|---|
| 6.0 | 36.3 | 5.2.3 | `ZED_SDK_Tegra_L4T36.3_v5.2.3.zstd.run` |
| 6.1 / 6.2 | 36.4 | 5.5.0 | `https://download.stereolabs.com/zedsdk/5.5/l4t36.4/jetsons` |
| 6.2.2 | 36.5 | 5.5.0 | `.../l4t36.5/jetsons` |

- **Install:**
  - Check `cat /etc/nv_tegra_release` first; the L4T version must match exactly.
  - `sudo apt install zstd nvidia-jetpack nvidia-jetpack-dev`
  - `./zed.run -- silent skip_tools`. **Do not** pass `skip_od_module`, because NEURAL depth needs it.
- **Wrapper:** `stereolabs/zed-ros2-wrapper`. `master` supports Humble; tag `v5.5.0` (pairs with SDK 5.2 → tag v5.2.2). Apache-2.0. `ros-humble-zed-msgs` is available as an apt binary.
  - Build: `rosdep install --from-paths src --ignore-src -r -y`, then `colcon build --packages-skip zed_debug`.
  - Launch: `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i`.
- **Topics (v5, renamed from v4):**
  - `/zed/zed_node/left/color/rect/image` (+ `.../camera_info`), `rgb/color/rect/image`
  - `depth/depth_registered` (32FC1, metres), `point_cloud/cloud_registered`
  - `imu/data`, `imu/data_raw`, `odom`, `pose`
- **Parameters:**
  - `general.grab_resolution` HD2K/HD1080/HD720 (zed2i default HD720); `grab_frame_rate` (default 60); `pub_frame_rate`.
  - `depth.depth_mode` NONE/NEURAL_LIGHT/NEURAL/NEURAL_PLUS.
  - To save compute: `pos_tracking.pos_tracking_enabled: false`, `publish_tf`/`publish_map_tf: false` (so the ZED doesn't fight the LIO TF), `depth.depth_stabilization: 0`, `od_enabled`/`bt_enabled: false`.
- **Timestamps:** host clock (`general.sdk_timestamp_clock: SYSTEM_CLOCK`). Once phc2sys disciplines the system clock, they are PTP-aligned.
- **SVO2 recording:** service `/zed/zed_node/start_svo_rec` (`zed_msgs/srv/StartSvoRec`; compression 0 = H265, NVENC), stopped with `stop_svo_rec`. Replay with `svo_file:=` and `svo.use_svo_timestamps: true`.
  - **Recommended:** record an SVO2 for full-quality images alongside an MCAP bag (LiDAR, IMU, TF, compressed preview images).

## LiDAR-camera extrinsic calibration
- **`koide3/direct_visual_lidar_calibration`:** MIT, targetless, ROS2.
  - **Dependencies:** GTSAM 4.2a9, Ceres @e47a42c2, Iridescence.
  - **Docker images are amd64 only**, so run calibration on an x86 PC from copied bags.
  - **Steps:**
    1. `preprocess <bags> <out> -av` (Livox) or `-adv` (Airy, spinning, dynamic)
    2. `initial_guess_manual <out>`. SuperGlue is non-commercial; avoid it.
    3. `calibrate <out>` → `calib.json` (`T_lidar_camera` as xyz + quat)
  - **Bags:** 5-10, each starting static. Livox: hold still 10-15 s. Airy: move slowly up and down for ~10 s. Use the rectified left image + camera_info with zero distortion.
- **Alternatives:** `tier4/CalibrationTools` (GPL-3), `ichangjian/FAST-Calib-ROS2` (target-based, includes Mid-360 samples).

## PTP
- **Orin NIC:** the AGX Orin devkit RJ45 is an MGBE NIC with hardware timestamping (`/dev/ptp0`). **Set `tx_timestamp_timeout 20`.**
- **Mid-360:** IEEE 1588v2 over UDPv4, E2E, or gPTP. It is always a slave. Without PTP, `header.stamp` is **time since LiDAR power-on**.
- **Airy:** set via the web UI at `http://192.168.1.200`, using **PTP-E2E** (L4). Its status page shows Lock/Unlock.
- **Grandmaster config (`gm.conf`):**
  - `gmCapable 1`, `priority1 10`, `domainNumber 0`
  - `network_transport UDPv4`, `delay_mechanism E2E`, `time_stamping hardware`
  - `tx_timestamp_timeout 20`, `logSyncInterval 0`
- **Run:**
  - `ptp4l -f gm.conf -i eth0 -m`
  - `phc2sys -s CLOCK_REALTIME -c eth0 -O 0 -m`
- **Verify:**
  - `pmc -u -b 0 'GET PORT_DATA_SET'`
  - `ros2 topic echo /livox/lidar --field header.stamp` should be close to `date +%s.%N`.
  - A constant 37 s offset means a UTC/TAI mismatch; adjust `-O`.
- **Cabling:** connect the LiDAR directly, or through a PTP-aware switch.

## nvblox
- **Version:** Isaac ROS **3.2** is the Humble / JetPack 6 line. 4.x moved to Jazzy / JetPack 7.
  - apt repo: `https://isaac.download.nvidia.com/isaac-ros/release-3`
  - Packages: `ros-humble-isaac-ros-nvblox`, `ros-humble-nvblox-examples-bringup`, `ros-humble-isaac-ros-h264-encoder` (arm64).
- **Inputs:** `camera_0/depth/image` + `camera_info` (optionally colour), or a LiDAR `pointcloud` with `use_lidar` and the non-equal vertical FoV params.
  - The Airy fits the LiDAR model. The Mid-360's non-repetitive pattern fits poorly, so use ZED depth for the Mid-360.
- **Pose:** `use_tf_transforms: true` with global_frame/pose_frame, so **the TF from LIO works**.
- **ZED example is outdated:** it targets SDK 4.2.2 and the old topic names. Write our own launch with v5 remaps.

## Recording
- **Packages:** `ros-humble-rosbag2-storage-mcap`, `ros-humble-rosbag2-compression-zstd`.
- **Command:** `ros2 bag record -s mcap --storage-preset-profile zstd_fast --max-cache-size 1073741824 ...`
- **Bandwidth:** don't bag raw ZED images (~110 MB/s at HD720). Bag compressed images (or isaac_ros_h264_encoder), plus the SVO2.
- **Performance:** run `sudo nvpmodel -m 0 && sudo jetson_clocks`, and record to NVMe.

## Native vs Docker
**Go native** for drivers, PTP and recording; the Humble arm64 debs exist. Run calibration on x86. Containerise only optional parts if they conflict.
