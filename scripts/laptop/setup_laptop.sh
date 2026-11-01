#!/usr/bin/env bash
# Sets up an x86-64 Ubuntu 22.04 laptop (ROS 2 Humble, ZED SDK already installed) to run the
# live stack with the RoboSense Airy + ZED 2i. Run it yourself in a terminal (it asks for sudo).
#
# Usage: scripts/laptop/setup_laptop.sh [--iface enp3s0] [--no-build]
#   1. apt packages (CycloneDDS, ZED messages/description, image transport, libpcap, PTP tools)
#   2. LiDAR network on the wired port (static 192.168.1.5 + .102, no default route; Wi-Fi untouched)
#   3. PTP grandmaster for the Airy (software timestamps if the NIC has no PTP hardware)
#   4. build the ROS workspace
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JET="${HERE}/../jetson"
# shellcheck source=../jetson/common.sh
source "${JET}/common.sh"

iface="enp3s0"; build=1
while [ $# -gt 0 ]; do
  case "$1" in
    --iface) iface="$2"; shift ;;
    --no-build) build=0 ;;
    *) die "unknown option $1" ;;
  esac
  shift
done
[ "$(uname -m)" = "x86_64" ] || warn "meant for an x86-64 laptop; on the Orin use scripts/jetson/setup_everything.sh"
[ -d /usr/local/zed ] || die "ZED SDK not found in /usr/local/zed (install it from stereolabs.com for your CUDA version first)"
ip link show "$iface" >/dev/null 2>&1 || die "no interface ${iface}; pass the wired port with --iface"

step "1/4 apt packages"
sudo apt-get update
apt_install ros-humble-rmw-cyclonedds-cpp ros-humble-zed-msgs ros-humble-zed-description ros-humble-nmea-msgs \
  ros-humble-image-transport-plugins ros-humble-compressed-image-transport ros-humble-compressed-depth-image-transport \
  ros-humble-theora-image-transport ros-humble-rosbag2-storage-mcap ros-humble-rosbag2-compression-zstd \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-tf2-tools \
  libpcap-dev libyaml-cpp-dev libapr1-dev libaprutil1-dev linuxptp ethtool

step "2/4 LiDAR network on ${iface}"
"${JET}/05_setup_network.sh" "$iface"

step "3/4 PTP for the Airy on ${iface}"
"${JET}/06_setup_ptp.sh" "$iface"

if [ "$build" -eq 1 ]; then
  step "4/4 build the workspace (${DT_WS})"
  require_ros
  cd "$DT_WS"
  rm -f src/external/zed-ros2-wrapper/COLCON_IGNORE
  # Airy-only laptop: build just the Livox message definitions (src/livox_msgs_only, needed at compile
  # time by FAST-LIO and dt_lidar_bridge), not the Livox driver, so no Livox-SDK2 version is required.
  touch src/external/livox_ros_driver2/COLCON_IGNORE
  colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
    --packages-skip zed_debug
fi

ok "laptop ready. Connect the Airy to ${iface} and the ZED to a USB 3 port, then:"
cat <<EOF
  source ${JET}/env.sh
  ros2 launch dt_bringup capture.launch.py profile:=airy rviz:=true rviz_config:=calib
EOF
