#!/usr/bin/env bash
# Installs ROS 2 Humble (ros-base + dev tools) and the system packages the stack needs.
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "01 ROS 2 Humble + system packages"

# -R follows symlinks: ros-apt-source installs ros2.sources as a symlink into /usr/share.
if ! grep -Rqs "packages.ros.org/ros2" /etc/apt/sources.list /etc/apt/sources.list.d/; then
  log "adding the ROS 2 apt repository"
  apt_install curl gnupg lsb-release software-properties-common
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
else
  ok "ROS 2 apt repository already configured"
fi

sudo apt-get update

apt_install \
  ros-humble-ros-base ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pip \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap ros-humble-rosbag2-compression-zstd \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-image-transport-plugins ros-humble-compressed-image-transport \
  ros-humble-tf2-tools ros-humble-tf2-ros ros-humble-tf-transformations \
  ros-humble-robot-state-publisher ros-humble-xacro \
  ros-humble-rviz2 \
  libpcap-dev libyaml-cpp-dev libpcl-dev libeigen3-dev \
  linuxptp ethtool zstd git cmake build-essential

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  log "rosdep init"
  sudo rosdep init
fi
log "rosdep update"
rosdep update --rosdistro humble

ok "ROS 2 Humble ready. Source scripts/jetson/env.sh in every shell."
