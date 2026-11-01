#!/usr/bin/env bash
# Installs GLIM (global LiDAR-IMU mapping) from koide3's PPA (arm64 jammy builds exist).
# Usage: 04_install_glim.sh [--cpu]
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "04 GLIM"
cpu=0
[ "${1:-}" = "--cpu" ] && cpu=1

if [ "$cpu" -eq 1 ]; then
  pkgs=(libgtsam-points-dev ros-humble-glim-ros)
  log "installing the CPU-only variant"
else
  require_l4t
  pkgs=("libgtsam-points-${GLIM_CUDA_VARIANT}-dev" "ros-humble-glim-ros-${GLIM_CUDA_VARIANT}")
  log "L4T ${L4T_VERSION} -> GLIM ${GLIM_CUDA_VARIANT} variant"
fi

if ! grep -rqs "koide3.github.io/ppa" /etc/apt/sources.list.d/; then
  log "adding koide3 PPA"
  apt_install curl gpg
  curl -s https://koide3.github.io/ppa/setup_ppa.sh | sudo bash
else
  ok "koide3 PPA already configured"
fi
sudo apt-get update

apt_install libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev zenity "${pkgs[@]}"
sudo ldconfig

[ -d /opt/ros/humble/share/glim/config ] || die "GLIM config dir not found at /opt/ros/humble/share/glim/config"
ok "GLIM installed. Stock config: /opt/ros/humble/share/glim/config"
log "Generate a profile config with: ros2 run dt_tools dt_glim_config --profile <mid360|airy> --out <dir>"
