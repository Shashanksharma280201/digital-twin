#!/usr/bin/env bash
# Read-only system check. Warns, never fails.
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "00 system check (read-only)"

resolve_versions
if [ -n "$L4T_VERSION" ]; then
  ok "L4T ${L4T_VERSION}"
  if [ -n "$ZED_SDK_URL" ]; then
    log "  -> ZED SDK ${ZED_SDK_VERSION} (wrapper tag ${ZED_WRAPPER_TAG}), GLIM variant ${GLIM_CUDA_VARIANT}"
  else
    warn "L4T ${L4T_VERSION} not in the supported table (36.3/36.4/36.5)"
  fi
else
  warn "No /etc/nv_tegra_release: this is not a Jetson"
fi

if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  if [ "${VERSION_ID:-}" = "22.04" ]; then ok "Ubuntu ${VERSION_ID}"; else warn "Ubuntu ${VERSION_ID:-?} (expected 22.04 for ROS 2 Humble)"; fi
fi

if have nvcc; then
  ok "CUDA: $(nvcc --version | sed -n 's/.*release \([0-9.]*\).*/\1/p')"
elif [ -x /usr/local/cuda/bin/nvcc ]; then
  ok "CUDA: $(/usr/local/cuda/bin/nvcc --version | sed -n 's/.*release \([0-9.]*\).*/\1/p') (not on PATH)"
else
  warn "nvcc not found (install nvidia-jetpack-dev in 03)"
fi

if have nvpmodel; then
  log "power mode: $(sudo -n nvpmodel -q 2>/dev/null | head -n 2 | tr '\n' ' ' || echo 'run: sudo nvpmodel -q')"
  log "  for capture use: sudo nvpmodel -m 0 && sudo jetson_clocks"
fi

step "Disk"
df -h / "${HOME}" 2>/dev/null | sort -u
if lsblk -d -o NAME,TRAN,SIZE 2>/dev/null | grep -q nvme; then
  ok "NVMe present:"; lsblk -d -o NAME,SIZE,MOUNTPOINT | grep nvme || true
else
  warn "No NVMe found. Recording LiDAR+camera to eMMC/SD will drop data."
fi
avail_gb="$(df -BG --output=avail "${HOME}" | tail -n1 | tr -dc '0-9')"
if [ "${avail_gb:-0}" -lt 100 ]; then warn "Only ${avail_gb} GB free in \$HOME (plan ~1-2 GB per capture minute)"; fi

step "Network"
ip -br addr || true
iface="$(detect_wired_iface || true)"
if [ -n "$iface" ]; then
  log "wired interface guess: ${iface}"
  if have ethtool; then
    ts="$(ethtool -T "$iface" 2>/dev/null || true)"
    if echo "$ts" | grep -q "hardware-transmit"; then
      ok "PTP hardware timestamping available on ${iface} ($(echo "$ts" | grep -i 'PTP Hardware Clock' || true))"
    else
      warn "ethtool -T ${iface} shows no hardware-transmit (PTP will fall back to software; newer ethtool may be needed)"
    fi
  else
    warn "ethtool not installed (01 installs it)"
  fi
else
  warn "No wired interface detected"
fi

step "Cameras / USB"
if lsusb 2>/dev/null | grep -qi "2b03"; then
  ok "ZED camera on USB: $(lsusb | grep -i 2b03)"
else
  warn "No ZED (USB vendor 2b03) detected. Use a USB 3 port."
fi

step "ROS"
if [ -f /opt/ros/humble/setup.bash ]; then ok "ROS 2 Humble installed"; else warn "ROS 2 Humble missing (run 01)"; fi
if [ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ]; then ok "Livox-SDK2 installed"; else warn "Livox-SDK2 missing (run 02)"; fi
if [ -d /usr/local/zed ]; then ok "ZED SDK installed: $(cat /usr/local/zed/include/sl/Camera.hpp 2>/dev/null | sed -n 's/#define ZED_SDK_\(MAJOR\|MINOR\|PATCH\)_VERSION \([0-9]*\)/\2/p' | paste -sd. -)"; else warn "ZED SDK missing (run 03)"; fi
if dpkg -l 2>/dev/null | grep -q "ros-humble-glim-ros"; then ok "GLIM installed: $(dpkg -l | awk '/ros-humble-glim-ros/{print $2" "$3}')"; else warn "GLIM missing (run 04)"; fi

log "check done"
