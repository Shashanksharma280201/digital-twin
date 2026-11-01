#!/usr/bin/env bash
# Shared helpers for the Jetson setup scripts. Source, do not execute.
# shellcheck disable=SC2034

_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DT_ROOT="${DT_ROOT:-$(cd "${_COMMON_DIR}/../.." && pwd)}"
DT_WS="${DT_WS:-${DT_ROOT}/ros2_ws}"
DT_DATA="${DT_DATA:-${HOME}/dt_data}"
DT_DEPS="${DT_DEPS:-${HOME}/dt_deps}"
ROS_DISTRO_EXPECTED="humble"

log()  { printf '\033[1;34m[dt]\033[0m %s\n' "$*"; }
step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()   { printf '\033[1;32m[ok]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; exit 1; }

have() { command -v "$1" >/dev/null 2>&1; }

# apt install only the packages that are missing
apt_install() {
  local missing=()
  local p
  for p in "$@"; do
    dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
  done
  if [ "${#missing[@]}" -eq 0 ]; then
    ok "already installed: $*"
    return 0
  fi
  log "apt install ${missing[*]}"
  sudo apt-get install -y "${missing[@]}"
}

# Sets L4T_MAJOR, L4T_MINOR, L4T_VERSION (e.g. 36.4) from /etc/nv_tegra_release.
# Example line: "# R36 (release), REVISION: 4.3, GCID: ..., BOARD: generic, ..."
detect_l4t() {
  L4T_MAJOR=""; L4T_MINOR=""; L4T_VERSION=""
  if [ -r /etc/nv_tegra_release ]; then
    L4T_MAJOR="$(sed -n 's/^# R\([0-9]*\) (release).*/\1/p' /etc/nv_tegra_release)"
    L4T_MINOR="$(sed -n 's/.*REVISION: \([0-9]*\)\..*/\1/p' /etc/nv_tegra_release)"
    if [ -n "$L4T_MAJOR" ] && [ -n "$L4T_MINOR" ]; then
      L4T_VERSION="${L4T_MAJOR}.${L4T_MINOR}"
    fi
  fi
}

# Maps L4T -> ZED SDK download URL + version, GLIM CUDA package variant.
# Facts from research/06 (ZED) and research/05 (GLIM PPA).
resolve_versions() {
  detect_l4t
  ZED_SDK_VERSION=""; ZED_SDK_URL=""; ZED_WRAPPER_TAG=""; GLIM_CUDA_VARIANT=""
  case "$L4T_VERSION" in
    36.3)
      ZED_SDK_VERSION="5.2"
      ZED_SDK_URL="https://download.stereolabs.com/zedsdk/5.2/l4t36.3/jetsons"
      ZED_WRAPPER_TAG="v5.2.2"
      GLIM_CUDA_VARIANT="cuda12.2"
      ;;
    36.4)
      ZED_SDK_VERSION="5.5"
      ZED_SDK_URL="https://download.stereolabs.com/zedsdk/5.5/l4t36.4/jetsons"
      ZED_WRAPPER_TAG="v5.5.0"
      GLIM_CUDA_VARIANT="cuda12.6"
      ;;
    36.5)
      ZED_SDK_VERSION="5.5"
      ZED_SDK_URL="https://download.stereolabs.com/zedsdk/5.5/l4t36.5/jetsons"
      ZED_WRAPPER_TAG="v5.5.0"
      GLIM_CUDA_VARIANT="cuda12.6"
      ;;
    *)
      ;;
  esac
}

require_l4t() {
  resolve_versions
  [ -n "$L4T_VERSION" ] || die "Not a Jetson (no /etc/nv_tegra_release). These scripts target JetPack 6 / L4T 36.x."
  [ -n "$ZED_SDK_URL" ] || die "Unsupported L4T ${L4T_VERSION}. Supported: 36.3 (JP6.0), 36.4 (JP6.1/6.2), 36.5 (JP6.2.2)."
}

require_ros() {
  if [ ! -f "/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash" ]; then
    die "ROS 2 ${ROS_DISTRO_EXPECTED} not found. Run 01_install_ros_humble.sh first."
  fi
  # setup.bash references unset vars; relax nounset while sourcing
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_EXPECTED}/setup.bash"
  set -u
}

# First wired ethernet interface that is not a bridge/docker/can/usb gadget
detect_wired_iface() {
  local i
  for i in /sys/class/net/*; do
    i="$(basename "$i")"
    case "$i" in
      lo|docker*|br-*|veth*|virbr*|can*|l4tbr*|usb*|rndis*|wl*|tailscale*|zt*) continue ;;
    esac
    [ -d "/sys/class/net/$i/device" ] || continue
    echo "$i"
    return 0
  done
  return 1
}
