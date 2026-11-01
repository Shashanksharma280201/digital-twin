#!/usr/bin/env bash
# Installs the ZED SDK build matching this Jetson's L4T release.
# Keeps the AI/object-detection module: NEURAL depth modes depend on it.
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "03 ZED SDK"
require_l4t
log "L4T ${L4T_VERSION} -> ZED SDK ${ZED_SDK_VERSION} (${ZED_SDK_URL})"

installed=""
if [ -f /usr/local/zed/include/sl/Camera.hpp ]; then
  major="$(sed -n 's/#define ZED_SDK_MAJOR_VERSION \([0-9]*\).*/\1/p' /usr/local/zed/include/sl/Camera.hpp)"
  minor="$(sed -n 's/#define ZED_SDK_MINOR_VERSION \([0-9]*\).*/\1/p' /usr/local/zed/include/sl/Camera.hpp)"
  installed="${major}.${minor}"
fi
if [ "$installed" = "$ZED_SDK_VERSION" ] && [ "${1:-}" != "--force" ]; then
  ok "ZED SDK ${installed} already installed (pass --force to reinstall)"
else
  [ -n "$installed" ] && warn "ZED SDK ${installed} installed; replacing with ${ZED_SDK_VERSION}"
  sudo apt-get update
  # nvidia-jetpack-dev provides the CUDA toolkit paths the wrapper build needs on JP6
  apt_install zstd wget nvidia-jetpack nvidia-jetpack-dev
  mkdir -p "$DT_DEPS"
  run="${DT_DEPS}/ZED_SDK_L4T${L4T_VERSION}_v${ZED_SDK_VERSION}.zstd.run"
  if [ ! -s "$run" ]; then
    log "downloading ${ZED_SDK_URL}"
    wget -q --show-progress -O "$run" "$ZED_SDK_URL"
  fi
  chmod +x "$run"
  log "running installer silently (skip_tools; object-detection module kept)"
  "$run" -- silent skip_tools
  ok "ZED SDK installed"
fi

# The ZED SDK needs the user in the video group for USB access
if ! id -nG "$USER" | grep -qw video; then
  log "adding ${USER} to the video group (log out/in afterwards)"
  sudo usermod -aG video "$USER"
fi

log "Use zed-ros2-wrapper tag: ${ZED_WRAPPER_TAG} (07_build_workspace.sh checks it out)"
