#!/usr/bin/env bash
# Builds Livox-SDK2 into /usr/local (needed by livox_ros_driver2 for the Mid-360).
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "02 Livox-SDK2 -> /usr/local"

if [ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ] && [ "${1:-}" != "--force" ]; then
  ok "Livox-SDK2 already installed (pass --force to rebuild)"
  exit 0
fi

apt_install cmake build-essential git
mkdir -p "$DT_DEPS"
src="${DT_DEPS}/Livox-SDK2"
if [ ! -d "$src/.git" ]; then
  log "cloning Livox-SDK2 into ${src}"
  git clone https://github.com/Livox-SDK/Livox-SDK2.git "$src"
else
  log "updating ${src}"
  git -C "$src" pull --ff-only || warn "could not fast-forward ${src}; building current checkout"
fi

build() {
  rm -rf "$src/build"
  cmake -S "$src" -B "$src/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$src/build" -j"$(nproc)"
}

# Newer GCC needs <cstdint> in two files. 22.04 normally builds without it; patch only on failure.
add_cstdint() {
  local f
  for f in "$src/sdk_core/comm/define.h" "$src/sdk_core/logger_handler/file_manager.h"; do
    if [ -f "$f" ] && ! grep -q '#include <cstdint>' "$f"; then
      log "adding #include <cstdint> to ${f#"$src"/}"
      sed -i '0,/#include/s//#include <cstdint>\n#include/' "$f"
    fi
  done
}

if ! build; then
  warn "build failed; applying the <cstdint> include fix and retrying"
  add_cstdint
  build
fi

sudo cmake --install "$src/build"
sudo ldconfig
[ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ] || die "liblivox_lidar_sdk_shared.so not found after install"
ok "Livox-SDK2 installed"
