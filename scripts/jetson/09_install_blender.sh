#!/usr/bin/env bash
# Builds Blender from source for Linux ARM64 (Jetson) and installs it as `blender`.
#
# Why build: blender.org, snap, Flathub and the daily builds ship x86-64 Linux only, and Ubuntu 22.04's
# apt Blender is 3.0 (too old for sim/blender, which targets the 5.x API). Blender does publish
# precompiled ARM64 dependency libraries (projects.blender.org/blender/lib-linux_arm64, branch
# blender-v5.2-release), so a source build only compiles Blender itself.
#
# Cost: ~15 GB disk (sources + libs + build), ~20-40 min download, ~60-120 min compile on AGX Orin.
# Usage: 09_install_blender.sh [--branch blender-v5.2-release] [--jobs N] [--prefix /opt/blender-5.2] [--cuda]
#   --cuda   also build Cycles CUDA kernels for Orin (sm_87); not needed for the sim (EEVEE)
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "09 Blender (source build for ARM64)"
branch="blender-v5.2-release"; jobs="$(nproc)"; prefix="/opt/blender-5.2"; cuda=0
while [ $# -gt 0 ]; do
  case "$1" in
    --branch) branch="$2"; shift ;;
    --jobs) jobs="$2"; shift ;;
    --prefix) prefix="$2"; shift ;;
    --cuda) cuda=1 ;;
    *) die "unknown option $1" ;;
  esac
  shift
done

if have blender && blender -b --factory-startup --python-expr "import bpy,sys; sys.exit(0 if bpy.app.version >= (4,2,0) else 1)" >/dev/null 2>&1; then
  ok "blender already installed: $(blender --version 2>/dev/null | head -1)"
  exit 0
fi

arch="$(uname -m)"
[ "$arch" = "aarch64" ] || warn "this script is meant for aarch64 (found ${arch}); on x86-64 just install the official build"
free_gb="$(df -BG --output=avail "${DT_DEPS%/*}" | tail -1 | tr -dc 0-9)"
[ "${free_gb:-0}" -ge 25 ] || warn "only ${free_gb} GB free; the build needs ~15 GB (25 GB recommended)"

apt_install git git-lfs build-essential cmake python3 libx11-dev libxxf86vm-dev libxcursor-dev libxi-dev \
  libxrandr-dev libxinerama-dev libegl-dev libwayland-dev wayland-protocols libxkbcommon-dev libdbus-1-dev \
  linux-libc-dev subversion
git lfs install --skip-repo >/dev/null

src="${DT_DEPS}/blender-git/blender"
mkdir -p "${DT_DEPS}/blender-git"
if [ ! -d "${src}/.git" ]; then
  log "cloning Blender ${branch} (shallow)"
  git clone --depth 1 --branch "$branch" https://projects.blender.org/blender/blender.git "$src"
fi
cd "$src"
[ "$(git rev-parse --abbrev-ref HEAD)" = "$branch" ] || die "${src} is on $(git rev-parse --abbrev-ref HEAD), expected ${branch}; remove it or pass --branch"

log "installing Blender's distro build dependencies"
python3 ./build_files/build_environment/install_linux_packages.py || warn "install_linux_packages.py reported issues; continuing (precompiled libs cover most deps)"

log "make update: git LFS content + submodules"
# On aarch64 the release branch registers no lib/linux_arm64 submodule, so make update ends with
# "Skipping libraries update: no configured submodule" (verified on blender-v5.2-release).
make update || [ "$arch" = "aarch64" ] || die "make update failed"
if [ "$arch" = "aarch64" ] && [ ! -d lib/linux_arm64/.git ]; then
  log "fetching precompiled ARM64 libraries (lib-linux_arm64 ${branch}, ~2-3 GB)"
  git clone --depth 1 --branch "$branch" https://projects.blender.org/blender/lib-linux_arm64.git lib/linux_arm64
  git -C lib/linux_arm64 lfs pull
fi

cmake_extra=""
if [ "$cuda" -eq 1 ]; then
  cmake_extra="-DWITH_CYCLES_CUDA_BINARIES=ON -DCYCLES_CUDA_BINARIES_ARCH=sm_87 -DWITH_CYCLES_DEVICE_OPTIX=OFF"
fi
log "building Blender with ${jobs} jobs (this takes 1-2 hours on AGX Orin)"
make -j"$jobs" BUILD_CMAKE_ARGS="${cmake_extra}"

build_bin="${DT_DEPS}/blender-git/build_linux/bin"
[ -x "${build_bin}/blender" ] || die "build finished but ${build_bin}/blender not found"
log "installing to ${prefix}"
sudo rm -rf "$prefix"
sudo mkdir -p "$prefix"
sudo cp -a "${build_bin}/." "$prefix/"
sudo ln -sf "${prefix}/blender" /usr/local/bin/blender

log "smoke test: headless Blender + Python API"
blender -b --factory-startup --python-expr "import bpy, sys; print('BLENDER_OK', bpy.app.version_string, sys.version.split()[0])" 2>&1 | grep BLENDER_OK \
  || die "blender does not start headless"

log "smoke test: headless EEVEE render on the Orin GPU (EGL)"
tmp="$(mktemp -d)"
"${DT_ROOT}/sim/blender/bl.sh" "${DT_ROOT}/scripts/jetson/blender_render_check.py" "${tmp}/check.png" \
  && ok "EEVEE render ok: ${tmp}/check.png" \
  || warn "EEVEE headless render failed: the sim can still generate worlds/LiDAR (use --no-camera), or render cameras on an x86 PC"
ok "Blender installed: $(blender --version 2>/dev/null | head -1) -> /usr/local/bin/blender"
