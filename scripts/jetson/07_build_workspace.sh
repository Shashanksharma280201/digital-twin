#!/usr/bin/env bash
# Imports upstream repos, applies the small required preparations, and builds the ROS 2 workspace.
# Usage: 07_build_workspace.sh [--no-import] [--clean] [-- extra colcon args]
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "07 build workspace (${DT_WS})"
do_import=1; clean=0; extra=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-import) do_import=0 ;;
    --clean) clean=1 ;;
    --) shift; extra=("$@"); break ;;
    *) die "unknown option $1" ;;
  esac
  shift
done

require_ros
have vcs || die "vcstool missing (run 01)"
[ -f /usr/local/lib/liblivox_lidar_sdk_shared.so ] || die "Livox-SDK2 missing (run 02)"

ext="${DT_WS}/src/external"
mkdir -p "$ext"

if [ "$do_import" -eq 1 ]; then
  log "vcs import --recursive -> ${ext}"
  vcs import --recursive --skip-existing "$ext" < "${DT_WS}/dt.repos"
  vcs pull "$ext" || warn "vcs pull reported problems (tags/detached heads are expected)"
fi

# The Orin builds the full Livox driver; the message-only stand-in (laptop, Airy only) would clash with it.
[ -d "${DT_WS}/src/livox_msgs_only" ] && touch "${DT_WS}/src/livox_msgs_only/COLCON_IGNORE"

# --- livox_ros_driver2: do the ROS2 file swap by hand. NEVER run its build.sh (it rm -rf's build/install).
lv="${ext}/livox_ros_driver2"
if [ -d "$lv" ]; then
  log "preparing livox_ros_driver2 for ROS 2"
  cp -f "$lv/package_ROS2.xml" "$lv/package.xml"
  rm -rf "$lv/launch"
  cp -r "$lv/launch_ROS2" "$lv/launch"
fi

# --- rslidar_sdk: XYZIRT points (ring + per-point time) and IMU parsing for the Airy
rs="${ext}/rslidar_sdk"
if [ -d "$rs" ]; then
  log "patching rslidar_sdk CMakeLists (POINT_TYPE XYZIRT, ENABLE_IMU_DATA_PARSE ON)"
  git -C "$rs" submodule update --init --recursive
  before="$(md5sum "$rs/CMakeLists.txt")"
  sed -i 's/^set(POINT_TYPE XYZI)\s*$/set(POINT_TYPE XYZIRT)/' "$rs/CMakeLists.txt"
  sed -i 's/^\(option(ENABLE_IMU_DATA_PARSE[^)]*\)OFF)/\1ON)/' "$rs/CMakeLists.txt"
  grep -q '^set(POINT_TYPE XYZIRT)' "$rs/CMakeLists.txt" || die "rslidar_sdk patch failed: POINT_TYPE"
  grep -Eq '^option\(ENABLE_IMU_DATA_PARSE.*ON\)' "$rs/CMakeLists.txt" || die "rslidar_sdk patch failed: IMU"
  if [ "$before" != "$(md5sum "$rs/CMakeLists.txt")" ]; then
    # option changes only take effect on a clean configure (rslidar_sdk #200)
    log "CMake options changed -> removing stale rslidar_sdk build dir"
    rm -rf "${DT_WS}/build/rslidar_sdk"
  fi
fi

# --- FAST_LIO: ikd-Tree submodule
[ -d "${ext}/FAST_LIO" ] && git -C "${ext}/FAST_LIO" submodule update --init --recursive

# --- zed-ros2-wrapper: tag matching the installed SDK
zw="${ext}/zed-ros2-wrapper"
if [ -d "$zw" ]; then
  resolve_versions
  if [ -n "${ZED_WRAPPER_TAG}" ]; then
    log "checking out zed-ros2-wrapper ${ZED_WRAPPER_TAG}"
    git -C "$zw" fetch --tags --quiet || true
    git -C "$zw" checkout --quiet "${ZED_WRAPPER_TAG}" || warn "tag ${ZED_WRAPPER_TAG} not found; staying on $(git -C "$zw" rev-parse --abbrev-ref HEAD)"
  else
    warn "unknown L4T; leaving zed-ros2-wrapper on its current ref"
  fi
  if [ ! -d /usr/local/zed ]; then
    warn "ZED SDK not installed -> skipping zed packages this build (run 03)"
    touch "$zw/COLCON_IGNORE"
  else
    rm -f "$zw/COLCON_IGNORE"
  fi
fi

cd "$DT_WS"
if [ "$clean" -eq 1 ]; then
  log "--clean: removing build/ install/ log/"
  rm -rf build install log
fi

log "rosdep install"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys "livox_ros_driver2 rslidar_msg rslidar_sdk fast_lio Livox-SDK2 livox_sdk2" || warn "rosdep reported unresolved keys (see above)"

log "colcon build"
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  --packages-skip zed_debug \
  "${extra[@]}"

log "freezing upstream commits -> dt.lock.repos"
vcs export --exact "$ext" > "${DT_WS}/dt.lock.repos" || warn "vcs export failed"

ok "workspace built. Now: source ${DT_ROOT}/scripts/jetson/env.sh"
