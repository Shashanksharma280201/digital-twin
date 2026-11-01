#!/usr/bin/env bash
# Source this in every shell:  source scripts/jetson/env.sh
# (Not executed: no `set -e` so it never kills the interactive shell.)

_dt_here="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
export DT_ROOT="${DT_ROOT:-$(cd "${_dt_here}/../.." && pwd)}"
export DT_WS="${DT_WS:-${DT_ROOT}/ros2_ws}"
export DT_DATA="${DT_DATA:-${HOME}/dt_data}"
mkdir -p "${DT_DATA}/sessions"

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[dt] ROS 2 Humble not installed (scripts/jetson/01_install_ros_humble.sh)" >&2
fi
if [ -f "${DT_WS}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${DT_WS}/install/setup.bash"
else
  echo "[dt] workspace not built yet (scripts/jetson/07_build_workspace.sh)" >&2
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ -f "${HOME}/.config/dt/cyclonedds.xml" ]; then
  export CYCLONEDDS_URI="file://${HOME}/.config/dt/cyclonedds.xml"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Offline tools (dtp, dt_sim) from 08 and Blender from 09.
export DT_VENV="${DT_VENV:-${DT_DATA}/venvs/dt}"
[ -d "${DT_VENV}/bin" ] && export PATH="${PATH}:${DT_VENV}/bin"
[ -x /usr/local/bin/blender ] && export BLENDER=/usr/local/bin/blender

unset _dt_here
echo "[dt] DT_ROOT=${DT_ROOT}  DT_DATA=${DT_DATA}  RMW=${RMW_IMPLEMENTATION}"
