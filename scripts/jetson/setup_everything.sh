#!/usr/bin/env bash
# ONE-SHOT SETUP for the Jetson AGX Orin (JetPack 6 / L4T 36.x): installs and verifies everything.
#
#   00 system check            05 LiDAR network + DDS buffers     09 Blender 5.2 (ARM64 source build)
#   01 ROS 2 Humble + tools    06 PTP grandmaster (systemd)       10 Gaussian splatting (opt-in, experimental)
#   02 Livox-SDK2              07 ROS workspace (drivers, FAST-LIO, dt_* packages incl. Blender bridge)
#   03 ZED SDK                 08 Python tools (dtp, dt_sim)      11 verify: unit tests + Blender sim on the Orin
#   04 GLIM
#
# Usage:
#   ./setup_everything.sh --iface eth0                  # full install + LiDAR-only sim verification
#   ./setup_everything.sh --iface eth0 --verify-camera  # verification also renders ZED frames on the Orin
#   ./setup_everything.sh --from 07                     # resume from a step after fixing something
#   ./setup_everything.sh --skip 09,11                  # e.g. skip Blender build and verification
#   ./setup_everything.sh --with-splat                  # also try splat training deps on the Orin
#   ./setup_everything.sh --glim-cpu --profile airy
#
# Time: roughly 3-4 hours on a fresh Orin (Blender compile ~1-2 h, workspace build ~30 min).
# Disk: ~40 GB free recommended. Network: internet needed (Wi-Fi is fine; step 05 takes over the RJ45 port).
# Log: ~/dt_data/setup_logs/setup_<timestamp>.log. Every step is idempotent and safe to re-run.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${HERE}/common.sh"

iface=""; skip=""; from="00"; glim_args=(); splat=0; verify_args=()
while [ $# -gt 0 ]; do
  case "$1" in
    --iface) iface="$2"; shift ;;
    --skip) skip="$2"; shift ;;
    --from) from="$2"; shift ;;
    --glim-cpu) glim_args=(--cpu) ;;
    --with-splat) splat=1 ;;
    --verify-camera) verify_args+=(--camera) ;;
    --profile) verify_args+=(--profile "$2"); shift ;;
    -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
  shift
done

mkdir -p "${DT_DATA}/setup_logs"
logfile="${DT_DATA}/setup_logs/setup_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee -a "$logfile") 2>&1
log "log file: ${logfile}"

[ "$(uname -m)" = "aarch64" ] || warn "not aarch64: this script targets the Jetson AGX Orin"
[ "$(id -u)" -ne 0 ] || die "run as your normal user (it calls sudo when needed), not as root"

# Ask for sudo once and keep the ticket alive for the long build steps.
# `sudo -v` prompts even with a NOPASSWD rule when another rule (e.g. %sudo) needs a password,
# so try a non-interactive sudo first.
sudo -n true 2>/dev/null || sudo -v
( while true; do sudo -n true; sleep 50; kill -0 "$$" 2>/dev/null || exit; done ) 2>/dev/null &
keepalive=$!
trap 'kill "$keepalive" 2>/dev/null || true' EXIT

declare -A took
run() {
  local n="$1"; shift
  if [[ ",${skip}," == *",${n},"* ]] || [ "$((10#$n))" -lt "$((10#$from))" ]; then
    log "skipping step ${n}"; took[$n]="skipped"; return 0
  fi
  local t0; t0=$(date +%s)
  "$@" || die "step ${n} failed. Fix the error above, then resume with: $0 --from ${n} ${iface:+--iface $iface}"
  took[$n]="$(( ($(date +%s) - t0) / 60 )) min"
}

iface_args=(); [ -n "$iface" ] && iface_args=("$iface")
run 00 "${HERE}/00_check_system.sh"
run 01 "${HERE}/01_install_ros_humble.sh"
run 02 "${HERE}/02_install_livox_sdk2.sh"
run 03 "${HERE}/03_install_zed_sdk.sh"
run 04 "${HERE}/04_install_glim.sh" "${glim_args[@]}"
run 05 "${HERE}/05_setup_network.sh" "${iface_args[@]}"
run 06 "${HERE}/06_setup_ptp.sh" "${iface_args[@]}"
run 07 "${HERE}/07_build_workspace.sh"
run 08 "${HERE}/08_install_python_tools.sh"
run 09 "${HERE}/09_install_blender.sh"
if [ "$splat" -eq 1 ]; then run 10 "${HERE}/10_install_splat.sh"; else took[10]="not requested"; fi
run 11 "${HERE}/11_verify.sh" "${verify_args[@]}"

step "summary"
for n in 00 01 02 03 04 05 06 07 08 09 10 11; do printf '  step %s  %s\n' "$n" "${took[$n]:-}"; done
cat <<EOF

Everything is installed. In every new shell:
  source ${HERE}/env.sh

Next (see docs/test-plan/2026-10-19-agx-real-hardware-test-plan.md):
  - Airy: set its time source to PTP-E2E at http://192.168.1.200 (one time)
  - sensors only:   ros2 launch dt_bringup capture.launch.py profile:=airy slam:=none
  - live LIO:       ros2 launch dt_bringup capture.launch.py profile:=airy
  - Blender sim:    ${DT_ROOT}/sim/run_demo.sh --profile airy --run ~/dt_data/sim/agx_airy
EOF
