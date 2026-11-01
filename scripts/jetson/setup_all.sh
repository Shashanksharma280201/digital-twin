#!/usr/bin/env bash
# Runs the setup steps 00..07 in order.
# Usage: setup_all.sh [--iface eth0] [--skip 03,04] [--glim-cpu]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${HERE}/common.sh"

iface=""; skip=""; glim_args=()
while [ $# -gt 0 ]; do
  case "$1" in
    --iface) iface="$2"; shift ;;
    --skip) skip="$2"; shift ;;
    --glim-cpu) glim_args=(--cpu) ;;
    -h|--help) sed -n '2,3p' "$0"; exit 0 ;;
    *) die "unknown option $1" ;;
  esac
  shift
done

skipped() { [[ ",${skip}," == *",$1,"* ]]; }

run() {
  local n="$1"; shift
  if skipped "$n"; then log "skipping step ${n}"; return 0; fi
  "$@"
}

run 00 "${HERE}/00_check_system.sh"
run 01 "${HERE}/01_install_ros_humble.sh"
run 02 "${HERE}/02_install_livox_sdk2.sh"
run 03 "${HERE}/03_install_zed_sdk.sh"
run 04 "${HERE}/04_install_glim.sh" "${glim_args[@]}"
if [ -n "$iface" ]; then
  run 05 "${HERE}/05_setup_network.sh" "$iface"
  run 06 "${HERE}/06_setup_ptp.sh" "$iface"
else
  run 05 "${HERE}/05_setup_network.sh"
  run 06 "${HERE}/06_setup_ptp.sh"
fi
run 07 "${HERE}/07_build_workspace.sh"

ok "setup complete. In each new shell: source ${HERE}/env.sh"
