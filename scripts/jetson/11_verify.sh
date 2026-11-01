#!/usr/bin/env bash
# Verifies the install end to end on the Orin, with no sensors attached:
#   1. unit tests of the ROS packages (bridge, bringup, tools, blender bridge)
#   2. Blender sim on the Orin: world + DT-1 + walk -> simulated sensor bags -> real stack on the Orin
#      (bridge + FAST-LIO, CycloneDDS) -> maps -> change detection -> accuracy vs ground truth.
# Usage: 11_verify.sh [--profile mid360|airy] [--camera] [--cam-rate HZ] [--skip-sim]
#   default is LiDAR-only (fast); --camera also renders ZED frames on the Orin GPU and fuses colour.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=common.sh
source "${HERE}/common.sh"

profile=mid360; camera=0; cam_rate=2; sim=1
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) profile="$2"; shift ;;
    --camera) camera=1 ;;
    --cam-rate) cam_rate="$2"; shift ;;
    --skip-sim) sim=0 ;;
    *) die "unknown option $1" ;;
  esac
  shift
done

step "11a unit tests (ROS packages)"
require_ros
[ -f "${DT_WS}/install/setup.bash" ] || die "workspace not built (run 07)"
set +u
# shellcheck disable=SC1091
source "${DT_WS}/install/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
[ -f "${HOME}/.config/dt/cyclonedds.xml" ] && export CYCLONEDDS_URI="file://${HOME}/.config/dt/cyclonedds.xml"
(cd "$DT_WS" && colcon test --packages-select dt_lidar_bridge dt_bringup dt_tools dt_blender_bridge \
   --event-handlers console_direct- >/dev/null; colcon test-result --test-result-base build) \
  || die "unit tests failed (see: colcon test-result --verbose)"
ok "unit tests passed"

[ "$sim" -eq 1 ] || { ok "skipping the simulation run"; exit 0; }

step "11b Blender sim on the Orin (${profile}$([ "$camera" -eq 1 ] && echo ", camera ${cam_rate} Hz" || echo ", LiDAR only"))"
have blender || die "blender missing (run 09)"
venv="${DT_DATA}/venvs/dt"
[ -x "${venv}/bin/dt_sim" ] || die "Python tools missing (run 08)"
sudo nvpmodel -m 0 >/dev/null 2>&1 && sudo jetson_clocks >/dev/null 2>&1 || warn "could not set max performance mode"

run="${DT_DATA}/sim/agx_${profile}"
mkdir -p "${DT_DATA}/sim"
args=(--profile "$profile" --run "$run" --no-hero)
[ "$camera" -eq 1 ] && args+=(--cam-rate "$cam_rate") || args+=(--no-camera)
t0=$(date +%s)
DT_WS="$DT_WS" DT_VENV="$venv" ROS_DOMAIN_ID=61 "${DT_ROOT}/sim/run_demo.sh" "${args[@]}" 2>&1 | tee "${run}.log"
log "simulation took $(( ($(date +%s) - t0) / 60 )) min"

step "11c results vs ground truth"
"${venv}/bin/python" - "$run" <<'EOF'
import json, sys
from pathlib import Path
run = Path(sys.argv[1]); ok = True
checks = [("ATE aligned rmse", lambda e: e["trajectory"]["aligned"]["rmse_m"], 0.05, "m"),
          ("drift (anchored)", lambda e: e["trajectory"]["anchored"]["drift_pct"], 0.5, "%"),
          ("map mean error", lambda e: e["map"]["mean_m"], 0.05, "m"),
          ("map completeness", lambda e: -e["map"]["completeness_pct"], -70, "%")]
for day in ("day1", "day2"):
    e = json.loads((run / f"eval_{day}.json").read_text())
    for name, get, limit, unit in checks:
        v = get(e); good = v <= limit; ok &= good
        shown, lim = (abs(v), abs(limit)) if limit < 0 else (v, limit)
        print(f"  {day} {name:18s} {shown:8.3f} {unit}  ({'>=' if limit < 0 else '<='} {lim})  {'PASS' if good else 'FAIL'}")
print("RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
EOF
ok "outputs in ${run} (result_day1.png, result_day2.png, eval_*.json)"
