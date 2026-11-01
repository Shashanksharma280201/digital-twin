#!/usr/bin/env bash
# One command from a recorded session to the twin. GPU (PyTorch / Open3D CUDA) where available:
#   ingest   read the bag once: scan cache + FAST-LIO trajectory (mixed pixels at edges dropped)
#   refine   keyframe pose graph with checked loop closures
#   fuse     voxel map + TSDF mesh in one GPU pass (operator/rig zone removed, fragments dropped)
#   colour   map points and mesh vertices from the camera (SVO2, else bag images): keyframes,
#            exposure compensation, best views
#   texture  image-textured mesh from the best views (products/textured/mesh.obj)
#   report   quality numbers + plots
# Prints the time of each step.
#
# Usage: scripts/process_session.sh SESSION [--self-zone 0.8] [--no-refine] [--calibrate-rings] [--ring-calib FILE] [--polish] [--no-mesh] [--no-texture]
#   --self-zone R   drop returns closer than R metres (operator and rig) from map and mesh
#   --calibrate-rings  learn each LiDAR ring's range/elevation error from this recording (ring_calib.json),
#                   then redo ingest + refine + fuse with it. The calibration belongs to the sensor: real
#                   sessions take config/profiles/<profile>/ring_calib.json unless they hold their own,
#                   and --ring-calib FILE copies one in.
#   --polish        also align scans to the fused surface and fuse again (+~2 min; no gain measured
#                   on capture 3, where per-scan errors were already ~4 mm)
# Products: SESSION/products/{traj_lidar.tum, map.laz, map_loose.laz, map_rgb.laz, mesh.ply, textured/, quality/}
#           SESSION/products/logs/process.log (full output of every step)
#           SESSION/products/cache/ (scan cache, ~1.5 GB per minute of Airy capture; safe to delete)
# View:     ~/dt_data/venvs/dt/bin/python scripts/view_map.py SESSION        (points)
#           ~/dt_data/venvs/dt/bin/python scripts/view_map.py SESSION --mesh (mesh)
set -eo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=jetson/env.sh
source "${here}/jetson/env.sh"
py="${DT_VENV}/bin/python"

session="$1"; shift || true
self_zone=0.8; refine=1; polish=0; mesh=1; calibrate=0; ring_file=""; texture=1
while [ $# -gt 0 ]; do
  case "$1" in
    --self-zone) self_zone="$2"; shift ;;
    --no-refine) refine=0 ;;
    --polish) polish=1 ;;
    --calibrate-rings) calibrate=1 ;;
    --ring-calib) ring_file="$2"; shift ;;
    --no-mesh) mesh=0; texture=0 ;;
    --no-texture) texture=0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done
[ -d "$session" ] || { echo "usage: $0 SESSION [options]" >&2; exit 2; }
mesh_flag=(); [ "$mesh" -eq 1 ] || mesh_flag=(--no-mesh)

mkdir -p "$session/products/logs"
log="$session/products/logs/process.log"; : >"$log"
run() {  # run NAME CMD... ; full output goes to $log, a failing step stops the run
  local t0=$SECONDS
  echo "== $1"
  shift
  "$@" 2>&1 | tee -a "$log" | { grep -E "INFO|WARN|ERROR|Error|\{" | grep -v "^\[Open3D" || true; }
  echo "   $((SECONDS - t0)) s"
}
[ -z "$ring_file" ] || cp -f "$ring_file" "$session/ring_calib.json"
profile=""  # sim sessions have no meta.yaml and no profile calibration
[ -f "$session/meta.yaml" ] && profile="$(sed -n 's/^profile: *//p' "$session/meta.yaml" | head -1)"
[ -f "$session/meta.yaml" ] && grep -q '^rehearsal: true' "$session/meta.yaml" && profile=""
profile_calib="${here}/../ros2_ws/src/dt_bringup/config/profiles/${profile}/ring_calib.json"
if [ -n "$profile" ] && [ -f "$profile_calib" ] && [ ! -f "$session/ring_calib.json" ]; then
  echo "== ring calibration from the ${profile} profile"
  cp -f "$profile_calib" "$session/ring_calib.json"
fi
run "ingest (bag read once)" dtp ingest "$session"
[ "$refine" -eq 1 ] && run "refine (pose graph + loop closures)" dtp refine "$session"
run "fuse (map + mesh, GPU)" dtp fuse "$session" --self-zone "$self_zone" "${mesh_flag[@]}"
if [ "$calibrate" -eq 1 ]; then
  run "ring calibration" dtp ringcal "$session"
  run "ingest (calibrated)" dtp ingest "$session"
  [ "$refine" -eq 1 ] && run "refine (calibrated)" dtp refine "$session"
  run "fuse (calibrated)" dtp fuse "$session" --self-zone "$self_zone" "${mesh_flag[@]}"
fi
if [ "$polish" -eq 1 ]; then
  run "polish (scans onto the surface)" dtp polish "$session"
  run "fuse again" dtp fuse "$session" --self-zone "$self_zone" "${mesh_flag[@]}"
fi
run "colour (GPU)" dtp colorize "$session"
[ "$texture" -eq 1 ] && run "texture (GPU)" dtp texture "$session"
run "quality report" "$py" "${here}/analysis/map_quality.py" "$session"
echo "total ${SECONDS} s -> ${session}/products"
