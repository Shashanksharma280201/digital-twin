#!/usr/bin/env bash
# End-to-end Blender sim demo (docs/design/2026-10-18-blender-sim-design.md):
#   Blender world (day1, day2) + DT-1 device + walk + camera renders
#   -> dt_sim driver-native bags -> real stack (bridge + FAST-LIO) on sim time, recorded
#   -> dtp traj/map/align/diff -> dt_sim eval vs ground truth -> Blender result renders.
#
# Usage: sim/run_demo.sh [--profile mid360|airy] [--run DIR] [--rate R] [--cam-rate HZ]
#                        [--no-camera] [--live] [--skip-blender] [--skip-bags]
#   --live   opens Blender (GUI) with the DT Live add-on connected while day 1 plays
#   --splat  also train the photoreal Gaussian-splat twin (needs DT_SPLAT_PY: python with torch+gsplat)
#   --no-hero  skip the Cycles product renders of the device (slow without a desktop GPU, e.g. on Jetson)
# Env: DT_WS (ROS workspace with dt_* + fast_lio built, default ros2_ws), DT_VENV (venv with
#      dt_pipeline + dt_sim installed; default: tools on PATH), BLENDER (default blender).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"

profile=mid360; run=""; rate=1.0; cam_rate=5; camera=1; live=0; skip_blender=0; skip_bags=0; splat=0; hero="--render"
while [ $# -gt 0 ]; do
  case "$1" in
    --profile) profile="$2"; shift ;;
    --run) run="$2"; shift ;;
    --rate) rate="$2"; shift ;;
    --cam-rate) cam_rate="$2"; shift ;;
    --no-camera) camera=0 ;;
    --live) live=1 ;;
    --splat) splat=1 ;;
    --no-hero) hero="" ;;
    --skip-blender) skip_blender=1 ;;
    --skip-bags) skip_bags=1 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done

run="${run:-${DT_DATA:-$HOME/dt_data}/sim/demo_${profile}}"
ws="${DT_WS:-$root/ros2_ws}"
bin() { if [ -n "${DT_VENV:-}" ]; then echo "$DT_VENV/bin/$1"; else command -v "$1"; fi; }
DT_SIM="$(bin dt_sim)"; DTP="$(bin dtp)"
BL="$here/blender/bl.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
log() { printf '\n\033[1;36m[demo] %s\033[0m\n' "$*"; }

ros() {  # run a command in a clean ROS environment (no venv on PATH)
  env -u VIRTUAL_ENV PATH="/usr/bin:/bin:/usr/local/bin" bash -c \
    "source /opt/ros/humble/setup.bash && source '$ws/install/setup.bash' && $*"
}

mkdir -p "$run"
log "run dir: $run  (profile $profile)"

if [ "$skip_blender" -eq 0 ]; then
  for v in day1 day2; do
    [ -f "$run/world/mesh_$v.npz" ] || { log "Blender: world $v"; "$BL" gen_world.py --out "$run" --variant "$v"; }
  done
  [ -f "$run/rig_sim.yaml" ] || { log "Blender: DT-1 device ($profile)"; "$BL" gen_device.py --out "$run" --profile "$profile" $hero; }
  [ -f "$run/trajectory.csv" ] || { log "Blender: operator walk"; "$BL" walk.py --out "$run" --variant day1; }
  if [ "$camera" -eq 1 ]; then
    log "Blender: ZED 2i frames at ${cam_rate} Hz (resumable)"
    "$BL" render_cams.py --out "$run" --variant day1 --rate "$cam_rate"
  fi
fi

if [ "$skip_bags" -eq 0 ]; then
  cam_flag=""; [ "$camera" -eq 1 ] || cam_flag="--no-camera"
  log "dt_sim: day1 bag"; "$DT_SIM" bag "$run" --profile "$profile" --variant day1 $cam_flag --out "$run/bag_day1" --overwrite
  log "dt_sim: day2 bag (LiDAR only)"; "$DT_SIM" bag "$run" --profile "$profile" --variant day2 --no-camera --out "$run/bag_day2" --overwrite
fi

play_day() {  # $1 = day1|day2 -> session dir $run/session_$1
  local day="$1" sess="$run/session_$1" pids=()
  rm -rf "$sess"; mkdir -p "$sess"; cp "$run/rig_sim.yaml" "$sess/rig.yaml"
  log "stack: replaying $day through bridge + FAST-LIO (sim time, rate $rate)"
  setsid bash -c "$(declare -f ros); ws='$ws'; ros ros2 launch dt_bringup capture.launch.py profile:=$profile \
      driver:=false camera:=false use_sim_time:=true monitor:=false" >"$sess/stack.log" 2>&1 & pids+=($!)
  setsid bash -c "$(declare -f ros); ws='$ws'; ros ros2 run dt_blender_bridge dt_blender_bridge --ros-args \
      -p use_sim_time:=true -p rig_path:=$run/rig_sim.yaml" >"$sess/bridge.log" 2>&1 & pids+=($!)
  sleep 4
  setsid bash -c "$(declare -f ros); ws='$ws'; ros ros2 bag record -s mcap -o '$sess/bag' \
      /dt/lidar/points /dt/lidar/imu /Odometry /cloud_registered \
      /zed/zed_node/rgb/color/rect/image/compressed /zed/zed_node/rgb/color/rect/camera_info \
      /sim/ground_truth/odometry" >"$sess/record.log" 2>&1 & pids+=($!)
  sleep 2
  if [ "$live" -eq 1 ] && [ "$day" = day1 ]; then
    log "opening Blender with DT Live connected (close it any time)"
    env -u VIRTUAL_ENV "${BLENDER:-blender}" --factory-startup "$run/world/walk_preview_day1.blend" \
      -P "$here/blender/addon/run_live.py" -- --connect 127.0.0.1:5577 >"$sess/blender_live.log" 2>&1 &
    sleep 8
  fi
  ros ros2 bag play "$run/bag_$day" --clock -r "$rate" >"$sess/play.log" 2>&1
  sleep 3
  for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done
  sleep 5
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done
}

play_day day1
play_day day2

log "dtp: trajectories and maps"
for d in day1 day2; do
  "$DTP" traj "$run/session_$d" --source fastlio
  "$DTP" map "$run/session_$d" --voxel 0.05
done
log "dtp: align day2 -> day1 and change detection"
"$DTP" align "$run/session_day1" "$run/session_day2"
"$DTP" diff "$run/session_day1" "$run/session_day2"

log "dt_sim: evaluation against ground truth"
"$DT_SIM" eval "$run" --session "$run/session_day2" --variant day2
cp "$run/products/map_eval.npz" "$run/products/map_eval_day2.npz"
"$DT_SIM" eval "$run" --session "$run/session_day1" --variant day1 --diff "$run/session_day2/products/diff"
cp "$run/products/map_eval.npz" "$run/products/map_eval_day1.npz"

if [ "$camera" -eq 1 ]; then
  log "fusion: colour the day1 LiDAR map from the ZED frames (occlusion-aware)"
  "$DTP" colorize "$run/session_day1"
  "$(bin python)" "$here/view_twin.py" "$run" --color rgb --export "$run/twin_rgb_day1.npz"
  "$BL" show_result.py --out "$run" --variant day1 --map "$run/twin_rgb_day1.npz" \
    --est-traj "$run/products/traj_est_world_day1.tum" --render
  for f in result_day1.png result_day1_inside.png; do mv "$run/$f" "$run/${f/result_/result_rgb_}"; done
fi

if [ "$splat" -eq 1 ] && [ "$camera" -eq 1 ]; then
  sp="${DT_SPLAT_PY:-$HOME/dt_data/venvs/splat/bin/python}"
  prod="$run/session_day1/products"
  log "splat: COLMAP export (LiDAR poses + images + coloured LiDAR points)"
  "$DTP" colmap "$run/session_day1" --min-translation 0.15 --min-rotation 5
  log "splat: training Gaussian splat twin with LiDAR depth supervision (~15 min on an RTX 4060)"
  "$sp" "$root/splat/train_splat.py" --data "$prod/colmap" --out "$prod/splat" --iters 20000
  "$sp" "$root/splat/render_flythrough.py" --splat "$prod/splat/point_cloud.ply" --data "$prod/colmap" \
    --out "$prod/splat/flythrough.mp4" --frames 600 --lift 2.5 --pitch -20
fi

log "Blender: result renders"
"$BL" show_result.py --out "$run" --variant day1 --map "$run/products/map_eval_day1.npz" \
  --est-traj "$run/products/traj_est_world_day1.tum" --render
"$BL" show_result.py --out "$run" --variant day2 --diff "$run/products/diff_day1" \
  --est-traj "$run/products/traj_est_world_day2.tum" --render

log "done"
echo "  device:   $run/device/hero_${profile}.png"
echo "  twin:     $run/result_day1.png (map coloured by error vs ground truth)"
echo "  fused:    $run/result_rgb_day1.png (LiDAR geometry + ZED camera colours)"
[ "$splat" -eq 1 ] && echo "  splat:    $run/session_day1/products/splat/{point_cloud.ply,flythrough.mp4,metrics.json}"
echo "  change:   $run/result_day2.png (green added / red removed, day1 -> day2)"
echo "  metrics:  $run/eval_day1.json  $run/eval_day2.json  $run/session_day2/products/diff/summary.json"
