#!/usr/bin/env bash
# Pillar capture in the Blender sim: one detailed site column, scanned by hand in three close laps,
# then the same processing as a real capture. Prints how long recording and processing take.
#
#   RECORD (what a real capture replaces)
#     world    Blender: column + courtyard            (sim only)
#     device   Blender: DT-1 with the Airy profile    (sim only)
#     walk     Blender: 3 laps around the column, facing it (1.25 m / 1.2 m, 0.85 / 1.0, 1.60 / 1.4)
#     render   Blender EEVEE: ZED left frames          (sim only; a real camera does this live)
#     sensors  dt_sim: Airy LiDAR + IMU + camera -> driver-native bag (sim only)
#     replay   real stack on sim time: bridge + FAST-LIO, recorded like dt_session  (= the live walk)
#   PROCESS
#     scripts/process_session.sh (ingest, refine, fuse, colour, report) + dt_sim eval vs ground truth
#
# Usage: sim/run_pillar.sh [--run DIR] [--speed 0.3] [--cam-rate 10] [--hd] [--rate 1.0]
#   --hd        render 1920x1080 instead of 1280x720 (sharper colour, ~2x render time)
#   --rate R    replay speed through the stack (1.0 = real time, as on the device)
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "$here/.." && pwd)"

run=""; speed=0.3; cam_rate=10; hd=""; rate=1.0
while [ $# -gt 0 ]; do
  case "$1" in
    --run) run="$2"; shift ;;
    --speed) speed="$2"; shift ;;
    --cam-rate) cam_rate="$2"; shift ;;
    --hd) hd="--hd" ;;
    --rate) rate="$2"; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done
run="${run:-${DT_DATA:-$HOME/dt_data}/sim/pillar}"
ws="${DT_WS:-$root/ros2_ws}"
venv="${DT_VENV:-${DT_DATA:-$HOME/dt_data}/venvs/dt}"
DT_SIM="$venv/bin/dt_sim"
BL="$here/blender/bl.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
mkdir -p "$run"
timing="$run/timing.txt"; : >"$timing"

progress() {  # extra live info per stage
  case "$1" in
    render) local total; total=$(awk -v r="$cam_rate" 'END {printf "%d", ($1 > 0 ? $1 : 0) * r + 1}' FS=, "$run/trajectory.csv")
            printf '%s/%s frames' "$(ls "$run/camera/left" 2>/dev/null | wc -l)" "$total" ;;
    replay) printf 'real-time replay of the %s s walk' "$(tail -1 "$run/trajectory.csv" | cut -d, -f1 | cut -d. -f1)" ;;
  esac
}
stage() {  # stage NAME CMD... ; logs to $run/logs/NAME.log, shows a live timer, records the seconds taken
  local name="$1" t0=$SECONDS pid; shift
  mkdir -p "$run/logs"
  "$@" >"$run/logs/$name.log" 2>&1 &
  pid=$!
  while kill -0 "$pid" 2>/dev/null; do
    printf '\r\033[1;36m[pillar] %-8s\033[0m %4d s  %s   ' "$name" $((SECONDS - t0)) "$(progress "$name")"
    sleep 1
  done
  if wait "$pid"; then
    printf '\r\033[1;36m[pillar] %-8s\033[0m %4d s  %-40s\n' "$name" $((SECONDS - t0)) "$(progress "$name")"
    echo "$name $((SECONDS - t0))" >>"$timing"
  else
    printf '\n[pillar] %s FAILED (see %s)\n' "$name" "$run/logs/$name.log"; tail -5 "$run/logs/$name.log"; exit 1
  fi
}
trap 'echo; echo "[pillar] stopped"; kill 0' INT
ros() {  # a clean ROS environment (no venv on PATH)
  env -u VIRTUAL_ENV PATH="/usr/bin:/bin:/usr/local/bin" bash -c \
    "source /opt/ros/humble/setup.bash && source '$ws/install/setup.bash' && $*"
}

replay() {  # driver-native bag -> real stack on sim time -> session/bag (what dt_session records)
  local sess="$run/session" pids=()
  rm -rf "$sess"; mkdir -p "$sess"; cp "$run/rig_sim.yaml" "$sess/rig.yaml"
  setsid bash -c "$(declare -f ros); ws='$ws'; ros ros2 launch dt_bringup capture.launch.py profile:=airy \
      driver:=false camera:=false use_sim_time:=true monitor:=false" >"$sess/stack.log" 2>&1 & pids+=($!)
  sleep 4
  setsid bash -c "$(declare -f ros); ws='$ws'; ros ros2 bag record -s mcap -o '$sess/bag' \
      /dt/lidar/points /dt/lidar/imu /Odometry \
      /zed/zed_node/rgb/color/rect/image/compressed /zed/zed_node/rgb/color/rect/camera_info \
      /sim/ground_truth/odometry" >"$sess/record.log" 2>&1 & pids+=($!)
  sleep 2
  ros ros2 bag play "$run/bag_pillar" --clock -r "$rate" >"$sess/play.log" 2>&1
  sleep 3
  for p in "${pids[@]}"; do kill -INT -- "-$p" 2>/dev/null || true; done
  sleep 5
  for p in "${pids[@]}"; do kill -KILL -- "-$p" 2>/dev/null || true; done
}

echo "[pillar] run dir $run"
[ -f "$run/world/mesh_pillar.npz" ] || stage world "$BL" gen_world.py --out "$run" --variant pillar
[ -f "$run/rig_sim.yaml" ] || stage device "$BL" gen_device.py --out "$run" --profile airy
stage walk "$BL" walk.py --out "$run" --variant pillar --speed "$speed" --no-preview
rm -rf "$run/camera"
stage render "$BL" render_cams.py --out "$run" --variant pillar --rate "$cam_rate" $hd
stage sensors "$DT_SIM" bag "$run" --profile airy --variant pillar --out "$run/bag_pillar" --overwrite
stage replay replay
stage process bash "$root/scripts/process_session.sh" "$run/session"
stage eval "$DT_SIM" eval "$run" --session "$run/session" --variant pillar

walk_s=$(grep -o "[0-9]*s, start-end" "$run/logs/walk.log" | grep -o "^[0-9]*")
echo
echo "[pillar] capture length (the walk itself): ${walk_s} s"
awk '{t[$1]=$2} END {
  rec = t["world"] + t["device"] + t["walk"] + t["render"] + t["sensors"] + t["replay"];
  printf "[pillar] record (sim):  %4d s   world %d, device %d, walk %d, render %d, sensors %d, replay %d\n",
         rec, t["world"], t["device"], t["walk"], t["render"], t["sensors"], t["replay"];
  printf "[pillar] process:       %4d s   pipeline %d, accuracy check %d\n", t["process"] + t["eval"], t["process"], t["eval"] }' "$timing"
echo "[pillar] step times:  $(grep -E '^   [0-9]+ s|^==' "$run/logs/process.log" | paste -sd' ' | sed 's/== //g')"
echo "[pillar] accuracy:    $run/eval_pillar.json"
echo "[pillar] view:        $venv/bin/python scripts/view_map.py $run/session --mesh"
