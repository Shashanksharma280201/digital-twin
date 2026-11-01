#!/usr/bin/env bash
# Re-run FAST-LIO on a recorded session with another parameter file and score the result.
# The recorded session is never modified: the variant lives in OUT with a symlink to the bag.
#
# Usage: scripts/analysis/replay_lio.sh SESSION FAST_LIO_YAML OUT [RATE]
#   OUT/odom/          the new /Odometry (rosbag2)
#   OUT/products/      traj_lidar.tum, map.laz, quality/quality.json
#   OUT/lio_cpu.txt    FAST-LIO CPU use while replaying (real-time check at RATE 1)
set -eo pipefail
session="$(realpath "$1")"; params="$(realpath "$2")"; out="$3"; rate="${4:-1.0}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=../jetson/env.sh
source "${here}/../jetson/env.sh"
export ROS_DOMAIN_ID="${REPLAY_DOMAIN:-77}"   # isolated from a live stack (and from parallel replays)

rm -rf "$out"; mkdir -p "$out"
out="$(realpath "$out")"
ln -s "${session}/bag" "${out}/bag"
cp "${session}/rig.yaml" "${out}/rig.yaml"

"$(ros2 pkg prefix fast_lio)/lib/fast_lio/fastlio_mapping" --ros-args -r __node:=fast_lio_replay \
  --params-file "$params" -p use_sim_time:=true >"${out}/lio.log" 2>&1 &
lio=$!
ros2 bag record -o "${out}/odom" /Odometry >/dev/null 2>&1 &
rec=$!
sleep 3
( while kill -0 "$lio" 2>/dev/null; do ps -o %cpu= -p "$lio"; sleep 2; done ) >"${out}/lio_cpu.txt" &
ros2 bag play "${session}/bag" --clock --rate "$rate" --topics /dt/lidar/points /dt/lidar/imu >/dev/null 2>&1
sleep 3
kill -INT "$rec" "$lio"
for _ in 1 2 3 4 5; do kill -0 "$lio" 2>/dev/null || break; sleep 1; done
kill -9 "$lio" 2>/dev/null || true
wait "$rec" 2>/dev/null || true

"${DT_VENV}/bin/python" - "$out" <<'EOF'
import sys
from dt_pipeline.session import Session
from dt_pipeline.traj import from_odometry
s = Session(sys.argv[1])
from_odometry(s.path / 'odom', s.rig).to_tum(s.traj_path)
EOF
dtp map "$out" --voxel 0.02 >/dev/null
"${DT_VENV}/bin/python" "${here}/map_quality.py" "$out" >/dev/null
"${DT_VENV}/bin/python" - "$out" <<'EOF'
import json, sys, numpy as np
out = sys.argv[1]
q = json.load(open(f'{out}/products/quality/quality.json'))
cpu = np.loadtxt(f'{out}/lio_cpu.txt', ndmin=1)
print(json.dumps({'variant': out, 'sharpness_mm': q['sharpness_mm'],
                  'return_to_start_m': q['capture']['return_to_start_m'],
                  'lio_cpu_pct_mean': round(float(cpu.mean()), 0) if len(cpu) else None}))
EOF
