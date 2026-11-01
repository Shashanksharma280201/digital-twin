#!/usr/bin/env bash
# Stops every process of the digital-twin stack (launch files leave orphaned nodes when only the
# `ros2 launch` process is killed; a stale container with the same name then swallows component loads).
# Usage: stop_stack.sh
set -u
patterns=(
  "[c]apture.launch.py" "[s]ensors.launch.py" "[s]lam.launch.py"
  "[r]slidar_sdk_node" "[l]ivox_ros_driver2_node" "[d]t_lidar_container" "[c]omponent_container"
  "[f]astlio_mapping" "[g]lim_rosnode" "[d]t_monitor" "[d]t_blender_bridge"
  "[s]tatic_transform_publisher" "[r]obot_state_publisher" "[z]ed_wrapper" "[r]viz2/rviz2"
)
for sig in INT TERM KILL; do
  alive=0
  for p in "${patterns[@]}"; do
    if pgrep -f "$p" >/dev/null; then alive=1; pkill "-$sig" -f "$p"; fi
  done
  [ "$alive" -eq 0 ] && break
  sleep 3
done
left="$(pgrep -fa "fastlio_mapping|rslidar_sdk_node|component_container|dt_monitor|dt_blender_bridge" || true)"
if [ -n "$left" ]; then echo "still running:"; echo "$left"; exit 1; fi
echo "stack stopped"
