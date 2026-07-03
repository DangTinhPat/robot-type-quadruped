#!/usr/bin/env bash
# Kill leftover gz sim / ros2 launch processes from previous main_bot sim runs.
#
# Gazebo Sim keeps a server, a GUI, and a shell wrapper alive as separate
# processes; if a `ros2 launch main_bot ...` terminal is closed without
# Ctrl+C (or a background run is left dangling), these survive and the next
# run either fails to bind or cross-talks with the stale instance over
# gz-transport (which isn't scoped by ROS_DOMAIN_ID). This kills all of them.

set -u

PATTERNS=(
  "ros2 launch main_bot"
  "gz sim"
  "ros_gz_sim/create"
  "ros_gz_bridge/parameter_bridge"
  "robot_state_publisher"
)

echo "Looking for leftover main_bot sim processes..."
found=0
for pattern in "${PATTERNS[@]}"; do
  pids=$(pgrep -f "$pattern" || true)
  if [ -n "$pids" ]; then
    found=1
    echo "  [$pattern] -> $pids"
  fi
done

if [ "$found" -eq 0 ]; then
  echo "Nothing to kill."
  exit 0
fi

for pattern in "${PATTERNS[@]}"; do
  pkill -TERM -f "$pattern" 2>/dev/null
done

sleep 2

for pattern in "${PATTERNS[@]}"; do
  pkill -KILL -f "$pattern" 2>/dev/null
done

echo "Done. Remaining matches (should be empty):"
pgrep -af "gz sim|ros2 launch main_bot|robot_state_publisher|parameter_bridge" || echo "  (none)"
