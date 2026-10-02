#!/usr/bin/env bash
# Y26 视觉包：相机脚本（真机 realsense；仿真改 source:=gazebo）
source "$(dirname "$(readlink -f "$0")")/_env.sh"
SRC="${1:-}"
if [ -z "$SRC" ]; then
  SRC="$(sed -n 's/^[[:space:]]*camera_source:[[:space:]]*\([A-Za-z0-9_]*\).*/\1/p' "$HOME/ROS2_CUADC2026/config/auto_drop.yaml" 2>/dev/null | head -1)"
  [ -n "$SRC" ] || SRC=gazebo
fi
echo "[Y26 相机] source=$SRC  show_rgb=true"
echo "  (仿真请改用: source:=gazebo)"
exec ros2 launch cuadc_visionY26 camera_node.launch.py source:="$SRC" show_rgb:=true
