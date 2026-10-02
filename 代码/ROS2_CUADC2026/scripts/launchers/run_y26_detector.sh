#!/usr/bin/env bash
# Y26 视觉包：识别脚本（后台，无画面）—— 适合飞行/省 CPU
source "$(dirname "$(readlink -f "$0")")/_env.sh"
SRC="${1:-}"
if [ -z "$SRC" ]; then
  SRC="$(sed -n 's/^[[:space:]]*camera_source:[[:space:]]*\([A-Za-z0-9_]*\).*/\1/p' "$HOME/ROS2_CUADC2026/config/auto_drop.yaml" 2>/dev/null | head -1)"
  [ -n "$SRC" ] || SRC=gazebo
fi
echo "[Y26 识别·后台] camera_source=$SRC  model=best_yolo26.onnx  show_window=false"
exec ros2 launch cuadc_visionY26 detector_node.launch.py \
  auto_start_mavros:=false auto_start_camera:=false camera_source:="$SRC" \
  show_window:=false
