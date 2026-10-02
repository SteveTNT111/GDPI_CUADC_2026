#!/usr/bin/env bash
# Y26 视觉包：识别脚本（带 UI 画面）—— 弹出标注画面窗口
#   - 锁定 X11（GDK_BACKEND=x11）并自动挑选可用的 X display，避免黑屏/后端错误
#   - 画面来自相机话题；请先开【视觉 Y26：相机】按钮（仿真用 source:=gazebo）
source "$(dirname "$(readlink -f "$0")")/_env.sh"

pick_display() {
  local d n
  for d in "${DISPLAY:-}" :1 :0; do
    [ -n "$d" ] || continue
    n="${d#:}"; n="${n%%.*}"
    if [ -S "/tmp/.X11-unix/X$n" ]; then echo "$d"; return 0; fi
  done
  echo "${DISPLAY:-:0}"
}
export DISPLAY="$(pick_display)"
export GDK_BACKEND=x11
SRC="${1:-}"
if [ -z "$SRC" ]; then
  SRC="$(sed -n 's/^[[:space:]]*camera_source:[[:space:]]*\([A-Za-z0-9_]*\).*/\1/p' "$HOME/ROS2_CUADC2026/config/auto_drop.yaml" 2>/dev/null | head -1)"
  [ -n "$SRC" ] || SRC=gazebo
fi
echo "[Y26 识别·UI] DISPLAY=$DISPLAY GDK_BACKEND=x11 window='YOLO26 Detection' camera_source=$SRC"
echo "  提示：需要相机在发图才会出画面；仿真请先开【视觉 Y26：相机】(source:=gazebo)"
exec ros2 launch cuadc_visionY26 detector_node.launch.py \
  auto_start_mavros:=false auto_start_camera:=false camera_source:="$SRC" \
  show_window:=true window_name:="YOLO26 Detection"
