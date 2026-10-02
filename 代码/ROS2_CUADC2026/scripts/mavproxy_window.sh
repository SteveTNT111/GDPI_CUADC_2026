#!/usr/bin/env bash
# =============================================================================
# mavproxy_window.sh — 在独立窗口打开 MAVProxy（console + map）
#   连 SITL 的 SERIAL1 = tcp:127.0.0.1:5762（MAVLink2），与 MAVROS 的 5760 隔离。
#   已有一个 MAVProxy 在跑则不重复打开。
# 用法: bash mavproxy_window.sh
# =============================================================================
export DISPLAY="${DISPLAY:-:1}"
SC="$HOME/ROS2_CUADC2026/scripts"

if pgrep -f 'mavproxy.py --master=tcp:127.0.0.1:5762' >/dev/null 2>&1; then
  echo "[mavproxy_window] 已有 MAVProxy 在跑，跳过"
  exit 0
fi

echo "[mavproxy_window] 打开独立 MAVProxy 窗口（console + map）..."
exec gnome-terminal --title="MAVProxy (console+map) @5762" -- \
  bash -lc "bash '$SC/pane_6_mavproxy.sh'"
