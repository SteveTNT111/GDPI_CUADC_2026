#!/usr/bin/env bash
# =============================================================================
# sim_up.sh — 一条命令：清场 + 打开 Terminator 五面板仿真终端
# 用法: bash sim_up.sh
# =============================================================================
SC="$HOME/ROS2_CUADC2026/scripts"
export DISPLAY="${DISPLAY:-:1}"

# 清仿真残留；如果已有 Terminator 窗口，也一并关掉（--terminator）
bash "$SC/sim_clean.sh" --terminator

# 删除 Terminator 自动保存的旧布局，避免覆盖 --config-json
rm -f "$HOME/.config/terminator/config"

# 独立窗口打开 MAVProxy（延迟几秒，等 Terminator/后端起来；内部会等 SITL 5762）
( sleep 10; bash "$SC/mavproxy_window.sh" > /tmp/opencode/mavproxy_window.log 2>&1 ) &

echo "[sim_up] 启动 Terminator 五面板 ..."
exec terminator --config-json="$SC/terminator_layout.json"
