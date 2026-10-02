#!/usr/bin/env bash
# Terminator 面板1：先自动清场，再启动 Gazebo 后端（headless server）
source /opt/ros/humble/setup.bash 2>/dev/null
SC="$HOME/ROS2_CUADC2026/scripts"

echo "[面板1] ===== 开仿真：自动清理残留 ====="
bash "$SC/sim_clean.sh"

echo "[面板1] ===== 启动 Gazebo 后端 ====="
cd "$HOME/ROS2_CUADC2026"
bash scripts/start_gz.sh
echo "[面板1] Gazebo 后端已退出"
exec bash
