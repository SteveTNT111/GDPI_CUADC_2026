#!/usr/bin/env bash
# Terminator 面板4：Gazebo 前端 GUI —— 等后端 /world 就绪后再附加
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$HOME/ROS2_CUADC2026/models:$HOME/SITL_Models/Gazebo:$HOME/ardupilot_gazebo/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build"
source /opt/ros/humble/setup.bash 2>/dev/null

echo "[面板4] 等待 Gazebo 后端 /world 就绪 ..."
for i in $(seq 1 120); do
  gz topic -l 2>/dev/null | grep -q '/world/' && { echo "[面板4] 后端已就绪 (${i}s)"; break; }
  sleep 1
done

echo "[面板4] 启动 Gazebo 前端 GUI ..."
gz sim -g "$HOME/ROS2_CUADC2026/worlds/cuadc_2026_field.world"
echo "[面板4] Gazebo 前端已退出（重开请在本面板重跑: bash scripts/pane_4_frontend.sh）"
exec bash
