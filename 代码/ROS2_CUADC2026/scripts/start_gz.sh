#!/usr/bin/env bash
# 启动比赛场地 GZ server（Harmonic 适配版, headless）
set -o pipefail
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="/home/lab/ROS2_CUADC2026/models:/home/lab/SITL_Models/Gazebo:/home/lab/ardupilot_gazebo/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/lab/ardupilot_gazebo/build"
WORLD="${GZ_WORLD:-/home/lab/ROS2_CUADC2026/worlds/cuadc_2026_field.world}"
exec gz sim -s -r "$WORLD"