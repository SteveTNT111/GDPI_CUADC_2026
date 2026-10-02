#!/usr/bin/env bash
# =============================================================================
# arm_guided.sh — 一键：切 GUIDED + 解锁（操作者用）
#   one_key_takeoff_forward_land.py 自己不解锁，跑之前先执行本脚本。
# 用法: bash arm_guided.sh
# =============================================================================
source /opt/ros/humble/setup.bash 2>/dev/null

echo "[arm] 切 GUIDED ..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
echo "[arm] 解锁 ..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
echo "[arm] 当前状态："
ros2 topic echo /mavros/state --once 2>/dev/null | grep -E "armed|mode"
