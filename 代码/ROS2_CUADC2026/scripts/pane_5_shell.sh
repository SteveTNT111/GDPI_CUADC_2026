#!/usr/bin/env bash
# Terminator 面板5：脚本终端（预装好 ROS2 与工作空间环境，直接 ros2 run）
source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/ROS2_CUADC2026/install/setup.bash" 2>/dev/null
cd "$HOME/ROS2_CUADC2026"
cat <<'EOF'
[面板5] 脚本终端就绪。

  操作者工具（先解锁再跑脚本）：
    bash scripts/arm_guided.sh          # 切 GUIDED + 解锁
    bash scripts/mavproxy_console.sh    # 挂 MAVProxy 控制台+地图（SERIAL1:5762）

  控制：
    ros2 run cuadc_control one_key_takeoff.py
    bash scripts/arm_guided.sh && ros2 run cuadc_control one_key_takeoff_forward_land.py
    ros2 run cuadc_control one_key_takeoff_wgs84_forward_rtl.py   # 终端输 YES

  视觉：
    ros2 run cuadc_vision camera_node.py --ros-args -p source:=gazebo
    ros2 run cuadc_vision detector_node.py

  提示：解锁报 result=4 → 先等 GPS fix_type:6 + EKF ~30s，并确认已跑过 request_streams。
EOF
exec bash
