#!/usr/bin/env bash
# 仿真：一键启动（先自动清场，再 demo_up）
#   - 参考五面板 sim_up.sh 的做法，启动前先 sim_clean；
#   - --terminator 会关闭可能残留的旧五面板 Terminator（其 pane_2/pane_3 会和 demo_up 抢 SITL/MAVROS）；
#     本按钮自身终端不是以 --config-json 启动，不会被误杀。
SC="$HOME/ROS2_CUADC2026/scripts"
echo "=================================================="
echo " 仿真一键启动：先清场，再 demo_up"
echo "=================================================="
echo "[仿真启动] 自动清场：杀掉上一轮残留"
echo "  (gz / arducopter / mavros / camera / detector / rc_center / env_guard / demo_up / verify_* 等)"
bash "$SC/sim_clean.sh" --terminator
echo "[仿真启动] 清场完成，开始 demo_up（Gazebo+SITL+MAVROS+相机+守护）..."
exec bash "$SC/demo_up.sh"
