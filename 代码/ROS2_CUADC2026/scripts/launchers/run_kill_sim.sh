#!/usr/bin/env bash
# 一键清场：杀仿真 + MAVROS（= 开仿真前 demo_up 按钮所做的清场 sim_clean --terminator）
# 会杀掉：gz sim(-s/-g/-r)、arducopter、mavros_node、camera_node、detector_node、
#         sitl_rc_center、show_camera、live_view、env_guard、demo_up、mavproxy、
#         以及 one_key_takeoff/verify_* 等残留，并关闭残留的旧五面板 Terminator。
# 本按钮自身终端不是以 --config-json 启动，不会被误杀。
SC="$HOME/ROS2_CUADC2026/scripts"
echo "=================================================="
echo " 一键清场：杀仿真 + MAVROS"
echo "=================================================="
bash "$SC/sim_clean.sh" --terminator
echo ""
echo "[清场] 完成。按回车关闭本窗口。"
read -r _ || true
