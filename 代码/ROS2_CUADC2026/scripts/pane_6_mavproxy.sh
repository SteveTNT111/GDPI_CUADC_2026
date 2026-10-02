#!/usr/bin/env bash
# Terminator 面板6：MAVProxy 控制台 + 地图（独立面板）
#   连接 SITL 的 SERIAL1 = tcp:127.0.0.1:5762（MAVLink2）
#   与 MAVROS 用的 SERIAL0(5760) 分开，互不干扰。
source /opt/ros/humble/setup.bash 2>/dev/null

echo "[面板6] 等待 SITL 5762 (SERIAL1) ..."
for i in $(seq 1 240); do
  ss -tln 2>/dev/null | grep -q ':5762 ' && { echo "[面板6] 5762 就绪 (${i}s)"; break; }
  sleep 1
done

echo "[面板6] 启动 MAVProxy（console + map）..."
mavproxy.py --master=tcp:127.0.0.1:5762 --console --map
echo "[面板6] MAVProxy 已退出（重开: bash scripts/pane_6_mavproxy.sh）"
exec bash
