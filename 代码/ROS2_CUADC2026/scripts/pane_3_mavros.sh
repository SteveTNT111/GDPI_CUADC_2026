#!/usr/bin/env bash
# Terminator 面板3：MAVROS（SITL 端口一开就立刻连——稳定公式要求开机 ≤25s 内连接）
source /opt/ros/humble/setup.bash 2>/dev/null
cd "$HOME/ROS2_CUADC2026"

echo "[面板3] 等待 SITL 5760 端口 ..."
for i in $(seq 1 240); do
  ss -tln 2>/dev/null | grep -q ':5760 ' && { echo "[面板3] 5760 就绪 (${i}s)"; break; }
  sleep 1
done

echo "[面板3] 立即启动 MAVROS（早连更稳）..."
pkill -9 -f '[m]avros_node' 2>/dev/null
sleep 1
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 &
MPID=$!

for i in $(seq 1 40); do
  st=$(timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null)
  echo "$st" | grep -q "connected: true" && { echo "[面板3] ✅ MAVROS 已连接 (${i}s)"; break; }
  sleep 1
done

echo "[面板3] 请求数据流 (request_streams.py) ..."
source "$HOME/ROS2_CUADC2026/install/setup.bash" 2>/dev/null
python3 "$HOME/ROS2_CUADC2026/scripts/request_streams.py"
echo "[面板3] 数据流请求完成。运行控制/视觉脚本前，请再等 GPS 定位 + EKF 收敛约 30s，否则解锁会 result=4。"
wait $MPID
exec bash
