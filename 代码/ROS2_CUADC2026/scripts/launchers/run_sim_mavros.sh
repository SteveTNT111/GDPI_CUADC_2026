#!/usr/bin/env bash
# 仿真：连接 SITL 的 MAVROS（tcp://127.0.0.1:5760），并请求数据流
source /opt/ros/humble/setup.bash
source "$HOME/ROS2_CUADC2026/install/setup.bash"
cd "$HOME/ROS2_CUADC2026"
echo "=================================================="
echo " 仿真 MAVROS： tcp://127.0.0.1:5760"
echo "=================================================="
# 关闭已有 MAVROS，避免抢链路
for p in $(pgrep -f mavros_node); do kill "$p" 2>/dev/null; done
sleep 2
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 &
MPID=$!
for i in $(seq 1 30); do
  if timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null | grep -q "connected: true"; then
    echo "[仿真 MAVROS] 已连接 (${i}s)"; break
  fi
  sleep 1
done
echo "[仿真 MAVROS] 请求数据流 request_streams.py ..."
python3 "$HOME/ROS2_CUADC2026/scripts/request_streams.py" || true
echo "[仿真 MAVROS] 就绪（/mavros/state connected:true）。Ctrl+C 退出。"
wait $MPID
