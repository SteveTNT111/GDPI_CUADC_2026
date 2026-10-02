#!/usr/bin/env bash
# CUADC 2026 — ROS2 Sin验证仿真一键启动
# 用法: ./start_sim.sh [--headless]
set -o pipefail

GZ_WORLD="${GZ_WORLD:-/home/lab/GDPI_CUADC_2026/备赛文档/机架电装选型/cuadc_2026_field.world}"
ARDSRC="${ARDSRC:-/home/lab/ardupilot-4.7.1}"
ROS_WS="${ROS_WS:-/home/lab/ROS2_CUADC2026}"

export GZ_VERSION=harmonic
source /opt/ros/humble/setup.bash
export GZ_SIM_RESOURCE_PATH="$ROS_WS/models:/home/lab/SITL_Models/Gazebo:/home/lab/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
# ardupilot_gazebo 插件库位置（无 sudo，直接指向 build 目录）
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/lab/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

pkill -9 gzserver 2>/dev/null; pkill -9 gz sim 2>/dev/null; pkill -9 arducopter 2>/dev/null
sleep 2

# 1) Gazebo 服务器
echo "[sim] starting gz server..."
if [[ "${1:-}" == "--headless" ]]; then
  gz sim -s -r "$GZ_WORLD" > /tmp/opencode/gz_server.log 2>&1 &
else
  gz sim -r "$GZ_WORLD" > /tmp/opencode/gz_server.log 2>&1 &
fi
GZ_PID=$!
echo "[sim] gz pid=$GZ_PID"

# 等待 Gazebo transport 起来
for i in $(seq 1 30); do
  if gz topic -l 2>/dev/null | grep -q "/world"; then break; fi
  sleep 1
done
sleep 3

# 2) SITL（gazebo-iris, JSON 连 GZ 插件, TCP 5760 给 MAVROS）
echo "[sim] starting SITL..."
cd "$ARDSRC"
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --no-rebuild \
  > /tmp/opencode/sitl.log 2>&1 &
SITL_PID=$!
echo "[sim] sitl pid=$SITL_PID"

# 3) MAVROS
for i in $(seq 1 40); do
  if grep -qi "Mavlink started\|Waiting for GZ" /tmp/opencode/sitl.log 2>/dev/null; then break; fi
  sleep 1
done
echo "[sim] starting mavros..."
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 \
  > /tmp/opencode/mavros.log 2>&1 &
MAVROS_PID=$!
echo "[sim] mavros pid=$MAVROS_PID"

for i in $(seq 1 30); do
  if timeout 2 ros2 topic echo /mavros/state --once >/dev/null 2>&1; then
    echo "[sim] MAVROS 已连上 SITL; gz=$GZ_PID sitl=$SITL_PID mavros=$MAVROS_PID"
    echo "GZ_PID=$GZ_PID SITL_PID=$SITL_PID MAVROS_PID=$MAVROS_PID" > /tmp/opencode/sim.pids
    exit 0
  fi
  sleep 1
done
echo "[sim] MAVROS 连接超时，看日志: /tmp/opencode/{gz_server,sitl,mavros}.log"
exit 1