#!/usr/bin/env bash
# 手动单拉 SITL（restart_sim.sh 之外的兜底），含 95s 看门狗
# 用法: bash start_sitl.sh
set -o pipefail
ARDSRC="/home/lab/ardupilot-4.7.1"
ROS_WS="/home/lab/ROS2_CUADC2026"
SITL_LOG="/tmp/opencode/sitl.log"

# 若已有实例在监听 5760 就不重复启动
if ss -tln | grep -q ':5760 '; then
  echo "SITL 已在监听 5760，跳过启动"
  exit 0
fi

pkill -9 -x arducopter 2>/dev/null
sleep 1
: > "$SITL_LOG"
cd "$ARDSRC/sitl/iris_cuadc" 2>/dev/null || cd /tmp
# setsid 必须有：harness 会在启动命令结束时清理其进程组，nohup 只挡 SIGHUP
# 挡不住进程组回收 → SITL 会在"启动命令结束瞬间"被 SIGTERM（日志留 Exitting）
# 掉线。setsid 换独立会话后免疫（错题#1 的真正根因，2026-09-23 定案）。
setsid nohup "$ARDSRC/build/sitl/bin/arducopter" --model JSON --speedup 1 --instance 0 \
  --sysid 1 \
  --defaults "$ARDSRC/Tools/autotest/default_params/copter.parm,$ARDSRC/Tools/autotest/default_params/gazebo-iris.parm,$ROS_WS/scripts/sitl_streams.parm,$ROS_WS/scripts/sitl_surface_fix.parm,$ROS_WS/scripts/sitl_rc_center.parm" \
  --sim-address 127.0.0.1 > "$SITL_LOG" 2>&1 &
SITL_PID=$!
echo "sitl pid=$SITL_PID"

BOOTED=0
for i in $(seq 1 40); do
  kill -0 "$SITL_PID" 2>/dev/null || { echo "❌ SITL 启动阶段退出"; tail -5 "$SITL_LOG"; exit 1; }
  grep -q "SERIAL0 on TCP port 5760" "$SITL_LOG" 2>/dev/null && { BOOTED=1; break; }
  sleep 1
done
[ "$BOOTED" = 1 ] && echo "✅ SITL 已打开 5760" || { echo "❌ SITL 未打开 5760"; tail -5 "$SITL_LOG"; exit 1; }

# 95s 看门狗（错题本 #1：首帧 JSON 负载崩溃窗口）
for i in $(seq 1 19); do
  sleep 5
  if ! kill -0 "$SITL_PID" 2>/dev/null; then
    echo "❌ SITL 看门狗检测到崩溃 (t=$((i*5))s)"
    tail -8 "$SITL_LOG"
    exit 2
  fi
done
echo "✅ SITL 看门狗95s 通过，pid=$SITL_PID"
