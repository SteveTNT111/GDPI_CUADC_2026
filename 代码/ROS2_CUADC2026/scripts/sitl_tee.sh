#!/usr/bin/env bash
# SITL 执行包装：全行时间戳 + 退出码捕获（错题#1 取证，2026-09-23）
#   rc 含义：0=正常退出  128+15=143 外部SIGTERM  137=外部SIGKILL  139=SIGSEGV崩溃
#   时间戳用于与 mavros.log（自带 epoch 时间戳）对齐，定位死亡前最后交互。
ARDSRC="/home/lab/ardupilot-4.7.1"
ROS_WS="/home/lab/ROS2_CUADC2026"
LOG="/tmp/opencode/sitl.log"
DEFAULTS="$ARDSRC/Tools/autotest/default_params/copter.parm,$ARDSRC/Tools/autotest/default_params/gazebo-iris.parm,$ROS_WS/scripts/sitl_streams.parm,$ROS_WS/scripts/sitl_surface_fix.parm,$ROS_WS/scripts/sitl_rc_center.parm"
cd "$ARDSRC/sitl/iris_cuadc" 2>/dev/null || cd /tmp
{
  "$ARDSRC/build/sitl/bin/arducopter" --model JSON --speedup 1 --instance 0 \
    --sysid 1 --defaults "$DEFAULTS" --sim-address 127.0.0.1
  echo "SITL_EXIT rc=$?"
} 2>&1 | while IFS= read -r line; do
  printf '%s %s\n' "$(date +%T)" "$line"
done >> "$LOG"
