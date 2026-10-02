#!/usr/bin/env bash
# env_guard.sh — 常驻自愈守护（#16：把"环境无声死"变成"无感自愈"）
#
# 机制: 15s/拍 查进程+端口（只读）；每 4 拍（≈60s）加查流实据；
#       缺啥 → 幂等的 `demo_up --fast` 补啥（击杀/重启逻辑全部集中在 demo_up，
#       本脚本自身不杀任何进程，天然免疫误杀 #15）。
# 自身免疫: setsid 常驻 + pidfile 单例；日志 → /tmp/opencode/env_guard.log。
# 频控: 触发间隔 ≥60s；SITL 每小时最多拉 6 次（防崩溃循环烧机，超限告警一次）。
set -o pipefail

WS="$HOME/ROS2_CUADC2026"
SC="$WS/scripts"
LOGD="/tmp/opencode"
PIDF="$LOGD/env_guard.pid"

# ---- 单实例 ----
if [ -f "$PIDF" ]; then
  old=$(cat "$PIDF" 2>/dev/null || true)
  if [ -n "$old" ] && [ "$old" != "$$" ] && kill -0 "$old" 2>/dev/null; then
    echo "$(date '+%F %T') env_guard 已在运行 pid=$old，本实例退出"
    exit 0
  fi
fi
echo $$ > "$PIDF"
# #65：trap 里不能用 `return 0`（顶层 return 无函数可退 → TERM/INT 被"吞掉"，
# env_guard 变成杀不死的僵尸；用函数+exit 才能正常收口）。
_cleanup_pidfile() { [ "$(cat "$PIDF" 2>/dev/null)" = "$$" ] && rm -f "$PIDF"; }
trap '_cleanup_pidfile; exit 0' EXIT INT TERM

source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
set -u  # 必须在 source 之后（#17：setup 链引用未设变量）
export GZ_VERSION=harmonic
export DISPLAY="${DISPLAY:-:0}"
export GZ_SIM_RESOURCE_PATH="$WS/models:/home/lab/SITL_Models/Gazebo:/home/lab/ardupilot_gazebo/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/lab/ardupilot_gazebo/build"

say() { echo "$(date '+%F %T') $*"; }
pids() {
  # 容忍 /.../ 记号：-v re 收到的是纯字符串，斜杠会被当字面量导致永不匹配（#17）
  local re="$1"
  re="${re#/}"; re="${re%/}"
  ps -eo pid=,args= | awk -v re="$re" '!/bash -c/ && !/awk/ && $0 ~ re {print $1}'
}
listen() { ss -ltn | grep -qE ":$1[[:space:]]"; }

LAST_UP=0
SITL_HR_START=0
SITL_HR_N=0
SITL_ALERT=0
TICK=0

trigger() { # trigger "缺件原因"
  local now
  now=$(date +%s)
  if [ $((now - LAST_UP)) -lt 60 ]; then
    say "  缺件[$1] 但距上次触发 <60s，跳过（rate-limit）"
    return 0
  fi
  LAST_UP=$now
  say "→ 触发 demo_up --fast：缺件=[$1]"
  setsid bash "$SC/demo_up.sh" --fast >> "$LOGD/demo_up.log" 2>&1 &
}

say "=== env_guard 启动 pid=$$（15s/拍，流探针 60s/拍）==="

while :; do
  TICK=$((TICK + 1))
  dead=()

  pids '/gz sim -s/' | grep -q . || dead+=("gz-server")
  pids '/gz sim -g/' | grep -q . || dead+=("gz-GUI")
  pids '/ros2 launch mavros|mavros_node/' | grep -q . || dead+=("mavros")
  pids '/sitl_rc_center\.py/' | grep -q . || dead+=("rc_center")
  pids '/camera_node\.py|ros2 run cuadc_vision/' | grep -q . || dead+=("camera")
  pids '/show_camera\.py/' | grep -q . || dead+=("show_camera")

  sitl_dead=0
  pids '/build/sitl/bin/arducopter/' | grep -q . || sitl_dead=1
  listen 5760 || sitl_dead=1
  if [ "$sitl_dead" = 1 ]; then
    now=$(date +%s)
    if [ $((now - SITL_HR_START)) -ge 3600 ]; then
      SITL_HR_START=$now; SITL_HR_N=0; SITL_ALERT=0
    fi
    SITL_HR_N=$((SITL_HR_N + 1))
    if [ "$SITL_HR_N" -gt 6 ]; then
      if [ "$SITL_ALERT" = 0 ]; then
        say "⚠ SITL 一小时内已拉 6 次仍死 — 暂停自动拉 SITL（其余照常；人工看 sitl.log 的 SITL_EXIT rc）"
        SITL_ALERT=1
      fi
    else
      dead+=("sitl")
    fi
  fi

  if [ "${#dead[@]}" -gt 0 ]; then
    say "第${TICK}拍 缺件: ${dead[*]}"
    trigger "${dead[*]}"
  fi

  # 流实据探针（进程全在才查；缺件时由 demo_up 自己的闸门负责）
  if [ "${#dead[@]}" -eq 0 ] && [ $((TICK % 4)) -eq 0 ]; then
    if ! python3 "$SC/stream_probe.py" --secs 5 \
        /mavros/local_position/pose \
        /vision/color/image_raw > /dev/null 2>&1; then
      say "第${TICK}拍 流实据中断（pose/vision）"
      trigger "流中断"
    fi
  fi

  sleep 15
done
