#!/usr/bin/env bash
# preflight.sh — 秒级体检（只读，不杀不启）
# 错题本 PRE-FLIGHT 第 1/2/3/7 项的自动化：进程 + 端口 + 窗口 + 流实据。
# 判据铁律（#16c）：链路只认 stream_probe 流实据；/mavros/state 一律不看。
# 用法: bash preflight.sh        退出码 0=全绿（WARN 不算），1=有 FAIL
set -o pipefail

WS="$HOME/ROS2_CUADC2026"
SC="$WS/scripts"
export GZ_VERSION="${GZ_VERSION:-harmonic}"
export DISPLAY="${DISPLAY:-:0}"
source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
# -u 必须等 source 完再开：ROS setup 链引用未设变量，set -u 下会致命退出（#17）
set -u

SECS="${PREFLIGHT_SECS:-5}"
NPASS=0; NFAIL=0; NWARN=0

row() { # row PASS|FAIL|WARN "名字" "详情"
  printf '  [%-4s] %-44s %s\n' "$1" "$2" "${3:-}"
  case "$1" in
    PASS) NPASS=$((NPASS + 1)) ;;
    FAIL) NFAIL=$((NFAIL + 1)) ;;
    WARN) NWARN=$((NWARN + 1)) ;;
  esac
}

pids() { # 精确匹配 + 排除（#15 杀进程铁律的同一套排除式，只读不杀）
  # 容忍 /.../ 记号：-v re 收到的是纯字符串，斜杠会被当字面量导致永不匹配（#17）
  local re="$1"
  re="${re#/}"; re="${re%/}"
  ps -eo pid=,args= | awk -v re="$re" '!/bash -c/ && !/awk/ && $0 ~ re {print $1}'
}

echo "== 进程 =="
check_proc() { # check_proc "名字" awk正则
  local p
  p=$(pids "$2" | head -1)
  if [ -n "$p" ]; then row PASS "$1" "pid=$p"; else row FAIL "$1" "不在"; fi
}
check_proc "gz server"   '/gz sim -s/'
check_proc "gz GUI"      '/gz sim -g/'
check_proc "MAVROS"      '/ros2 launch mavros|mavros_node/'
check_proc "rc_center"   '/sitl_rc_center\.py/'
check_proc "camera_node" '/camera_node\.py|ros2 run cuadc_vision/'
check_proc "show_camera" '/show_camera\.py/'

# SITL：5760 LISTEN + 进程双证 + 龄（130s 危险窗口 #1/#16b）
sitl_pid=$(pids '/build/sitl/bin/arducopter/' | head -1)
if ss -ltn | grep -qE ':5760[[:space:]]'; then
  if [ -n "$sitl_pid" ]; then
    age=$(ps -o etimes= -p "$sitl_pid" 2>/dev/null | tr -d ' ')
    if [ -n "$age" ] && [ "$age" -ge 135 ]; then
      row PASS "SITL" "pid=$sitl_pid 龄=$((age / 60))m$((age % 60))s 已过130s窗口"
    else
      row WARN "SITL" "pid=$sitl_pid 龄=${age}s 危险窗口期(#16b)"
    fi
  else
    row WARN "SITL" "5760 通但抓不到 arducopter 进程(竞态?)"
  fi
else
  row FAIL "SITL" "5760 无 LISTEN"
fi

echo "== 窗口（#15：_NET_CLIENT_LIST + IsViewable）=="
win_check() { # win_check "名字子串" req|opt
  local pat="$1" need="${2:-req}" id nm ms found=""
  for id in $(xprop -root _NET_CLIENT_LIST 2>/dev/null | sed 's/.*# //' | tr ',' ' '); do
    nm=$(xprop -id "$id" _NET_WM_NAME 2>/dev/null | sed -n 's/.*= "\(.*\)"/\1/p')
    case "$nm" in
      *"$pat"*)
        ms=$(xwininfo -id "$id" 2>/dev/null | grep -c 'Map State:.*IsViewable')
        if [ "$ms" -ge 1 ]; then found="IsViewable id=$id"; else found="未映射 id=$id"; fi
        break
        ;;
    esac
  done
  if [ -n "$found" ]; then
    case "$found" in IsViewable*) row PASS "窗口 <$1>" "$found" ;;
      *) if [ "$need" = req ]; then row FAIL "窗口 <$1>" "$found"
         else row WARN "窗口 <$1>" "$found"; fi ;;
    esac
  else
    if [ "$need" = req ]; then row FAIL "窗口 <$1>" "不在受管列表"
    else row WARN "窗口 <$1>" "不在（进程未跑时属正常）"; fi
  fi
}
win_check "Gazebo Sim"
win_check "camera_node live" "opt"

echo "== 流实据（stream_probe ${SECS}s，#16c 唯一判据）=="
probe_out=$(python3 "$SC/stream_probe.py" --secs "$SECS" 2>/dev/null)
while IFS= read -r line; do
  case "$line" in
    OK*)  row PASS "流 ${line#OK  }" ;;
    MISS*) row FAIL "流 ${line#MISS }" "无消息（先跑 demo_up.sh）" ;;
  esac
done <<< "$probe_out"

echo
echo "== 结果: PASS=$NPASS FAIL=$NFAIL WARN=$NWARN =="
if [ "$NFAIL" -eq 0 ]; then
  echo "== PREFLIGHT 全绿 ✅ =="
  exit 0
else
  echo "== PREFLIGHT 有 FAIL ❌ → 跑 bash $SC/demo_up.sh 修复 =="
  exit 1
fi
