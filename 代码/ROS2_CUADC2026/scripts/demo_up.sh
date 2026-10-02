#!/usr/bin/env bash
# demo_up.sh — 一键全栈拉起/自愈（幂等；#16 稳定公式的固化）
#
# 用法: bash demo_up.sh [--fast] [--reset] [--no-guard]
#   默认      缺啥补啥；新拉的 SITL 必须活过 130s 危险窗口才报 READY（冷启约 3~4 分钟）
#   --fast    跳过 130s 窗口等待（env_guard 会继续盯，间歇崩溃自动重拉，#16b）
#   --reset   全清场（含残留飞行节点）再重建 —— restart_sim.sh 即本脚本的 --reset 壳
#   --no-guard 不启动 env_guard 守护
#
# 铁律（错题本）:
#   #16a 一切长活进程必须 setsid，否则死在启动命令结束瞬间
#   #15  一切击杀按 pid 精确 + awk 排除（!/bash -c/ && !/awk/），禁 pkill -f
#   #16c 链路判定只用 stream_probe 流实据，/mavros/state 一律不看
#   #2   新 MAVROS 连接必须 request_streams 才有推流
#   #17  set -u 必须等 ROS source 完再开（setup 链引用未设变量会致命退出）
set -o pipefail

WS="$HOME/ROS2_CUADC2026"
SC="$WS/scripts"
LOGD="/tmp/opencode"
WORLD="$WS/worlds/cuadc_2026_field.world"
LOCK="$LOGD/demo_up.lock"

FAST=0; RESET=0; GUARD=1
for a in "$@"; do
  case "$a" in
    --fast) FAST=1 ;;
    --reset) RESET=1 ;;
    --no-guard) GUARD=0 ;;
    *) echo "未知参数: $a（可用: --fast --reset --no-guard）"; exit 64 ;;
  esac
done

ts() { date '+%T'; }
log() { echo "[$(ts)] $*"; }

# ---------- 单实例锁（防 demo_up × guard × restart_sim 并发互踩） ----------
if [ -d "$LOCK" ]; then
  owner=$(cat "$LOCK/pid" 2>/dev/null || true)
  if [ -n "$owner" ] && [ "$owner" != "$$" ] && kill -0 "$owner" 2>/dev/null; then
    log "已有 demo_up 在跑 (pid=$owner)，本实例退出"
    exit 0
  fi
  log "发现陈旧锁 (owner=${owner:-?} 已死)，接管"
  rm -rf "$LOCK"
fi
mkdir "$LOCK" 2>/dev/null || { echo "拿锁失败"; exit 1; }
echo $$ > "$LOCK/pid"
cleanup() {
  if [ "$(cat "$LOCK/pid" 2>/dev/null)" = "$$" ]; then rm -rf "$LOCK"; fi
  return 0
}
trap cleanup EXIT INT TERM

T0=$(date +%s)

source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
set -u
export GZ_VERSION=harmonic
export DISPLAY="${DISPLAY:-:0}"
export GZ_SIM_RESOURCE_PATH="$WS/models:/home/lab/SITL_Models/Gazebo:/home/lab/ardupilot_gazebo/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/lab/ardupilot_gazebo/build"

pids() {
  # 容忍 /.../ 记号：-v re 收到的是纯字符串，斜杠会被当字面量导致永不匹配（#17）
  local re="$1"
  re="${re#/}"; re="${re%/}"
  ps -eo pid=,args= | awk -v re="$re" '!/bash -c/ && !/awk/ && $0 ~ re {print $1}'
}
kill_pids() { # 精确杀（#15）
  local p
  for p in $(pids "$1"); do
    log "  kill $p ($(ps -o comm= -p "$p" 2>/dev/null || echo '?'))"
    kill -9 "$p" 2>/dev/null || true
  done
}
sitl_listen() { ss -ltn | grep -qE ':5760[[:space:]]'; }
probe() { # probe secs topic... → stdout 打 OK/MISS 行，rc=0 全绿
  python3 "$SC/stream_probe.py" --secs "$1" "${@:2}" 2>/dev/null
}
stream_gate() {
  probe 5 /mavros/local_position/pose \
    /mavros/global_position/compass_hdg \
    /mavros/global_position/rel_alt > /dev/null
}
vision_ok() { probe 4 /vision/color/image_raw > /dev/null; }
start_mavros() { # #16 公式：pid 精确杀旧 → setsid 新拉 → DDS 宽限
  kill_pids '/ros2 launch mavros|mavros_node/'
  sleep 1
  setsid ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760 \
    > "$LOGD/mavros.log" 2>&1 &
  log "  MAVROS 已拉起（等 6s DDS 宽限 #9）"
  sleep 6
}
req_streams() { # #2：新连接不请求就不推流
  python3 "$SC/request_streams.py" > "$LOGD/request_streams.log" 2>&1
  local rc=$?
  log "  request_streams rc=$rc（$(tail -1 "$LOGD/request_streams.log" | tr -d '\r')）"
}
winid() { # 按 _NET_WM_NAME 子串找窗口 id（#15 判据）
  local id nm
  for id in $(xprop -root _NET_CLIENT_LIST 2>/dev/null | sed 's/.*# //' | tr ',' ' '); do
    nm=$(xprop -id "$id" _NET_WM_NAME 2>/dev/null | sed -n 's/.*= "\(.*\)"/\1/p')
    case "$nm" in *"$1"*) echo "$id"; return 0 ;; esac
  done
  return 1
}

log "==== demo_up 开始 (args: $*，pid=$$) ===="

# ---------- [0] --reset 全清场（保留 env_guard 自身） ----------
if [ "$RESET" = 1 ]; then
  log "[0/11] --reset 全清场..."
  kill_pids '/gz sim -s|gz sim -g|gzserver|build/sitl/bin/arducopter|sitl_tee\.sh|start_sitl\.sh|ros2 launch mavros|mavros_node|sitl_rc_center\.py|camera_node\.py|ros2 run cuadc_vision|show_camera\.py|live_view\.py|one_key_takeoff|fly_to_dropzone|detector_node|verify_[a-z0-9]+\.py/'
  sleep 2
fi

# ---------- [1] GZ server ----------
FULL=0
[ "$RESET" = 1 ] && FULL=1
log "[1/11] GZ server..."
if pids '/gz sim -s/' | grep -q .; then
  log "  已在位"
else
  FULL=1
  setsid bash "$SC/start_gz.sh" > "$LOGD/gz_server.log" 2>&1 &
  ok=0
  for _ in $(seq 1 45); do
    if gz topic -l 2>/dev/null | grep -q '/world/'; then ok=1; break; fi
    sleep 1
  done
  if [ "$ok" = 1 ]; then log "  ✅ server 就绪"
  else log "  ❌ server 45s 未就绪 → $LOGD/gz_server.log"; exit 2; fi
fi

# ---------- [2] SITL ----------
SITL_FRESH=0
log "[2/11] SITL..."
if [ "$FULL" = 1 ]; then
  kill_pids '/build/sitl/bin/arducopter|sitl_tee\.sh|start_sitl\.sh'
  sleep 1
fi
if sitl_listen && pids '/build/sitl/bin/arducopter/' | grep -q .; then
  log "  已在位"
else
  kill_pids '/build/sitl/bin/arducopter|sitl_tee\.sh' 2>/dev/null || true
  sleep 1
  setsid bash "$SC/sitl_tee.sh" > /dev/null 2>&1 &
  ok=0
  for _ in $(seq 1 35); do
    if sitl_listen; then ok=1; break; fi
    sleep 1
  done
  if [ "$ok" = 1 ]; then SITL_FRESH=1; log "  ✅ 5760 已开（新实例，取证→ sitl.log）"
  else log "  ❌ 35s 未开 5760 → $LOGD/sitl.log"; exit 3; fi
fi

# ---------- [3] MAVROS + 流闸门（#16 稳定公式核心） ----------
log "[3/11] MAVROS + 流闸门..."
if [ "$SITL_FRESH" = 1 ]; then
  start_mavros   # 换 SITL 必换 MAVROS（旧 TCP 已断，state 会冻结假 connected #16c）
  req_streams    # #2 新连接必请求
fi
gate_ok=0
for attempt in 1 2 3; do
  if stream_gate; then gate_ok=1; break; fi
  log "  流闸门第 $attempt 次未过（pose/hdg/rel_alt）"
  if [ "$attempt" = 1 ]; then
    req_streams            # 可能只是没人请求过（#2），先补请求不动 MAVROS
  elif [ "$attempt" = 2 ]; then
    start_mavros           # 补请求无效 → 链路真断，按公式整套重启
    req_streams
  fi
  sleep 1
done
if [ "$gate_ok" != 1 ]; then
  log "❌ 流闸门 3 次未过，现场:"
  probe 5 /mavros/local_position/pose \
    /mavros/global_position/compass_hdg \
    /mavros/global_position/rel_alt || true
  exit 4
fi
log "  ✅ 流闸门全绿"

# ---------- [4] rc_center ----------
log "[4/11] rc_center（油门中位注入）..."
if pids '/sitl_rc_center\.py/' | grep -q .; then
  log "  已在位"
else
  setsid python3 "$SC/sitl_rc_center.py" --fixed 1500 \
    > "$LOGD/rc_center.log" 2>&1 &
  sleep 1
  if pids '/sitl_rc_center\.py/' | grep -q .; then log "  ✅ 已拉起"
  else log "  ❌ rc_center 拉起失败 → $LOGD/rc_center.log"; exit 5; fi
fi

# ---------- [5] camera_node（进程 + vision 流双证） ----------
log "[5/11] camera_node..."
if pids '/camera_node\.py|ros2 run cuadc_vision/' | grep -q . && vision_ok; then
  log "  已在位且有流"
else
  kill_pids '/camera_node\.py|ros2 run cuadc_vision/'
  sleep 1
  setsid ros2 run cuadc_vision camera_node.py --ros-args -p source:=gazebo \
    > "$LOGD/camera_node.log" 2>&1 &
  ok=0
  for _ in $(seq 1 20); do
    if vision_ok; then ok=1; break; fi
    sleep 1
  done
  if [ "$ok" = 1 ]; then log "  ✅ vision 流恢复"
  else log "  ❌ vision 无流 → $LOGD/camera_node.log"; exit 6; fi
fi

# ---------- [6] show_camera 窗口 ----------
log "[6/11] show_camera..."
if pids '/show_camera\.py/' | grep -q .; then
  log "  已在位"
else
  setsid env GDK_BACKEND=x11 python3 -u "$SC/show_camera.py" \
    > "$LOGD/show_camera.log" 2>&1 &
  sleep 1
  log "  已拉起（窗口在[9]统一等待/摆位）"
fi

# ---------- [7] live_view :8899（已移除：live_view.py 不存在，旁路页面非必需）----------

# ---------- [8] GZ GUI ----------
log "[8/11] GZ GUI..."
if pids '/gz sim -g/' | grep -q .; then
  log "  已在位"
elif [ -n "${DISPLAY:-}" ]; then
  setsid gz sim -g "$WORLD" > "$LOGD/gz_gui.log" 2>&1 &
  log "  拉起中（server 已活，#15 前置条件满足）"
else
  log "  ⚠ 无 DISPLAY，跳过"
fi

# ---------- [9] 窗口等待 + 摆位（用户确认过的标准布局） ----------
log "[9/11] 窗口等待/摆位..."
gid=""
for _ in $(seq 1 40); do
  gid=$(winid "Gazebo Sim" || true)
  [ -n "$gid" ] && break
  sleep 1
done
if [ -n "$gid" ]; then
  # Qt 刚 map 时几何是中间值（实测读到过 1200x1000），勿动态读取 ——
  # 直接用用户确认过的标准尺寸 1000x845
  /tmp/opencode/xplace "$gid" 70 164 1000 845 2>/dev/null \
    && log "  Gazebo Sim → (70,164) 1000x845" || log "  ⚠ Gazebo Sim 摆位失败(非致命)"
else
  log "  ⚠ Gazebo Sim 窗口 40s 未现（GUI 拉起失败? → $LOGD/gz_gui.log）"
fi
cid=""
for _ in $(seq 1 20); do
  cid=$(winid "camera_node live" || true)
  [ -n "$cid" ] && break
  sleep 1
done
if [ -n "$cid" ]; then
  # 标定（#17）：GTK 相机窗口 xwininfo 绝对坐标 = 请求 y + 37（标题栏），
  # 请求 64 → 内容落位 abs(640,101)，即验证记录里用户确认的布局
  /tmp/opencode/xplace "$cid" 640 64 1280 480 2>/dev/null \
    && log "  相机窗口 → abs(640,101) 1280x480" || log "  ⚠ 相机窗口摆位失败(非致命)"
else
  log "  ⚠ 相机窗口未现（show_camera 进程在吗？）"
fi

# ---------- [10] SITL 130s 危险窗口（#16b；--fast 跳过） ----------
need_window=0
if [ "$FAST" = 0 ]; then
  if [ "$SITL_FRESH" = 1 ]; then
    need_window=1
  else
    spid=$(pids '/build/sitl/bin/arducopter/' | head -1)
    sage=$(ps -o etimes= -p "$spid" 2>/dev/null | tr -d ' ')
    if [ -n "$sage" ] && [ "$sage" -lt 135 ]; then need_window=1; fi
  fi
fi
if [ "$need_window" = 1 ]; then
  log "[10/11] 等 SITL 活过 130s 危险窗口（#16b，夭折自动重拉×3）..."
  pulls=1
  next_print=0
  while :; do
    spid=$(pids '/build/sitl/bin/arducopter/' | head -1)
    if [ -z "$spid" ] && ! sitl_listen; then
      # 真死了（pid 无 + 端口无）
      if [ "$pulls" -ge 3 ]; then
        log "❌ SITL 窗口内 3 次拉起均夭折 — 死因取证: tail $LOGD/sitl.log（SITL_EXIT rc=）"
        exit 7
      fi
      pulls=$((pulls + 1))
      log "  SITL 死于窗口内 → 重拉（第 $pulls 次，#16b 复发证据已写 sitl.log）"
      sleep 1
      setsid bash "$SC/sitl_tee.sh" > /dev/null 2>&1 &
      next_print=0
      for _ in $(seq 1 35); do
        sitl_listen && break
        sleep 1
      done
      if ! sitl_listen; then continue; fi
      start_mavros
      req_streams
      continue
    fi
    if [ -n "$spid" ]; then
      age=$(ps -o etimes= -p "$spid" 2>/dev/null | tr -d ' ')
      if [ -n "$age" ] && [ "$age" -ge 135 ]; then
        log "  ✅ SITL 过 130s 危险窗口（龄 ${age}s）"
        break
      fi
      # 用"到点就打"而非取模（sleep5 相位与 15 互质性不保证，取模会相位锁死）
      if [ -n "$age" ] && [ "$age" -ge "$next_print" ]; then
        log "  窗口期 ${age}s / 135s ..."
        next_print=$((age + 15))
      fi
    fi
    sleep 5
  done
else
  log "[10/11] 无需等窗口（SITL 已老/--fast）"
fi

# ---------- [11] env_guard 守护 ----------
if [ "$GUARD" = 1 ]; then
  log "[11/11] env_guard 守护..."
  gpid=$(cat "$LOGD/env_guard.pid" 2>/dev/null || true)
  if [ -n "$gpid" ] && kill -0 "$gpid" 2>/dev/null; then
    log "  已在位 pid=$gpid"
  else
    setsid bash "$SC/env_guard.sh" >> "$LOGD/env_guard.log" 2>&1 &
    sleep 1
    log "  已启动 pid=$(cat "$LOGD/env_guard.pid" 2>/dev/null || echo '?')"
  fi
else
  log "[11/11] --no-guard，不启动守护"
fi

# ---------- 收官体检 ----------
log "==== 体检 ===="
bash "$SC/preflight.sh"
pf_rc=$?
elapsed=$(( $(date +%s) - T0 ))
if [ "$pf_rc" -eq 0 ]; then
  log "==== READY ✅ (耗时 ${elapsed}s) ===="
else
  log "==== 未就绪 ❌ (耗时 ${elapsed}s，见上表) ===="
fi
exit "$pf_rc"
