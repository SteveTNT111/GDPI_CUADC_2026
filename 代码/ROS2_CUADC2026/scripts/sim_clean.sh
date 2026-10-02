#!/usr/bin/env bash
# =============================================================================
# sim_clean.sh — 开仿真前彻底清理残留（防止"幽灵起飞"/双实例互踩）
# -----------------------------------------------------------------------------
# 用法:
#   bash sim_clean.sh                # 只清仿真相关进程（Terminator 面板1 调用）
#   bash sim_clean.sh --terminator   # 额外关闭已存在的 Terminator 窗口（独立启动时）
#
# 说明:
#   - 用 [x] 括号写法避免 pkill 匹配到本脚本自身；
#   - 不会杀掉当前正在运行本脚本的 Terminator 窗口（除非加 --terminator）。
# =============================================================================
set +e

EXTRA=0
[ "${1:-}" = "--terminator" ] && EXTRA=1

# 需要清理的残留进程（正则，用 [x] 自保护）
PATTERNS=(
  '[e]nv_guard'
  '[d]emo_up'
  '[g]z sim server'
  '[g]z sim -s'
  '[g]z sim -r'
  '[g]z sim -g'
  '[a]rducopter'
  '[m]avproxy'
  '[s]im_vehicle'
  '[s]itl_rc_center'
  '[h]old_rc_center'
  '[c]amera_node.py'
  '[d]etector_node.py'
  '[g]eopose_node.py'
  '[c]ompetition_main'
  '[s]ingle_bucket_aim_test'
  '[s]ervo_test'
  '[s]how_camera'
  '[l]ive_view'
  '[s]tream_probe'
  '[o]ne_key_takeoff'
  '[f]ly_to_dropzone'
  '[v]erify_'
  '[m]av param get'
)

echo "[sim_clean] 开始清理残留仿真进程 ..."
for round in 1 2; do
  killed=0
  for p in "${PATTERNS[@]}"; do
    if pgrep -f "$p" >/dev/null 2>&1; then
      pkill -9 -f "$p" 2>/dev/null
      [ "$round" = 1 ] && echo "  kill: $p"
      killed=1
    fi
  done
  sleep 1
  [ "$killed" = 0 ] && break
done

# ---- MAVROS：只杀"仿真" MAVROS（fcu_url=tcp://127.0.0.1:5760）；真机 MAVROS 跳过 ----
# 背景（2026-10-02 外场实测）：原 PATTERNS 里的 'ros2 launch mavros'/'mavros_node'
# 会把操作手用【真机】按钮拉起的 MAVROS 一起杀掉 → 全自动任务报"飞控未连接/连接超时"。
echo "[sim_clean] 处理 MAVROS（区分仿真/真机）..."
for p in $(pgrep -f 'launch [m]avros' 2>/dev/null); do
  cmd=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null)
  if echo "$cmd" | grep -q 'tcp://127.0.0.1:5760'; then
    pgid=$(ps -o pgid= -p "$p" 2>/dev/null | tr -d ' ')
    [ -n "$pgid" ] && kill -9 "-$pgid" 2>/dev/null
    echo "  kill: 仿真 MAVROS launch (pid=$p pgid=${pgid:-?})"
  else
    echo "  跳过真机 MAVROS launch: pid=$p（fcu_url 非仿真，避免误杀）"
  fi
done
for p in $(pgrep -f '[m]avros_node' 2>/dev/null); do
  cmd=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null)
  ppid=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
  pcmd=""
  [ -n "$ppid" ] && pcmd=$(tr '\0' ' ' < "/proc/$ppid/cmdline" 2>/dev/null)
  if echo "$cmd $pcmd" | grep -q 'tcp://127.0.0.1:5760'; then
    kill -9 "$p" 2>/dev/null
    echo "  kill: 仿真 mavros_node (pid=$p)"
  else
    echo "  跳过真机 mavros_node: pid=$p（fcu_url 非仿真，避免误杀）"
  fi
done

if [ "$EXTRA" = 1 ]; then
  echo "[sim_clean] 关闭已有 Terminator 窗口 ..."
  pkill -9 -f '[t]erminator --config-json' 2>/dev/null
  sleep 1
fi

rm -f /tmp/opencode/sitl_ready 2>/dev/null
rm -rf /tmp/opencode/demo_up.lock 2>/dev/null

# 等端口释放
for i in $(seq 1 10); do
  ss -tln 2>/dev/null | grep -q ':5760 ' || break
  sleep 1
done
echo "[sim_clean] 清理完成（5760 已释放）"
