#!/usr/bin/env bash
# Terminator 面板2：SITL（自愈：崩溃自动重拉，直到活过 ~130s 危险窗口）
source /opt/ros/humble/setup.bash 2>/dev/null
cd "$HOME/ROS2_CUADC2026"
rm -f /tmp/opencode/sitl_ready

echo "[面板2] 等待 Gazebo 后端 /world ..."
for i in $(seq 1 180); do
  gz topic -l 2>/dev/null | grep -q '/world/' && { echo "[面板2] 后端就绪"; break; }
  sleep 1
done

attempt=0
while true; do
  attempt=$((attempt + 1))
  echo "[面板2] 第 $attempt 次启动 SITL ..."
  if bash scripts/start_sitl.sh; then
    # 95s 看门狗已过，再盯 40s 确认活过 ~130s 危险窗口
    sleep 40
    if pgrep -x arducopter >/dev/null; then
      echo "[面板2] ✅ SITL 已稳定（活过 130s 窗口），PID=$(pgrep -x arducopter)"
      touch /tmp/opencode/sitl_ready
      break
    fi
    echo "[面板2] ⚠ SITL 在 95~130s 窗口内死亡，重试 ..."
  else
    echo "[面板2] ⚠ SITL 启动失败，重试 ..."
  fi
  pkill -9 -x arducopter 2>/dev/null
  sleep 3
done

# 保持面板存活；SITL 万一再死，提示手动重跑
while pgrep -x arducopter >/dev/null; do sleep 5; done
echo "[面板2] ⚠ SITL 已退出，请在本面板重跑: bash scripts/start_sitl.sh"
exec bash
