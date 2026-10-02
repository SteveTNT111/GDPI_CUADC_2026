#!/usr/bin/env bash
# webui_up.sh — 启动 CUADC 桌面 Web UI（本地服务 + 打开浏览器）
#
# 用法: bash scripts/webui_up.sh
#   WEBUI_PORT=8765（默认）
# 行为:
#   1. source ROS2 环境（/opt/ros/humble 再工作区 install，顺序不能反，#4）
#   2. 端口已占用 → 不重复起服务，直接开页面
#   3. setsid 起 server.py，日志 tee 到 /tmp/opencode/webui.log
#   4. xdg-open 打开 http://127.0.0.1:<port>/?token=<token>
#
# 安装桌面图标:
#   cp scripts/cuadc_webui.desktop ~/.local/share/applications/ && update-desktop-database
#   cp scripts/cuadc_webui.desktop ~/桌面/        # 需要桌面图标时
set -o pipefail

WS="$HOME/ROS2_CUADC2026"
PORT="${WEBUI_PORT:-8765}"
LOG="/tmp/opencode/webui.log"

source /opt/ros/humble/setup.bash 2>/dev/null
source "$WS/install/setup.bash" 2>/dev/null
# -u 必须等 source 完再开（#17）
set -u
mkdir -p /tmp/opencode

port_busy() {
  ss -ltn "sport = :$PORT" 2>/dev/null | grep -q ":$PORT"
}

if port_busy; then
  echo "[webui] 端口 $PORT 已在监听，不重复启动"
else
  echo "[webui] 启动服务 (port=$PORT)，日志: $LOG"
  # setsid：服务独立会话（#16a）；输出经 tee 落盘（#4：不要裸管道接 tail）
  setsid bash -c "python3 '$WS/webui/server.py' --host 0.0.0.0 --port '$PORT' 2>&1 | tee -a '$LOG'" \
    >/dev/null 2>&1 &
  for _ in $(seq 1 50); do
    port_busy && break
    sleep 0.2
  done
fi

TOKEN=""
for _ in $(seq 1 20); do
  if [ -f "$WS/config/webui.token" ]; then
    TOKEN="$(cat "$WS/config/webui.token")"
    [ -n "$TOKEN" ] && break
  fi
  sleep 0.2
done

URL="http://127.0.0.1:$PORT/"
[ -n "$TOKEN" ] && URL="${URL}?token=${TOKEN}"
LANIP="$(hostname -I 2>/dev/null | awk '{print $1}')"

echo "=================================================="
echo " CUADC Web UI"
echo "  本机   : $URL"
[ -n "$LANIP" ] && echo "  局域网 : http://$LANIP:$PORT/?token=$TOKEN"
echo "  口令   : ${TOKEN:-（未启用）}  (存于 config/webui.token)"
echo "  停止   : 按 pid 精确 kill server.py（勿用 pkill -f，#15）"
echo "=================================================="

xdg-open "$URL" >/dev/null 2>&1 || echo "[webui] xdg-open 失败，请手动打开上面的 URL"
