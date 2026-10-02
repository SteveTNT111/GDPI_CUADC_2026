#!/usr/bin/env bash
# 运行 cuadc_control 包脚本： run_control.sh <script.py> [--ros-args -p k:=v ...]
# 输出同时显示在终端，并 tee 落盘 /tmp/opencode/webui_mission.log，
# 供 WebUI「实时任务日志」面板实时查看（免看混乱终端）。
source "$(dirname "$(readlink -f "$0")")/_env.sh"
LOG=/tmp/opencode/webui_mission.log
mkdir -p /tmp/opencode
echo "=================================================="
echo " 控制包:  ros2 run cuadc_control $*"
echo " 日志同步到: $LOG  （WebUI「实时任务日志」面板可看）"
echo "=================================================="
# 2>&1 | tee -a：终端照常显示，同时追加落盘；PIPESTATUS[0] 取任务本身返回码。
# PYTHONUNBUFFERED=1：经管道后 stdout 会块缓冲，加此保证日志实时刷出。
PYTHONUNBUFFERED=1 ros2 run cuadc_control "$@" 2>&1 | tee -a "$LOG"
rc=${PIPESTATUS[0]}
echo ""
echo "[启动器] 脚本已退出 rc=$rc。按回车关闭窗口。"
read -r _ || true
