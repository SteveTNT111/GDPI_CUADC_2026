#!/usr/bin/env bash
# =============================================================================
# mavproxy_console.sh — 给正在运行的 SITL 挂一个 MAVProxy 控制台 + 地图
#   5760 被 MAVROS 占用；这里默认用 SERIAL1（5762，MAVLink2）。
# 用法:
#   bash mavproxy_console.sh                 # 连 tcp:127.0.0.1:5762
#   bash mavproxy_console.sh udp:127.0.0.1:14550
#
# MAVProxy 控制台常用命令:
#   status            查看状态
#   mode GUIDED       切模式
#   arm throttle      解锁
#   disarm            上锁
#   watch ATTITUDE    实时看某消息
#   module load map   打开地图
# =============================================================================
MASTER="${1:-tcp:127.0.0.1:5762}"
echo "[mavproxy] 连接 $MASTER （console + map）"
exec mavproxy.py --master="$MASTER" --console --map
