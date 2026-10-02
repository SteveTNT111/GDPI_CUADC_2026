#!/usr/bin/env bash
# =============================================================================
# mavros_conn.sh — 一键切换 MAVROS 的连接对象
# -----------------------------------------------------------------------------
# 用途：在「SITL 仿真 / 实机网口 / 实机 USB」之间快速切换 MAVROS 的 fcu_url，
#       测试运行脚本时无需修改任何 launch/config 文件。
#
# 用法:
#   bash mavros_conn.sh {sim|sim-udp|usb|usb0|net}
#   bash mavros_conn.sh px4 {sim|usb|net}        # 连 PX4（用 px4.launch）
#
# 临时覆盖（不改脚本）:
#   FCU_URL=tcp://192.168.144.10:5760 bash mavros_conn.sh net
#   GCS_URL=udp://@127.0.0.1:14550    bash mavros_conn.sh sim   # 转发给地面站
#   TGT_SYSTEM=2                      bash mavros_conn.sh sim   # 指定目标系统号
#
# 连接方式说明:
#   sim      SITL，TCP 127.0.0.1:5760（SERIAL0，当前工程默认）
#   sim-udp  SITL，UDP 监听 0.0.0.0:14555
#            （需先给 SITL 加 --out=udp:127.0.0.1:14555）
#   usb      实机 USB CDC，/dev/ttyACM0:57600
#   usb0     实机 USB 数传，/dev/ttyUSB0:921600
#   net      实机网口/数传桥，udp://192.168.144.10:14550
#
# URL 语法（libmavconn）:
#   tcp://host:port      TCP 客户端（连 SITL 5760 用这个）
#   udp://host:port      UDP（自动判断收/发）
#   udpin://host:port    UDP 监听（本机作为接收端）
#   udpout://host:port   UDP 发送
#   serial:///dev/xxx:baud   串口（也兼容裸写 /dev/ttyACM0:57600）
#
# 注意:
#   1) 切换连接前请先杀掉旧 MAVROS，运行中改 fcu_url 不会可靠重连：
#          pkill -f mavros_node
#   2) 实机测试请勿使用 scripts/demo_up.sh（其中 fcu_url 写死 TCP 5760）。
#   3) USB 设备名会变，建议用稳定路径 /dev/serial/by-id/usb-...:57600。
#   4) 连上后确认: ros2 topic echo /mavros/state --once   (connected: true)
# =============================================================================
set -e

source /opt/ros/humble/setup.bash

# 第一参数可选 "px4"，切换所用 launch 与默认端口
LAUNCH="apm.launch"
ARGS=()
if [ "${1:-}" = "px4" ]; then
  LAUNCH="px4.launch"
  shift
fi
MODE="${1:-sim}"

case "$MODE" in
  sim|sitl) FCU_URL="${FCU_URL:-tcp://127.0.0.1:5760}" ;;          # 仿真 TCP 5760
  sim-udp)  FCU_URL="${FCU_URL:-udpin://0.0.0.0:14555}" ;;         # 仿真 UDP 监听
  usb)      FCU_URL="${FCU_URL:-/dev/ttyACM0:57600}" ;;            # 实机 USB CDC
  usb0)     FCU_URL="${FCU_URL:-/dev/ttyUSB0:921600}" ;;           # 实机 USB 数传
  net)      FCU_URL="${FCU_URL:-udp://192.168.144.10:14550}" ;;    # 实机网口
  *)
    echo "用法: bash $0 [px4] {sim|sim-udp|usb|usb0|net}"
    echo "  临时覆盖: FCU_URL=... GCS_URL=... TGT_SYSTEM=... bash $0 [px4] <模式>"
    exit 1
    ;;
esac

echo "[mavros_conn] launch=$LAUNCH"
echo "[mavros_conn] fcu_url=$FCU_URL"
[ -n "${GCS_URL:-}" ] && echo "[mavros_conn] gcs_url=$GCS_URL"
[ -n "${TGT_SYSTEM:-}" ] && echo "[mavros_conn] tgt_system=$TGT_SYSTEM"

LAUNCH_ARGS=(fcu_url:="$FCU_URL" tgt_system:="${TGT_SYSTEM:-1}")
if [ -n "${GCS_URL:-}" ]; then
  LAUNCH_ARGS+=(gcs_url:="$GCS_URL")
fi

exec ros2 launch mavros "$LAUNCH" "${LAUNCH_ARGS[@]}"
