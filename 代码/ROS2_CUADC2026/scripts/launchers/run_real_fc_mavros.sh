#!/usr/bin/env bash
# 真机：连接 CUAV V6X v2 以太网 MAVLink（UDP 服务器 192.168.13.14:14550）
#   URL 规范（libmavconn）：udp://[bind_host]:[bind_port]@[remote_host]:[remote_port]
#   飞控是 UDP 服务器，MAVROS 作为客户端：本机绑 14555，发往飞控 14550，飞控回发到 14555。
#   （错误写法 udp://192.168.13.14:14550 会被当成"绑定到飞控 IP"，导致连不上→connected:false）
source /opt/ros/humble/setup.bash
LOCAL_PORT=14555
FC_IP=192.168.13.14
FC_PORT=14550
FCU="udp://:${LOCAL_PORT}@${FC_IP}:${FC_PORT}"
echo "=================================================="
echo " 真机 MAVROS：CUAV V6X v2   fcu_url=$FCU"
echo "=================================================="
echo "[真机] 拉起网口 cuav-v6x (192.168.13.100/24) ..."
nmcli connection up cuav-v6x >/dev/null 2>&1 || true
sleep 1
if ip -4 addr show enp86s0 2>/dev/null | grep -q "192.168.13.100"; then
  echo "[真机] 网口 OK : $(ip -4 -o addr show enp86s0 | awk '{print $4}')"
else
  echo "[真机] 警告: enp86s0 上没有 192.168.13.100/24，请检查网线与 nmcli cuav-v6x"
fi
if ping -c1 -W2 "$FC_IP" >/dev/null 2>&1; then
  echo "[真机] 飞控 $FC_IP 可达"
else
  echo "[真机] 警告: ping $FC_IP 不通（飞控未上电/网线/网段不对？）"
fi
echo "[真机] 关闭已有 MAVROS（避免抢链路）..."
for p in $(pgrep -f mavros_node); do kill "$p" 2>/dev/null; done
sleep 2
echo "[真机] 连接中 ..."
ros2 launch mavros apm.launch fcu_url:="$FCU" tgt_system:=1 &
MPID=$!
CONNECTED=0
for i in $(seq 1 30); do
  C=$(timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null | grep -m1 "connected:")
  if echo "$C" | grep -q "connected: true"; then
    echo "[真机] ✅ MAVROS 已连接飞控 (${i}s)：$(timeout 3 ros2 topic echo /mavros/state --once 2>/dev/null | grep -m1 mode)"
    CONNECTED=1; break
  fi
  sleep 1
done
if [ "$CONNECTED" != 1 ]; then
  echo "[真机] ❌ 30s 内未 connected:true。排查："
  echo "       1) 飞控 NET_P1 是否在 192.168.13.14:14550（UDP 服务器）"
  echo "       2) 是否有别的 GCS（MAVProxy/地面站）占着同一 UDP 服务器"
  echo "       3) 看 MAVROS 日志是否出现 'Got HEARTBEAT'"
fi
echo "[真机] Ctrl+C 退出。连上后自检: ros2 topic echo /mavros/state --once"
wait $MPID
