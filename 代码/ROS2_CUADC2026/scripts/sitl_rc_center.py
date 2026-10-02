#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sitl_rc_center.py — 向 SITL 注入"油门摇杆" RC 输入（默认状态感知）。

背景（A3 LOITER 掉高根因，已定位）：
  ArduPilot SITL 的 UDP RC 后端 `AP_RCProtocol_UDP` 在收不到外部 RC 包时，
  会以 50Hz 用一组硬编码默认值合成 RC 输入：

      [1500, 1500, 1000, 1500, 1800, 1000, 1000, 1800]
                  ^ 通道 3（油门）= 1000（最低位）

  - GUIDED 模式忽略油门摇杆 -> 一键起飞正常；
  - LOITER 用油门摇杆控制升降 -> 摇杆在最低位 -> 持续下沉至地面。

  这不是 `one_key_takeoff.py` 的翻译问题，而是 SITL 没有真实 RC 发射机。

修复：
  向 SITL 的 RC-in UDP 端口发送 `uint16_t pwm[16]`（见
  `AP_RCProtocol_UDP::read_all_socket_input`）。端口默认 5501（`RCIN_PORT`），
  多实例时每实例 +10。

  但油门不能一直居中：Copter 解锁检查要求 "Throttle (RC3) is neutral"
  （油门在最低位）。因此本脚本默认"状态感知"：
    - 未解锁 -> ch3 = 1000（允许解锁）
    - 已解锁 -> ch3 = 1500（LOITER 悬停）
  解锁状态从 SITL SERIAL1(5762) 的 HEARTBEAT 读取。

用法：
  python3 sitl_rc_center.py                 # 状态感知（推荐）
  python3 sitl_rc_center.py --fixed 1500    # 固定 ch3，其余 1500
  python3 sitl_rc_center.py --port 5501
"""
import argparse
import socket
import struct
import sys
import time

DEFAULT_PORT = 5501
DISARMED_THROTTLE = 1000
ARMED_THROTTLE = 1500


def send_rc(port, ch3):
    """发送 16 通道包：ch3 为给定值，其余 1500。"""
    vals = [1500] * 16
    vals[2] = ch3
    pkt = struct.pack("<16H", *vals)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(pkt, ("127.0.0.1", port))
    finally:
        sock.close()


def run_fixed(port, ch3):
    print("固定模式：ch3=%d，其余 1500 -> 127.0.0.1:%d" % (ch3, port))
    while True:
        send_rc(port, ch3)
        time.sleep(1.0)


def run_state_aware(port, mav_port):
    from pymavlink import mavutil

    print("状态感知模式：未解锁 ch3=%d / 已解锁 ch3=%d -> UDP %d，"
          "心跳源 tcp:%d" % (DISARMED_THROTTLE, ARMED_THROTTLE, port, mav_port))
    last = None
    while True:
        try:
            m = mavutil.mavlink_connection(
                "tcp:127.0.0.1:%d" % mav_port, source_system=252
            )
            m.wait_heartbeat(timeout=20)
            print("已连接 SITL(SERIAL1:%d)，开始注入 RC" % mav_port)
            while True:
                msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
                if msg is None:
                    # 连接可能中断，重发一次保持设定
                    send_rc(port, last if last is not None else DISARMED_THROTTLE)
                    continue
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                ch3 = ARMED_THROTTLE if armed else DISARMED_THROTTLE
                send_rc(port, ch3)
                if armed != last:
                    print("armed=%s -> 注入 ch3=%d" % (armed, ch3))
                    last = armed
        except Exception as exc:
            print("RC 注入异常: %s，2s 后重连" % exc)
            time.sleep(2.0)


def main():
    ap = argparse.ArgumentParser(description="SITL RC 油门注入")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help="SITL RC-in UDP 端口（默认 %d）" % DEFAULT_PORT)
    ap.add_argument("--mav-port", type=int, default=5762,
                    help="读取 HEARTBEAT 的 SITL SERIAL1 端口（默认 5762）")
    ap.add_argument("--fixed", type=int, default=None,
                    help="固定 ch3 值（不走状态感知）")
    args = ap.parse_args()

    try:
        if args.fixed is not None:
            run_fixed(args.port, args.fixed)
        else:
            run_state_aware(args.port, args.mav_port)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
