#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
request_streams.py — 请求 SITL 推送周期性 MAVLink 消息（独立版）
====================================================================
SITL 后端（arducopter --model JSON）不会按 SR0_* 参数主动推流，
新连接默认只能收到 HEARTBEAT / STATUSTEXT。跑完本脚本后，
MAVROS 的 GPS/位置/姿态/高度话题才会有数据。

用法（在 MAVROS 启动之后）：
    source /opt/ros/humble/setup.bash
    source ~/ROS2_CUADC2026/install/setup.bash
    python3 request_streams.py

验证效果：
    ros2 topic echo /mavros/global_position/global --once
"""
import sys
import os
import time

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "install",
        "cuadc_vision",
        "lib",
        "cuadc_vision",
    ),
)

import rclpy
from mavros_msgs.srv import CommandLong

MAV_CMD_SET_MESSAGE_INTERVAL = 511
STREAM_MESSAGES = [
    ("ATTITUDE", 30),
    ("GPS_RAW_INT", 24),
    ("GLOBAL_POSITION_INT", 33),
    ("SYS_STATUS", 1),
    ("VFR_HUD", 74),
    ("RC_CHANNELS", 65),
    ("LOCAL_POSITION_NED", 32),
    ("HIGHRES_IMU", 105),
]


def main():
    rclpy.init()
    node = rclpy.create_node("request_streams")
    cli = node.create_client(CommandLong, "/mavros/cmd/command")
    if not cli.wait_for_service(timeout_sec=10):
        print("❌ /mavros/cmd/command 服务不可用。MAVROS 起来了吗？")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print("✅ 已连接 /mavros/cmd/command")
    ok_count = 0
    for name, msgid in STREAM_MESSAGES:
        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_SET_MESSAGE_INTERVAL
        req.confirmation = 0
        req.param1 = float(msgid)
        req.param2 = 100000.0  # 100 ms => 10 Hz
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
        if fut.result() is not None and fut.result().success:
            ok_count += 1
            print("  ✅ %-22s msgid=%d" % (name, msgid))
        else:
            print("  ❌ %-22s msgid=%d" % (name, msgid))
        time.sleep(0.1)
    print("请求完成：%d/%d 成功" % (ok_count, len(STREAM_MESSAGES)))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok_count == len(STREAM_MESSAGES) else 1


if __name__ == "__main__":
    sys.exit(main())