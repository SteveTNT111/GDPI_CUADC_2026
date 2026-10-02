#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_fc_link.py — 用与 auto_drop_test 相同的判据，检查"任务能不能看到 MAVROS/飞控连接"。

外场用法（MAVROS 已启动、能看飞控信息的那个终端里）：
    source /opt/ros/humble/setup.bash && source ~/ROS2_CUADC2026/install/setup.bash
    python3 ~/ROS2_CUADC2026/scripts/check_fc_link.py

它复刻全流程的两条判据：
  1) `_node_is_running("mavros")` / `_mavros_topic_exists()`（看得到 mavros 节点或 /mavros/state）
  2) `wait_for_mavros()`：/mavros/state 新鲜且 connected=True（QoS 与任务一致：BEST_EFFORT）
"""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import State


def main():
    print("=" * 60)
    print("环境：ROS_DOMAIN_ID=%s  RMW_IMPLEMENTATION=%s  ROS_LOCALHOST_ONLY=%s"
          % (os.environ.get("ROS_DOMAIN_ID", "(unset)"),
             os.environ.get("RMW_IMPLEMENTATION", "(default)"),
             os.environ.get("ROS_LOCALHOST_ONLY", "(unset)")))
    rclpy.init()
    node = Node("check_fc_link")
    got = {}
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(State, "/mavros/state", lambda m: got.setdefault("s", m), qos)

    deadline = time.time() + 3.0
    nodes = []
    while time.time() < deadline and not nodes:
        rclpy.spin_once(node, timeout_sec=0.3)
        nodes = list(node.get_node_names())
    topics = dict(node.get_topic_names_and_types())
    pubs = node.count_publishers("/mavros/state")
    mavros_node = any(x.rstrip("/").split("/")[-1] == "mavros" for x in nodes)

    print("mavros 节点可见:", mavros_node)
    print("节点列表:", nodes)
    print("/mavros/state 在话题表:", "/mavros/state" in topics,
          " 发布者数:", pubs)

    t0 = time.time()
    while rclpy.ok() and time.time() - t0 < 6.0 and "s" not in got:
        rclpy.spin_once(node, timeout_sec=0.5)
    s = got.get("s")
    print("-" * 60)
    if s is None:
        print("结论：❌ 6s 内收不到 /mavros/state —— 任务会判「飞控未连接」。")
        print("      可能：MAVROS 与任务不在同一 ROS 环境/用户/域，或 MAVROS 已退出。")
    else:
        print("state: connected=%s armed=%s mode=%s system_status=%s"
              % (s.connected, s.armed, s.mode, s.system_status))
        if s.connected:
            print("结论：✅ 任务会判「MAVROS 已连接」。")
        else:
            print("结论：❌ 能收到 state 但 connected=False —— MAVROS 没连上飞控，任务判「飞控未连接」。")
            print("      真机正确 URL：udp://:14555@192.168.13.14:14550（本机绑 14555→飞控 14550）")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
