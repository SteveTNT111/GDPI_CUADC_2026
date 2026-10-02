#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""servo_pub.py — 向 /servo/cmd 单发一条指令（Web UI 舵机按钮用）。

为什么要独立进程：`ros2 topic pub --once` 会阻塞等待"至少 1 个订阅者"（实测无订阅者时
一直打印 Waiting... 到超时），不适合 HTTP 请求。本脚本有界等待订阅者（默认 3s）：
  有订阅者 -> 发布并 rc=0；无订阅者 -> rc=3（附带说明），绝不长时间挂住 Web 请求。

用法: python3 servo_pub.py "A on"
"""

import sys
import time

try:
    import rclpy
    from std_msgs.msg import String
except Exception as exc:  # pragma: no cover
    sys.stderr.write("rclpy/std_msgs 不可用: %s\n" % exc)
    sys.exit(2)

TOPIC = "/servo/cmd"
WAIT_SUB_S = 3.0


def main():
    if len(sys.argv) < 2 or not sys.argv[1]:
        sys.stderr.write("用法: servo_pub.py \"A on\"\n")
        return 2
    text = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("webui_servo_pub")
    pub = node.create_publisher(String, TOPIC, 10)
    deadline = time.time() + WAIT_SUB_S
    while time.time() < deadline and pub.get_subscription_count() == 0:
        time.sleep(0.05)
    count = pub.get_subscription_count()
    if count == 0:
        print("无 %s 订阅者（servo_test.py 未运行？）" % TOPIC)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 3
    pub.publish(String(data=text))
    time.sleep(0.15)
    print("已发布 %s: '%s'（订阅者=%d）" % (TOPIC, text, count))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
