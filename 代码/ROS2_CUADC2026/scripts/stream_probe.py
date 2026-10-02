#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""stream_probe.py — 多话题"流实据"探针（错题 #16c：链路只认流，不认 state）

用法:
    python3 stream_probe.py [--secs 5] [话题[:类型] ...]
    无参 = 默认四件套: pose / compass_hdg / rel_alt / vision color

判据: 窗口内 消息数/secs >= 1 Hz 记 OK（收不到消息的冻结流 = MISS）。
输出: 每话题一行  "OK  <topic> hz=<rate> n=<count>" 或 "MISS <topic> ..."
退出码: 全部 OK = 0，否则 1。

rclpy 订阅判据（复用 verify_c1 已证模式），不用 `ros2 topic hz` CLI ——
后者管道缓冲会吞输出（错题 #4）。
"""
import argparse
import importlib
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

DEFAULT_TOPICS = [
    ("/mavros/local_position/pose", "geometry_msgs/msg/PoseStamped"),
    ("/mavros/global_position/compass_hdg", "std_msgs/msg/Float64"),
    ("/mavros/global_position/rel_alt", "std_msgs/msg/Float64"),
    ("/vision/color/image_raw", "sensor_msgs/msg/Image"),
]
MIN_HZ = 1.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument(
        "topics", nargs="*",
        help="话题[:类型]；省略类型时仅支持默认四件套",
    )
    args = ap.parse_args()

    defaults = dict(DEFAULT_TOPICS)
    items = []
    if args.topics:
        for spec in args.topics:
            topic, _, typ = spec.partition(":")
            if not typ:
                if topic in defaults:
                    typ = defaults[topic]
                else:
                    print("MISS %s — 无默认类型，请写 话题:类型" % topic, flush=True)
                    return 1
            items.append((topic, typ))
    else:
        items = list(DEFAULT_TOPICS)

    rclpy.init()
    node = rclpy.create_node("stream_probe")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin_th = threading.Thread(target=ex.spin, daemon=True)
    spin_th.start()

    qos = QoSProfile(
        depth=20,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    counts = {t: 0 for t, _ in items}
    for topic, typ in items:
        pkg, cls = typ.rsplit("/", 1)  # geometry_msgs/msg/PoseStamped → 模块 geometry_msgs.msg
        msg_cls = getattr(
            importlib.import_module(pkg.replace("/", ".")), cls
        )
        node.create_subscription(
            msg_cls, topic,
            lambda _m, t=topic: counts.__setitem__(t, counts[t] + 1),
            qos,
        )

    time.sleep(args.secs)

    all_ok = True
    for topic, _ in items:
        n = counts[topic]
        hz = n / args.secs
        ok = hz >= MIN_HZ
        all_ok = all_ok and ok
        print(
            "%s %s hz=%.1f n=%d" % ("OK  " if ok else "MISS", topic, hz, n),
            flush=True,
        )

    ex.shutdown()
    spin_th.join(timeout=2)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
