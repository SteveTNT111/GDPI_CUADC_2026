#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""show_camera.py — 屏幕实时窗口显示 B1 camera_node 的输出（当面验证用）。
左：/vision/color/image_raw (bgr8)   右：/vision/aligned_depth/image_raw 伪彩
顶部横幅显示实测 hz 与帧号。
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

WIN = "B1 camera_node live output"
q = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
frames = {"color": None, "depth": None, "n": 0, "t": 0.0}
t0 = time.time()


def on_color(msg):
    frames["color"] = msg
    frames["n"] += 1
    frames["t"] = time.time()


def on_depth(msg):
    frames["depth"] = msg


def main():
    rclpy.init()
    node = rclpy.create_node("show_camera")
    node.create_subscription(Image, "/vision/color/image_raw", on_color, q)
    node.create_subscription(Image, "/vision/aligned_depth/image_raw", on_depth, q)

    import threading
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    last_n, last_t = 0, time.time()
    hz = 0.0
    while True:
        if frames["color"] is None:
            time.sleep(0.05)
            continue
        # 实测 hz（1s 窗口）
        dt = time.time() - last_t
        if dt >= 1.0:
            hz = (frames["n"] - last_n) / dt
            last_n, last_t = frames["n"], time.time()

        c = frames["color"]
        color = np.frombuffer(bytes(c.data), dtype=np.uint8).reshape(
            c.height, c.width, 3).copy()

        right = np.zeros_like(color)
        if frames["depth"] is not None:
            d = frames["depth"]
            dep = np.frombuffer(bytes(d.data), dtype=np.float32).reshape(
                d.height, d.width)
            finite = np.isfinite(dep) & (dep > 0.05)
            valid_pct = 100.0 * float(finite.sum()) / dep.size
            if finite.any():
                lo = float(dep[finite].min())
                hi = float(np.percentile(dep[finite], 95))
                norm = np.clip((dep - lo) / max(hi - lo, 1e-6), 0, 1)
            else:
                norm = np.zeros_like(dep)
            right = cv2.applyColorMap((norm * 255).astype(np.uint8),
                                       cv2.COLORMAP_JET)

        canvas = np.hstack([color, right])
        banner = "B1 output  /vision/color/image_raw  %.1f Hz  frame #%d" % (
            hz, frames["n"])
        cv2.putText(canvas, banner, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(canvas, "%dx%d bgr8 step=%d" % (c.width, c.height, c.step),
                    (8, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1,
                    cv2.LINE_AA)
        if frames["depth"] is not None:
            d = frames["depth"]
            cv2.putText(canvas,
                        "depth 32FC1 %.0f%% valid" % valid_pct,
                        (c.width + 8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow(WIN, canvas)
        if cv2.waitKey(30) & 0xFF == 27:
            break
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
