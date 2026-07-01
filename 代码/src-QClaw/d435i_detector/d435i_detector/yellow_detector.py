#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
黄色圆检测节点
- 订阅 /d435i/color_image (彩色图)
- 订阅 /d435i/depth_image (深度图)
- 使用 HSV 颜色过滤 + 轮廓检测识别黄色圆
- 发布检测结果到 /d435i/yellow_circle (自定义消息 YellowCircle)
- 发布标注后的图像到 /d435i/detected_image (sensor_msgs/Image)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from d435i_detector.msg import YellowCircle
import cv2
import numpy as np


class YellowDetector(Node):
    """黄色圆检测器"""

    def __init__(self):
        super().__init__('yellow_detector')

        # ---- 声明参数 ----
        # HSV 黄色范围（可根据实际场景调参）
        self.declare_parameter('h_min', 20)
        self.declare_parameter('h_max', 40)
        self.declare_parameter('s_min', 80)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 80)
        self.declare_parameter('v_max', 255)
        self.declare_parameter('min_contour_area', 200)   # 最小轮廓面积
        self.declare_parameter('circularity_thresh', 0.6) # 圆度阈值（越接近1越圆）
        self.declare_parameter('depth_scale', 0.001)      # 深度缩放系数（米/单位）

        self.h_min   = self.get_parameter('h_min').value
        self.h_max   = self.get_parameter('h_max').value
        self.s_min   = self.get_parameter('s_min').value
        self.s_max   = self.get_parameter('s_max').value
        self.v_min   = self.get_parameter('v_min').value
        self.v_max   = self.get_parameter('v_max').value
        self.min_area = self.get_parameter('min_contour_area').value
        self.circ_thresh = self.get_parameter('circularity_thresh').value
        self.depth_scale = self.get_parameter('depth_scale').value

        # ---- QoS ----
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ---- 订阅者 ----
        self.color_sub = self.create_subscription(
            Image, '/d435i/color_image', self.color_callback, qos)
        self.depth_sub = self.create_subscription(
            Image, '/d435i/depth_image', self.depth_callback, qos)

        # ---- 发布者 ----
        self.result_pub = self.create_publisher(YellowCircle, '/d435i/yellow_circle', 10)
        self.image_pub  = self.create_publisher(Image, '/d435i/detected_image', qos)

        self.cv_bridge = CvBridge()

        # 缓存最新深度帧
        self.latest_depth = None

        # 检测结果状态
        self.det_x = 0
        self.det_y = 0
        self.det_depth = 0.0
        self.detected = False

        self.get_logger().info(
            f'Yellow detector started | HSV H:[{self.h_min}-{self.h_max}] '
            f'S:[{self.s_min}-{self.s_max}] V:[{self.v_min}-{self.v_max}] '
            f'min_area:{self.min_area} circ>{self.circ_thresh}')

    # ------------------------------------------------------------------
    def depth_callback(self, msg):
        """缓存最新深度帧"""
        try:
            self.latest_depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    # ------------------------------------------------------------------
    def color_callback(self, msg):
        """彩色帧回调：HSV 过滤 → 轮廓检测 → 圆度筛选 → 发布"""
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Color conversion failed: {e}')
            return

        # ---- HSV 黄色过滤 ----
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([self.h_min, self.s_min, self.v_min])
        upper_yellow = np.array([self.h_max, self.s_max, self.v_max])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 形态学去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # ---- 轮廓检测 ----
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_cnt = None
        best_score = 0  # 圆度 * 面积 综合评分

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue

            # 圆度 = 4π × 面积 / 周长²，完美圆=1
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)

            if circularity < self.circ_thresh:
                continue

            score = circularity * area
            if score > best_score:
                best_score = score
                best_cnt = cnt

        # ---- 发布结果 ----
        result = YellowCircle()
        result.x = 0
        result.y = 0
        result.depth = 0.0
        self.detected = False

        if best_cnt is not None:
            # 最小外接圆
            (cx, cy), radius = cv2.minEnclosingCircle(best_cnt)
            cx, cy, radius = int(cx), int(cy), int(radius)

            result.x = cx
            result.y = cy

            # ---- 取深度 ----
            if self.latest_depth is not None:
                h, w = self.latest_depth.shape[:2]
                if 0 <= cy < h and 0 <= cx < w:
                    raw_depth = self.latest_depth[cy, cx]
                    result.depth = float(raw_depth) * self.depth_scale

            self.detected = True
            self.det_x, self.det_y, self.det_depth = result.x, result.y, result.depth

            # ---- 在图像上标注 ----
            cv2.drawContours(frame, [best_cnt], -1, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)

            label = f'({cx},{cy}) d={result.depth:.2f}m'
            cv2.putText(frame, label, (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            self.get_logger().info(f'Yellow circle at ({cx},{cy}), depth={result.depth:.3f}m')
        else:
            # 无检测
            cv2.putText(frame, 'No yellow circle detected', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 发布检测结果消息
        result.header.stamp = msg.header.stamp
        self.result_pub.publish(result)

        # 发布标注后图像
        try:
            annotated_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
        except Exception:
            pass

        # 本地显示（调试用，部署时可关闭）
        cv2.imshow('Yellow Detector', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YellowDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
