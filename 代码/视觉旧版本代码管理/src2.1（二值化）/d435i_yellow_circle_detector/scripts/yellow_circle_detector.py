#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from d435i_yellow_circle_detector.msg import YellowCircle


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def find_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 2:
        contours, hierarchy = result
    else:
        _, contours, hierarchy = result
    return contours, hierarchy


class YellowCircleDetector:
    def __init__(self):
        color_topic = rospy.get_param("~color_topic", "/d435i/color/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/d435i/aligned_depth/image_raw")
        result_topic = rospy.get_param("~result_topic", "/yellow_circle")
        annotated_topic = rospy.get_param("~annotated_topic", "/yellow_circle/annotated_image")
        binary_topic = rospy.get_param("~binary_topic", "/yellow_circle/binary_image")

        self.lower_yellow = np.array(
            [
                rospy.get_param("~h_min", 18),
                rospy.get_param("~s_min", 80),
                rospy.get_param("~v_min", 80),
            ],
            dtype=np.uint8,
        )
        self.upper_yellow = np.array(
            [
                rospy.get_param("~h_max", 42),
                rospy.get_param("~s_max", 255),
                rospy.get_param("~v_max", 255),
            ],
            dtype=np.uint8,
        )
        self.min_area = float(rospy.get_param("~min_area", 120.0))
        self.max_area = float(rospy.get_param("~max_area", 200000.0))
        self.min_radius = float(rospy.get_param("~min_radius", 6.0))
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.55))
        self.center_roi_ratio = float(rospy.get_param("~center_roi_ratio", 0.85))
        self.morph_kernel = self.make_odd(int(rospy.get_param("~morph_kernel", 5)), minimum=1)
        self.morph_open_iterations = int(rospy.get_param("~morph_open_iterations", 1))
        self.morph_close_iterations = int(rospy.get_param("~morph_close_iterations", 2))
        self.show_window = get_bool_param("~show_window", False)
        self.display_scale = float(rospy.get_param("~display_scale", 1.0))
        self.window_name = "yellow_circle_detector"
        self.binary_window_name = "yellow_binary"
        self.window_ready = False

        self.bridge = CvBridge()
        self.depth_lock = threading.Lock()
        self.latest_depth = None
        self.latest_depth_header = None

        self.result_pub = rospy.Publisher(result_topic, YellowCircle, queue_size=1)
        self.annotated_pub = rospy.Publisher(annotated_topic, Image, queue_size=1)
        self.binary_pub = rospy.Publisher(binary_topic, Image, queue_size=1)
        self.depth_sub = rospy.Subscriber(
            depth_topic,
            Image,
            self.depth_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.color_sub = rospy.Subscriber(
            color_topic,
            Image,
            self.color_callback,
            queue_size=1,
            buff_size=2**24,
        )

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Depth conversion failed: %s", exc)
            return

        with self.depth_lock:
            self.latest_depth = np.array(depth, dtype=np.float32, copy=True)
            self.latest_depth_header = msg.header

    def color_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Color conversion failed: %s", exc)
            return

        detection, annotated, binary_mask = self.detect(image)
        result = self.build_result(msg, detection, image.shape)
        self.result_pub.publish(result)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        binary_msg = self.bridge.cv2_to_imgmsg(binary_mask, encoding="mono8")
        binary_msg.header = msg.header
        self.binary_pub.publish(binary_msg)

        if self.show_window:
            self.show_images(annotated, binary_mask)

    def show_images(self, annotated, binary_mask):
        if not self.window_ready:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow(self.binary_window_name, cv2.WINDOW_AUTOSIZE)
            self.window_ready = True

        display_annotated = annotated
        display_binary = binary_mask
        if self.display_scale > 0.0 and self.display_scale != 1.0:
            display_annotated = cv2.resize(
                annotated,
                None,
                fx=self.display_scale,
                fy=self.display_scale,
                interpolation=cv2.INTER_AREA,
            )
            display_binary = cv2.resize(
                binary_mask,
                None,
                fx=self.display_scale,
                fy=self.display_scale,
                interpolation=cv2.INTER_AREA,
            )

        cv2.imshow(self.window_name, display_annotated)
        cv2.imshow(self.binary_window_name, display_binary)
        cv2.waitKey(1)

    def make_odd(self, value, minimum):
        value = max(int(value), minimum)
        if value % 2 == 0:
            value += 1
        return value

    def detect(self, image):
        height, width = image.shape[:2]
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

        kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
        if self.morph_open_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=self.morph_open_iterations)
        if self.morph_close_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.morph_close_iterations)

        contours, _ = find_contours(mask)
        annotated = image.copy()
        detection = None
        best_score = -float("inf")

        center_x = width * 0.5
        center_y = height * 0.5
        roi_half_w = width * self.center_roi_ratio * 0.5
        roi_half_h = height * self.center_roi_ratio * 0.5

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area or area > self.max_area:
                continue

            perimeter = cv2.arcLength(contour, True)
            if perimeter <= 0:
                continue

            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.min_circularity:
                continue

            (x_float, y_float), radius = cv2.minEnclosingCircle(contour)
            if radius < self.min_radius:
                continue

            if abs(x_float - center_x) > roi_half_w or abs(y_float - center_y) > roi_half_h:
                continue

            distance_to_center = math.hypot(x_float - center_x, y_float - center_y)
            score = area * circularity - distance_to_center * 2.0
            if score > best_score:
                best_score = score
                detection = {
                    "x": int(round(x_float)),
                    "y": int(round(y_float)),
                    "radius": float(radius),
                    "area": float(area),
                    "circularity": float(circularity),
                    "contour": contour,
                }

        if detection is not None:
            x = detection["x"]
            y = detection["y"]
            radius = int(round(detection["radius"]))
            depth_m = self.lookup_depth(x, y)
            detection["depth_m"] = depth_m

            cv2.drawContours(annotated, [detection["contour"]], -1, (0, 255, 0), 2)
            cv2.circle(annotated, (x, y), radius, (0, 255, 255), 2)
            cv2.circle(annotated, (x, y), 4, (0, 0, 255), -1)
            label = "yellow circle ({:.3f} m)".format(depth_m) if depth_m > 0 else "yellow circle"
            cv2.putText(
                annotated,
                label,
                (max(0, x - radius), max(20, y - radius - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        return detection, annotated, mask

    def lookup_depth(self, x, y):
        with self.depth_lock:
            if self.latest_depth is None:
                return 0.0
            depth = self.latest_depth.copy()

        height, width = depth.shape[:2]
        if x < 0 or y < 0 or x >= width or y >= height:
            return 0.0

        radius = 2
        y0 = max(0, y - radius)
        y1 = min(height, y + radius + 1)
        x0 = max(0, x - radius)
        x1 = min(width, x + radius + 1)
        patch = depth[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.05)]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid))

    def build_result(self, image_msg, detection, image_shape):
        height, width = image_shape[:2]
        result = YellowCircle()
        result.header = image_msg.header
        result.detected = detection is not None

        if detection is None:
            result.x = -1
            result.y = -1
            result.radius = 0.0
            result.area = 0.0
            result.depth_m = 0.0
            result.center_offset_x = 0.0
            result.center_offset_y = 0.0
            return result

        result.x = detection["x"]
        result.y = detection["y"]
        result.radius = detection["radius"]
        result.area = detection["area"]
        result.depth_m = detection.get("depth_m", 0.0)
        result.center_offset_x = float(detection["x"] - width * 0.5)
        result.center_offset_y = float(detection["y"] - height * 0.5)
        return result

    def shutdown(self):
        if self.show_window:
            cv2.destroyAllWindows()


def main():
    rospy.init_node("yellow_circle_detector")
    node = YellowCircleDetector()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
