#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

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


class YellowCircleDetectorNode:
    def __init__(self):
        color_topic = rospy.get_param("~color_topic", "/d435i/color/image_raw")
        binary_topic = rospy.get_param("~binary_topic", "/yellow_circle/binary_image")
        depth_topic = rospy.get_param("~depth_topic", "/d435i/aligned_depth/image_raw")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/d435i/color/camera_info")
        result_topic = rospy.get_param("~result_topic", "/yellow_circle")
        annotated_topic = rospy.get_param("~annotated_topic", "/yellow_circle/annotated_image")

        self.min_area = float(rospy.get_param("~min_area", 120.0))
        self.max_area = float(rospy.get_param("~max_area", 200000.0))
        self.min_radius = float(rospy.get_param("~min_radius", 6.0))
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.55))
        self.center_roi_ratio = float(rospy.get_param("~center_roi_ratio", 0.85))
        self.target_diameter_m = float(rospy.get_param("~target_diameter_m", 0.20))
        self.use_diameter_depth_fallback = get_bool_param("~use_diameter_depth_fallback", True)
        self.prefer_diameter_depth = get_bool_param("~prefer_diameter_depth", True)
        self.depth_min_valid_m = float(rospy.get_param("~depth_min_valid_m", 0.05))
        self.depth_max_valid_m = float(rospy.get_param("~depth_max_valid_m", 0.35))
        self.depth_disagreement_ratio = float(rospy.get_param("~depth_disagreement_ratio", 0.35))
        self.auto_calibrate_distance = get_bool_param("~auto_calibrate_distance", False)
        self.calibration_distance_m = float(rospy.get_param("~calibration_distance_m", 0.20))
        self.calibration_radius_px = float(rospy.get_param("~calibration_radius_px", 0.0))
        self.calibration_samples = int(rospy.get_param("~calibration_samples", 30))
        self.calibration_radii = []
        self.enable_body_transform = get_bool_param("~enable_body_transform", False)
        self.camera_to_body_t = np.array(
            [
                float(rospy.get_param("~camera_to_body_x_m", 0.0)),
                float(rospy.get_param("~camera_to_body_y_m", 0.0)),
                float(rospy.get_param("~camera_to_body_z_m", 0.0)),
            ],
            dtype=np.float32,
        )
        self.camera_to_body_r = self.make_rotation_matrix(
            math.radians(float(rospy.get_param("~camera_to_body_roll_deg", 0.0))),
            math.radians(float(rospy.get_param("~camera_to_body_pitch_deg", 0.0))),
            math.radians(float(rospy.get_param("~camera_to_body_yaw_deg", 0.0))),
        )

        self.bridge = CvBridge()
        self.image_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        self.camera_info_lock = threading.Lock()
        self.latest_color = None
        self.latest_color_header = None
        self.latest_depth = None
        self.latest_camera_info = None

        self.result_pub = rospy.Publisher(result_topic, YellowCircle, queue_size=1)
        self.annotated_pub = rospy.Publisher(annotated_topic, Image, queue_size=1)
        self.color_sub = rospy.Subscriber(
            color_topic,
            Image,
            self.color_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.binary_sub = rospy.Subscriber(
            binary_topic,
            Image,
            self.binary_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.depth_sub = rospy.Subscriber(
            depth_topic,
            Image,
            self.depth_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "Yellow circle detector started. target_diameter=%.3f m depth_range=[%.2f, %.2f] prefer_diameter=%s auto_calibrate=%s calibration_distance=%.3f m calibration_radius=%.1f px camera_info=%s body_transform=%s",
            self.target_diameter_m,
            self.depth_min_valid_m,
            self.depth_max_valid_m,
            self.prefer_diameter_depth,
            self.auto_calibrate_distance,
            self.calibration_distance_m,
            self.calibration_radius_px,
            camera_info_topic,
            self.enable_body_transform,
        )

    def make_rotation_matrix(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float32)
        ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float32)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
        return rz.dot(ry).dot(rx)

    def color_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Color conversion failed in detector: %s", exc)
            return

        with self.image_lock:
            self.latest_color = np.array(image, copy=True)
            self.latest_color_header = msg.header

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Depth conversion failed in detector: %s", exc)
            return

        with self.depth_lock:
            self.latest_depth = np.array(depth, dtype=np.float32, copy=True)

    def camera_info_callback(self, msg):
        with self.camera_info_lock:
            self.latest_camera_info = msg

    def binary_callback(self, msg):
        try:
            binary_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Binary conversion failed in detector: %s", exc)
            return

        with self.image_lock:
            if self.latest_color is None:
                rospy.logwarn_throttle(2.0, "Detector is waiting for raw color image.")
                return
            image = self.latest_color.copy()
            color_header = self.latest_color_header

        detection, annotated = self.detect_from_binary(image, binary_mask)
        result = self.build_result(color_header or msg.header, detection, image.shape)
        self.result_pub.publish(result)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = color_header or msg.header
        self.annotated_pub.publish(annotated_msg)

    def detect_from_binary(self, image, binary_mask):
        height, width = image.shape[:2]
        contours, _ = find_contours(binary_mask)
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
            raw_depth_m = self.lookup_depth(x, y)
            diameter_depth_m = self.estimate_depth_from_diameter(detection["radius"])
            calibrated_depth_m = self.estimate_depth_from_calibration(detection["radius"])
            depth_m, depth_source = self.choose_depth(raw_depth_m, diameter_depth_m, calibrated_depth_m)
            detection["depth_m"] = depth_m
            detection["raw_depth_m"] = raw_depth_m
            detection["diameter_depth_m"] = diameter_depth_m
            detection["calibrated_depth_m"] = calibrated_depth_m
            detection["depth_source"] = depth_source
            position = self.project_pixel_to_position(x, y, depth_m)
            if position is not None:
                camera_xyz, body_xyz = position
                detection["position_valid"] = True
                detection["camera_x_m"] = float(camera_xyz[0])
                detection["camera_y_m"] = float(camera_xyz[1])
                detection["camera_z_m"] = float(camera_xyz[2])
                detection["distance_m"] = float(np.linalg.norm(camera_xyz))
                detection["body_x_m"] = float(body_xyz[0])
                detection["body_y_m"] = float(body_xyz[1])
                detection["body_z_m"] = float(body_xyz[2])
                detection["body_distance_m"] = float(np.linalg.norm(body_xyz))
            else:
                detection["position_valid"] = False

            cv2.drawContours(annotated, [detection["contour"]], -1, (0, 255, 0), 2)
            cv2.circle(annotated, (x, y), radius, (0, 255, 255), 2)
            cv2.circle(annotated, (x, y), 4, (0, 0, 255), -1)
            if detection.get("position_valid", False):
                label = "x={:.2f} y={:.2f} z={:.2f} d={:.2f}m {}".format(
                    detection["camera_x_m"],
                    detection["camera_y_m"],
                    detection["camera_z_m"],
                    detection["distance_m"],
                    detection["depth_source"],
                )
            else:
                label = "yellow circle r={:.0f}px {}".format(detection["radius"], detection["depth_source"])
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
            debug_label = "r={:.0f}px raw={:.2f} dia={:.2f} cal={:.2f}".format(
                detection["radius"],
                detection["raw_depth_m"],
                detection["diameter_depth_m"],
                detection["calibrated_depth_m"],
            )
            cv2.putText(
                annotated,
                debug_label,
                (max(0, x - radius), min(height - 8, y + radius + 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        return detection, annotated

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

    def is_expected_depth(self, depth_m):
        return (
            math.isfinite(depth_m)
            and depth_m >= self.depth_min_valid_m
            and depth_m <= self.depth_max_valid_m
        )

    def choose_depth(self, raw_depth_m, diameter_depth_m, calibrated_depth_m):
        raw_ok = self.is_expected_depth(raw_depth_m)
        diameter_ok = self.is_expected_depth(diameter_depth_m)
        calibrated_ok = self.is_expected_depth(calibrated_depth_m)

        if calibrated_ok:
            return calibrated_depth_m, "calibrated"

        if self.prefer_diameter_depth and diameter_ok:
            if not raw_ok:
                return diameter_depth_m, "diameter"

            diff = abs(raw_depth_m - diameter_depth_m)
            allowed = max(0.02, abs(diameter_depth_m) * self.depth_disagreement_ratio)
            if diff > allowed:
                rospy.logwarn_throttle(
                    2.0,
                    "Depth disagreement: raw=%.3f m diameter=%.3f m. Using diameter estimate.",
                    raw_depth_m,
                    diameter_depth_m,
                )
                return diameter_depth_m, "diameter"

        if raw_ok:
            return raw_depth_m, "depth"

        if diameter_ok:
            rospy.logwarn_throttle(
                2.0,
                "Raw depth is outside expected range: raw=%.3f m, diameter=%.3f m. Using diameter estimate.",
                raw_depth_m,
                diameter_depth_m,
            )
            return diameter_depth_m, "diameter"

        if raw_depth_m > 0.0 or diameter_depth_m > 0.0 or calibrated_depth_m > 0.0:
            rospy.logwarn_throttle(
                2.0,
                "All distance estimates are outside expected range [%.3f, %.3f] m: raw=%.3f diameter=%.3f calibrated=%.3f. Check target_diameter_m or use auto calibration.",
                self.depth_min_valid_m,
                self.depth_max_valid_m,
                raw_depth_m,
                diameter_depth_m,
                calibrated_depth_m,
            )

        return 0.0, "none"

    def get_latest_camera_info(self):
        with self.camera_info_lock:
            return self.latest_camera_info

    def estimate_depth_from_diameter(self, radius_px):
        if not self.use_diameter_depth_fallback:
            return 0.0
        if self.target_diameter_m <= 0.0 or radius_px <= 1.0:
            return 0.0

        camera_info = self.get_latest_camera_info()
        if camera_info is None:
            return 0.0

        fx = float(camera_info.K[0])
        if fx <= 0.0:
            return 0.0

        return float(fx * self.target_diameter_m / (2.0 * radius_px))

    def estimate_depth_from_calibration(self, radius_px):
        if radius_px <= 1.0 or self.calibration_distance_m <= 0.0:
            return 0.0

        if self.calibration_radius_px > 1.0:
            return float(self.calibration_distance_m * self.calibration_radius_px / radius_px)

        if not self.auto_calibrate_distance:
            return 0.0

        if len(self.calibration_radii) < self.calibration_samples:
            self.calibration_radii.append(float(radius_px))
            if len(self.calibration_radii) == self.calibration_samples:
                self.calibration_radius_px = float(np.median(np.array(self.calibration_radii, dtype=np.float32)))
                rospy.logwarn(
                    "Distance auto calibration complete: calibration_distance=%.3f m calibration_radius=%.1f px",
                    self.calibration_distance_m,
                    self.calibration_radius_px,
                )
            else:
                rospy.logwarn_throttle(
                    1.0,
                    "Collecting distance calibration samples: %d/%d. Keep target at %.3f m.",
                    len(self.calibration_radii),
                    self.calibration_samples,
                    self.calibration_distance_m,
                )
            return 0.0

        return float(self.calibration_distance_m * self.calibration_radius_px / radius_px)

    def project_pixel_to_position(self, x, y, depth_m):
        if depth_m <= 0.0 or not math.isfinite(depth_m):
            return None

        camera_info = self.get_latest_camera_info()
        if camera_info is None:
            return None

        fx = float(camera_info.K[0])
        fy = float(camera_info.K[4])
        cx = float(camera_info.K[2])
        cy = float(camera_info.K[5])
        if fx <= 0.0 or fy <= 0.0:
            return None

        camera_xyz = np.array(
            [
                (float(x) - cx) * depth_m / fx,
                (float(y) - cy) * depth_m / fy,
                depth_m,
            ],
            dtype=np.float32,
        )

        if self.enable_body_transform:
            body_xyz = self.camera_to_body_r.dot(camera_xyz) + self.camera_to_body_t
        else:
            body_xyz = camera_xyz.copy()

        return camera_xyz, body_xyz

    def build_result(self, header, detection, image_shape):
        height, width = image_shape[:2]
        result = YellowCircle()
        result.header = header
        result.detected = detection is not None

        if detection is None:
            result.x = -1
            result.y = -1
            result.radius = 0.0
            result.area = 0.0
            result.depth_m = 0.0
            result.raw_depth_m = 0.0
            result.diameter_depth_m = 0.0
            result.calibrated_depth_m = 0.0
            result.depth_source = "none"
            result.center_offset_x = 0.0
            result.center_offset_y = 0.0
            result.position_valid = False
            result.camera_x_m = 0.0
            result.camera_y_m = 0.0
            result.camera_z_m = 0.0
            result.distance_m = 0.0
            result.body_x_m = 0.0
            result.body_y_m = 0.0
            result.body_z_m = 0.0
            result.body_distance_m = 0.0
            return result

        result.x = detection["x"]
        result.y = detection["y"]
        result.radius = detection["radius"]
        result.area = detection["area"]
        result.depth_m = detection.get("depth_m", 0.0)
        result.raw_depth_m = detection.get("raw_depth_m", 0.0)
        result.diameter_depth_m = detection.get("diameter_depth_m", 0.0)
        result.calibrated_depth_m = detection.get("calibrated_depth_m", 0.0)
        result.depth_source = detection.get("depth_source", "none")
        result.center_offset_x = float(detection["x"] - width * 0.5)
        result.center_offset_y = float(detection["y"] - height * 0.5)
        result.position_valid = bool(detection.get("position_valid", False))
        result.camera_x_m = detection.get("camera_x_m", 0.0)
        result.camera_y_m = detection.get("camera_y_m", 0.0)
        result.camera_z_m = detection.get("camera_z_m", 0.0)
        result.distance_m = detection.get("distance_m", 0.0)
        result.body_x_m = detection.get("body_x_m", 0.0)
        result.body_y_m = detection.get("body_y_m", 0.0)
        result.body_z_m = detection.get("body_z_m", 0.0)
        result.body_distance_m = detection.get("body_distance_m", 0.0)
        return result


def main():
    rospy.init_node("yellow_circle_detector_node")
    YellowCircleDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
