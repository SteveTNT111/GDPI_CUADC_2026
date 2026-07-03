#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading

import cv2
import math
import numpy as np
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from d435i_yellow_circle_detector.msg import GeoTarget, YoloDetection, YoloDetections


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class YoloDetectorNode:
    def __init__(self):
        _default_model = os.path.join(rospkg.RosPack().get_path('d435i_yellow_circle_detector'), 'models', 'best.pt')
        self.model_path = os.path.expanduser(rospy.get_param("~model_path", _default_model))
        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.5))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "cpu")
        self.image_topic = rospy.get_param("~image_topic", "/d435i/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/d435i/aligned_depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/d435i/color/camera_info")
        self.annotated_topic = rospy.get_param("~annotated_topic", "/yolo/annotated_image")
        self.result_topic = rospy.get_param("~result_topic", "/yolo/detection")
        self.results_topic = rospy.get_param("~results_topic", "/yolo/detections")
        self.global_target_topic = rospy.get_param("~global_target_topic", "/competition/target_global")
        self.global_target_max_age_sec = float(rospy.get_param("~global_target_max_age_sec", 1.0))
        self.show_window = get_bool_param("~show_window", True)
        self.window_name = rospy.get_param("~window_name", "YOLO Detection")
        self.depth_patch_radius = int(rospy.get_param("~depth_patch_radius", 2))
        self.invert_camera_x = get_bool_param("~invert_camera_x", True)

        if not os.path.isfile(self.model_path):
            rospy.logerr("YOLO model file not found: %s", self.model_path)
            rospy.signal_shutdown("YOLO model file not found")
            return

        try:
            from ultralytics import YOLO
        except Exception as exc:
            rospy.logerr(
                "Failed to import ultralytics: %s. Install it with: pip3 install ultralytics",
                exc,
            )
            rospy.signal_shutdown("ultralytics import failed")
            return

        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()
        self.depth_lock = threading.Lock()
        self.camera_info_lock = threading.Lock()
        self.global_target_lock = threading.Lock()
        self.latest_depth = None
        self.latest_camera_info = None
        self.latest_global_target = None

        self.annotated_pub = rospy.Publisher(self.annotated_topic, Image, queue_size=1)
        self.result_pub = rospy.Publisher(self.result_topic, YoloDetection, queue_size=1)
        self.results_pub = rospy.Publisher(self.results_topic, YoloDetections, queue_size=1)
        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.depth_sub = rospy.Subscriber(
            self.depth_topic,
            Image,
            self.depth_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )
        self.global_target_sub = rospy.Subscriber(
            self.global_target_topic,
            GeoTarget,
            self.global_target_callback,
            queue_size=1,
        )
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "YOLO detector started. model=%s image=%s depth=%s camera_info=%s conf=%.2f imgsz=%d device=%s show_window=%s",
            self.model_path,
            self.image_topic,
            self.depth_topic,
            self.camera_info_topic,
            self.conf_threshold,
            self.imgsz,
            self.device,
            self.show_window,
        )

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "YOLO depth conversion failed: %s", exc)
            return

        with self.depth_lock:
            self.latest_depth = np.array(depth, dtype=np.float32, copy=True)

    def camera_info_callback(self, msg):
        with self.camera_info_lock:
            self.latest_camera_info = msg

    def global_target_callback(self, msg):
        with self.global_target_lock:
            self.latest_global_target = msg

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "YOLO image conversion failed: %s", exc)
            return

        try:
            results = self.model(
                frame,
                conf=self.conf_threshold,
                imgsz=self.imgsz,
                device=self.device,
                verbose=False,
            )
        except Exception as exc:
            rospy.logerr_throttle(2.0, "YOLO inference failed: %s", exc)
            return

        result_msg, detections_msg, best = self.build_results(msg.header, results[0], frame.shape)
        annotated = results[0].plot()
        if best is not None:
            self.draw_best_detection_overlay(annotated, best)

        self.result_pub.publish(result_msg)
        self.results_pub.publish(detections_msg)
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        if self.show_window:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)

    def make_empty_detection(self, header):
        result_msg = YoloDetection()
        result_msg.header = header
        result_msg.detected = False
        result_msg.class_id = -1
        result_msg.class_name = ""
        result_msg.confidence = 0.0
        result_msg.x_min = -1
        result_msg.y_min = -1
        result_msg.x_max = -1
        result_msg.y_max = -1
        result_msg.center_x = -1
        result_msg.center_y = -1
        result_msg.depth_m = 0.0
        result_msg.position_valid = False
        result_msg.camera_x_m = 0.0
        result_msg.camera_y_m = 0.0
        result_msg.camera_z_m = 0.0
        result_msg.distance_m = 0.0
        result_msg.bbox_width_m = 0.0
        result_msg.bbox_height_m = 0.0
        return result_msg

    def build_results(self, header, yolo_result, image_shape):
        best_msg = self.make_empty_detection(header)
        detections_msg = YoloDetections()
        detections_msg.header = header
        detections_msg.detections = []

        boxes = getattr(yolo_result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return best_msg, detections_msg, None

        best = None
        best_confidence = -1.0
        for box in boxes:
            detection = self.build_single_detection(header, yolo_result, box, image_shape)
            detections_msg.detections.append(detection)
            if detection.confidence > best_confidence:
                best_confidence = detection.confidence
                best_msg = detection
                best = detection

        return best_msg, detections_msg, best

    def build_single_detection(self, header, yolo_result, box, image_shape):
        result_msg = self.make_empty_detection(header)
        xyxy = box.xyxy[0].detach().cpu().numpy()
        x_min, y_min, x_max, y_max = [int(round(v)) for v in xyxy]
        center_x = int(round((x_min + x_max) * 0.5))
        center_y = int(round((y_min + y_max) * 0.5))
        class_id = int(box.cls[0].detach().cpu().item())
        confidence = float(box.conf[0].detach().cpu().item())
        class_name = str(yolo_result.names.get(class_id, class_id))

        height, width = image_shape[:2]
        center_x = max(0, min(width - 1, center_x))
        center_y = max(0, min(height - 1, center_y))
        depth_m = self.lookup_depth(center_x, center_y)
        position = self.project_pixel_to_position(center_x, center_y, depth_m)

        result_msg.detected = True
        result_msg.class_id = class_id
        result_msg.class_name = class_name
        result_msg.confidence = confidence
        result_msg.x_min = x_min
        result_msg.y_min = y_min
        result_msg.x_max = x_max
        result_msg.y_max = y_max
        result_msg.center_x = center_x
        result_msg.center_y = center_y
        result_msg.depth_m = depth_m

        if position is not None:
            camera_xyz = position
            distance_m = float(np.linalg.norm(camera_xyz))
            result_msg.position_valid = True
            result_msg.camera_x_m = float(camera_xyz[0])
            result_msg.camera_y_m = float(camera_xyz[1])
            result_msg.camera_z_m = float(camera_xyz[2])
            result_msg.distance_m = distance_m
            result_msg.bbox_width_m = self.pixel_length_to_meters(max(0, x_max - x_min), depth_m, axis="x")
            result_msg.bbox_height_m = self.pixel_length_to_meters(max(0, y_max - y_min), depth_m, axis="y")

        return result_msg

    def lookup_depth(self, x, y):
        with self.depth_lock:
            if self.latest_depth is None:
                return 0.0
            depth = self.latest_depth.copy()

        height, width = depth.shape[:2]
        if x < 0 or y < 0 or x >= width or y >= height:
            return 0.0

        radius = max(0, self.depth_patch_radius)
        y0 = max(0, y - radius)
        y1 = min(height, y + radius + 1)
        x0 = max(0, x - radius)
        x1 = min(width, x + radius + 1)
        patch = depth[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.01)]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid))

    def get_latest_camera_info(self):
        with self.camera_info_lock:
            return self.latest_camera_info

    def get_latest_global_target(self):
        with self.global_target_lock:
            target = self.latest_global_target
        if target is None or not target.valid:
            return None
        if self.global_target_max_age_sec <= 0.0:
            return target
        stamp = target.header.stamp
        if stamp == rospy.Time(0):
            return target
        age = abs((rospy.Time.now() - stamp).to_sec())
        if age > self.global_target_max_age_sec:
            return None
        return target

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

        return np.array(
            [
                self.camera_x_sign() * (float(x) - cx) * depth_m / fx,
                (float(y) - cy) * depth_m / fy,
                depth_m,
            ],
            dtype=np.float32,
        )

    def camera_x_sign(self):
        return -1.0 if self.invert_camera_x else 1.0

    def pixel_length_to_meters(self, pixel_length, depth_m, axis="x"):
        if depth_m <= 0.0 or not math.isfinite(depth_m):
            return 0.0

        camera_info = self.get_latest_camera_info()
        if camera_info is None:
            return 0.0

        focal = float(camera_info.K[0] if axis == "x" else camera_info.K[4])
        if focal <= 0.0:
            return 0.0

        return float(pixel_length) * depth_m / focal

    def draw_best_detection_overlay(self, image, detection):
        x = detection.center_x
        y = detection.center_y
        global_target = self.get_latest_global_target()
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
        cv2.line(image, (x - 12, y), (x + 12, y), (0, 0, 255), 2)
        cv2.line(image, (x, y - 12), (x, y + 12), (0, 0, 255), 2)
        cv2.rectangle(
            image,
            (max(0, detection.x_min), max(0, detection.y_min)),
            (max(0, detection.x_max), max(0, detection.y_max)),
            (0, 255, 255),
            2,
        )

        if detection.position_valid:
            label = "{} {:.2f} x={:.2f} y={:.2f} z={:.2f} d={:.2f}m".format(
                detection.class_name,
                detection.confidence,
                detection.camera_x_m,
                detection.camera_y_m,
                detection.camera_z_m,
                detection.distance_m,
            )
            panel_lines = [
                "Best: {} conf={:.2f}".format(detection.class_name, detection.confidence),
                "x={:.2f}m y={:.2f}m z={:.2f}m d={:.2f}m".format(
                    detection.camera_x_m,
                    detection.camera_y_m,
                    detection.camera_z_m,
                    detection.distance_m,
                ),
                "center=({}, {})".format(detection.center_x, detection.center_y),
            ]
            if global_target is not None:
                panel_lines.extend(
                    [
                        "lat={:.7f} lon={:.7f}".format(
                            global_target.latitude,
                            global_target.longitude,
                        ),
                        "alt={:.2f}m body=({:.2f},{:.2f},{:.2f})".format(
                            global_target.altitude,
                            global_target.body_x_m,
                            global_target.body_y_m,
                            global_target.body_z_m,
                        ),
                    ]
                )
        else:
            label = "{} {:.2f} depth none".format(
                detection.class_name,
                detection.confidence,
            )
            panel_lines = [
                "Best: {} conf={:.2f}".format(detection.class_name, detection.confidence),
                "depth none",
                "center=({}, {})".format(detection.center_x, detection.center_y),
            ]

        self.draw_text_with_background(
            image,
            label,
            max(4, detection.x_min),
            max(22, detection.y_min - 8),
            font_scale=0.55,
            text_color=(0, 255, 255),
            bg_color=(0, 0, 0),
        )
        if global_target is not None:
            self.draw_text_with_background(
                image,
                "lat={:.7f} lon={:.7f}".format(
                    global_target.latitude,
                    global_target.longitude,
                ),
                max(4, detection.x_min),
                max(44, detection.y_min + 22),
                font_scale=0.45,
                text_color=(255, 255, 255),
                bg_color=(30, 30, 30),
            )
            self.draw_text_with_background(
                image,
                "alt={:.2f}m".format(global_target.altitude),
                max(4, detection.x_min),
                max(64, detection.y_min + 44),
                font_scale=0.45,
                text_color=(255, 255, 255),
                bg_color=(30, 30, 30),
            )
        self.draw_fixed_status_panel(image, panel_lines)

    def draw_fixed_status_panel(self, image, lines):
        height, width = image.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.58
        thickness = 2
        padding = 8
        line_gap = 7
        sizes = [cv2.getTextSize(line, font, font_scale, thickness)[0] for line in lines]
        panel_width = min(width - 8, max(size[0] for size in sizes) + padding * 2)
        line_height = max(size[1] for size in sizes)
        panel_height = len(lines) * line_height + (len(lines) - 1) * line_gap + padding * 2
        x0 = 4
        y0 = max(4, height - panel_height - 4)
        x1 = x0 + panel_width
        y1 = y0 + panel_height
        cv2.rectangle(image, (x0, y0), (x1, y1), (0, 0, 0), -1)
        cv2.rectangle(image, (x0, y0), (x1, y1), (0, 255, 255), 1)

        y = y0 + padding + line_height
        for line in lines:
            cv2.putText(
                image,
                line,
                (x0 + padding, y),
                font,
                font_scale,
                (0, 255, 255),
                thickness,
                cv2.LINE_AA,
            )
            y += line_height + line_gap

    def draw_text_with_background(self, image, text, x, y, font_scale=0.55, text_color=(0, 255, 0), bg_color=(0, 0, 0)):
        height, width = image.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        x = max(0, min(width - text_width - 8, int(x)))
        y = max(text_height + 6, min(height - baseline - 4, int(y)))
        cv2.rectangle(
            image,
            (x - 4, y - text_height - 6),
            (x + text_width + 4, y + baseline + 4),
            bg_color,
            -1,
        )
        cv2.putText(
            image,
            text,
            (x, y),
            font,
            font_scale,
            text_color,
            thickness,
            cv2.LINE_AA,
        )

    def shutdown(self):
        if self.show_window:
            cv2.destroyWindow(self.window_name)


def main():
    rospy.init_node("yolo_detector_node")
    YoloDetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
