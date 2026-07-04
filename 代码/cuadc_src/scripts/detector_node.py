#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import threading

import cv2
import numpy as np
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from cuadc_vision.msg import YoloDetection, YoloDetections


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class DetectorNode:
    @staticmethod
    def _make_img_msg(array, encoding):
        """Construct a sensor_msgs/Image from a numpy array without cv_bridge.

        cv_bridge on ROS Noetic + OpenCV 4.x has a bug where cvtype_to_name
        is missing keys (e.g. CV_8UC3=16) that encoding_to_cvtype2 produces,
        causing KeyError when encoding "bgr8" is used.
        """
        if not array.flags["C_CONTIGUOUS"]:
            array = np.ascontiguousarray(array)
        msg = Image()
        msg.height = array.shape[0]
        msg.width = array.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(array.shape[1] * array.dtype.itemsize)
        msg.data = array.tobytes()
        return msg

    def __init__(self):
        _default_model = os.path.join(rospkg.RosPack().get_path('cuadc_vision'), 'models', 'best.pt')
        self.model_path = os.path.expanduser(rospy.get_param("~model_path", _default_model))
        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.5))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "cpu")
        self.invert_camera_x = get_bool_param("~invert_camera_x", True)
        self.depth_patch_radius = int(rospy.get_param("~depth_patch_radius", 2))
        self.show_window = get_bool_param("~show_window", False)
        self.window_name = rospy.get_param("~window_name", "YOLO Detection")

        color_topic = rospy.get_param("~color_topic", "/vision/color/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/vision/aligned_depth/image_raw")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/vision/color/camera_info")

        if not os.path.isfile(self.model_path):
            rospy.logerr("YOLO model not found: %s", self.model_path)
            rospy.signal_shutdown("Model not found")
            return

        try:
            from ultralytics import YOLO
        except Exception as exc:
            rospy.logerr("Failed to import ultralytics: %s. Run: pip3 install ultralytics", exc)
            rospy.signal_shutdown("ultralytics not installed")
            return

        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()
        self.depth_lock = threading.Lock()
        self.camera_info_lock = threading.Lock()
        self.latest_depth = None
        self.latest_camera_info = None

        self.result_pub = rospy.Publisher("/vision/yolo/detection", YoloDetection, queue_size=1)
        self.results_pub = rospy.Publisher("/vision/yolo/detections", YoloDetections, queue_size=1)
        self.annotated_pub = rospy.Publisher("/vision/annotated_image", Image, queue_size=1)

        self.color_sub = rospy.Subscriber(color_topic, Image, self.image_callback,
                                          queue_size=1, buff_size=2 ** 24)
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback,
                                          queue_size=1, buff_size=2 ** 24)
        self.camera_info_sub = rospy.Subscriber(camera_info_topic, CameraInfo,
                                                self.camera_info_callback, queue_size=1)

        rospy.loginfo("Detector started. model=%s conf=%.2f imgsz=%d device=%s",
                       self.model_path, self.conf_threshold, self.imgsz, self.device)

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception:
            return
        with self.depth_lock:
            self.latest_depth = np.array(depth, dtype=np.float32, copy=True)

    def camera_info_callback(self, msg):
        with self.camera_info_lock:
            self.latest_camera_info = msg

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Image conversion failed: %s", exc)
            return

        try:
            results = self.model(frame, conf=self.conf_threshold, imgsz=self.imgsz,
                                 device=self.device, verbose=False)
        except Exception as exc:
            rospy.logerr_throttle(2.0, "YOLO inference failed: %s", exc)
            return

        result_msg, detections_msg, best = self.build_results(msg.header, results[0], frame.shape)
        annotated = results[0].plot()
        if best is not None:
            self.draw_overlay(annotated, best)

        self.result_pub.publish(result_msg)
        self.results_pub.publish(detections_msg)
        annotated_msg = self._make_img_msg(annotated, "bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        if self.show_window:
            try:
                cv2.imshow(self.window_name, annotated)
                cv2.waitKey(1)
            except Exception:
                pass

    def lookup_depth(self, x, y):
        with self.depth_lock:
            if self.latest_depth is None:
                return 0.0
            depth = self.latest_depth.copy()
        h, w = depth.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return 0.0
        r = max(0, self.depth_patch_radius)
        y0, y1 = max(0, y - r), min(h, y + r + 1)
        x0, x1 = max(0, x - r), min(w, x + r + 1)
        patch = depth[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.01)]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid))

    def get_camera_info(self):
        with self.camera_info_lock:
            return self.latest_camera_info

    def build_results(self, header, yolo_result, image_shape):
        best_msg = self.empty_detection(header)
        detections_msg = YoloDetections()
        detections_msg.header = header
        detections_msg.detections = []

        boxes = getattr(yolo_result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return best_msg, detections_msg, None

        best = None
        best_conf = -1.0
        for box in boxes:
            det = self.build_single(header, yolo_result, box, image_shape)
            detections_msg.detections.append(det)
            if det.confidence > best_conf:
                best_conf = det.confidence
                best_msg = det
                best = det
        return best_msg, detections_msg, best

    def empty_detection(self, header):
        det = YoloDetection()
        det.header = header
        det.detected = False
        det.class_id = -1
        det.class_name = ""
        det.confidence = 0.0
        det.x_min = -1
        det.y_min = -1
        det.x_max = -1
        det.y_max = -1
        det.center_x = -1
        det.center_y = -1
        det.depth_m = 0.0
        det.position_valid = False
        det.camera_x_m = 0.0
        det.camera_y_m = 0.0
        det.camera_z_m = 0.0
        det.distance_m = 0.0
        det.bbox_width_m = 0.0
        det.bbox_height_m = 0.0
        return det

    def build_single(self, header, yolo_result, box, image_shape):
        det = self.empty_detection(header)
        xyxy = box.xyxy[0].detach().cpu().numpy()
        x_min, y_min, x_max, y_max = [int(round(v)) for v in xyxy]
        cx = int(round((x_min + x_max) * 0.5))
        cy = int(round((y_min + y_max) * 0.5))
        class_id = int(box.cls[0].detach().cpu().item())
        confidence = float(box.conf[0].detach().cpu().item())
        class_name = str(yolo_result.names.get(class_id, class_id))

        h, w = image_shape[:2]
        cx = max(0, min(w - 1, cx))
        cy = max(0, min(h - 1, cy))
        depth_m = self.lookup_depth(cx, cy)
        position = self.project_pixel(cx, cy, depth_m)

        det.detected = True
        det.class_id = class_id
        det.class_name = class_name
        det.confidence = confidence
        det.x_min = x_min
        det.y_min = y_min
        det.x_max = x_max
        det.y_max = y_max
        det.center_x = cx
        det.center_y = cy
        det.depth_m = depth_m

        if position is not None:
            d = float(np.linalg.norm(position))
            det.position_valid = True
            det.camera_x_m = float(position[0])
            det.camera_y_m = float(position[1])
            det.camera_z_m = float(position[2])
            det.distance_m = d
            det.bbox_width_m = self.pixel_to_m(max(0, x_max - x_min), depth_m, "x")
            det.bbox_height_m = self.pixel_to_m(max(0, y_max - y_min), depth_m, "y")
        return det

    def project_pixel(self, x, y, depth_m):
        if depth_m <= 0.0 or not math.isfinite(depth_m):
            return None
        ci = self.get_camera_info()
        if ci is None:
            return None
        fx, fy = float(ci.K[0]), float(ci.K[4])
        cx, cy = float(ci.K[2]), float(ci.K[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        sign = -1.0 if self.invert_camera_x else 1.0
        return np.array([sign * (float(x) - cx) * depth_m / fx,
                         (float(y) - cy) * depth_m / fy, depth_m], dtype=np.float32)

    def pixel_to_m(self, pixel_length, depth_m, axis="x"):
        if depth_m <= 0.0 or not math.isfinite(depth_m):
            return 0.0
        ci = self.get_camera_info()
        if ci is None:
            return 0.0
        focal = float(ci.K[0] if axis == "x" else ci.K[4])
        return float(pixel_length) * depth_m / focal if focal > 0.0 else 0.0

    def draw_overlay(self, image, det):
        x, y = det.center_x, det.center_y
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
        cv2.line(image, (x - 12, y), (x + 12, y), (0, 0, 255), 2)
        cv2.line(image, (x, y - 12), (x, y + 12), (0, 0, 255), 2)
        cv2.rectangle(image, (max(0, det.x_min), max(0, det.y_min)),
                      (max(0, det.x_max), max(0, det.y_max)), (0, 255, 255), 2)
        h_img, w_img = image.shape[:2]
        if det.position_valid:
            label = "{} {:.2f} x={:.2f} y={:.2f} z={:.2f} d={:.2f}m".format(
                det.class_name, det.confidence, det.camera_x_m, det.camera_y_m,
                det.camera_z_m, det.distance_m)
            panel = ["Best: {} conf={:.2f}".format(det.class_name, det.confidence),
                     "x={:.2f}m y={:.2f}m z={:.2f}m d={:.2f}m".format(
                         det.camera_x_m, det.camera_y_m, det.camera_z_m, det.distance_m),
                     "center=({}, {})".format(det.center_x, det.center_y)]
        else:
            label = "{} {:.2f} depth none".format(det.class_name, det.confidence)
            panel = ["Best: {} conf={:.2f}".format(det.class_name, det.confidence),
                     "depth none", "center=({}, {})".format(det.center_x, det.center_y)]
        self.draw_text_bg(image, label, max(4, det.x_min), max(22, det.y_min - 8),
                          text_color=(0, 255, 255))
        self.draw_panel(image, panel, w_img, h_img)

    def draw_text_bg(self, image, text, x, y, font_scale=0.55,
                      text_color=(0, 255, 0), bg_color=(0, 0, 0)):
        h, w = image.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 2
        (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        x = max(0, min(w - tw - 8, int(x)))
        y = max(th + 6, min(h - baseline - 4, int(y)))
        cv2.rectangle(image, (x - 4, y - th - 6), (x + tw + 4, y + baseline + 4), bg_color, -1)
        cv2.putText(image, text, (x, y), font, font_scale, text_color, thickness, cv2.LINE_AA)

    def draw_panel(self, image, lines, img_w, img_h):
        font = cv2.FONT_HERSHEY_SIMPLEX
        fs, thickness, pad, gap = 0.58, 2, 8, 7
        sizes = [cv2.getTextSize(l, font, fs, thickness)[0] for l in lines]
        pw = min(img_w - 8, max(s[0] for s in sizes) + pad * 2)
        lh = max(s[1] for s in sizes)
        ph = len(lines) * lh + (len(lines) - 1) * gap + pad * 2
        x0, y0 = 4, max(4, img_h - ph - 4)
        cv2.rectangle(image, (x0, y0), (x0 + pw, y0 + ph), (0, 0, 0), -1)
        cv2.rectangle(image, (x0, y0), (x0 + pw, y0 + ph), (0, 255, 255), 1)
        y = y0 + pad + lh
        for line in lines:
            cv2.putText(image, line, (x0 + pad, y), font, fs, (0, 255, 255), thickness, cv2.LINE_AA)
            y += lh + gap


def main():
    rospy.init_node("detector_node")
    DetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
