#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import threading

import cv2
import math
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from d435i_yellow_circle_detector.msg import YoloDetection


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class YoloDetectorNode:
    def __init__(self):
        self.model_path = os.path.expanduser(rospy.get_param("~model_path", "/home/lab/model/best.pt"))
        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.5))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "cpu")
        self.image_topic = rospy.get_param("~image_topic", "/d435i/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/d435i/aligned_depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/d435i/color/camera_info")
        self.annotated_topic = rospy.get_param("~annotated_topic", "/yolo/annotated_image")
        self.result_topic = rospy.get_param("~result_topic", "/yolo/detection")
        self.show_window = get_bool_param("~show_window", True)
        self.window_name = rospy.get_param("~window_name", "YOLO Detection")
        self.depth_patch_radius = int(rospy.get_param("~depth_patch_radius", 2))

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
        self.latest_depth = None
        self.latest_camera_info = None

        self.annotated_pub = rospy.Publisher(self.annotated_topic, Image, queue_size=1)
        self.result_pub = rospy.Publisher(self.result_topic, YoloDetection, queue_size=1)
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

        result_msg, best = self.build_result(msg.header, results[0], frame.shape)
        annotated = results[0].plot()
        if best is not None:
            self.draw_center_and_distance(annotated, best)

        self.result_pub.publish(result_msg)
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        if self.show_window:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)

    def build_result(self, header, yolo_result, image_shape):
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

        boxes = getattr(yolo_result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return result_msg, None

        best_index = int(np.argmax(boxes.conf.detach().cpu().numpy()))
        box = boxes[best_index]
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

        best = {
            "class_name": class_name,
            "confidence": confidence,
            "center_x": center_x,
            "center_y": center_y,
            "depth_m": depth_m,
            "position_valid": False,
        }

        if position is not None:
            camera_xyz = position
            distance_m = float(np.linalg.norm(camera_xyz))
            result_msg.position_valid = True
            result_msg.camera_x_m = float(camera_xyz[0])
            result_msg.camera_y_m = float(camera_xyz[1])
            result_msg.camera_z_m = float(camera_xyz[2])
            result_msg.distance_m = distance_m

            best["position_valid"] = True
            best["camera_x_m"] = result_msg.camera_x_m
            best["camera_y_m"] = result_msg.camera_y_m
            best["camera_z_m"] = result_msg.camera_z_m
            best["distance_m"] = distance_m

        return result_msg, best

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
                (float(x) - cx) * depth_m / fx,
                (float(y) - cy) * depth_m / fy,
                depth_m,
            ],
            dtype=np.float32,
        )

    def draw_center_and_distance(self, image, detection):
        x = detection["center_x"]
        y = detection["center_y"]
        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)
        cv2.line(image, (x - 12, y), (x + 12, y), (0, 0, 255), 2)
        cv2.line(image, (x, y - 12), (x, y + 12), (0, 0, 255), 2)

        if detection.get("position_valid", False):
            label = "{} {:.2f} x={:.2f} y={:.2f} z={:.2f} d={:.2f}m".format(
                detection["class_name"],
                detection["confidence"],
                detection["camera_x_m"],
                detection["camera_y_m"],
                detection["camera_z_m"],
                detection["distance_m"],
            )
        else:
            label = "{} {:.2f} depth none".format(
                detection["class_name"],
                detection["confidence"],
            )

        cv2.putText(
            image,
            label,
            (max(0, x - 120), max(22, y - 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
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
