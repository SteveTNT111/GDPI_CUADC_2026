#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
detector_node.py — CUADC 2026 YOLO 目标检测节点

功能：
  1. 订阅相机图像话题，加载 YOLOv8 模型逐帧推理
  2. 按类别关键词过滤（只保留圆筒相关目标）
  3. 查深度图获取目标距离，反投影为相机系 3D 坐标
  4. 发布检测结果 + 标注画面
  5. 可选弹出 OpenCV 窗口实时显示

订阅话题：
  - 彩色图像 (sensor_msgs/Image)
  - 对齐深度图 (sensor_msgs/Image)
  - 相机内参 (sensor_msgs/CameraInfo)

发布话题：
  - /vision/yolo/detection  (YoloDetection) — 最高置信度目标
  - /vision/yolo/detections (YoloDetections) — 全部目标
  - /vision/annotated_image  (sensor_msgs/Image) — 标注画面
"""

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
    """解析 ROS 参数为 bool（兼容字符串 "true"/"1" 等写法）"""
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class DetectorNode:
    """YOLO 目标检测 ROS 节点"""

    # ========================================================================
    # 初始化
    # ========================================================================

    def __init__(self):
        # ---------- 模型参数 ----------
        # 默认从包内 models/ 目录找 best.pt
        _default_model = os.path.join(rospkg.RosPack().get_path('cuadc_vision'), 'models', 'best.pt')
        self.model_path = os.path.expanduser(rospy.get_param("~model_path", _default_model))
        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.5))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.device = rospy.get_param("~device", "cpu")

        # ---------- 坐标 ----------
        self.invert_camera_x = get_bool_param("~invert_camera_x", True)

        # ---------- 深度 ----------
        self.depth_patch_radius = int(rospy.get_param("~depth_patch_radius", 2))

        # ---------- 显示 ----------
        self.show_window = get_bool_param("~show_window", False)
        self.window_name = rospy.get_param("~window_name", "YOLO Detection")

        # ---------- 类别过滤 ----------
        # 只保留名称包含这些关键词的目标（逗号分隔）
        target_str = rospy.get_param(
            "~target_classes",
            "cylinder,tong,barrel,can,yuantong,white_cylinder,tube,drum"
        )
        self.target_classes = [c.strip().lower() for c in target_str.split(",") if c.strip()]

        # ---------- ROS 话题名 ----------
        color_topic = rospy.get_param("~color_topic", "/vision/color/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/vision/aligned_depth/image_raw")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/vision/color/camera_info")

        # ---------- 加载模型 ----------
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

        # ---------- 数据容器 ----------
        self.bridge = CvBridge()
        self.depth_lock = threading.Lock()
        self.camera_info_lock = threading.Lock()
        self.latest_depth = None          # 最新深度图 (H×W float32, 单位 m)
        self.latest_camera_info = None    # 最新相机内参 (CameraInfo)

        # FPS 计时
        self._fps_t0 = rospy.Time.now()
        self._fps_counter = 0
        self._fps_display = 0.0

        # ---------- 发布 ----------
        self.result_pub = rospy.Publisher(
            "/vision/yolo/detection", YoloDetection, queue_size=1
        )
        self.results_pub = rospy.Publisher(
            "/vision/yolo/detections", YoloDetections, queue_size=1
        )
        self.annotated_pub = rospy.Publisher(
            "/vision/annotated_image", Image, queue_size=1
        )

        # ---------- 订阅 ----------
        self.color_sub = rospy.Subscriber(
            color_topic, Image, self.image_callback,
            queue_size=1, buff_size=2 ** 24
        )
        self.depth_sub = rospy.Subscriber(
            depth_topic, Image, self.depth_callback,
            queue_size=1, buff_size=2 ** 24
        )
        self.camera_info_sub = rospy.Subscriber(
            camera_info_topic, CameraInfo,
            self.camera_info_callback, queue_size=1
        )

        rospy.loginfo("Detector started. model=%s conf=%.2f imgsz=%d device=%s",
                       self.model_path, self.conf_threshold, self.imgsz, self.device)

    # ========================================================================
    # 回调：接收数据
    # ========================================================================

    def depth_callback(self, msg):
        """存储最新深度图（线程安全）"""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception:
            return
        with self.depth_lock:
            self.latest_depth = np.array(depth, dtype=np.float32, copy=True)

    def camera_info_callback(self, msg):
        """存储最新相机内参（线程安全）"""
        with self.camera_info_lock:
            self.latest_camera_info = msg

    # ========================================================================
    # 核心：图像回调 —— 整个检测流程的入口
    # ========================================================================

    def image_callback(self, msg):
        """
        每一帧彩色图到来时：
          1. 转 OpenCV 格式
          2. 跑 YOLO 推理
          3. 构建结果 + 类别过滤
          4. 画框 + 画文字标注
          5. 发布结果
          6. 可选弹窗
        """
        # 第 1 步：ROS Image → OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Image conversion failed: %s", exc)
            return

        # 第 2 步：YOLO 推理
        try:
            results = self.model(
                frame, conf=self.conf_threshold, imgsz=self.imgsz,
                device=self.device, verbose=False
            )
        except Exception as exc:
            rospy.logerr_throttle(2.0, "YOLO inference failed: %s", exc)
            return

        # 第 3 步：构建 ROS 消息 + 类别过滤
        result_msg, detections_msg, best = self.build_results(
            msg.header, results[0], frame.shape
        )

        # 第 4 步：画标注画面
        # ╔══════════════════════════════════════════════════════════╗
        # ║  绘制流程：先画框 → 再画最佳目标标注 → 再画底部面板  ║
        # ╚══════════════════════════════════════════════════════════╝
        annotated = frame.copy()

        # 4a. 给每个通过过滤的目标画黄色矩形框
        for det in detections_msg.detections:
            if not det.detected:
                continue
            cv2.rectangle(
                annotated,
                (det.x_min, det.y_min), (det.x_max, det.y_max),
                (0, 255, 255), 2   # 黄色 BGR=(0,255,255)
            )

        # 4b. 画画面中心的红色十字准心（标注相机光心位置）
        h, w = frame.shape[:2]
        cx_c, cy_c = w // 2, h // 2
        r = 7
        cv2.line(annotated, (cx_c - r, cy_c), (cx_c + r, cy_c), (0, 0, 255), 1)
        cv2.line(annotated, (cx_c, cy_c - r), (cx_c, cy_c + r), (cx_c, 0, 255), 1)

        # 4c. 画最佳目标的详细标注（见 draw_overlay）
        if best is not None:
            self.draw_overlay(annotated, best)

        # 4d. 左下角面板：帧摘要（不重复框旁边的信息）
        n_dets = len(detections_msg.detections)
        panel = [
            "detected: {}  FPS {:.1f}".format(n_dets, self._fps_display),
            "model: {}  device: {}".format(
                os.path.basename(self.model_path), self.device
            ),
        ]
        self.draw_panel(annotated, panel, frame.shape[1], frame.shape[0])

        # 4d. 左上角显示帧率
        self._fps_counter += 1
        now = rospy.Time.now()
        dt = (now - self._fps_t0).to_sec()
        if dt >= 1.0:
            self._fps_display = self._fps_counter / dt
            self._fps_counter = 0
            self._fps_t0 = now
        self.draw_text_bg(annotated, "FPS {:.1f}".format(self._fps_display),
                          4, 24, font_scale=0.50, text_color=(0, 255, 0))

        # 第 5 步：发布
        self.result_pub.publish(result_msg)
        self.results_pub.publish(detections_msg)
        # 强制连续化 —— OpenCV 画图操作可能破坏数组连续性，导致 cv_bridge 报 KeyError
        annotated = np.ascontiguousarray(annotated)
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        # 第 6 步：OpenCV 窗口
        if self.show_window:
            try:
                cv2.imshow(self.window_name, annotated)
                cv2.waitKey(1)
            except Exception:
                pass

    # ========================================================================
    # 深度查询
    # ========================================================================

    def lookup_depth(self, x, y):
        """
        查询 (x, y) 像素处的深度值（取周围小区域的中位数）
        返回：深度 (m)，无效则返回 0.0
        """
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
        """获取最新相机内参"""
        with self.camera_info_lock:
            return self.latest_camera_info

    # ========================================================================
    # 结果构建：YOLO 输出 → ROS 消息
    # ========================================================================

    def build_results(self, header, yolo_result, image_shape):
        """
        遍历 YOLO 推理结果中的每个检测框：
          1. 提取坐标、类别、置信度
          2. 类别过滤（_is_target_class）
          3. 查深度 → 反投影 → 填好 YoloDetection 消息
          4. 找出置信度最高的作为 best
        """
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

            # 类别过滤：不在此列表的直接丢弃
            if not self._is_target_class(det.class_name):
                continue

            detections_msg.detections.append(det)
            if det.confidence > best_conf:
                best_conf = det.confidence
                best_msg = det
                best = det
        return best_msg, detections_msg, best

    def _is_target_class(self, class_name):
        """检查类别名中是否包含目标关键词（模糊匹配）"""
        name = class_name.strip().lower()
        for kw in self.target_classes:
            if kw in name:
                return True
        return False

    def empty_detection(self, header):
        """构造一个空的（未检测到目标）的 YoloDetection"""
        det = YoloDetection()
        det.header = header
        det.detected = False
        det.class_id = -1
        det.class_name = ""
        det.confidence = 0.0
        det.x_min = det.y_min = det.x_max = det.y_max = -1
        det.center_x = det.center_y = -1
        det.depth_m = 0.0
        det.position_valid = False
        det.camera_x_m = det.camera_y_m = det.camera_z_m = 0.0
        det.distance_m = 0.0
        det.bbox_width_m = det.bbox_height_m = 0.0
        return det

    def build_single(self, header, yolo_result, box, image_shape):
        """
        从单个 YOLO box 构造一个 YoloDetection：
          - 提取 xyxy → 算中心像素
          - 查深度图 → 反投影 → 相机系 3D 坐标
        """
        det = self.empty_detection(header)

        # 框坐标
        xyxy = box.xyxy[0].detach().cpu().numpy()
        x_min, y_min, x_max, y_max = [int(round(v)) for v in xyxy]

        # 中心像素（限定在图像范围内）
        cx = int(round((x_min + x_max) * 0.5))
        cy = int(round((y_min + y_max) * 0.5))

        # 类别和置信度
        class_id = int(box.cls[0].detach().cpu().item())
        confidence = float(box.conf[0].detach().cpu().item())
        class_name = str(yolo_result.names.get(class_id, class_id))

        h, w = image_shape[:2]
        cx = max(0, min(w - 1, cx))
        cy = max(0, min(h - 1, cy))

        # 查深度 → 反投影
        depth_m = self.lookup_depth(cx, cy)
        position = self.project_pixel(cx, cy, depth_m)

        # 填字段
        det.detected = True
        det.class_id = class_id
        det.class_name = class_name
        det.confidence = confidence
        det.x_min, det.y_min = x_min, y_min
        det.x_max, det.y_max = x_max, y_max
        det.center_x, det.center_y = cx, cy
        det.depth_m = depth_m

        if position is not None:
            d = float(np.linalg.norm(position))
            det.position_valid = True
            det.camera_x_m = float(position[0])
            det.camera_y_m = float(position[1])
            det.camera_z_m = float(position[2])
            det.distance_m = d
            det.bbox_width_m = self.pixel_to_m(x_max - x_min, depth_m, "x")
            det.bbox_height_m = self.pixel_to_m(y_max - y_min, depth_m, "y")
        return det

    # ========================================================================
    # 坐标反投影：像素 + 深度 → 相机系 3D
    # ========================================================================

    def project_pixel(self, x, y, depth_m):
        """
        针孔相机模型反投影：
          X = (x - cx) * Z / fx
          Y = (y - cy) * Z / fy
          Z = depth

        invert_camera_x=True 时 X 取反（镜头朝下时用）
        """
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
        return np.array([
            sign * (float(x) - cx) * depth_m / fx,
            (float(y) - cy) * depth_m / fy,
            depth_m
        ], dtype=np.float32)

    def pixel_to_m(self, pixel_length, depth_m, axis="x"):
        """像素长度 → 实际米（用相似三角形）"""
        if depth_m <= 0.0 or not math.isfinite(depth_m):
            return 0.0
        ci = self.get_camera_info()
        if ci is None:
            return 0.0
        focal = float(ci.K[0] if axis == "x" else ci.K[4])
        return float(pixel_length) * depth_m / focal if focal > 0.0 else 0.0

    # ========================================================================
    #  绘制标注（文字显示在这里！）
    # ========================================================================

    def draw_overlay(self, image, det):
        """
        画最佳目标的标注（跟框走的浮动文字 + 圆心十字）：
          1. 圆心 + 十字线（红色）
          2. 黄色矩形框
          3. 框上方文字标签 ← draw_text_bg（预测框旁边的文字）
        （左下角面板由 image_callback 统一画，不在这里画）
        """

        # ---- ① 红色圆点（标注框中心） ----
        x, y = det.center_x, det.center_y
        cv2.circle(image, (x, y), 4, (0, 0, 255), -1)  # 实心红点

        # ---- ② 黄色矩形框 ----
        cv2.rectangle(
            image,
            (max(0, det.x_min), max(0, det.y_min)),
            (max(0, det.x_max), max(0, det.y_max)),
            (0, 255, 255), 2   # 黄色
        )

        # ---- ③ 框上方多行浮动标签 ----
        # 取框左上角作为基准位置
        bx = max(4, det.x_min)
        by = max(22, det.y_min - 8)
        line_h = 18  # 行高
        text_color = (0, 255, 255)  # 黄色
        font_scale = 0.45

        # 第 1 行：类别 + 置信度
        self.draw_text_bg(
            image, "{} {:.2f}".format(det.class_name, det.confidence),
            bx, by, font_scale=font_scale, text_color=text_color
        )

        if det.position_valid:
            # 第 2 行：水平 + 垂直偏移（相机系，带方向箭头）
            self.draw_text_bg(
                image, "x→{:+.2f}  y↑{:+.2f}m".format(det.camera_x_m, det.camera_y_m),
                bx, by + line_h, font_scale=font_scale, text_color=text_color
            )
            # 第 3 行：深度 + 直线距离
            self.draw_text_bg(
                image, "z={:.2f}  d={:.2f}m".format(det.camera_z_m, det.distance_m),
                bx, by + line_h * 2, font_scale=font_scale, text_color=text_color
            )
            # 第 4 行：框中心 vs 画面中心的像素差（带方向箭头）
            h_img, w_img = image.shape[:2]
            px_dx = det.center_x - w_img // 2
            px_dy = det.center_y - h_img // 2
            self.draw_text_bg(
                image, "px→{:+.0f}  py↑{:+.0f}".format(px_dx, px_dy),
                bx, by + line_h * 3, font_scale=font_scale, text_color=text_color
            )
        else:
            self.draw_text_bg(
                image, "depth none",
                bx, by + line_h, font_scale=font_scale, text_color=text_color
            )

    def draw_text_bg(self, image, text, x, y,
                     font_scale=0.55, text_color=(0, 255, 0), bg_color=(0, 0, 0)):
        """
        在指定位置画带黑底白字的文字。
        位置跟随 —— 用于预测框旁边的标签（"cylinder 0.85 x=... y=..."）。
        """
        h, w = image.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 2
        (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)

        # 防止文字超出画面
        x = max(0, min(w - tw - 8, int(x)))
        y = max(th + 6, min(h - baseline - 4, int(y)))

        # 黑底
        cv2.rectangle(
            image,
            (x - 4, y - th - 6),
            (x + tw + 4, y + baseline + 4),
            bg_color, -1
        )
        # 文字
        cv2.putText(
            image, text, (x, y), font, font_scale,
            text_color, thickness, cv2.LINE_AA
        )

    def draw_panel(self, image, lines, img_w, img_h):
        """
        在画面左下角画一个固定位置的信息面板。
        位置固定 —— 用于始终显示最佳目标摘要（"Best: cylinder conf=0.85..."）。
        不管检测框在哪，面板始终在左下角。
        """
        font = cv2.FONT_HERSHEY_SIMPLEX
        fs, thickness, pad, gap = 0.58, 2, 8, 7

        # 计算面板尺寸
        sizes = [cv2.getTextSize(l, font, fs, thickness)[0] for l in lines]
        pw = min(img_w - 8, max(s[0] for s in sizes) + pad * 2)   # 面板宽度
        lh = max(s[1] for s in sizes)                               # 行高
        ph = len(lines) * lh + (len(lines) - 1) * gap + pad * 2     # 面板高度

        # 面板定位：左下角
        x0, y0 = 4, max(4, img_h - ph - 4)

        # 黑底
        cv2.rectangle(image, (x0, y0), (x0 + pw, y0 + ph), (0, 0, 0), -1)
        # 黄色边框
        cv2.rectangle(image, (x0, y0), (x0 + pw, y0 + ph), (0, 255, 255), 1)

        # 逐行写入文字
        y = y0 + pad + lh
        for line in lines:
            cv2.putText(
                image, line, (x0 + pad, y), font, fs,
                (0, 255, 255), thickness, cv2.LINE_AA
            )
            y += lh + gap


# ========================================================================
# 启动入口
# ========================================================================

def main():
    rospy.init_node("detector_node")
    DetectorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
