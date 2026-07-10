#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
detector_node.py — CUADC 2026 YOLO 目标检测节点

功能：
  1. 自动检测并启动 roscore / MAVROS（如未运行）
  2. 订阅相机图像话题，加载 YOLOv8 模型逐帧推理
  3. 按类别关键词过滤（只保留圆筒相关目标）
  4. 查深度图获取目标距离，反投影为相机系 3D 坐标
  5. 读取 MAVROS 飞控数据（连接状态、电压、卫星数、RTK GPS）
  6. 发布检测结果 + 飞控数据 + 标注画面
  7. 可选弹出 OpenCV 窗口实时显示（含飞控状态面板）

订阅话题：
  - 彩色图像 (sensor_msgs/Image)
  - 对齐深度图 (sensor_msgs/Image)
  - 相机内参 (sensor_msgs/CameraInfo)
  - 飞控状态 (mavros_msgs/State)
  - 电池状态 (sensor_msgs/BatteryState)
  - GPS 原始数据 (mavros_msgs/GPSRAW) — 卫星数 + RTK 状态
  - GPS 全局位置 (sensor_msgs/NavSatFix) — 厘米级大地坐标

发布话题：
  - /vision/yolo/detection  (YoloDetection) — 最高置信度目标
  - /vision/yolo/detections (YoloDetections) — 全部目标
  - /vision/bucket/info     (BucketInfo) — 目标数量 + 像素偏差
  - /vision/annotated_image (sensor_msgs/Image) — 标注画面
"""

import math
import os
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rospy
import rospkg
from cv_bridge import CvBridge
from sensor_msgs.msg import BatteryState, CameraInfo, Image, NavSatFix

from cuadc_vision.msg import YoloDetection, YoloDetections, BucketInfo, MissionStatus

# MAVROS 消息类型 — 可选依赖，导入失败时不阻塞
try:
    from mavros_msgs.msg import GPSRAW, State
    _HAS_MAVROS_MSGS = True
except ImportError:
    GPSRAW = None   # type: ignore
    State = None    # type: ignore
    _HAS_MAVROS_MSGS = False


def get_bool_param(name, default):
    """解析 ROS 参数为 bool（兼容字符串 "true"/"1" 等写法）"""
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


# ========================================================================
#  环境自检与自动启动 (roscore / MAVROS)
# ========================================================================

def _ros_master_is_alive():
    """检查 ROS master 是否可达"""
    try:
        import xmlrpc.client
        master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        master = xmlrpc.client.ServerProxy(master_uri)
        master.getPid('/detector_startup_check')
        return True
    except Exception:
        return False


def _start_roscore():
    """后台启动 roscore"""
    print("[auto-start] 正在启动 roscore ...")
    try:
        subprocess.Popen(
            ['roscore'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except Exception as exc:
        print("[auto-start] 启动 roscore 失败: {}".format(exc))
        sys.exit(1)


def _wait_for_ros_master(timeout=30):
    """等待 ROS master 上线，返回是否成功"""
    waited = 0.0
    while not _ros_master_is_alive():
        if waited >= timeout:
            return False
        time.sleep(1.0)
        waited += 1.0
    return True


def _mavros_is_alive():
    """检查 MAVROS 是否在运行（通过检查 /mavros/state 话题是否存在）"""
    try:
        return any(t[0] == '/mavros/state' for t in rospy.get_published_topics())
    except Exception:
        return False


def _wait_for_mavros(timeout=30):
    """等待 MAVROS 上线，返回是否成功"""
    waited = 0.0
    while not _mavros_is_alive():
        if waited >= timeout:
            return False
        time.sleep(1.0)
        waited += 1.0
    return True


def _ensure_roscore():
    """确保 roscore 在运行，如果不在则自动启动"""
    if _ros_master_is_alive():
        return
    print("[auto-start] ROS master 未运行，自动启动 roscore ...")
    _start_roscore()
    if not _wait_for_ros_master():
        print("[auto-start] 错误：roscore 启动超时（30s），请手动检查")
        sys.exit(1)
    print("[auto-start] roscore 已就绪")


def _ensure_mavros(fcu_url):
    """确保 MAVROS 在运行，如果不在则自动启动"""
    if _mavros_is_alive():
        rospy.loginfo("[auto-start] MAVROS 已在运行")
        return
    rospy.loginfo("[auto-start] MAVROS 未运行，自动启动 mavros apm.launch ...")
    try:
        subprocess.Popen(
            ['roslaunch', 'mavros', 'apm.launch',
             'fcu_url:={}'.format(fcu_url)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except Exception as exc:
        rospy.logerr("[auto-start] 启动 MAVROS 失败: %s", exc)
        return
    if _wait_for_mavros():
        rospy.loginfo("[auto-start] MAVROS 已就绪")
    else:
        rospy.logwarn("[auto-start] MAVROS 启动超时（30s），飞控数据将不可用")


# ========================================================================
#  DetectorNode
# ========================================================================

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
        # 使用标准相机光学坐标系：X+ = 右，Y+ = 下，Z+ = 光轴前方
        # 相机→机体的旋转由 TF 树和 geopose_node 负责

        # ---------- 深度 ----------
        self.depth_patch_radius = int(rospy.get_param("~depth_patch_radius", 2))

        # ---------- 显示 ----------
        self.show_window = get_bool_param("~show_window", False)
        self.window_name = rospy.get_param("~window_name", "YOLO Detection")

        # ---------- 类别过滤 ----------
        # 只保留名称包含这些关键词的目标（逗号分隔）
        target_str = rospy.get_param(
            "~target_classes",
            "cylinder,tong,barrel,can,yuantong,white_cylinder,tube,drum,bucket"
        )
        self.target_classes = [c.strip().lower() for c in target_str.split(",") if c.strip()]

        # ---------- 飞控 MAVROS ----------
        self.auto_start_mavros = get_bool_param("~auto_start_mavros", True)
        self.mavros_fcu_url = rospy.get_param("~mavros_fcu_url", "/dev/ttyACM0:921600")
        self.fc_state_topic = rospy.get_param("~fc_state_topic", "/mavros/state")
        self.fc_battery_topic = rospy.get_param("~fc_battery_topic", "/mavros/battery")
        self.fc_gps_topic = rospy.get_param("~fc_gps_topic", "/mavros/global_position/global")
        self.fc_gpsraw_topic = rospy.get_param("~fc_gpsraw_topic", "/mavros/gpsstatus/gps1/raw")

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

        # ---- 飞控数据容器 ----
        self.fc_lock = threading.Lock()
        self.fc_connected = False         # 飞控是否连接
        self.fc_armed = False             # 是否解锁
        self.fc_mode = "N/A"              # 飞行模式
        self.fc_voltage = 0.0             # 电池电压 (V)
        self.fc_satellites = 0            # 卫星数
        self.fc_fix_type = 0              # GPS 定位类型 (0=无, 6=RTK Fixed)
        self.fc_lat = 0.0                 # 纬度 (度)
        self.fc_lon = 0.0                 # 经度 (度)
        self.fc_alt = 0.0                 # 海拔 (m)
        self._has_fc_data = False         # 是否收到过飞控数据

        # ---- 任务状态（来自 main.py）----
        self.mission_lock = threading.Lock()
        self.mission_ammo_a = 0
        self.mission_ammo_b = 0
        self.mission_aiming = False
        self._drop_display_until = 0.0   # 抛投文字显示截止时间
        self._drop_display_label = ""    # 当前显示的抛投标签 ("A" / "B")

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
        self.bucket_info_pub = rospy.Publisher(
            "/vision/bucket/info", BucketInfo, queue_size=1
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

        # ---- 飞控 MAVROS 订阅（可选，失败时静默跳过）----
        self._init_fc_subscribers()

        # ---- 任务状态订阅（来自 main.py）----
        self.mission_sub = rospy.Subscriber(
            "/vision/mission_status", MissionStatus, self._mission_cb, queue_size=5
        )

        rospy.loginfo("Detector started. model=%s conf=%.2f imgsz=%d device=%s",
                       self.model_path, self.conf_threshold, self.imgsz, self.device)
        if self._has_fc_data:
            rospy.loginfo("FC data available: state=%s battery=%s gps=%s gpsraw=%s",
                          self.fc_state_topic, self.fc_battery_topic,
                          self.fc_gps_topic, self.fc_gpsraw_topic)
        else:
            rospy.loginfo("FC data NOT available (MAVROS not running or mavros_msgs not installed)")

    def _init_fc_subscribers(self):
        """初始化飞控数据订阅（容错：任一 topic 不可用不影响整体）"""
        if not _HAS_MAVROS_MSGS:
            rospy.loginfo("mavros_msgs not installed, skipping FC data subscriptions")
            return
        try:
            self.fc_state_sub = rospy.Subscriber(
                self.fc_state_topic, State, self.fc_state_cb, queue_size=1
            )
            self.fc_battery_sub = rospy.Subscriber(
                self.fc_battery_topic, BatteryState, self.fc_battery_cb, queue_size=1
            )
            self.fc_gpsraw_sub = rospy.Subscriber(
                self.fc_gpsraw_topic, GPSRAW, self.fc_gpsraw_cb, queue_size=1
            )
            self.fc_gps_sub = rospy.Subscriber(
                self.fc_gps_topic, NavSatFix, self.fc_gps_cb, queue_size=1
            )
            self._has_fc_data = True
        except Exception as exc:
            rospy.logwarn("FC subscriber init failed: %s. FC data will be N/A.", exc)
            self._has_fc_data = False

    @staticmethod
    def _make_img_msg(array, encoding):
        """Construct sensor_msgs/Image directly to avoid cv_bridge bgr8 KeyError."""
        if not array.flags["C_CONTIGUOUS"]:
            array = np.ascontiguousarray(array)
        msg = Image()
        msg.height = array.shape[0]
        msg.width = array.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(array.shape[1] * array.shape[2] * array.dtype.itemsize)
        msg.data = array.tobytes()
        return msg

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
    # 飞控 MAVROS 回调
    # ========================================================================

    def fc_state_cb(self, msg):
        """飞控状态回调"""
        with self.fc_lock:
            self.fc_connected = getattr(msg, 'connected', False)
            self.fc_armed = getattr(msg, 'armed', False)
            self.fc_mode = getattr(msg, 'mode', 'N/A')

    def fc_battery_cb(self, msg):
        """电池状态回调"""
        with self.fc_lock:
            self.fc_voltage = getattr(msg, 'voltage', 0.0)

    def fc_gpsraw_cb(self, msg):
        """GPS 原始数据回调（卫星数 + RTK 定位类型）"""
        with self.fc_lock:
            self.fc_satellites = getattr(msg, 'satellites_visible', 0)
            self.fc_fix_type = getattr(msg, 'fix_type', 0)

    def fc_gps_cb(self, msg):
        """GPS 全局位置回调（RTK 厘米级大地坐标）"""
        with self.fc_lock:
            self.fc_lat = getattr(msg, 'latitude', 0.0)
            self.fc_lon = getattr(msg, 'longitude', 0.0)
            self.fc_alt = getattr(msg, 'altitude', 0.0)

    def _mission_cb(self, msg):
        """任务状态回调（来自 main.py）"""
        with self.mission_lock:
            self.mission_ammo_a = msg.ammo_a
            self.mission_ammo_b = msg.ammo_b
            self.mission_aiming = msg.aiming
            if msg.last_drop:
                # 新的抛投事件 → 记录 3 秒显示
                self._drop_display_label = msg.last_drop
                self._drop_display_until = time.time() + 3.0
            elif not msg.last_drop:
                # 抛投事件结束
                if time.time() >= self._drop_display_until:
                    self._drop_display_label = ""

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

        # 4b. 画画面中心的坐标系（x/y 轴带箭头和字母）
        h, w = frame.shape[:2]
        self.draw_center_axes(annotated, w // 2, h // 2)

        # 4c. 画最佳目标的详细标注（见 draw_overlay）
        if best is not None:
            self.draw_overlay(annotated, best)

        # 4c2. 任务状态叠加文字 (AIMING!! / DROP!!!)
        self._draw_mission_overlay(annotated, w, h)

        # 4d. 左下角面板：检测摘要 + 飞控状态
        n_dets = len(detections_msg.detections)
        fc_lines = self._format_fc_lines()
        panel = [
            "detected: {}  FPS {:.1f}".format(n_dets, self._fps_display),
        ] + fc_lines + [
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
        annotated_msg = self._make_img_msg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        # 5b. 发布桶信息（数量 + 最佳目标的像素偏差）
        binfo = BucketInfo()
        binfo.header = msg.header
        binfo.count = len(detections_msg.detections)
        if best is not None:
            h_img, w_img = frame.shape[:2]
            binfo.delta_x = float(best.center_x - w_img // 2)
            binfo.delta_y = float(best.center_y - h_img // 2)
        else:
            binfo.delta_x = 0.0
            binfo.delta_y = 0.0
        self.bucket_info_pub.publish(binfo)

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
        filtered_class_names = set()
        for box in boxes:
            det = self.build_single(header, yolo_result, box, image_shape)

            # 类别过滤：不在此列表的直接丢弃
            if not self._is_target_class(det.class_name):
                filtered_class_names.add(det.class_name)
                continue

            detections_msg.detections.append(det)
            if det.confidence > best_conf:
                best_conf = det.confidence
                best_msg = det
                best = det

        # 有检测但全部被过滤 → 打印警告，方便排查类别名不匹配
        if filtered_class_names and not detections_msg.detections:
            rospy.logwarn_throttle(
                5.0,
                "YOLO detected %d object(s) but all filtered out. "
                "Detected classes: %s. Target keywords: %s",
                len(boxes), list(filtered_class_names), self.target_classes
            )

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
        针孔相机模型反投影（标准相机光学坐标系）：
          相机光学坐标系：X+ = 右，Y+ = 下，Z+ = 光轴前方

          X = (x - cx) * Z / fx
          Y = (y - cy) * Z / fy
          Z = depth

        OpenCV 图像像素坐标：u 向右增大，v 向下增大。
        因此画面右侧目标 X > 0，画面下方目标 Y > 0。
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
        return np.array([
            (float(x) - cx) * depth_m / fx,
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
                image, "x→{:+.2f}  y↓{:+.2f}m".format(det.camera_x_m, det.camera_y_m),
                bx, by + line_h, font_scale=font_scale, text_color=text_color
            )
            # 第 3 行：深度 + 直线距离
            self.draw_text_bg(
                image, "z={:.2f}  d={:.2f}m".format(det.camera_z_m, det.distance_m),
                bx, by + line_h * 2, font_scale=font_scale, text_color=text_color
            )
            # 第 4 行：框中心 vs 画面中心的像素差（Δx, Δy）
            h_img, w_img = image.shape[:2]
            px_dx = det.center_x - w_img // 2
            px_dy = det.center_y - h_img // 2
            self.draw_delta_label(
                image, "x", px_dx,
                bx, by + line_h * 3, font_scale=font_scale, text_color=text_color
            )
            self.draw_delta_label(
                image, "y", px_dy,
                bx + 90, by + line_h * 3, font_scale=font_scale, text_color=text_color
            )
        else:
            self.draw_text_bg(
                image, "depth none",
                bx, by + line_h, font_scale=font_scale, text_color=text_color
            )

    def draw_center_axes(self, image, cx, cy, length=45):
        """
        在画面中心画标准相机光学坐标系：
          x 轴（红色）指向右方 (X+)，末端有箭头和字母 "x"
          y 轴（绿色）指向下方 (Y+)，末端有箭头和字母 "y"
          符合标准相机光学坐标系：X+ = 右，Y+ = 下，Z+ = 光轴前方
        """
        # x 轴 — 红色，向右 (X+)
        end_x = cx + length
        cv2.arrowedLine(image, (cx, cy), (end_x, cy), (0, 0, 255), 1,
                        tipLength=0.25)
        cv2.putText(image, "x", (end_x + 4, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

        # y 轴 — 绿色，向下 (Y+)
        end_y = cy + length
        cv2.arrowedLine(image, (cx, cy), (cx, end_y), (0, 255, 0), 1,
                        tipLength=0.25)
        cv2.putText(image, "y", (cx + 5, end_y + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

        # 原点小圆点
        cv2.circle(image, (cx, cy), 2, (255, 255, 255), -1)

    def draw_delta_label(self, image, letter, value, x, y,
                         font_scale=0.45, text_color=(0, 255, 255)):
        """
        画 "Δ{letter} {value:+}" 样式的标签。
        由于 OpenCV Hershey 字体不支持 Δ 字符，这里手绘一个小三角。
        """
        text = "{}{:+.0f}".format(letter, value)
        font = cv2.FONT_HERSHEY_SIMPLEX
        thickness = 2
        (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)

        # 三角占位宽度
        tri_w = 10
        total_w = tri_w + 4 + tw + 4

        h, w_img = image.shape[:2]
        x0 = max(0, min(w_img - total_w, x))
        y0 = max(th + 6, min(h - baseline - 4, y))

        # 黑底
        cv2.rectangle(
            image,
            (x0, y0 - th - 6),
            (x0 + total_w, y0 + baseline + 4),
            (0, 0, 0), -1
        )

        # 手绘 delta 三角（Δ）
        tri_cx = x0 + 4 + tri_w // 2
        tri_top = y0 - th // 2 - 2
        tri_bot = y0 - th // 2 + 7
        pts = np.array([
            [tri_cx, tri_top],             # 顶点
            [tri_cx - 4, tri_bot],         # 左下
            [tri_cx + 4, tri_bot],         # 右下
        ], np.int32)
        cv2.fillPoly(image, [pts], text_color)

        # 文字（字母 + 数值）
        cv2.putText(
            image, text, (x0 + tri_w + 4, y0), font, font_scale,
            text_color, thickness, cv2.LINE_AA
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

    # ========================================================================
    #  任务状态叠加文字 (AIMING!! / DROP!!!)
    # ========================================================================

    def _draw_mission_overlay(self, image, img_w, img_h):
        """在画面中下区域叠加 AIMING!! 闪烁或 DROP!!! 文字"""
        now = time.time()

        with self.mission_lock:
            aiming = self.mission_aiming
        drop_label = self._drop_display_label
        drop_active = drop_label and now < self._drop_display_until

        if drop_active:
            # 抛投显示：大号红色闪烁文字
            text = "{} DROP!!!".format(drop_label)
            font = cv2.FONT_HERSHEY_SIMPLEX
            fs = 1.6
            thickness = 3
            color = (0, 0, 255)  # 红色
        elif aiming:
            # 瞄准显示：闪烁效果（0.5 Hz toggle）
            if int(now * 2) % 2 == 0:
                text = "AIMING!!"
            else:
                text = ""
            font = cv2.FONT_HERSHEY_SIMPLEX
            fs = 1.3
            thickness = 3
            color = (0, 255, 255)  # 黄色
        else:
            return

        if not text:
            return

        # 居中在中下区域（画面中心偏下 60px）
        (tw, th), baseline = cv2.getTextSize(text, font, fs, thickness)
        cx = (img_w - tw) // 2
        cy = img_h // 2 + 60

        # 黑底背景
        pad = 8
        cv2.rectangle(
            image,
            (cx - pad, cy - th - pad),
            (cx + tw + pad, cy + baseline + pad),
            (0, 0, 0), -1,
        )
        cv2.putText(
            image, text, (cx, cy), font, fs,
            color, thickness, cv2.LINE_AA,
        )

    # ========================================================================
    #  飞控状态格式化
    # ========================================================================

    @staticmethod
    def _gps_fix_name(fix_type):
        """GPS 定位类型 → 可读字符串"""
        names = {
            0: "NoGPS", 1: "NoFix", 2: "2D", 3: "3D",
            4: "DGPS", 5: "RTK Float", 6: "RTK Fixed", 7: "Static", 8: "PPP",
        }
        return names.get(fix_type, "?{}".format(fix_type))

    def _format_fc_lines(self):
        """构建左下角面板中的飞控状态文本行"""
        with self.fc_lock:
            connected = self.fc_connected
            armed = self.fc_armed
            mode = self.fc_mode
            voltage = self.fc_voltage
            satellites = self.fc_satellites
            fix_type = self.fc_fix_type
            lat = self.fc_lat
            lon = self.fc_lon
            alt = self.fc_alt

        with self.mission_lock:
            ammo_a = self.mission_ammo_a
            ammo_b = self.mission_ammo_b

        # 弹药行
        ammo_a_str = str(ammo_a) if ammo_a > 0 else "N/A"
        ammo_b_str = str(ammo_b) if ammo_b > 0 else "N/A"
        ammo_line = "AMMO: A-{}  B-{}".format(ammo_a_str, ammo_b_str)

        if not self._has_fc_data or not connected:
            # 飞控未连接：显示简化信息
            return [
                "FC: disconnected",
                ammo_line,
            ]

        # 飞控已连接：显示完整信息
        arm_str = "ARM" if armed else "DISARM"
        fix_str = self._gps_fix_name(fix_type)

        lines = [
            "FC: connected  {}  {}".format(mode, arm_str),
        ]

        # 电压 + 卫星数 + RTK 状态
        if voltage > 0.0:
            lines.append(
                "Bat: {:.1f}V  Sat: {}  {}".format(voltage, satellites, fix_str)
            )
        else:
            lines.append("Sat: {}  {}".format(satellites, fix_str))

        # GPS 厘米级大地坐标
        if lat != 0.0 or lon != 0.0:
            lines.append(
                "GPS: {:.7f}  {:.7f}  {:.2f}m".format(lat, lon, alt)
            )
        else:
            lines.append("GPS: waiting for fix...")

        # 弹药
        lines.append(ammo_line)

        return lines

    # ========================================================================
    #  面板绘制
    # ========================================================================

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
    # ---- 第 0 步：环境自检 ----
    # 如果通过 roslaunch 启动，roscore 已在运行。
    # 如果直接 python3 启动，自动检测并启动 roscore。
    _ensure_roscore()

    # ---- 第 1 步：初始化节点 ----
    rospy.init_node("detector_node")

    # ---- 第 2 步：按需启动 MAVROS ----
    auto_start = get_bool_param("~auto_start_mavros", True)
    if auto_start:
        fcu_url = rospy.get_param("~mavros_fcu_url", "/dev/ttyACM0:921600")
        _ensure_mavros(fcu_url)

    # ---- 第 3 步：创建检测节点 ----
    DetectorNode()

    # ---- 第 4 步：事件循环 ----
    rospy.spin()


if __name__ == "__main__":
    main()
