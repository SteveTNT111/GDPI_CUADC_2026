#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
detector_node.py — CUADC 2026 YOLO 目标检测节点（全功能版）

功能：
  1. 自动检测并启动 roscore / MAVROS（如未运行）
  2. 订阅相机图像话题，加载 YOLOv8 模型逐帧推理（支持 .pt 和 .onnx）
  3. 按类别关键词过滤（只保留圆筒相关目标）
  4. 查深度图获取目标距离，反投影为相机系 3D 坐标
  5. 读取 MAVROS 飞控数据（连接状态、电压、卫星数、RTK GPS、Z 高度）
  6. 监听 main.py 任务状态（弹药余量、瞄准、抛投事件）
  7. 内联相机→机体→ENU→WGS84 变换，计算最佳目标 NED / WGS84
  8. 发布检测结果 + 飞控数据 + 标注画面（可选发布 GeoTarget）
  9. 可选弹出 OpenCV 窗口实时显示（含飞控状态面板 + 底部坐标栏）

订阅话题：
  - 彩色图像 (sensor_msgs/Image)
  - 对齐深度图 (sensor_msgs/Image)
  - 相机内参 (sensor_msgs/CameraInfo)
  - 飞控状态 (mavros_msgs/State)
  - 电池状态 (sensor_msgs/BatteryState)
  - GPS 原始数据 (mavros_msgs/GPSRAW) — 卫星数 + RTK 状态
  - GPS 全局位置 (sensor_msgs/NavSatFix) — 厘米级大地坐标
  - 本地位置 (geometry_msgs/PoseStamped) — Z 高度
  - 任务状态 (cuadc_vision/MissionStatus) — 弹药 + 瞄准 + 抛投

发布话题：
  - /vision/yolo/detection  (YoloDetection) — 最高置信度目标
  - /vision/yolo/detections (YoloDetections) — 全部目标
  - /vision/bucket/info     (BucketInfo) — 目标数量 + 像素偏差
  - /vision/bucket/aim_info (BucketAimInfo) — A/B 真实抛投瞄准点 + NED
  - /vision/annotated_image (sensor_msgs/Image) — 标注画面
  - /vision/target_global   (GeoTarget, 可选) — 最佳目标大地坐标
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
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import BatteryState, CameraInfo, Image, NavSatFix, NavSatStatus

from cuadc_vision.msg import (
    BucketAimInfo,
    BucketInfo,
    GeoTarget,
    MissionStatus,
    YoloDetection,
    YoloDetections,
)

try:
    import tf2_ros
    _HAS_TF2 = True
except ImportError:
    tf2_ros = None  # type: ignore
    _HAS_TF2 = False

# MAVROS 消息类型 — 可选依赖，导入失败时不阻塞
try:
    from mavros_msgs.msg import GPSRAW, State
    from mavros_msgs.srv import MessageInterval
    _HAS_MAVROS_MSGS = True
except ImportError:
    GPSRAW = None   # type: ignore
    State = None    # type: ignore
    MessageInterval = None  # type: ignore
    _HAS_MAVROS_MSGS = False


def get_bool_param(name, default):
    """解析 ROS 参数为 bool（兼容字符串 "true"/"1" 等写法）"""
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def quaternion_rotate_vector(q, vector):
    """用四元数将机体系向量旋转到 ENU。"""
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def frd_to_local_pose_axes(vector, right_sign=1.0):
    """把视觉机体 FRD 向量转换为本机 MAVROS 位姿旋转所用的轴。

    2026-07-19 真机日志确认：本机链路的横向输入必须保持“右为正”；
    若取标准 FLU 的负号，桶在右侧时飞机反而向左移动。竖直方向仍需
    将“下为正”转换为“上为正”，否则会重新引入高度正反馈。
    ``right_sign`` 保留为现场标定参数，默认值 +1.0 是本机实测值。
    """
    forward, right, down = [float(value) for value in vector]
    right_sign = float(right_sign)
    if not math.isfinite(right_sign) or abs(right_sign) < 1e-9:
        raise ValueError("right_sign 必须是非零有限数")
    return forward, right_sign * right, -down


def enu_offset_to_geodetic(origin_lat, origin_lon, origin_alt, east_m, north_m, up_m):
    """ENU 偏移转换到 WGS84 经纬高。"""
    try:
        from geographiclib.geodesic import Geodesic
        horizontal_m = math.hypot(east_m, north_m)
        if horizontal_m > 0.0:
            azimuth_deg = math.degrees(math.atan2(east_m, north_m))
            result = Geodesic.WGS84.Direct(origin_lat, origin_lon, azimuth_deg, horizontal_m)
            return result["lat2"], result["lon2"], origin_alt + up_m
        return origin_lat, origin_lon, origin_alt + up_m
    except Exception:
        return enu_offset_to_geodetic_local(
            origin_lat, origin_lon, origin_alt, east_m, north_m, up_m
        )


def enu_offset_to_geodetic_local(origin_lat, origin_lon, origin_alt, east_m, north_m, up_m):
    """无 geographiclib 时的本地近似回退。"""
    lat_rad = math.radians(origin_lat)
    semi_major = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_sq = flattening * (2.0 - flattening)
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
    prime_vertical_radius = semi_major / denom
    meridian_radius = semi_major * (1.0 - eccentricity_sq) / (denom ** 3)
    d_lat = north_m / (meridian_radius + origin_alt)
    cos_lat = max(1e-12, math.cos(lat_rad))
    d_lon = east_m / ((prime_vertical_radius + origin_alt) * cos_lat)
    return (
        math.degrees(lat_rad + d_lat),
        math.degrees(math.radians(origin_lon) + d_lon),
        origin_alt + up_m,
    )


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
    """YOLO 目标检测 ROS 节点（全功能版）"""

    # ========================================================================
    # 初始化
    # ========================================================================

    def __init__(self):
        # ---------- 模型参数 ----------
        # 默认从包内 models/ 目录找 best.pt（也支持 .onnx）
        _default_model = os.path.join(rospkg.RosPack().get_path('cuadc_vision'), 'models', 'best.pt')
        self.model_path = os.path.expanduser(rospy.get_param("~model_path", _default_model))
        self.conf_threshold = float(rospy.get_param("~conf_threshold", 0.5))
        self.imgsz = int(rospy.get_param("~imgsz", 640))
        self.imgsz_explicit = rospy.has_param("~imgsz")
        self.device = rospy.get_param("~device", "cpu")
        self.model_ext = os.path.splitext(self.model_path)[1].lower()

        # ---------- 坐标 ----------
        self.invert_camera_x = get_bool_param("~invert_camera_x", True)
        self.local_pose_right_sign = float(
            rospy.get_param("~local_pose_right_sign", 1.0)
        )
        if (
            not math.isfinite(self.local_pose_right_sign)
            or abs(self.local_pose_right_sign) < 1e-9
        ):
            rospy.logwarn(
                "Invalid ~local_pose_right_sign=%s; using flight-calibrated +1.0",
                self.local_pose_right_sign,
            )
            self.local_pose_right_sign = 1.0
        # A 抛投器在相机前方：使用 A 时飞机需向后瞄，因此 A 虚拟瞄准点
        # 位于桶中心下方；B 点位于桶中心上方。运行时按深度和 fy 把
        # aim_offset_m 换算成像素，绝不把 5 cm 写死为固定像素。
        self.aim_offset_m = float(rospy.get_param("~aim_offset_m", 0.05))
        if not math.isfinite(self.aim_offset_m) or self.aim_offset_m < 0.0:
            rospy.logwarn("Invalid ~aim_offset_m=%s; using 0.05 m", self.aim_offset_m)
            self.aim_offset_m = 0.05

        # ---------- 深度 ----------
        self.depth_patch_radius = int(rospy.get_param("~depth_patch_radius", 5))

        # ---------- 显示 ----------
        self.show_window = get_bool_param("~show_window", False)
        self.window_name = rospy.get_param("~window_name", "YOLO Detection")
        self.warmup = get_bool_param("~warmup", True)
        self.cv_threads = int(rospy.get_param("~cv_threads", 1))
        class_names_str = rospy.get_param("~class_names", "").strip()
        self.class_name_overrides = [x.strip() for x in class_names_str.split(",") if x.strip()]
        self.class_name_map = {}

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
        self.fc_local_pose_topic = rospy.get_param("~fc_local_pose_topic", "/mavros/local_position/pose")
        self.output_topic = rospy.get_param("~output_topic", "/vision/target_global")
        self.body_frame = rospy.get_param("~body_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "d435i_color_optical_frame")
        self.geo_min_confidence = float(rospy.get_param("~min_confidence", 0.30))
        self.transform_timeout_sec = float(rospy.get_param("~transform_timeout_sec", 0.10))
        self.publish_invalid = get_bool_param("~publish_invalid", True)
        self.publish_geo_target = get_bool_param("~publish_geo_target", False)
        self.geo_result_timeout_sec = float(rospy.get_param("~geo_result_timeout_sec", 0.50))

        # ---------- ROS 话题名 ----------
        color_topic = rospy.get_param("~color_topic", "/vision/color/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/vision/aligned_depth/image_raw")
        camera_info_topic = rospy.get_param("~camera_info_topic", "/vision/color/camera_info")

        # ---------- 加载模型 ----------
        if not os.path.isfile(self.model_path):
            rospy.logerr("YOLO model not found: %s", self.model_path)
            rospy.signal_shutdown("Model not found")
            return

        # Offline environments should not let Ultralytics try package auto-install.
        os.environ.setdefault("ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS", "1")
        try:
            cv2.setNumThreads(max(1, self.cv_threads))
        except Exception:
            pass

        try:
            from ultralytics import YOLO
        except Exception as exc:
            rospy.logerr("Failed to import ultralytics: %s. Run: pip3 install ultralytics", exc)
            rospy.signal_shutdown("ultralytics not installed")
            return

        # ONNX 模型：检查 onnxruntime + 同步输入尺寸
        if self.model_ext == ".onnx":
            try:
                import onnxruntime  # noqa: F401
            except Exception as exc:
                rospy.logerr(
                    "ONNX model requires onnxruntime: %s. Run: pip3 install onnxruntime",
                    exc,
                )
                rospy.signal_shutdown("onnxruntime not installed")
                return
            self._sync_imgsz_with_onnx_input()

        try:
            self.model = YOLO(self.model_path, task="detect")
        except Exception as exc:
            rospy.logerr("Failed to load YOLO model %s: %s", self.model_path, exc)
            if self.model_ext == ".onnx":
                rospy.logerr("For ONNX models, confirm onnxruntime is installed and the file is valid.")
            rospy.signal_shutdown("model load failed")
            return

        self._build_class_name_map()

        if self.warmup:
            self._warmup_model()

        # ---------- 数据容器 ----------
        self.bridge = CvBridge()
        self.depth_lock = threading.Lock()
        self.camera_info_lock = threading.Lock()
        self.latest_depth = None          # 最新深度图 (H×W float32, 单位 m)
        self.latest_camera_info = None    # 最新相机内参 (CameraInfo)

        # ---- 最佳桶中心点平滑 ----
        # 保存最近 5 帧检测中心；连续 10 帧无目标时丢弃旧轨迹。
        self.center_buffer = []
        self.center_buffer_size = 5
        self.no_detection_frames = 0
        self.no_detection_clear_frames = 10

        # ---- 飞控数据容器 ----
        self.fc_lock = threading.Lock()
        self.fc_connected = False         # 飞控是否连接
        self.fc_armed = False             # 是否解锁
        self.fc_mode = "N/A"              # 飞行模式
        self.fc_voltage = 0.0             # 电池电压 (V)
        self.fc_current = 0.0             # 电池电流 (A)，负值=放电
        self.fc_satellites = 0            # 卫星数
        self.fc_fix_type = 0              # GPS 定位类型 (0=无, 6=RTK Fixed)
        self.fc_lat = 0.0                 # 纬度 (度)
        self.fc_lon = 0.0                 # 经度 (度)
        self.fc_alt = 0.0                 # GPS 海拔 (m)
        self.fc_rel_z = 0.0               # 本地 Z 高度 (m) — NED 坐标系，Z 向下为负
        self.fc_local_east = 0.0          # 本地 ENU East (m)
        self.fc_local_north = 0.0         # 本地 ENU North (m)
        self.fc_local_up = 0.0            # 本地 ENU Up (m)
        self._has_fc_data = False         # 是否收到过飞控数据
        self.latest_global_msg = None     # 最新 NavSatFix
        self.latest_local_pose_msg = None # 最新 PoseStamped

        # ---- 任务状态（来自 main.py）----
        self.mission_lock = threading.Lock()
        self.mission_ammo_a = 0
        self.mission_ammo_b = 0
        self.mission_aiming = False
        self._drop_display_until = 0.0   # 抛投文字显示截止时间
        self._drop_display_label = ""    # 当前显示的抛投标签 ("A" / "B")

        # ---- 目标坐标变换缓存（最佳目标）----
        self.geo_lock = threading.Lock()
        self.latest_geo_status = "no_target"
        self.latest_geo_stamp = rospy.Time(0)
        self.latest_geo_body = None          # (x, y, z) in body frame
        self.latest_geo_enu = None           # (east, north, up)
        self.latest_geo_target_wgs84 = None  # (lat, lon, alt)
        self.latest_geo_bucket_ned = None    # (N, E, D)
        self.latest_geo_offset_ned = None    # (dN, dE, dD)

        # ---- TF / GeoTarget 发布 ----
        self.tf_buffer = tf2_ros.Buffer() if _HAS_TF2 else None
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) if _HAS_TF2 else None
        self.geo_pub = None
        if self.publish_geo_target:
            self.geo_pub = rospy.Publisher(self.output_topic, GeoTarget, queue_size=1)

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
        self.bucket_aim_info_pub = rospy.Publisher(
            "/vision/bucket/aim_info", BucketAimInfo, queue_size=1
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

        # 延迟 3 秒后检查是否需要请求飞控 MAVLink 消息流（参考 poshold_to_land_test.py）
        self._mavlink_stream_requested = False
        rospy.Timer(rospy.Duration(3.0), self._ensure_fc_mavlink_streams, oneshot=True)

        # ---- 任务状态订阅（来自 main.py）----
        self.mission_sub = rospy.Subscriber(
            "/vision/mission_status", MissionStatus, self._mission_cb, queue_size=5
        )

        rospy.loginfo("Detector started. model=%s ext=%s conf=%.2f imgsz=%s device=%s",
                       self.model_path, self.model_ext, self.conf_threshold,
                       str(self.imgsz), self.device)
        rospy.loginfo("Detector class names: %s", self.class_name_map)
        if self.publish_geo_target:
            rospy.loginfo("GeoTarget publishing enabled. topic=%s body=%s camera=%s",
                          self.output_topic, self.body_frame, self.camera_frame)
        elif not _HAS_TF2:
            rospy.loginfo("tf2_ros unavailable, bottom geopose bar will show TF placeholders")
        if self._has_fc_data:
            rospy.loginfo("FC data available: state=%s battery=%s gps=%s gpsraw=%s local_pose=%s",
                          self.fc_state_topic, self.fc_battery_topic,
                          self.fc_gps_topic, self.fc_gpsraw_topic, self.fc_local_pose_topic)
        else:
            rospy.loginfo("FC data NOT available (MAVROS not running or mavros_msgs not installed)")

    def _sync_imgsz_with_onnx_input(self):
        """Read ONNX input shape and force imgsz to match static models."""
        try:
            import onnxruntime as ort

            session = ort.InferenceSession(self.model_path, providers=["CPUExecutionProvider"])
            inputs = session.get_inputs()
            if not inputs:
                return

            shape = list(inputs[0].shape)
            if len(shape) < 4:
                return

            h = shape[2]
            w = shape[3]
            if not isinstance(h, int) or not isinstance(w, int) or h <= 0 or w <= 0:
                return

            resolved = h if h == w else (h, w)
            if self.imgsz != resolved:
                if self.imgsz_explicit:
                    rospy.logwarn(
                        "ONNX model expects input %sx%s, overriding requested imgsz=%s",
                        h, w, self.imgsz
                    )
                else:
                    rospy.loginfo("ONNX model input resolved to %sx%s", h, w)
                self.imgsz = resolved
        except Exception as exc:
            rospy.logwarn("Failed to inspect ONNX input shape, keep imgsz=%s: %s", self.imgsz, exc)

    def _warmup_model(self):
        """Run one dummy inference during startup to avoid first-frame stall."""
        try:
            if isinstance(self.imgsz, tuple):
                height, width = self.imgsz
            else:
                height = width = int(self.imgsz)
            dummy = np.zeros((height, width, 3), dtype=np.uint8)
            self.model(dummy, conf=self.conf_threshold, imgsz=self.imgsz, device=self.device, verbose=False)
            rospy.loginfo("YOLO warmup finished. input=%sx%s", width, height)
        except Exception as exc:
            rospy.logwarn("YOLO warmup failed, continue without warmup: %s", exc)

    def _build_class_name_map(self):
        """Build a stable class-id to class-name map independent of backend metadata."""
        model_names = getattr(self.model, "names", None) or {}
        if isinstance(model_names, dict):
            self.class_name_map.update({int(k): str(v) for k, v in model_names.items()})

        if self.class_name_overrides:
            for idx, name in enumerate(self.class_name_overrides):
                self.class_name_map[idx] = name

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
            self.fc_local_pose_sub = rospy.Subscriber(
                self.fc_local_pose_topic, PoseStamped, self.fc_local_pose_cb, queue_size=1
            )
            self._has_fc_data = True
        except Exception as exc:
            rospy.logwarn("FC subscriber init failed: %s. FC data will be N/A.", exc)
            self._has_fc_data = False
    def _ensure_fc_mavlink_streams(self, event=None):
        """自动请求飞控发送必要的 MAVLink 消息流。

        飞控默认 SR0_* 参数可能为 0，导致 LOCAL_POSITION_NED / GLOBAL_POSITION_INT
        等关键消息不会通过串口发送到 MAVROS。本方法在飞控已连接但数据缺失时，
        主动调用 /mavros/set_message_interval 请求飞控开启对应消息流。

        参考：cuadc_control/poshold_to_land_test.py 的 _wait_for_local_pose()
        """
        if self._mavlink_stream_requested:
            return
        if not self._has_fc_data:
            # MAVROS 未运行或 mavros_msgs 未安装，不请求
            return

        # 等待飞控连接 + 2 秒让现有流（如果有的话）到达
        deadline = time.time() + 5.0
        while time.time() < deadline and not rospy.is_shutdown():
            with self.fc_lock:
                connected = self.fc_connected
                has_latlon = self.fc_lat != 0.0 or self.fc_lon != 0.0
                has_local_pose = self.latest_local_pose_msg is not None
            if connected and has_latlon and has_local_pose:
                # 数据已经在流，不需要请求
                self._mavlink_stream_requested = True
                return
            if not connected:
                rospy.sleep(0.5)
                continue
            break

        with self.fc_lock:
            connected = self.fc_connected
        if not connected:
            # 飞控仍未连接，等下次再试
            return

        self._mavlink_stream_requested = True
        rospy.logwarn(
            "飞控已连接但缺少 NED/WGS84 数据，尝试自动请求 MAVLink 消息流 ..."
        )

        # 需要的 MAVLink 消息：
        #  ID 32: LOCAL_POSITION_NED   → /mavros/local_position/pose
        #  ID 33: GLOBAL_POSITION_INT  → /mavros/global_position/global
        #  ID 24: GPS_RAW_INT          → /mavros/gpsstatus/gps1/raw
        #  ID 31: ATTITUDE_QUATERNION  → orientation（四元数旋转必需）
        requests = [
            (32, 10.0, "LOCAL_POSITION_NED"),
            (33, 5.0,  "GLOBAL_POSITION_INT"),
            (24, 2.0,  "GPS_RAW_INT"),
            (31, 10.0, "ATTITUDE_QUATERNION"),
        ]

        try:
            rospy.wait_for_service("/mavros/set_message_interval", timeout=3.0)
            srv = rospy.ServiceProxy("/mavros/set_message_interval", MessageInterval)
            for msg_id, rate, name in requests:
                try:
                    resp = srv(message_id=msg_id, message_rate=rate)
                    if resp.success:
                        rospy.loginfo(
                            "  已请求 MAVLink %s (ID %d) @ %.0f Hz", name, msg_id, rate
                        )
                    else:
                        rospy.logwarn(
                            "  请求 %s (ID %d) 被拒绝，可能需要手动配置飞控 SR0_* 参数",
                            name, msg_id,
                        )
                except Exception as exc:
                    rospy.logwarn("  请求 %s (ID %d) 失败: %s", name, msg_id, exc)
        except (rospy.ROSException, rospy.ServiceException, Exception) as exc:
            rospy.logwarn(
                "/mavros/set_message_interval 调用失败: %s\n"
                "如飞控已定位，可手动执行:\n"
                "  rosservice call /mavros/set_message_interval "
                "'{message_id: 32, message_rate: 10.0}'\n"
                "  rosservice call /mavros/set_message_interval "
                "'{message_id: 33, message_rate: 5.0}'",
                exc,
            )

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
            self.fc_current = getattr(msg, 'current', 0.0)

    def fc_gpsraw_cb(self, msg):
        """GPS 原始数据回调（卫星数 + RTK 定位类型）"""
        with self.fc_lock:
            self.fc_satellites = getattr(msg, 'satellites_visible', 0)
            self.fc_fix_type = getattr(msg, 'fix_type', 0)

    def fc_gps_cb(self, msg):
        """GPS 全局位置回调（RTK 厘米级大地坐标）"""
        with self.fc_lock:
            self.latest_global_msg = msg
            self.fc_lat = getattr(msg, 'latitude', 0.0)
            self.fc_lon = getattr(msg, 'longitude', 0.0)
            self.fc_alt = getattr(msg, 'altitude', 0.0)

    def fc_local_pose_cb(self, msg):
        """本地位置回调（Z 高度 — NED 坐标系）"""
        with self.fc_lock:
            self.latest_local_pose_msg = msg
            self.fc_local_east = getattr(msg.pose.position, 'x', 0.0)
            self.fc_local_north = getattr(msg.pose.position, 'y', 0.0)
            self.fc_local_up = getattr(msg.pose.position, 'z', 0.0)
            self.fc_rel_z = getattr(msg.pose.position, 'z', 0.0)

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

    def is_valid_global(self, msg):
        """检查 GPS 数据是否可用于 WGS84 显示/换算。"""
        if msg is None:
            return False
        if getattr(msg.status, "status", NavSatStatus.STATUS_NO_FIX) < 0:
            return False
        return all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude))

    def _camera_position_to_body(self, camera_position):
        """相机光学系 XYZ → 机体系 FRD，复用节点已有的安装关系。"""
        cam_x, cam_y, cam_z = [float(value) for value in camera_position]
        mount_z = float(rospy.get_param("~camera_mount_z", 0.40))
        return (-cam_y, cam_x, cam_z + mount_z)

    def transform_camera_to_body(self, detection):
        """把最佳目标从相机光学系转换到机体系（直接数学变换，不依赖 TF 查询）。

        相机安装：在 CG 正下方，镜头朝下，相机顶部朝机体前方。
        X_body(前) = -Y_cam, Y_body(右) = X_cam, Z_body(下) = Z_cam + mount_offset
        """
        bx, by, bz = self._camera_position_to_body((
            detection.camera_x_m,
            detection.camera_y_m,
            detection.camera_z_m,
        ))

        result = PointStamped()
        result.header.frame_id = self.body_frame
        result.header.stamp = rospy.Time.now()
        result.point.x = bx
        result.point.y = by
        result.point.z = bz
        return result

    def _body_position_to_ned(self, body_position, local_pose):
        """机体系 FRD → ENU 偏移 → 飞控本地绝对 NED。

        返回 ``(enu_offset, absolute_ned, ned_offset)``；local_pose 缺失或
        数据非法时返回 None。BucketAimInfo 的 A/B NED 与最佳桶坐标共用此函数，
        从而保持相机系→机体系→ENU/NED 的坐标约定完全一致。
        """
        if local_pose is None:
            return None

        position = local_pose.pose.position
        orientation = local_pose.pose.orientation
        pose_values = (
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in pose_values):
            return None
        quaternion_norm = math.sqrt(
            orientation.x * orientation.x + orientation.y * orientation.y
            + orientation.z * orientation.z + orientation.w * orientation.w
        )
        if quaternion_norm <= 1e-6:
            return None

        # 上游 _camera_position_to_body() 输出 FRD。竖直轴必须把 +down
        # 变为 +up，防止高度正反馈；横向轴则使用 2026-07-19 真机日志
        # 标定值（默认保持 +right），避免桶在右边时向左飞。
        body_pose_axes = frd_to_local_pose_axes(
            body_position, self.local_pose_right_sign
        )
        east_m, north_m, up_m = quaternion_rotate_vector(
            orientation, body_pose_axes
        )
        enu = (float(east_m), float(north_m), float(up_m))
        offset_ned = (float(north_m), float(east_m), float(-up_m))
        absolute_ned = (
            float(position.y) + offset_ned[0],
            float(position.x) + offset_ned[1],
            float(-position.z) + offset_ned[2],
        )
        return enu, absolute_ned, offset_ned

    def _make_base_geo_target(self, detection):
        """构造与 geopose_node 相同结构的 GeoTarget 基础消息。"""
        target = GeoTarget()
        target.header = detection.header
        target.valid = False
        target.status = ""
        target.source_topic = "/vision/yolo/detection"
        target.class_name = detection.class_name
        target.confidence = detection.confidence
        target.center_x = detection.center_x
        target.center_y = detection.center_y
        target.camera_x_m = detection.camera_x_m
        target.camera_y_m = detection.camera_y_m
        target.camera_z_m = detection.camera_z_m
        return target

    def _publish_geo_status(self, target, status):
        """按 geopose_node 语义发布无效状态。"""
        if self.geo_pub is None:
            return
        target.status = status
        if self.publish_invalid:
            self.geo_pub.publish(target)

    def _set_geo_state(self, status, stamp, body=None, enu=None,
                       bucket_ned=None, offset_ned=None, target_wgs84=None):
        """缓存最佳目标坐标变换结果，供底部信息栏显示。"""
        with self.geo_lock:
            self.latest_geo_status = status
            self.latest_geo_stamp = stamp
            self.latest_geo_body = body
            self.latest_geo_enu = enu
            self.latest_geo_bucket_ned = bucket_ned
            self.latest_geo_offset_ned = offset_ned
            self.latest_geo_target_wgs84 = target_wgs84

    def _update_best_target_geo(self, detection):
        """对最佳目标执行相机→机体→ENU→NED/WGS84 变换，并可选发布 GeoTarget。
        相机→机体 TF 不依赖飞控数据，因此 BUCKET BODY FRD 在无 MAVROS/local_pose 时仍然可用。
        """
        stamp = detection.header.stamp if detection.header.stamp != rospy.Time() else rospy.Time.now()
        geo_target = self._make_base_geo_target(detection)

        with self.fc_lock:
            latest_global = self.latest_global_msg
            latest_local_pose = self.latest_local_pose_msg

        if not detection.detected:
            self._set_geo_state("no_detection", stamp)
            self._publish_geo_status(geo_target, "no_detection")
            return

        if detection.confidence < self.geo_min_confidence:
            self._set_geo_state("low_confidence", stamp)
            self._publish_geo_status(geo_target, "low_confidence")
            return

        if not detection.position_valid:
            self._set_geo_state("invalid_camera_position", stamp)
            self._publish_geo_status(geo_target, "invalid_camera_position")
            return

        # ── Step A：相机→机体 TF 变换（不依赖飞控，单独缓存 body 坐标） ──
        body_point = self.transform_camera_to_body(detection)
        if body_point is None:
            self._set_geo_state("tf_camera_to_body_failed", stamp)
            self._publish_geo_status(geo_target, "tf_camera_to_body_failed")
            return

        bx = float(body_point.point.x)
        by = float(body_point.point.y)
        bz = float(body_point.point.z)
        body = (bx, by, bz)

        # ── Step B：机体→ENU→NED→WGS84（需要飞控 local_pose + GPS） ──
        enu = None
        bucket_ned = None
        offset_ned = None
        target_wgs84 = None

        ned_result = self._body_position_to_ned(body, latest_local_pose)
        if ned_result is not None:
            enu, bucket_ned, offset_ned = ned_result
            east_m, north_m, up_m = enu

            if self.is_valid_global(latest_global):
                target_wgs84 = enu_offset_to_geodetic(
                    latest_global.latitude, latest_global.longitude, latest_global.altitude,
                    east_m, north_m, up_m
                )

            self._set_geo_state("ok", stamp, body, enu, bucket_ned, offset_ned, target_wgs84)
        else:
            # 无飞控 local_pose：body 坐标有效，但 ENU/NED/WGS84 不可用
            self._set_geo_state("ok_body_only", stamp, body,
                                bucket_ned=None, offset_ned=None, target_wgs84=None)

        # ── 可选 GeoTarget 发布 ──
        if self.geo_pub is None:
            return

        if ned_result is None:
            self._publish_geo_status(geo_target, "no_local_pose")
            return

        if not self.is_valid_global(latest_global):
            self._publish_geo_status(geo_target, "no_valid_global_position")
            return

        geo_target.valid = True
        geo_target.status = "ok"
        geo_target.body_x_m = body[0]
        geo_target.body_y_m = body[1]
        geo_target.body_z_m = body[2]
        geo_target.enu_east_m = enu[0] if enu else 0.0
        geo_target.enu_north_m = enu[1] if enu else 0.0
        geo_target.enu_up_m = enu[2] if enu else 0.0
        geo_target.latitude = float(target_wgs84[0]) if target_wgs84 else 0.0
        geo_target.longitude = float(target_wgs84[1]) if target_wgs84 else 0.0
        geo_target.altitude = float(target_wgs84[2]) if target_wgs84 else 0.0
        self.geo_pub.publish(geo_target)

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

        # 第 3.1 步：最佳桶中心点满 5 帧后使用算术平均值输出。
        # best、result_msg 和 detections_msg 中的最佳目标是同一个消息对象，
        # 因此这里更新一次即可同步影响后续发布与画面标注。
        self._smooth_best_center(best)

        # 第 3.2 步：基于稳定后的桶中心、当前深度和相机内参计算 A/B 瞄准点。
        aim_info = self._build_bucket_aim_info(
            msg.header, len(detections_msg.detections), best, frame.shape
        )

        # 第 3.5 步：只对最佳目标做一次坐标变换（用于底部栏 / 可选 GeoTarget）
        self._update_best_target_geo(result_msg)

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
        self.draw_bucket_aim_points(annotated, aim_info)

        # 4c. 画最佳目标的详细标注（见 draw_overlay）
        if best is not None:
            self.draw_overlay(annotated, best)

        # 4c2. 任务状态叠加文字 (AIMING!! / DROP!!!)
        self._draw_mission_overlay(annotated, w, h)

        # 4d. 画面底部整条信息栏：左=状态，右=目标坐标
        l_lines, r_lines = self._build_bottom_bar(rospy.Time.now(), len(detections_msg.detections))
        bottom_bar_h = self.draw_bottom_bar(annotated, w, h, l_lines, r_lines)

        # 4f. 帧率统计（仅内部计数，不再单独画）
        self._fps_counter += 1
        now = rospy.Time.now()
        dt = (now - self._fps_t0).to_sec()
        if dt >= 1.0:
            self._fps_display = self._fps_counter / dt
            self._fps_counter = 0
            self._fps_t0 = now

        # 第 5 步：发布
        self.result_pub.publish(result_msg)
        self.results_pub.publish(detections_msg)
        self.bucket_aim_info_pub.publish(aim_info)
        if self.annotated_pub.get_num_connections() > 0:
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
        # 兼容已经存在的 BucketInfo 扩展字段；新脚本应优先订阅
        # /vision/bucket/aim_info，以同时获得米制 delta 和 NED。
        binfo.aim_offsets_valid = aim_info.valid
        binfo.a_crosshair_x = float(aim_info.a_aim_x)
        binfo.a_crosshair_y = float(aim_info.a_aim_y)
        binfo.a_delta_x = aim_info.a_delta_x_px
        binfo.a_delta_y = aim_info.a_delta_y_px
        binfo.b_crosshair_x = float(aim_info.b_aim_x)
        binfo.b_crosshair_y = float(aim_info.b_aim_y)
        binfo.b_delta_x = aim_info.b_delta_x_px
        binfo.b_delta_y = aim_info.b_delta_y_px
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

    def _smooth_best_center(self, best):
        """使用最近 5 帧算术平均平滑最佳桶中心，连续 10 帧丢失则清空。"""
        if best is None or not best.detected:
            self.no_detection_frames += 1
            if self.no_detection_frames >= self.no_detection_clear_frames:
                self.center_buffer = []
                self.no_detection_frames = self.no_detection_clear_frames
            return

        self.no_detection_frames = 0
        self.center_buffer.append((float(best.center_x), float(best.center_y)))

        # 缓冲区固定保留最近 5 帧。
        if len(self.center_buffer) > self.center_buffer_size:
            self.center_buffer.pop(0)

        # 前 4 帧仍输出当前检测值；从第 5 帧开始输出 5 帧算术平均。
        if len(self.center_buffer) == self.center_buffer_size:
            best.center_x = int(round(
                sum(point[0] for point in self.center_buffer) / self.center_buffer_size
            ))
            best.center_y = int(round(
                sum(point[1] for point in self.center_buffer) / self.center_buffer_size
            ))

    def lookup_depth(self, x, y):
        """
        查询 (x, y) 像素处的深度值（取周围小区域的中位数）。
        如果初始半径无效，逐步扩大搜索范围直到找到有效值。
        返回：深度 (m)，无效则返回 0.0
        """
        with self.depth_lock:
            if self.latest_depth is None:
                return 0.0
            depth = self.latest_depth.copy()
        h, w = depth.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return 0.0

        # 逐级扩大搜索半径：2→5→10→20，只要有一级找到有效值就返回
        for r in (self.depth_patch_radius, 5, 10, 20):
            y0, y1 = max(0, y - r), min(h, y + r + 1)
            x0, x1 = max(0, x - r), min(w, x + r + 1)
            patch = depth[y0:y1, x0:x1]
            valid = patch[np.isfinite(patch) & (patch > 0.01)]
            if valid.size > 0:
                return float(np.median(valid))
        return 0.0

    def get_camera_info(self):
        """获取最新相机内参"""
        with self.camera_info_lock:
            return self.latest_camera_info

    def get_center_depth(self, img_w=640, img_h=480):
        """获取画面中心的深度值（相机朝下时≈离地高度）。
        取中心周围区域的 median，提高抗空洞能力。
        """
        with self.depth_lock:
            if self.latest_depth is None:
                return 0.0
            depth = self.latest_depth.copy()
        h, w = depth.shape[:2]
        cx, cy = img_w // 2 if img_w else w // 2, img_h // 2 if img_h else h // 2
        r = 6  # 中心区域半径，比目标查询更大以提高稳定性
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        patch = depth[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch) & (patch > 0.01)]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid))

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
        class_name = str(self.class_name_map.get(class_id, class_id))

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

    def _build_bucket_aim_info(self, header, count, best, image_shape):
        """构造 A/B 抛投瞄准信息。

        Delta 定义（半自动投放脚本的统一约定）：
          delta = aim_point - image_center
          * ``*_delta_*_px`` 使用像素；图像 x 向右、y 向下为正。
          * ``*_delta_*_m`` 使用当前桶深度和相机内参估计。
        后续脚本可用 ``abs(delta_x_m) < 0.15`` 且
        ``abs(delta_y_m) < 0.15`` 判断对应抛投器是否瞄准。

        ``valid`` 仅表示桶、深度、内参及 A/B 图像/米制点有效；没有
        local_pose 时仍发布这些字段，但 ``a_ned_valid/b_ned_valid`` 为 false。
        """
        aim = BucketAimInfo()
        aim.header = header
        aim.count = int(count)
        aim.valid = False
        aim.bucket_center_x = -1
        aim.bucket_center_y = -1
        aim.a_aim_x = aim.a_aim_y = -1
        aim.b_aim_x = aim.b_aim_y = -1
        aim.a_ned_valid = False
        aim.b_ned_valid = False

        if best is None or not best.detected:
            return aim

        aim.bucket_center_x = int(best.center_x)
        aim.bucket_center_y = int(best.center_y)
        aim.confidence = float(best.confidence)
        aim.source_class = best.class_name

        depth_m = float(best.depth_m)
        camera_info = self.get_camera_info()
        if (not best.position_valid or camera_info is None
                or depth_m <= 0.0 or not math.isfinite(depth_m)):
            return aim

        fx = float(camera_info.K[0])
        fy = float(camera_info.K[4])
        if fx <= 0.0 or fy <= 0.0:
            return aim

        # 真实距离通过针孔模型换算：pixel_offset = metre_offset * fy / depth。
        # 图像 y 向下为正。A 抛投器在相机前方，所以 A 点放在桶中心下方；
        # B 抛投器在相机后方，所以 B 点放在桶中心上方。
        offset_y_px = self.aim_offset_m * fy / depth_m
        a_x = float(best.center_x)
        a_y = float(best.center_y) + offset_y_px
        b_x = float(best.center_x)
        b_y = float(best.center_y) - offset_y_px

        aim.a_aim_x = int(round(a_x))
        aim.a_aim_y = int(round(a_y))
        aim.b_aim_x = int(round(b_x))
        aim.b_aim_y = int(round(b_y))

        image_h, image_w = image_shape[:2]
        image_center_x = image_w // 2
        image_center_y = image_h // 2
        aim.a_delta_x_px = float(aim.a_aim_x - image_center_x)
        aim.a_delta_y_px = float(aim.a_aim_y - image_center_y)
        aim.b_delta_x_px = float(aim.b_aim_x - image_center_x)
        aim.b_delta_y_px = float(aim.b_aim_y - image_center_y)
        aim.a_delta_x_m = self.pixel_to_m(aim.a_delta_x_px, depth_m, "x")
        aim.a_delta_y_m = self.pixel_to_m(aim.a_delta_y_px, depth_m, "y")
        aim.b_delta_x_m = self.pixel_to_m(aim.b_delta_x_px, depth_m, "x")
        aim.b_delta_y_m = self.pixel_to_m(aim.b_delta_y_px, depth_m, "y")
        aim.valid = True

        with self.fc_lock:
            latest_local_pose = self.latest_local_pose_msg

        for prefix, pixel_x, pixel_y in (
            ("a", a_x, a_y),
            ("b", b_x, b_y),
        ):
            camera_position = self.project_pixel(pixel_x, pixel_y, depth_m)
            if camera_position is None:
                continue
            body_position = self._camera_position_to_body(camera_position)
            ned_result = self._body_position_to_ned(body_position, latest_local_pose)
            if ned_result is None:
                continue
            absolute_ned = ned_result[1]
            setattr(aim, "{}_ned_valid".format(prefix), True)
            setattr(aim, "{}_ned_n".format(prefix), absolute_ned[0])
            setattr(aim, "{}_ned_e".format(prefix), absolute_ned[1])
            setattr(aim, "{}_ned_d".format(prefix), absolute_ned[2])

        return aim

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
                image, "x{:+.2f}  y{:+.2f}m".format(det.camera_x_m, det.camera_y_m),
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

    def draw_bucket_aim_points(self, image, aim_info):
        """绘制蓝色真实抛投瞄准点：A 在桶下方，B 在桶上方。"""
        if aim_info is None or not aim_info.valid:
            return
        height, width = image.shape[:2]
        for label, x, y in (
            ("A", aim_info.a_aim_x, aim_info.a_aim_y),
            ("B", aim_info.b_aim_x, aim_info.b_aim_y),
        ):
            if x < 0 or x >= width or y < 0 or y >= height:
                continue
            cv2.circle(image, (x, y), 5, (255, 0, 0), -1)
            cv2.putText(
                image, label, (min(width - 16, x + 7), max(15, y - 7)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA,
            )

    def draw_center_axes(self, image, cx, cy, length=45):
        """
        在画面中心画标准相机光学坐标系：
          x 轴（红色）指向右方，末端有箭头和字母 "x"  ← X+ = 右
          y 轴（绿色）指向下方，末端有箭头和字母 "y"  ← Y+ = 下
        右手定则：X(右) × Y(下) = Z(前)
        """
        # x 轴 — 红色，向右
        end_x = cx + length
        cv2.arrowedLine(image, (cx, cy), (end_x, cy), (0, 0, 255), 1,
                        tipLength=0.25)
        cv2.putText(image, "x", (end_x + 4, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1, cv2.LINE_AA)

        # y 轴 — 绿色，向下（标准相机光学坐标系 Y+ = 下，即图像 v 增大方向）
        end_y = cy + length
        cv2.arrowedLine(image, (cx, cy), (cx, end_y), (0, 255, 0), 1,
                        tipLength=0.25)
        cv2.putText(image, "y", (cx + 5, end_y + 12),
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

    def _build_bottom_bar(self, now, n_detections):
        """构建底部两栏合并栏，返回 (left_lines, right_lines)，
        每项为 [(text, color_bgr), ...]。
        LEFT  = 检测摘要 + 飞控状态 + 模型
        RIGHT = 坐标：BUCKET BODY FRD / FC NED / FC WGS84 / BUCKET NED / BUCKET WGS84
        """
        # ──────────── LEFT 栏：状态信息 ────────────
        left = []

        # 行1：检测摘要 + 模型
        model_name = os.path.basename(self.model_path)
        left.append((
            "DETECT: {}  FPS {:.1f}  |  {} @ {}".format(
                n_detections, self._fps_display, model_name, self.device
            ),
            (0, 255, 255),   # 黄色
        ))

        with self.fc_lock:
            connected = self.fc_connected
            armed = self.fc_armed
            mode = self.fc_mode
            voltage = self.fc_voltage
            current = self.fc_current
            satellites = self.fc_satellites
            fix_type = self.fc_fix_type
            lat = self.fc_lat
            lon = self.fc_lon
            alt = self.fc_alt
            rel_z = self.fc_rel_z

        with self.mission_lock:
            ammo_a = self.mission_ammo_a
            ammo_b = self.mission_ammo_b

        ammo_a_str = str(ammo_a) if ammo_a > 0 else "N/A"
        ammo_b_str = str(ammo_b) if ammo_b > 0 else "N/A"

        # 飞控 Z 高度：无 EKF 时用相机画面中心深度（= 相机离地高度）
        z_disp = rel_z
        z_from_depth = False
        if abs(rel_z) < 0.01 and self._has_fc_data:
            cam_h = self.get_center_depth()
            if cam_h > 0.01:
                z_disp = cam_h
                z_from_depth = True

        if not self._has_fc_data or not connected:
            left.append(("FC: disconnected", self._CLR_WARN))
            left.append(("AMMO: A-{}  B-{}".format(ammo_a_str, ammo_b_str), (0, 255, 255)))
        else:
            arm_str = "ARM" if armed else "DISARM"
            fix_str = self._gps_fix_name(fix_type)

            # 行2：FC 状态 + Z 高度（无 GPS 时自动从深度相机估算）
            z_tag = "Z_cam" if z_from_depth else "Z"
            left.append((
                "FC: {}  {}  {}  |  {}: {:+.2f} m".format(
                    mode, arm_str,
                    "conn" if connected else "disc",
                    z_tag,
                    z_disp,
                ),
                self._CLR_FC,
            ))

            # 行3：电池 + 卫星 + 弹药
            if voltage > 0.0:
                left.append((
                    "Bat: {:.1f}V  {:.1f}A  Sat: {}  {}  |  AMMO: A-{} B-{}".format(
                        voltage, abs(current), satellites, fix_str, ammo_a_str, ammo_b_str
                    ),
                    self._CLR_BODY,
                ))
            else:
                left.append((
                    "Sat: {}  {}  |  AMMO: A-{} B-{}".format(
                        satellites, fix_str, ammo_a_str, ammo_b_str
                    ),
                    self._CLR_BODY,
                ))

        # ──────────── RIGHT 栏：目标坐标 ────────────
        right = []

        with self.fc_lock:
            latest_global = self.latest_global_msg
            latest_local_pose = self.latest_local_pose_msg

        with self.geo_lock:
            geo_status = self.latest_geo_status
            geo_stamp = self.latest_geo_stamp
            geo_body = self.latest_geo_body
            bucket_ned = self.latest_geo_bucket_ned
            offset_ned = self.latest_geo_offset_ned
            target_wgs84 = self.latest_geo_target_wgs84

        stale = (
            geo_stamp == rospy.Time(0) or
            (now - geo_stamp) > rospy.Duration.from_sec(max(0.0, self.geo_result_timeout_sec))
        )

        # 行1：BUCKET BODY FRD（无需 GPS/飞控定位）
        if stale:
            right.append(("BUCKET BODY FRD --- (no bucket)", self._CLR_WARN))
        elif geo_body is not None:
            right.append((
                "BUCKET BODY FRD  x={:+.2f}  y={:+.2f}  z={:.2f} m".format(
                    geo_body[0], geo_body[1], geo_body[2]
                ),
                self._CLR_BODY,
            ))
        elif geo_status == "no_detection":
            right.append(("BUCKET BODY FRD --- (no detection)", self._CLR_WARN))
        elif geo_status == "no_local_pose":
            right.append(("BUCKET BODY FRD --- (no local pose)", self._CLR_WARN))
        elif geo_status == "tf_camera_to_body_failed":
            right.append(("BUCKET BODY FRD --- (tf failed)", self._CLR_WARN))
        elif geo_status == "invalid_camera_position":
            right.append(("BUCKET BODY FRD --- (invalid depth)", self._CLR_WARN))
        elif geo_status == "low_confidence":
            right.append(("BUCKET BODY FRD --- (low confidence)", self._CLR_WARN))
        else:
            right.append(("BUCKET BODY FRD --- ({})".format(geo_status), self._CLR_WARN))

        # 行2：FC NED
        if latest_local_pose is None:
            right.append(("FC NED --- (no local pose)", self._CLR_WARN))
        else:
            pos = latest_local_pose.pose.position
            right.append((
                "FC NED  N={:+06.2f}  E={:+06.2f}  D={:+06.2f} m".format(
                    float(pos.y), float(pos.x), float(-pos.z)
                ),
                self._CLR_FC,
            ))

        # 行3：FC WGS84
        if lat != 0.0 or lon != 0.0:
            right.append((
                "FC WGS84  lat={:.7f}  lon={:.7f}  alt={:.1f} m".format(lat, lon, alt),
                self._CLR_GPS,
            ))
        else:
            right.append(("FC WGS84 --- (no GPS fix)", self._CLR_WARN))

        # 行4：BUCKET NED
        if stale:
            right.append(("BUCKET NED --- (no bucket)", self._CLR_WARN))
        elif bucket_ned is not None and offset_ned is not None:
            right.append((
                "BUCKET NED  N={:+06.2f}  E={:+06.2f}  D={:+06.2f}  "
                "dN={:+05.2f} dE={:+05.2f} dD={:+05.2f}".format(
                    bucket_ned[0], bucket_ned[1], bucket_ned[2],
                    offset_ned[0], offset_ned[1], offset_ned[2]
                ),
                self._CLR_TARGET,
            ))
        elif geo_status in ("tf_camera_to_body_failed",):
            right.append(("BUCKET NED --- (tf failed)", self._CLR_WARN))
        elif geo_status in ("no_local_pose", "ok_body_only"):
            right.append(("BUCKET NED --- (no local pose)", self._CLR_WARN))
        else:
            right.append(("BUCKET NED --- (no bucket)", self._CLR_WARN))

        # 行5：BUCKET WGS84
        if not self.is_valid_global(latest_global):
            right.append(("BUCKET WGS84 --- (no GPS fix)", self._CLR_WARN))
        elif stale:
            right.append(("BUCKET WGS84 --- (no bucket)", self._CLR_WARN))
        elif target_wgs84 is not None:
            right.append((
                "BUCKET WGS84  lat={:.7f}  lon={:.7f}  alt={:.1f} m".format(
                    target_wgs84[0], target_wgs84[1], target_wgs84[2]
                ),
                self._CLR_TARGET,
            ))
        else:
            right.append(("BUCKET WGS84 --- (no bucket)", self._CLR_WARN))

        return left, right

    @staticmethod
    def _truncate_text_to_width(text, font, font_scale, thickness, max_width):
        """把超宽文本裁剪到指定宽度以内。"""
        if cv2.getTextSize(text, font, font_scale, thickness)[0][0] <= max_width:
            return text

        ellipsis = "..."
        base = text
        while base:
            candidate = base + ellipsis
            if cv2.getTextSize(candidate, font, font_scale, thickness)[0][0] <= max_width:
                return candidate
            base = base[:-1]
        return ellipsis

    # 底部栏颜色定义 (BGR)
    _CLR_TARGET = (0, 255, 255)       # 黄色 — 桶/目标相关
    _CLR_BODY   = (255, 200, 100)     # 天蓝 — 机体系坐标
    _CLR_FC     = (120, 255, 120)     # 绿色 — 飞控 NED
    _CLR_GPS    = (100, 220, 255)     # 金色 — GPS/WGS84
    _CLR_WARN   = (80, 80, 255)       # 红色 — 缺失/错误

    def draw_bottom_bar(self, image, img_w, img_h, left_lines, right_lines):
        """在画面最底部绘制半透明两栏合并状态栏。
        left_lines / right_lines: 各为 [(text, color_bgr), ...]
        左栏=状态信息，右栏=目标坐标，中间用竖线分割。
        """
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.48
        min_scale = 0.30
        thickness = 1
        pad_x = 8
        pad_y = 6
        gap = 3
        divider_w = 2     # 竖线宽度
        divider_gap = 6   # 竖线两侧间距

        mid_x = img_w // 2
        left_w = mid_x - divider_gap - divider_w // 2
        right_w = img_w - mid_x - divider_gap - divider_w // 2 - pad_x

        # 自适应字号
        all_texts = [t for t, _ in left_lines] + [t for t, _ in right_lines]
        max_left_w = max(40, left_w - pad_x * 2)
        max_right_w = max(40, right_w - pad_x)

        while font_scale > min_scale:
            left_widths = [cv2.getTextSize(t, font, font_scale, thickness)[0][0] for t, _ in left_lines]
            right_widths = [cv2.getTextSize(t, font, font_scale, thickness)[0][0] for t, _ in right_lines]
            if max(left_widths) <= max_left_w and max(right_widths) <= max_right_w:
                break
            font_scale -= 0.02

        # 截断超宽文字
        def clip(lines_list, max_w):
            result = []
            for t, c in lines_list:
                result.append((self._truncate_text_to_width(t, font, font_scale, thickness, max_w), c))
            return result

        left_clipped = clip(left_lines, max_left_w)
        right_clipped = clip(right_lines, max_right_w)

        # 行高由较高栏决定
        left_sizes = [cv2.getTextSize(t, font, font_scale, thickness)[0] for t, _ in left_clipped]
        right_sizes = [cv2.getTextSize(t, font, font_scale, thickness)[0] for t, _ in right_clipped]
        line_h = max(
            max(s[1] for s in left_sizes) if left_sizes else 16,
            max(s[1] for s in right_sizes) if right_sizes else 16,
        )
        n_rows = max(len(left_clipped), len(right_clipped))
        bar_h = n_rows * line_h + (n_rows - 1) * gap + pad_y * 2
        y0 = max(0, img_h - bar_h)

        # 半透明黑底
        overlay = image.copy()
        cv2.rectangle(overlay, (0, y0), (img_w, img_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.72, image, 0.28, 0.0, image)
        # 顶部分割线
        cv2.line(image, (0, y0), (img_w, y0), (0, 255, 255), 1)
        # 中间竖线
        cv2.line(image, (mid_x, y0 + 4), (mid_x, img_h - 4), (100, 100, 100), 1)

        # 逐行绘制
        y = y0 + pad_y + line_h
        for row in range(n_rows):
            # 左栏
            if row < len(left_clipped):
                text, clr = left_clipped[row]
                cv2.putText(image, text, (pad_x, y), font, font_scale, clr, thickness, cv2.LINE_AA)
            # 右栏
            if row < len(right_clipped):
                text, clr = right_clipped[row]
                cv2.putText(image, text, (mid_x + divider_gap, y), font, font_scale, clr, thickness, cv2.LINE_AA)
            y += line_h + gap

        return bar_h

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
