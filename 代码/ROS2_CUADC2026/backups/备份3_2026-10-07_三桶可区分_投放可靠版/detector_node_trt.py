#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cuadc_vision_trt.detector_node

TensorRT 识别节点，消息接口与 cuadc_vision / cuadc_visionY26 一致：
  - TensorRT FP16 engine（独立子进程 + 共享内存环形缓冲）
  - Stabilizer：双阈值迟滞 + EMA 平滑 + hold 丢失缓冲 + 嵌套框抑制
  - 深度 -> 相机系 XYZ -> 机体系 -> ENU -> WGS84（前后双投放点）
  - direct_camera 模式：节点内直开 D435（彩色 + 深度对齐），不走 ROS 图像话题

发布：
  /vision/yolo/detection   (YoloDetection)   最高置信度目标
  /vision/yolo/detections  (YoloDetections)  全部目标（含稳定器保持框）
  /vision/bucket/info      (BucketInfo)      数量 + 像素偏差
  /vision/bucket/estimates (BucketEstimates) 前 3 目标独立 WGS84/ENU
  /vision/annotated_image  (Image)
  /vision/target_global    (GeoTarget)       每帧每目标一条（最多 max_geo_targets）
"""
import json
import math
import os
import signal
import time
import threading
import multiprocessing as mp
import warnings
from collections import deque

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
os.environ.setdefault("ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS", "1")

import cv2
import numpy as np

warnings.filterwarnings("ignore")
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import BatteryState, CameraInfo, Image, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, Header, String

from cuadc_vision.msg import (
    YoloDetection, YoloDetections, BucketInfo, BucketEstimates, BucketTarget,
    MissionStatus, GeoTarget,
)

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None

try:
    from mavros_msgs.msg import GPSRAW, State
    from mavros_msgs.srv import MessageInterval
    _HAS_MAVROS_MSGS = True
except Exception:
    GPSRAW = None
    State = None
    MessageInterval = None
    _HAS_MAVROS_MSGS = False

from ultralytics import YOLO


# ---------------------------------------------------------------------------
# QoS
# ---------------------------------------------------------------------------
IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
NAV_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
MSG_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
PUB_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


# ---------------------------------------------------------------------------
# 默认模型路径
# ---------------------------------------------------------------------------
ONNX_REL_PATH = os.path.join(
    "models", "v8_merged_bucket_clean_noaug640", "weights", "best.onnx"
)
ENGINE_896_REL_PATH = os.path.join(
    "models", "engines", "orin_best_896_fp16.engine"
)
ENGINE_640_REL_PATH = os.path.join(
    "models", "engines", "orin_best_640_fp16.engine"
)


def _model_roots():
    roots = []
    try:
        from ament_index_python.packages import get_package_share_directory
        roots.append(get_package_share_directory("cuadc_vision_trt"))
    except Exception:
        pass
    roots.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    roots.append(os.path.join(os.path.expanduser("~"), "ROS2_CUADC2026", "src",
                              "cuadc_vision_trt"))
    return roots


def default_model_path():
    """默认 896 engine，回退 640 engine / ONNX。"""
    roots = _model_roots()
    for rel in (ENGINE_896_REL_PATH, ENGINE_640_REL_PATH, ONNX_REL_PATH):
        for root in roots:
            path = os.path.join(root, rel)
            if os.path.isfile(path):
                return path
    return os.path.join(roots[0], ONNX_REL_PATH)


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------
def node_param(node, name, default):
    try:
        if node.has_parameter(name):
            return node.get_parameter(name).value
    except Exception:
        pass
    try:
        node.declare_parameter(name, default)
    except Exception:
        pass
    try:
        return node.get_parameter(name).value
    except Exception:
        return default


def get_bool_param(node, name, default):
    v = node_param(node, name, default)
    if isinstance(v, bool):
        return v
    return str(v).strip().lower() in ("1", "true", "t", "yes", "y", "on")


def quaternion_rotate_vector(q, vector):
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


def enu_offset_to_geodetic(origin_lat, origin_lon, origin_alt, east_m, north_m, up_m):
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


_throttle_state = {}


def _throttle_log(node, interval_s, fmt, *args, level="warn"):
    now = time.monotonic()
    last = _throttle_state.get(fmt, 0.0)
    if now - last >= interval_s:
        _throttle_state[fmt] = now
        text = fmt % args if args else fmt
        getattr(node.get_logger(), level)(text)


def _stamp_is_zero(stamp):
    return stamp.sec == 0 and stamp.nanosec == 0


def _decode_image(msg, encoding):
    if msg.height <= 0 or msg.width <= 0:
        return None
    try:
        if encoding == "bgr8":
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            need = msg.step * msg.height
            if buf.size < need:
                return None
            row = buf[:need].reshape(msg.height, msg.step)[:, : msg.width * 3]
            return row.reshape(msg.height, msg.width, 3).copy()
        if encoding == "32FC1":
            buf = np.frombuffer(bytes(msg.data), dtype=np.float32)
            step_f = msg.step // 4
            if step_f <= 0:
                return None
            need = step_f * msg.height
            if buf.size < need:
                return None
            return buf[:need].reshape(msg.height, step_f)[:, : msg.width].copy()
    except Exception:
        return None
    return None


def _mavros_is_alive(node):
    try:
        return node.count_publishers("/mavros/state") > 0
    except Exception:
        return False


def _wait_for_mavros(node, timeout=30):
    waited = 0.0
    while not _mavros_is_alive(node):
        if waited >= timeout:
            return False
        time.sleep(1.0)
        waited += 1.0
    return True


def _ensure_mavros(node, fcu_url):
    if _mavros_is_alive(node):
        node.get_logger().info("[auto-start] MAVROS 已在运行")
        return
    grace = float(node_param(node, "mavros_discovery_grace", 3.0))
    time.sleep(grace)
    if _mavros_is_alive(node):
        node.get_logger().info("[auto-start] MAVROS 已在运行（宽限期后确认）")
        return
    node.get_logger().info("[auto-start] MAVROS 未运行，自动启动 mavros apm.launch ...")
    try:
        import subprocess
        subprocess.Popen(
            ['ros2', 'launch', 'mavros', 'apm.launch.py',
             'fcu_url:={}'.format(fcu_url)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except Exception as exc:
        node.get_logger().error("[auto-start] 启动 MAVROS 失败: %s" % exc)
        return
    if _wait_for_mavros(node, 30):
        node.get_logger().info("[auto-start] MAVROS 已就绪")
    else:
        node.get_logger().warn("[auto-start] MAVROS 启动超时（30s），飞控数据不可用")


# ---------------------------------------------------------------------------
# 稳定器（沿用 LSY d435_ui 实现）
# ---------------------------------------------------------------------------
def box_iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def box_dist(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    dx = ((ax1 + ax2) - (bx1 + bx2)) / 2.0
    dy = ((ay1 + ay2) - (by1 + by2)) / 2.0
    return (dx * dx + dy * dy) ** 0.5


def suppress_nested(dets, iomin_thr=0.6):
    dets = sorted(dets, key=lambda d: d["conf"], reverse=True)
    keep = []
    for d in dets:
        ax1, ay1, ax2, ay2 = d["box"]
        area_a = max(1e-6, (ax2 - ax1) * (ay2 - ay1))
        drop = False
        for k in keep:
            bx1, by1, bx2, by2 = k["box"]
            iw = max(0, min(ax2, bx2) - max(ax1, bx1))
            ih = max(0, min(ay2, by2) - max(ay1, by1))
            inter = iw * ih
            area_b = (bx2 - bx1) * (by2 - by1)
            if inter / min(area_a, area_b) >= iomin_thr:
                drop = True
                break
        if not drop:
            keep.append(d)
    return keep


class Stabilizer:
    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks = []

    def _absorb(self, tr, rd, now):
        cfg = self.cfg
        if cfg["smooth"]:
            tr["box"] = [cfg["alpha"] * rd["box"][i] + (1 - cfg["alpha"]) * tr["box"][i]
                         for i in range(4)]
            tr["conf"] = cfg["alpha"] * rd["conf"] + (1 - cfg["alpha"]) * tr["conf"]
        else:
            tr["box"], tr["conf"], tr["cls"] = rd["box"], rd["conf"], rd["cls"]
        tr["last_seen"] = now
        tr["held"] = False

    def update(self, raw_list, now):
        cfg = self.cfg
        raw_dets = []
        for x in raw_list:
            raw_dets.append({"box": list(x[0]), "conf": x[1], "cls": x[2]})
        raw_dets = suppress_nested(raw_dets, iomin_thr=0.6)
        if cfg["max_boxes"] >= 1:
            raw_dets = sorted(raw_dets, key=lambda d: d["conf"], reverse=True)[:cfg["max_boxes"]]
        used = [False] * len(raw_dets)
        out = []
        for tr in list(self.tracks):
            if tr.get("held"):
                if now - tr["last_seen"] <= cfg["hold"]:
                    out.append(tr)
                continue
            best_j, best_iou = -1, -1.0
            for j, rd in enumerate(raw_dets):
                if used[j]:
                    continue
                iou = box_iou(tr["box"], rd["box"])
                if iou > best_iou:
                    best_iou, best_j = iou, j
            if best_j >= 0 and best_iou >= 0.25:
                used[best_j] = True
                self._absorb(tr, raw_dets[best_j], now)
                out.append(tr)
                continue
            if now - tr["last_seen"] <= cfg["hold"]:
                tr["held"] = True
                tr["conf"] = max(0.0, tr["conf"] - 0.02)
                out.append(tr)
        for j, rd in enumerate(raw_dets):
            if used[j] or rd["conf"] < cfg["enter_thr"]:
                continue
            nt = {"box": list(rd["box"]), "conf": rd["conf"], "cls": rd["cls"],
                  "last_seen": now, "held": False}
            self.tracks.append(nt)
            out.append(nt)
        self.tracks = [t for t in out if not t["held"] or now - t["last_seen"] <= cfg["hold"]]
        return out


# ---------------------------------------------------------------------------
# 桩对象：让稳定器轨道复用 build_single 的 box 接口
# ---------------------------------------------------------------------------
class _FakeTensor(np.ndarray):
    def __new__(cls, arr):
        return np.asarray(arr, dtype=np.float32).view(cls)

    def __array_finalize__(self, obj):
        pass

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return np.asarray(self)


class _StubBox:
    def __init__(self, xyxy, cls, conf):
        self.xyxy = _FakeTensor(xyxy)
        self.cls = _FakeTensor([cls])
        self.conf = _FakeTensor([conf])


# ---------------------------------------------------------------------------
# 独立推理进程
# ---------------------------------------------------------------------------
def _infer_worker_trt(model_path, w, h, device, imgsz, shm_names, in_q, out_q, stop_evt):
    import numpy as _np
    import signal as _signal
    import time
    from multiprocessing import shared_memory as _shm
    from ultralytics import YOLO as _YOLO
    try:
        _signal.signal(_signal.SIGINT, _signal.SIG_IGN)
    except Exception:
        pass
    model = _YOLO(model_path)
    bufs = [_shm.SharedMemory(name=n) for n in shm_names]
    warm = _np.zeros((h, w, 3), _np.uint8)
    for _ in range(3):
        try:
            model.predict(warm, conf=0.10, imgsz=imgsz, device=device, verbose=False)
        except Exception:
            pass
    try:
        out_q.put(("names", dict(model.names)))
    except Exception:
        out_q.put(("names", {0: "bucket"}))
    while not stop_evt.is_set():
        try:
            item = in_q.get(timeout=0.2)
        except Exception:
            continue
        if item is None:
            break
        bi, seq = item
        try:
            buf = _np.ndarray((h, w, 3), _np.uint8, buffer=bufs[bi].buf)
            t0 = time.perf_counter()
            r = model.predict(buf, conf=0.10, imgsz=imgsz, device=device, verbose=False)[0]
            boxes = [(b.xyxy[0].tolist(), float(b.conf[0]), int(b.cls[0])) for b in r.boxes]
            out_q.put((seq, boxes, (time.perf_counter() - t0) * 1000.0))
        except Exception as e:
            out_q.put(("error", repr(e)))
    try:
        in_q.put(None)
    except Exception:
        pass


# ===========================================================================
#  DetectorNode
# ===========================================================================
class TRTDetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        node = self

        # ---------- 模型 / 推理 ----------
        self.model_path = os.path.expanduser(str(node_param(node, "model_path", "")))
        self.conf_threshold = float(node_param(node, "conf_threshold", 0.35))
        self.device = str(node_param(node, "device", "cpu")).strip()
        self.imgsz = int(node_param(node, "imgsz", 896))
        self.show_window = get_bool_param(node, "show_window", True)
        self.window_name = str(node_param(node, "window_name", "YOLO TRT Detection"))
        self.use_proc = get_bool_param(node, "use_proc", True)
        self.w = int(node_param(node, "w", 640))
        self.h = int(node_param(node, "h", 480))

        # ---------- 稳定器 ----------
        self.cfg = {
            "enter_thr": float(node_param(node, "enter_thr", 0.35)),
            "exit_thr": float(node_param(node, "exit_thr", 0.15)),
            "alpha": float(node_param(node, "alpha", 0.4)),
            "hold": float(node_param(node, "hold", 0.5)),
            "max_boxes": int(node_param(node, "max_boxes", 0)),
            "smooth": True,
        }
        self.cfg["hold"] = max(0.0, min(0.5, self.cfg["hold"]))
        self.stab = Stabilizer(self.cfg)

        # ---------- 相机 ----------
        self.direct_camera = get_bool_param(node, "direct_camera", False)
        self.direct_depth = get_bool_param(node, "direct_depth", True)
        self.cam_fps = int(node_param(node, "cam_fps", 30))
        self.depth_w = int(node_param(node, "depth_w", 1280))
        self.depth_h = int(node_param(node, "depth_h", 720))
        self.publish_annotated = get_bool_param(node, "publish_annotated", True)

        # ---------- 坐标 / 深度 ----------
        self.invert_camera_x = get_bool_param(node, "invert_camera_x", True)
        self.camera_mount_z = float(node_param(node, "camera_mount_z", 0.40))
        self.dual_offset_m = float(node_param(node, "dual_offset_m", 0.05))
        self.depth_patch_radius = int(node_param(node, "depth_patch_radius", 5))

        # ---------- 类别过滤 ----------
        target_str = node_param(
            node, "target_classes",
            "cylinder,tong,barrel,can,yuantong,white_cylinder,tube,drum,bucket"
        )
        self.target_classes = [c.strip().lower() for c in target_str.split(",") if c.strip()]
        class_names_str = str(node_param(node, "class_names", "bucket")).strip()
        self.class_name_overrides = [x.strip() for x in class_names_str.split(",") if x.strip()]
        self.class_name_map = {}

        # ---------- 飞控 MAVROS ----------
        self.auto_start_mavros = get_bool_param(node, "auto_start_mavros", False)
        self.mavros_fcu_url = node_param(node, "mavros_fcu_url", "/dev/ttyACM0:921600")
        self.fc_state_topic = node_param(node, "fc_state_topic", "/mavros/state")
        self.fc_battery_topic = node_param(node, "fc_battery_topic", "/mavros/battery")
        self.fc_gps_topic = node_param(node, "fc_gps_topic", "/mavros/global_position/global")
        self.fc_gpsraw_topic = node_param(node, "fc_gpsraw_topic", "/mavros/gpsstatus/gps1/raw")
        self.fc_local_pose_topic = node_param(node, "fc_local_pose_topic", "/mavros/local_position/pose")
        self.fc_rel_alt_topic = node_param(node, "fc_rel_alt_topic", "/mavros/global_position/rel_alt")
        self.output_topic = node_param(node, "output_topic", "/vision/target_global")
        self.body_frame = node_param(node, "body_frame", "base_link")
        self.camera_frame = node_param(node, "camera_frame", "d435i_color_optical_frame")
        self.geo_min_confidence = float(node_param(node, "min_confidence", 0.30))
        self.publish_invalid = get_bool_param(node, "publish_invalid", True)
        self.publish_geo_target = get_bool_param(node, "publish_geo_target", True)
        self.max_geo_targets = int(node_param(node, "max_geo_targets", 3))

        # ---------- 话题 ----------
        self.color_topic = node_param(node, "color_topic", "/vision/color/image_raw")
        self.depth_topic = node_param(node, "depth_topic", "/vision/aligned_depth/image_raw")
        self.camera_info_topic = node_param(node, "camera_info_topic", "/vision/color/camera_info")

        # ---------- 数据容器 ----------
        self.bridge = CvBridge() if CvBridge else None
        self.depth_lock = threading.Lock()
        self.camera_info_lock = threading.Lock()
        self.latest_depth = None
        self.latest_camera_info = None

        self.fc_lock = threading.Lock()
        self.fc_connected = False
        self.fc_armed = False
        self.fc_mode = "N/A"
        self.fc_voltage = 0.0
        self.fc_current = 0.0
        self.fc_satellites = 0
        self.fc_fix_type = 0
        self.fc_lat = 0.0
        self.fc_lon = 0.0
        self.fc_alt = 0.0
        self.fc_rel_alt = 0.0
        self._has_fc_data = False
        self.latest_global_msg = None
        self.latest_local_pose_msg = None

        self.mission = MissionStatus()

        # ---------- 订阅 ----------
        self.image_sub = None
        self.depth_sub = None
        if not self.direct_camera:
            self.image_sub = node.create_subscription(
                Image, self.color_topic, self.image_cb, IMAGE_QOS)
            self.depth_sub = node.create_subscription(
                Image, self.depth_topic, self.depth_cb, IMAGE_QOS)
        self.caminfo_sub = node.create_subscription(
            CameraInfo, self.camera_info_topic, self.ci_cb, IMAGE_QOS)
        self.mission_sub = node.create_subscription(
            MissionStatus, "/vision/mission_status", self.mission_cb, MSG_QOS)

        # ---------- 发布 ----------
        self.result_pub = node.create_publisher(YoloDetection, "/vision/yolo/detection", PUB_QOS)
        self.results_pub = node.create_publisher(YoloDetections, "/vision/yolo/detections", PUB_QOS)
        self.annotated_pub = node.create_publisher(Image, "/vision/annotated_image", PUB_QOS)
        self.bucket_info_pub = node.create_publisher(BucketInfo, "/vision/bucket/info", PUB_QOS)
        self.bucket_estimates_pub = node.create_publisher(BucketEstimates, "/vision/bucket/estimates", PUB_QOS)
        self.geo_pub = None
        if self.publish_geo_target:
            self.geo_pub = node.create_publisher(GeoTarget, self.output_topic, PUB_QOS)
        self.status_pub = node.create_publisher(String, "/vision/detector_status", PUB_QOS)
        self.cam_info_pub = None
        if self.direct_camera:
            self.cam_info_pub = node.create_publisher(
                CameraInfo, self.camera_info_topic, PUB_QOS)

        # ---------- 模型加载 ----------
        if not self.model_path:
            self.model_path = default_model_path()
        if self.direct_camera and not self.use_proc:
            self.get_logger().warn("direct_camera 需要 use_proc=true，已强制开启")
            self.use_proc = True
        self.model = None
        self.names = {0: "bucket"}
        try:
            if not self.use_proc:
                self.model = YOLO(self.model_path)
                try:
                    self.names = self.model.names
                except Exception:
                    pass
            else:
                from multiprocessing import shared_memory as _shm
                self._shms = [_shm.SharedMemory(create=True, size=self.w * self.h * 3)
                              for _ in range(4)]
                ctx = mp.get_context("spawn")
                self._in_q, self._out_q = ctx.Queue(maxsize=2), ctx.Queue(maxsize=64)
                self._stop_evt = ctx.Event()
                p = ctx.Process(
                    target=_infer_worker_trt,
                    args=(self.model_path, self.w, self.h, self.device, self.imgsz,
                          [x.name for x in self._shms], self._in_q, self._out_q,
                          self._stop_evt),
                    daemon=True)
                p.start()
                self._proc = p
                try:
                    msg = self._out_q.get(timeout=15)
                    if msg and msg[0] == "names":
                        self.names = msg[1]
                except Exception:
                    pass
        except Exception as e:
            self.get_logger().error("Failed to load model: %s" % e)
        self._build_class_name_map()

        names_disp = self.names if len(self.names) <= 5 else "(%d classes)" % len(self.names)
        self.get_logger().info(
            "detector model=%s device=%s use_proc=%s names=%s"
            % (self.model_path, self.device, self.use_proc, names_disp))
        cm_disp = (self.class_name_map if len(self.class_name_map) <= 5
                   else "(%d classes)" % len(self.class_name_map))
        self.get_logger().info("class map: %s  targets=%s" % (cm_disp, self.target_classes))

        # ---------- 飞控订阅 + 消息流请求 ----------
        self._init_fc_subscribers()
        self._mavlink_stream_requested = False
        self._stream_timer = node.create_timer(3.0, self._on_stream_timer)

        # ---------- 流水线状态 ----------
        self._fps_t0 = time.time()
        self._fps_n = 0
        self._fps_disp = 0.0
        self._cam_n = 0
        self._cam_disp = 0.0
        self._last_ms = 0.0
        self._seq = 0
        self._last_drawn_seq = 0
        # 200mm 桶 ratio 标定：最高置信度实时目标最近 30 帧比值（约 1s）的中位数
        self._cal_ratio_hist = deque(maxlen=30)
        # 抛投器对准标定：最高置信度实时目标的相机系原始差值 (dx, dy_cam) 最近 30 帧
        self._cal_delta_hist = deque(maxlen=30)
        self._headers = {}
        self._depth_ring = {}
        self._nbuf = 4
        self._size_warned = False
        self._cap_run = False
        self._cap_thread = None
        self._cap_lock = threading.Lock()
        self._align = None
        self._direct_depth_scale = 0.001
        # GUI 显示独立线程：cv2.imshow 在 X/远程桌面下可能阻塞数秒，
        # 放在 ROS executor 线程会导致结果积压、话题停发（控制端判"视觉停更"）。
        self._disp_run = bool(self.show_window)
        self._disp_evt = threading.Event()
        self._disp_lock = threading.Lock()
        self._disp_frame = None
        self._disp_thread = None
        if self.show_window:
            self._disp_thread = threading.Thread(
                target=self._display_loop, daemon=True, name="gui_display")
            self._disp_thread.start()
        if self.direct_camera:
            self._start_direct_camera()
        if self.use_proc:
            node.create_timer(0.005, self._on_drain_timer)

    # =======================================================================
    # 类别名
    # =======================================================================
    def _build_class_name_map(self):
        if isinstance(self.names, dict):
            self.class_name_map.update({int(k): str(v) for k, v in self.names.items()})
        for idx, name in enumerate(self.class_name_overrides):
            self.class_name_map[idx] = name

    def _is_target_class(self, class_name):
        name = str(class_name).strip().lower()
        for kw in self.target_classes:
            if kw in name:
                return True
        return False

    # =======================================================================
    # 飞控订阅 / 回调
    # =======================================================================
    def _init_fc_subscribers(self):
        if not _HAS_MAVROS_MSGS:
            self.get_logger().info("mavros_msgs not installed, skipping FC subscriptions")
            return
        try:
            self.node_fc_state = self.create_subscription(
                State, self.fc_state_topic, self.fc_state_cb, NAV_QOS)
            self.node_fc_battery = self.create_subscription(
                BatteryState, self.fc_battery_topic, self.fc_battery_cb, NAV_QOS)
            self.node_fc_gpsraw = self.create_subscription(
                GPSRAW, self.fc_gpsraw_topic, self.fc_gpsraw_cb, NAV_QOS)
            self.node_fc_gps = self.create_subscription(
                NavSatFix, self.fc_gps_topic, self.fc_gps_cb, NAV_QOS)
            self.node_fc_pose = self.create_subscription(
                PoseStamped, self.fc_local_pose_topic, self.fc_local_pose_cb, NAV_QOS)
            self.node_fc_rel_alt = self.create_subscription(
                Float64, self.fc_rel_alt_topic, self.fc_rel_alt_cb, NAV_QOS)
            self._has_fc_data = True
        except Exception as exc:
            self.get_logger().warn("FC subscriber init failed: %s" % exc)
            self._has_fc_data = False

    def fc_state_cb(self, msg):
        with self.fc_lock:
            self.fc_connected = getattr(msg, 'connected', False)
            self.fc_armed = getattr(msg, 'armed', False)
            self.fc_mode = getattr(msg, 'mode', 'N/A')

    def fc_battery_cb(self, msg):
        with self.fc_lock:
            self.fc_voltage = getattr(msg, 'voltage', 0.0)
            self.fc_current = getattr(msg, 'current', 0.0)

    def fc_gpsraw_cb(self, msg):
        with self.fc_lock:
            self.fc_satellites = getattr(msg, 'satellites_visible', 0)
            self.fc_fix_type = getattr(msg, 'fix_type', 0)

    def fc_gps_cb(self, msg):
        with self.fc_lock:
            self.latest_global_msg = msg
            self.fc_lat = getattr(msg, 'latitude', 0.0)
            self.fc_lon = getattr(msg, 'longitude', 0.0)
            self.fc_alt = getattr(msg, 'altitude', 0.0)

    def fc_local_pose_cb(self, msg):
        with self.fc_lock:
            self.latest_local_pose_msg = msg

    def fc_rel_alt_cb(self, msg):
        try:
            value = float(msg.data)
        except (TypeError, ValueError):
            return
        if math.isfinite(value):
            with self.fc_lock:
                self.fc_rel_alt = value

    def mission_cb(self, msg):
        self.mission = msg

    def _on_stream_timer(self):
        if self._stream_timer is not None:
            self._stream_timer.cancel()
            self._stream_timer = None
        threading.Thread(target=self._ensure_fc_mavlink_streams, daemon=True).start()

    def _ensure_fc_mavlink_streams(self):
        if self._mavlink_stream_requested or not self._has_fc_data:
            return
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and rclpy.ok():
            with self.fc_lock:
                connected = self.fc_connected
                has_latlon = self.fc_lat != 0.0 or self.fc_lon != 0.0
                has_pose = self.latest_local_pose_msg is not None
            if connected and has_latlon and has_pose:
                self._mavlink_stream_requested = True
                return
            if not connected:
                time.sleep(0.5)
                continue
            break
        with self.fc_lock:
            connected = self.fc_connected
        if not connected:
            return
        self._mavlink_stream_requested = True
        log = self.get_logger()
        log.warn("飞控已连接但缺少 NED/WGS84 数据，尝试请求 MAVLink 消息流 ...")
        requests = [
            (32, 10.0, "LOCAL_POSITION_NED"),
            (33, 5.0, "GLOBAL_POSITION_INT"),
            (24, 2.0, "GPS_RAW_INT"),
            (31, 10.0, "ATTITUDE_QUATERNION"),
        ]
        try:
            client = self.create_client(MessageInterval, "/mavros/set_message_interval")
            if not client.wait_for_service(timeout_sec=3.0):
                log.warn("/mavros/set_message_interval 不可用，跳过消息流请求")
                return
            for msg_id, rate, name in requests:
                try:
                    req = MessageInterval.Request()
                    req.message_id = msg_id
                    req.message_rate = float(rate)
                    future = client.call_async(req)
                    t0 = time.monotonic()
                    while rclpy.ok() and not future.done() and time.monotonic() - t0 < 3.0:
                        time.sleep(0.02)
                    if future.done() and future.result() is not None:
                        if future.result().success:
                            log.info("  已请求 MAVLink %s (ID %d) @ %.0f Hz" % (name, msg_id, rate))
                        else:
                            log.warn("  请求 %s (ID %d) 被拒绝" % (name, msg_id))
                    else:
                        log.warn("  请求 %s (ID %d) 超时" % (name, msg_id))
                except Exception as exc:
                    log.warn("  请求 %s (ID %d) 失败: %s" % (name, msg_id, exc))
        except Exception as exc:
            log.warn("/mavros/set_message_interval 调用失败: %s" % exc)

    # =======================================================================
    # 相机回调 / 直连相机
    # =======================================================================
    def ci_cb(self, msg):
        with self.camera_info_lock:
            self.latest_camera_info = msg

    def depth_cb(self, msg):
        depth = _decode_image(msg, "32FC1")
        if depth is None:
            return
        with self.depth_lock:
            self.latest_depth = np.array(depth, dtype=np.float32, copy=True)

    def _start_direct_camera(self):
        self._cap_run = True
        self._cap_thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="direct_camera")
        self._cap_thread.start()

    def _capture_loop(self):
        import pyrealsense2 as rs
        # 深度流降级链：配置尺寸 → 彩色同尺寸 → 纯彩色（USB2 相机只支持 640x480）
        depth_tries = []
        if self.direct_depth:
            depth_tries.append((self.depth_w, self.depth_h))
            if (self.depth_w, self.depth_h) != (self.w, self.h):
                depth_tries.append((self.w, self.h))
        depth_tries.append(None)
        pipeline = None
        use_depth = False
        self._depth_size = None
        while self._cap_run and pipeline is None:
            for dep in depth_tries:
                try:
                    p = rs.pipeline()
                    c = rs.config()
                    c.enable_stream(rs.stream.color, self.w, self.h, rs.format.bgr8, self.cam_fps)
                    if dep is not None:
                        c.enable_stream(rs.stream.depth, dep[0], dep[1], rs.format.z16, self.cam_fps)
                    p.start(c)
                    pipeline = p
                    use_depth = dep is not None
                    self._depth_size = dep
                    break
                except Exception as e:
                    self.get_logger().warn(
                        "direct camera start failed (depth=%s): %s"
                        % ("%dx%d" % dep if dep else "off", e))
            if pipeline is None:
                time.sleep(1.0)
        if pipeline is None:
            return
        if use_depth:
            self._align = rs.align(rs.stream.color)
            try:
                self._direct_depth_scale = float(
                    pipeline.get_active_profile().get_device()
                    .first_depth_sensor().get_depth_scale())
            except Exception:
                self._direct_depth_scale = 0.001
        self.get_logger().info(
            "direct camera started: color %dx%d@%d depth %s" % (
                self.w, self.h, self.cam_fps,
                ("%dx%d scale=%.5f" % (self._depth_size[0], self._depth_size[1],
                                       self._direct_depth_scale))
                if use_depth else "off"))
        try:
            prof = pipeline.get_active_profile().get_stream(rs.stream.color)
            intr = prof.as_video_stream_profile().get_intrinsics()
            ci = CameraInfo()
            ci.width, ci.height = int(intr.width), int(intr.height)
            ci.k = [float(intr.fx), 0.0, float(intr.ppx),
                    0.0, float(intr.fy), float(intr.ppy),
                    0.0, 0.0, 1.0]
            ci.header.frame_id = self.camera_frame
            with self.camera_info_lock:
                self.latest_camera_info = ci
        except Exception as exc:
            self.get_logger().warn("direct camera intrinsics failed: %s" % exc)

        while self._cap_run:
            try:
                fs = pipeline.wait_for_frames()
                af = self._align.process(fs) if self._align is not None else fs
                cfr = af.get_color_frame()
                if cfr is None:
                    continue
                frame = np.asanyarray(cfr.get_data())
                dfr = af.get_depth_frame() if self._align is not None else None
            except Exception as e:
                self.get_logger().warn("direct camera frame failed: %s" % e)
                time.sleep(0.5)
                continue
            with self._cap_lock:
                self._seq += 1
                seq = self._seq
                bi = seq % self._nbuf
                try:
                    arr = np.ndarray((self.h, self.w, 3), np.uint8, buffer=self._shms[bi].buf)
                    arr[:] = frame
                except Exception:
                    continue
                if dfr is not None:
                    self._depth_ring[seq] = np.asanyarray(dfr.get_data()).copy()
                    if len(self._depth_ring) > 12:
                        for k in [k for k in self._depth_ring if k <= seq - 8]:
                            self._depth_ring.pop(k, None)
                hdr = Header()
                hdr.stamp = self.get_clock().now().to_msg()
                hdr.frame_id = self.camera_frame
                self._headers[seq] = hdr
                if len(self._headers) > 32:
                    for k in [k for k in self._headers if k <= seq - 16]:
                        self._headers.pop(k, None)
            try:
                self._in_q.put_nowait((bi, seq))
            except Exception:
                try:
                    self._in_q.get_nowait()
                except Exception:
                    pass
                try:
                    self._in_q.put_nowait((bi, seq))
                except Exception:
                    pass
            self._cam_n += 1
        try:
            pipeline.stop()
        except Exception:
            pass

    def image_cb(self, msg):
        if self.bridge is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        self._cam_n += 1
        if self.use_proc and hasattr(self, "_proc") and self._proc.is_alive():
            h, w = frame.shape[:2]
            if (h, w) != (self.h, self.w):
                if not self._size_warned:
                    self._size_warned = True
                    self.get_logger().error(
                        "frame %dx%d != configured %dx%d: w/h 必须与相机分辨率一致"
                        % (w, h, self.w, self.h))
                return
            self._seq += 1
            seq = self._seq
            bi = seq % self._nbuf
            try:
                arr = np.ndarray((h, w, 3), np.uint8, buffer=self._shms[bi].buf)
                arr[:] = frame
            except Exception:
                return
            with self._cap_lock:
                self._headers[seq] = msg.header
                if len(self._headers) > 32:
                    for k in [k for k in self._headers if k <= seq - 16]:
                        self._headers.pop(k, None)
            try:
                self._in_q.put_nowait((bi, seq))
            except Exception:
                try:
                    self._in_q.get_nowait()
                except Exception:
                    pass
                try:
                    self._in_q.put_nowait((bi, seq))
                except Exception:
                    pass
            return
        if self.model is not None:
            t0 = time.perf_counter()
            raw_list = []
            try:
                r = self.model.predict(frame, conf=0.10, imgsz=self.imgsz,
                                       device=self.device, verbose=False)[0]
                raw_list = [(b.xyxy[0].tolist(), float(b.conf[0]), int(b.cls[0]))
                            for b in r.boxes]
            except Exception:
                pass
            self._process_result(frame, msg.header, raw_list,
                                 (time.perf_counter() - t0) * 1000.0, None)
        self._log_fps()

    # =======================================================================
    # 深度 / 内参 / 反投影
    # =======================================================================
    def _get_latest_depth_copy(self):
        with self.depth_lock:
            if self.latest_depth is None:
                return None
            return self.latest_depth.copy()

    def get_camera_info(self):
        with self.camera_info_lock:
            return self.latest_camera_info

    def lookup_depth(self, x, y, depth_img=None):
        """查询 (x, y) 深度（周围中值），支持 float32(m) 与 uint16(mm)。"""
        if depth_img is not None:
            depth = depth_img
            scale = self._direct_depth_scale
            raw = (depth.dtype == np.uint16)
        else:
            with self.depth_lock:
                depth = self.latest_depth
            scale = 1.0
            raw = False
        if depth is None:
            return 0.0
        h, w = depth.shape[:2]
        if x < 0 or y < 0 or x >= w or y >= h:
            return 0.0
        min_valid = 10.0 if raw else 0.01
        for r in (self.depth_patch_radius, 5, 10, 20):
            y0, y1 = max(0, y - r), min(h, y + r + 1)
            x0, x1 = max(0, x - r), min(w, x + r + 1)
            patch = depth[y0:y1, x0:x1]
            valid = patch[np.isfinite(patch) & (patch > min_valid)]
            if valid.size > 0:
                val = float(np.median(valid))
                return val * scale if raw else val
        return 0.0

    def get_center_depth(self):
        with self.depth_lock:
            depth = self.latest_depth
        if depth is None:
            return 0.0
        h, w = depth.shape[:2]
        cx, cy = w // 2, h // 2
        r = 6
        patch = depth[max(0, cy - r):cy + r + 1, max(0, cx - r):cx + r + 1]
        valid = patch[np.isfinite(patch) & (patch > 0.01)]
        if valid.size == 0:
            return 0.0
        return float(np.median(valid))

    def project_pixel(self, x, y, depth_m):
        if depth_m <= 0.0 or not math.isfinite(depth_m):
            return None
        ci = self.get_camera_info()
        if ci is None:
            return None
        fx, fy = float(ci.k[0]), float(ci.k[4])
        cx, cy = float(ci.k[2]), float(ci.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        sign = -1.0 if self.invert_camera_x else 1.0
        return np.array([
            sign * (float(x) - cx) * depth_m / fx,
            (float(y) - cy) * depth_m / fy,
            depth_m,
        ], dtype=np.float32)

    def pixel_to_m(self, pixel_length, depth_m, axis="x"):
        if depth_m <= 0.0 or not math.isfinite(depth_m):
            return 0.0
        ci = self.get_camera_info()
        if ci is None:
            return 0.0
        focal = float(ci.k[0] if axis == "x" else ci.k[4])
        return float(pixel_length) * depth_m / focal if focal > 0.0 else 0.0

    def bucket_size_ratio(self, det):
        try:
            depth = float(det.depth_m)
            width_px = float(det.x_max - det.x_min)
            height_px = float(det.y_max - det.y_min)
            ci = self.get_camera_info()
            if ci is None:
                return None
            fx, fy = float(ci.k[0]), float(ci.k[4])
        except (AttributeError, TypeError, ValueError):
            return None
        if depth <= 0.0 or width_px <= 0.0 or height_px <= 0.0 or fx <= 0.0 or fy <= 0.0:
            return None
        return depth * math.hypot(width_px / fx, height_px / fy)

    def _dropper_delta_m(self, det):
        """相机中心→桶心的原始差值 (dx, dy_cam, depth)，与 auto_drop._fill_aim_slot 同式。

        2026-10-05 标定用：auto_drop 的 dx/dy 直接用像素差*depth/fx（不含 invert_camera_x）。
        """
        ci = self.get_camera_info()
        if ci is None or len(ci.k) < 6 or det is None:
            return None
        try:
            depth = float(det.depth_m)
            cx = float(det.center_x)
            cy = float(det.center_y)
            fx, fy = float(ci.k[0]), float(ci.k[4])
            ccx, ccy = float(ci.k[2]), float(ci.k[5])
        except (AttributeError, TypeError, ValueError):
            return None
        if depth <= 0.1 or fx <= 0.0 or fy <= 0.0:
            return None
        return ((cx - ccx) * depth / fx, (cy - ccy) * depth / fy, depth)

    # =======================================================================
    # 检测消息构建
    # =======================================================================
    @staticmethod
    def _to_numpy(v):
        if hasattr(v, "detach"):
            v = v.detach()
        if hasattr(v, "cpu"):
            try:
                v = v.cpu()
            except Exception:
                pass
        if hasattr(v, "numpy"):
            v = v.numpy()
        arr = np.asarray(v)
        return arr.copy() if arr.ndim == 0 else arr

    def empty_detection(self, header):
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

    def build_single(self, header, box, image_shape, depth_img=None):
        det = self.empty_detection(header)
        xyxy = self._to_numpy(box.xyxy)
        if xyxy.ndim > 1:
            xyxy = xyxy[0]
        x_min, y_min, x_max, y_max = [int(round(v)) for v in xyxy]
        cx = int(round((x_min + x_max) * 0.5))
        cy = int(round((y_min + y_max) * 0.5))
        cls_arr = self._to_numpy(box.cls).reshape(-1)
        conf_arr = self._to_numpy(box.conf).reshape(-1)
        class_id = int(cls_arr[0]) if cls_arr.size else 0
        confidence = float(conf_arr[0]) if conf_arr.size else 0.0
        class_name = str(self.class_name_map.get(class_id, class_id))
        h, w = image_shape[:2]
        cx = max(0, min(w - 1, cx))
        cy = max(0, min(h - 1, cy))
        depth_m = self.lookup_depth(cx, cy, depth_img)
        if depth_m <= 0.0:
            # 20261005_043621 实据：瞄准 FINE 段框中心深度偶发无效 → position_valid=false，
            # 控制侧反复"检测到桶但深度…无效"拒绝投放。中心无效时在框内四向采样兜底。
            fw = max(2.0, (x_max - x_min) / 4.0)
            fh = max(2.0, (y_max - y_min) / 4.0)
            candidates = []
            for ox, oy in ((fw, 0.0), (-fw, 0.0), (0.0, fh), (0.0, -fh)):
                alt = self.lookup_depth(int(round(cx + ox)), int(round(cy + oy)),
                                        depth_img)
                if alt > 0.0:
                    candidates.append(alt)
            if candidates:
                depth_m = float(np.median(candidates))
                _throttle_log(
                    self, 2.0,
                    "检测框中心深度无效，改用框内邻近采样 %.2fm（bbox=%d,%d,%d,%d）"
                    % (depth_m, x_min, y_min, x_max, y_max))
            else:
                _throttle_log(
                    self, 2.0,
                    "检测框深度无效（中心+邻近采样均无有效值，bbox=%d,%d,%d,%d）："
                    "检查深度流/反光/遮挡" % (x_min, y_min, x_max, y_max))
        position = self.project_pixel(cx, cy, depth_m)
        det.detected = True
        det.class_id = class_id
        det.class_name = class_name
        det.confidence = confidence
        det.x_min, det.y_min = x_min, y_min
        det.x_max, det.y_max = x_max, y_max
        det.center_x, det.center_y = cx, cy
        det.depth_m = depth_m
        if position is not None:
            det.position_valid = True
            det.camera_x_m = float(position[0])
            det.camera_y_m = float(position[1])
            det.camera_z_m = float(position[2])
            det.distance_m = float(np.linalg.norm(position))
            det.bbox_width_m = self.pixel_to_m(x_max - x_min, depth_m, "x")
            det.bbox_height_m = self.pixel_to_m(y_max - y_min, depth_m, "y")
        return det

    # =======================================================================
    # 地理坐标变换
    # =======================================================================
    def is_valid_global(self, msg):
        if msg is None:
            return False
        if getattr(msg.status, "status", NavSatStatus.STATUS_NO_FIX) < 0:
            return False
        return all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude))

    def transform_camera_to_body(self, detection):
        cam_x = detection.camera_x_m
        cam_y = detection.camera_y_m
        cam_z = detection.camera_z_m
        result = PointStamped()
        result.header.frame_id = self.body_frame
        result.header.stamp = self.get_clock().now().to_msg()
        result.point.x = -cam_y
        result.point.y = cam_x
        result.point.z = cam_z + self.camera_mount_z
        return result

    def _make_base_geo_target(self, detection):
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
        if self.geo_pub is None:
            return
        target.status = status
        if self.publish_invalid:
            self.geo_pub.publish(target)

    def _compute_geo(self, detection):
        with self.fc_lock:
            latest_global = self.latest_global_msg
            latest_local_pose = self.latest_local_pose_msg
        if not detection.detected:
            return "no_detection", None, None, None, None, None
        if detection.confidence < self.geo_min_confidence:
            return "low_confidence", None, None, None, None, None
        if not detection.position_valid:
            return "invalid_camera_position", None, None, None, None, None
        body_point = self.transform_camera_to_body(detection)
        if body_point is None:
            return "tf_camera_to_body_failed", None, None, None, None, None
        bx = float(body_point.point.x)
        by = float(body_point.point.y)
        bz = float(body_point.point.z)
        body = (bx, by, bz)
        if latest_local_pose is None:
            return "no_local_pose", body, None, None, None, None
        east_m, north_m, up_m = quaternion_rotate_vector(
            latest_local_pose.pose.orientation, (bx, by, -bz))
        enu = (float(east_m), float(north_m), float(up_m))
        drone_n = float(latest_local_pose.pose.position.y)
        drone_e = float(latest_local_pose.pose.position.x)
        drone_d = float(-latest_local_pose.pose.position.z)
        d_n = float(north_m)
        d_e = float(east_m)
        d_d = float(-up_m)
        bucket_ned = (drone_n + d_n, drone_e + d_e, drone_d + d_d)
        offset_ned = (d_n, d_e, d_d)
        if not self.is_valid_global(latest_global):
            return "no_valid_global_position", body, enu, bucket_ned, offset_ned, None
        target_wgs84 = enu_offset_to_geodetic(
            latest_global.latitude, latest_global.longitude, latest_global.altitude,
            east_m, north_m, up_m)
        return "ok", body, enu, bucket_ned, offset_ned, target_wgs84

    def _dual_solution(self, detection, body, dx):
        if body is None:
            return None, None, None
        with self.fc_lock:
            latest_global = self.latest_global_msg
            latest_local_pose = self.latest_local_pose_msg
        bx, by, bz = body
        rx, ry, rz = bx - dx, by, bz
        cam = (float(ry), float(-rx), float(rz - self.camera_mount_z))
        if latest_local_pose is None:
            return cam, None, None
        east, north, up = quaternion_rotate_vector(
            latest_local_pose.pose.orientation, (rx, ry, -rz))
        drone_n = float(latest_local_pose.pose.position.y)
        drone_e = float(latest_local_pose.pose.position.x)
        drone_d = float(-latest_local_pose.pose.position.z)
        ned = (drone_n + float(north), drone_e + float(east), drone_d + float(-up))
        wgs = None
        if self.is_valid_global(latest_global):
            wgs = enu_offset_to_geodetic(
                latest_global.latitude, latest_global.longitude, latest_global.altitude,
                float(east), float(north), float(up))
        return cam, ned, wgs

    def _publish_geo_targets(self, detections_msg):
        if self.geo_pub is None:
            return
        detections = [d for d in detections_msg.detections if d.detected]
        detections.sort(key=lambda d: (-float(d.confidence), int(d.center_x)))
        count = len(detections)
        if count == 0:
            target = GeoTarget()
            target.header = detections_msg.header
            target.source_topic = "/vision/yolo/detection"
            target.target_id = 0
            target.count = 0
            self._publish_geo_status(target, "no_detection")
            return
        for idx, det in enumerate(detections[:max(0, self.max_geo_targets)]):
            status, body, enu, _bn, _on, wgs84 = self._compute_geo(det)
            target = self._make_base_geo_target(det)
            target.target_id = idx + 1
            target.count = count
            ratio = self.bucket_size_ratio(det)
            if ratio is not None:
                target.size_proxy_m = float(ratio)
            target.status = status
            target.valid = (status == "ok" and wgs84 is not None)
            if body is not None:
                target.body_x_m = body[0]
                target.body_y_m = body[1]
                target.body_z_m = body[2]
            if enu is not None:
                target.enu_east_m = enu[0]
                target.enu_north_m = enu[1]
                target.enu_up_m = enu[2]
            cam_f, ned_f, wgs_f = self._dual_solution(det, body, +self.dual_offset_m)
            cam_r, ned_r, wgs_r = self._dual_solution(det, body, -self.dual_offset_m)
            if cam_f is not None:
                target.front_cam_x_m, target.front_cam_y_m, target.front_cam_z_m = cam_f
            if cam_r is not None:
                target.rear_cam_x_m, target.rear_cam_y_m, target.rear_cam_z_m = cam_r
            if ned_f is not None:
                target.front_ned_n_m, target.front_ned_e_m, target.front_ned_d_m = ned_f
            if ned_r is not None:
                target.rear_ned_n_m, target.rear_ned_e_m, target.rear_ned_d_m = ned_r
            if wgs_f is not None:
                target.front_latitude, target.front_longitude, target.front_altitude = wgs_f
            if wgs_r is not None:
                target.rear_latitude, target.rear_longitude, target.rear_altitude = wgs_r
            if target.valid:
                target.latitude = float(wgs84[0])
                target.longitude = float(wgs84[1])
                target.altitude = float(wgs84[2])
                self.geo_pub.publish(target)
            elif self.publish_invalid:
                self.geo_pub.publish(target)

    def _enu_abs(self, enu_offset):
        if enu_offset is None:
            return None
        with self.fc_lock:
            latest_local_pose = self.latest_local_pose_msg
        if latest_local_pose is None:
            return None
        return (
            float(latest_local_pose.pose.position.x) + float(enu_offset[0]),
            float(latest_local_pose.pose.position.y) + float(enu_offset[1]),
            float(latest_local_pose.pose.position.z) + float(enu_offset[2]),
        )

    def _build_bucket_estimates(self, header, detections_msg):
        estimates = BucketEstimates()
        estimates.header = header
        detections = [d for d in detections_msg.detections if d.detected]
        detections.sort(key=lambda d: (-float(d.confidence), int(d.center_x)))
        estimates.count = len(detections)
        targets = []
        for det in detections[:3]:
            t = BucketTarget()
            t.valid = bool(det.detected and det.position_valid)
            t.confidence = float(det.confidence)
            t.center_x = int(det.center_x)
            t.center_y = int(det.center_y)
            t.depth_m = float(det.depth_m)
            dw = float(det.x_max - det.x_min)
            dh = float(det.y_max - det.y_min)
            t.bbox_diagonal_px = float(math.hypot(dw, dh)) if (dw > 0.0 and dh > 0.0) else 0.0
            t.bbox_width_m = float(det.bbox_width_m)
            t.bbox_height_m = float(det.bbox_height_m)
            t.ratio = (float(det.bbox_width_m) / float(det.bbox_height_m)
                       if det.bbox_height_m > 0.0 else 0.0)
            _status, _body, enu_offset, _bn, _on, wgs84 = self._compute_geo(det)
            enu_abs = self._enu_abs(enu_offset)
            if enu_abs is not None:
                t.enu_east_m = enu_abs[0]
                t.enu_north_m = enu_abs[1]
                t.enu_up_m = enu_abs[2]
            if wgs84 is not None:
                t.latitude = float(wgs84[0])
                t.longitude = float(wgs84[1])
                t.altitude = float(wgs84[2])
            targets.append(t)
        estimates.targets = targets
        return estimates

    # =======================================================================
    # 结果处理
    # =======================================================================
    def _on_drain_timer(self):
        self._drain_results()
        self._log_fps()

    def _drain_results(self):
        latest = None
        while True:
            try:
                res = self._out_q.get_nowait()
            except Exception:
                break
            if not res:
                continue
            tag = res[0]
            if tag == "names":
                self.names = res[1]
                self._build_class_name_map()
                continue
            if tag == "error":
                self.get_logger().warn("infer worker error: %s"
                                       % (res[1] if len(res) > 1 else "?"))
                continue
            try:
                seq, boxes, ms = res
            except Exception:
                continue
            if seq > self._last_drawn_seq:
                self._last_drawn_seq = seq
                latest = (seq, boxes, ms)
        if latest is None:
            return
        seq, boxes, ms = latest
        with self._cap_lock:
            header = self._headers.pop(seq, None)
            depth_img = self._depth_ring.pop(seq, None) if self.direct_camera else None
        if header is None:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.camera_frame
        bi = seq % self._nbuf
        try:
            frame = np.ndarray((self.h, self.w, 3), np.uint8, buffer=self._shms[bi].buf)
        except Exception:
            frame = None
        self._process_result(frame, header, boxes, ms, depth_img)

    def _process_result(self, frame, header, raw_list, inf_ms, depth_img=None):
        self._last_ms = inf_ms
        st = self.stab.update(raw_list, time.time())
        dets_msg = YoloDetections()
        dets_msg.header = header
        best = None
        best_conf = -1.0
        best_live = None
        best_live_conf = -1.0
        filtered_names = set()
        track_dets = {}
        for t in st:
            box = _StubBox(t["box"], int(t.get("cls", 0)), float(t["conf"]))
            det = self.build_single(header, box, (self.h, self.w, 3), depth_img)
            track_dets[id(t)] = det
            if t.get("held"):
                # hold 框是旧位置（≤hold 时间），却用当前帧深度采样，会产出
                # "中心冻结/深度漂移"的假观测（20261005_202604 实据：FINAL hold 中心
                # 完全相同占 26%，低 conf 帧被控制侧拒收后目标闪烁、判据永远不稳）。
                # 对下游统一按"本帧无检测"发布；标注画面仍按 st 画灰框，不受影响。
                det.detected = False
                det.position_valid = False
            if not self._is_target_class(det.class_name):
                filtered_names.add(det.class_name)
                continue
            dets_msg.detections.append(det)
            if det.confidence > best_conf:
                best_conf = det.confidence
                best = det
            if not t.get("held") and det.confidence > best_live_conf:
                best_live_conf = det.confidence
                best_live = det
        if filtered_names and not dets_msg.detections:
            _throttle_log(
                self, 5.0,
                "detected %d object(s) but all filtered. classes=%s targets=%s"
                % (len(st), list(filtered_names), self.target_classes))

        best_msg = best if best is not None else self.empty_detection(header)
        self.result_pub.publish(best_msg)
        self.results_pub.publish(dets_msg)

        binfo = BucketInfo()
        binfo.header = header
        # hold 框按本帧无检测发布，计数只统计 detected=True 的实时框。
        binfo.count = sum(1 for d in dets_msg.detections if d.detected)
        ref = best_live if best_live is not None else best
        if ref is not None:
            binfo.delta_x = float(ref.center_x - self.w // 2)
            binfo.delta_y = float(ref.center_y - self.h // 2)
        else:
            binfo.delta_x = binfo.delta_y = 0.0
        self.bucket_info_pub.publish(binfo)
        self.bucket_estimates_pub.publish(self._build_bucket_estimates(header, dets_msg))
        self._publish_geo_targets(dets_msg)

        want_annotated = (self.publish_annotated and self.bridge is not None
                          and self.annotated_pub.get_subscription_count() > 0)
        if frame is not None and (self.show_window or want_annotated):
            for t in st:
                x1, y1, x2, y2 = [int(v) for v in t["box"]]
                color = (0, 255, 0) if not t.get("held") else (140, 140, 140)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                # 2026-10-05 用户要求：TRT 标注也画识别框中心点（红点，与发布坐标同源）
                track_det = track_dets.get(id(t))
                if track_det is not None and track_det.detected:
                    cv2.circle(frame,
                               (int(track_det.center_x), int(track_det.center_y)),
                               4, (0, 0, 255), -1)
                cv2.putText(frame, "%.2f" % t["conf"], (x1, max(y1 - 4, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                # 2026-10-05 标定用：每个框旁直接打尺寸代理比值
                # ratio = depth*sqrt((w/fx)^2+(h/fy)^2)，与 auto_drop._bucket_ratio 同式。
                ratio = self.bucket_size_ratio(track_det) if track_det is not None else None
                if ratio is not None:
                    cv2.putText(frame, "r=%.3f" % ratio,
                                (x1, min(y2 + 20, self.h - 4)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # 标定读数：最高置信度实时目标的最近 30 帧中位数（消除单帧抖动）
            best_live_ratio = (self.bucket_size_ratio(best_live)
                               if best_live is not None else None)
            if best_live_ratio is not None:
                self._cal_ratio_hist.append(float(best_live_ratio))
            if self._cal_ratio_hist:
                ordered = sorted(self._cal_ratio_hist)
                ratio_med = ordered[len(ordered) // 2]
                cv2.putText(
                    frame,
                    "bucket_ratio_med=%.3f (n=%d, last 1s)"
                    % (ratio_med, len(self._cal_ratio_hist)),
                    (20, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            # 2026-10-05 抛投器对准标定：相机中心→桶心原始差值（与 auto_drop 判据同式）
            # + 换算到前/后准星的差值。物理对准抛投器后读 A(front) 的 dy 即实际偏差：
            #   A_dy = dy_cam + dual_offset；若 A_dy>0 则真实前偏移 < dual_offset。
            cal = None
            if (best_live is not None and bool(getattr(best_live, "position_valid", False))
                    and float(getattr(best_live, "depth_m", 0.0)) > 0.1):
                cal = best_live
            elif (ref is not None and bool(getattr(ref, "position_valid", False))
                    and float(getattr(ref, "depth_m", 0.0)) > 0.1):
                cal = ref
            delta = self._dropper_delta_m(cal) if cal is not None else None
            if delta is not None:
                self._cal_delta_hist.append((delta[0], delta[1]))
            if self._cal_delta_hist and cal is not None:
                xs = sorted(v[0] for v in self._cal_delta_hist)
                ys = sorted(v[1] for v in self._cal_delta_hist)
                dx_med = xs[len(xs) // 2]
                dy_med = ys[len(ys) // 2]
                off = float(self.dual_offset_m)
                cal_lines = (
                    "CAL depth=%.2fm center=(%d,%d) n=%d"
                    % (float(cal.depth_m), int(cal.center_x), int(cal.center_y),
                       len(self._cal_delta_hist)),
                    "A(front %+.0fmm): dx=%+.0fmm dy=%+.0fmm"
                    % (off * 1000.0, dx_med * 1000.0, (dy_med + off) * 1000.0),
                    "B(rear  %+.0fmm): dx=%+.0fmm dy=%+.0fmm"
                    % (-off * 1000.0, dx_med * 1000.0, (dy_med - off) * 1000.0),
                )
                for i, text in enumerate(cal_lines):
                    cv2.putText(frame, text, (20, 72 + 26 * i),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            # 2026-10-05 用户拍板几何：相机在机腹中心，A/B 抛投器中心在相机前后各 5cm
            # （dual_offset_m=0.05）。相机系 y 向下 ⇒ 前投 A 在图像上方(cy-off)、
            # 后投 B 在下方(cy+off)，随目标深度投影。TRT 标注画面此前缺这两个准星。
            cross_depth = None
            if (ref is not None and bool(getattr(ref, "position_valid", False))
                    and float(getattr(ref, "depth_m", 0.0)) > 0.1):
                cross_depth = float(ref.depth_m)
            elif depth_img is not None:
                cross_depth = self.lookup_depth(self.w // 2, self.h // 2, depth_img)
            self._draw_aim_crosshairs(frame, cross_depth)
        if frame is not None and self.show_window:
            with self._disp_lock:
                self._disp_frame = frame.copy()
            self._disp_evt.set()
        if frame is not None and want_annotated:
            self.annotated_pub.publish(self._make_img_msg(frame, "bgr8"))
        self._fps_n += 1

    def _draw_aim_crosshairs(self, image, depth_m):
        """画 A/B 抛投器准星（相机腹部中心为原点，前后各 dual_offset_m）。

        相机系 y 向下，机体系 X(前) = -相机 y ⇒ 前投 A 在图像上方 cy-off、
        后投 B 在下方 cy+off；像素偏移 = fy * dual_offset_m / depth。
        不画文字标签（用户 2026-10-05），靠颜色与上下列区分：上=前 A 紫、下=后 B 橙。
        """
        ci = self.get_camera_info()
        if ci is None or len(ci.k) < 6:
            return
        try:
            depth = float(depth_m)
        except (TypeError, ValueError):
            return
        if not math.isfinite(depth) or depth <= 0.1:
            return
        cx, cy, fy = float(ci.k[2]), float(ci.k[5]), float(ci.k[4])
        if fy <= 0.0:
            return
        off = fy * float(self.dual_offset_m) / depth
        height = image.shape[0]
        for iy, color in ((cy - off, (255, 0, 255)),
                          (cy + off, (0, 165, 255))):
            iy = int(round(iy))
            ix = int(round(cx))
            if iy < 4 or iy > height - 5:
                continue
            cv2.line(image, (ix - 12, iy), (ix + 12, iy), color, 1)
            cv2.line(image, (ix, iy - 8), (ix, iy + 8), color, 1)

    def _display_loop(self):
        """独立线程刷新识别窗口（imshow 阻塞不影响发布/解算）。"""
        while self._disp_run:
            if not self._disp_evt.wait(timeout=0.5):
                continue
            self._disp_evt.clear()
            with self._disp_lock:
                frame = self._disp_frame
                self._disp_frame = None
            if frame is None:
                continue
            try:
                # 缩小后再显示：1080p 原图 imshow 在 X/远程桌面下会长时间持 GIL
                h, w = frame.shape[:2]
                if w > 960:
                    scale = 960.0 / w
                    frame = cv2.resize(frame, (960, max(1, int(round(h * scale)))),
                                       interpolation=cv2.INTER_AREA)
                cv2.imshow(self.window_name, frame)
                cv2.waitKey(1)
            except Exception as exc:
                self.get_logger().warn("cv2.imshow 失败，识别 UI 降级: %s" % exc)
                self._disp_run = False
                return

    def _make_img_msg(self, img, encoding="bgr8"):
        if not self.bridge:
            return Image()
        try:
            return self.bridge.cv2_to_imgmsg(img, encoding=encoding)
        except Exception:
            return Image()

    def _log_fps(self):
        nowt = time.time()
        dt = nowt - self._fps_t0
        if dt >= 1.0:
            self._fps_disp = self._fps_n / dt
            self._cam_disp = self._cam_n / dt
            self._fps_t0 = nowt
            self._fps_n = 0
            self._cam_n = 0
            self.get_logger().info(
                "detector FPS %.1f (cam %.1f, infer %.1fms)"
                % (self._fps_disp, self._cam_disp, self._last_ms))
            self._publish_status()

    def _publish_status(self):
        try:
            payload = {
                "engine": os.path.basename(self.model_path),
                "imgsz": int(self.imgsz),
                "fps": round(float(self._fps_disp), 1),
                "cam_fps": round(float(self._cam_disp), 1),
                "infer_ms": round(float(self._last_ms), 1),
                "use_proc": bool(self.use_proc),
                "direct_camera": bool(self.direct_camera),
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.status_pub.publish(msg)
        except Exception:
            pass
        if self.direct_camera and self.cam_info_pub is not None:
            with self.camera_info_lock:
                ci = self.latest_camera_info
            if ci is not None:
                try:
                    ci.header.stamp = self.get_clock().now().to_msg()
                    self.cam_info_pub.publish(ci)
                except Exception:
                    pass

    # =======================================================================
    def destroy_node(self):
        try:
            self._disp_run = False
            self._disp_evt.set()
            if self._disp_thread is not None and self._disp_thread.is_alive():
                self._disp_thread.join(timeout=1.0)
        except Exception:
            pass
        try:
            self._cap_run = False
            if self._cap_thread is not None and self._cap_thread.is_alive():
                self._cap_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            if hasattr(self, '_stop_evt'):
                self._stop_evt.set()
            if hasattr(self, '_proc') and self._proc.is_alive():
                try:
                    self._in_q.put(None)
                except Exception:
                    pass
                self._proc.join(timeout=2)
        except Exception:
            pass
        try:
            if hasattr(self, '_shms'):
                for s in self._shms:
                    s.close()
                    try:
                        s.unlink()
                    except Exception:
                        pass
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TRTDetectorNode()
    try:
        if node.auto_start_mavros:
            _ensure_mavros(node, node.mavros_fcu_url)
        elif not _HAS_MAVROS_MSGS:
            node.get_logger().warn("mavros_msgs 不可用：GeoTarget 只能给到机体系/相机系")
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
