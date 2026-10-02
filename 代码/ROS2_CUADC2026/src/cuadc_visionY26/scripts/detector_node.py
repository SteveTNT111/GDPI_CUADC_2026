#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
detector_node.py — CUADC 2026 YOLO 目标检测节点（YOLO26 ONNX 版）
ROS1 → ROS2 (Humble) 翻译版；本副本（cuadc_visionY26）用于加载 YOLO26 ONNX 权重。
与 cuadc_vision 版本共用同一套消息类型 / 话题名 / UI，控制脚本无需改动。

原文件：代码/比赛main工作版本/src/cuadc_vision/scripts/detector_node.py (1725 行)

功能：
  1. 自动检测并启动 MAVROS（如未运行；ROS2 无 roscore，此逻辑删除）
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
  - /vision/bucket/estimates (BucketEstimates) — 桶数量 + 最多 3 个目标独立 WGS84/ENU
  - /vision/annotated_image (sensor_msgs/Image) — 标注画面
  - /vision/target_global   (GeoTarget, 可选) — 每帧每个目标一条（最多 3 条）：
      target_id=1/2/3（按置信度降序，1=最佳）、count=本帧桶数、
      WGS84(latitude/longitude/altitude) 经 相机→机体→ENU→WGS84 反投影。

T1 多目标输出契约（识别节点只输出，不做平滑/估计/记录）：
  * 每帧在 output_topic（默认 /vision/target_global）上对每个通过类别过滤的目标
    发布一条 GeoTarget，最多 max_geo_targets（默认 3）条。
  * 编号规则：按 confidence 降序，target_id=1 为置信度最高者（与既有
    /vision/yolo/detection 的"最佳目标"一致）；同置信度按 center_x 升序。
  * count 恒为本帧全部目标的个数（可 >3）；无目标帧 count=0 且 target_id=0，
    仅在 publish_invalid=true 时发一条 valid=false,status="no_detection"。
  * 既有字段（header/valid/status/class_name/.../altitude）语义不变，
    单目标消费者取首条即等价于旧的"最佳目标"。

ROS2 翻译要点
--------------
1. rospy → rclpy：参数 `~x` → declare_parameter（node_param 辅助函数）；
   日志 `rospy.loginfo("fmt %s", a)` → `logger.info("fmt %s" % a)`（rclpy 不支持
   printf 式多参，会抛 TypeError，见错题本 #8）。
2. 无 roscore：ROS2 无需 master，相关自检/启动逻辑删除。
3. MAVROS 自动启动：`roslaunch mavros apm.launch` → `ros2 launch mavros apm.launch.py
   fcu_url:=...`；检测改用 count_publishers("/mavros/state") + DDS 发现宽限窗口
   （mavros_discovery_grace，修复 A3 的 DDS 异步竞态，见错题本 #9）。
4. 服务调用：`rospy.ServiceProxy + wait_for_service` → create_client + call_async
   + 后台轮询 future（MultiThreadedExecutor 在 main 线程 spin）。
5. 图像 QoS：RELIABLE(depth=1)，与原 queue_size=1 语义一致；飞控话题
   BEST_EFFORT(depth=10)。
6. 时间戳：rospy.Time.now() → node.get_clock().now()；消息内时间用 .to_msg()。
7. CameraInfo：ROS2 字段小写 k/d/r/p（错题本 #7）。
8. _make_img_msg：uint8[] 赋值必须 array.array('B', arr.tobytes())，step 用
   arr.strides[0]（错题本 #6/#7）——避免 92KB 图像 ≈52ms/帧的慢路径。
9. 图像解码不依赖 cv_bridge（本机虽装有，但手工解码更可控），支持 bgr8/32FC1。
10. 新增 stub_detector 参数（默认 false）：真机/真权重路径完全保持原样；
    仿真无 best.pt 权重时置 true，用深度图突起分割在真实相机图像上找圆筒
    （投放区/侦察区筒是地面 15~30cm 高的圆柱突起，深度比周围近）
    输出与 ultralytics box 相同的接口（xyxy/cls/conf），下游管线全部复用。
"""

import glob
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time
import array as _array

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import BatteryState, CameraInfo, Image, NavSatFix, NavSatStatus

from cuadc_vision.msg import (
    YoloDetection, YoloDetections, BucketInfo, BucketEstimates, BucketTarget,
    MissionStatus, GeoTarget,
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


# ---------------------------------------------------------------------------
# QoS 常量
# ---------------------------------------------------------------------------
# 图像类：RELIABLE(depth=1) —— 与原 ROS1 queue_size=1 "只留最新一帧"语义一致，
# 且兼容 camera_node 的 RELIABLE 发布（错题本见 B1）。
IMAGE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
# 飞控导航类话题：MAVROS 以 BEST_EFFORT 发布；BEST_EFFORT 订阅对 RELIABLE 与
# BEST_EFFORT 发布方均兼容（/mavros/state 为 RELIABLE）。
NAV_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
# 通用消息话题
MSG_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
# 发布 QoS
PUB_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


# ---------------------------------------------------------------------------
# 小工具
# ---------------------------------------------------------------------------

def node_param(node, name, default):
    """声明并读取节点参数（等价于 ROS1 的 rospy.get_param('~name', default)）。
    只声明一次，后续直接读取；launch / --ros-args -p 传入同名参数即可覆盖。
    """
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    try:
        return node.get_parameter(name).value
    except Exception:
        return default


def get_bool_param(node, name, default):
    """解析 ROS 参数为 bool（兼容字符串 "true"/"1" 等写法）"""
    value = node_param(node, name, default)
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


# 节流日志共享状态（按格式化字符串分组）
_throttle_state = {}


def _throttle_log(node, interval_s, fmt, *args, level="warn"):
    """等价于 rospy.logxxx_throttle。level ∈ {"info","warn","error"}。"""
    now = time.monotonic()
    last = _throttle_state.get(fmt, 0.0)
    if now - last >= interval_s:
        _throttle_state[fmt] = now
        text = fmt % args if args else fmt
        getattr(node.get_logger(), level)(text)


def _stamp_is_zero(stamp):
    """builtin_interfaces/Time 是否为零时间（等价 rospy.Time(0) 比较）。"""
    return stamp.sec == 0 and stamp.nanosec == 0


def _decode_image(msg, encoding):
    """sensor_msgs/Image → np.ndarray（手工解码，不依赖 cv_bridge）。
    支持 bgr8 / 32FC1。解码失败返回 None。
    """
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


# ---------------------------------------------------------------------------
# 仿真桩检测（仅 stub_detector=true 时启用）
# ---------------------------------------------------------------------------

class _FakeTensor(np.ndarray):
    """带上 .detach()/.cpu()/.numpy()/.item() 的 ndarray 子类，
    用于桩检测输出与 ultralytics torch 张量相同的接口
    （向下兼容 build_single 里 box.xyxy[0].detach().cpu().numpy() 等调用）。"""

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

    def item(self):
        return float(np.asarray(self).item())


class _StubBox:
    """桩检测的单个框：xyxy/cls/conf，接口对齐 ultralytics box。"""

    def __init__(self, xyxy, cls, conf):
        self.xyxy = _FakeTensor(xyxy)
        self.cls = _FakeTensor([cls])
        self.conf = _FakeTensor([conf])


class _StubResult:
    """桩检测的单帧结果（对齐 yolo_result，含 .boxes）。"""

    def __init__(self, boxes):
        self.boxes = boxes


class StubDetector:
    """仿真桩检测器（stub_detector=true 时替代 ultralytics YOLO）。

    真机/真权重路径不经过本类。仿真无权重时，用深度图突起分割在真实相机
    图像上找出场地的圆筒目标：投放/侦察筒是地面上 15~30cm 高的圆柱突起，
    在深度图上比周围地面近 ~0.15~0.3m；对深度图取局部均值，与像素自身
    相减，>阈值即为突起，再做形态学 + 连通域过滤。

    输出与 ultralytics 相同的接口，下游 build_single / build_results 完全复用；
    因此桩模式下同样能验证"检测框 → 深度 → 反投影 → 机体系 → ENU → WGS84"
    整条真实管线。
    """

    def __init__(self, detector, ksize=21, min_gap_m=0.08):
        self.detector = detector
        self.ksize = int(ksize)
        self.min_gap_m = float(min_gap_m)

    def __call__(self, frame, conf=None, imgsz=None, device=None, verbose=None):
        depth = self.detector._get_latest_depth_copy()
        boxes = []
        if depth is not None and depth.size > 0:
            boxes = self._detect_cylinders(depth)
        boxes = boxes[:12]
        return [_StubResult(boxes)]

    def _detect_cylinders(self, depth):
        d = np.asarray(depth, dtype=np.float32)
        valid = np.isfinite(d) & (d > 0.05)
        work = np.where(valid, d, 0.0).astype(np.float32)

        # 局部均值 ≈ 周围（地面）深度；突起 = 比周围近超过阈值
        blur = cv2.blur(work, (self.ksize, self.ksize))
        gap = blur - work
        mask = ((gap > self.min_gap_m) & valid).astype(np.uint8) * 255

        # 清理：开/闭运算去掉噪点；轻腐蚀去掉图像边缘/无效区边界误检
        k = 3
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        erode_k = 1  # 筒在图上仅 ~21px 宽，重腐蚀会将其蚀没（见 debug_stub）
        ek = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (erode_k, erode_k))
        mask = cv2.erode(mask, ek)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            if w < 6 or h < 6:
                continue
            area = float(cv2.contourArea(c))
            if area < 40.0:
                continue
            if w * h > 0 and area / (w * h) < 0.25:
                continue  # 形状太碎/太薄
            cx, cy = int(x + w * 0.5), int(y + h * 0.5)
            cy_c = max(0, min(d.shape[0] - 1, cy))
            cx_c = max(0, min(d.shape[1] - 1, cx))
            if not (np.isfinite(d[cy_c, cx_c]) and d[cy_c, cx_c] > 0.05):
                continue
            boxes.append(_StubBox([x, y, x + w, y + h], 0, 0.90))
        return boxes


# ---------------------------------------------------------------------------
# 原生推理后端（绕过 ultralytics predict 包装层；默认关闭）
# ---------------------------------------------------------------------------

class NativeDetector:
    """原生 ONNX Runtime / OpenVINO 推理后端（可选；infer_backend!=ultralytics 时启用）。

    动机（本机实测，见 docs/YOLO26_ONNX_CPU部署与验证.md "帧率优化实测"）：
      ultralytics 的 predict 包装层每帧额外开销 ~65–70ms（640 端到端 106ms vs
      原生 39ms），且其 ONNX 后端把 provider 硬编码为 CPUExecutionProvider、
      session_options=None（无法调线程/关自旋）。本类直接用 ORT/OV 引擎，
      可：调 intra_op 线程、关 ORT 自旋（避免 1000% CPU 空转）、走 OpenVINO IR/INT8。

    精度对齐：预处理（letterbox→RGB→/255→NCHW）与后处理（end2end 直取 /
    原始头 NMS + 反 letterbox 缩放回原图坐标）均按 ultralytics 语义实现，
    输出接口与 ultralytics 一致（.boxes[].xyxy/.cls/.conf，复用 _StubBox）。
    """

    def __init__(self, model_path, backend="ort", device="cpu",
                 threads=6, providers=("CPUExecutionProvider",), log=None):
        self.model_path = os.path.expanduser(model_path)
        self.backend = backend
        self.device = device
        self.threads = int(threads)
        self.providers = list(providers)
        self.log = log
        self.names = {}
        self.imgsz = None
        self._session = None       # ORT
        self._compiled = None      # OpenVINO
        self._ov_input = None
        self._load()

    # ---- 加载 ----
    def _log(self, level, msg):
        if self.log is not None:
            getattr(self.log, level)(msg)

    def _load(self):
        if self.backend == "ort":
            import onnxruntime as ort
            so = ort.SessionOptions()
            so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            if self.threads > 0:
                so.intra_op_num_threads = self.threads
                so.inter_op_num_threads = 1
            # 关闭 ORT 自旋等待：否则空转线程会吃满全部核心并饿死相机（实测 1007% CPU）
            for k, v in (("session.intra_op.allow_spinning", "0"),
                         ("session.inter_op.allow_spinning", "0")):
                try:
                    so.add_session_config_entry(k, v)
                except Exception:
                    pass
            self._session = ort.InferenceSession(
                self.model_path, sess_options=so, providers=self.providers)
            self._input_name = self._session.get_inputs()[0].name
            shape = self._session.get_inputs()[0].shape
            h = shape[2] if len(shape) > 3 else None
            self.imgsz = h if isinstance(h, int) and h > 0 else None
            self.names = self._read_onnx_names()
        elif self.backend == "openvino":
            import openvino as ov
            core = ov.Core()
            # 设备名归一化（节点 device 默认 "cpu" → OpenVINO 需 "CPU"）
            dev = {"cpu": "CPU", "gpu": "GPU", "auto": "AUTO"}.get(
                str(self.device).strip().lower(), str(self.device).strip().upper())
            mp = self.model_path
            if os.path.isdir(mp):
                xmls = sorted(glob.glob(os.path.join(mp, "*.xml")))
                if not xmls:
                    raise RuntimeError("no .xml found in OpenVINO IR dir: %s" % mp)
                mp = xmls[0]
            model = core.read_model(mp)
            cfg = {}
            if self.threads > 0:
                cfg["INFERENCE_NUM_THREADS"] = int(self.threads)
            core.set_property(dev, cfg) if cfg else None
            self._compiled = core.compile_model(model, dev)
            inp = self._compiled.input(0)
            self._ov_input = inp.any_name
            shp = inp.shape
            h = shp[2] if len(shp) > 3 else None
            self.imgsz = h if isinstance(h, int) and h > 0 else None
            self.names = self._read_onnx_names()
        else:
            raise ValueError("unknown native backend: %s" % self.backend)

    def _read_onnx_names(self):
        """读取类别名：ONNX 元数据 names，或 OpenVINO IR 目录里的 metadata.yaml。"""
        mp = self.model_path
        # OpenVINO IR 目录：解析 metadata.yaml 的 names
        if os.path.isdir(mp):
            try:
                import yaml
                meta = os.path.join(mp, "metadata.yaml")
                if os.path.isfile(meta):
                    with open(meta) as f:
                        d = yaml.safe_load(f) or {}
                    names = d.get("names")
                    if isinstance(names, dict):
                        return {int(k): str(v) for k, v in names.items()}
                    if isinstance(names, list):
                        return {i: str(v) for i, v in enumerate(names)}
            except Exception:
                pass
            return {}
        try:
            import ast
            import onnx
            m = onnx.load(mp, load_external_data=False)
            for p in m.metadata_props:
                if p.key == "names":
                    return {int(k): str(v) for k, v in ast.literal_eval(p.value).items()}
        except Exception:
            pass
        return {}

    # ---- 预处理（对齐 ultralytics letterbox）----
    def _preprocess(self, frame, size):
        h, w = frame.shape[:2]
        r = min(size / h, size / w)
        nw, nh = int(round(w * r)), int(round(h * r))
        dw, dh = (size - nw) / 2.0, (size - nh) / 2.0
        if (w, h) != (nw, nh):
            frame = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
        frame = cv2.copyMakeBorder(frame, top, bottom, left, right,
                                   cv2.BORDER_CONSTANT, value=(114, 114, 114))
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        x = rgb.transpose(2, 0, 1).astype(np.float32) / 255.0
        return np.ascontiguousarray(x[None, ...]), r, left, top

    # ---- 后处理 ----
    @staticmethod
    def _nms(boxes, scores, iou_thr=0.45):
        if len(boxes) == 0:
            return []
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(int(i))
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
            order = order[1:][iou <= iou_thr]
        return keep

    def _decode(self, out, conf_thr, r, dw, dh, w, h):
        o = np.asarray(out)
        if o.ndim == 3:
            o = o[0]
        # end2end 头：形如 [300,6] → (x1,y1,x2,y2,conf,cls)
        if o.ndim == 2 and o.shape[-1] == 6 and o.shape[0] <= 1000:
            rows = o[o[:, 4] >= conf_thr]
            boxes, cls, conf = rows[:, :4].copy(), rows[:, 5], rows[:, 4]
        else:
            # 原始头：[C,N] → [N,C]；前 4 列 cx,cy,w,h，其余为类别分
            s = o.T
            boxes_cxcywh = s[:, :4]
            scores = s[:, 4:]
            cls = scores.argmax(1).astype(np.float32)
            conf = scores[np.arange(len(s)), cls.astype(int)]
            m = conf >= conf_thr
            boxes_cxcywh, cls, conf = boxes_cxcywh[m], cls[m], conf[m]
            if len(boxes_cxcywh) == 0:
                return []
            cx, cy, bw, bh = boxes_cxcywh.T
            boxes = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], 1)
            keep = self._nms(boxes.astype(np.float32), conf.astype(np.float32))
            boxes, cls, conf = boxes[keep], cls[keep], conf[keep]
        # 反 letterbox：缩回原图坐标
        if len(boxes):
            boxes = boxes.astype(np.float32)
            boxes[:, [0, 2]] = (boxes[:, [0, 2]] - dw) / r
            boxes[:, [1, 3]] = (boxes[:, [1, 3]] - dh) / r
            boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, w - 1)
            boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, h - 1)
        return [_StubBox(boxes[i], int(cls[i]), float(conf[i])) for i in range(len(boxes))]

    def __call__(self, frame, conf=None, imgsz=None, device=None, verbose=None):
        thr = 0.5 if conf is None else float(conf)
        size = self.imgsz or (imgsz if isinstance(imgsz, int) else 640)
        x, r, dw, dh = self._preprocess(frame, size)
        if self.backend == "ort":
            out = self._session.run(None, {self._input_name: x})[0]
        else:
            out = list(self._compiled.infer_new_request([x]).values())[0]
        h, w = frame.shape[:2]
        boxes = self._decode(out, thr, r, dw, dh, w, h)
        return [_StubResult(boxes)]


# ========================================================================
#  环境自检与自动启动 (MAVROS)
# ========================================================================
# ROS1 版在这里还会自动检查/启动 roscore —— ROS2 无 master，该部分已删除。

def _mavros_is_alive(node):
    """检查 MAVROS 是否在运行（等价 ROS1 的 get_published_topics 检查）"""
    try:
        return node.count_publishers("/mavros/state") > 0
    except Exception:
        return False


def _wait_for_mavros(node, timeout=30):
    """等待 MAVROS 上线，返回是否成功"""
    waited = 0.0
    while not _mavros_is_alive(node):
        if waited >= timeout:
            return False
        time.sleep(1.0)
        waited += 1.0
    return True


def _ensure_mavros(node, fcu_url):
    """确保 MAVROS 在运行，如果不在则自动启动。

    增加 DDS 发现宽限窗口（mavros_discovery_grace，默认 3s）：
    新建节点后 DDS 发现是异步的，立刻 count_publishers 可能返回 0 而误判
     MAVROS 未运行，进而重复拉起第二个 MAVROS 抢同一串口（A3 踩过，错题本 #9）。
    """
    if _mavros_is_alive(node):
        node.get_logger().info("[auto-start] MAVROS 已在运行")
        return

    grace = float(node_param(node, "mavros_discovery_grace", 3.0))
    node.get_logger().info(
        "[auto-start] 等待 %.1fs 确认 MAVROS 确实不在（DDS 发现延迟） ..." % grace
    )
    time.sleep(grace)
    if _mavros_is_alive(node):
        node.get_logger().info("[auto-start] MAVROS 已在运行（宽限期后确认）")
        return

    node.get_logger().info("[auto-start] MAVROS 未运行，自动启动 mavros apm.launch ...")
    try:
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
        node.get_logger().warn("[auto-start] MAVROS 启动超时（30s），飞控数据将不可用")


# ========================================================================
#  DetectorNode
# ========================================================================

class DetectorNode:
    """YOLO 目标检测 ROS 节点（全功能版）"""

    def __init__(self, node):
        self.node = node
        log = node.get_logger()

        # ---------- 模型参数 ----------
        # 本包默认加载 YOLO26 ONNX 权重：<脚本目录>/../models/best_yolo26.onnx
        # （也可用 -p model_path:=/绝对路径/xxx.onnx 显式指定；仍兼容 .pt）
        _default_model = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), '..', 'models', 'best_yolo26.onnx'
        )
        self.model_path = os.path.expanduser(node_param(node, "model_path", _default_model))
        self.conf_threshold = float(node_param(node, "conf_threshold", 0.5))
        self.imgsz_explicit = node.has_parameter("imgsz")
        self.imgsz = int(node_param(node, "imgsz", 640))
        self.device = node_param(node, "device", "cpu")
        self.model_ext = os.path.splitext(self.model_path)[1].lower()

        # ---------- 原生推理后端（可选；默认 ultralytics 保持原行为） ----------
        # ultralytics：原路径（包装层重，但兼容性最好）
        # ort / openvino：原生引擎，可调线程、关自旋、走 OpenVINO IR/INT8
        self.infer_backend = str(node_param(node, "infer_backend", "ultralytics")).strip().lower()
        self.ort_intra_threads = int(node_param(node, "ort_intra_threads", 6))
        self.ov_device = str(node_param(node, "ov_device", "CPU")).strip()
        self.ov_num_threads = int(node_param(node, "ov_num_threads", 0))
        _prov = node_param(node, "ort_providers", "CPUExecutionProvider")
        self.ort_providers = [p.strip() for p in str(_prov).split(",") if p.strip()]

        # ---------- 仿真桩检测（无权重时替代 YOLO，仅仿真验证用） ----------
        self.stub_detector = get_bool_param(node, "stub_detector", False)

        # ---------- 坐标 ----------
        self.invert_camera_x = get_bool_param(node, "invert_camera_x", True)
        # 相机安装外参（launch 中 static_transform_publisher 一并使用）
        self.camera_mount_z = float(node_param(node, "camera_mount_z", 0.40))
        # 双投放点：虚拟相机原点沿机体系 X（前）平移 ±dual_offset_m（默认 5cm）
        self.dual_offset_m = float(node_param(node, "dual_offset_m", 0.05))

        # ---------- 深度 ----------
        self.depth_patch_radius = int(node_param(node, "depth_patch_radius", 5))

        # ---------- 显示 ----------
        # 需求：启动顺序 headless camera → 图像流就绪 → 识别 UI。
        # 默认打开有 UI 的识别画面（-p show_window:=false 可关）；
        # 无 DISPLAY / cv2.imshow 失败时降级 WARN 且不崩。
        self.show_window = get_bool_param(node, "show_window", True)
        self.window_name = node_param(node, "window_name", "YOLO Detection")
        self.warmup = get_bool_param(node, "warmup", True)
        self.cv_threads = int(node_param(node, "cv_threads", 1))
        class_names_str = node_param(node, "class_names", "").strip()
        self.class_name_overrides = [x.strip() for x in class_names_str.split(",") if x.strip()]
        self.class_name_map = {}

        # ---------- 相机自动联动 ----------
        # auto_start_camera=true（默认）：先以 headless（show_rgb:=false）拉起
        # camera_node.py，等图像流就绪后再显示识别 UI；false 时完全不拉起
        # （用 `ros2 node list` 防重）。camera_source 与 camera_node 的 source 参数
        # 一致（仿真 gazebo / 真机 realsense）。
        self.auto_start_camera = get_bool_param(node, "auto_start_camera", True)
        self.camera_source = str(node_param(node, "camera_source", "gazebo")).strip().lower()
        # 自动拉起相机时用哪个包（本包自己的 camera_node）
        self.camera_package = str(
            node_param(node, "camera_package", "cuadc_visionY26")).strip()
        # 相机源校验严格模式：默认 false＝源不一致只告警并照常订阅（仿真/真机共用同名话题）；
        # true＝沿用旧行为（不一致直接中止，避免误用仿真画面跑真机任务）。
        self.strict_camera_source = get_bool_param(node, "strict_camera_source", False)
        self.camera_start_timeout_s = float(
            node_param(node, "camera_start_timeout_s", 20.0)
        )
        self._camera_process = None
        self._camera_source_mismatch = False
        self._first_frame_event = threading.Event()
        self._ui_window_created = False
        self._ui_disabled = False
        self._ui_warned = False
        # GUI 必须在主线程刷新（Qt/cv2 的窗口绑在创建它的线程）：
        # image_callback（executor 线程）只把最新标注帧放进这里，主线程 display_once 负责 imshow。
        self._annotated_lock = threading.Lock()
        self._latest_annotated = None

        # ---------- 类别过滤 ----------
        # 只保留名称包含这些关键词的目标（逗号分隔）
        target_str = node_param(
            node, "target_classes",
            "cylinder,tong,barrel,can,yuantong,white_cylinder,tube,drum,bucket"
        )
        self.target_classes = [c.strip().lower() for c in target_str.split(",") if c.strip()]

        # ---------- 飞控 MAVROS ----------
        self.auto_start_mavros = get_bool_param(node, "auto_start_mavros", False)
        self.mavros_fcu_url = node_param(node, "mavros_fcu_url", "/dev/ttyACM0:921600")
        self.fc_state_topic = node_param(node, "fc_state_topic", "/mavros/state")
        self.fc_battery_topic = node_param(node, "fc_battery_topic", "/mavros/battery")
        self.fc_gps_topic = node_param(node, "fc_gps_topic", "/mavros/global_position/global")
        self.fc_gpsraw_topic = node_param(node, "fc_gpsraw_topic", "/mavros/gpsstatus/gps1/raw")
        self.fc_local_pose_topic = node_param(node, "fc_local_pose_topic", "/mavros/local_position/pose")
        self.output_topic = node_param(node, "output_topic", "/vision/target_global")
        self.body_frame = node_param(node, "body_frame", "base_link")
        self.camera_frame = node_param(node, "camera_frame", "d435i_color_optical_frame")
        self.geo_min_confidence = float(node_param(node, "min_confidence", 0.30))
        self.transform_timeout_sec = float(node_param(node, "transform_timeout_sec", 0.10))
        self.publish_invalid = get_bool_param(node, "publish_invalid", True)
        self.publish_geo_target = get_bool_param(node, "publish_geo_target", False)
        self.geo_result_timeout_sec = float(node_param(node, "geo_result_timeout_sec", 0.50))
        # T1：每帧最多发布多少个目标的 GeoTarget（默认 3）
        self.max_geo_targets = int(node_param(node, "max_geo_targets", 3))

        # ---------- ROS 话题名 ----------
        self.color_topic = node_param(node, "color_topic", "/vision/color/image_raw")
        self.depth_topic = node_param(node, "depth_topic", "/vision/aligned_depth/image_raw")
        self.camera_info_topic = node_param(node, "camera_info_topic", "/vision/color/camera_info")

        # ---------- 加载模型 ----------
        self._init_error = None
        if self.stub_detector:
            log.warn(
                "stub_detector=true：仿真桩检测模式（深度图突起分割），"
                "不加载 YOLO 权重（model_path=%s）" % self.model_path
            )
            self.model = StubDetector(self)
            self.model_ext = ".stub"
            self.device = "stub"
            self.class_name_map = {0: "cylinder"}
        else:
            if not (os.path.isfile(self.model_path) or os.path.isdir(self.model_path)):
                log.error("YOLO model not found: %s" % self.model_path)
                self._init_error = "Model not found: %s" % self.model_path
                return

            # Offline environments should not let Ultralytics try package auto-install.
            os.environ.setdefault("ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS", "1")
            try:
                cv2.setNumThreads(max(1, self.cv_threads))
            except Exception:
                pass

            if self.infer_backend in ("ort", "openvino"):
                # ---- 原生推理后端（绕过 ultralytics 包装层）----
                if os.path.isdir(self.model_path):
                    self.model_ext = ".openvino"
                try:
                    self.model = NativeDetector(
                        self.model_path, backend=self.infer_backend, device=self.device,
                        threads=(self.ov_num_threads if self.infer_backend == "openvino"
                                 else self.ort_intra_threads),
                        providers=tuple(self.ort_providers), log=log)
                except Exception as exc:
                    log.error("Native(%s) 后端加载失败 %s: %s"
                              % (self.infer_backend, self.model_path, exc))
                    self._init_error = "native model load failed"
                    return
                if self.model.imgsz and not self.imgsz_explicit:
                    self.imgsz = self.model.imgsz
                log.info("原生推理后端: %s  model=%s  imgsz=%s  threads=%s  device=%s"
                         % (self.infer_backend, self.model_path, self.imgsz,
                            self.ort_intra_threads if self.infer_backend == "ort"
                            else self.ov_num_threads,
                            self.ov_device if self.infer_backend == "openvino" else self.device))
                self.device = "%s" % self.infer_backend
            else:
                # ---- 原 ultralytics 路径（默认，行为不变）----
                try:
                    from ultralytics import YOLO
                except Exception as exc:
                    log.error("Failed to import ultralytics: %s. Run: pip3 install ultralytics" % exc)
                    self._init_error = "ultralytics not installed"
                    return

                # ONNX 模型：检查 onnxruntime + 同步输入尺寸
                if self.model_ext == ".onnx":
                    try:
                        import onnxruntime
                    except Exception as exc:
                        log.error("ONNX model requires onnxruntime: %s. Run: pip3 install onnxruntime" % exc)
                        self._init_error = "onnxruntime not installed"
                        return
                    log.info("YOLO26 ONNX backend: ultralytics + onnxruntime (%s)"
                             % ", ".join(onnxruntime.get_available_providers()))
                    self._sync_imgsz_with_onnx_input()

                try:
                    self.model = YOLO(self.model_path, task="detect")
                except Exception as exc:
                    log.error("Failed to load YOLO model %s: %s" % (self.model_path, exc))
                    if self.model_ext == ".onnx":
                        log.error("For ONNX models, confirm onnxruntime is installed and the file is valid.")
                    self._init_error = "model load failed"
                    return

            self._build_class_name_map()

            if self.warmup:
                self._warmup_model()

        # ---------- 数据容器 ----------
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
        from builtin_interfaces.msg import Time as _BuiltinTime
        self.geo_lock = threading.Lock()
        self.latest_geo_status = "no_target"
        self.latest_geo_stamp = _BuiltinTime()
        self.latest_geo_body = None          # (x, y, z) in body frame
        self.latest_geo_enu = None           # (east, north, up)
        self.latest_geo_target_wgs84 = None  # (lat, lon, alt)
        self.latest_geo_bucket_ned = None    # (N, E, D)
        self.latest_geo_offset_ned = None    # (dN, dE, dD)

        # ---- TF / GeoTarget 发布 ----
        self.tf_buffer = tf2_ros.Buffer() if _HAS_TF2 else None
        self.tf_listener = (
            tf2_ros.TransformListener(self.tf_buffer, node) if _HAS_TF2 else None
        )
        self.geo_pub = None
        if self.publish_geo_target:
            self.geo_pub = node.create_publisher(GeoTarget, self.output_topic, PUB_QOS)

        # FPS 计时
        self._fps_t0 = node.get_clock().now()
        self._fps_counter = 0
        self._fps_display = 0.0
        self._fps_log_t = time.monotonic()

        # ---------- 发布 ----------
        self.result_pub = node.create_publisher(
            YoloDetection, "/vision/yolo/detection", PUB_QOS
        )
        self.results_pub = node.create_publisher(
            YoloDetections, "/vision/yolo/detections", PUB_QOS
        )
        self.annotated_pub = node.create_publisher(
            Image, "/vision/annotated_image", PUB_QOS
        )
        self.bucket_info_pub = node.create_publisher(
            BucketInfo, "/vision/bucket/info", PUB_QOS
        )
        # 桶估计（数量 + 最多 3 个目标的独立 WGS84/ENU）——只发布，不落盘。
        self.bucket_estimates_pub = node.create_publisher(
            BucketEstimates, "/vision/bucket/estimates", PUB_QOS
        )

        # ---------- 订阅 ----------
        # 图像 QoS（默认 RELIABLE 保持原行为；false=BEST_EFFORT，需与 camera_node 的
        # image_qos_reliable 一致；BEST_EFFORT 可消除背压、提升端到端帧率）
        self.image_reliable = get_bool_param(node, "image_qos_reliable", True)
        self.image_qos = QoSProfile(
            depth=1,
            reliability=(ReliabilityPolicy.RELIABLE if self.image_reliable
                         else ReliabilityPolicy.BEST_EFFORT),
        )
        self.color_sub = node.create_subscription(
            Image, self.color_topic, self.image_callback, self.image_qos
        )
        self.depth_sub = node.create_subscription(
            Image, self.depth_topic, self.depth_callback, self.image_qos
        )
        self.camera_info_sub = node.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, self.image_qos
        )

        # ---- 飞控 MAVROS 订阅（可选，失败时静默跳过）----
        self._init_fc_subscribers()

        # 延迟 3 秒后检查是否需要请求飞控 MAVLink 消息流（参考 poshold_to_land_test.py）
        self._mavlink_stream_requested = False
        self._stream_timer = node.create_timer(3.0, self._on_stream_timer)

        # ---- 任务状态订阅（来自 main.py）----
        self.mission_sub = node.create_subscription(
            MissionStatus, "/vision/mission_status", self._mission_cb, MSG_QOS
        )

        log.info(
            "Detector started. model=%s ext=%s conf=%.2f imgsz=%s device=%s"
            % (self.model_path, self.model_ext, self.conf_threshold,
               str(self.imgsz), self.device)
        )
        log.info("Detector class names: %s" % self.class_name_map)
        if self.publish_geo_target:
            log.info(
                "GeoTarget publishing enabled (multi-target, up to %d/frame). "
                "topic=%s body=%s camera=%s"
                % (self.max_geo_targets, self.output_topic, self.body_frame, self.camera_frame)
            )
        elif not _HAS_TF2:
            log.info("tf2_ros unavailable, bottom geopose bar will show TF placeholders")
        if self._has_fc_data:
            log.info(
                "FC data available: state=%s battery=%s gps=%s gpsraw=%s local_pose=%s"
                % (self.fc_state_topic, self.fc_battery_topic,
                   self.fc_gps_topic, self.fc_gpsraw_topic, self.fc_local_pose_topic)
            )
        else:
            log.info("FC data NOT available (MAVROS not running or mavros_msgs not installed)")

    # ------------------------------------------------------------------
    # 深度图访问（供桩检测器使用）
    # ------------------------------------------------------------------
    def _get_latest_depth_copy(self):
        with self.depth_lock:
            if self.latest_depth is None:
                return None
            return self.latest_depth.copy()

    # ------------------------------------------------------------------
    # 相机自动联动（headless camera → 图像流就绪 → 识别 UI）
    # ------------------------------------------------------------------
    def _running_cameras_ps(self):
        """用 `ps` 找出正在运行的 camera_node 进程及其 source 参数。

        不依赖 DDS 发现（本机 `ros2 node list` / `ros2 param get` 偶发不可靠），
        直接解析进程命令行里的 `-p source:=xxx`。返回 source 字符串列表。
        """
        try:
            out = subprocess.run(
                ["ps", "-eo", "args="],
                capture_output=True, text=True, timeout=5.0,
            ).stdout
        except Exception as exc:
            self.node.get_logger().warn("[auto-start] ps 失败: %s" % exc)
            return []
        srcs = []
        for line in out.splitlines():
            if "camera_node.py" not in line or "detector_node.py" in line:
                continue
            m = re.search(r"source[:=]+\s*([A-Za-z0-9_]+)", line)
            srcs.append(m.group(1).lower() if m else "unknown")
        return srcs

    def _camera_node_running(self):
        """是否存在正在运行的 camera_node（优先 ps，兜底 ros2 node list）。"""
        if self._running_cameras_ps():
            return True
        try:
            out = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True, text=True, timeout=5.0,
            ).stdout
        except Exception as exc:
            self.node.get_logger().warn("[auto-start] ros2 node list 失败: %s" % exc)
            return False
        return any(
            n.rstrip("/").split("/")[-1] == "camera_node"
            for n in out.splitlines()
        )

    def _running_camera_source(self):
        """正在运行的 camera_node 的 source。

        无 camera_node → None；只有一个且明确 → 该值；多个/解析不出 → 'unknown'。
        """
        srcs = self._running_cameras_ps()
        if not srcs:
            return None
        if len(set(srcs)) == 1:
            return srcs[0]
        return "unknown"

    def check_camera_source(self):
        """校验正在运行的 camera_node 的 source 是否与本节点 camera_source 一致。

        坑：仿真里常有一个 camera_node(source:=gazebo) 在发 /vision/color/image_raw。
        此时即使你传了 camera_source:=realsense，本节点也只会订阅那条被仿真相机占用
        的话题，看着"在跑"其实用的是仿真画面。这里显式报错并中止，避免静默用错相机。
        返回 True 表示一致（或没有相机节点）；False 表示不一致。
        """
        running = self._running_camera_source()
        if running is None or running == "unknown":
            return True
        if running != self.camera_source:
            if self.strict_camera_source:
                self.node.get_logger().error(
                    "相机源不匹配：正在运行的 camera_node 是 source=%s，"
                    "但本节点 camera_source=%s。" % (running, self.camera_source))
                self.node.get_logger().error(
                    "话题 %s 会被那个相机占用，检测会用到错误的画面。" % self.color_topic)
                self.node.get_logger().error(
                    "要用真机 USB 相机，请先停掉仿真相机（或设 strict_camera_source:=false）")
                self._camera_source_mismatch = True
                return False
            # 默认：仿真/真机共用同名话题，直接订阅正在发布的那路；仅告警提示。
            self.node.get_logger().warn(
                "相机源不匹配：正在运行的 camera_node 是 source=%s，本节点 camera_source=%s；"
                "将直接订阅话题 %s（如需强制一致请设 strict_camera_source:=true）"
                % (running, self.camera_source, self.color_topic))
            return True
        return True

    def start_headless_camera(self):
        """以 headless（show_rgb:=false）拉起 camera_node.py；已运行则不重复。"""
        if not self.auto_start_camera:
            return
        # #9 DDS 异步发现宽限：launch 可能同时起了 camera_node，先等几拍再判定，
        # 避免"刚起还没被发现"时误判缺失而重复拉起第二个 camera。
        for _ in range(4):
            if self._camera_node_running():
                running = self._running_camera_source()
                self.node.get_logger().info(
                    "[auto-start] camera_node 已在运行(source=%s)，不重复拉起" % running)
                return
            time.sleep(0.5)
        cmd = [
            "ros2", "run", self.camera_package, "camera_node.py",
            "--ros-args",
            "-p", "source:=%s" % self.camera_source,
            "-p", "show_rgb:=false",
            # 与本节点保持一致，否则 RELIABLE/BEST_EFFORT 不匹配收不到图
            "-p", "image_qos_reliable:=%s" % ("true" if self.image_reliable else "false"),
        ]
        try:
            self._camera_process = subprocess.Popen(
                cmd, stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.node.get_logger().warn(
                "[auto-start] 拉起 headless camera_node 失败: %s" % exc)
            return
        self.node.get_logger().warn(
            "[auto-start] 已拉起 headless camera_node (show_rgb:=false, source=%s)"
            % self.camera_source)

    def wait_for_camera_stream(self, timeout_s=None):
        """等图像流第一帧（流就绪后才显示识别 UI）。返回是否就绪。"""
        if timeout_s is None:
            timeout_s = self.camera_start_timeout_s
        if self._first_frame_event.is_set():
            return True
        deadline = time.monotonic() + max(0.0, timeout_s)
        self.node.get_logger().warn(
            "[auto-start] 等待图像流 %s 就绪 ..." % self.color_topic)
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self._first_frame_event.is_set():
                self.node.get_logger().info(
                    "[auto-start] 图像流已就绪，打开识别 UI")
                return True
        self.node.get_logger().warn(
            "[auto-start] 等待图像流超时（%.1fs）；UI 待流到达后再显示"
            % timeout_s)
        return False

    def stop_headless_camera(self):
        p = self._camera_process
        if p is None:
            return
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            except Exception:
                try:
                    p.terminate()
                except Exception:
                    pass
            try:
                p.wait(timeout=5.0)
            except Exception:
                pass
        self._camera_process = None

    def _ui_ready(self):
        if not self.show_window or self._ui_disabled:
            return False
        if cv2 is None:
            self._disable_ui("cv2 不可用，识别 UI 降级（不弹窗）")
            return False
        if not os.environ.get("DISPLAY"):
            self._disable_ui("无 DISPLAY 环境变量，识别 UI 降级（不弹窗）")
            return False
        return True

    def _disable_ui(self, reason):
        self._ui_disabled = True
        if not self._ui_warned:
            self._ui_warned = True
            self.node.get_logger().warn(reason)

    def display_once(self):
        """在主线程里刷新识别窗口。

        cv2/Qt 的窗口绑定在"创建它的线程"上；在 executor 后台线程里 imshow 会得到
        黑屏（Qt 报 QApplication not created in main thread）。因此 sw 画面刷新统一
        放到主线程（main() 的显示循环），executor 线程只负责推理并投递最新帧。
        """
        if not self._ui_ready():
            time.sleep(0.01)
            return
        with self._annotated_lock:
            frame = self._latest_annotated
            self._latest_annotated = None
        if frame is None:
            time.sleep(0.005)
            return
        try:
            if not self._ui_window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, frame.shape[1], frame.shape[0])
                cv2.moveWindow(self.window_name, 40, 40)
                self._ui_window_created = True
                self.node.get_logger().info(
                    "识别 UI 已打开：窗口 '%s'（%dx%d）"
                    % (self.window_name, frame.shape[1], frame.shape[0]))
            cv2.imshow(self.window_name, frame)
            cv2.waitKey(1)
        except Exception as exc:
            self._disable_ui("cv2.imshow 失败，识别 UI 降级: %s" % exc)

    # ------------------------------------------------------------------
    # 定时器：延迟请求 MAVLink 消息流
    # ------------------------------------------------------------------
    def _on_stream_timer(self):
        if self._stream_timer is not None:
            self._stream_timer.cancel()
            self._stream_timer = None
        # 放到独立线程执行，避免阻塞 executor
        threading.Thread(target=self._ensure_fc_mavlink_streams, daemon=True).start()

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
                    self.node.get_logger().warn(
                        "ONNX model expects input %sx%s, overriding requested imgsz=%s"
                        % (h, w, self.imgsz)
                    )
                else:
                    self.node.get_logger().info("ONNX model input resolved to %sx%s" % (h, w))
                self.imgsz = resolved
        except Exception as exc:
            self.node.get_logger().warn(
                "Failed to inspect ONNX input shape, keep imgsz=%s: %s" % (self.imgsz, exc)
            )

    def _warmup_model(self):
        """Run one dummy inference during startup to avoid first-frame stall."""
        try:
            if isinstance(self.imgsz, tuple):
                height, width = self.imgsz
            else:
                height = width = int(self.imgsz)
            dummy = np.zeros((height, width, 3), dtype=np.uint8)
            self.model(dummy, conf=self.conf_threshold, imgsz=self.imgsz, device=self.device, verbose=False)
            self.node.get_logger().info("YOLO warmup finished. input=%sx%s" % (width, height))
        except Exception as exc:
            self.node.get_logger().warn("YOLO warmup failed, continue without warmup: %s" % exc)

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
            self.node.get_logger().info("mavros_msgs not installed, skipping FC data subscriptions")
            return
        try:
            self.fc_state_sub = self.node.create_subscription(
                State, self.fc_state_topic, self.fc_state_cb, NAV_QOS
            )
            self.fc_battery_sub = self.node.create_subscription(
                BatteryState, self.fc_battery_topic, self.fc_battery_cb, NAV_QOS
            )
            self.fc_gpsraw_sub = self.node.create_subscription(
                GPSRAW, self.fc_gpsraw_topic, self.fc_gpsraw_cb, NAV_QOS
            )
            self.fc_gps_sub = self.node.create_subscription(
                NavSatFix, self.fc_gps_topic, self.fc_gps_cb, NAV_QOS
            )
            self.fc_local_pose_sub = self.node.create_subscription(
                PoseStamped, self.fc_local_pose_topic, self.fc_local_pose_cb, NAV_QOS
            )
            self._has_fc_data = True
        except Exception as exc:
            self.node.get_logger().warn(
                "FC subscriber init failed: %s. FC data will be N/A." % exc
            )
            self._has_fc_data = False

    def _ensure_fc_mavlink_streams(self):
        """自动请求飞控发送必要的 MAVLink 消息流。

        飞控默认 SR0_* 参数可能为 0，导致 LOCAL_POSITION_NED / GLOBAL_POSITION_INT
        等关键消息不会通过串口发送到 MAVROS。本方法在飞控已连接但数据缺失时，
        主动调用 /mavros/set_message_interval 请求飞控开启对应消息流。
        """
        if self._mavlink_stream_requested:
            return
        if not self._has_fc_data:
            # MAVROS 未运行或 mavros_msgs 未安装，不请求
            return

        # 等待飞控连接 + 2 秒让现有流（如果有的话）到达
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and rclpy.ok():
            with self.fc_lock:
                connected = self.fc_connected
                has_latlon = self.fc_lat != 0.0 or self.fc_lon != 0.0
                has_local_pose = self.latest_local_pose_msg is not None
            if connected and has_latlon and has_local_pose:
                # 数据已经在流，不需要请求
                self._mavlink_stream_requested = True
                return
            if not connected:
                time.sleep(0.5)
                continue
            break

        with self.fc_lock:
            connected = self.fc_connected
        if not connected:
            # 飞控仍未连接，等下次再试
            return

        self._mavlink_stream_requested = True
        log = self.node.get_logger()
        log.warn("飞控已连接但缺少 NED/WGS84 数据，尝试自动请求 MAVLink 消息流 ...")

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

        client = None
        try:
            client = self.node.create_client(MessageInterval, "/mavros/set_message_interval")
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
                        resp = future.result()
                        if resp.success:
                            log.info("  已请求 MAVLink %s (ID %d) @ %.0f Hz" % (name, msg_id, rate))
                        else:
                            log.warn(
                                "  请求 %s (ID %d) 被拒绝，可能需要手动配置飞控 SR0_* 参数"
                                % (name, msg_id)
                            )
                    else:
                        log.warn("  请求 %s (ID %d) 超时" % (name, msg_id))
                except Exception as exc:
                    log.warn("  请求 %s (ID %d) 失败: %s" % (name, msg_id, exc))
        except Exception as exc:
            log.warn(
                "/mavros/set_message_interval 调用失败: %s\n"
                "如飞控已定位，可手动执行:\n"
                "  ros2 service call /mavros/set_message_interval "
                "mavros_msgs/srv/MessageInterval '{message_id: 32, message_rate: 10.0}'\n"
                "  ros2 service call /mavros/set_message_interval "
                "mavros_msgs/srv/MessageInterval '{message_id: 33, message_rate: 5.0}'"
                % exc
            )
        finally:
            if client is not None:
                self.node.destroy_client(client)

    @staticmethod
    def _make_img_msg(array, encoding):
        """Construct sensor_msgs/Image directly to avoid cv_bridge bgr8 KeyError.

        性能要点（错题本 #6）：
          - step 用 arr.strides[0]（不是 w*channels*itemsize，多通道会错）
          - uint8[] 赋值必须 array.array('B', arr.tobytes())，
            直接赋 bytes 会让 rclpy 逐元素转换（92KB ≈52ms/帧）
        """
        if not array.flags["C_CONTIGUOUS"]:
            array = np.ascontiguousarray(array)
        msg = Image()
        msg.height = array.shape[0]
        msg.width = array.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(array.strides[0])
        msg.data = _array.array('B', array.tobytes())
        return msg

    # ========================================================================
    # 回调：接收数据
    # ========================================================================

    def depth_callback(self, msg):
        """存储最新深度图（线程安全）"""
        try:
            depth = _decode_image(msg, "32FC1")
        except Exception:
            return
        if depth is None:
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

    def transform_camera_to_body(self, detection):
        """把最佳目标从相机光学系转换到机体系（直接数学变换，不依赖 TF 查询）。

        相机安装：在 CG 正下方，镜头朝下，相机顶部朝机体前方。
        X_body(前) = -Y_cam, Y_body(右) = X_cam, Z_body(下) = Z_cam + mount_offset
        """
        cam_x = detection.camera_x_m
        cam_y = detection.camera_y_m
        cam_z = detection.camera_z_m
        mount_z = self.camera_mount_z

        result = PointStamped()
        result.header.frame_id = self.body_frame
        result.header.stamp = self.node.get_clock().now().to_msg()
        result.point.x = -cam_y           # X_body (前) = -Y_cam
        result.point.y = cam_x            # Y_body (右) = X_cam
        result.point.z = cam_z + mount_z  # Z_body (下) = Z_cam + 偏移
        return result

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

    def _compute_geo(self, detection):
        """对单个目标执行相机→机体→ENU→NED/WGS84 变换（不发布、不缓存）。

        返回 (status, body, enu, bucket_ned, offset_ned, target_wgs84)：
          - status 沿用既有语义：no_detection / low_confidence /
            invalid_camera_position / tf_camera_to_body_failed / no_local_pose /
            no_valid_global_position / ok；
          - body=(bx,by,bz) 机体系；enu=(E,N,U) 仅在 local_pose 可用时有效；
          - target_wgs84=(lat,lon,alt) 仅在 status=="ok" 时有效。
        相机→机体 TF 不依赖飞控数据，因此无 MAVROS/local_pose 时 body 仍可用。
        """
        with self.fc_lock:
            latest_global = self.latest_global_msg
            latest_local_pose = self.latest_local_pose_msg

        if not detection.detected:
            return "no_detection", None, None, None, None, None
        if detection.confidence < self.geo_min_confidence:
            return "low_confidence", None, None, None, None, None
        if not detection.position_valid:
            return "invalid_camera_position", None, None, None, None, None

        # ── Step A：相机→机体 TF 变换（不依赖飞控，单独缓存 body 坐标） ──
        body_point = self.transform_camera_to_body(detection)
        if body_point is None:
            return "tf_camera_to_body_failed", None, None, None, None, None

        bx = float(body_point.point.x)
        by = float(body_point.point.y)
        bz = float(body_point.point.z)
        body = (bx, by, bz)

        # ── Step B：机体→ENU→NED→WGS84（需要飞控 local_pose + GPS） ──
        if latest_local_pose is None:
            # 无飞控 local_pose：body 坐标有效，但 ENU/NED/WGS84 不可用
            return "no_local_pose", body, None, None, None, None

        # body 是 FRD（机体系 z 向下为正）→ ENU 前必须把 z 取反。
        # 修正（T2 2026-09-30）：旧代码直接旋转 +z（向下深度当向上），
        # GeoTarget 的 enu_up_m/altitude 会偏高 ≈2×离地高度（仿真实测地面桶
        # alt=610.4 vs 地面 603.5，差 +6.9m）；水平分量不受影响。
        east_m, north_m, up_m = quaternion_rotate_vector(
            latest_local_pose.pose.orientation, (bx, by, -bz)
        )
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
            east_m, north_m, up_m
        )
        return "ok", body, enu, bucket_ned, offset_ned, target_wgs84

    def _dual_solution(self, detection, body, dx):
        """虚拟相机原点（沿机体系 X 平移 dx：前+ / 后-）下桶的一解。

        返回 (cam_xyz, ned, wgs84)：
          cam_xyz = 桶相对该虚拟原点的相机光学系坐标（x右/y下/z前）；
          ned     = 全局 NED（需 local_pose）；wgs84 = (lat, lon, alt)（需 GPS）。
        相机→机体为显式式：X_body=-Y_cam, Y_body=X_cam, Z_body=Z_cam+mount_z。
        """
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

    def _update_best_target_geo(self, detection):
        """对最佳目标执行相机→机体→ENU→WGS84 变换，缓存结果供底部信息栏显示。
        （发布改由 _publish_geo_targets 每帧对每个目标统一完成。）
        """
        stamp = detection.header.stamp if not _stamp_is_zero(detection.header.stamp) \
            else self.node.get_clock().now().to_msg()
        status, body, enu, bucket_ned, offset_ned, target_wgs84 = self._compute_geo(detection)
        self._set_geo_state(status, stamp, body, enu, bucket_ned, offset_ned, target_wgs84)

    def _publish_geo_targets(self, detections_msg):
        """T1：每帧对每个目标发布一条 GeoTarget（最多 max_geo_targets 条）。

        - target_id：按 confidence 降序编号 1/2/3（1=最佳，与 /vision/yolo/detection
          的"最佳目标"一致）；同置信度按 center_x 升序，保证帧内次序稳定。
        - count：本帧全部目标数（可 > max_geo_targets）。
        - 无目标帧：仅在 publish_invalid=true 时发一条
          valid=false,status="no_detection",target_id=0,count=0。
        - 只输出：不做坐标平滑/估计，不写任何 CSV/JSON/日志数据文件。
        """
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
            status, body, enu, _bucket_ned, _offset_ned, wgs84 = self._compute_geo(det)
            target = self._make_base_geo_target(det)
            target.target_id = idx + 1
            target.count = count
            _ratio = self.bucket_size_ratio(det)
            if _ratio is not None:
                target.size_proxy_m = float(_ratio)
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
        """把"目标相对机体"的 ENU 偏移换算为"相对 EKF 原点"的绝对 ENU。

        返回 (E, N, U)；无有效 local_pose 时返回 None。
        """
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
        """构造 BucketEstimates：本帧桶数 + 按 confidence 降序前 3 个目标的独立 WGS84/ENU。

        只组装消息，不发布、不缓存、不落盘。
          - count   = 本帧全部通过过滤的目标数（可 >3）；
          - targets = 前 3 个目标，每个独立反求 WGS84 与绝对 ENU；
          - ratio   = bbox_width_m / bbox_height_m（height<=0 时为 0）；
          - enu_*   = 相对 EKF 原点的绝对 ENU；无有效全局定位时 WGS84 为 0。
        """
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
        # 图像流就绪标记（供 main 的 headless camera → 流就绪 → UI 顺序门控）
        self._first_frame_event.set()

        # 第 1 步：ROS Image → OpenCV BGR
        frame = _decode_image(msg, "bgr8")
        if frame is None:
            _throttle_log(self.node, 2.0, "Image conversion failed (bad frame)", level="warn")
            return

        # 第 2 步：YOLO 推理
        try:
            results = self.model(
                frame, conf=self.conf_threshold, imgsz=self.imgsz,
                device=self.device, verbose=False
            )
        except Exception as exc:
            _throttle_log(self.node, 2.0, "YOLO inference failed: %s" % exc, level="error")
            return

        # 第 3 步：构建 ROS 消息 + 类别过滤
        result_msg, detections_msg, best = self.build_results(
            msg.header, results[0], frame.shape
        )

        # 第 3.5 步：最佳目标坐标变换（底部栏缓存）+ T1 每帧多目标 GeoTarget 发布
        self._update_best_target_geo(result_msg)
        self._publish_geo_targets(detections_msg)

        # 第 4 步：画标注画面
        annotated = frame.copy()

        # 4a. 给每个通过过滤的目标画黄色矩形框
        for target_index, det in enumerate(detections_msg.detections, start=1):
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
        # 前后投放器固定准星（虚拟相机原点投影；前=上、后=下，随深度移动）
        if best is not None and best.position_valid and float(best.camera_z_m) > 0.1:
            _depth_cross = float(best.camera_z_m)
        else:
            _depth_cross = self.get_center_depth()
        self.draw_aim_crosshairs(annotated, _depth_cross)

        # 4c. 画最佳目标的详细标注（见 draw_overlay）
        if best is not None:
            self.draw_overlay(annotated, best)

        # 4c2. 任务状态叠加文字 (AIMING!! / DROP!!!)
        self._draw_mission_overlay(annotated, w, h)

        # 4d. 画面底部整条信息栏：左=状态，右=目标坐标
        l_lines, r_lines = self._build_bottom_bar(
            self.node.get_clock().now(), len(detections_msg.detections)
        )
        self.draw_bottom_bar(annotated, w, h, l_lines, r_lines)

        # 4f. 帧率统计（仅内部计数，不再单独画）
        self._fps_counter += 1
        now = self.node.get_clock().now()
        dt = (now - self._fps_t0).nanoseconds / 1e9
        if dt >= 1.0:
            self._fps_display = self._fps_counter / dt
            self._fps_counter = 0
            self._fps_t0 = now
        # 每 5s 落一行内部 FPS 日志：无窗口/无订阅者时也能量化节点真实处理帧率
        if self._fps_display and (time.monotonic() - self._fps_log_t >= 5.0):
            self._fps_log_t = time.monotonic()
            self.node.get_logger().info(
                "detector FPS %.1f (imgsz=%s)" % (self._fps_display, self.imgsz))

        # 第 5 步：发布
        self.result_pub.publish(result_msg)
        self.results_pub.publish(detections_msg)
        if self.annotated_pub.get_subscription_count() > 0:
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

        # 5c. 发布桶估计（数量 + 最多 3 个目标的独立 WGS84/ENU）——只发布，不落盘。
        self.bucket_estimates_pub.publish(
            self._build_bucket_estimates(msg.header, detections_msg)
        )

        # 第 6 步：把标注帧交给主线程显示（GUI 必须在主线程，见 display_once）
        with self._annotated_lock:
            self._latest_annotated = annotated

    # ========================================================================
    # 深度查询
    # ========================================================================

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
            _throttle_log(
                self.node, 5.0,
                "YOLO detected %d object(s) but all filtered out. "
                "Detected classes: %s. Target keywords: %s"
                % (len(boxes), list(filtered_class_names), self.target_classes)
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

    @staticmethod
    def _to_numpy(v):
        """把 torch 张量、_FakeTensor 或 ndarray 统一转 numpy 数组。"""
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

    def build_single(self, header, yolo_result, box, image_shape):
        """
        从单个 YOLO box 构造一个 YoloDetection：
          - 提取 xyxy → 算中心像素
          - 查深度图 → 反投影 → 相机系 3D 坐标
        """
        det = self.empty_detection(header)

        # 框坐标。ultralytics 的 box.xyxy 是 (1,4) torch 张量；桩检测的
        # _StubBox.xyxy 是 _FakeTensor(ndarray)。统一转 numpy 再取首行。
        xyxy = self._to_numpy(box.xyxy)
        if xyxy.ndim > 1:
            xyxy = xyxy[0]
        x_min, y_min, x_max, y_max = [int(round(v)) for v in xyxy]

        # 中心像素（限定在图像范围内）
        cx = int(round((x_min + x_max) * 0.5))
        cy = int(round((y_min + y_max) * 0.5))

        # 类别和置信度
        cls_arr = self._to_numpy(box.cls).reshape(-1)
        conf_arr = self._to_numpy(box.conf).reshape(-1)
        class_id = int(cls_arr[0]) if cls_arr.size else 0
        confidence = float(conf_arr[0]) if conf_arr.size else 0.0
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
        fx, fy = float(ci.k[0]), float(ci.k[4])
        cx, cy = float(ci.k[2]), float(ci.k[5])
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
        focal = float(ci.k[0] if axis == "x" else ci.k[4])
        return float(pixel_length) * depth_m / focal if focal > 0.0 else 0.0

    def bucket_size_ratio(self, det):
        """返回 bbox 的物理对角线代理值（m），供标定和 auto_drop 使用。"""
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

    # ========================================================================
    #  绘制标注（文字显示在这里！）
    # ========================================================================

    def draw_overlay(self, image, det):
        """只画最佳目标的识别框 + 红色中心点（差值/坐标文字已移到 WebUI）。"""
        x, y = det.center_x, det.center_y
        cv2.circle(image, (x, y), 4, (0, 0, 255), -1)  # 实心红点
        cv2.rectangle(
            image,
            (max(0, det.x_min), max(0, det.y_min)),
            (max(0, det.x_max), max(0, det.y_max)),
            (0, 255, 255), 2
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

    def draw_aim_crosshairs(self, image, depth_m):
        """画前后投放器的固定准星：前(上, 机体系+X)/后(下, -X)，像素位置=主点±fy*偏移/深度。"""
        ci = self.get_camera_info()
        if ci is None or len(ci.k) < 6:
            return
        cx, cy, fy = float(ci.k[2]), float(ci.k[5]), float(ci.k[4])
        if depth_m is None or depth_m <= 0.1:
            return
        off = fy * self.dual_offset_m / float(depth_m)
        h = image.shape[0]
        for label, yy, color in (("前", cy - off, (255, 0, 255)),
                                 ("后", cy + off, (0, 165, 255))):
            iy, ix = int(round(yy)), int(round(cx))
            if iy < 4 or iy > h - 5:
                continue
            cv2.line(image, (ix - 12, iy), (ix + 12, iy), color, 1)
            cv2.line(image, (ix, iy - 8), (ix, iy + 8), color, 1)
            cv2.putText(image, label, (ix + 14, iy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

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
        """只保留检测摘要一行；飞机参数已移到 WebUI「飞机状态栏」，桶参数已移到 WebUI「桶坐标/大小」。"""
        model_name = os.path.basename(self.model_path)
        left = [(
            "DETECT: {}  FPS {:.1f}  |  {} @ {}".format(
                n_detections, self._fps_display, model_name, self.device),
            (0, 255, 255),
        )]
        return left, []

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
        # 允许某一栏为空（飞机/桶参数已移到 WebUI）
        if not left_lines:
            left_lines = [("", (0, 0, 0))]
        if not right_lines:
            right_lines = [("", (0, 0, 0))]
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

def _guarded_spin(executor, node):
    """后台 spin：关停竞态（#23）与真异常分流。"""
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            import traceback
            node.get_logger().error(
                "spin 线程异常退出:\n%s" % traceback.format_exc()
            )


def _make_signal_handler():
    """#29：夺回 Python 级 SIGINT/SIGTERM 处理器；双击 Ctrl-C 走 SIG_DFL 保底。"""
    state = {"count": 0}

    def handler(signum, frame):
        state["count"] += 1
        if state["count"] >= 2:
            try:
                signal.signal(signum, signal.SIG_DFL)
                signal.raise_signal(signum)
            except Exception:
                pass
            return
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

    return handler


def main(args=None):
    # ---- 第 0 步：环境自检 ----
    # ROS1 版会在这里启动 roscore —— ROS2 无 master，无需此步。

    # ---- 第 1 步：初始化节点（#29 入口先堵信号）----
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM})
    rclpy.init(args=args)
    node = Node("detector_node")
    signal.signal(signal.SIGINT, _make_signal_handler())
    signal.signal(signal.SIGTERM, _make_signal_handler())
    detector = None
    executor = None
    spin_thread = None

    try:
        # ---- 第 2 步：MAVROS 策略（房规：脚本不自启 MAVROS）----
        # 需要飞控数据时必须由操作者先用桌面按钮启动 MAVROS
        # （仿真 tcp://127.0.0.1:5760 / 真机网口 udp://192.168.13.14:14550）。
        # 仅当显式传 auto_start_mavros:=true 才允许自启（保留应急开关）。
        auto_start = get_bool_param(node, "auto_start_mavros", False)
        if auto_start:
            fcu_url = node_param(node, "mavros_fcu_url", "/dev/ttyACM0:921600")
            _ensure_mavros(node, fcu_url)
        grace = float(node_param(node, "mavros_discovery_grace", 3.0))
        if not _wait_for_mavros(node, timeout=grace):
            node.get_logger().error(
                "未检测到 MAVROS（/mavros/state 无发布者）。本节点不自启 MAVROS；"
                "请先用桌面按钮启动【仿真 MAVROS】或【真机 网口 MAVROS】后重试。")
            raise SystemExit(1)

        # ---- 第 3 步：创建检测节点 ----
        detector = DetectorNode(node)
        if detector._init_error:
            node.get_logger().error("detector_node 初始化失败: %s" % detector._init_error)
            return

        # ---- 第 3.5 步：相机源校验 + 联动（headless camera → 图像流就绪 → 识别 UI） ----
        if not detector.check_camera_source():
            node.get_logger().error(
                "camera_source 不匹配，已中止；请先停掉仿真相机再启动本节点。")
            return
        if detector.auto_start_camera:
            detector.start_headless_camera()
            detector.wait_for_camera_stream()

        node.get_logger().info("detector_node spinning ...")

        # ---- 第 4 步：事件循环（后台 spin 线程 + 主线程等 join，#28/#29）----
        # executor 只负责 ROS 回调；cv2 窗口刷新已移到主线程 display_once
        # （Qt 要求 GUI 调用发生在主线程，否则窗口黑屏）。单线程 executor 足够。
        executor = MultiThreadedExecutor(num_threads=1)
        executor.add_node(node)
        spin_thread = threading.Thread(
            target=_guarded_spin, args=(executor, node), daemon=True
        )
        spin_thread.start()
        signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT, signal.SIGTERM})
        # 主线程跑显示循环（GUI 必须在主线程）；ROS 回调在后台 spin 线程。
        while rclpy.ok() and spin_thread.is_alive():
            detector.display_once()
        spin_thread.join(timeout=3.0)
    finally:
        if detector is not None:
            detector.stop_headless_camera()
        if executor is not None:
            executor.shutdown(3.0)
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
