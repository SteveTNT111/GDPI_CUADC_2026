#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""camera_node.py —— ROS1 → ROS2 (Humble) 翻译版

原文件：代码/比赛main工作版本/src/cuadc_vision/scripts/camera_node.py
原功能：驱动 Intel RealSense D435i，发布
        /vision/color/image_raw        (bgr8)
        /vision/aligned_depth/image_raw(32FC1, 米)
        /vision/color/camera_info

ROS2 翻译要点
-------------
1. rospy → rclpy；参数 `~x` → node.declare_parameter("x", ...)。
2. 新增 `source` 参数，支持两种图像源：
     - "realsense"（默认，真机）：保留原 D435i 逻辑（profile 枚举/回退、超时重启）。
     - "gazebo"（仿真）：Gazebo 里没有真 D435i，改用 GZ 相机传感器，
       通过 gz.transport13（Python 绑定）订阅 GZ 图像话题，转成 sensor_msgs/Image
       后发到同样的 /vision/* 话题。
   之所以不用 ros_gz_bridge：本机未安装该包且无 sudo；而 gz.transport13 /
   gz.msgs10 Python 绑定可用，故在节点内直接桥接，外部接口保持不变。
3. 发布 QoS：depth=1、RELIABLE（兼容 BEST_EFFORT/RELIABLE 两种订阅者，
   对应原 ROS1 queue_size=1 的“只留最新一帧”语义）。
4. 时间戳：rospy.Time.now() → node.get_clock().now().to_msg()。
"""

import array as _array
import os
import signal
import sys
import time

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image

try:
    import cv2
except ImportError:  # 仿真 source=gazebo 时可无 cv2（仅用 numpy 也能跑）
    cv2 = None

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

try:
    import gz.transport13 as gz_transport
    import gz.msgs10.image_pb2 as gz_image_pb2
    import gz.msgs10.camera_info_pb2 as gz_ci_pb2
    HAVE_GZ = True
except ImportError:
    HAVE_GZ = False


# ---------------------------------------------------------------------------
# RealSense 相关常量（真机路径用）
# ---------------------------------------------------------------------------
COLOR_FORMATS = {
    "bgr8": rs.format.bgr8,
    "rgb8": rs.format.rgb8,
    "yuyv": rs.format.yuyv,
    "y8": rs.format.y8,
} if rs is not None else {}

COLOR_FORMAT_NAMES = {
    rs.format.bgr8: "bgr8",
    rs.format.rgb8: "rgb8",
    rs.format.yuyv: "yuyv",
    rs.format.y8: "y8",
} if rs is not None else {}

COLOR_BYTES_PER_PIXEL = {"bgr8": 3, "rgb8": 3, "yuyv": 2, "y8": 1}

# GZ pixel_format_type 枚举值（gz.msgs10.image_pb2）
GZ_FMT = {
    "L_INT8": 1,
    "L_INT16": 2,
    "RGB_INT8": 3,
    "RGBA_INT8": 4,
    "BGRA_INT8": 5,
    "BGR_INT8": 8,
    "R_FLOAT16": 11,
    "R_FLOAT32": 13,
}

IMAGE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
)


def get_bool_param(node, name, default):
    value = node.declare_parameter(name, default).value
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class CameraNode:
    def __init__(self, node: Node):
        self.node = node

        # ---- 公共参数 ----
        self.source = str(node.declare_parameter("source", "realsense").value).strip().lower()
        if self.source not in ("realsense", "gazebo"):
            node.get_logger().warn(
                "Unsupported source=%s. Falling back to realsense." % self.source)
            self.source = "realsense"

        self.frame_id = node.declare_parameter(
            "frame_id", "d435i_color_optical_frame").value

        # ---- 纯 RGB 原图显示窗口（独立、无任何叠加；realsense/gazebo 两路通用）----
        # 需求：show_rgb=true（默认 false）时弹出一个【纯 RGB 原图窗口】，画面
        # 不叠加任何 UI/标注/文字；窗口名由 rgb_window_name 指定（默认 "RGB"）。
        # 无 DISPLAY / cv2 不可用 / imshow 失败 → WARN 一次后降级禁用，不崩。
        self.show_rgb = get_bool_param(node, "show_rgb", False)
        self.rgb_window_name = str(
            node.declare_parameter("rgb_window_name", "RGB").value)
        self._rgb_window_created = False
        self._rgb_display_disabled = False
        self._rgb_warned = False

        # ---- 发布者（两种 source 共用）----
        self.color_pub = node.create_publisher(Image, "/vision/color/image_raw", IMAGE_QOS)
        self.depth_pub = node.create_publisher(Image, "/vision/aligned_depth/image_raw", IMAGE_QOS)
        self.camera_info_pub = node.create_publisher(CameraInfo, "/vision/color/camera_info", IMAGE_QOS)

        self.shutting_down = False
        self.camera_info_template = None

        # ---- 真机 RealSense 专用状态 ----
        self.pipeline = None
        self.profile_candidates = []
        self.profile_index = 0
        self.active_profile = None
        self.align = None
        self.depth_scale = None
        self.consecutive_timeouts = 0
        self.pipeline_started = False
        self.device_logged = False

        # ---- 仿真 GZ 专用状态 ----
        self.gz_node = None
        self.gz_subs = []
        self._warned_formats = set()
        self._gz_counts = {"color": 0, "depth": 0}
        self._gz_last_log = time.time()

        if self.source == "realsense":
            self._init_realsense_params()
        else:
            self._init_gazebo_params()

    # =======================================================================
    # 真机 RealSense 路径（原逻辑逐行翻译）
    # =======================================================================
    def _init_realsense_params(self):
        if rs is None:
            self.node.get_logger().error(
                "pyrealsense2 不可用，无法使用 source=realsense。"
                "仿真请用 -p source:=gazebo")
            sys.exit(1)

        node = self.node
        fps = int(node.declare_parameter("fps", 30).value)
        color_format = str(node.declare_parameter("color_format", "bgr8").value).strip().lower()
        if color_format not in COLOR_FORMATS:
            node.get_logger().warn(
                "Unsupported color_format=%s. Falling back to bgr8." % color_format)
            color_format = "bgr8"

        self.wait_timeout_ms = int(node.declare_parameter("wait_timeout_ms", 5000).value)
        self.timeout_restart_count = int(node.declare_parameter("timeout_restart_count", 3).value)
        self.auto_profile_fallback = get_bool_param(node, "auto_profile_fallback", True)
        self.align_depth_to_color = get_bool_param(node, "align_depth_to_color", True)

        color_width = int(node.declare_parameter("color_width", 640).value)
        color_height = int(node.declare_parameter("color_height", 480).value)
        depth_width = int(node.declare_parameter("depth_width", 640).value)
        depth_height = int(node.declare_parameter("depth_height", 480).value)

        self.pipeline = rs.pipeline()
        self.profile_candidates = self.build_profile_candidates(
            color_width, color_height, depth_width, depth_height, fps, color_format)
        self.profile_index = 0
        self.active_profile = self.profile_candidates[self.profile_index]
        self.align = rs.align(rs.stream.color) if self.align_depth_to_color else None
        self.depth_scale = None
        self.camera_info_template = None

    def build_profile_candidates(self, cw, ch, dw, dh, fps, cf):
        def make_profile(cw, ch, dw, dh, fps, cf):
            return {"color_width": int(cw), "color_height": int(ch),
                    "depth_width": int(dw), "depth_height": int(dh),
                    "fps": int(fps), "color_format": cf}
        requested = make_profile(cw, ch, dw, dh, fps, cf)
        candidates = [requested]
        candidates.extend(self.enumerate_supported_profiles(max_requested_fps=fps))
        unique, seen = [], set()
        for p in candidates:
            key = tuple(p.items())
            if key not in seen:
                unique.append(p)
                seen.add(key)
        return unique

    def enumerate_supported_profiles(self, max_requested_fps):
        try:
            devices = rs.context().query_devices()
        except Exception as exc:
            self.node.get_logger().warn("Could not query RealSense devices: %s" % exc)
            return []
        if len(devices) == 0:
            self.node.get_logger().warn("No RealSense device found.")
            return []
        color_profiles, depth_profiles = [], []
        device = devices[0]
        usb_type = self.get_device_info(device, rs.camera_info.usb_type_descriptor)
        prefer_native_bgr = str(usb_type).startswith("3")
        for sensor in device.query_sensors():
            for sp in sensor.get_stream_profiles():
                try:
                    vp = sp.as_video_stream_profile()
                except Exception:
                    continue
                st, fmt = sp.stream_type(), sp.format()
                p = {"width": int(vp.width()), "height": int(vp.height()),
                     "fps": int(sp.fps()), "format": fmt}
                if st == rs.stream.color and fmt in COLOR_FORMAT_NAMES:
                    color_profiles.append(p)
                elif st == rs.stream.depth and fmt == rs.format.z16:
                    depth_profiles.append(p)

        candidates = []
        max_fps = max(6, int(max_requested_fps))
        for c in color_profiles:
            for d in depth_profiles:
                if c["fps"] != d["fps"] or c["fps"] > max_fps:
                    continue
                candidates.append({
                    "color_width": c["width"], "color_height": c["height"],
                    "depth_width": d["width"], "depth_height": d["height"],
                    "fps": c["fps"], "color_format": COLOR_FORMAT_NAMES[c["format"]],
                })

        def bandwidth_score(p):
            cb = p["color_width"] * p["color_height"] * COLOR_BYTES_PER_PIXEL[p["color_format"]]
            db = p["depth_width"] * p["depth_height"] * 2
            if prefer_native_bgr:
                order = {"bgr8": 0, "rgb8": 1, "yuyv": 2, "y8": 3}
            else:
                order = {"yuyv": 0, "bgr8": 1, "rgb8": 2, "y8": 3}
            return (p["fps"], cb + db + order.get(p["color_format"], 9) * 1000000)

        candidates.sort(key=bandwidth_score)
        self.node.get_logger().info("Found %d supported profile candidates." % len(candidates))
        return candidates

    def make_config(self, profile):
        config = rs.config()
        config.enable_stream(rs.stream.color, profile["color_width"], profile["color_height"],
                             COLOR_FORMATS[profile["color_format"]], profile["fps"])
        config.enable_stream(rs.stream.depth, profile["depth_width"], profile["depth_height"],
                             rs.format.z16, profile["fps"])
        return config

    def get_device_info(self, device, info_type):
        try:
            return device.get_info(info_type)
        except Exception:
            return "unknown"

    def log_device_info(self, device):
        if self.device_logged:
            return
        get_info = lambda t: self.get_device_info(device, t)
        usb_type = get_info(rs.camera_info.usb_type_descriptor)
        self.node.get_logger().info(
            "RealSense: name=%s serial=%s firmware=%s usb=%s" % (
                get_info(rs.camera_info.name), get_info(rs.camera_info.serial_number),
                get_info(rs.camera_info.firmware_version), usb_type))
        if not str(usb_type).startswith("3"):
            self.node.get_logger().warn(
                "RealSense on USB %s. USB 3.x required for color+depth." % usb_type)
        self.device_logged = True

    def build_camera_info(self, pipeline_profile):
        cp = pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = cp.get_intrinsics()
        ci = CameraInfo()
        ci.width, ci.height = int(intr.width), int(intr.height)
        ci.distortion_model = "plumb_bob"
        ci.d = list(intr.coeffs)
        ci.k = [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    @staticmethod
    def _make_img_msg(arr, encoding):
        """不依赖 cv_bridge，手工构造 sensor_msgs/Image。

        原注释：cv_bridge 在 Noetic + OpenCV4 有 cvtype_to_name 缺键 bug。
        ROS2 版沿用同样做法，既避免该问题，也省掉 cv_bridge 依赖。
        """
        if not arr.flags["C_CONTIGUOUS"]:
            arr = np.ascontiguousarray(arr)
        msg = Image()
        msg.height = arr.shape[0]
        msg.width = arr.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        # 注意：原 ROS1 版用 shape[1]*dtype.itemsize，忽略了通道数，
        # 对 bgr8（3 通道）会算出 width 而非 width*3，导致 cv_bridge 误读。
        # 这里用行字节步长（C 连续时 = width*channels*itemsize）修正。
        msg.step = int(arr.strides[0])
        # rclpy 的 uint8[] 字段底层是 array.array('B')：直接赋 bytes 会触发
        # 逐元素转换（921KB 约 50ms，把 30Hz 拖到 8Hz），必须先转成
        # array.array('B') 再赋值，此时是 C 级 memcpy。
        msg.data = _array.array("B", arr.tobytes())
        return msg

    def convert_to_bgr(self, color_frame):
        fp = color_frame.get_profile().as_video_stream_profile()
        w, h, fmt = int(fp.width()), int(fp.height()), fp.format()
        image = np.asanyarray(color_frame.get_data())
        if fmt == rs.format.bgr8:
            return image if image.ndim == 3 else image.reshape((h, w, 3))
        if fmt == rs.format.rgb8:
            if image.ndim == 1:
                image = image.reshape((h, w, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if fmt == rs.format.yuyv:
            flat = image.reshape(-1)
            if flat.size == h * w * 2:
                return cv2.cvtColor(flat.reshape((h, w, 2)), cv2.COLOR_YUV2BGR_YUY2)
            return cv2.cvtColor(image.reshape((h, w)), cv2.COLOR_GRAY2BGR)
        if fmt == rs.format.y8:
            return cv2.cvtColor(image.reshape((h, w)), cv2.COLOR_GRAY2BGR)
        raise RuntimeError("Unsupported format: %s" % fmt)

    def _start_realsense(self):
        config = self.make_config(self.active_profile)
        try:
            profile = self.pipeline.start(config)
        except Exception as exc:
            if self.auto_profile_fallback and self.try_next_profile("start failed: %s" % exc):
                self.pipeline = rs.pipeline()
                self._start_realsense()
                return
            self.node.get_logger().error("Failed to start RealSense: %s" % exc)
            sys.exit(1)
        device = profile.get_device()
        self.log_device_info(device)
        self.camera_info_template = self.build_camera_info(profile)
        self.depth_scale = device.first_depth_sensor().get_depth_scale()
        self.pipeline_started = True
        self.consecutive_timeouts = 0
        self.node.get_logger().info(
            "D435i started. %dx%d %s@%d depth_scale=%.8f" % (
                self.active_profile["color_width"], self.active_profile["color_height"],
                self.active_profile["color_format"], self.active_profile["fps"],
                self.depth_scale))

    def try_next_profile(self, reason):
        if not self.auto_profile_fallback:
            return False
        if self.profile_index + 1 >= len(self.profile_candidates):
            return False
        self.profile_index += 1
        self.active_profile = self.profile_candidates[self.profile_index]
        self.node.get_logger().warn(
            "Switching profile after %s. New: %dx%d %s@%d" % (
                reason, self.active_profile["color_width"], self.active_profile["color_height"],
                self.active_profile["color_format"], self.active_profile["fps"]))
        return True

    def restart_pipeline(self):
        if self.shutting_down or not rclpy.ok():
            return
        self.node.get_logger().warn("Restarting RealSense pipeline.")
        try:
            if self.pipeline_started:
                self.pipeline.stop()
        except Exception as exc:
            self.node.get_logger().warn("Stop error during restart: %s" % exc)
        time.sleep(1.0)
        self.pipeline_started = False
        self.pipeline = rs.pipeline()
        self._start_realsense()

    def _spin_realsense(self):
        while rclpy.ok() and not self.shutting_down:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=self.wait_timeout_ms)
                if self.shutting_down or not rclpy.ok():
                    break
                if self.align is not None:
                    frames = self.align.process(frames)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                self.consecutive_timeouts = 0
                color_img = self.convert_to_bgr(color_frame)
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth_m = depth_raw.astype(np.float32) * self.depth_scale

                self._show_rgb_frame(color_img)

                stamp = self.node.get_clock().now().to_msg()
                color_msg = self._make_img_msg(color_img, "bgr8")
                depth_msg = self._make_img_msg(depth_m, "32FC1")
                color_msg.header.stamp = stamp
                depth_msg.header.stamp = stamp
                color_msg.header.frame_id = self.frame_id
                depth_msg.header.frame_id = self.frame_id

                if self.camera_info_template is not None:
                    ci = CameraInfo()
                    ci.header.stamp = stamp
                    ci.header.frame_id = self.frame_id
                    ci.width = self.camera_info_template.width
                    ci.height = self.camera_info_template.height
                    ci.distortion_model = self.camera_info_template.distortion_model
                    ci.d = list(self.camera_info_template.d)
                    ci.k = list(self.camera_info_template.k)
                    ci.r = list(self.camera_info_template.r)
                    ci.p = list(self.camera_info_template.p)
                    self.camera_info_pub.publish(ci)

                self.color_pub.publish(color_msg)
                self.depth_pub.publish(depth_msg)
            except RuntimeError as exc:
                if self.shutting_down or not rclpy.ok():
                    break
                self.consecutive_timeouts += 1
                self.node.get_logger().warn(
                    "Frame read failed (%d/%d): %s" % (
                        self.consecutive_timeouts, self.timeout_restart_count, exc))
                if self.consecutive_timeouts >= self.timeout_restart_count:
                    self.try_next_profile("repeated timeouts")
                    self.restart_pipeline()
            except Exception as exc:
                if self.shutting_down or not rclpy.ok():
                    break
                self.consecutive_timeouts += 1
                self.node.get_logger().warn(
                    "Frame processing error (%d/%d): %s" % (
                        self.consecutive_timeouts, self.timeout_restart_count, exc))
                if self.consecutive_timeouts >= self.timeout_restart_count:
                    self.try_next_profile("repeated errors")
                    self.restart_pipeline()

    # =======================================================================
    # 仿真 Gazebo 路径（gz.transport13 桥接）
    # =======================================================================
    def _init_gazebo_params(self):
        if not HAVE_GZ:
            self.node.get_logger().error(
                "gz.transport13 / gz.msgs10 Python 绑定不可用，无法使用 source=gazebo")
            sys.exit(1)

        node = self.node
        self.gz_color_topic = node.declare_parameter(
            "gz_color_topic", "/camera/color/image").value
        self.gz_depth_topic = node.declare_parameter(
            "gz_depth_topic", "/camera/depth/image").value
        self.gz_info_topic = node.declare_parameter(
            "gz_camera_info_topic", "/camera/color/camera_info").value
        self.gz_publish_camera_info = get_bool_param(node, "publish_camera_info", True)
        self.gz_fallback_info = get_bool_param(node, "fallback_camera_info", True)

    def _gz_convert_color(self, msg):
        """GZ Image(RGB/BGR/L) → numpy BGR uint8。"""
        h, w, step = int(msg.height), int(msg.width), int(msg.step)
        fmt = int(msg.pixel_format_type)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if fmt == GZ_FMT["RGB_INT8"]:
            row = step // 3
            img = buf.reshape(h, row, 3)[:, :w, :]
            if cv2 is not None:
                return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            return img[:, :, ::-1].copy()
        if fmt == GZ_FMT["BGR_INT8"]:
            row = step // 3
            return buf.reshape(h, row, 3)[:, :w, :]
        if fmt == GZ_FMT["L_INT8"]:
            img = buf.reshape(h, step)[:, :w]
            if cv2 is not None:
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            return np.repeat(img[:, :, None], 3, axis=2)
        self._warn_once("color fmt=%d" % fmt)
        return None

    def _gz_convert_depth(self, msg):
        """GZ depth Image → numpy float32 (米) 2D 数组。"""
        h, w, step = int(msg.height), int(msg.width), int(msg.step)
        fmt = int(msg.pixel_format_type)
        if fmt == GZ_FMT["R_FLOAT32"]:
            buf = np.frombuffer(msg.data, dtype=np.float32)
            return buf.reshape(h, step // 4)[:, :w].copy()
        if fmt == GZ_FMT["L_INT16"]:
            buf = np.frombuffer(msg.data, dtype=np.uint16)
            # 毫米 → 米
            return (buf.reshape(h, step // 2)[:, :w].astype(np.float32) / 1000.0)
        if fmt == GZ_FMT["R_FLOAT16"]:
            buf = np.frombuffer(msg.data, dtype=np.float16)
            return buf.reshape(h, step // 2)[:, :w].astype(np.float32)
        self._warn_once("depth fmt=%d" % fmt)
        return None

    def _warn_once(self, key):
        if key not in self._warned_formats:
            self._warned_formats.add(key)
            self.node.get_logger().warn("暂不支持的 GZ 像素格式 %s，已跳过该帧" % key)

    def _gz_make_camera_info(self, msg):
        ci = CameraInfo()
        ci.header.stamp = self.node.get_clock().now().to_msg()
        ci.header.frame_id = self.frame_id
        ci.width = int(msg.width)
        ci.height = int(msg.height)
        ci.distortion_model = "plumb_bob"
        k = list(msg.intrinsics.k)
        ci.k = k if len(k) == 9 else [0.0] * 9
        ci.d = list(msg.distortion.k)
        p = list(msg.projection.p)
        ci.p = p if len(p) == 12 else [0.0] * 12
        r = list(msg.rectification_matrix)
        ci.r = r if len(r) == 9 else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        return ci

    def _gz_fallback_camera_info(self, w, h):
        ci = CameraInfo()
        ci.header.stamp = self.node.get_clock().now().to_msg()
        ci.header.frame_id = self.frame_id
        ci.width, ci.height = int(w), int(h)
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # 水平 FOV 1.211 rad（与模型 horizontal_fov 一致）估内参
        fx = (w / 2.0) / np.tan(1.211 / 2.0)
        fy = fx
        cx, cy = w / 2.0, h / 2.0
        ci.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    def _gz_tick(self, kind):
        self._gz_counts[kind] += 1
        now = time.time()
        if now - self._gz_last_log >= 5.0:
            dt = now - self._gz_last_log
            self.node.get_logger().info(
                "GZ→ROS 发布速率: color=%.1fHz depth=%.1fHz" % (
                    self._gz_counts["color"] / dt, self._gz_counts["depth"] / dt))
            self._gz_counts = {"color": 0, "depth": 0}
            self._gz_last_log = now

    def _on_gz_color(self, msg):
        if self.shutting_down:
            return
        try:
            img = self._gz_convert_color(msg)
            if img is None:
                return
            self._show_rgb_frame(img)
            out = self._make_img_msg(img, "bgr8")
            out.header.stamp = self.node.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id
            self.color_pub.publish(out)
            self._gz_tick("color")
        except Exception as exc:
            self.node.get_logger().warn("color 转换失败: %s" % exc)

    def _on_gz_depth(self, msg):
        if self.shutting_down:
            return
        try:
            depth = self._gz_convert_depth(msg)
            if depth is None:
                return
            out = self._make_img_msg(depth, "32FC1")
            out.header.stamp = self.node.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id
            self.depth_pub.publish(out)
            self._gz_tick("depth")
        except Exception as exc:
            self.node.get_logger().warn("depth 转换失败: %s" % exc)

    def _on_gz_info(self, msg):
        if self.shutting_down or not self.gz_publish_camera_info:
            return
        try:
            self.camera_info_pub.publish(self._gz_make_camera_info(msg))
        except Exception as exc:
            self.node.get_logger().warn("camera_info 转换失败: %s" % exc)

    def _start_gazebo(self):
        self.gz_node = gz_transport.Node()
        self.gz_node.subscribe(gz_image_pb2.Image, self.gz_color_topic, self._on_gz_color)
        self.gz_node.subscribe(gz_image_pb2.Image, self.gz_depth_topic, self._on_gz_depth)
        if self.gz_publish_camera_info:
            self.gz_node.subscribe(gz_ci_pb2.CameraInfo, self.gz_info_topic, self._on_gz_info)
        self.node.get_logger().info(
            "Gazebo 图像源已启动: color=%s depth=%s info=%s" % (
                self.gz_color_topic, self.gz_depth_topic, self.gz_info_topic))
        if self.gz_fallback_info and self.gz_publish_camera_info:
            # 先发一帧兜底 camera_info（等 GZ 的首帧到达后再被真实值覆盖）
            self.camera_info_pub.publish(self._gz_fallback_camera_info(640, 480))

    def _spin_gazebo(self):
        # GZ 回调由 gz.transport 自己的线程驱动，这里只需保持进程存活。
        while rclpy.ok() and not self.shutting_down:
            time.sleep(0.1)

    # =======================================================================
    # 纯 RGB 原图窗口（不叠加任何 UI/标注/文字）
    # =======================================================================
    def _show_rgb_frame(self, bgr_img):
        if not self.show_rgb or bgr_img is None or self._rgb_display_disabled:
            return
        if cv2 is None:
            self._rgb_disable("cv2 不可用，show_rgb 降级（不弹窗）")
            return
        if not os.environ.get("DISPLAY"):
            self._rgb_disable("无 DISPLAY 环境变量，show_rgb 降级（不弹窗）")
            return
        try:
            if not self._rgb_window_created:
                cv2.namedWindow(self.rgb_window_name, cv2.WINDOW_NORMAL)
                self._rgb_window_created = True
                self.node.get_logger().info(
                    "show_rgb 已开启：纯 RGB 原图窗口 '%s'（无任何叠加）"
                    % self.rgb_window_name)
            cv2.imshow(self.rgb_window_name, bgr_img)
            cv2.waitKey(1)
        except Exception as exc:
            self._rgb_disable("cv2.imshow 失败，show_rgb 降级: %s" % exc)

    def _rgb_disable(self, reason):
        self._rgb_display_disabled = True
        if not self._rgb_warned:
            self._rgb_warned = True
            self.node.get_logger().warn(reason)

    # =======================================================================
    # 统一入口
    # =======================================================================
    def start(self):
        if self.source == "realsense":
            self._start_realsense()
        else:
            self._start_gazebo()

    def spin(self):
        if self.source == "realsense":
            self._spin_realsense()
        else:
            self._spin_gazebo()

    def stop(self):
        self.shutting_down = True
        if self.source == "realsense" and self.pipeline_started:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            finally:
                self.pipeline_started = False
        if self.source == "gazebo" and self.gz_node is not None:
            for topic in (self.gz_color_topic, self.gz_depth_topic):
                try:
                    self.gz_node.unsubscribe(topic)
                except Exception:
                    pass
            self.gz_node = None
        if cv2 is not None:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


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
    # #29 第一半：入口先堵住 SIGINT/SIGTERM，其后创建的线程继承被掩码。
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM})
    rclpy.init(args=args)
    node = Node("camera_node")
    # #29 第二半：rclpy.init 装好 C 处理器后，用 Python 级处理器夺回控制权。
    signal.signal(signal.SIGINT, _make_signal_handler())
    signal.signal(signal.SIGTERM, _make_signal_handler())
    cam = None
    try:
        cam = CameraNode(node)
        cam.start()
        # 线程建齐 → 主线程单线程放行信号（信号必投主线程，Python 处理器执行）。
        signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT, signal.SIGTERM})
        cam.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        # #23：SIGINT → rclpy 关 context → spin 抛 ExternalShutdownException
        #（非 KeyboardInterrupt）——属正常关停路径，不得误记成异常退出
        pass
    except Exception as exc:
        node.get_logger().error("camera_node 异常退出: %s" % exc)
    finally:
        if cam is not None:
            try:
                cam.stop()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            except Exception as exc:
                node.get_logger().warn("camera 收尾异常: %s" % exc)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
