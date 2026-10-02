#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""t2_vision_inject.py — T2 仿真视觉注入器（测试固定装置，不属于交付飞行逻辑）。

为什么需要它
------------
仿真世界的 Iris 相机是"前下 45°"安装（错题本 #14），而 detector / auto_drop 的
相机→机体数学是"正下安装"（真机 D435i 语义）。两者外参不一致 → 真实相机链在
仿真里的 body/ENU 水平位置有系统偏差，且相机俯仰改变了深度几何，无法直接驱动
T2 的"选桶→飞向桶→分层瞄准→投放"。T2 步骤 5/6/7 需要一份"深度 + 内参有效、
且与消费方约定一致"的视觉输入，故用本注入器替代 detector 来验证 T2 与瞄准投放
逻辑本身。真实相机链的仿真一致性由 T4 决策（改相机俯仰为 0° 或继续用注入）。

注入内容（每周期）：
  - /vision/yolo/detections (YoloDetections)：每桶一条；depth_m/position_valid/
    camera_*/body_*/bbox_* 全部有效。
  - /vision/target_global (GeoTarget)：target_id=1..N、count=N；
    enu_* 为"相对机身的 ENU 偏移"（与 detector 字段语义一致，消费方自行加位姿）；
    lat/lon/alt 为绝对 WGS84（用当前 GPS 原点 + 偏移换算）。

几何约定（与修正后的 detector / auto_drop 一致；FRD→ENU 需 z 取反）：
  目标绝对 ENU = T，飞机位姿 P/q：
      delta = T - P
      (ux,uy,uz) = R(q)^-1 · delta        # 机体系 FRD（z 向下为正）
      camera = (uy, -ux, -uz)             # 正下安装相机反解（与消费方互为逆）
      depth  = -uz > 0
      u_px   = cx + fx*cam_x/depth ; v_px = cy + fy*cam_y/depth

  ratio 为**物理框对角线代理值（米）**；框像素尺寸按 ratio、depth、fx/fy 反算，
  不随距离人为放大。默认 small=0.15 / mid=0.20 / large=0.25；真机 200mm 标定
  由测试员手工对准桶后填入 config。不做 FOV 角度裁剪（判据确定），但有
  `max_range_m`（默认 25m）水平距离门限：太远的桶不发（避免"从 50m 外也看得见"
  的错误几何运行出现假成功）。

用法（source ROS2 + install 后；先停掉真实 detector，避免同话题双发布）：
  python3 scripts/t2_vision_inject.py --ros-args \
    -p buckets:="31.5,1.2,0.30,150,0.0750,0.85;33.5,-0.8,0.30,200,0.0580,0.90;32.5,0.3,0.30,250,0.0450,0.80"

  buckets 规格：e,n,u,diameter_mm,ratio,confidence（分号分隔多个；e/n/u 为
  仿真世界桶位的 ENU 坐标；两桶降级场景只写两条即可）。
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, NavSatFix

from cuadc_vision.msg import GeoTarget, YoloDetection, YoloDetections

DEFAULT_BUCKETS = ("31.5,1.2,0.30,150,0.1500,0.85;"
                   "33.5,-0.8,0.30,200,0.2000,0.90;"
                   "32.5,0.3,0.30,250,0.2500,0.80")

WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

IMG_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
PUB_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
# MAVROS 位姿/GPS 发布端为 BEST_EFFORT（RELIABLE 订阅会 QoS 不兼容收不到，实测）
MAV_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def q_rotate(q, v):
    """与 detector/auto_drop 完全一致的四元数旋转（Hamilton，body→map）。"""
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def enu_to_wgs84(lat0, lon0, alt0, e, n, u):
    sinp = math.sin(math.radians(lat0))
    w = math.sqrt(1.0 - WGS84_E2 * sinp * sinp)
    rn = WGS84_A / w
    rm = WGS84_A * (1.0 - WGS84_E2) / (w ** 3)
    lat = lat0 + math.degrees(n / rm)
    lon = lon0 + math.degrees(e / (rn * math.cos(math.radians(lat0))))
    return lat, lon, alt0 + u


class T2VisionInject(Node):
    def __init__(self):
        super().__init__("t2_vision_inject")
        self.rate_hz = float(self.declare_parameter("rate_hz", 10.0).value)
        spec = str(self.declare_parameter("buckets", DEFAULT_BUCKETS).value)
        self.min_depth_m = float(self.declare_parameter("min_depth_m", 0.25).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 25.0).value)
        self.max_targets = int(self.declare_parameter("max_targets", 3).value)
        self.publish_geo = bool(self.declare_parameter("publish_geo", True).value)

        self.buckets = []
        for item in spec.split(";"):
            item = item.strip()
            if not item:
                continue
            parts = [p.strip() for p in item.split(",")]
            if len(parts) != 6:
                raise ValueError("buckets 规格必须为 e,n,u,d_mm,ratio,conf：%r" % item)
            self.buckets.append({
                "e": float(parts[0]), "n": float(parts[1]), "u": float(parts[2]),
                "d_mm": float(parts[3]), "ratio": float(parts[4]),
                "conf": float(parts[5]),
            })
        if not self.buckets:
            raise ValueError("buckets 不能为空")

        self.pose = None
        self.fix = None
        self.info = None
        self._last_log = 0.0

        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._pose_cb, MAV_QOS)
        self.create_subscription(NavSatFix, "/mavros/global_position/global",
                                 self._fix_cb, MAV_QOS)
        self.create_subscription(CameraInfo, "/vision/color/camera_info",
                                 self._info_cb, IMG_QOS)
        self.det_pub = self.create_publisher(YoloDetections, "/vision/yolo/detections", PUB_QOS)
        self.geo_pub = self.create_publisher(GeoTarget, "/vision/target_global", PUB_QOS)

        self.timer = self.create_timer(1.0 / max(0.5, self.rate_hz), self.tick)
        self.get_logger().warn(
            "T2 视觉注入器启动：%d 桶，rate=%.1fHz | %s"
            % (len(self.buckets), self.rate_hz,
               "; ".join("(e=%.1f,n=%.1f,u=%.2f D=%.0fmm ratio=%.4f conf=%.2f)"
                         % (b["e"], b["n"], b["u"], b["d_mm"], b["ratio"], b["conf"])
                         for b in self.buckets)))

    def _pose_cb(self, msg):
        self.pose = msg

    def _fix_cb(self, msg):
        if (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
                and math.isfinite(msg.altitude)):
            self.fix = msg

    def _info_cb(self, msg):
        if len(msg.k) >= 6 and float(msg.k[0]) > 0.0 and float(msg.k[4]) > 0.0:
            self.info = msg

    def _publish_empty(self):
        dets = YoloDetections()
        dets.header.stamp = self.get_clock().now().to_msg()
        dets.header.frame_id = "d435i_color_optical_frame"
        dets.detections = []
        self.det_pub.publish(dets)
        if self.publish_geo:
            geo = GeoTarget()
            geo.header = dets.header
            geo.source_topic = "/vision/yolo/detection"
            geo.valid = False
            geo.status = "no_detection"
            geo.target_id = 0
            geo.count = 0
            self.geo_pub.publish(geo)

    def tick(self):
        if self.pose is None or self.fix is None or self.info is None:
            self._publish_empty()
            return
        p = self.pose.pose.position
        q = (self.pose.pose.orientation.x, self.pose.pose.orientation.y,
             self.pose.pose.orientation.z, self.pose.pose.orientation.w)
        qc = q_conj(q)
        fx, fy = float(self.info.k[0]), float(self.info.k[4])
        cx, cy = float(self.info.k[2]), float(self.info.k[5])
        now = self.get_clock().now().to_msg()

        entries = []
        for b in self.buckets:
            if math.hypot(b["e"] - float(p.x), b["n"] - float(p.y)) > self.max_range_m:
                continue
            delta = (b["e"] - float(p.x), b["n"] - float(p.y), b["u"] - float(p.z))
            ux, uy, uz = q_rotate(qc, delta)          # FRD 机体系（z 向下为正）
            depth = -uz
            if depth <= self.min_depth_m:
                continue
            cam_x, cam_y = uy, -ux                    # 正下安装相机反解
            u_px = cx + fx * cam_x / depth
            v_px = cy + fy * cam_y / depth
            # ratio = depth*sqrt((w/fx)^2+(h/fy)^2); use a square pixel bbox
            # to synthesize a frame whose physical proxy remains constant.
            denom = depth * math.sqrt(0.5 * (1.0 / (fx * fx) + 1.0 / (fy * fy)))
            if b["ratio"] <= 0.0 or denom <= 0.0:
                continue
            diag_px = b["ratio"] / denom
            half = diag_px / math.sqrt(2.0) / 2.0
            entries.append({
                "b": b, "depth": depth, "cam": (cam_x, cam_y, depth),
                "body": (-cam_y, cam_x, depth), "ux": ux, "uy": uy,
                "u_px": u_px, "v_px": v_px, "half": half,
                "dist": math.sqrt(cam_x * cam_x + cam_y * cam_y + depth * depth),
                "delta": delta,
            })
        entries.sort(key=lambda x: (-x["b"]["conf"], x["u_px"]))
        entries = entries[:max(1, self.max_targets)]
        count = len(entries)

        dets = YoloDetections()
        dets.header.stamp = now
        dets.header.frame_id = "d435i_color_optical_frame"
        dets.detections = []
        for idx, e in enumerate(entries):
            b = e["b"]
            det = YoloDetection()
            det.header = dets.header
            det.detected = True
            det.class_id = 0
            det.class_name = "bucket"
            det.confidence = float(b["conf"])
            det.x_min = int(e["u_px"] - e["half"])
            det.y_min = int(e["v_px"] - e["half"])
            det.x_max = int(e["u_px"] + e["half"])
            det.y_max = int(e["v_px"] + e["half"])
            det.center_x = int(round(e["u_px"]))
            det.center_y = int(round(e["v_px"]))
            det.depth_m = float(e["depth"])
            det.position_valid = True
            det.camera_x_m, det.camera_y_m, det.camera_z_m = [float(v) for v in e["cam"]]
            det.distance_m = float(e["dist"])
            det.bbox_width_m = float(e["half"] * 2.0 * e["depth"] / fx)
            det.bbox_height_m = float(e["half"] * 2.0 * e["depth"] / fy)
            dets.detections.append(det)

            if self.publish_geo:
                lat, lon, alt = enu_to_wgs84(
                    float(self.fix.latitude), float(self.fix.longitude),
                    float(self.fix.altitude), e["delta"][0], e["delta"][1],
                    e["delta"][2])
                geo = GeoTarget()
                geo.header = dets.header
                geo.valid = True
                geo.status = "ok"
                geo.source_topic = "/vision/yolo/detection"
                geo.class_name = "bucket"
                geo.confidence = float(b["conf"])
                geo.center_x, geo.center_y = det.center_x, det.center_y
                geo.camera_x_m, geo.camera_y_m, geo.camera_z_m = [float(v) for v in e["cam"]]
                geo.body_x_m, geo.body_y_m, geo.body_z_m = [float(v) for v in e["body"]]
                geo.enu_east_m = float(e["delta"][0])
                geo.enu_north_m = float(e["delta"][1])
                geo.enu_up_m = float(e["delta"][2])
                geo.latitude, geo.longitude, geo.altitude = float(lat), float(lon), float(alt)
                geo.target_id = idx + 1
                geo.count = count
                self.geo_pub.publish(geo)

        self.det_pub.publish(dets)

        t = time.monotonic()
        if t - self._last_log >= 1.0:
            self._last_log = t
            parts = []
            for idx, e in enumerate(entries):
                parts.append("#%d(D=%.0fmm r=%.4f) depth=%.2f px=(%d,%d)"
                             % (idx + 1, e["b"]["d_mm"], e["b"]["ratio"], e["depth"],
                                round(e["u_px"]), round(e["v_px"])))
            self.get_logger().info(
                "INJECT count=%d pose=(%.2f,%.2f,%.2f) | %s"
                % (count, float(p.x), float(p.y), float(p.z), " | ".join(parts)))


def main():
    rclpy.init()
    node = None
    try:
        node = T2VisionInject()
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
