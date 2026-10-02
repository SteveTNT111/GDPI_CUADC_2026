#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T2 离线端到端验证：/vision/bucket/estimates（BucketEstimates）。

不依赖 Gazebo/SITL：合成相机（color/depth/camera_info）+ 合成当地定位
（local_pose / NavSatFix 注入到自定义话题），仅用本机真实 MAVROS 提供
/mavros/state 供 detector main 门控。被测 detector 用 stub_detector 桩。

阶段（按时间）：
  0) 无桶            → count=0, targets=[]
  1) 单桶(中央)      → count=1, targets 长度 1
  2) 三桶(左中右)    → count=3, targets 长度 3，逐个独立 WGS84/ENU
  3) 四桶            → count=4, targets 长度 3（验证只取前 3）

独立真值：像素+深度 → 相机系 → 机体系(正下装) → ENU（恒等姿态）；WGS84 用与
被测节点不同的平面近似独立求解。错题 #18/#40：真值必须独立于被测通道。

另做：
  - ratio == bbox_width_m/bbox_height_m；bbox_diagonal_px>0；
  - enu_* 为“相对 EKF 原点”的绝对 ENU = 无人机 ENU + 目标偏移；
  - 排序：confidence 非增，同值 center_x 非减；
  - 画面显示：注入 MissionStatus(ammo_a=2, ammo_b=1, aiming=True, last_drop='A')
    并抓一帧 /vision/annotated_image 存 PNG（由验证脚本写，非 detector）；
  - detector rc=0 / 0 Traceback；detector 源码无写文件 API（另由 shell 检查）。

用法（source ROS2 + install 后）：
  T2_DETECTOR=cuadc_visionY26 python3 scripts/verify_t2_estimates.py
  T2_DETECTOR=cuadc_vision   python3 scripts/verify_t2_estimates.py
"""
import array as _array
import math
import os
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

from cuadc_vision.msg import BucketEstimates, MissionStatus

W, H = 640, 480
BG = 6.0                       # 背景地面深度 (m)
ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = 30.0, 120.0, 100.0
FX = FY = 600.0
CX, CY = 320.0, 240.0
MOUNT_Z = 0.40                 # detector camera_mount_z 默认
DRONE_E, DRONE_N, DRONE_U = 0.0, 0.0, 5.0   # 合成无人机 ENU（EKF 原点系）

TAG = os.environ.get("T2_TAG", "cv")
PKG = os.environ.get("T2_DETECTOR", "cuadc_vision")
DETECTOR = os.environ.get(
    "T2_DETECTOR_PATH",
    "/home/lab/ROS2_CUADC2026/install/%s/lib/%s/detector_node.py" % (PKG, PKG))
DET_LOG = os.environ.get("T2_DET_LOG", "/tmp/opencode/t2_detector_%s.log" % TAG)
ANN_PNG = os.environ.get("T2_ANN_PNG", "/tmp/opencode/t2_annotated_%s.png" % TAG)
ANN_AIM_PNG = os.environ.get("T2_ANN_AIM_PNG", "/tmp/opencode/t2_ann_aiming_%s.png" % TAG)
ANN_DROP_PNG = os.environ.get("T2_ANN_DROP_PNG", "/tmp/opencode/t2_ann_drop_%s.png" % TAG)

BUMPS1 = [((320, 240), 5.0)]
BUMPS3 = [((200, 200), 4.6), ((320, 240), 4.8), ((440, 280), 4.4)]
BUMPS4 = [((160, 200), 4.6), ((280, 240), 4.8), ((400, 280), 4.4), ((520, 320), 5.0)]

IMG_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
PUB_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


def make_depth(bumps):
    d = np.full((H, W), BG, dtype=np.float32)
    yy, xx = np.mgrid[0:H, 0:W]
    for (bx, by), dep in bumps:
        d[(xx - bx) ** 2 + (yy - by) ** 2 <= 14 * 14] = dep
    return d


def to_img(arr, encoding):
    m = Image()
    m.height, m.width = arr.shape[0], arr.shape[1]
    m.encoding = encoding
    m.is_bigendian = False
    m.step = int(arr.strides[0])
    m.data = _array.array('B', np.ascontiguousarray(arr).tobytes())
    return m


def expected_offset(depth, cx, cy):
    """独立解析：像素+深度 → 相机系 → 机体系(正下装) → ENU 偏移（恒等姿态）。"""
    cam_x = -1.0 * (cx - CX) * depth / FX
    cam_y = (cy - CY) * depth / FY
    return -cam_y, cam_x, depth + MOUNT_Z     # (E, N, U)


def geodetic(origin_lat, origin_lon, origin_alt, e, n, u):
    """独立 WGS84 真值（平面近似，独立于被测节点回退公式）。"""
    r_earth = 6371000.0
    lat = origin_lat + math.degrees(n / r_earth)
    lon = origin_lon + math.degrees(e / (r_earth * math.cos(math.radians(origin_lat))))
    return lat, lon, origin_alt + u


def _center_text_mask(frame, kind):
    """判断画面中下区域是否渲染了 AIMING(黄)/DROP(红) 大字。

    只取中下窄条（y=h/2+32..h/2+72, x=w/2±100），避开画面中央检测框
    （~h/2）与右栏文字（x>w/2+110），只对覆盖在中心的横幅文字敏感。
    """
    h, w = frame.shape[:2]
    crop = frame[h // 2 + 32:h // 2 + 72, w // 2 - 100:w // 2 + 100]
    b = crop[:, :, 0].astype(int)
    g = crop[:, :, 1].astype(int)
    r = crop[:, :, 2].astype(int)
    if kind == "yellow":
        cnt = np.count_nonzero((b < 80) & (g > 170) & (r > 170))
    else:  # red
        cnt = np.count_nonzero((b < 80) & (g < 80) & (r > 170))
    return int(cnt) > 120


class T2Synth(Node):
    def __init__(self):
        super().__init__("t2_synth")
        self.pub_color = self.create_publisher(Image, "/vision/color/image_raw", IMG_QOS)
        self.pub_depth = self.create_publisher(Image, "/vision/aligned_depth/image_raw", IMG_QOS)
        self.pub_info = self.create_publisher(CameraInfo, "/vision/color/camera_info", IMG_QOS)
        self.pub_gps = self.create_publisher(NavSatFix, "/test/t2/global", PUB_QOS)
        self.pub_pose = self.create_publisher(PoseStamped, "/test/t2/pose", PUB_QOS)
        self.pub_mission = self.create_publisher(MissionStatus, "/vision/mission_status", PUB_QOS)
        self.sub_est = self.create_subscription(
            BucketEstimates, "/vision/bucket/estimates", self._on_est, 50)
        self.sub_ann = self.create_subscription(
            Image, "/vision/annotated_image", self._on_ann, IMG_QOS)
        self.ests = []
        self.last_ann = None
        self.last_ann_aim = None
        self.last_ann_drop = None
        self._mission_tag = None
        self._tick = 0
        self.t0 = time.monotonic()

    def _on_est(self, msg):
        targets = []
        for t in msg.targets:
            targets.append({
                "valid": t.valid, "conf": t.confidence,
                "cx": t.center_x, "cy": t.center_y, "depth": t.depth_m,
                "diag": t.bbox_diagonal_px,
                "bw": t.bbox_width_m, "bh": t.bbox_height_m, "ratio": t.ratio,
                "lat": t.latitude, "lon": t.longitude, "alt": t.altitude,
                "e": t.enu_east_m, "n": t.enu_north_m, "u": t.enu_up_m,
            })
        self.ests.append({"t": time.monotonic(), "count": msg.count, "targets": targets})

    def _on_ann(self, msg):
        if msg.encoding != "bgr8" or not msg.data:
            return
        arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        try:
            frame = arr.reshape((msg.height, msg.width, 3)).copy()
        except Exception:
            return
        self.last_ann = frame
        if self._mission_tag == "aim" and _center_text_mask(frame, "yellow"):
            self.last_ann_aim = frame
        elif self._mission_tag == "drop" and _center_text_mask(frame, "red"):
            self.last_ann_drop = frame

    def publish_once(self):
        self._tick += 1
        el = time.monotonic() - self.t0
        if el < 6.0:
            bumps = []            # 阶段0：无桶
        elif el < 12.0:
            bumps = BUMPS1        # 阶段1：单桶
        elif el < 19.0:
            bumps = BUMPS3        # 阶段2：三桶
        else:
            bumps = BUMPS4        # 阶段3：四桶
        now = self.get_clock().now().to_msg()

        dm = to_img(make_depth(bumps), "32FC1"); dm.header.stamp = now
        self.pub_depth.publish(dm)

        color = np.zeros((H, W, 3), dtype=np.uint8)
        cm = to_img(color, "bgr8"); cm.header.stamp = now
        self.pub_color.publish(cm)

        ci = CameraInfo()
        ci.header.stamp = now
        ci.width, ci.height = W, H
        ci.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        ci.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(ci)

        gps = NavSatFix()
        gps.header.stamp = now
        gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS
        gps.latitude, gps.longitude, gps.altitude = ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_gps.publish(gps)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = DRONE_E
        pose.pose.position.y = DRONE_N
        pose.pose.position.z = DRONE_U
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)

        # 任务状态注入（验证显示路径）：12.5~16s → AIMING!!；16s 之后 → 'A' DROP!!!
        if el >= 12.5:
            ms = MissionStatus()
            ms.ammo_a = 2
            ms.ammo_b = 1
            ms.aiming = True
            if el < 16.0:
                ms.last_drop = ""
                self._mission_tag = "aim"
            else:
                ms.last_drop = "A"
                self._mission_tag = "drop"
            self.pub_mission.publish(ms)


def main():
    if not os.path.isfile(DETECTOR):
        print("detector not found:", DETECTOR)
        return 2
    log_f = open(DET_LOG, "w")
    proc = subprocess.Popen(
        [sys.executable, DETECTOR, "--ros-args",
         "-p", "stub_detector:=true",
         "-p", "auto_start_camera:=false",
         "-p", "show_window:=false",
         "-p", "publish_geo_target:=true",
         "-p", "max_geo_targets:=3",
         "-p", "fc_gps_topic:=/test/t2/global",
         "-p", "fc_local_pose_topic:=/test/t2/pose",
         "-p", "fc_rel_alt_topic:=/test/t2/rel_alt"],
        stdout=log_f, stderr=subprocess.STDOUT, start_new_session=True)

    rclpy.init()
    node = T2Synth()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    t_end = time.monotonic() + 26.0
    next_pub = time.monotonic()
    try:
        while time.monotonic() < t_end and proc.poll() is None:
            now = time.monotonic()
            if now >= next_pub:
                node.publish_once()
                next_pub = now + 0.1
            ex.spin_once(timeout_sec=0.02)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except Exception:
            proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=6)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    pass
        if node.last_ann is not None:
            cv2.imwrite(ANN_PNG, node.last_ann)
        if node.last_ann_aim is not None:
            cv2.imwrite(ANN_AIM_PNG, node.last_ann_aim)
        if node.last_ann_drop is not None:
            cv2.imwrite(ANN_DROP_PNG, node.last_ann_drop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log_f.close()

    msgs = node.ests
    print("[%s] detector=%s" % (TAG, DETECTOR))
    print("[%s] captured estimates: %d detector rc: %s"
          % (TAG, len(msgs), proc.returncode))

    p0 = [m for m in msgs if m["count"] == 0]
    p1 = [m for m in msgs if m["count"] == 1]
    p3 = [m for m in msgs if m["count"] == 3]
    p4 = [m for m in msgs if m["count"] >= 4]

    ok = True

    def check(cond, desc):
        nonlocal ok
        print(("  PASS " if cond else "  FAIL ") + desc)
        if not cond:
            ok = False

    print("\n[阶段0] 无桶（期望 count=0, targets=[]）")
    check(len(p0) > 0, "有 count=0 的消息 (%d)" % len(p0))
    if p0:
        check(all(len(m["targets"]) == 0 for m in p0), "targets 为空")

    print("\n[阶段1] 单桶（count=1, len(targets)=1）")
    check(len(p1) > 0, "有 count=1 的消息 (%d)" % len(p1))
    if p1:
        m = p1[-1]
        print("   样本:", m["count"], m["targets"][:1])
        check(len(m["targets"]) == 1, "len(targets)==1")

    print("\n[阶段2] 三桶（count=3, len(targets)=3, 逐个独立真值）")
    check(len(p3) > 0, "有 count=3 的消息 (%d)" % len(p3))
    if p3:
        m = p3[-1]
        check(len(m["targets"]) == 3, "len(targets)==3 (实际 %d)" % len(m["targets"]))
        # 排序：confidence 非增；同值 center_x 非减
        confs = [t["conf"] for t in m["targets"]]
        check(all(confs[i] >= confs[i + 1] - 1e-9 for i in range(len(confs) - 1)),
              "confidence 非增 %s" % [round(c, 3) for c in confs])
        cxs = [t["cx"] for t in m["targets"]]
        check(all(cxs[i] <= cxs[i + 1] for i in range(len(cxs) - 1)),
              "center_x 非减（同值稳定序）%s" % cxs)
        # 逐目标比对独立真值（用该目标自身像素/深度）
        for k, t in enumerate(m["targets"]):
            e, n, u = expected_offset(t["depth"], t["cx"], t["cy"])
            elat, elon, ealt = geodetic(ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT, e, n, u)
            abs_e, abs_n, abs_u = DRONE_E + e, DRONE_N + n, DRONE_U + u
            print("   目标%d: cx=%d cy=%d depth=%.2f ratio=%.3f diag=%.1f" %
                  (k + 1, t["cx"], t["cy"], t["depth"], t["ratio"], t["diag"]))
            print("           实测 WGS84 (%.7f,%.7f,%.3f) ENU(abs) (%.3f,%.3f,%.3f)" %
                  (t["lat"], t["lon"], t["alt"], t["e"], t["n"], t["u"]))
            print("           真值 WGS84 (%.7f,%.7f,%.3f) ENU(abs) (%.3f,%.3f,%.3f)" %
                  (elat, elon, ealt, abs_e, abs_n, abs_u))
            check(t["valid"], "目标%d valid" % (k + 1))
            check(t["depth"] > 0.5 and t["diag"] > 0.0, "目标%d 深度/对角线>0" % (k + 1))
            check(abs(t["ratio"] - (t["bw"] / t["bh"] if t["bh"] > 0 else 0.0)) < 1e-6,
                  "目标%d ratio==width/height" % (k + 1))
            check(abs(t["e"] - abs_e) < 0.06 and abs(t["n"] - abs_n) < 0.06
                  and abs(t["u"] - abs_u) < 0.06,
                  "目标%d 绝对 ENU 与真值差<0.06m" % (k + 1))
            check(abs(t["lat"] - elat) < 2e-5 and abs(t["lon"] - elon) < 2e-5
                  and abs(t["alt"] - ealt) < 0.06,
                  "目标%d WGS84 与真值一致" % (k + 1))

    print("\n[阶段3] 四桶（count>=4 但仅取前 3）")
    if p4:
        m = p4[-1]
        print("   count=%d, len(targets)=%d" % (m["count"], len(m["targets"])))
        check(m["count"] >= 4, "count>=4")
        check(len(m["targets"]) == 3, "len(targets)==3（上限）")
    else:
        print("  SKIP 未检出 4 桶（桩参数所致，不判 FAIL）")

    # MissionStatus 订阅确认（display 数据源）
    print("\n[显示] MissionStatus 注入 + annotated 抓帧")
    check(node.last_ann is not None, "抓到 annotated 帧")
    check(node.last_ann_aim is not None, "抓到 AIMING 帧")
    check(node.last_ann_drop is not None, "抓到 DROP 帧")
    for pth in (ANN_PNG, ANN_AIM_PNG, ANN_DROP_PNG):
        if os.path.isfile(pth):
            print("   PNG:", pth, os.path.getsize(pth), "bytes")

    with open(DET_LOG, "r", errors="ignore") as f:
        txt = f.read()
    print("\n[进程]")
    check(proc.returncode == 0, "detector rc=0 (实际 %s)" % proc.returncode)
    check("Traceback" not in txt, "detector 日志无 Traceback")

    print("\n==== T2 [%s] 结果: %s ====" % (TAG, "ALL PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
