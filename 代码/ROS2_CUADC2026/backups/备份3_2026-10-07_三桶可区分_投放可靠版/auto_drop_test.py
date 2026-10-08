#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""auto_drop_test.py — CUADC 全自动 WGS84 航点 / W/Z/I 搜索 / A/B 瞄准投放 / 侦察 RTL
ROS1 → ROS2 (Humble) 逐段译本。

源文件：代码/NUC上面的测试功能包.../20260722外场测试版本，自动飞行良好/src/
        cuadc_control/scripts/auto_drop_test.py（4347 行，只读）

ROS1 → ROS2 对应 + 房规编号映射
--------------------------------
  rospy                     -> rclpy；类构造显式传入 rclpy Node（D1 风格）。
  rospy.get_param("~x", d)  -> node_param / get_bool_param（本文件内联，见下）。
  rospy.Publisher/Subscriber-> node.create_publisher / create_subscription。
  rospy.ServiceProxy 同步   -> create_client + call_async + 轮询 future（后台
                               MultiThreadedExecutor 投递响应；D1 common 同款）。
  rospy.Rate / rospy.sleep  -> time.sleep(period)（#27：Humble C 层 SIGINT 直接
                               rcl_shutdown 不跑 Python on_shutdown 回调，
                               Rate.sleep 会永久挂死）。
  while not rospy.is_shutdown()-> while rclpy.ok() and self.node.context.ok()。
  rospy.Time.now()          -> self.node.get_clock().now().to_msg()。
  rospy.loginfo("fmt",a)    -> logger.info("fmt" % (a,))（#8：rclpy 不吃 printf 多参）。
  loginfo_throttle          -> throttle_log(node, ...)（INFO 级，等价）。
  logwarn_throttle/logerr_throttle -> logger.warning/error(..., throttle_duration_sec=)
                               （#24：节流但保级别）。
  rospy.on_shutdown         -> main() 收尾显式调用 self._on_shutdown()。
  main 退出                 -> #28 主线程 KeyboardInterrupt 收口 + #29 pthread_sigmask
                               先堵后放 + signal.signal 夺回 Python 处理器；
                               executor.shutdown(3.0) 有界。
  返回码                    -> 0=COMPLETE / 1=中断或任务失败 / 2=构造参数错。

视觉层适配（房规决策：ROS2 工作区无 BucketAimInfo，且 detector_node.py 归任务 1）
----------------------------------------------------------------------------------
  源订阅 /vision/bucket/aim_info (cuadc_vision/BucketAimInfo)。ROS2 工作区只有
  /vision/yolo/detections (YoloDetections) + /vision/color/camera_info。故本译本的
  _aim_cb 订阅 YoloDetections，_fresh_aim() 现场把它折算成与源同形的 AimView
  （count/valid/a_ned_*/b_ned_*/a_delta_*_m/b_delta_*_m）。源下游的目标锁定、跳变
  滤波、黑名单、冻结逻辑因此可逐行保留。ENU->NED：n=enu_y, e=enu_x, d=-enu_z。
  地面测试 delta 由像素偏差 * 深度 / 焦距 折算（使用 CameraInfo k[0]/k[4]）。
  不改 detector_node.py；若日后需真 detector 直供 BucketAimInfo，按报告转派。

YAML 配置驱动（单一事实源）
----------------------------
  `--config <file>` 或环境变量 `CUADC_AUTO_DROP_CONFIG` 或默认
  `~/ROS2_CUADC2026/config/auto_drop.yaml`（见 auto_drop_config.py）。
  解析优先级：命令行 `--ros-args -p` 明确覆盖 > YAML > 脚本内置默认；
  缺文件/缺字段一律回落内置默认（回归保证）。YAML 可配置依赖自启/收尾、
  航线类型、各高度与全部可调参数；`auto_shutdown_children` 控制退出收尾。

安全护栏：参数校验失败 -> ValueError -> rc=2；未授权 -> rc=2；任务失败 -> rc=1；
完成 -> rc=0。flight_only_mode 全程不启视觉/舵机/瞄准/投放，不发任何 /servo/cmd。

T2 全自动桶流程（2026-09-30）
--------------------------------
  投放区上方 W/Z/I 搜索（search_altitude_m；起飞后先到该高度再飞往投放区）期间，
  订阅 /vision/target_global (GeoTarget, 每帧最多 3 个目标, target_id=1/2/3) 做多目标
  累积，并用 /vision/yolo/detections 的 depth_m 与识别框对角线算尺寸比值：

      diag_px = hypot(x_max - x_min, y_max - y_min)      (diag_px>0)
      ratio   = depth_m * hypot(width_px/fx, height_px/fy) (meters)

  这是识别框物理对角线代理值，必须用相机实测的 200mm 标定参考值。
  比值方向（默认，可用 mission.bucket_ratio_invert 翻转）：
  **比值越大 = 桶越大**。以 200mm 标定参考值
  `mission.bucket_ratio_ref_200`（手工实测填入）与容差 `mission.bucket_ratio_tol` 分档：
      ratio > ref*(1+tol)  -> 大号(≈250mm, class=large)
      ratio < ref*(1-tol)  -> 小号(≈150mm, class=small)
      否则                 -> 中号(≈200mm, class=mid)

  GeoTarget↔YoloDetections 关联：同一帧内按 (-confidence, center_x) 排序索引对齐
  （与 detector 的 target_id 编号键完全一致），再用"中心像素距 ≤60px 且置信度差
  ≤0.2"做最近邻校验，校验失败则退化为同帧中心像素最近邻。位置估计用逐桶滑动
  中位数（去跳变）。注意 GeoTarget.enu_* 是**相对机身的 ENU 偏移**（旧字段语义），
  T2 消费时加当前本地位姿换算成绝对 ENU（否则飞机一移动同一桶会被拆散）。

  瞄准（A/B 槽位）：A=最高置信度目标、B=次高且像素可区分者；相机系→机体系(FRD)
  →ENU 时 **z 取反**（FRD→ENU；旧实现漏翻导致目标被算到飞机上方、越爬越高）。

  选桶（先近后远）：目标桶 = mid(中号) + small(小号)，按飞机当前水平距离升序投放；
  只搜索到两个桶 -> 直接依次投放这两个（降级）；档位不可辨 -> 取最接近 ref 的桶 +
  比值更大的另一个（更小号）。降级原因写入记录文件 notes。记录原子写
  `bucket_record_dir`（默认 ~/ROS2_CUADC2026/logs/buckets）：
  `buckets_<时间戳>.json` + 覆盖 `buckets_latest.json`（结构见 §六.5 冻结 schema）。

  模式：recon_enabled=false -> 投放完直接 RTL；recon_enabled=true -> 先飞侦察区悬停再
  RTL；aim_only_mode=true -> 搜索识别到 >=3 个桶后切入瞄准（不投放），瞄准完成后
  RTL；flight_only_mode 与 ground_test 路径完全不变。
"""

import atexit
import math
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import threading
import time
import json
from collections import deque
from datetime import datetime
from statistics import median

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import (
    ExtendedState,
    GlobalPositionTarget,
    RCIn,
    State,
    StatusText,
)
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, SetMode
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64, String

from cuadc_vision.msg import MissionStatus, YoloDetections, GeoTarget

# YAML 配置（单一事实源 config/auto_drop.yaml）——与本脚本同目录安装。
# 读失败只回落内置默认，绝不让节点因缺配置而崩（回归保证）。
try:
    import auto_drop_config
except Exception:  # pragma: no cover
    auto_drop_config = None


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563


def monotonic_s():
    return time.monotonic()


def node_param(node, name, default):
    """等价 rospy.get_param("~"+name, default)，declare-once。"""
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    try:
        return node.get_parameter(name).value
    except Exception:
        return default


def get_bool_param(node, name, default):
    value = node_param(node, name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def throttle_log(node, interval_s, fmt, *args):
    """等价 rospy.loginfo_throttle（INFO 级）。"""
    key = "_throttle_last_" + str(abs(hash(fmt)))
    now = monotonic_s()
    last = getattr(node, key, 0.0)
    if now - last >= interval_s:
        setattr(node, key, now)
        node.get_logger().info(fmt % args if args else fmt)


def destination_wgs84(latitude, longitude, distance, heading_deg):
    lat_rad = math.radians(latitude)
    heading_rad = math.radians(heading_deg)
    eccentricity_sq = WGS84_F * (2.0 - WGS84_F)
    sin_lat = math.sin(lat_rad)
    denominator = math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
    prime_vertical_radius = WGS84_A / denominator
    meridian_radius = WGS84_A * (1.0 - eccentricity_sq) / denominator ** 3
    north = distance * math.cos(heading_rad)
    east = distance * math.sin(heading_rad)
    target_lat = latitude + math.degrees(north / meridian_radius)
    cos_lat = math.cos(lat_rad)
    if abs(cos_lat) < 1e-12:
        raise ValueError("当前位置过于接近极点，无法解算经度")
    target_lon = longitude + math.degrees(
        east / (prime_vertical_radius * cos_lat)
    )
    return target_lat, (target_lon + 180.0) % 360.0 - 180.0


def horizontal_wgs84_distance(lat1, lon1, lat2, lon2):
    mean_lat = math.radians((lat1 + lat2) * 0.5)
    eccentricity_sq = WGS84_F * (2.0 - WGS84_F)
    sin_lat = math.sin(mean_lat)
    denominator = math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
    prime_vertical_radius = WGS84_A / denominator
    meridian_radius = WGS84_A * (1.0 - eccentricity_sq) / denominator ** 3
    north = math.radians(lat2 - lat1) * meridian_radius
    dlon = math.radians((lon2 - lon1 + 180.0) % 360.0 - 180.0)
    east = dlon * prime_vertical_radius * math.cos(mean_lat)
    return math.hypot(north, east)


class AimView:
    """与源 BucketAimInfo 同形的瞄准视图（由 YoloDetections 折算）。"""

    __slots__ = (
        "count", "valid", "confidence", "source_class",
        "a_ned_valid", "a_ned_n", "a_ned_e", "a_ned_d",
        "b_ned_valid", "b_ned_n", "b_ned_e", "b_ned_d",
        "a_delta_x_m", "a_delta_y_m", "b_delta_x_m", "b_delta_y_m",
    )

    def __init__(self):
        self.count = 0
        self.valid = False
        self.confidence = 0.0
        self.source_class = ""
        self.a_ned_valid = self.b_ned_valid = False
        self.a_ned_n = self.a_ned_e = self.a_ned_d = 0.0
        self.b_ned_n = self.b_ned_e = self.b_ned_d = 0.0
        self.a_delta_x_m = self.a_delta_y_m = 0.0
        self.b_delta_x_m = self.b_delta_y_m = 0.0


class AutoDropTest:
    WGS84_A = 6378137.0
    WGS84_F = 1.0 / 298.257223563

    @staticmethod
    def _normalize_area_search_pattern(value):
        pattern = str(value).strip().upper()
        return "I" if pattern == "LINE" else pattern

    def __init__(self, node):
        self.node = node
        # ---- 仅飞行航线验证模式（四参数，源 L56-65）----
        self.flight_only_mode = get_bool_param(node, "flight_only_mode", False)
        self.flight_only_pattern_laps = int(node_param(node, "flight_only_pattern_laps", 1))
        self.flight_only_search_altitude_m = float(
            node_param(node, "flight_only_search_altitude_m", 3.0))
        self.flight_only_continue_to_recon = get_bool_param(
            node, "flight_only_continue_to_recon", True)

        # ---- 任务模式（源 L66-69）----
        self.ground_test = get_bool_param(node, "ground_test", False)
        self.full_mission_mode = get_bool_param(node, "full_mission_mode", True)
        self.autonomous_mode = self.full_mission_mode
        self.enable_b_dropper = get_bool_param(node, "enable_b_dropper", True)

        # ---- 航点/高度基准（源 L70-87）----
        self.takeoff_altitude_m = float(node_param(node, "takeoff_altitude_m", 3.0))
        self.drop_zone_transit_altitude_m = float(
            node_param(node, "drop_zone_transit_altitude_m", self.takeoff_altitude_m))
        self.drop_zone_distance_m = float(node_param(node, "drop_zone_distance_m", 32.0))
        self.recon_zone_distance_m = float(node_param(node, "recon_zone_distance_m", 60.0))
        self.recon_altitude_m = float(node_param(node, "recon_altitude_m", 5.0))
        self.recon_hold_s = float(node_param(node, "recon_hold_s", 5.0))
        # 锁定航向（度，0=北顺时针；<0=用起始航向）。#68
        self.drop_zone_bearing_deg = float(
            node_param(node, "drop_zone_bearing_deg", -1.0))

        # ---- T2 桶搜索/选桶/记录 与模式开关（2026-09-30）----
        # search_altitude_m 默认跟随 drop_zone_transit_altitude_m（不传新键时行为不变）。
        self.search_altitude_m = float(
            node_param(node, "search_altitude_m", self.drop_zone_transit_altitude_m))
        self.recon_enabled = get_bool_param(node, "recon_enabled", False)
        self.aim_only_mode = get_bool_param(node, "aim_only_mode", False)
        # 仅搜索记录：视觉开、搜索并记录桶坐标后直接返航（不瞄准、不投放）。
        # 与 aim_only_mode 互斥（本文档优先；两者同真时按 survey_only 处理并告警）。
        self.survey_only_mode = get_bool_param(node, "survey_only_mode", False)
        if self.survey_only_mode and self.aim_only_mode:
            self.node.get_logger().warn(
                "survey_only_mode 与 aim_only_mode 同时为 true；按 survey_only_mode 处理（仅搜索记录）")
        self.record_buckets = get_bool_param(node, "record_buckets", True)
        self.bucket_ratio_ref_200 = float(
            node_param(node, "bucket_ratio_ref_200", 0.20))
        self.bucket_ratio_tol = float(node_param(node, "bucket_ratio_tol", 0.15))
        self.bucket_ratio_invert = get_bool_param(node, "bucket_ratio_invert", False)
        self.bucket_record_dir = os.path.expanduser(str(node_param(
            node, "bucket_record_dir", "~/ROS2_CUADC2026/logs/buckets")))
        # 同一桶的 ENU 关联门限（米）：须明显小于场地最小桶距（规则允许三筒随机
        # 摆放在 8×5m 内，world 实测最小 1.35m），又明显大于观测噪声（仿真 ~0.05m）。
        # 2026-09-30 修正（错题 #59）：原硬编码 1.5m 会把 1.35m 间距的两桶并成假桶。
        self.bucket_assoc_distance_m = float(
            node_param(node, "bucket_assoc_distance_m", 0.6))
        self.bucket_min_samples = max(
            1, int(node_param(node, "bucket_min_samples", 3)))
        self.waypoint_tolerance_m = float(node_param(node, "waypoint_tolerance_m", 1.0))
        self.waypoint_vertical_tolerance_m = float(
            node_param(node, "waypoint_vertical_tolerance_m", 0.4))
        self.waypoint_arrival_hold_s = float(
            node_param(node, "waypoint_arrival_hold_s", 2.0))
        self.waypoint_timeout_s = float(node_param(node, "waypoint_timeout_s", 120.0))

        # ---- 依赖自启（ROS2 房规：默认全部 false，交给 demo_up/launch 管理，
        #      避免与常驻管理双起；仍可传 true 走 ros2 launch/run）----
        self.auto_start_mavros = get_bool_param(node, "auto_start_mavros", False)
        self.mavros_fcu_url = str(node_param(node, "mavros_fcu_url", "/dev/ttyACM0:115200"))
        self.detector_show_window = get_bool_param(node, "detector_show_window", True)
        # 相机源（仿真 gazebo / 真机 realsense）——拉起相机/识别时下传。
        self.camera_source = str(node_param(node, "camera_source", "gazebo"))
        # TRT 包专属（detector_package=cuadc_vision_trt 时生效）
        self.trt_direct_camera = get_bool_param(node, "trt_direct_camera", True)
        self.trt_width = int(node_param(node, "trt_width", 1920))
        self.trt_height = int(node_param(node, "trt_height", 1080))
        self.trt_cam_fps = int(node_param(node, "trt_cam_fps", 30))
        self.trt_depth_width = int(node_param(node, "trt_depth_width", 1280))
        self.trt_depth_height = int(node_param(node, "trt_depth_height", 720))
        self.trt_imgsz = int(node_param(node, "trt_imgsz", 640))
        self.trt_engine = str(node_param(node, "trt_engine", "640"))
        # 任务结束是否自动收尾本脚本拉起的子进程（内置默认 true = 与改造前一致）。
        self.auto_shutdown_children = get_bool_param(
            node, "auto_shutdown_children", True)
        self.dependency_restart_attempts = max(
            0, int(node_param(node, "dependency_restart_attempts", 0)))
        self.dependency_restart_backoff_s = max(
            0.0, float(node_param(node, "dependency_restart_backoff_s", 2.0)))
        self.detector_ready_timeout_s = float(
            node_param(node, "detector_ready_timeout_s", 60.0))
        self.servo_ready_timeout_s = float(
            node_param(node, "servo_ready_timeout_s", 20.0))
        self.enroute_detect_stable_s = float(
            node_param(node, "enroute_detect_stable_s", 0.3))
        self.second_target_search_timeout_s = float(
            node_param(node, "second_target_search_timeout_s", 30.0))
        self.aim_timeout_s = float(node_param(node, "aim_timeout_s", 20.0))

        # ---- 目标锁定/滤波（源 L112-126）----
        self.target_lock_center_threshold_m = float(
            node_param(node, "target_lock_center_threshold_m", 0.20))
        self.target_lock_window_s = float(
            node_param(node, "target_lock_window_s", 0.5))
        self.target_lock_min_samples = max(
            2, int(node_param(node, "target_lock_min_samples", 5)))
        self.target_lock_max_xy_span_m = float(
            node_param(node, "target_lock_max_xy_span_m", 0.10))
        self.target_lock_max_d_span_m = float(
            node_param(node, "target_lock_max_d_span_m", 0.15))

        # ---- 恢复/服务（源 L127-146）----
        self.navigation_timeout_s = float(node_param(node, "navigation_timeout_s", 2.0))
        self.navigation_grace_s = float(node_param(node, "navigation_grace_s", 5.0))
        self.connection_loss_grace_s = float(
            node_param(node, "connection_loss_grace_s", 10.0))
        self.service_retry_attempts = max(
            1, int(node_param(node, "service_retry_attempts", 1)))
        self.service_retry_backoff_s = max(
            0.0, float(node_param(node, "service_retry_backoff_s", 0.5)))
        # 183151 实飞：FCU 报 "Arm: Safety Switch" 时只报“自动解锁失败”，没提示
        # 操作手按开关。该窗口内自动重试并明确提示（0=旧行为）。
        self.arming_safety_switch_grace_s = max(
            0.0, float(node_param(node, "arming_safety_switch_grace_s", 8.0)))
        self.takeoff_timeout_s = float(node_param(node, "takeoff_timeout_s", 40.0))
        self.takeoff_started_threshold_m = float(
            node_param(node, "takeoff_started_threshold_m", 0.30))
        self.rtl_timeout_s = float(node_param(node, "rtl_timeout_s", 240.0))

        # ---- 投放（源 L147-158）----
        self.align_threshold_m = float(node_param(node, "align_threshold_m", 0.15))
        # 双投放点（2026-10-05 用户拍板几何）：相机在机腹中心，A/B 抛投器中心在
        # 相机前后各 crosshair_offset_m=0.05m（前=图像上、后=图像下）。准星 =
        # 虚拟相机原点沿机体系 X ±crosshair_offset_m；front_dropper 指定哪个舵机是
        # "前"；对准触发用相机系差值（米）。启动识别时同值下传 dual_offset_m。
        self.crosshair_offset_m = float(node_param(node, "crosshair_offset_m", 0.05))
        # 2026-10-05 垂线标定（用户实读）：A 实际在相机前 68mm、B 后 74mm（均值 71mm），
        # 且两投均比相机中轴偏左约 48~79mm（均值 64mm）。前后用 crosshair_offset_m，
        # 横向用 crosshair_lateral_y_m（机体系 Y 正=右）。相机倒装 invert_camera_x=true
        # 时，原始像素 dx 正=图像右=机体左，故该值取 -64mm。
        self.crosshair_lateral_y_m = float(
            node_param(node, "crosshair_lateral_y_m", 0.0))
        self.camera_invert_x = get_bool_param(node, "camera_invert_x", True)
        # 相机光学中心到机体 CG 的竖直距离（ROS1 detector_node.launch=0.40）。
        # 识别侧 _compute_geo 已加该值；控制侧 _aim_ned_for_det 若漏加，锁定 NED 的
        # D 会比真实地面高 0.4m，物理投放高度=配置+0.4m（风漂/落体时间都变大）。
        self.camera_mount_z = float(node_param(node, "camera_mount_z", 0.40))
        self.front_dropper = str(node_param(node, "front_dropper", "A")).strip().upper()
        self.aim_cam_align_threshold_m = float(
            node_param(node, "aim_cam_align_threshold_m", 0.05))
        # T2 修（2026-10-04 实飞 181706）：survey 位置估计误差 0.67~0.79m，旧硬编码
        # 0.6m 门限把选中桶的所有观测拒掉 → B 桶 30s 重捕获超时。门限与重锚定均
        # 可配置，默认 0.8/1.0m，仍小于场地最小桶距（防误锁邻桶）。
        self.preferred_target_gate_m = float(
            node_param(node, "preferred_target_gate_m", 0.8))
        self.preferred_target_reanchor_m = float(
            node_param(node, "preferred_target_reanchor_m", 1.0))
        self.preferred_target_reanchor_s = float(
            node_param(node, "preferred_target_reanchor_s", 2.0))
        # FINAL 对准判据滑窗（s）：dx/dy 取窗口内中位数再判，抑制单帧噪声（0=关）。
        self.align_filter_window_s = float(
            node_param(node, "align_filter_window_s", 0.8))
        self.target_drop_height_m = float(
            node_param(node, "target_drop_height_m", 1.5))
        self.height_tolerance_m = float(node_param(node, "height_tolerance_m", 0.15))
        self.stable_time_s = float(node_param(node, "stable_time_s", 0.3))
        self.setpoint_interval_s = float(node_param(node, "setpoint_interval_s", 0.1))
        self.after_a_delay_s = float(node_param(node, "after_a_delay_s", 3.0))

        # ---- 视觉/舵机有效配置（源 L159-179，flight_only 强制禁用）----
        self.auto_start_detector = get_bool_param(node, "auto_start_detector", False)
        self.auto_start_servo_test = get_bool_param(node, "auto_start_servo_test", False)
        # 舵机 A/B 输出通道与 PWM（转发给 servo_test.py；#43 真机 AUX1/A2=SERVO9/10）。
        self.servo_a_channel = int(node_param(node, "servo_a_channel", 9))
        self.servo_b_channel = int(node_param(node, "servo_b_channel", 10))
        self.pwm_open = int(node_param(node, "pwm_open", 1500))
        self.pwm_close = int(node_param(node, "pwm_close", 1000))
        self.enable_ch5 = get_bool_param(node, "enable_ch5", True)
        self.enable_ch6 = get_bool_param(node, "enable_ch6", True)
        self.detector_enabled = self.auto_start_detector and not self.flight_only_mode
        self.servo_enabled = self.auto_start_servo_test and not self.flight_only_mode
        self.b_dropper_enabled = self.enable_b_dropper and not self.flight_only_mode
        self.vision_enabled = not self.flight_only_mode
        self.servo_interface_enabled = not self.flight_only_mode
        self.aiming_enabled = not self.flight_only_mode
        self.drop_enabled = not self.flight_only_mode
        self.close_after_drop = get_bool_param(node, "close_after_drop", False)
        self.dry_run = get_bool_param(node, "dry_run", False)
        self.min_bucket_count = max(1, int(node_param(node, "min_bucket_count", 1)))

        # ---- 自主瞄准状态机（源 L181-254）----
        self.acquire_stable_time_s = float(node_param(node, "acquire_stable_time_s", 0.8))
        self.coarse_align_height_m = float(node_param(node, "coarse_align_height_m", 2.0))
        self.fine_align_height_m = float(node_param(node, "fine_align_height_m", 1.7))
        self.coarse_align_threshold_m = float(
            node_param(node, "coarse_align_threshold_m", 0.45))
        self.fine_align_threshold_m = float(
            node_param(node, "fine_align_threshold_m", 0.20))
        self.phase_stable_time_s = float(node_param(node, "phase_stable_time_s", 0.5))
        self.target_filter_alpha = float(node_param(node, "target_filter_alpha", 0.25))
        self.max_target_jump_m = float(node_param(node, "max_target_jump_m", 0.80))
        # 2026-10-05 用户口径：瞄准时中心点小抖动忽略（<死区不动 XY），大变化才跟新。
        self.target_track_deadband_m = float(
            node_param(node, "target_track_deadband_m", 0.05))
        # 20261005_131212 实据：冻结后制导点跟着测量 EMA 漂 ±0.2m，飞机追噪声形成
        # 极限环（hspeed 中位 0.18m/s，FINAL 183 帧仅 1 帧满足对准）。冻结后改为慢速
        # 带死区跟踪：每 hold_s 才允许动一次、单次最多 step_m、误差 < deadband 不动。
        self.aim_guidance_hold_s = float(
            node_param(node, "aim_guidance_hold_s", 0.5))
        self.aim_guidance_step_m = float(
            node_param(node, "aim_guidance_step_m", 0.05))
        self.aim_guidance_deadband_m = float(
            node_param(node, "aim_guidance_deadband_m", 0.03))
        self.brief_loss_s = float(node_param(node, "brief_loss_s", 1.5))
        self.loss_hover_s = float(node_param(node, "loss_hover_s", 2.0))
        # 2026-10-05 用户拍板：丢失后重新搜索高度先设 5m。
        self.reacquire_height_m = float(node_param(node, "reacquire_height_m", 5.0))
        self.search_start_radius_m = float(node_param(node, "search_start_radius_m", 0.30))
        self.search_radius_step_m = float(node_param(node, "search_radius_step_m", 0.30))
        self.search_max_radius_m = float(node_param(node, "search_max_radius_m", 1.20))
        self.search_dwell_s = float(node_param(node, "search_dwell_s", 1.5))
        self.target_loss_timeout_s = float(
            node_param(node, "target_loss_timeout_s", 30.0))
        self.max_horizontal_setpoint_distance_m = float(
            node_param(node, "max_horizontal_setpoint_distance_m", 0.60))
        self.max_vertical_setpoint_distance_m = float(
            node_param(node, "max_vertical_setpoint_distance_m", 0.20))
        self.max_horizontal_setpoint_rate_mps = float(
            node_param(node, "max_horizontal_setpoint_rate_mps", 0.25))
        self.max_vertical_setpoint_rate_mps = float(
            node_param(node, "max_vertical_setpoint_rate_mps", 0.20))
        self.phase_max_horizontal_speed_mps = float(
            node_param(node, "phase_max_horizontal_speed_mps", 0.25))
        self.phase_max_vertical_speed_mps = float(
            node_param(node, "phase_max_vertical_speed_mps", 0.20))
        self.max_horizontal_speed_mps = float(
            node_param(node, "max_horizontal_speed_mps", 0.15))
        self.max_vertical_speed_mps = float(
            node_param(node, "max_vertical_speed_mps", 0.15))
        self.require_distinct_b_target = get_bool_param(
            node, "require_distinct_b_target", True)
        self.distinct_target_distance_m = float(
            node_param(node, "distinct_target_distance_m", 0.60))

        # ---- 投放区快速搜索（源 L256-286）----
        self.area_search_pattern = self._normalize_area_search_pattern(
            node_param(node, "area_search_pattern", "W"))
        self.drop_zone_width_m = float(node_param(node, "drop_zone_width_m", 8.0))
        self.drop_zone_length_m = float(node_param(node, "drop_zone_length_m", 5.0))
        # 识别范围：在"参考高度"下测得的地面覆盖（宽=机身边方向/画面横向，长=机身前后/画面纵向）；
        # 实际搜索时按 实际搜索高度/参考高度 线性折算（针孔线性）。
        self.detect_range_ref_altitude_m = float(
            node_param(node, "detect_range_ref_altitude_m", 3.0))
        self.detect_range_width_m = float(node_param(node, "detect_range_width_m", 2.25))
        self.detect_range_length_m = float(node_param(node, "detect_range_length_m", 1.5))
        self.area_search_timeout_s = float(
            node_param(node, "area_search_timeout_s", 120.0))
        self.area_search_waypoint_tolerance_m = float(
            node_param(node, "area_search_waypoint_tolerance_m", 0.25))
        self.area_search_height_tolerance_m = float(
            node_param(node, "area_search_height_tolerance_m", 0.20))
        self.area_search_dwell_s = float(node_param(node, "area_search_dwell_s", 0.5))
        self.reuse_area_search_anchor_for_b = get_bool_param(
            node, "reuse_area_search_anchor_for_b", True)

        # ---- 遥测/服务超时（源 L288-321）----
        self.aim_info_timeout_s = float(node_param(node, "aim_info_timeout_s", 1.0))
        self.state_timeout_s = float(node_param(node, "state_timeout_s", 2.0))
        self.guided_state_grace_s = float(node_param(node, "guided_state_grace_s", 5.0))
        self.pose_timeout_s = float(node_param(node, "pose_timeout_s", self.state_timeout_s))
        self.guided_pose_grace_s = float(node_param(node, "guided_pose_grace_s", 5.0))
        self.local_position_rate_hz = float(
            node_param(node, "local_position_rate_hz", 10.0))
        self.diagnostic_log_interval_s = float(
            node_param(node, "diagnostic_log_interval_s", 1.0))
        self.connection_timeout_s = float(node_param(node, "connection_timeout_s", 30.0))
        self.mode_timeout_s = float(node_param(node, "mode_timeout_s", 8.0))
        self.service_timeout_s = float(node_param(node, "service_timeout_s", 10.0))
        self.servo_wait_timeout_s = float(node_param(node, "servo_wait_timeout_s", 20.0))
        self.close_delay_s = float(node_param(node, "close_delay_s", 0.8))
        self.detector_package = str(node_param(node, "detector_package", "cuadc_vision"))
        self.detector_launch = str(
            node_param(node, "detector_launch", "detector_node.launch.py"))
        self.vision_min_confidence = float(
            node_param(node, "vision_min_confidence", 0.30))

        self._validate_params()
        self._init_state()
        self._init_ros()

    def _validate_params(self):
        if self.align_threshold_m <= 0.0:
            raise ValueError("align_threshold_m 必须大于 0")
        if self.front_dropper not in ("A", "B"):
            raise ValueError("front_dropper 只能是 A 或 B")
        if self.crosshair_offset_m <= 0.0:
            raise ValueError("crosshair_offset_m 必须大于 0")
        if abs(self.crosshair_lateral_y_m) > 0.5:
            raise ValueError("crosshair_lateral_y_m 超出 ±0.5m 合理范围")
        if self.camera_mount_z < 0.0:
            raise ValueError("camera_mount_z 不能为负（机体系 Z+ 向下，相机在 CG 下方为正）")
        if self.aim_cam_align_threshold_m <= 0.0:
            raise ValueError("aim_cam_align_threshold_m 必须大于 0")
        if self.preferred_target_gate_m <= 0.0:
            raise ValueError("preferred_target_gate_m 必须大于 0")
        if self.preferred_target_reanchor_m < self.preferred_target_gate_m:
            raise ValueError(
                "preferred_target_reanchor_m 不能小于 preferred_target_gate_m")
        if self.preferred_target_reanchor_s < 0.0 or self.align_filter_window_s < 0.0:
            raise ValueError("preferred_target_reanchor_s 与 align_filter_window_s 不能为负")
        if self.target_drop_height_m <= 0.0:
            raise ValueError("target_drop_height_m 必须大于 0")
        if self.height_tolerance_m <= 0.0:
            raise ValueError("height_tolerance_m 必须大于 0")
        if self.stable_time_s <= 0.0:
            raise ValueError("stable_time_s 必须大于 0")
        if self.setpoint_interval_s <= 0.0:
            raise ValueError("setpoint_interval_s 必须大于 0")
        if self.acquire_stable_time_s <= 0.0 or self.phase_stable_time_s <= 0.0:
            raise ValueError("目标获取和阶段稳定时间必须大于 0")
        if self.coarse_align_height_m < self.fine_align_height_m:
            raise ValueError("coarse_align_height_m 不能低于 fine_align_height_m")
        if self.fine_align_height_m < self.target_drop_height_m:
            raise ValueError("fine_align_height_m 不能低于 target_drop_height_m")
        if self.reacquire_height_m < self.fine_align_height_m:
            raise ValueError("reacquire_height_m 不能低于 fine_align_height_m")
        if not 0.0 < self.target_filter_alpha <= 1.0:
            raise ValueError("target_filter_alpha 必须在 (0, 1] 范围")
        if not 0.0 <= self.target_track_deadband_m < self.max_target_jump_m:
            raise ValueError(
                "target_track_deadband_m 必须在 [0, max_target_jump_m) 范围")
        if (self.aim_guidance_hold_s <= 0.0 or self.aim_guidance_step_m <= 0.0
                or self.aim_guidance_deadband_m < 0.0):
            raise ValueError(
                "aim_guidance_hold_s/step_m 必须大于 0，deadband_m 不能为负")
        positive_values = (
            self.coarse_align_threshold_m, self.fine_align_threshold_m,
            self.max_target_jump_m, self.search_start_radius_m,
            self.search_radius_step_m, self.search_max_radius_m,
            self.search_dwell_s, self.target_loss_timeout_s,
            self.max_horizontal_setpoint_distance_m,
            self.max_vertical_setpoint_distance_m,
            self.max_horizontal_setpoint_rate_mps,
            self.max_vertical_setpoint_rate_mps,
            self.phase_max_horizontal_speed_mps,
            self.phase_max_vertical_speed_mps,
            self.max_horizontal_speed_mps, self.max_vertical_speed_mps,
            self.distinct_target_distance_m,
        )
        if any(value <= 0.0 for value in positive_values):
            raise ValueError("自主瞄准距离、速度、搜索和超时参数必须大于 0")
        if self.brief_loss_s < 0.0 or self.loss_hover_s < self.brief_loss_s:
            raise ValueError("必须满足 0 <= brief_loss_s <= loss_hover_s")
        if self.diagnostic_log_interval_s <= 0.0:
            raise ValueError("diagnostic_log_interval_s 必须大于 0")
        if self.guided_state_grace_s <= self.state_timeout_s:
            raise ValueError("guided_state_grace_s 必须大于 state_timeout_s")
        if self.pose_timeout_s <= 0.0:
            raise ValueError("pose_timeout_s 必须大于 0")
        if self.guided_pose_grace_s <= self.pose_timeout_s:
            raise ValueError("guided_pose_grace_s 必须大于 pose_timeout_s")
        if self.local_position_rate_hz <= 0.0:
            raise ValueError("local_position_rate_hz 必须大于 0")
        if self.area_search_pattern not in ("W", "Z", "I"):
            raise ValueError("area_search_pattern 只能是 W、Z、I 或 LINE")
        if self.flight_only_pattern_laps < 1:
            raise ValueError("flight_only_pattern_laps 必须大于等于 1")
        if self.flight_only_search_altitude_m <= 0.0:
            raise ValueError("flight_only_search_altitude_m 必须大于 0")
        if self.flight_only_search_altitude_m > self.recon_altitude_m:
            raise ValueError(
                "flight_only_search_altitude_m 必须小于等于 recon_altitude_m")
        area_positive_values = (
            self.drop_zone_width_m, self.drop_zone_length_m,
            self.detect_range_ref_altitude_m, self.detect_range_width_m,
            self.detect_range_length_m,
            self.area_search_timeout_s, self.area_search_waypoint_tolerance_m,
            self.area_search_height_tolerance_m, self.area_search_dwell_s,
        )
        if any(value <= 0.0 for value in area_positive_values):
            raise ValueError("区域搜索尺寸/识别范围、超时、容差和停留时间必须大于 0")
        eff_range_w, eff_range_l = self._effective_detect_range()
        if eff_range_w >= self.drop_zone_width_m:
            raise ValueError(
                "识别范围宽度（按实际搜索高度折算=%.2fm）必须小于投放区宽度"
                % eff_range_w)
        if eff_range_l >= self.drop_zone_length_m:
            raise ValueError(
                "识别范围长度（按实际搜索高度折算=%.2fm）必须小于投放区长度"
                % eff_range_l)
        if self.drop_zone_distance_m <= 0.0:
            raise ValueError("drop_zone_distance_m 必须大于 0")
        if self.recon_zone_distance_m <= self.drop_zone_distance_m:
            raise ValueError("recon_zone_distance_m 必须大于 drop_zone_distance_m")
        if self.takeoff_altitude_m <= 0.0:
            raise ValueError("takeoff_altitude_m 必须大于 0")
        if self.drop_zone_transit_altitude_m <= 0.0:
            raise ValueError("drop_zone_transit_altitude_m 必须大于 0")
        if self.recon_altitude_m < self.takeoff_altitude_m:
            raise ValueError("recon_altitude_m 必须大于等于 takeoff_altitude_m")
        if self.recon_altitude_m < self.drop_zone_transit_altitude_m:
            raise ValueError("recon_altitude_m 必须大于等于 drop_zone_transit_altitude_m")
        if self.search_altitude_m <= 0.0:
            raise ValueError("search_altitude_m 必须大于 0")
        if self.search_altitude_m > self.recon_altitude_m:
            raise ValueError("search_altitude_m 必须小于等于 recon_altitude_m")
        if self.bucket_ratio_ref_200 <= 0.0:
            raise ValueError("bucket_ratio_ref_200 必须大于 0")
        if not 0.0 <= self.bucket_ratio_tol <= 1.0:
            raise ValueError("bucket_ratio_tol 必须在 0..1 范围")
        if self.bucket_assoc_distance_m <= 0.0:
            raise ValueError("bucket_assoc_distance_m 必须大于 0")
        if not self.bucket_record_dir:
            raise ValueError("bucket_record_dir 不能为空")
        mission_positive = (
            self.recon_hold_s, self.waypoint_tolerance_m,
            self.waypoint_vertical_tolerance_m, self.waypoint_arrival_hold_s,
            self.waypoint_timeout_s, self.detector_ready_timeout_s,
            self.servo_ready_timeout_s, self.enroute_detect_stable_s,
            self.second_target_search_timeout_s, self.aim_timeout_s,
            self.target_lock_center_threshold_m, self.target_lock_window_s,
            self.target_lock_max_xy_span_m, self.target_lock_max_d_span_m,
            self.navigation_timeout_s, self.navigation_grace_s,
            self.connection_loss_grace_s, self.takeoff_timeout_s,
            self.takeoff_started_threshold_m, self.rtl_timeout_s,
        )
        if any(value <= 0.0 for value in mission_positive):
            raise ValueError("完整任务航点、恢复、瞄准和超时参数必须大于 0")
        if self.navigation_grace_s <= self.navigation_timeout_s:
            raise ValueError("navigation_grace_s 必须大于 navigation_timeout_s")

    # ==================================================================
    # 运行期状态与 ROS 接线
    # ==================================================================
    def _init_state(self):
        self._lock = threading.RLock()
        self.current_state = None
        self.state_received_at = 0.0
        self.latest_aim = None
        self.aim_received_at = 0.0
        # T2 选桶投放：偏好目标 NED（选出待投桶后设置，瞄准锁定它而不是"最高置信度"）
        self.preferred_target_ned = None
        # #69：偏好桶门限（m，参数在 __init__）。选桶后瞄准只允许锁定"选中的桶"；
        # 门限过大（曾=2.0）会在选中桶暂时不在视野时锁定邻近的别的桶（实飞 2026-10-02：
        # A 应投 15cm 小桶却锁到 1.2m 外的 25cm 桶 → 40s 超时）；太小（曾=0.6）又会被
        # survey 估计误差（2026-10-04 实测 0.67~0.79m）长期拒收 → 30s 超时。
        # 现在门限 0.8m + 持续近邻重锚定（见 _build_aim_view），兼顾两者。
        self._preferred_reanchor_since = None
        self.current_detections = None
        self.detections_received_at = 0.0
        self.camera_fx = 0.0
        self.camera_fy = 0.0
        self.camera_center_x = 640.0
        self.camera_center_y = 360.0
        self.camera_info_received_at = 0.0
        self.current_pose = None
        self.pose_received_at = 0.0
        self.current_velocity = None
        self.velocity_received_at = 0.0
        self.current_fix = None
        self.fix_received_at = 0.0
        self.heading_deg = None
        self.heading_received_at = 0.0
        self.relative_altitude = None
        self.relative_altitude_received_at = 0.0
        self.current_extended_state = None
        self.extended_state_received_at = 0.0
        self.current_rc = None
        self.rc_received_at = 0.0
        self.recent_statustext = deque(maxlen=8)
        self.last_setpoint_desired_enu = None
        self.last_setpoint_commanded_enu = None
        self.last_setpoint_label = ""
        self.last_setpoint_at = 0.0
        self.servo_ready = False
        self.guided_was_requested = False
        self._set_mode_srv = None
        self._arming_srv = None
        self._takeoff_srv = None
        self._message_interval_srv = None
        self._children = []
        self._child_specs = {}
        self._dependency_restart_counts = {}
        self._cleanup_done = False
        self.ammo_a = 1
        self.ammo_b = 1 if self.enable_b_dropper else 0
        self.completed_drop_targets = []
        self.last_aligned_target_ned = None
        self.area_search_anchor = None
        self.frozen_target_ned = None
        self.forced_drop = False
        self.top_state = "BOOT"
        self.previous_top_state = None
        self.recovery_resume_state = None
        self.airborne = False
        self.authorized = False
        self.start_ned = None
        self.start_wgs84 = None
        self.start_heading_deg = None
        self.lock_heading_deg = None  # #68：投放区/侦察区锁定航向（None=用 start_heading_deg）
        self.start_relative_altitude = None
        self.ground_relative_altitude = None
        self.drop_zone_wgs84 = None
        self.recon_zone_wgs84 = None
        self.drop_zone_reached = False
        self.enroute_a_intercepted = False
        self.last_safe_setpoint_kind = None
        self.last_safe_global_target = None
        self.flight_only_current_lap = 0
        self.flight_only_current_point = 0
        self.flight_only_total_points = 0
        # ---- T2 多目标桶估计状态 ----
        self.latest_geo_targets = {}          # target_id -> GeoTarget
        self.geo_targets_received_at = 0.0
        self.geo_count = 0
        self.buckets = []                     # 估计出的桶（含滑动样本）
        self.selected_buckets = []            # 选中的桶（先近后远）
        self.bucket_notes = []                # 降级/说明
        # 瞄准对照日志（aim_trace.jsonl，落本轮 run 目录）：每轮循环一行，
        # 含逐检测框 conf/depth/position_valid/camera/bbox + 控制侧判据，
        # 用于把"拒绝投放"和识别侧异常对照排查（2026-10-05 需求）。
        self._aim_trace_fh = None

    def _init_ros(self):
        log = self.node.get_logger()
        if self.flight_only_mode:
            log.warn("================ FLIGHT ONLY MODE ================")
            log.warn("vision=DISABLED servo=DISABLED aiming=DISABLED drop=DISABLED")
            if self.auto_start_detector:
                log.warn("flight_only_mode=true：忽略 auto_start_detector=true；"
                         "不会检查、启动或等待 camera/detector")
            if self.auto_start_servo_test:
                log.warn("flight_only_mode=true：忽略 auto_start_servo_test=true；"
                         "不会检查、启动或等待 servo_test.py")
            if self.enable_b_dropper:
                log.warn("flight_only_mode=true：忽略 enable_b_dropper=true；"
                         "A/B 瞄准与投放均禁用")
            if self.ground_test:
                log.warn("flight_only_mode=true：忽略 ground_test=true，按仅飞行任务流程执行")
            if not self.full_mission_mode:
                log.warn("flight_only_mode=true：忽略 full_mission_mode=false，"
                         "Enter 后执行仅飞行航线")

        nav_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(State, "/mavros/state", self._state_cb, nav_qos)
        self.node.create_subscription(
            ExtendedState, "/mavros/extended_state", self._extended_state_cb, nav_qos)
        self.node.create_subscription(RCIn, "/mavros/rc/in", self._rc_cb, nav_qos)
        self.node.create_subscription(
            StatusText, "/mavros/statustext/recv", self._statustext_cb,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        if self.vision_enabled:
            self.node.create_subscription(
                YoloDetections, "/vision/yolo/detections", self._aim_cb,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.node.create_subscription(
                GeoTarget, "/vision/target_global", self._geo_target_cb,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.node.create_subscription(
                CameraInfo, "/vision/color/camera_info", self._camera_info_cb, 1)
        self.node.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, nav_qos)
        self.node.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local",
            self._velocity_cb, nav_qos)
        self.node.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._fix_cb, nav_qos)
        self.node.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self._heading_cb, nav_qos)
        self.node.create_subscription(
            Float64, "/mavros/global_position/rel_alt", self._relative_altitude_cb, nav_qos)
        if self.servo_interface_enabled:
            self.node.create_subscription(
                Bool, "/servo/ready", self._servo_ready_cb, 1)

        self.setpoint_pub = self.node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.global_setpoint_pub = self.node.create_publisher(
            GlobalPositionTarget, "/mavros/setpoint_raw/global", 20)
        self.servo_cmd_pub = self.node.create_publisher(String, "/servo/cmd", 10)
        self.mission_status_pub = self.node.create_publisher(
            MissionStatus, "/vision/mission_status", 10)

    # ==================================================================
    # 订阅回调（源 L604-703）
    # ==================================================================
    def _state_cb(self, msg):
        with self._lock:
            previous = self.current_state
            self.current_state = msg
            self.state_received_at = monotonic_s()
            rc_summary = self._rc_summary_unlocked()
        if previous is None:
            self.node.get_logger().info(
                "MAVROS 首个状态 | connected=%s armed=%s mode=%s system_status=%s RC=%s"
                % (msg.connected, msg.armed, msg.mode, msg.system_status, rc_summary))
        elif (previous.connected != msg.connected
              or previous.armed != msg.armed
              or previous.mode != msg.mode
              or previous.system_status != msg.system_status):
            self.node.get_logger().warn(
                "MAVROS 状态变化 | connected %s->%s armed %s->%s mode %s->%s "
                "system_status %s->%s RC=%s recent_fcu=%s"
                % (previous.connected, msg.connected, previous.armed, msg.armed,
                   previous.mode, msg.mode, previous.system_status, msg.system_status,
                   rc_summary, self._recent_statustext_summary()))

    def _camera_info_cb(self, msg):
        # ROS2 CameraInfo 字段小写 k，且固定数组为 numpy 数组 —— 只用 len 判断
        # （禁止真值判断，错题 #19）。
        try:
            if len(msg.k) >= 6:
                self.camera_fx = float(msg.k[0])
                self.camera_fy = float(msg.k[4])
                self.camera_center_x = float(msg.k[2])
                self.camera_center_y = float(msg.k[5])
                if self.camera_fx > 0.0 and self.camera_fy > 0.0:
                    with self._lock:
                        self.camera_info_received_at = monotonic_s()
        except Exception:
            pass

    def _aim_cb(self, msg):
        if self.flight_only_mode:
            return
        with self._lock:
            self.current_detections = msg
            self.detections_received_at = monotonic_s()

    def _geo_target_cb(self, msg):
        """T2：缓存本帧多目标 GeoTarget（target_id 1/2/3，count 为本帧桶数）。

        识别节点只输出不平滑；这里只做缓存，位置估计在 bucket_survey 里完成。
        target_id<=0（无效帧）或 count<=0 时清空本帧；否则按 count 清理旧序号。
        """
        if self.flight_only_mode:
            return
        with self._lock:
            self.geo_targets_received_at = monotonic_s()
            target_id = int(getattr(msg, "target_id", 0))
            count = int(getattr(msg, "count", 0))
            if target_id <= 0 or count <= 0:
                self.latest_geo_targets = {}
                self.geo_count = 0
                return
            self.geo_count = count
            for stale_id in [k for k in self.latest_geo_targets if k > count]:
                del self.latest_geo_targets[stale_id]
            self.latest_geo_targets[target_id] = msg

    def _pose_cb(self, msg):
        with self._lock:
            self.current_pose = msg
            self.pose_received_at = monotonic_s()

    def _velocity_cb(self, msg):
        with self._lock:
            self.current_velocity = msg
            self.velocity_received_at = monotonic_s()

    def _fix_cb(self, msg):
        valid = (
            msg.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and math.isfinite(msg.altitude)
            and -90.0 <= msg.latitude <= 90.0
            and -180.0 <= msg.longitude <= 180.0
        )
        if valid:
            with self._lock:
                self.current_fix = msg
                self.fix_received_at = monotonic_s()

    def _heading_cb(self, msg):
        if math.isfinite(msg.data):
            with self._lock:
                self.heading_deg = float(msg.data) % 360.0
                self.heading_received_at = monotonic_s()

    def _relative_altitude_cb(self, msg):
        if math.isfinite(msg.data):
            with self._lock:
                self.relative_altitude = float(msg.data)
                self.relative_altitude_received_at = monotonic_s()

    def _extended_state_cb(self, msg):
        with self._lock:
            self.current_extended_state = msg
            self.extended_state_received_at = monotonic_s()

    def _rc_cb(self, msg):
        with self._lock:
            self.current_rc = msg
            self.rc_received_at = monotonic_s()

    def _statustext_cb(self, msg):
        now = monotonic_s()
        with self._lock:
            self.recent_statustext.append((now, int(msg.severity), str(msg.text)))
        self.node.get_logger().warn(
            "FCU STATUSTEXT | severity=%d text=%s" % (msg.severity, msg.text))

    def _servo_ready_cb(self, msg):
        with self._lock:
            self.servo_ready = bool(msg.data)

    def _transition(self, new_state, reason):
        old_state = self.top_state
        self.previous_top_state = old_state
        self.top_state = str(new_state)
        now = monotonic_s()
        with self._lock:
            state = self.current_state
            pose = self.current_pose
            fix = self.current_fix
            rel_alt = self.relative_altitude
            state_age = now - self.state_received_at if state is not None else float("inf")
        ned_text = "none"
        if pose is not None:
            ned_text = "({:+.2f},{:+.2f},{:+.2f})".format(
                pose.pose.position.y, pose.pose.position.x, -pose.pose.position.z)
        wgs_text = "none"
        if fix is not None:
            wgs_text = "({:.8f},{:.8f},{:.2f})".format(
                fix.latitude, fix.longitude, fix.altitude)
        local_setpoint_text = "none"
        if self.last_setpoint_commanded_enu is not None:
            local_setpoint_text = "({:+.2f},{:+.2f},{:+.2f})".format(
                *self.last_setpoint_commanded_enu)
        global_setpoint_text = "none"
        if self.last_safe_global_target is not None:
            global_setpoint_text = "({:.8f},{:.8f},{:.2f})".format(
                self.last_safe_global_target.latitude,
                self.last_safe_global_target.longitude,
                self.last_safe_global_target.altitude)
        self.node.get_logger().warn(
            "MISSION TRANSITION | %s -> %s | reason=%s | mode=%s armed=%s "
            "connected=%s state_age=%.2fs NED=%s WGS84=%s rel_alt=%s "
            "ammo=A%d/B%d safe=%s:%s local_sp=%s global_sp=%s "
            "pattern=%s lap=%d/%d point=%d/%d"
            % (old_state, self.top_state, reason,
               state.mode if state is not None else "NONE",
               state.armed if state is not None else None,
               state.connected if state is not None else None,
               state_age, ned_text, wgs_text,
               "{:.2f}".format(rel_alt) if rel_alt is not None else "none",
               self.ammo_a, self.ammo_b, self.last_safe_setpoint_kind,
               self.last_setpoint_label, local_setpoint_text, global_setpoint_text,
               self.area_search_pattern, self.flight_only_current_lap,
               self.flight_only_pattern_laps, self.flight_only_current_point,
               self.flight_only_total_points))

    # ==================================================================
    # 有界恢复 / 依赖管理（源 L766-953）
    # ==================================================================
    def _begin_recovery(self, reason):
        if self.top_state != "RECOVERY":
            self.recovery_resume_state = self.top_state
            self._transition("RECOVERY", reason)

    def _end_recovery(self, reason):
        if self.top_state == "RECOVERY" and self.recovery_resume_state:
            resume = self.recovery_resume_state
            self.recovery_resume_state = None
            self._transition(resume, reason)

    @staticmethod
    def _node_name_matches(node_name, expected_basename):
        return node_name.rstrip("/").split("/")[-1] == expected_basename

    def _node_is_running(self, expected_basename):
        try:
            return any(self._node_name_matches(name, expected_basename)
                       for name in self.node.get_node_names())
        except Exception as exc:
            self.node.get_logger().warn("读取 ROS 节点列表失败: %s" % exc)
            return False

    def _spawn(self, command, label, restartable=True):
        self.node.get_logger().warn("自动启动 %s: %s" % (label, " ".join(command)))
        out_fh = None
        kwargs = {}
        if RUN_DIR:
            safe = re.sub(r"[^A-Za-z0-9_.-]", "_", label)
            try:
                out_fh = open(os.path.join(RUN_DIR, "%s.log" % safe), "ab", buffering=0)
                kwargs = {"stdout": out_fh, "stderr": subprocess.STDOUT}
            except OSError:
                out_fh = None
                kwargs = {}
        try:
            process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, preexec_fn=os.setsid, **kwargs)
        except (OSError, subprocess.SubprocessError) as exc:
            self.node.get_logger().error("启动 %s 失败: %s" % (label, exc))
            return False
        finally:
            if out_fh is not None:
                out_fh.close()
        self._children.append((label, process))
        if restartable:
            self._child_specs[label] = list(command)
            self._dependency_restart_counts.setdefault(label, 0)
        return True

    def start_dependencies(self):
        # #72：DDS 发现宽限——MAVROS 刚启动时，节点/发布者可能还没被本节点发现，
        # 不留宽限会把"刚起"误判成"未运行"（尤其真机手动拉起 MAVROS 后立刻开任务）。
        # 2026-10-05：WebUI/终端刚拉起 MAVROS 时 connected 需十几秒；auto_start_mavros=true
        # 若只等 3s 会误判未运行 → 本脚本又拉起第二份 MAVROS 抢 UDP 端口/链路。
        discover_grace = 15.0 if self.auto_start_mavros else 3.0
        discover_deadline = monotonic_s() + discover_grace
        while (rclpy.ok() and self.node.context.ok()
               and monotonic_s() < discover_deadline):
            if self._node_is_running("mavros") or self._mavros_topic_exists():
                break
            time.sleep(0.1)
        if not self._node_is_running("mavros") and not self._mavros_topic_exists():
            if not self.auto_start_mavros:
                self.node.get_logger().error("MAVROS 未运行且 auto_start_mavros=false")
                return False
            if not self._spawn(
                ["ros2", "launch", "mavros", "apm.launch.py",
                 "fcu_url:={}".format(self.mavros_fcu_url)],
                "MAVROS"):
                return False
            deadline = monotonic_s() + self.connection_timeout_s
            while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
                if self._mavros_topic_exists():
                    break
                throttle_log(self.node, 1.0,
                             "等待自动启动的 MAVROS 发布 /mavros/state...")
                time.sleep(0.1)
            else:
                self.node.get_logger().error("自动启动 MAVROS 后 /mavros/state 未出现")
                return False
        else:
            self.node.get_logger().info("MAVROS 已运行，不重复启动")

        if self.detector_enabled:
            # 相机与识别各自独立启动：冷启动（两者都没跑）时必须把两个都拉起来。
            # 旧写法用 elif —— "识别没跑"时命中第一分支、跳过相机，而 Y26 识别 launch 又传了
            # auto_start_camera:=false，导致冷启动没人启动相机 → 就绪判据
            # (camera_node + detector_node + aim_info) 30s 超时失败。
            detector_running = self._node_is_running("detector_node")
            camera_running = self._node_is_running("camera_node")
            if self._trt_direct():
                # TRT direct 模式：节点内直连 RealSense，没有 camera_node，也不允许
                # 再起 camera_node（会抢 USB 设备）。
                self.node.get_logger().info(
                    "TRT direct_camera 模式：不启动/不等待 camera_node")
                if not detector_running:
                    if not self._spawn(self._detector_launch_command(), "识别 launch"):
                        return False
            elif detector_running and camera_running:
                self.node.get_logger().info("detector_node 与 camera_node 已运行，不重复启动")
            else:
                if not detector_running:
                    if not self._spawn(self._detector_launch_command(), "识别 launch"):
                        return False
                if not camera_running:
                    if not self._spawn(self._camera_launch_command(), "camera_node launch"):
                        return False

        if self.servo_enabled:
            if self._node_is_running("servo_controller"):
                self.node.get_logger().info("servo_test.py 已运行，不重复启动")
            elif not self._spawn(self._servo_run_command(), "servo_test.py"):
                return False
        return True

    def _trt_direct(self):
        """TRT 包 + 真机 RealSense + 配置允许 → 节点内直连相机（无 camera_node）。"""
        return (self.detector_package == "cuadc_vision_trt"
                and self.camera_source == "realsense"
                and self.trt_direct_camera)

    def _trt_engine_path(self):
        eng = self.trt_engine if self.trt_engine in ("640", "896") else "640"
        return os.path.join(
            os.path.expanduser("~/ROS2_CUADC2026"), "install", "cuadc_vision_trt",
            "share", "cuadc_vision_trt", "models", "engines",
            "orin_best_%s_fp16.engine" % eng)

    def _detector_launch_command(self):
        """识别启动命令（按包适配已声明的 launch 参数）。

        cuadc_vision 的 detector_node.launch.py 未声明 camera_source/show_window，
        传未声明参数会报错 → 只对 cuadc_visionY26 下传；两个包都声明了
        auto_start_mavros。相机由本脚本单独管理，故关掉 Y26 的自带相机。
        cuadc_vision_trt：direct 模式节点内直连 RealSense（无 camera_node）；
        仿真 gazebo 走 ROS 相机话题（1280x720，匹配 demo_up 相机）。
        """
        cmd = ["ros2", "launch", self.detector_package, self.detector_launch,
               "auto_start_mavros:=false"]
        if self.detector_package == "cuadc_visionY26":
            cmd += [
                "auto_start_camera:=false",
                "camera_source:=%s" % self.camera_source,
                "show_window:=%s" % ("true" if self.detector_show_window else "false"),
            ]
        elif self.detector_package == "cuadc_vision_trt":
            if self.camera_source == "gazebo":
                w, h = 1280, 720
            else:
                w, h = self.trt_width, self.trt_height
            cmd += [
                "auto_start_camera:=false",
                "camera_source:=%s" % self.camera_source,
                "show_window:=%s" % ("true" if self.detector_show_window else "false"),
                "direct_camera:=%s" % ("true" if self._trt_direct() else "false"),
                "direct_depth:=true",
                "w:=%d" % w,
                "h:=%d" % h,
                "depth_w:=%d" % self.trt_depth_width,
                "depth_h:=%d" % self.trt_depth_height,
                "cam_fps:=%d" % self.trt_cam_fps,
                "imgsz:=%d" % self.trt_imgsz,
                "model_path:=%s" % self._trt_engine_path(),
                "publish_annotated:=true",
                # 2026-10-05 标定：抛投器偏移、相机 x 反向与相机安装高度均由本配置
                # 单一事实源下传（识别 _compute_geo/_dual_solution 与画准星同值）。
                "dual_offset_m:=%.3f" % self.crosshair_offset_m,
                "invert_camera_x:=%s" % ("true" if self.camera_invert_x else "false"),
                "camera_mount_z:=%.3f" % self.camera_mount_z,
            ]
        return cmd

    def _camera_launch_command(self):
        """相机启动命令；cuadc_vision 无 camera_node.launch.py → 直跑节点。"""
        if self.detector_package == "cuadc_visionY26":
            return ["ros2", "launch", self.detector_package, "camera_node.launch.py",
                    "source:=%s" % self.camera_source, "show_rgb:=false"]
        if self.detector_package == "cuadc_vision_trt":
            cmd = ["ros2", "run", "cuadc_vision_trt", "camera_node.py",
                   "--ros-args", "-p", "source:=%s" % self.camera_source]
            if self.camera_source == "realsense":
                cmd += ["-p", "color_width:=%d" % self.trt_width,
                        "-p", "color_height:=%d" % self.trt_height,
                        "-p", "depth_width:=%d" % self.trt_depth_width,
                        "-p", "depth_height:=%d" % self.trt_depth_height,
                        "-p", "fps:=%d" % self.trt_cam_fps]
            return cmd
        # #26：直跑节点传参必须 --ros-args -p k:=v
        return ["ros2", "run", "cuadc_vision", "camera_node.py",
                "--ros-args", "-p", "source:=%s" % self.camera_source]

    def _servo_run_command(self):
        """舵机测试启动命令（通道/PWM 来自 YAML；#43 默认 SERVO9/10）。"""
        return [
            "ros2", "run", "cuadc_vision", "servo_test.py", "--ros-args",
            "-p", "servo_a_channel:=%d" % self.servo_a_channel,
            "-p", "servo_b_channel:=%d" % self.servo_b_channel,
            "-p", "pwm_open:=%d" % self.pwm_open,
            "-p", "pwm_close:=%d" % self.pwm_close,
            "-p", "enable_ch5:=%s" % ("true" if self.enable_ch5 else "false"),
            "-p", "enable_ch6:=%s" % ("true" if self.enable_ch6 else "false"),
        ]

    def _mavros_topic_exists(self):
        # #72：必须看"发布者数"，不能用 get_topic_names_and_types()——本节点自己也订阅
        # /mavros/state，topic 列表里永远有它，会把"MAVROS 根本没在跑"误判为"在跑但没连上"。
        try:
            return self.node.count_publishers("/mavros/state") > 0
        except Exception:
            return False

    def _wait_for_dependencies_ready(self):
        if self.flight_only_mode:
            self.node.get_logger().warn(
                "FLIGHT ONLY：仅等待 MAVROS；视觉、相机和舵机就绪检查已跳过")
            return True
        if self.detector_enabled:
            deadline = monotonic_s() + self.detector_ready_timeout_s
            while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
                # 2026-10-05 实据：本脚本拉起的识别进程会被 SIGKILL(-9) 后无人重启，
                # 任务干等到超时。依赖等待期间也按配置重启死亡子进程（越界给日志）。
                self._restart_dead_children()
                with self._lock:
                    detection_age = monotonic_s() - self.detections_received_at
                    camera_info_age = monotonic_s() - self.camera_info_received_at
                    detections_seen = self.current_detections is not None
                    camera_info_ready = (self.camera_fx > 0.0 and self.camera_fy > 0.0
                                         and self.camera_info_received_at > 0.0)
                    fx, fy = self.camera_fx, self.camera_fy
                # TRT direct 模式没有 camera_node，但 detector 会发布
                # /vision/color/camera_info（RealSense 内参）→ 视为相机就绪。
                camera_node_running = (self._node_is_running("camera_node")
                                       or self._trt_direct())
                detector_node_running = self._node_is_running("detector_node")
                # 起飞前只确认相机/识别节点在发布新鲜帧和有效内参。
                # 不能要求 latest_aim 有目标：投放区在航线末端，飞机尚未飞到那里时
                # 正常就是空检测帧；把“依赖就绪”和“发现桶”混在一起会在起飞前超时返航。
                if (camera_node_running and detector_node_running
                        and detections_seen and detection_age <= self.aim_info_timeout_s
                        and camera_info_ready and camera_info_age <= self.aim_info_timeout_s):
                    self.node.get_logger().info(
                        "camera_node、detector_node、视觉空/目标帧与相机内参流已就绪")
                    break
                # 逐项状态（2026-10-01 实据：两次 30s 超时只留一句“等待…”，无法区分
                # 是节点没起、没有检测帧还是内参没来；排障必须能一眼看出缺哪项）。
                throttle_log(
                    self.node, 2.0,
                    "等待视觉依赖（不要求当前已有桶）：camera_node=%s detector_node=%s "
                    "检测帧=%s(age=%.1fs) 内参=%s(fx=%.1f fy=%.1f age=%.1fs)",
                    camera_node_running, detector_node_running, detections_seen,
                    detection_age if self.detections_received_at > 0.0 else float("inf"),
                    camera_info_ready, fx, fy,
                    camera_info_age if self.camera_info_received_at > 0.0 else float("inf"))
                if detector_node_running and not detections_seen:
                    # 183103 实据：TRT 冷启动 load engine ~24s + 相机启动 ~1s，
                    # 30s 超时会在首帧前误杀；提示操作手这是预热而非故障。
                    throttle_log(
                        self.node, 5.0,
                        "detector_node 已运行但尚无检测帧：TRT 引擎/相机预热中"
                        "（冷启动约 20~45s，超时 %.0fs）",
                        self.detector_ready_timeout_s)
                time.sleep(0.1)
            else:
                with self._lock:
                    detection_age = monotonic_s() - self.detections_received_at
                    camera_info_age = monotonic_s() - self.camera_info_received_at
                    detections_seen = self.current_detections is not None
                    camera_info_ready = (self.camera_fx > 0.0 and self.camera_fy > 0.0
                                         and self.camera_info_received_at > 0.0)
                    fx, fy = self.camera_fx, self.camera_fy
                self.node.get_logger().error(
                    "视觉依赖未在 %.1fs 内就绪 | camera_node=%s detector_node=%s "
                    "检测帧=%s(age=%.1fs) 内参=%s(fx=%.1f fy=%.1f age=%.1fs) | "
                    "若仿真刚重启，请先跑 scripts/preflight.sh 等 READY 再启动任务"
                    % (self.detector_ready_timeout_s,
                       self._node_is_running("camera_node"),
                       self._node_is_running("detector_node"),
                       detections_seen,
                       detection_age if self.detections_received_at > 0.0 else float("inf"),
                       camera_info_ready, fx, fy,
                       camera_info_age if self.camera_info_received_at > 0.0 else float("inf")))
                return False
        if self.servo_enabled and not self._wait_for_servo_subscriber(
                timeout_s=self.servo_ready_timeout_s):
            return False
        return True

    def _restart_dead_children(self):
        for index, (label, process) in enumerate(list(self._children)):
            if process.poll() is None or label not in self._child_specs:
                continue
            attempts = self._dependency_restart_counts.get(label, 0)
            if attempts >= self.dependency_restart_attempts:
                self.node.get_logger().error(
                    "%s 已退出且重启次数耗尽 %d/%d"
                    % (label, attempts, self.dependency_restart_attempts),
                    throttle_duration_sec=1.0)
                return False
            if self.airborne:
                hold = self._safe_hold_enu()
                if hold is None:
                    self.node.get_logger().error("依赖退出且没有最后安全本地保持 setpoint")
                    return False
                self._publish_hold_setpoint(hold, "RECOVERY 依赖重启保持")
            self.node.get_logger().warn(
                "%s 意外退出(code=%s)，%.1fs 后执行第 %d/%d 次重启"
                % (label, process.poll(), self.dependency_restart_backoff_s,
                   attempts + 1, self.dependency_restart_attempts))
            if not self._interruptible_delay(self.dependency_restart_backoff_s):
                return False
            command = self._child_specs[label]
            try:
                replacement = subprocess.Popen(
                    command, stdin=subprocess.DEVNULL, preexec_fn=os.setsid)
            except (OSError, subprocess.SubprocessError) as exc:
                self.node.get_logger().error("重启 %s 失败: %s" % (label, exc))
                self._dependency_restart_counts[label] = attempts + 1
                return True
            self._children[index] = (label, replacement)
            self._dependency_restart_counts[label] = attempts + 1
        return True

    # ==================================================================
    # 数据新鲜度 / 安全保持（源 L955-1062）
    # ==================================================================
    def _fresh_state(self):
        with self._lock:
            msg = self.current_state
            age = monotonic_s() - self.state_received_at
        if msg is None or age > self.state_timeout_s:
            return None
        return msg

    @staticmethod
    def _finite_number(value):
        try:
            return math.isfinite(float(value))
        except Exception:
            return False

    @staticmethod
    def _quaternion_rotate(q, vector):
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        vx, vy, vz = vector
        tx = 2.0 * (y * vz - z * vy)
        ty = 2.0 * (z * vx - x * vz)
        tz = 2.0 * (x * vy - y * vx)
        rx = vx + w * tx + (y * tz - z * ty)
        ry = vy + w * ty + (z * tx - x * tz)
        rz = vz + w * tz + (x * ty - y * tx)
        return rx, ry, rz

    def _aim_ned_for_det(self, det, pose):
        """相机系→机体系(FRD, z 向下)→ENU→NED；返回 (n, e, d) 或 None。

        body_z(down) = camera_z_m + camera_mount_z（与 ROS1 detector_node
        _camera_position_to_body 及本工作区 detector._compute_geo 同源）；漏加会让
        锁定 NED 的地面高 0.4m，投放高度基准随之偏高。
        """
        try:
            body = (-float(det.camera_y_m), float(det.camera_x_m),
                    -(float(det.camera_z_m) + self.camera_mount_z))
        except Exception:
            return None
        if not all(math.isfinite(v) for v in body):
            return None
        offset = self._quaternion_rotate(pose.pose.orientation, body)
        p = pose.pose.position
        enu = (float(p.x) + offset[0], float(p.y) + offset[1], float(p.z) + offset[2])
        return (enu[1], enu[0], -enu[2])

    def _fill_aim_slot(self, view, prefix, det, pose, ned=None):
        """把一条检测折算进 AimView 的 A/B 槽位（FRD→ENU，z 取反）。

        机体系是 FRD（x 前 / y 右 / z **向下**），pose.orientation 是 ENU(map) 姿态；
        FRD→ENU 必须把 z 取反。旧代码直接旋转 z，导致目标的"米制 NED"被算到飞机
        **上方约 depth 高度**（仿真实测 +6.6m）→ 粗对准悬停目标+2m 越爬越高、
        深度超限后 FAILSAFE（T2 步骤 5 首跑实据 2026-09-30 06:03）。
        """
        if ned is None:
            ned = self._aim_ned_for_det(det, pose)
        if ned is None:
            return
        n, e, d = ned
        setattr(view, prefix + "_ned_valid", True)
        setattr(view, prefix + "_ned_n", n)
        setattr(view, prefix + "_ned_e", e)
        setattr(view, prefix + "_ned_d", d)
        depth = float(getattr(det, "depth_m", 0.0))
        if not (depth > 0.0):
            depth = math.hypot(float(det.camera_x_m), float(det.camera_y_m),
                               float(det.camera_z_m))
        dx = dy = 0.0
        if self.camera_fx > 0.0 and self.camera_fy > 0.0:
            dx = (float(det.center_x) - self.camera_center_x) * depth / self.camera_fx
            dy = (float(det.center_y) - self.camera_center_y) * depth / self.camera_fy
        # 差值基准 = 该投放器实际中心（前后 ±crosshair_offset_m + 横向
        # crosshair_lateral_y_m，2026-10-05 垂线标定）。桶相对该中心的原始像素差值：
        #   dy = dy_raw + offset_x（前投 offset_x=+off、后投 -off）
        #   dx = dx_raw - sign*offset_y（相机倒装 sign=-1 ⇒ dx_raw = sign*机体右侧）
        offset_x = (self.crosshair_offset_m
                    if str(prefix).upper() == self.front_dropper
                    else -self.crosshair_offset_m)
        sign = -1.0 if self.camera_invert_x else 1.0
        dx -= sign * self.crosshair_lateral_y_m
        dy += offset_x
        setattr(view, prefix + "_delta_x_m", dx)
        setattr(view, prefix + "_delta_y_m", dy)

    def _build_aim_view(self, msg):
        """把 YoloDetections 折算成源同形 AimView（ENU->NED）。

        T2 修正：
        - 源 AimView 的 a_*/b_* 是**两个不同目标**（A=最高置信度；B=次高、且与 A
          像素位置可区分者）。旧实现把 a_*/b_* 都填同一个 best → B 投放永远看到
          A 已投的桶（黑名单拒绝后反复"丢失-重捕获"到超时 FAILSAFE）。
        - 选桶投放：T2 在 `preferred_target_ned` 里给出"本槽应投的桶"，本函数优先
          锁定离它最近的检测（两槽同填该目标）；偏好目标不在视野（>门限）时返回
          valid=false，让瞄准走"丢失-重捕获"（宁可等/失败，也不投未选中的桶）。
        - 单目标帧 B 槽保持 invalid（B 无法投放，语义正确）；像素间距 >15px 才认为
          B 是另一目标（去同桶重复框）。
        """
        view = AimView()
        raw = list(getattr(msg, "detections", []))
        view.count = len([d for d in raw if bool(getattr(d, "detected", False))])
        dets = []
        for det in raw:
            if not bool(getattr(det, "detected", False)):
                continue
            if float(getattr(det, "confidence", 0.0)) < self.vision_min_confidence:
                continue
            if not bool(getattr(det, "position_valid", False)):
                continue
            if not (float(getattr(det, "depth_m", 0.0)) > 0.0):
                continue
            cam = (float(det.camera_x_m), float(det.camera_y_m), float(det.camera_z_m))
            if not all(self._finite_number(v) for v in cam):
                continue
            dets.append(det)
        if not dets:
            view.valid = False
            return view
        with self._lock:
            pose = self.current_pose
        if pose is None:
            view.valid = False
            return view
        dets.sort(key=lambda d: (-float(d.confidence), int(d.center_x)))
        entries = []
        for det in dets:
            ned = self._aim_ned_for_det(det, pose)
            if ned is not None:
                entries.append((det, ned))
        if not entries:
            view.valid = False
            return view
        preferred = self.preferred_target_ned
        if preferred is not None:
            best_entry = None
            best_dist = float("inf")
            active_dropper = getattr(self, "_active_dropper", "")
            for det, ned in entries:
                # 2026-10-07：B 选桶时跳过已完成（黑名单）目标，否则 A 桶会一直
                # 占据"最近检测"位置，B 永远等不到自己的桶。
                if self._target_is_blacklisted(ned, active_dropper):
                    continue
                dist = math.hypot(ned[0] - float(preferred[0]),
                                  ned[1] - float(preferred[1]))
                if dist < best_dist:
                    best_dist = dist
                    best_entry = (det, ned)
            within_gate = (best_entry is not None
                           and best_dist <= self.preferred_target_gate_m)
            if not within_gate and best_entry is not None \
                    and self.frozen_target_ned is None:
                # 2026-10-04 181706 实飞：survey 估计误差 0.67~0.79m 全被 0.6m 门限
                # 拒收 → B 桶 30s 超时。若最近检测持续出现在重锚定距离内，说明估计
                # 偏差而非"选错桶"，把偏好重锚定到观测（仍不误投别的桶：超距直接拒绝）。
                if best_dist <= self.preferred_target_reanchor_m:
                    if self._preferred_reanchor_since is None:
                        self._preferred_reanchor_since = monotonic_s()
                    elif (monotonic_s() - self._preferred_reanchor_since
                          >= self.preferred_target_reanchor_s):
                        self.node.get_logger().warn(
                            "偏好桶估计重锚定 | 最近观测距估计 %.2fm（门限 %.2fm），"
                            "持续 %.1fs，按观测重设偏好 NED"
                            % (best_dist, self.preferred_target_gate_m,
                               self.preferred_target_reanchor_s))
                        self.preferred_target_ned = best_entry[1]
                        self._preferred_reanchor_since = None
                        within_gate = True
                else:
                    self._preferred_reanchor_since = None
            if within_gate:
                self._preferred_reanchor_since = None
                self._fill_aim_slot(view, "a", best_entry[0], pose, ned=best_entry[1])
                self._fill_aim_slot(view, "b", best_entry[0], pose, ned=best_entry[1])
                view.valid = True
                view.confidence = float(best_entry[0].confidence)
                view.source_class = str(getattr(best_entry[0], "class_name", ""))
                return view
            self.node.get_logger().warning(
                "偏好桶不在视野（最近检测距偏好 %.2fm > 门限 %.2fm），等待重见"
                % (best_dist, self.preferred_target_gate_m), throttle_duration_sec=2.0)
            view.valid = False
            return view
        best = entries[0]
        second = None
        for cand in entries[1:]:
            if math.hypot(int(cand[0].center_x) - int(best[0].center_x),
                          int(cand[0].center_y) - int(best[0].center_y)) > 15.0:
                second = cand
                break
        self._fill_aim_slot(view, "a", best[0], pose, ned=best[1])
        if second is not None:
            self._fill_aim_slot(view, "b", second[0], pose, ned=second[1])
        view.valid = True
        view.confidence = float(best[0].confidence)
        view.source_class = str(getattr(best[0], "class_name", ""))
        return view

    def _fresh_aim(self, require_count=True):
        if self.flight_only_mode:
            return None
        with self._lock:
            msg = self.current_detections
            age = monotonic_s() - self.detections_received_at
        if msg is None or age > self.aim_info_timeout_s:
            return None
        view = self._build_aim_view(msg)
        if view is None:
            return None
        with self._lock:
            self.latest_aim = view
            self.aim_received_at = monotonic_s()
        if require_count and (view.count < self.min_bucket_count or not view.valid):
            return None
        return view

    def _fresh_pose(self):
        with self._lock:
            msg = self.current_pose
            age = monotonic_s() - self.pose_received_at
        if msg is None or age > self.pose_timeout_s:
            return None
        return msg

    def _fresh_velocity(self):
        with self._lock:
            msg = self.current_velocity
            age = monotonic_s() - self.velocity_received_at
        if msg is None or age > self.pose_timeout_s:
            return None
        return msg

    def _control_stream_snapshot(self):
        now = monotonic_s()
        with self._lock:
            state = self.current_state
            pose = self.current_pose
            velocity = self.current_velocity
            commanded = self.last_setpoint_commanded_enu
            return {
                "state": state,
                "state_age": (now - self.state_received_at
                              if state is not None else float("inf")),
                "pose": pose,
                "pose_age": (now - self.pose_received_at
                             if pose is not None else float("inf")),
                "velocity": velocity,
                "velocity_age": (now - self.velocity_received_at
                                 if velocity is not None else float("inf")),
                "commanded_enu": commanded,
            }

    @staticmethod
    def _pose_position_enu(pose):
        if pose is None:
            return None
        position = pose.pose.position
        values = (float(position.x), float(position.y), float(position.z))
        return values if all(math.isfinite(value) for value in values) else None

    def _safe_hold_enu(self, snapshot=None, fallback=None):
        snapshot = snapshot or self._control_stream_snapshot()
        if snapshot["pose_age"] <= self.pose_timeout_s:
            pose_enu = self._pose_position_enu(snapshot["pose"])
            if pose_enu is not None:
                return pose_enu
        commanded = snapshot["commanded_enu"]
        if commanded is not None and all(math.isfinite(value) for value in commanded):
            return tuple(float(value) for value in commanded)
        if fallback is not None and all(math.isfinite(value) for value in fallback):
            return tuple(float(value) for value in fallback)
        return None

    def _rc_summary_unlocked(self):
        if self.current_rc is None:
            return "none"
        channels = list(self.current_rc.channels)
        shown = channels[:12]
        suffix = "..." if len(channels) > len(shown) else ""
        return "rssi={} ch={}{}".format(self.current_rc.rssi, shown, suffix)

    def _rc_summary(self):
        with self._lock:
            return self._rc_summary_unlocked()

    def _recent_statustext_summary(self):
        now = monotonic_s()
        with self._lock:
            recent = list(self.recent_statustext)
        if not recent:
            return "none"
        return " | ".join(
            "{:.1f}s ago sev={} {}".format(max(0.0, now - stamp), severity, text)
            for stamp, severity, text in recent[-4:])

    def _recent_statustext_contains(self, needle):
        needle = str(needle).lower()
        with self._lock:
            recent = [text for _, _, text in self.recent_statustext]
        return any(needle in text.lower() for text in recent)

    # ==================================================================
    # 诊断 / 瞄准辅助（源 L1064-1286）
    # ==================================================================
    @staticmethod
    def _landed_state_name(value):
        names = {
            ExtendedState.LANDED_STATE_UNDEFINED: "UNDEFINED",
            ExtendedState.LANDED_STATE_ON_GROUND: "ON_GROUND",
            ExtendedState.LANDED_STATE_IN_AIR: "IN_AIR",
            ExtendedState.LANDED_STATE_TAKEOFF: "TAKEOFF",
            ExtendedState.LANDED_STATE_LANDING: "LANDING",
        }
        return names.get(value, "UNKNOWN({})".format(value))

    def _control_diagnostic_text(self, dropper, phase, locked_ned=None):
        now = monotonic_s()
        with self._lock:
            state = self.current_state
            state_age = now - self.state_received_at if state is not None else float("inf")
            pose = self.current_pose
            pose_age = now - self.pose_received_at if pose is not None else float("inf")
            velocity = self.current_velocity
            velocity_age = (now - self.velocity_received_at
                            if velocity is not None else float("inf"))
            aim = self.latest_aim
            aim_age = now - self.aim_received_at if aim is not None else float("inf")
            extended = self.current_extended_state
            extended_age = (now - self.extended_state_received_at
                            if extended is not None else float("inf"))
            rc = self._rc_summary_unlocked()
            rc_age = now - self.rc_received_at if self.current_rc is not None else float("inf")
            desired = self.last_setpoint_desired_enu
            commanded = self.last_setpoint_commanded_enu
            setpoint_label = self.last_setpoint_label
            setpoint_age = (now - self.last_setpoint_at
                            if self.last_setpoint_at > 0.0 else float("inf"))

        state_text = "none age=inf"
        if state is not None:
            state_text = ("mode={} connected={} armed={} system_status={} age={:.3f}s"
                          ).format(state.mode, state.connected, state.armed,
                                   state.system_status, state_age)
        pose_text = "none age=inf"
        if pose is not None:
            pos = pose.pose.position
            pose_text = "ENU=({:+.3f},{:+.3f},{:+.3f}) age={:.3f}s".format(
                pos.x, pos.y, pos.z, pose_age)
        velocity_text = "none age=inf"
        if velocity is not None:
            linear = velocity.twist.linear
            velocity_text = "ENU=({:+.3f},{:+.3f},{:+.3f}) age={:.3f}s".format(
                linear.x, linear.y, linear.z, velocity_age)
        aim_text = "none age=inf"
        if aim is not None:
            aim_text = ("count={} valid={} A_ned={} B_ned={} age={:.3f}s"
                        ).format(aim.count, aim.valid, aim.a_ned_valid,
                                 aim.b_ned_valid, aim_age)
        landed_text = "none age=inf"
        if extended is not None:
            landed_text = "{} age={:.3f}s".format(
                self._landed_state_name(extended.landed_state), extended_age)
        setpoint_text = "none"
        if desired is not None and commanded is not None:
            setpoint_text = ("label={} desired=({:+.3f},{:+.3f},{:+.3f}) "
                             "commanded=({:+.3f},{:+.3f},{:+.3f}) age={:.3f}s"
                             ).format(setpoint_label, desired[0], desired[1], desired[2],
                                      commanded[0], commanded[1], commanded[2], setpoint_age)
        lock_text = ("none" if locked_ned is None
                     else "({:+.3f},{:+.3f},{:+.3f})".format(*locked_ned))
        return ("dropper={} phase={} | STATE[{}] | POSE[{}] | VEL[{}] | "
                "AIM[{}] | EXT[landed={}] | RC[{} age={:.3f}s] | LOCK_NED={} | "
                "SETPOINT[{}] | FCU_RECENT[{}]"
                ).format(dropper, phase, state_text, pose_text, velocity_text,
                         aim_text, landed_text, rc, rc_age, lock_text,
                         setpoint_text, self._recent_statustext_summary())

    def _log_control_diagnostics(self, dropper, phase, locked_ned=None, error=None):
        text = self._control_diagnostic_text(dropper, phase, locked_ned)
        if error:
            self.node.get_logger().error("CONTROL DIAGNOSTIC | reason=%s | %s" % (error, text))
        else:
            throttle_log(self.node, self.diagnostic_log_interval_s, "CONTROL TRACE | %s", text)

    def _aim_invalid_reason(self, dropper=None):
        with self._lock:
            msg = self.current_detections
            age = monotonic_s() - self.detections_received_at
        if msg is None:
            return "尚未收到 /vision/yolo/detections"
        if age > self.aim_info_timeout_s:
            return "视觉消息停更 {:.2f}s".format(age)
        view = self._build_aim_view(msg)
        if view.count < self.min_bucket_count:
            return "未检测到足够桶（count={}，需要≥{}）".format(
                view.count, self.min_bucket_count)
        if not view.valid:
            # 逐项拆开，下一次实飞一眼能区分：无框 / 置信度低于门限 / 深度或位置无效。
            # 20261005_202604 实据：低空夜间 conf 掉到 0.17~0.29，被控制侧拒收，
            # 旧文案只写"深度/位置无效"误导排查；这里把 conf 一并打出来。
            raw = [d for d in getattr(msg, "detections", [])
                   if bool(getattr(d, "detected", False))]
            n_pos = sum(1 for d in raw if bool(getattr(d, "position_valid", False)))
            n_depth = sum(1 for d in raw if float(getattr(d, "depth_m", 0.0)) > 0.0)
            n_conf = sum(1 for d in raw
                         if float(getattr(d, "confidence", 0.0)) >= self.vision_min_confidence)
            max_conf = max((float(getattr(d, "confidence", 0.0)) for d in raw), default=0.0)
            return ("检测到桶但被控制侧过滤（detected=%d conf≥%.2f=%d 最高conf=%.2f "
                    "position_valid=%d depth>0=%d）"
                    % (len(raw), self.vision_min_confidence, n_conf, max_conf,
                       n_pos, n_depth))
        if dropper is not None and not getattr(view, "{}_ned_valid".format(dropper.lower())):
            return "{} 视觉有效但绝对 NED 无效".format(dropper)
        return "视觉目标有效"

    def _detection_snapshot(self):
        """当前 YoloDetections 的逐框摘要（最多 3 框），供 aim_trace 对照用。"""
        with self._lock:
            msg = self.current_detections
        snapshot = []
        if msg is None:
            return snapshot
        for det in list(getattr(msg, "detections", []))[:3]:
            snapshot.append({
                "det": bool(getattr(det, "detected", False)),
                "conf": round(float(getattr(det, "confidence", 0.0)), 3),
                "cx": int(getattr(det, "center_x", -1)),
                "cy": int(getattr(det, "center_y", -1)),
                "depth_m": round(float(getattr(det, "depth_m", 0.0)), 3),
                "pos": bool(getattr(det, "position_valid", False)),
                "cam": [round(float(getattr(det, "camera_x_m", 0.0)), 3),
                        round(float(getattr(det, "camera_y_m", 0.0)), 3),
                        round(float(getattr(det, "camera_z_m", 0.0)), 3)],
                "bbox": [int(getattr(det, "x_min", -1)), int(getattr(det, "y_min", -1)),
                         int(getattr(det, "x_max", -1)), int(getattr(det, "y_max", -1))],
            })
        return snapshot

    def _aim_trace(self, payload):
        """追加一条瞄准对照记录到 logs/runs/<ts>/aim_trace.jsonl（失败静默）。"""
        if self._aim_trace_fh is None:
            if not RUN_DIR:
                self._aim_trace_fh = False
            else:
                try:
                    self._aim_trace_fh = open(
                        os.path.join(RUN_DIR, "aim_trace.jsonl"), "a", encoding="utf-8")
                except OSError:
                    self._aim_trace_fh = False
        if not self._aim_trace_fh:
            return
        try:
            self._aim_trace_fh.write(json.dumps(payload, ensure_ascii=False) + "\n")
            self._aim_trace_fh.flush()
        except Exception:
            pass

    @staticmethod
    def _aim_target_ned(aim, dropper):
        prefix = dropper.lower()
        if aim is None or not getattr(aim, "{}_ned_valid".format(prefix)):
            return None
        values = (float(getattr(aim, "{}_ned_n".format(prefix))),
                  float(getattr(aim, "{}_ned_e".format(prefix))),
                  float(getattr(aim, "{}_ned_d".format(prefix))))
        if not all(math.isfinite(value) for value in values):
            return None
        return values

    @staticmethod
    def _ned_xy_distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _target_is_blacklisted(self, target_ned, dropper):
        if dropper != "B" or not self.require_distinct_b_target:
            return False
        return any(self._ned_xy_distance(target_ned, old_target) < self.distinct_target_distance_m
                   for old_target in self.completed_drop_targets)

    def _update_target_lock(self, locked_ned, measured_ned, dropper):
        if self._target_is_blacklisted(measured_ned, dropper):
            self.node.get_logger().warning(
                "%s 当前目标与已投放目标距离小于 %.2fm，继续搜索新桶"
                % (dropper, self.distinct_target_distance_m), throttle_duration_sec=1.0)
            return locked_ned, False
        if locked_ned is None:
            return measured_ned, True
        jump = self._ned_xy_distance(locked_ned, measured_ned)
        if jump > self.max_target_jump_m:
            self.node.get_logger().warning(
                "%s 目标 NED 单帧跳变 %.2fm，超过 %.2fm，忽略该观测"
                % (dropper, jump, self.max_target_jump_m), throttle_duration_sec=1.0)
            return locked_ned, False
        # 2026-10-05 用户口径：瞄准时跟踪新的中心点，但中心面内小抖动忽略。
        # 面内变化 < 死区：XY 保持不动（防抖），深度仍慢跟（高度判据需要）；
        # 变化 ≥ 死区：按 target_filter_alpha 跟新到新的中心。
        alpha = self.target_filter_alpha
        if jump < self.target_track_deadband_m:
            filtered = (locked_ned[0], locked_ned[1],
                        locked_ned[2] + alpha * (measured_ned[2] - locked_ned[2]))
            return filtered, True
        filtered = tuple(old + alpha * (new - old)
                         for old, new in zip(locked_ned, measured_ned))
        return filtered, True

    def _slow_guidance_ned(self, now, locked_ned):
        """冻结后的慢速制导跟踪（破"逐帧追噪声"极限环，20261005_131212 实据）。

        - 每 `aim_guidance_hold_s` 才评估一次；
        - 与当前制导点差 < `aim_guidance_deadband_m` 不动（忽略小抖动）；
        - 单次最多挪 `aim_guidance_step_m`，给飞机留出停稳时间。
        """
        if self._aim_guidance_ned is None:
            self._aim_guidance_ned = tuple(float(v) for v in locked_ned)
            self._next_guidance_at = now + self.aim_guidance_hold_s
            return self._aim_guidance_ned
        if now < self._next_guidance_at:
            return self._aim_guidance_ned
        jump = self._ned_xy_distance(self._aim_guidance_ned, locked_ned)
        if jump > self.aim_guidance_deadband_m + 1e-9:
            step = min(jump, self.aim_guidance_step_m) / jump
            self._aim_guidance_ned = tuple(
                old + step * (new - old)
                for old, new in zip(self._aim_guidance_ned, locked_ned))
        self._next_guidance_at = now + self.aim_guidance_hold_s
        return self._aim_guidance_ned

    def _aim_wait_message(self):
        with self._lock:
            msg = self.current_detections
            age = monotonic_s() - self.detections_received_at
        if msg is None:
            return "尚未收到视觉瞄准消息，请检查 detector_node"
        if age > self.aim_info_timeout_s:
            return "视觉瞄准消息已超时，请检查相机和识别节点是否仍在运行"
        view = self._build_aim_view(msg)
        if view.count < self.min_bucket_count:
            return "当前未识别到桶（检测到 {} 个，至少需要 {} 个）".format(
                view.count, self.min_bucket_count)
        if not view.valid:
            return ("已识别到桶，但深度或相机内参无效；请调整距离、角度，"
                    "并检查桶中心是否有有效深度")
        return "视觉目标有效"

    # ==================================================================
    # 导航快照 / 航点解算（源 L1288-1393）
    # ==================================================================
    def _navigation_snapshot(self):
        now = monotonic_s()
        with self._lock:
            return {
                "pose": self.current_pose,
                "pose_age": now - self.pose_received_at if self.current_pose else float("inf"),
                "fix": self.current_fix,
                "fix_age": now - self.fix_received_at if self.current_fix else float("inf"),
                "heading": self.heading_deg,
                "heading_age": (now - self.heading_received_at
                                if self.heading_deg is not None else float("inf")),
                "rel_alt": self.relative_altitude,
                "rel_alt_age": (now - self.relative_altitude_received_at
                                if self.relative_altitude is not None else float("inf")),
            }

    def _navigation_is_fresh(self):
        nav = self._navigation_snapshot()
        return (nav["pose"] is not None and nav["fix"] is not None
                and nav["heading"] is not None and nav["rel_alt"] is not None
                and nav["pose_age"] <= self.navigation_timeout_s
                and nav["fix_age"] <= self.navigation_timeout_s
                and nav["heading_age"] <= self.navigation_timeout_s
                and nav["rel_alt_age"] <= self.navigation_timeout_s)

    def wait_for_navigation(self):
        deadline = monotonic_s() + self.connection_timeout_s
        while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
            if self._navigation_is_fresh() and self._fresh_velocity() is not None:
                self.node.get_logger().info("GPS、本地位姿、速度、航向和相对高度均已就绪")
                return True
            nav = self._navigation_snapshot()
            throttle_log(self.node, 1.0,
                         "等待导航数据 | pose=%.2fs fix=%.2fs heading=%.2fs rel_alt=%.2fs vel=%.2fs",
                         nav["pose_age"], nav["fix_age"], nav["heading_age"],
                         nav["rel_alt_age"], self._control_stream_snapshot()["velocity_age"])
            time.sleep(0.1)
        self.node.get_logger().error("等待完整导航数据超时")
        return False

    def plan_waypoints(self):
        if not self._navigation_is_fresh():
            self.node.get_logger().error("导航数据不新鲜，不能冻结起点和航点")
            return False
        with self._lock:
            pose = self.current_pose.pose.position
            fix = self.current_fix
            self.start_ned = (float(pose.y), float(pose.x), float(-pose.z))
            self.start_wgs84 = (float(fix.latitude), float(fix.longitude), float(fix.altitude))
            self.start_heading_deg = float(self.heading_deg)
            self.start_relative_altitude = float(self.relative_altitude)
        # #68：锁定航向。真机默认 -1 = 用起始航向（操作手把机头对准投放区中线）；
        # 0..360 = 固定航向（仿真里起飞前 EKF 偏航尚未对齐，start_heading 会漂 2~9°，
        # 用固定 90° 对齐正东中线，避免投放区被锁偏导致 W 搜索漏桶）。
        bearing = (self.start_heading_deg if self.drop_zone_bearing_deg < 0.0
                   else self.drop_zone_bearing_deg % 360.0)
        self.lock_heading_deg = bearing
        if self.drop_zone_bearing_deg >= 0.0:
            self.node.get_logger().warn(
                "锁定航向=固定 %.1fdeg（drop_zone_bearing_deg）；起始航向 %.1fdeg 仅作显示参考"
                % (bearing, self.start_heading_deg))
            diff = abs(((bearing - self.start_heading_deg + 180.0) % 360.0) - 180.0)
            if diff > 30.0:
                self.node.get_logger().warn(
                    "⚠️ 固定锁定航向 %.1f° 与机头 %.1f° 相差 %.1f°：飞机将朝固定方向飞而不是机头方向。"
                    "真机请把 drop_zone_bearing_deg 改回 -1（用机头航向）！"
                    % (bearing, self.start_heading_deg, diff))
        drop_lat, drop_lon = destination_wgs84(
            self.start_wgs84[0], self.start_wgs84[1],
            self.drop_zone_distance_m, bearing)
        recon_lat, recon_lon = destination_wgs84(
            self.start_wgs84[0], self.start_wgs84[1],
            self.recon_zone_distance_m, bearing)
        self.drop_zone_wgs84 = (drop_lat, drop_lon)
        self.recon_zone_wgs84 = (recon_lat, recon_lon)
        return True

    def _lock_heading(self):
        """投放区/侦察区锁定航向（#68）：固定值优先，否则用起始航向。"""
        return (self.lock_heading_deg if self.lock_heading_deg is not None
                else self.start_heading_deg)

    # ==================================================================
    # 任务摘要 / 授权 / 服务 / 起飞（源 L1395-1685）
    # ==================================================================
    def print_mission_summary(self):
        if self.flight_only_mode:
            drop_distance = horizontal_wgs84_distance(
                self.start_wgs84[0], self.start_wgs84[1],
                self.drop_zone_wgs84[0], self.drop_zone_wgs84[1])
            recon_distance = horizontal_wgs84_distance(
                self.start_wgs84[0], self.start_wgs84[1],
                self.recon_zone_wgs84[0], self.recon_zone_wgs84[1])
            route = ("GUIDED -> ARM -> TAKEOFF -> 投放区 -> {} 字完整航线 x{} -> "
                     "侦察区爬升/保持 -> RTL").format(
                         self.area_search_pattern, self.flight_only_pattern_laps)
            if not self.flight_only_continue_to_recon:
                route = ("GUIDED -> ARM -> TAKEOFF -> 投放区 -> {} 字完整航线 x{} -> RTL"
                         ).format(self.area_search_pattern, self.flight_only_pattern_laps)
            lines = [
                "",
                "================ FLIGHT ONLY 任务摘要 ================",
                "MODE          = FLIGHT_ONLY",
                "VISION        = DISABLED",
                "SERVO         = DISABLED",
                "AIM/DROP      = DISABLED",
                "起飞点 WGS84  : lat={:.9f}, lon={:.9f}, alt={:.3f}m".format(*self.start_wgs84),
                "起始 NED       : N={:.3f}, E={:.3f}, D={:.3f}m".format(*self.start_ned),
                "启动航向       : {:.2f}deg（0=北，顺时针）".format(self.start_heading_deg),
                "锁定航向       : {:.2f}deg（{}）".format(
                    self._lock_heading(),
                    "固定 drop_zone_bearing_deg" if self.drop_zone_bearing_deg >= 0
                    else "起始航向（机头对准中线）"),
                "投放区 WGS84  : distance={:.2f}m lat={:.9f}, lon={:.9f}".format(
                    drop_distance, *self.drop_zone_wgs84),
                "侦察区 WGS84  : distance={:.2f}m lat={:.9f}, lon={:.9f}".format(
                    recon_distance, *self.recon_zone_wgs84),
                "起飞高度       : ground rel_alt + {:.2f}m".format(self.takeoff_altitude_m),
                "投放区航程高度 : ground rel_alt + {:.2f}m".format(
                    self.drop_zone_transit_altitude_m),
                "区域航线高度   : ground rel_alt + {:.2f}m".format(
                    self.flight_only_search_altitude_m),
                "区域航线模式   : {}（W/Z/I，LINE 等同 I）".format(self.area_search_pattern),
                "区域航线圈数   : {}".format(self.flight_only_pattern_laps),
                "侦察高度/保持  : ground rel_alt + {:.2f}m / {:.1f}s".format(
                    self.recon_altitude_m, self.recon_hold_s),
                "continue_recon : {}".format(self.flight_only_continue_to_recon),
                "完整路线       : {}".format(route),
                "Enter 前不会切 GUIDED、解锁、起飞或发布会移动飞机的 setpoint。",
                "=======================================================",
            ]
            print("\n".join(lines), flush=True)
            return
        lines = [
            "",
            "================ CUADC 全自动任务摘要 ================",
            "起飞点 WGS84 : lat={:.9f}, lon={:.9f}, alt={:.3f}m".format(*self.start_wgs84),
            "起始 NED      : N={:.3f}, E={:.3f}, D={:.3f}m".format(*self.start_ned),
            "起始航向      : {:.2f}deg（0=北，顺时针）".format(self.start_heading_deg),
            "锁定航向      : {:.2f}deg（{}）".format(
                self._lock_heading(),
                "固定 drop_zone_bearing_deg" if self.drop_zone_bearing_deg >= 0
                else "起始航向（操作手把机头对准投放区中线）"),
            "投放区 WGS84 : {:.2f}m, lat={:.9f}, lon={:.9f}".format(
                self.drop_zone_distance_m, *self.drop_zone_wgs84),
            "侦察区 WGS84 : {:.2f}m, lat={:.9f}, lon={:.9f}".format(
                self.recon_zone_distance_m, *self.recon_zone_wgs84),
            "区域中心间距  : 约 {:.2f}m".format(
                self.recon_zone_distance_m - self.drop_zone_distance_m),
            "起飞高度      : 地面 rel_alt + {:.2f}m".format(self.takeoff_altitude_m),
            "投放区航程高度: 地面 rel_alt + {:.2f}m".format(self.drop_zone_transit_altitude_m),
            "桶搜索高度    : 地面 rel_alt + {:.2f}m".format(self.search_altitude_m),
            "侦察航高      : 地面 rel_alt + {:.2f}m；保持 {:.1f}s".format(
                self.recon_altitude_m, self.recon_hold_s),
            "W/Z/I 搜索    : {}，区域 {:.1f}m x {:.1f}m".format(
                self.area_search_pattern, self.drop_zone_width_m, self.drop_zone_length_m),
            "T2 模式       : recon_enabled={} aim_only={} record_buckets={}"
            .format(self.recon_enabled, self.aim_only_mode, self.record_buckets),
            "桶尺寸标定    : ref_200={}m tol={} invert={}（depth*sqrt((w/fx)^2+(h/fy)^2)）"
            .format(self.bucket_ratio_ref_200, self.bucket_ratio_tol,
                    self.bucket_ratio_invert),
            "桶记录目录    : {}".format(self.bucket_record_dir),
            "A/B 余弹      : A={} B={}".format(self.ammo_a, self.ammo_b),
            "dry-run       : {}（true 时完整飞行但不发布舵机打开命令）".format(self.dry_run),
            "流程           : GUIDED/ARM/TAKEOFF -> 投放区桶搜索/估计 -> 选桶(先近后远) "
            "-> 投放 -> {} -> RTL".format(
                "侦察区" if self.recon_enabled else "直接"),
            "Enter 前不会切模式、解锁、起飞、发布移动 setpoint 或打开舵机。",
            "=======================================================",
        ]
        print("\n".join(lines), flush=True)

    @staticmethod
    def _read_enter(prompt):
        """读一次 Enter 授权；**EOF/Ctrl-D/异常一律视为未授权**（返回 False）。

        #54（2026-09-30）：`readline()` 在 EOF 返回 `''`（不是 None）。旧写法
        `is not None` 把"stdin=/dev/null / 无 tty / 服务化运行"的 EOF 当成
        "操作者已按 Enter" → 无人值守也会自动切 GUIDED/解锁/起飞（实测飞了一整段
        任务）。Enter 返回 `'\\n'`（truthy）、EOF 返回 `''`（falsy），用 bool() 区分。
        """
        try:
            sys.stdout.write(prompt)
            sys.stdout.flush()
            try:
                with open("/dev/tty", "r", encoding="utf-8") as terminal:
                    return bool(terminal.readline())
            except OSError:
                return bool(sys.stdin.readline())
        except (EOFError, KeyboardInterrupt, OSError):
            return False

    def wait_for_enter_authorization(self):
        if self.ground_test and not self.flight_only_mode:
            self.node.get_logger().warn("ground_test=true：启动命令视为地面测试授权，不实际起飞")
            self.authorized = True
            return True
        mission_name = "仅飞行航线验证" if self.flight_only_mode else "完整任务"
        ok = self._read_enter(
            "确认空域和载荷安全后按 Enter，授权执行{}（Ctrl+C 取消）: ".format(mission_name))
        self.authorized = bool(ok and rclpy.ok() and self.node.context.ok())
        if self.authorized:
            self.node.get_logger().warn("操作者已按 Enter，授权执行%s" % mission_name)
        return self.authorized

    def _call_service(self, client, request, timeout_s):
        """等价 rospy.ServiceProxy(req)：call_async + 轮询 future（后台 spin 投递响应）。"""
        if client is None:
            return (False, None)
        try:
            if not client.service_is_ready() and not client.wait_for_service(timeout_sec=timeout_s):
                self.node.get_logger().warning(
                    "服务 %s 不可用" % client.srv_name)
                return (False, None)
            future = client.call_async(request)
            deadline = monotonic_s() + timeout_s
            while not future.done():
                if monotonic_s() >= deadline:
                    break
                if not (rclpy.ok() and self.node.context.ok()):
                    break
                time.sleep(0.02)
            if future.done():
                return (True, future.result())
        except Exception as exc:
            self.node.get_logger().warning("服务调用异常: %s" % exc)
        return (False, None)

    def _ensure_flight_services(self):
        try:
            self._set_mode_srv = self.node.create_client(SetMode, "/mavros/set_mode")
            self._arming_srv = self.node.create_client(CommandBool, "/mavros/cmd/arming")
            self._takeoff_srv = self.node.create_client(CommandTOL, "/mavros/cmd/takeoff")
            for client in (self._set_mode_srv, self._arming_srv, self._takeoff_srv):
                if not client.wait_for_service(timeout_sec=10.0):
                    self.node.get_logger().error(
                        "等待 MAVROS 飞行服务失败: %s" % client.srv_name)
                    return False
            # /mavros/param/get 在 mavros 2.14 已移除且源中未被调用（错题 #18）→ 不建。
            return True
        except Exception as exc:
            self.node.get_logger().error("等待 MAVROS 飞行服务失败: %s" % exc)
            return False

    def _ensure_set_mode_service(self):
        if self._set_mode_srv is not None:
            return True
        try:
            self._set_mode_srv = self.node.create_client(SetMode, "/mavros/set_mode")
            if not self._set_mode_srv.wait_for_service(timeout_sec=10.0):
                self.node.get_logger().error("/mavros/set_mode 不可用")
                return False
            return True
        except Exception as exc:
            self.node.get_logger().error("/mavros/set_mode 不可用: %s" % exc)
            return False

    def arm_confirmed(self):
        for attempt in range(1, self.service_retry_attempts + 1):
            state = self._fresh_state()
            if state is not None and state.armed:
                return True
            req = CommandBool.Request()
            req.value = True
            ok, response = self._call_service(self._arming_srv, req, self.service_timeout_s)
            if not ok:
                self.node.get_logger().warning(
                    "解锁服务第 %d/%d 次异常" % (attempt, self.service_retry_attempts))
            deadline = monotonic_s() + self.mode_timeout_s
            while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
                state = self._fresh_state()
                if state is not None and state.armed:
                    self.node.get_logger().warn("已回读确认 armed=true")
                    return True
                time.sleep(0.1)
            self.node.get_logger().warning(
                "解锁第 %d/%d 次未确认，response=%s"
                % (attempt, self.service_retry_attempts, response))
            if attempt < self.service_retry_attempts:
                time.sleep(self.service_retry_backoff_s)
        # 183151 实据：ArduCopter 安全开关未按下时拒绝解锁（FCU "Arm: Safety Switch"，
        # CommandBool result=4），旧实现只报"自动解锁失败"就进 FAILSAFE。这里给操作手
        # 明确提示并按配置窗口自动重试（0=关闭，回落旧行为）。
        if (self.arming_safety_switch_grace_s > 0.0
                and self._recent_statustext_contains("Safety Switch")):
            self.node.get_logger().error(
                "解锁被拒：飞控安全开关未按下（FCU: Arm: Safety Switch）。"
                "请按下安全开关，%.0fs 内脚本会自动重试" % self.arming_safety_switch_grace_s)
            deadline = monotonic_s() + self.arming_safety_switch_grace_s
            while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
                req = CommandBool.Request()
                req.value = True
                self._call_service(self._arming_srv, req, self.service_timeout_s)
                wait_deadline = monotonic_s() + min(1.5, self.mode_timeout_s)
                while (rclpy.ok() and self.node.context.ok()
                       and monotonic_s() < wait_deadline):
                    state = self._fresh_state()
                    if state is not None and state.armed:
                        self.node.get_logger().warn(
                            "安全开关按下后已回读确认 armed=true")
                        return True
                    time.sleep(0.1)
                time.sleep(0.3)
        return False

    def capture_ground_relative_altitude(self):
        deadline = monotonic_s() + self.connection_timeout_s
        fresh_since = None
        while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
            state = self._fresh_state()
            nav = self._navigation_snapshot()
            if state is None or not state.connected or not state.armed:
                self.node.get_logger().error("记录地面 rel_alt 时飞控状态无效")
                return False
            if nav["rel_alt"] is not None and nav["rel_alt_age"] <= self.navigation_timeout_s:
                if fresh_since is None:
                    fresh_since = monotonic_s()
                if monotonic_s() - fresh_since >= 1.0:
                    self.ground_relative_altitude = float(nav["rel_alt"])
                    self.node.get_logger().warn(
                        "地面相对高度基准已记录: %.3fm；起飞目标=%.3fm "
                        "投放区航程目标=%.3fm 侦察目标=%.3fm"
                        % (self.ground_relative_altitude,
                           self.ground_relative_altitude + self.takeoff_altitude_m,
                           self.ground_relative_altitude + self.drop_zone_transit_altitude_m,
                           self.ground_relative_altitude + self.recon_altitude_m))
                    return True
            else:
                fresh_since = None
            time.sleep(0.1)
        return False

    def _takeoff_has_started(self):
        nav = self._navigation_snapshot()
        return (self.ground_relative_altitude is not None
                and nav["rel_alt"] is not None
                and nav["rel_alt_age"] <= self.navigation_grace_s
                and nav["rel_alt"] - self.ground_relative_altitude
                >= self.takeoff_started_threshold_m)

    def command_takeoff_and_wait(self):
        target_rel_alt = self.ground_relative_altitude + self.takeoff_altitude_m
        command_accepted = False
        for attempt in range(1, self.service_retry_attempts + 1):
            req = CommandTOL.Request()
            req.altitude = float(target_rel_alt)
            req.latitude = 0.0
            req.longitude = 0.0
            req.min_pitch = 0.0
            req.yaw = 0.0
            ok, response = self._call_service(self._takeoff_srv, req, self.service_timeout_s)
            if ok and response is not None and response.success:
                command_accepted = True
                break
            time.sleep(min(1.0, self.service_retry_backoff_s))
            if self._takeoff_has_started():
                self.node.get_logger().warn("起飞 ACK 不可靠，但检测到飞机已开始上升，继续监控")
                command_accepted = True
                break
            self.node.get_logger().warning(
                "起飞服务第 %d/%d 次未确认" % (attempt, self.service_retry_attempts))
        if not command_accepted:
            self.node.get_logger().error("起飞命令有界重试后仍未确认，且未检测到起飞开始")
            return False
        deadline = monotonic_s() + self.takeoff_timeout_s
        while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
            state = self._fresh_state()
            nav = self._navigation_snapshot()
            if state is not None and not state.armed:
                return False
            if nav["rel_alt"] is not None and nav["rel_alt_age"] <= self.navigation_grace_s:
                climbed = nav["rel_alt"] - self.ground_relative_altitude
                if climbed >= self.takeoff_altitude_m - 0.20:
                    self.airborne = True
                    self.node.get_logger().warn(
                        "已到达起飞高度: 离地 %.2f/%.2fm" % (climbed, self.takeoff_altitude_m))
                    return True
                throttle_log(self.node, 1.0, "起飞爬升中: 离地 %.2f/%.2fm",
                             climbed, self.takeoff_altitude_m)
            time.sleep(0.1)
        return False

    def _make_global_target(self, latitude, longitude, height_above_ground_m):
        msg = GlobalPositionTarget()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = (GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY
                         | GlobalPositionTarget.IGNORE_VZ | GlobalPositionTarget.IGNORE_AFX
                         | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ
                         | GlobalPositionTarget.IGNORE_YAW | GlobalPositionTarget.IGNORE_YAW_RATE)
        msg.latitude = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude = self.ground_relative_altitude + float(height_above_ground_m)
        return msg

    def _publish_global_target(self, target, label):
        target.header.stamp = self.node.get_clock().now().to_msg()
        self.global_setpoint_pub.publish(target)
        self.last_safe_global_target = target
        self.last_safe_setpoint_kind = "GLOBAL"
        self.last_setpoint_label = label
        self.last_setpoint_at = monotonic_s()

    def _flight_state_health(self):
        snapshot = self._control_stream_snapshot()
        state = snapshot["state"]
        if state is None:
            return "recover", "未收到 /mavros/state"
        if snapshot["state_age"] <= self.state_timeout_s:
            if not state.armed:
                return "fatal", "收到新鲜 armed=false"
            if state.mode != "GUIDED":
                return "fatal", "收到新鲜非 GUIDED 模式 {}，飞手接管".format(state.mode)
            if not state.connected:
                return "recover", "收到新鲜 connected=false"
            return "ok", ""
        stale_grace = (self.connection_loss_grace_s if not state.connected
                       else self.guided_state_grace_s)
        if snapshot["state_age"] <= stale_grace and state.armed and state.mode == "GUIDED":
            return "recover", "state age {:.2f}s".format(snapshot["state_age"])
        return "fatal", "state 超过恢复宽限 age={:.2f}s".format(snapshot["state_age"])

    # ==================================================================
    # 全局航点 / 保持 / 锚点 / RTL（源 L1687-2050）
    # ==================================================================
    def fly_global_waypoint(self, target, label, detect_dropper=None, require_full_telemetry=False):
        started_at = monotonic_s()
        inside_since = None
        detect_since = None
        recovery_started = None
        recovery_hold = None
        recovery_use_global = False
        last_recovery_log = 0.0
        while rclpy.ok() and self.node.context.ok():
            now = monotonic_s()
            if not self._restart_dead_children():
                return "failed"
            health, reason = self._flight_state_health()
            nav = self._navigation_snapshot()
            control = self._control_stream_snapshot()
            nav_fresh = (nav["fix"] is not None and nav["rel_alt"] is not None
                         and nav["fix_age"] <= self.navigation_timeout_s
                         and nav["rel_alt_age"] <= self.navigation_timeout_s)
            stale_fields = []
            expired_fields = []
            if require_full_telemetry:
                if nav["heading"] is None or nav["heading_age"] > self.navigation_timeout_s:
                    stale_fields.append("heading={:.2f}s".format(nav["heading_age"]))
                    if nav["heading"] is None or nav["heading_age"] > self.navigation_grace_s:
                        expired_fields.append(stale_fields[-1])
                if nav["pose"] is None or nav["pose_age"] > self.pose_timeout_s:
                    stale_fields.append("pose={:.2f}s".format(nav["pose_age"]))
                    if nav["pose"] is None or nav["pose_age"] > self.guided_pose_grace_s:
                        expired_fields.append(stale_fields[-1])
                if control["velocity"] is None or control["velocity_age"] > self.pose_timeout_s:
                    stale_fields.append("velocity={:.2f}s".format(control["velocity_age"]))
                    if (control["velocity"] is None
                            or control["velocity_age"] > self.guided_pose_grace_s):
                        expired_fields.append(stale_fields[-1])
                if nav["fix"] is None or nav["fix_age"] > self.navigation_grace_s:
                    expired_fields.append("fix={:.2f}s".format(nav["fix_age"]))
                if nav["rel_alt"] is None or nav["rel_alt_age"] > self.navigation_grace_s:
                    expired_fields.append("rel_alt={:.2f}s".format(nav["rel_alt_age"]))
            full_telemetry_fresh = not stale_fields
            if health == "fatal":
                self.node.get_logger().error("%s 安全退出: %s" % (label, reason))
                return "failed"
            if expired_fields:
                self.node.get_logger().error(
                    "%s 飞行遥测超过恢复宽限: %s" % (label, " ".join(expired_fields)))
                return "failed"
            if health == "recover" or not nav_fresh or not full_telemetry_fresh:
                if health == "recover":
                    recovery_reason = reason
                elif not nav_fresh:
                    recovery_reason = "导航不新鲜 fix={:.2f}s rel_alt={:.2f}s".format(
                        nav["fix_age"], nav["rel_alt_age"])
                else:
                    recovery_reason = "飞行遥测不新鲜 " + " ".join(stale_fields)
                if recovery_started is None:
                    recovery_started = now
                    self._begin_recovery("{}: {}".format(label, recovery_reason))
                    recovery_hold = self._safe_hold_enu()
                    if recovery_hold is None:
                        if self.last_safe_global_target is None:
                            self.node.get_logger().error(
                                "%s 进入恢复但没有最后安全 setpoint" % label)
                            return "failed"
                        recovery_use_global = True
                    else:
                        recovery_use_global = False
                if "connected" in recovery_reason:
                    grace = self.connection_loss_grace_s
                elif health == "recover":
                    grace = self.guided_state_grace_s
                elif any(f.startswith(("pose=", "velocity=")) for f in stale_fields):
                    grace = max(self.navigation_grace_s, self.guided_pose_grace_s)
                else:
                    grace = self.navigation_grace_s
                elapsed = now - recovery_started
                if elapsed > grace:
                    self.node.get_logger().error(
                        "%s 恢复超时 %.1f/%.1fs: %s" % (label, elapsed, grace, recovery_reason))
                    return "failed"
                if recovery_use_global:
                    self._publish_global_target(self.last_safe_global_target,
                                                "{} RECOVERY GLOBAL HOLD".format(label))
                else:
                    self._publish_hold_setpoint(recovery_hold,
                                                "{} RECOVERY HOLD".format(label))
                if now - last_recovery_log >= 1.0:
                    self.node.get_logger().warn(
                        "%s RECOVERY | %s | 已恢复等待 %.1fs 剩余宽限 %.1fs"
                        % (label, recovery_reason, elapsed, max(0.0, grace - elapsed)))
                    last_recovery_log = now
                time.sleep(0.1)
                continue
            if recovery_started is not None:
                paused = now - recovery_started
                started_at += paused
                self.node.get_logger().warn(
                    "%s 遥测已恢复，继续原航点；暂停计时 %.2fs" % (label, paused))
                recovery_started = None
                recovery_hold = None
                recovery_use_global = False
                self._end_recovery("{} 遥测恢复，继续原航点".format(label))

            if detect_dropper is not None:
                aim = self._fresh_aim()
                target_ned = self._aim_target_ned(aim, detect_dropper)
                eligible = (target_ned is not None
                            and not self._target_is_blacklisted(target_ned, detect_dropper))
                if eligible:
                    if detect_since is None:
                        detect_since = now
                    if now - detect_since >= self.enroute_detect_stable_s:
                        pose = self._fresh_pose()
                        if pose is None:
                            detect_since = None
                        else:
                            hold = self._pose_position_enu(pose)
                            remaining = horizontal_wgs84_distance(
                                nav["fix"].latitude, nav["fix"].longitude,
                                target.latitude, target.longitude)
                            aim_age = monotonic_s() - self.aim_received_at
                            self.node.get_logger().warn(
                                "%s 途中稳定发现目标 | aircraft_NED=(%.2f,%.2f,%.2f) "
                                "aircraft_WGS84=(%.8f,%.8f) bucket_NED=(%.2f,%.2f,%.2f) "
                                "aim_age=%.3fs remaining=%.2fm；停止全局 setpoint"
                                % (detect_dropper, pose.pose.position.y, pose.pose.position.x,
                                   -pose.pose.position.z, nav["fix"].latitude,
                                   nav["fix"].longitude, target_ned[0], target_ned[1],
                                   target_ned[2], aim_age, remaining))
                            self._publish_hold_setpoint(
                                hold, "{} 途中截获本地保持".format(detect_dropper))
                            time.sleep(self.setpoint_interval_s)
                            return "detected"
                else:
                    detect_since = None

            self._publish_global_target(target, label)
            horizontal_error = horizontal_wgs84_distance(
                nav["fix"].latitude, nav["fix"].longitude,
                target.latitude, target.longitude)
            vertical_error = abs(nav["rel_alt"] - target.altitude)
            inside = (horizontal_error <= self.waypoint_tolerance_m
                      and vertical_error <= self.waypoint_vertical_tolerance_m)
            if inside:
                if inside_since is None:
                    inside_since = now
                if now - inside_since >= self.waypoint_arrival_hold_s:
                    self.node.get_logger().warn(
                        "%s 已连续到达: horizontal=%.2fm vertical=%.2fm"
                        % (label, horizontal_error, vertical_error))
                    return "arrived"
            else:
                inside_since = None
            if now - started_at >= self.waypoint_timeout_s:
                self.node.get_logger().error(
                    "%s 超时: horizontal=%.2fm vertical=%.2fm"
                    % (label, horizontal_error, vertical_error))
                return "failed"
            if detect_dropper is None:
                throttle_log(self.node, 1.0,
                             "%s | horizontal=%.2fm vertical=%.2fm vision_monitor=DISABLED",
                             label, horizontal_error, vertical_error)
            else:
                throttle_log(self.node, 1.0,
                             "%s | horizontal=%.2fm vertical=%.2fm detect_stable=%.2f/%.2fs",
                             label, horizontal_error, vertical_error,
                             0.0 if detect_since is None else now - detect_since,
                             self.enroute_detect_stable_s)
            time.sleep(0.1)
        return "failed"

    def hold_global_target(self, target, duration_s, label, require_full_telemetry=False):
        held = 0.0
        last = monotonic_s()
        recovery_started = None
        while rclpy.ok() and self.node.context.ok() and held < duration_s:
            now = monotonic_s()
            if not self._restart_dead_children():
                self.node.get_logger().error("%s 期间依赖有界重启失败" % label)
                return False
            health, reason = self._flight_state_health()
            nav = self._navigation_snapshot()
            control = self._control_stream_snapshot()
            telemetry_reasons = []
            if require_full_telemetry:
                if nav["fix"] is None or nav["fix_age"] > self.navigation_timeout_s:
                    telemetry_reasons.append("fix={:.2f}s".format(nav["fix_age"]))
                if nav["heading"] is None or nav["heading_age"] > self.navigation_timeout_s:
                    telemetry_reasons.append("heading={:.2f}s".format(nav["heading_age"]))
                if nav["rel_alt"] is None or nav["rel_alt_age"] > self.navigation_timeout_s:
                    telemetry_reasons.append("rel_alt={:.2f}s".format(nav["rel_alt_age"]))
                if nav["pose"] is None or nav["pose_age"] > self.pose_timeout_s:
                    telemetry_reasons.append("pose={:.2f}s".format(nav["pose_age"]))
                if control["velocity"] is None or control["velocity_age"] > self.pose_timeout_s:
                    telemetry_reasons.append("velocity={:.2f}s".format(control["velocity_age"]))
            if health == "fatal":
                self.node.get_logger().error("%s: %s" % (label, reason))
                return False
            if health == "ok" and not telemetry_reasons:
                if recovery_started is not None:
                    self.node.get_logger().warn("%s 遥测恢复，继续累计保持时间" % label)
                    recovery_started = None
                self._publish_global_target(target, label)
                held += max(0.0, now - last)
            elif self.last_safe_global_target is not None:
                if recovery_started is None:
                    recovery_started = now
                self._publish_global_target(self.last_safe_global_target, label + " RECOVERY")
                recovery_text = reason or "飞行遥测不新鲜 " + " ".join(telemetry_reasons)
                max_age = max([nav["fix_age"], nav["heading_age"], nav["rel_alt_age"],
                               nav["pose_age"], control["velocity_age"]])
                grace = max(self.navigation_grace_s, self.guided_pose_grace_s,
                            self.guided_state_grace_s)
                if "connected" in recovery_text:
                    grace = self.connection_loss_grace_s
                recovery_elapsed = now - recovery_started
                if (max_age > grace and telemetry_reasons) or recovery_elapsed > grace:
                    self.node.get_logger().error("%s 恢复宽限耗尽: %s" % (label, recovery_text))
                    return False
                self.node.get_logger().warning(
                    "%s 暂停保持计时 %.2f/%.2fs: %s" % (label, recovery_elapsed, grace,
                                                         recovery_text),
                    throttle_duration_sec=1.0)
            else:
                return False
            last = now
            time.sleep(0.1)
        return rclpy.ok() and self.node.context.ok()

    def set_area_search_anchor_from_current(self, label):
        pose = self._fresh_pose()
        if pose is None or self.start_heading_deg is None:
            return False
        yaw_enu = math.radians(90.0 - self._lock_heading())
        p = pose.pose.position
        self.area_search_anchor = (float(p.x), float(p.y), float(p.z), yaw_enu)
        self.node.get_logger().warn(
            "%s 搜索中心已冻结: ENU=(%.2f,%.2f,%.2f), start_heading=%.2fdeg"
            % (label, p.x, p.y, p.z, self.start_heading_deg))
        return True

    def request_rtl_and_wait(self):
        if not self.set_mode_confirmed("RTL"):
            self.node.get_logger().error("RTL 请求或回读失败")
            return False
        deadline = monotonic_s() + self.rtl_timeout_s
        while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
            with self._lock:
                state = self.current_state
                state_age = monotonic_s() - self.state_received_at if state else float("inf")
            if state is not None and state_age <= self.connection_loss_grace_s:
                if not state.armed:
                    self.airborne = False
                    self.node.get_logger().warn("RTL 已完成并上锁")
                    return True
                if state.mode not in ("RTL", "LAND"):
                    self.node.get_logger().error(
                        "RTL 等待期间收到模式 %s，停止自动控制" % state.mode)
                    return False
            throttle_log(self.node, 2.0,
                         "RTL 中等待返航/降落 | mode=%s armed=%s state_age=%.2fs",
                         state.mode if state else "NONE", state.armed if state else None,
                         state_age)
            time.sleep(0.2)
        self.node.get_logger().error("RTL 等待超时；保持飞控当前 RTL，不在异地强制 LAND")
        return False

    def wait_for_mavros(self):
        deadline = monotonic_s() + self.connection_timeout_s
        while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
            state = self._fresh_state()
            if state is not None and state.connected:
                self.node.get_logger().info(
                    "MAVROS 已连接: armed=%s mode=%s" % (state.armed, state.mode))
                return True
            throttle_log(self.node, 2.0, "等待 /mavros/state 与飞控连接...")
            time.sleep(0.1)
        self.node.get_logger().error("等待 MAVROS 飞控连接超时")
        return False

    def request_local_position_stream(self):
        try:
            if self._message_interval_srv is None:
                self._message_interval_srv = self.node.create_client(
                    MessageInterval, "/mavros/set_message_interval")
            req = MessageInterval.Request()
            req.message_id = 32
            req.message_rate = float(self.local_position_rate_hz)
            ok, response = self._call_service(
                self._message_interval_srv, req, 3.0)
            if not ok:
                self._message_interval_srv = None
                self.node.get_logger().warning(
                    "请求 LOCAL_POSITION_NED %.1fHz 失败，将依靠遥测恢复宽限"
                    % self.local_position_rate_hz)
                return False
            if response.success:
                self.node.get_logger().warn(
                    "已请求 MAVLink LOCAL_POSITION_NED(message_id=32) 频率 %.1fHz"
                    % self.local_position_rate_hz)
                return True
            self.node.get_logger().warning(
                "飞控未接受 LOCAL_POSITION_NED %.1fHz 请求，将依靠遥测恢复宽限"
                % self.local_position_rate_hz)
            return False
        except Exception as exc:
            self._message_interval_srv = None
            self.node.get_logger().warning(
                "请求 LOCAL_POSITION_NED %.1fHz 失败，将依靠遥测恢复宽限: %s"
                % (self.local_position_rate_hz, exc))
            return False

    # ==================================================================
    # 目标确认 / 模式 / ENU setpoint（源 L2052-2432）
    # ==================================================================
    def wait_for_target_and_confirm(self, dropper="A"):
        if self.flight_only_mode:
            self.node.get_logger().error("FLIGHT ONLY 禁止等待或确认视觉目标")
            return False
        valid_since = None
        while rclpy.ok() and self.node.context.ok():
            aim = self._fresh_aim()
            target_ok = aim is not None and (
                self.ground_test or self._aim_target_ned(aim, dropper) is not None)
            if not target_ok:
                valid_since = None
                throttle_log(self.node, 2.0, "等待 %s 稳定视觉目标：%s", dropper,
                             self._aim_invalid_reason(None if self.ground_test else dropper))
                time.sleep(0.1)
                continue
            now = monotonic_s()
            if valid_since is None:
                valid_since = now
            stable = now - valid_since
            if stable < self.acquire_stable_time_s:
                throttle_log(self.node, 0.5, "%s 目标获取稳定时间 %.1f/%.1fs",
                             dropper, stable, self.acquire_stable_time_s)
                time.sleep(0.1)
                continue
            if self.ground_test:
                self.node.get_logger().warn(
                    "地面测试已由启动命令授权：检测到稳定有效桶目标，"
                    "不等待 Enter，直接开始 %s 抛投器对准监测" % dropper)
                return True
            if self.autonomous_mode:
                self.node.get_logger().warn(
                    "autonomous_mode=true：%s 目标稳定 %.1fs，自动进入 GUIDED" % (dropper, stable))
                return True
            prompt = ("检测到稳定桶目标，按 Enter 后进入 GUIDED，并执行分层自动瞄准；"
                      "等待期间目标失效将自动重新获取，Ctrl+C 取消。")
            self.node.get_logger().warn(prompt)
            try:
                sys.stdout.write("\n{}\n".format(prompt))
                sys.stdout.flush()
                tty = None
                try:
                    tty = open("/dev/tty", "r", encoding="utf-8", errors="replace")
                    stream = tty
                except OSError:
                    stream = sys.stdin
                try:
                    while rclpy.ok() and self.node.context.ok():
                        current = self._fresh_aim()
                        current_ok = current is not None and (
                            self._aim_target_ned(current, dropper) is not None)
                        if not current_ok:
                            self.node.get_logger().warn(
                                "等待 Enter 期间目标失效：%s；回到目标获取状态"
                                % self._aim_invalid_reason(dropper))
                            valid_since = None
                            break
                        ready, _, _ = select.select([stream], [], [], 0.1)
                        if ready:
                            return bool(stream.readline()) and rclpy.ok()
                finally:
                    if tty is not None:
                        tty.close()
            except (EOFError, KeyboardInterrupt, OSError):
                return False
        return False

    def _validate_before_guided(self, dropper):
        if self.ground_test:
            state = self._fresh_state()
            if state is None:
                self.node.get_logger().warn(
                    "ground_test=true：当前飞控状态消息不新鲜，但地面瞄准不依赖模式/定位")
            else:
                self.node.get_logger().warn(
                    "ground_test=true：忽略 armed=%s mode=%s，不要求 NED，不进入 GUIDED"
                    % (state.armed, state.mode))
            self.node.get_logger().warn(
                "%s 地面流程直接进入持续瞄准循环；目标无效时只等待并清零计时" % dropper)
            return True
        state = self._fresh_state()
        if state is None or not state.connected:
            self.node.get_logger().error("飞控状态无效，无法执行舵机测试")
            return False
        if not state.armed:
            self.node.get_logger().error("current_state.armed == false，拒绝进入投放流程")
            return False
        if state.mode != "LOITER":
            self.node.get_logger().error("当前模式不是 LOITER（实际为 %s），拒绝接管" % state.mode)
            return False
        if self._fresh_pose() is None:
            self.node.get_logger().error(
                "本地位姿无效，无法确认 %.2fm 投放高度" % self.target_drop_height_m)
            return False
        aim = self._fresh_aim()
        if aim is None or not getattr(aim, "{}_ned_valid".format(dropper.lower())):
            if self.autonomous_mode:
                self.node.get_logger().warn(
                    "autonomous_mode=true：进入 GUIDED 前暂无有效 %s 目标（%s）；"
                    "将从当前位置升高并搜索" % (dropper, self._aim_invalid_reason(dropper)))
                return True
            self.node.get_logger().error(
                "未收到新鲜有效的 %s /vision/bucket/aim_info，拒绝开始瞄准" % dropper)
            return False
        return True

    def set_mode_confirmed(self, mode):
        if not self._ensure_set_mode_service():
            return False
        if mode == "GUIDED":
            self.guided_was_requested = True
        actual = "UNKNOWN"
        for attempt in range(1, self.service_retry_attempts + 1):
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = mode
            ok, response = self._call_service(self._set_mode_srv, req, self.service_timeout_s)
            if not ok:
                self.node.get_logger().warning(
                    "调用 set_mode(%s) 第 %d/%d 次失败" % (mode, attempt, self.service_retry_attempts))
                self._set_mode_srv = None
                if not self._ensure_set_mode_service():
                    continue
            deadline = monotonic_s() + self.mode_timeout_s
            while monotonic_s() < deadline and rclpy.ok() and self.node.context.ok():
                state = self._fresh_state()
                if state is not None:
                    actual = state.mode
                    if actual == mode:
                        self.node.get_logger().warn("已回读 /mavros/state，确认模式为 %s" % mode)
                        if mode == "LOITER":
                            self.guided_was_requested = False
                        return True
                time.sleep(0.1)
            self.node.get_logger().warning(
                "模式第 %d/%d 次未确认: expected=%s actual=%s mode_sent=%s"
                % (attempt, self.service_retry_attempts, mode, actual,
                   getattr(response, "mode_sent", None)))
            if attempt < self.service_retry_attempts:
                time.sleep(self.service_retry_backoff_s)
        self.node.get_logger().error("模式确认重试耗尽: expected=%s actual=%s" % (mode, actual))
        return False

    @staticmethod
    def _ned_to_enu(north, east, down):
        return float(east), float(north), -float(down)

    def _target_enu_from_ned(self, target_ned, height_m):
        x, y, surface_z = self._ned_to_enu(*target_ned)
        return x, y, surface_z + float(height_m)

    def _aim_setpoint_ned(self, bucket_ned, dropper, pose):
        """投放器制导点：把"桶位置 NED"平移成"飞机应到的位置 NED"。

        飞机位置 = 桶位置 − 该投放器相对相机的实际偏移（2026-10-05 垂线标定）：
            前后 offset_x = 前投 +crosshair_offset_m / 后投 −crosshair_offset_m；
            横向 offset_y = crosshair_lateral_y_m（机体系 Y 正=右）。
        若仍把飞机飞到桶正上方（相机原点对桶），判据恒差一个 offset：
        T2 实据 2026-10-01 21:29 FINAL 阶段 Y 恒 +0.050m，20s 瞄准超时 → FAILSAFE。
        """
        offset_x = (self.crosshair_offset_m
                    if str(dropper).upper() == self.front_dropper
                    else -self.crosshair_offset_m)
        offset_y = self.crosshair_lateral_y_m
        if pose is None:
            return (float(bucket_ned[0]), float(bucket_ned[1]), float(bucket_ned[2]))
        offset_enu = self._quaternion_rotate(
            pose.pose.orientation, (offset_x, offset_y, 0.0))
        return (float(bucket_ned[0]) - float(offset_enu[1]),
                float(bucket_ned[1]) - float(offset_enu[0]),
                float(bucket_ned[2]))

    def _target_enu(self, aim, dropper):
        target_ned = self._aim_target_ned(aim, dropper)
        if target_ned is None:
            raise ValueError("{} NED 无效".format(dropper))
        return self._target_enu_from_ned(target_ned, self.target_drop_height_m)

    def _limit_setpoint_from_current_pose(self, desired_enu):
        pose = self._fresh_pose()
        if pose is None:
            return desired_enu
        now = monotonic_s()
        current = pose.pose.position
        dx = desired_enu[0] - float(current.x)
        dy = desired_enu[1] - float(current.y)
        horizontal = math.hypot(dx, dy)
        if horizontal > self.max_horizontal_setpoint_distance_m:
            scale = self.max_horizontal_setpoint_distance_m / horizontal
            dx *= scale
            dy *= scale
        dz = desired_enu[2] - float(current.z)
        dz = max(-self.max_vertical_setpoint_distance_m,
                 min(self.max_vertical_setpoint_distance_m, dz))
        candidate = (float(current.x) + dx, float(current.y) + dy, float(current.z) + dz)
        with self._lock:
            previous = self.last_setpoint_commanded_enu
            previous_at = self.last_setpoint_at
            previous_kind = self.last_safe_setpoint_kind
        # 来源切换（全局 setpoint → 本地）或旧命令点离飞机过远时：以"当前位姿+本次
        # 限幅"重新起算。否则速率限制会从陈旧基点上慢慢"飞回去"，飞机跟着旧命令点
        # 走（T2 实据：A 飞抵桶后首次本地 setpoint 从 50m 外基点回拉，20s 瞄准超时
        # 触发 FORCED_DROP、飞机被带离目标 14m）。
        if previous is None or previous_at <= 0.0 or previous_kind != "LOCAL":
            return candidate
        if math.hypot(previous[0] - float(current.x),
                      previous[1] - float(current.y)) > 5.0:
            return candidate
        dt = max(0.02, min(1.0, now - previous_at))
        step_x = candidate[0] - previous[0]
        step_y = candidate[1] - previous[1]
        horizontal_step = math.hypot(step_x, step_y)
        max_horizontal_step = self.max_horizontal_setpoint_rate_mps * dt
        if horizontal_step > max_horizontal_step:
            scale = max_horizontal_step / horizontal_step
            step_x *= scale
            step_y *= scale
        step_z = candidate[2] - previous[2]
        max_vertical_step = self.max_vertical_setpoint_rate_mps * dt
        step_z = max(-max_vertical_step, min(max_vertical_step, step_z))
        return (previous[0] + step_x, previous[1] + step_y, previous[2] + step_z)

    def _publish_enu_setpoint(self, desired_enu, label, yaw=None):
        commanded = self._limit_setpoint_from_current_pose(desired_enu)
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = commanded[0]
        msg.pose.position.y = commanded[1]
        msg.pose.position.z = commanded[2]
        pose = self._fresh_pose()
        if yaw is not None:
            msg.pose.orientation.z = math.sin(0.5 * float(yaw))
            msg.pose.orientation.w = math.cos(0.5 * float(yaw))
        elif pose is not None:
            msg.pose.orientation = pose.pose.orientation
        else:
            msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)
        with self._lock:
            self.last_setpoint_desired_enu = tuple(float(v) for v in desired_enu)
            self.last_setpoint_commanded_enu = tuple(float(v) for v in commanded)
            self.last_setpoint_label = str(label)
            self.last_setpoint_at = monotonic_s()
            self.last_safe_setpoint_kind = "LOCAL"
        throttle_log(self.node, 0.5,
                     "%s | desired ENU=(%.3f, %.3f, %.3f) limited=(%.3f, %.3f, %.3f)",
                     label, desired_enu[0], desired_enu[1], desired_enu[2],
                     commanded[0], commanded[1], commanded[2])
        return commanded

    def _publish_ned_setpoint(self, target_ned, height_m, label):
        return self._publish_enu_setpoint(
            self._target_enu_from_ned(target_ned, height_m), label)

    def _publish_hold_setpoint(self, hold_enu, label):
        return self._publish_enu_setpoint(hold_enu, label)

    def _search_target_ned(self, center_ned, search_elapsed_s):
        points = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0),
                  (-1.0, 0.0), (-1.0, -1.0), (0.0, -1.0), (1.0, -1.0))
        step_index = int(max(0.0, search_elapsed_s) / self.search_dwell_s)
        ring = step_index // len(points)
        point = points[step_index % len(points)]
        radius = min(self.search_start_radius_m + ring * self.search_radius_step_m,
                     self.search_max_radius_m)
        return (center_ned[0] + point[0] * radius, center_ned[1] + point[1] * radius,
                center_ned[2])

    def _publish_setpoint(self, aim, dropper):
        prefix = dropper.lower()
        if not getattr(aim, "{}_ned_valid".format(prefix)):
            self.node.get_logger().error("%s NED 无效，拒绝发送 setpoint" % dropper)
            return False
        north = getattr(aim, "{}_ned_n".format(prefix))
        east = getattr(aim, "{}_ned_e".format(prefix))
        down = getattr(aim, "{}_ned_d".format(prefix))
        target_ned = (float(north), float(east), float(down))
        x, y, z = self._target_enu_from_ned(target_ned, self.target_drop_height_m)
        self._publish_ned_setpoint(target_ned, self.target_drop_height_m,
                                   "{} 兼容投放 setpoint".format(dropper))
        self.node.get_logger().info(
            "%s 桶目标 NED=(%.3f, %.3f, %.3f)，发送桶上方 %.2fm ENU setpoint=(%.3f, %.3f, %.3f)"
            % (dropper, north, east, down, self.target_drop_height_m, x, y, z))
        return True

    def _publish_status(self, aiming=False, last_drop=""):
        if self.flight_only_mode:
            aiming = False
            last_drop = ""
        msg = MissionStatus()
        msg.ammo_a = self.ammo_a
        msg.ammo_b = self.ammo_b
        msg.aiming = aiming
        msg.last_drop = last_drop
        self.mission_status_pub.publish(msg)

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        siny_cosp = 2.0 * (float(quaternion.w) * float(quaternion.z)
                           + float(quaternion.x) * float(quaternion.y))
        cosy_cosp = 1.0 - 2.0 * (float(quaternion.y) ** 2 + float(quaternion.z) ** 2)
        return math.atan2(siny_cosp, cosy_cosp)

    # ==================================================================
    # 区域搜索（源 L2434-2567）
    # ==================================================================
    def _validate_before_area_search(self, dropper):
        state = self._fresh_state()
        if state is None or not state.connected:
            self.node.get_logger().error("%s 区域搜索前飞控状态无效" % dropper)
            return False
        if not state.armed:
            self.node.get_logger().error("%s 区域搜索前 armed=false，拒绝接管" % dropper)
            return False
        if state.mode != "LOITER":
            self.node.get_logger().error(
                "%s 区域搜索要求飞手先保持 LOITER，当前模式=%s" % (dropper, state.mode))
            return False
        if self._fresh_pose() is None:
            self.node.get_logger().error("%s 区域搜索前本地位姿无效" % dropper)
            return False
        return True

    def wait_for_area_search_confirm(self, dropper):
        if not self._validate_before_area_search(dropper):
            return False
        anchor_text = ("复用 A 搜索区域中心"
                       if (dropper == "B" and self.reuse_area_search_anchor_for_b
                           and self.area_search_anchor is not None)
                       else "把当前 LOITER 位置作为搜索区域中心")
        prompt = ("准备执行 {pattern} 字搜索 {dropper} 目标：{anchor}，区域 {width:.1f}m×"
                  "{length:.1f}m。按 Enter 后请求 GUIDED；Ctrl+C 取消。").format(
                      pattern=self.area_search_pattern, dropper=dropper, anchor=anchor_text,
                      width=self.drop_zone_width_m, length=self.drop_zone_length_m)
        self.node.get_logger().warn(prompt)
        try:
            sys.stdout.write("\n{}\n".format(prompt))
            sys.stdout.flush()
            tty = None
            try:
                tty = open("/dev/tty", "r", encoding="utf-8", errors="replace")
                stream = tty
            except OSError:
                stream = sys.stdin
            try:
                return bool(stream.readline()) and rclpy.ok()
            finally:
                if tty is not None:
                    tty.close()
        except (EOFError, KeyboardInterrupt, OSError):
            return False

    def _prepare_area_search_anchor(self, dropper):
        if (dropper == "B" and self.reuse_area_search_anchor_for_b
                and self.area_search_anchor is not None):
            self.node.get_logger().warn("B 搜索复用 A 已冻结的区域中心和机头方向")
            return True
        pose = self._fresh_pose()
        if pose is None:
            self.node.get_logger().error("%s 无法冻结搜索区域：本地位姿不新鲜" % dropper)
            return False
        position = pose.pose.position
        yaw = self._yaw_from_quaternion(pose.pose.orientation)
        values = (float(position.x), float(position.y), float(position.z), yaw)
        if not all(math.isfinite(value) for value in values):
            self.node.get_logger().error("%s 无法冻结搜索区域：位姿或航向包含非有限值" % dropper)
            return False
        self.area_search_anchor = values
        range_w, range_l = self._effective_detect_range()
        self.node.get_logger().warn(
            "%s 搜索区域已冻结 | center_ENU=(%.3f, %.3f, %.3f) yaw=%.1fdeg "
            "pattern=%s size=%.1fx%.1fm 识别范围(参考%.1fm)=%.2fx%.2fm→本次高度折算=%.2fx%.2fm"
            % (dropper, values[0], values[1], values[2], math.degrees(values[3]),
               self.area_search_pattern, self.drop_zone_width_m, self.drop_zone_length_m,
               self.detect_range_ref_altitude_m, self.detect_range_width_m,
               self.detect_range_length_m, range_w, range_l))
        return True

    def _effective_detect_range(self):
        """把"参考高度下的识别范围"按实际搜索高度线性折算为当前地面覆盖（宽, 长）。

        宽 = 机身边方向（画面横向）；长 = 机身前后方向（画面纵向）。
        flight_only 用 flight_only_search_altitude_m，其余用 search_altitude_m。
        """
        ref = self.detect_range_ref_altitude_m
        if ref <= 0.0:
            ref = 1.0
        alt = (self.flight_only_search_altitude_m if self.flight_only_mode
               else self.search_altitude_m)
        scale = max(0.0, float(alt)) / ref
        return (self.detect_range_width_m * scale, self.detect_range_length_m * scale)

    def _area_search_local_points(self):
        range_w, range_l = self._effective_detect_range()
        lateral = 0.5 * (self.drop_zone_width_m - range_w)
        forward = 0.5 * (self.drop_zone_length_m - range_l)
        if self.area_search_pattern == "I":
            return ((0.0, -lateral), (0.0, lateral))
        if self.area_search_pattern == "W":
            return ((-forward, -lateral), (-forward / 3.0, lateral),
                    (forward / 3.0, -lateral), (forward, lateral))
        return ((-forward, -lateral), (-forward, lateral),
                (forward, -lateral), (forward, lateral))

    def _area_search_enu_points(self):
        if self.area_search_anchor is None:
            raise ValueError("搜索区域中心尚未冻结")
        center_e, center_n, center_u, _yaw = self.area_search_anchor
        center_d = -center_u
        heading = math.radians(self._lock_heading())
        points = []
        for forward, right in self._area_search_local_points():
            north = center_n + forward * math.cos(heading) - right * math.sin(heading)
            east = center_e + forward * math.sin(heading) + right * math.cos(heading)
            down = center_d
            points.append((east, north, -down))
        return tuple(points)

    # ==================================================================
    # 仅飞行本地航线（源 L2569-2884）
    # ==================================================================
    def _flight_only_local_health(self):
        nav = self._navigation_snapshot()
        control = self._control_stream_snapshot()
        health, state_reason = self._flight_state_health()
        if health == "fatal":
            return "fatal", state_reason, nav, control, 0.0
        stale_reasons = []
        grace = 0.0
        if health == "recover":
            stale_reasons.append("STATE {}".format(state_reason))
            grace = max(grace, self.connection_loss_grace_s
                        if "connected" in state_reason else self.guided_state_grace_s)
        local_fields = (("POSE", nav["pose"], nav["pose_age"]),
                        ("VELOCITY", control["velocity"], control["velocity_age"]))
        for name, message, age in local_fields:
            if message is not None and age <= self.pose_timeout_s:
                continue
            if message is None or age > self.guided_pose_grace_s:
                return ("fatal", "{} 超过恢复宽限 age={:.2f}s grace={:.2f}s".format(
                    name, age, self.guided_pose_grace_s), nav, control, 0.0)
            stale_reasons.append("{}={:.2f}s".format(name, age))
            grace = max(grace, self.guided_pose_grace_s)
        navigation_fields = (("GPS", nav["fix"], nav["fix_age"]),
                             ("HEADING", nav["heading"], nav["heading_age"]),
                             ("REL_ALT", nav["rel_alt"], nav["rel_alt_age"]))
        for name, value, age in navigation_fields:
            if value is not None and age <= self.navigation_timeout_s:
                continue
            if value is None or age > self.navigation_grace_s:
                return ("fatal", "{} 超过恢复宽限 age={:.2f}s grace={:.2f}s".format(
                    name, age, self.navigation_grace_s), nav, control, 0.0)
            stale_reasons.append("{}={:.2f}s".format(name, age))
            grace = max(grace, self.navigation_grace_s)
        if stale_reasons:
            return "recover", " + ".join(stale_reasons), nav, control, grace
        return "ok", "", nav, control, 0.0

    def fly_area_pattern_only(self):
        if not self.flight_only_mode:
            self.node.get_logger().error("fly_area_pattern_only 仅允许在 flight_only_mode=true 时调用")
            return False
        if self.ground_relative_altitude is None or self.start_heading_deg is None:
            self.node.get_logger().error("FLIGHT ONLY 航线缺少地面 rel_alt 或冻结启动航向")
            return False
        health, reason, nav, _control, _grace = self._flight_only_local_health()
        if health != "ok":
            self.node.get_logger().error("FLIGHT ONLY 航线启动前遥测无效: %s" % reason)
            return False
        current_enu = self._pose_position_enu(nav["pose"])
        if current_enu is None:
            self.node.get_logger().error("FLIGHT ONLY 无法读取抵达投放区时的本地 ENU")
            return False
        self._publish_hold_setpoint(current_enu, "FLIGHT_ONLY GLOBAL_TO_LOCAL HOLD")
        time.sleep(self.setpoint_interval_s)
        target_rel_alt = self.ground_relative_altitude + self.flight_only_search_altitude_m
        search_center_z = current_enu[2] + (target_rel_alt - float(nav["rel_alt"]))
        yaw_enu = math.radians(90.0 - self._lock_heading())
        self.area_search_anchor = (current_enu[0], current_enu[1], search_center_z, yaw_enu)
        points = self._area_search_enu_points()
        self.flight_only_total_points = len(points)
        self.flight_only_current_lap = 0
        self.flight_only_current_point = 0
        self.node.get_logger().warn(
            "FLIGHT ONLY 区域航线中心已冻结 | center_ENU=(%.3f, %.3f) arrival_ENU_z=%.3f "
            "target_ENU_z=%.3f target_rel_alt=%.3f start_heading=%.2fdeg pattern=%s "
            "points=%d laps=%d"
            % (current_enu[0], current_enu[1], current_enu[2], search_center_z, target_rel_alt,
               self.start_heading_deg, self.area_search_pattern, len(points),
               self.flight_only_pattern_laps))
        altitude_phase = True
        lap_index = 0
        point_index = 0
        inside_since = None
        next_setpoint_at = 0.0
        recovery_started = None
        recovery_hold = None
        while rclpy.ok() and self.node.context.ok():
            now = monotonic_s()
            if not self._restart_dead_children():
                self.node.get_logger().error("FLIGHT ONLY 航线期间依赖有界重启失败")
                return False
            health, reason, nav, control, grace = self._flight_only_local_health()
            if health == "fatal":
                self.node.get_logger().error("FLIGHT ONLY 航线安全退出: %s" % reason)
                return False
            if health == "recover":
                if recovery_started is None:
                    recovery_started = now
                    recovery_hold = self._safe_hold_enu(
                        control, fallback=self.area_search_anchor[:3])
                    if recovery_hold is None:
                        self.node.get_logger().error(
                            "FLIGHT ONLY 遥测恢复没有可用安全保持 setpoint")
                        return False
                    self._begin_recovery("FLIGHT_ONLY_PATTERN: {}".format(reason))
                elapsed = now - recovery_started
                if elapsed > grace:
                    self.node.get_logger().error(
                        "FLIGHT ONLY 遥测恢复超时 %.2f/%.2fs: %s" % (elapsed, grace, reason))
                    return False
                inside_since = None
                if now >= next_setpoint_at:
                    self._publish_hold_setpoint(recovery_hold,
                                                "FLIGHT_ONLY_PATTERN RECOVERY HOLD")
                    next_setpoint_at = now + self.setpoint_interval_s
                state = control["state"]
                self.node.get_logger().warning(
                    "FLIGHT ONLY RECOVERY | reason=%s elapsed=%.2f/%.2fs state_age=%.2fs "
                    "pose_age=%.2fs velocity_age=%.2fs mode=%s RC=%s hold_ENU=(%.3f,%.3f,%.3f)"
                    % (reason, elapsed, grace, control["state_age"], control["pose_age"],
                       control["velocity_age"], state.mode if state is not None else "NONE",
                       self._rc_summary(), recovery_hold[0], recovery_hold[1], recovery_hold[2]),
                    throttle_duration_sec=1.0)
                time.sleep(0.1)
                continue
            if recovery_started is not None:
                self.node.get_logger().warn(
                    "FLIGHT ONLY 遥测恢复，继续圈 %d/%d 点 %d/%d；到达停留重新计时"
                    % (self.flight_only_current_lap, self.flight_only_pattern_laps,
                       self.flight_only_current_point, self.flight_only_total_points))
                recovery_started = None
                recovery_hold = None
                inside_since = None
                self._end_recovery("FLIGHT_ONLY_PATTERN 遥测恢复")
            if altitude_phase:
                target_enu = (self.area_search_anchor[0], self.area_search_anchor[1],
                              self.area_search_anchor[2])
                phase_text = "ALTITUDE"
                shown_lap = 0
                shown_point = 0
            else:
                if lap_index >= self.flight_only_pattern_laps:
                    self.flight_only_current_lap = self.flight_only_pattern_laps
                    self.flight_only_current_point = self.flight_only_total_points
                    self.node.get_logger().warn(
                        "FLIGHT ONLY %s 字航线全部完成，共 %d 圈"
                        % (self.area_search_pattern, self.flight_only_pattern_laps))
                    return True
                target_enu = points[point_index]
                shown_lap = lap_index + 1
                shown_point = point_index + 1
                self.flight_only_current_lap = shown_lap
                self.flight_only_current_point = shown_point
                phase_text = "PATTERN"
            if now >= next_setpoint_at:
                self._publish_enu_setpoint(
                    target_enu,
                    "FLIGHT_ONLY {} {} lap {}/{} point {}/{}".format(
                        self.area_search_pattern, phase_text, shown_lap,
                        self.flight_only_pattern_laps, shown_point,
                        self.flight_only_total_points),
                    yaw=self.area_search_anchor[3])
                next_setpoint_at = now + self.setpoint_interval_s
            current_enu = self._pose_position_enu(nav["pose"])
            if current_enu is None:
                self.node.get_logger().error("FLIGHT ONLY 收到非有限本地 ENU")
                return False
            horizontal_error = math.hypot(target_enu[0] - current_enu[0],
                                          target_enu[1] - current_enu[1])
            vertical_error = abs(float(nav["rel_alt"]) - target_rel_alt)
            inside = (horizontal_error <= self.area_search_waypoint_tolerance_m
                      and vertical_error <= self.area_search_height_tolerance_m)
            if inside:
                if inside_since is None:
                    inside_since = now
                if now - inside_since >= self.area_search_dwell_s:
                    if altitude_phase:
                        altitude_phase = False
                        inside_since = None
                        self.node.get_logger().warn(
                            "FLIGHT ONLY 已到区域航线高度：rel_alt=%.3f target=%.3f；开始第一段"
                            % (nav["rel_alt"], target_rel_alt))
                    else:
                        self.node.get_logger().warn(
                            "FLIGHT ONLY %s | 圈 %d/%d 到达点 %d/%d，horizontal=%.3fm "
                            "vertical=%.3fm dwell=%.2fs"
                            % (self.area_search_pattern, shown_lap,
                               self.flight_only_pattern_laps, shown_point,
                               self.flight_only_total_points, horizontal_error, vertical_error,
                               self.area_search_dwell_s))
                        point_index += 1
                        inside_since = None
                        if point_index >= len(points):
                            lap_index += 1
                            self.node.get_logger().warn(
                                "FLIGHT ONLY %s 字第 %d/%d 圈完整完成"
                                % (self.area_search_pattern, lap_index,
                                   self.flight_only_pattern_laps))
                            point_index = 0
            else:
                inside_since = None
            state = control["state"]
            throttle_log(
                self.node, 1.0,
                "FLIGHT ONLY TRACE | pattern=%s phase=%s lap=%d/%d point=%d/%d "
                "target_ENU=(%.3f,%.3f,%.3f) current_ENU=(%.3f,%.3f,%.3f) "
                "horizontal_error=%.3fm vertical_error=%.3fm state_age=%.3fs pose_age=%.3fs "
                "velocity_age=%.3fs mode=%s RC=%s",
                self.area_search_pattern, phase_text, shown_lap,
                self.flight_only_pattern_laps, shown_point, self.flight_only_total_points,
                target_enu[0], target_enu[1], target_enu[2], current_enu[0], current_enu[1],
                current_enu[2], horizontal_error, vertical_error, control["state_age"],
                control["pose_age"], control["velocity_age"],
                state.mode if state is not None else "NONE", self._rc_summary())
            time.sleep(0.1)
        return False

    def area_search_for_target(self, dropper):
        if self.flight_only_mode:
            self.node.get_logger().error("FLIGHT ONLY 禁止进入目标区域搜索")
            return False
        points = self._area_search_enu_points()
        started_at = monotonic_s()
        point_index = 0
        point_inside_since = None
        candidate_ned = None
        candidate_valid_since = None
        candidate_hold_enu = None
        next_setpoint_at = 0.0
        telemetry_stale_started_at = None
        telemetry_stale_hold_enu = None
        telemetry_stale_reason = None
        self.node.get_logger().warn(
            "%s 开始 %s 字区域搜索，共 %d 个路径点；航段中持续检测目标"
            % (dropper, self.area_search_pattern, len(points)))
        while rclpy.ok() and self.node.context.ok():
            now = monotonic_s()
            if not self._restart_dead_children():
                self.node.get_logger().error("%s 区域搜索期间依赖重启失败" % dropper)
                return False
            phase_label = "AREA_{}".format(self.area_search_pattern)
            state = self._fresh_state()
            pose = self._fresh_pose()
            snapshot = self._control_stream_snapshot()
            stale_reasons = []
            fatal_reason = None
            if state is None:
                raw_state = snapshot["state"]
                state_grace = (self.connection_loss_grace_s
                               if raw_state is not None and not raw_state.connected
                               else self.guided_state_grace_s)
                if not (raw_state is not None and raw_state.armed
                        and raw_state.mode == "GUIDED"
                        and snapshot["state_age"] <= state_grace):
                    fatal_reason = (
                        "区域搜索 /mavros/state 恢复失败：last_mode={} connected={} armed={} "
                        "age={:.3f}s grace={:.3f}s").format(
                            raw_state.mode if raw_state is not None else "NONE",
                            raw_state.connected if raw_state is not None else None,
                            raw_state.armed if raw_state is not None else None,
                            snapshot["state_age"], self.guided_state_grace_s)
                else:
                    stale_reasons.append("STATE age={:.3f}/{:.3f}s".format(
                        snapshot["state_age"], self.guided_state_grace_s))
            elif not state.armed or state.mode != "GUIDED":
                fatal_reason = "区域搜索安全状态失效：connected={} armed={} mode={}".format(
                    state.connected, state.armed, state.mode)
            elif not state.connected:
                stale_reasons.append("CONNECTION connected=false")
            if pose is None:
                if (snapshot["pose"] is None
                        or snapshot["pose_age"] > self.guided_pose_grace_s):
                    fatal_reason = (
                        "区域搜索本地位姿恢复失败：pose_age={:.3f}s timeout={:.3f}s "
                        "grace={:.3f}s").format(snapshot["pose_age"], self.pose_timeout_s,
                                                self.guided_pose_grace_s)
                else:
                    stale_reasons.append("POSE age={:.3f}/{:.3f}s".format(
                        snapshot["pose_age"], self.guided_pose_grace_s))
            if fatal_reason is not None:
                self._log_control_diagnostics(dropper, phase_label, candidate_ned, fatal_reason)
                return False
            if stale_reasons:
                if telemetry_stale_started_at is None:
                    telemetry_stale_started_at = now
                    telemetry_stale_reason = " + ".join(stale_reasons)
                    self._begin_recovery("{} 区域搜索: {}".format(dropper, telemetry_stale_reason))
                    telemetry_stale_hold_enu = self._safe_hold_enu(
                        snapshot, fallback=self.area_search_anchor[:3])
                    if telemetry_stale_hold_enu is None:
                        self._log_control_diagnostics(
                            dropper, phase_label, candidate_ned,
                            "区域搜索遥测短暂不新鲜，但没有可安全保持的 ENU setpoint")
                        return False
                stale_grace = (self.connection_loss_grace_s
                               if any("CONNECTION" in r for r in stale_reasons)
                               else max(self.guided_state_grace_s, self.guided_pose_grace_s))
                if now - telemetry_stale_started_at > stale_grace:
                    self._log_control_diagnostics(
                        dropper, phase_label, candidate_ned,
                        "区域搜索恢复超时 {:.1f}/{:.1f}s: {}".format(
                            now - telemetry_stale_started_at, stale_grace,
                            " + ".join(stale_reasons)))
                    return False
                point_inside_since = None
                candidate_valid_since = None
                candidate_ned = None
                candidate_hold_enu = None
                if now >= next_setpoint_at:
                    self._publish_enu_setpoint(
                        telemetry_stale_hold_enu,
                        "{} AREA_{} TELEMETRY_STALE 保持".format(
                            dropper, self.area_search_pattern),
                        yaw=self.area_search_anchor[3])
                    next_setpoint_at = now + self.setpoint_interval_s
                self.node.get_logger().warning(
                    "%s AREA_%s TELEMETRY_STALE 恢复等待 | %s | 保持ENU=(%.3f, %.3f, %.3f)"
                    % (dropper, self.area_search_pattern, " + ".join(stale_reasons),
                       telemetry_stale_hold_enu[0], telemetry_stale_hold_enu[1],
                       telemetry_stale_hold_enu[2]), throttle_duration_sec=0.5)
                self._publish_status(aiming=True)
                self._log_control_diagnostics(dropper, phase_label, candidate_ned)
                time.sleep(0.1)
                continue
            if telemetry_stale_started_at is not None:
                stale_duration = now - telemetry_stale_started_at
                started_at += stale_duration
                self.node.get_logger().warn(
                    "%s 区域搜索遥测已恢复，停滞 %.2fs；继续原路径点 %d/%d，"
                    "搜索/停留/候选稳定计时重新开始"
                    % (dropper, stale_duration, point_index + 1, len(points)))
                telemetry_stale_started_at = None
                telemetry_stale_hold_enu = None
                telemetry_stale_reason = None
                self._end_recovery("{} 区域搜索遥测恢复".format(dropper))
            elapsed = now - started_at
            if elapsed >= self.area_search_timeout_s:
                self.node.get_logger().error(
                    "%s %s 字区域搜索超时 %.1f/%.1fs，未获得稳定目标"
                    % (dropper, self.area_search_pattern, elapsed, self.area_search_timeout_s))
                return False
            aim = self._fresh_aim()
            measured_ned = self._aim_target_ned(aim, dropper)
            accepted = False
            if measured_ned is not None:
                candidate_ned, accepted = self._update_target_lock(
                    candidate_ned, measured_ned, dropper)
            if accepted:
                if candidate_valid_since is None:
                    candidate_valid_since = now
                    candidate_hold_enu = (float(pose.pose.position.x),
                                          float(pose.pose.position.y),
                                          float(pose.pose.position.z))
                    self.node.get_logger().warn(
                        "%s 区域搜索发现候选目标，停止航线并保持当前位置确认" % dropper)
                stable = now - candidate_valid_since
                if now >= next_setpoint_at:
                    self._publish_enu_setpoint(
                        candidate_hold_enu,
                        "{} AREA_{} 目标确认保持".format(dropper, self.area_search_pattern),
                        yaw=self.area_search_anchor[3])
                    next_setpoint_at = now + self.setpoint_interval_s
                self.node.get_logger().warning(
                    "%s 候选目标稳定 %.1f/%.1fs" % (dropper, stable, self.acquire_stable_time_s),
                    throttle_duration_sec=0.5)
                self._publish_status(aiming=True)
                if stable >= self.acquire_stable_time_s:
                    self.node.get_logger().warn(
                        "%s 区域搜索目标连续稳定 %.1fs，转入分层瞄准" % (dropper, stable))
                    return True
                time.sleep(0.1)
                continue
            if candidate_valid_since is not None:
                self.node.get_logger().warn("%s 候选目标确认期间失效，恢复原搜索航线" % dropper)
            candidate_ned = None
            candidate_valid_since = None
            candidate_hold_enu = None
            if point_index >= len(points):
                self.node.get_logger().warn(
                    "%s %s 字航线一轮完成但未找到稳定目标，返回第一个路径点继续搜索至超时"
                    % (dropper, self.area_search_pattern))
                point_index = 0
                point_inside_since = None
            target_enu = points[point_index]
            if now >= next_setpoint_at:
                self._publish_enu_setpoint(
                    target_enu,
                    "{} AREA_{} 点 {}/{}".format(dropper, self.area_search_pattern,
                                                 point_index + 1, len(points)),
                    yaw=self.area_search_anchor[3])
                next_setpoint_at = now + self.setpoint_interval_s
            position = pose.pose.position
            horizontal_error = math.hypot(target_enu[0] - float(position.x),
                                          target_enu[1] - float(position.y))
            vertical_error = abs(target_enu[2] - float(position.z))
            inside = (horizontal_error <= self.area_search_waypoint_tolerance_m
                      and vertical_error <= self.area_search_height_tolerance_m)
            if inside:
                if point_inside_since is None:
                    point_inside_since = now
                if now - point_inside_since >= self.area_search_dwell_s:
                    self.node.get_logger().warn(
                        "%s %s 字搜索到达点 %d/%d，继续下一航段"
                        % (dropper, self.area_search_pattern, point_index + 1, len(points)))
                    point_index += 1
                    point_inside_since = None
            else:
                point_inside_since = None
            throttle_log(self.node, 0.5,
                         "%s AREA_%s | 点=%d/%d 平面误差=%.2fm 高度误差=%.2fm 已搜索=%.1f/%.1fs",
                         dropper, self.area_search_pattern,
                         min(point_index + 1, len(points)), len(points),
                         horizontal_error, vertical_error, elapsed, self.area_search_timeout_s)
            self._publish_status(aiming=True)
            self._log_control_diagnostics(
                dropper,
                "AREA_{}_{}".format(self.area_search_pattern, min(point_index + 1, len(points))),
                candidate_ned)
            time.sleep(0.1)
        return False

    # ==================================================================
    # T2 桶搜索 / 位置估计 / 选桶 / 持久记录（2026-09-30）
    # ==================================================================
    @staticmethod
    def _bucket_diameter_mm(size_class):
        return {"small": 150, "mid": 200, "large": 250}.get(size_class, 200)

    def _bucket_ratio(self, det):
        """物理尺寸代理 ratio=Z*sqrt((w/fx)^2+(h/fy)^2)，单位 m。"""
        try:
            dx = float(det.x_max) - float(det.x_min)
            dy = float(det.y_max) - float(det.y_min)
            depth = float(getattr(det, "depth_m", 0.0))
        except Exception:
            return None
        fx = float(getattr(self, "camera_fx", 0.0))
        fy = float(getattr(self, "camera_fy", 0.0))
        if dx <= 0.0 or dy <= 0.0 or depth <= 0.0 or fx <= 0.0 or fy <= 0.0:
            return None
        return depth * math.hypot(dx / fx, dy / fy)

    def _bucket_size_class(self, ratio):
        """按 200mm 标定参考值分档；物理尺寸代理越大表示桶越大。"""
        ref = self.bucket_ratio_ref_200
        tol = self.bucket_ratio_tol
        high = ref * (1.0 + tol)
        low = ref * (1.0 - tol)
        if self.bucket_ratio_invert:
            if ratio > high:
                return "small"
            if ratio < low:
                return "large"
            return "mid"
        if ratio > high:
            return "large"
        if ratio < low:
            return "small"
        return "mid"

    def _associate_detection(self, geo, detections):
        """GeoTarget↔YoloDetections 目标关联（稳健方案）。

        主方案：按 (-confidence, center_x) 排序索引对齐 —— 与 detector 的
        target_id 编号键完全一致；再用"中心像素距 ≤60px 且 |Δconfidence| ≤0.2"
        做校验。校验不过退化为同帧中心像素最近邻（同限幅）。两者都不满足返回 None。
        """
        best = None
        best_score = float("inf")
        for det in detections:
            if not bool(getattr(det, "detected", False)):
                continue
            if float(getattr(det, "confidence", 0.0)) < self.vision_min_confidence:
                continue
            if not bool(getattr(det, "position_valid", False)):
                continue
            if not (float(getattr(det, "depth_m", 0.0)) > 0.0):
                continue
            px = math.hypot(int(det.center_x) - int(geo.center_x),
                            int(det.center_y) - int(geo.center_y))
            dconf = abs(float(det.confidence) - float(geo.confidence))
            if px > 60.0 or dconf > 0.2:
                continue
            score = px + 100.0 * dconf
            if score < best_score:
                best_score = score
                best = det
        return best

    def _merge_bucket(self, east, north, up, lat, lon, alt, ratio):
        """单条观测并入估计：≤关联门限并进最近桶，否则新建桶。

        整帧观测请优先走 `_associate_frame`（每桶每帧至多吸收一条，避免近距桶
        被并成假桶）；本函数用于帧内未配对的观测——同一桶的重复观测仍应并入。
        """
        target = None
        best_distance = float("inf")
        for bucket in self.buckets:
            distance = math.hypot(bucket["e"] - east, bucket["n"] - north)
            if distance < best_distance:
                best_distance = distance
                target = bucket
        if target is not None and best_distance <= self.bucket_assoc_distance_m:
            target["samples"].append((east, north, up, lat, lon, alt, ratio))
        else:
            samples = deque(maxlen=15)
            samples.append((east, north, up, lat, lon, alt, ratio))
            target = {"id": len(self.buckets) + 1, "samples": samples}
            self.buckets.append(target)
            self.node.get_logger().warn(
                "发现桶候选 #%d：ENU=(%.2f,%.2f,%.2f) ratio=%.4f"
                % (target["id"], east, north, up, ratio))
        self._recompute_bucket(target)
        return target

    def _associate_frame(self, observations):
        """整帧一对一关联（错题 #59 修正）：近者优先，每桶每帧至多吸收一条观测。

        旧实现逐条观测独立"找最近桶，≤1.5m 就并"：当两桶间距小于门限（场地规则
        允许三筒随机摆放，world 实测最小 1.35m）时，第二桶的首帧观测会被并进第一
        桶，生成位置=两桶中点、比值=两者中位混合的**假桶**，选桶随之选错。现在：
          1) 先枚举 (观测, 桶) 距离对，按距离升序贪心配对（≤ 关联门限）；
          2) 每桶、每观测本帧至多配对一次 → 相邻桶首帧各自成桶；
          3) 未配对观测再按单条规则处理（≤门限 → 同桶重复观测并入；否则新建桶）。
        """
        pairs = []
        for index, obs in enumerate(observations):
            for bucket in self.buckets:
                distance = math.hypot(bucket["e"] - obs[0], bucket["n"] - obs[1])
                if distance <= self.bucket_assoc_distance_m:
                    pairs.append((distance, index, bucket))
        pairs.sort(key=lambda item: item[0])
        used_obs = set()
        used_bucket = set()
        for _distance, index, bucket in pairs:
            if index in used_obs or id(bucket) in used_bucket:
                continue
            used_obs.add(index)
            used_bucket.add(id(bucket))
            bucket["samples"].append(observations[index])
            self._recompute_bucket(bucket)
        for index, obs in enumerate(observations):
            if index not in used_obs:
                self._merge_bucket(*obs)

    def _recompute_bucket(self, bucket):
        """滑动中位数估计（抑制 GeoTarget 坐标跳变）。"""
        samples = list(bucket["samples"])
        bucket["e"] = median(s[0] for s in samples)
        bucket["n"] = median(s[1] for s in samples)
        bucket["u"] = median(s[2] for s in samples)
        bucket["lat"] = median(s[3] for s in samples)
        bucket["lon"] = median(s[4] for s in samples)
        bucket["alt"] = median(s[5] for s in samples)
        bucket["ratio"] = median(s[6] for s in samples)
        bucket["size_class"] = self._bucket_size_class(bucket["ratio"])
        bucket["diameter_mm"] = self._bucket_diameter_mm(bucket["size_class"])
        bucket["samples_count"] = len(samples)
        bucket["updated_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        bucket["distance_m"] = 0.0

    def _observe_buckets(self):
        """消费最新一帧 GeoTarget + YoloDetections，更新桶估计；返回本帧观测数。

        注意（T2 修正）：detector 的 GeoTarget.enu_* 是**目标相对机身的位置偏移**
        （机身系→ENU 旋转后的 (east, north, up) 偏移，旧字段语义不变），不是绝对
        坐标。此处加当前本地位姿（ENU）换算成绝对 ENU 后再参与关联/中位数估计；
        否则飞机一移动，同一物理桶的观测会被拆散、距飞机距离失真（选桶"先近后远"
        会错）。lat/lon/altitude 本就是绝对 WGS84，不做处理。
        """
        now = monotonic_s()
        with self._lock:
            geo_fresh = now - self.geo_targets_received_at
            det_fresh = now - self.detections_received_at
            geo_map = dict(self.latest_geo_targets)
            det_msg = self.current_detections
        if geo_fresh > self.aim_info_timeout_s or det_fresh > self.aim_info_timeout_s:
            return 0
        if not geo_map or det_msg is None:
            return 0
        pose = self._fresh_pose()
        if pose is None:
            return 0
        base_e = float(pose.pose.position.x)
        base_n = float(pose.pose.position.y)
        base_u = float(pose.pose.position.z)
        detections = list(getattr(det_msg, "detections", []))
        observations = []
        for target_id in sorted(geo_map):
            geo = geo_map[target_id]
            if not bool(getattr(geo, "valid", False)):
                continue
            det = self._associate_detection(geo, detections)
            if det is None:
                continue
            ratio = self._bucket_ratio(det)
            if ratio is None or not math.isfinite(ratio):
                continue
            values = (base_e + float(geo.enu_east_m),
                      base_n + float(geo.enu_north_m),
                      base_u + float(geo.enu_up_m),
                      float(geo.latitude), float(geo.longitude), float(geo.altitude))
            if not all(math.isfinite(v) for v in values):
                continue
            observations.append(values + (ratio,))
        if observations:
            self._associate_frame(observations)
        return len(observations)

    def _update_bucket_distances(self):
        pose = self._fresh_pose()
        current_e = current_n = None
        if pose is not None:
            current_e = float(pose.pose.position.x)
            current_n = float(pose.pose.position.y)
        for bucket in self.buckets:
            if current_e is None:
                bucket["distance_m"] = 0.0
            else:
                bucket["distance_m"] = math.hypot(bucket["e"] - current_e,
                                                  bucket["n"] - current_n)

    def _prune_buckets(self):
        """丢弃噪声：优先保留 ≥bucket_min_samples 次观测的桶；否则保留全部 ≥1 次的。"""
        min_samples = max(1, int(getattr(self, "bucket_min_samples", 3)))
        strong = [b for b in self.buckets
                  if b.get("samples_count", 0) >= min_samples]
        if strong:
            self.buckets = strong
        else:
            self.buckets = [b for b in self.buckets if b.get("samples_count", 0) >= 1]
        for index, bucket in enumerate(self.buckets, start=1):
            bucket["id"] = index

    def _bucket_by_id(self, bucket_id):
        for bucket in self.buckets:
            if bucket["id"] == bucket_id:
                return bucket
        return None

    def _select_buckets(self):
        """选桶（2026-10-05 用户实测改口径）：按比值大小排序取"中号+小号"。

        三桶：中间比值=中号(≈200mm)、最小比值=小号(≈150mm)；两桶：直接都投；
        单桶：只投它。不再依赖 ref 绝对匹配（实测 200mm≈0.40 / 150mm≈0.26，
        ref 仅用于档位标签）。先近后远依据 `_update_bucket_distances()`。
        """
        ref = self.bucket_ratio_ref_200
        buckets = list(self.buckets)
        notes = list(self.bucket_notes)
        if not buckets:
            return [], ["未搜索到任何桶"]
        # 2026-10-05 用户实测拍板（200mm≈0.40 / 150mm≈0.26，1200~1500mm 高度稳定）：
        # 不做绝对比值匹配，只按比值大小排序——三桶取中间比值=中号(200)、最小比值=
        # 小号(150)；两桶/单桶沿用降级。ref 只用于档位标签与记录。
        ordered = sorted(buckets, key=lambda b: b["ratio"], reverse=True)
        if len(ordered) >= 3:
            mid = ordered[len(ordered) // 2]
            small = ordered[-1]
            reason_mid = "ratio_rank_median"
            reason_small = "ratio_rank_smallest"
            if mid["id"] == small["id"]:
                small = ordered[-2]
        elif len(ordered) == 2:
            mid, small = ordered[0], ordered[1]
            reason_mid = "ratio_rank_larger"
            reason_small = "ratio_rank_smallest"
            notes.append("只搜索到两个桶：直接飞向这两个桶依次投放（降级）")
        else:
            mid, small = ordered[0], None
            reason_mid = "ratio_rank_only"
            reason_small = ""
            notes.append("只搜索到一个桶：仅投放该桶（降级）")
        # 2026-10-07 实据：survey #1/#3 仅差 0.50m（#3 只有 3 样本），按比值选出的
        # 两投几乎同一个点 → A 投完 B 无独立目标可锁。约束"两投目标必须可区分"：
        # 过近时保留小号，换用剩余桶中距小号最远的（通常是最大桶）。
        if small is not None:
            def _pair_dist(a, b):
                return math.hypot(float(a["e"]) - float(b["e"]),
                                  float(a["n"]) - float(b["n"]))
            distinct_m = float(getattr(self, "distinct_target_distance_m", 0.6))
            if _pair_dist(mid, small) < distinct_m:
                others = [b for b in ordered
                          if b["id"] not in (mid["id"], small["id"])]
                if others:
                    far = max(others, key=lambda b: _pair_dist(b, small))
                    notes.append(
                        "选出的两个目标过近（%.2fm < %.2fm）：中号位改用距小号更远的"
                        " #%d" % (_pair_dist(mid, small),
                                  distinct_m, far["id"]))
                    mid = far
                    reason_mid = "distinct_swap"
        selected = [{"id": mid["id"], "size_class": mid["size_class"],
                     "reason": reason_mid}]
        if small is not None:
            selected.append({"id": small["id"], "size_class": small["size_class"],
                             "reason": reason_small})
            if mid["size_class"] == small["size_class"]:
                notes.append("两个目标档位标签不可辨（ref=%.3f）：仍按比值排序取"
                             "中号+小号" % ref)
        self._update_bucket_distances()
        for entry in selected:
            bucket = self._bucket_by_id(entry["id"])
            entry["distance_m"] = round(bucket["distance_m"], 4)
        selected.sort(key=lambda s: s["distance_m"])
        return selected, notes

    def _record_buckets(self):
        """原子写桶记录（§六.5 冻结 schema）。失败只 WARN，不影响飞行。"""
        if not self.record_buckets:
            return False
        try:
            self._update_bucket_distances()
            now = datetime.now().astimezone()
            payload = {
                "schema": 1,
                "generated_at": now.isoformat(timespec="seconds"),
                "mission": {
                    "pattern": self.area_search_pattern,
                    "search_altitude_m": round(float(self.search_altitude_m), 3),
                    "drop_zone_wgs84": ([self.drop_zone_wgs84[0], self.drop_zone_wgs84[1]]
                                        if self.drop_zone_wgs84 else []),
                    "recon_zone_wgs84": ([self.recon_zone_wgs84[0], self.recon_zone_wgs84[1]]
                                         if self.recon_zone_wgs84 else []),
                    "recon_enabled": bool(self.recon_enabled),
                },
                "buckets": [
                    {
                        "id": b["id"],
                        "lat": round(b["lat"], 9),
                        "lon": round(b["lon"], 9),
                        "alt": round(b["alt"], 4),
                        "ratio": round(b["ratio"], 6),
                        "size_class": b["size_class"],
                        "diameter_mm": b["diameter_mm"],
                        "samples": b["samples_count"],
                        "updated_at": b["updated_at"],
                        "distance_m": round(b["distance_m"], 4),
                    }
                    for b in self.buckets
                ],
                "selected": [dict(s) for s in self.selected_buckets],
                "drop_order": [s["id"] for s in self.selected_buckets],
                "notes": list(self.bucket_notes),
            }
            directory = self.bucket_record_dir
            os.makedirs(directory, exist_ok=True)
            stamp = now.strftime("%Y%m%d_%H%M%S")
            text = json.dumps(payload, ensure_ascii=False, indent=2)
            for name in ("buckets_%s.json" % stamp, "buckets_latest.json"):
                final_path = os.path.join(directory, name)
                tmp_path = final_path + ".tmp"
                with open(tmp_path, "w", encoding="utf-8") as handle:
                    handle.write(text)
                os.replace(tmp_path, final_path)
            self.node.get_logger().warn(
                "桶记录已原子写: %s/buckets_%s.json 与 buckets_latest.json"
                "（%d 桶，选中 %d）"
                % (directory, stamp, len(self.buckets), len(self.selected_buckets)))
            return True
        except Exception as exc:
            self.node.get_logger().error("写桶记录失败（不影响飞行）: %s" % exc)
            return False

    def bucket_survey(self):
        """投放区上方 W/Z/I 搜索并估计桶坐标（T2 核心）。

        与 area_search_for_target 不同：不在单个目标处停下，而是沿整个搜索航线
        连续消费 /vision/target_global 多目标，逐桶滑动中位数估计位置、抑制跳变。
        停止条件：稳定桶满 3 个 / 跑完一圈且有桶 / 超时。返回 True 表示找到足够桶
        （full: >=1；aim_only: >=3）。
        """
        if self.flight_only_mode:
            self.node.get_logger().error("FLIGHT ONLY 禁止进入桶搜索")
            return False
        if self.area_search_anchor is None:
            self.node.get_logger().error("桶搜索缺少已冻结的搜索中心")
            return False
        try:
            points = self._area_search_enu_points()
        except ValueError as exc:
            self.node.get_logger().error("桶搜索航线解算失败: %s" % exc)
            return False
        # 2026-09-30（用户拍板，#59）：full 模式也要求 3 个稳定桶才提前收；
        # 旧行为"凑够 2 个就停"可能只看到 large+small 而选错投放对。
        min_strong = 3
        self.buckets = []
        started_at = monotonic_s()
        point_index = 0
        point_inside_since = None
        next_setpoint_at = 0.0
        laps = 0
        stale_started_at = None
        stale_hold_enu = None
        reason = "timeout"
        self._transition("BUCKET_SURVEY",
                         "%s 字桶搜索，min_strong=%d" % (self.area_search_pattern, min_strong))
        self.node.get_logger().warn(
            "开始 %s 字桶搜索：高度 rel_alt+%.2fm 区域 %.1fx%.1fm 路径点 %d 个"
            % (self.area_search_pattern, self.search_altitude_m,
               self.drop_zone_width_m, self.drop_zone_length_m, len(points)))
        while rclpy.ok() and self.node.context.ok():
            now = monotonic_s()
            if not self._restart_dead_children():
                self.node.get_logger().error("桶搜索期间依赖重启失败")
                return False
            state = self._fresh_state()
            pose = self._fresh_pose()
            snapshot = self._control_stream_snapshot()
            stale_reasons = []
            fatal_reason = None
            if state is None:
                raw_state = snapshot["state"]
                grace = (self.connection_loss_grace_s
                         if raw_state is not None and not raw_state.connected
                         else self.guided_state_grace_s)
                if not (raw_state is not None and raw_state.armed
                        and raw_state.mode == "GUIDED"
                        and snapshot["state_age"] <= grace):
                    fatal_reason = (
                        "桶搜索 /mavros/state 恢复失败：mode=%s connected=%s armed=%s "
                        "age=%.3fs" % (raw_state.mode if raw_state else "NONE",
                                       raw_state.connected if raw_state else None,
                                       raw_state.armed if raw_state else None,
                                       snapshot["state_age"]))
                else:
                    stale_reasons.append("STATE age=%.3f" % snapshot["state_age"])
            elif not state.armed or state.mode != "GUIDED":
                fatal_reason = ("桶搜索安全状态失效：connected=%s armed=%s mode=%s"
                                % (state.connected, state.armed, state.mode))
            elif not state.connected:
                stale_reasons.append("CONNECTION")
            if pose is None:
                if snapshot["pose"] is None or snapshot["pose_age"] > self.guided_pose_grace_s:
                    fatal_reason = ("桶搜索本地位姿恢复失败：pose_age=%.3f"
                                    % snapshot["pose_age"])
                else:
                    stale_reasons.append("POSE age=%.3f" % snapshot["pose_age"])
            if fatal_reason is not None:
                self._log_control_diagnostics("BUCKET", "SURVEY", None, fatal_reason)
                return False
            if stale_reasons:
                if stale_started_at is None:
                    stale_started_at = now
                    self._begin_recovery("桶搜索: " + " + ".join(stale_reasons))
                    stale_hold_enu = self._safe_hold_enu(
                        snapshot, fallback=self.area_search_anchor[:3])
                    if stale_hold_enu is None:
                        self.node.get_logger().error("桶搜索遥测不新鲜且无可保持 ENU")
                        return False
                stale_grace = (self.connection_loss_grace_s
                               if any("CONNECTION" in r for r in stale_reasons)
                               else max(self.guided_state_grace_s, self.guided_pose_grace_s))
                if now - stale_started_at > stale_grace:
                    self.node.get_logger().error(
                        "桶搜索恢复超时 %.1f/%.1fs" % (now - stale_started_at, stale_grace))
                    return False
                point_inside_since = None
                if now >= next_setpoint_at:
                    self._publish_enu_setpoint(
                        stale_hold_enu, "BUCKET_SURVEY TELEMETRY_STALE 保持",
                        yaw=self.area_search_anchor[3])
                    next_setpoint_at = now + self.setpoint_interval_s
                self._publish_status(aiming=True)
                time.sleep(0.1)
                continue
            if stale_started_at is not None:
                started_at += now - stale_started_at
                self.node.get_logger().warn("桶搜索遥测恢复，继续原航线")
                stale_started_at = None
                stale_hold_enu = None
                self._end_recovery("桶搜索遥测恢复")
            self._observe_buckets()
            strong = [b for b in self.buckets
                      if b["samples_count"] >= max(1, int(getattr(
                          self, "bucket_min_samples", 3)))]
            if len(strong) >= min_strong:
                reason = "found"
                break
            elapsed = now - started_at
            if elapsed >= self.area_search_timeout_s:
                reason = "timeout"
                break
            if point_index >= len(points):
                laps += 1
                point_index = 0
                point_inside_since = None
                self.node.get_logger().warn(
                    "桶搜索第 %d 圈完成：稳定桶 %d 个" % (laps, len(strong)))
                if not self.aim_only_mode and len(strong) >= 1:
                    reason = "lap_done_with_target"
                    break
                if self.aim_only_mode and laps >= 3:
                    reason = "aim_only_laps"
                    break
            target_enu = points[point_index]
            if now >= next_setpoint_at:
                self._publish_enu_setpoint(
                    target_enu,
                    "BUCKET_SURVEY {} 点 {}/{} 稳定桶 {}".format(
                        self.area_search_pattern, point_index + 1, len(points), len(strong)),
                    yaw=self.area_search_anchor[3])
                next_setpoint_at = now + self.setpoint_interval_s
            position = pose.pose.position
            horizontal_error = math.hypot(target_enu[0] - float(position.x),
                                          target_enu[1] - float(position.y))
            vertical_error = abs(target_enu[2] - float(position.z))
            if (horizontal_error <= self.area_search_waypoint_tolerance_m
                    and vertical_error <= self.area_search_height_tolerance_m):
                if point_inside_since is None:
                    point_inside_since = now
                if now - point_inside_since >= self.area_search_dwell_s:
                    point_index += 1
                    point_inside_since = None
            else:
                point_inside_since = None
            throttle_log(
                self.node, 0.5,
                "BUCKET_SURVEY | 点=%d/%d 稳定桶=%d 平面误差=%.2fm 高度误差=%.2fm "
                "已用=%.1f/%.1fs",
                min(point_index + 1, len(points)), len(points), len(strong),
                horizontal_error, vertical_error, elapsed, self.area_search_timeout_s)
            self._publish_status(aiming=True)
            time.sleep(0.1)
        self._prune_buckets()
        self._update_bucket_distances()
        enough = (len(self.buckets) >= 3) if self.aim_only_mode else (len(self.buckets) >= 1)
        self.node.get_logger().warn(
            "桶搜索结束（%s）：稳定桶 %d 个 [%s]"
            % (reason, len(self.buckets),
               ", ".join("#%d/%s ratio=%.4f" % (b["id"], b["size_class"], b["ratio"])
                         for b in self.buckets) or "无"))
        if not enough:
            self.node.get_logger().error(
                "桶搜索目标不足：需要 %s，实际 %d"
                % ("3（仅瞄准）" if self.aim_only_mode else ">=1", len(self.buckets)))
        return enough

    def aim_dropper(self, dropper):
        if self.flight_only_mode:
            self.node.get_logger().error("FLIGHT ONLY 禁止进入 A/B 瞄准")
            return False
        prefix = dropper.lower()
        final_aligned_since = None
        self.frozen_target_ned = None
        self.forced_drop = False
        if self.ground_test:
            self.node.get_logger().warn(
                "开始 %s 抛投器地面瞄准：目标丢失或偏差超限只会清零计时，不会退出" % dropper)
            while rclpy.ok() and self.node.context.ok():
                aim = self._fresh_aim()
                if aim is None:
                    final_aligned_since = None
                    self._publish_status(aiming=True)
                    self.node.get_logger().warning(
                        "%s 地面瞄准目标无效：%s；连续计时已清零"
                        % (dropper, self._aim_invalid_reason()), throttle_duration_sec=1.0)
                    time.sleep(0.1)
                    continue
                now = monotonic_s()
                dx = float(getattr(aim, "{}_delta_x_m".format(prefix)))
                dy = float(getattr(aim, "{}_delta_y_m".format(prefix)))
                aligned = (abs(dx) < self.aim_cam_align_threshold_m
                           and abs(dy) < self.aim_cam_align_threshold_m)
                if aligned:
                    if final_aligned_since is None:
                        final_aligned_since = now
                    stable = now - final_aligned_since
                else:
                    final_aligned_since = None
                    stable = 0.0
                self._publish_status(aiming=True)
                throttle_log(self.node, 0.5,
                             "地面 %s 瞄准 | X偏差=%+.3fm Y偏差=%+.3fm 稳定=%.1f/%.1fs",
                             dropper, dx, dy, stable, self.stable_time_s)
                if aligned and stable >= self.stable_time_s:
                    self.node.get_logger().warn("%s 连续对准 %.1fs，瞄准完成" % (dropper, stable))
                    self._publish_status(aiming=False)
                    return True
                time.sleep(0.1)
            return False

        control_mode = "AUTONOMOUS" if self.autonomous_mode else "SEMI_AUTO"
        self.node.get_logger().warn(
            "开始 %s 分层瞄准 | control_mode=%s COARSE=%.2fm FINE=%.2fm DROP=%.2fm"
            % (dropper, control_mode, self.coarse_align_height_m, self.fine_align_height_m,
               self.target_drop_height_m))
        phase = "COARSE"
        self._active_dropper = str(dropper).upper()
        phase_aligned_since = None
        final_frozen = False
        locked_ned = None
        loss_started_at = None
        loss_hold_enu = None
        reacquire_valid_since = None
        relock_reset_done = False
        telemetry_stale_started_at = None
        telemetry_stale_hold_enu = None
        telemetry_stale_reason = None
        next_setpoint_at = 0.0
        target_lock_samples = deque()
        align_samples = deque()
        self._preferred_reanchor_since = None
        self._aim_guidance_ned = None
        self._next_guidance_at = 0.0
        aim_elapsed_s = 0.0
        aim_timer_started = False
        aim_active_last = None
        start_pose = self._fresh_pose()
        if start_pose is None:
            self.node.get_logger().error("%s 开始瞄准时没有本地位姿" % dropper)
            return False
        search_center_ned = (float(start_pose.pose.position.y),
                             float(start_pose.pose.position.x),
                             float(-start_pose.pose.position.z))
        if self.autonomous_mode:
            loss_started_at = monotonic_s()
            loss_hold_enu = (float(start_pose.pose.position.x),
                             float(start_pose.pose.position.y),
                             float(start_pose.pose.position.z))
            self.node.get_logger().warn(
                "%s 自主 ACQUIRE：先保持当前位置，目标连续有效 %.1fs 后开始移动"
                % (dropper, self.acquire_stable_time_s))
        # 丢失后的局部搜索应以"本槽选中的桶"为中心；旧实现 B 用 A 已投目标为
        # 中心（2026-10-04 181706 实据：B 丢失后围着 A 桶找），先改用 preferred。
        if self.preferred_target_ned is not None:
            search_center_ned = (float(self.preferred_target_ned[0]),
                                 float(self.preferred_target_ned[1]),
                                 float(self.preferred_target_ned[2]))
        elif dropper == "B" and self.completed_drop_targets:
            search_center_ned = self.completed_drop_targets[-1]
        while rclpy.ok() and self.node.context.ok():
            now = monotonic_s()
            if not self._restart_dead_children():
                self.node.get_logger().error("%s 瞄准期间依赖重启失败" % dropper)
                return False
            state = self._fresh_state()
            pose = self._fresh_pose()
            velocity = self._fresh_velocity()
            snapshot = self._control_stream_snapshot()
            stale_reasons = []
            fatal_reason = None
            if state is None:
                raw_state = snapshot["state"]
                raw_state_age = snapshot["state_age"]
                raw_mode = raw_state.mode if raw_state is not None else "NONE"
                state_grace = (self.connection_loss_grace_s
                               if raw_state is not None and not raw_state.connected
                               else self.guided_state_grace_s)
                if not (raw_state is not None and raw_state.armed
                        and raw_state.mode == "GUIDED" and raw_state_age <= state_grace):
                    fatal_reason = (
                        "/mavros/state 恢复失败：last_mode={} connected={} armed={} "
                        "age={:.3f}s timeout={:.3f}s grace={:.3f}s").format(
                            raw_mode,
                            raw_state.connected if raw_state is not None else None,
                            raw_state.armed if raw_state is not None else None,
                            raw_state_age, self.state_timeout_s, self.guided_state_grace_s)
                else:
                    stale_reasons.append("STATE age={:.3f}/{:.3f}s".format(
                        raw_state_age, self.guided_state_grace_s))
            elif state.mode != "GUIDED":
                fatal_reason = (
                    "收到新鲜状态但实际模式不是 GUIDED：mode={} connected={} armed={} "
                    "system_status={}").format(state.mode, state.connected, state.armed,
                                               state.system_status)
            elif not state.armed:
                fatal_reason = "瞄准期间 armed 变为 false"
            elif not state.connected:
                stale_reasons.append("CONNECTION connected=false")
            if pose is None:
                if (snapshot["pose"] is None
                        or snapshot["pose_age"] > self.guided_pose_grace_s):
                    fatal_reason = (
                        "本地位姿恢复失败：pose_age={:.3f}s timeout={:.3f}s grace={:.3f}s"
                        ).format(snapshot["pose_age"], self.pose_timeout_s,
                                 self.guided_pose_grace_s)
                else:
                    stale_reasons.append("POSE age={:.3f}/{:.3f}s".format(
                        snapshot["pose_age"], self.guided_pose_grace_s))
            if velocity is None:
                if (snapshot["velocity"] is None
                        or snapshot["velocity_age"] > self.guided_pose_grace_s):
                    fatal_reason = (
                        "本地速度恢复失败：velocity_age={:.3f}s timeout={:.3f}s grace={:.3f}s"
                        ).format(snapshot["velocity_age"], self.pose_timeout_s,
                                 self.guided_pose_grace_s)
                else:
                    stale_reasons.append("VEL age={:.3f}/{:.3f}s".format(
                        snapshot["velocity_age"], self.guided_pose_grace_s))
            if fatal_reason is not None:
                self._log_control_diagnostics(dropper, phase, locked_ned, fatal_reason)
                return False
            if stale_reasons:
                aim_active_last = None
                if telemetry_stale_started_at is None:
                    telemetry_stale_started_at = now
                    telemetry_stale_reason = " + ".join(stale_reasons)
                    self._begin_recovery("{} 瞄准: {}".format(dropper, telemetry_stale_reason))
                    telemetry_stale_hold_enu = self._safe_hold_enu(
                        snapshot, fallback=self._pose_position_enu(start_pose))
                    if telemetry_stale_hold_enu is None:
                        self._log_control_diagnostics(
                            dropper, phase, locked_ned,
                            "遥测短暂不新鲜，但没有可安全保持的 ENU setpoint")
                        return False
                stale_grace = (self.connection_loss_grace_s
                               if any("CONNECTION" in r for r in stale_reasons)
                               else max(self.guided_state_grace_s, self.guided_pose_grace_s))
                if now - telemetry_stale_started_at > stale_grace:
                    self._log_control_diagnostics(
                        dropper, phase, locked_ned,
                        "瞄准恢复超时 {:.1f}/{:.1f}s: {}".format(
                            now - telemetry_stale_started_at, stale_grace,
                            " + ".join(stale_reasons)))
                    return False
                phase_aligned_since = None
                final_aligned_since = None
                reacquire_valid_since = None
                if now >= next_setpoint_at:
                    self._publish_hold_setpoint(telemetry_stale_hold_enu,
                                                "{} TELEMETRY_STALE 保持".format(dropper))
                    next_setpoint_at = now + self.setpoint_interval_s
                self.node.get_logger().warning(
                    "%s TELEMETRY_STALE 恢复等待 | %s | 保持ENU=(%.3f, %.3f, %.3f)"
                    % (dropper, " + ".join(stale_reasons), telemetry_stale_hold_enu[0],
                       telemetry_stale_hold_enu[1], telemetry_stale_hold_enu[2]),
                    throttle_duration_sec=0.5)
                self._publish_status(aiming=True)
                self._log_control_diagnostics(dropper, phase, locked_ned)
                time.sleep(0.1)
                continue
            if telemetry_stale_started_at is not None:
                stale_duration = now - telemetry_stale_started_at
                if loss_started_at is not None:
                    loss_started_at += stale_duration
                self.node.get_logger().warn(
                    "%s 遥测已恢复，停滞 %.2fs；保持原阶段 %s，稳定计时重新开始"
                    % (dropper, stale_duration, phase))
                telemetry_stale_started_at = None
                telemetry_stale_hold_enu = None
                telemetry_stale_reason = None
                self._end_recovery("{} 瞄准遥测恢复，继续 {}".format(dropper, phase))
            aim = self._fresh_aim()
            measured_ned = self._aim_target_ned(aim, dropper)
            accepted = False
            loss_reason = self._aim_invalid_reason(dropper)
            if self.frozen_target_ned is not None:
                accepted = measured_ned is not None
                if accepted and self._target_is_blacklisted(measured_ned, dropper):
                    accepted = False
                    loss_reason = "当前观测命中已完成目标黑名单"
                elif (accepted and self._ned_xy_distance(measured_ned, self.frozen_target_ned)
                        > self.max_target_jump_m):
                    accepted = False
                    loss_reason = "冻结后观测与 frozen NED 距离过大，拒绝覆盖/误用"
                if accepted:
                    # #67：frozen 只用于防跳/黑名单/丢失回退；FINAL 首次对准前制导点
                    # 继续按实时观测 EMA 滤波更新。否则"冻结估计"与"实时相机系判据"
                    # 存在静态偏差，FINAL 3cm 永远无法满足（2026-10-02 实飞：Y 恒
                    # -0.08m → 40s 超时）。
                    # 2026-10-07：FINAL 首次对准后制导切换为 frozen 静态点（见下方
                    # final_frozen），lock 仅供防跳/回退/诊断，不再牵引制导。
                    locked_ned, _ = self._update_target_lock(locked_ned, measured_ned, dropper)
                elif locked_ned is None:
                    locked_ned = self.frozen_target_ned
            elif measured_ned is not None:
                update_base = locked_ned
                if (loss_started_at is not None
                        and now - loss_started_at >= self.loss_hover_s
                        and not relock_reset_done
                        and not self._target_is_blacklisted(measured_ned, dropper)):
                    update_base = None
                    relock_reset_done = True
                locked_ned, accepted = self._update_target_lock(update_base, measured_ned, dropper)
                if not accepted:
                    reacquire_valid_since = None
                    loss_reason = "目标处于黑名单或 NED 跳变被拒绝"
                elif loss_started_at is not None:
                    if reacquire_valid_since is None:
                        reacquire_valid_since = now
                    reacquire_stable = now - reacquire_valid_since
                    if reacquire_stable < self.acquire_stable_time_s:
                        accepted = False
                        loss_reason = "重捕获目标稳定 {:.1f}/{:.1f}s".format(
                            reacquire_stable, self.acquire_stable_time_s)
            else:
                reacquire_valid_since = None
            if not accepted:
                aim_active_last = None
                final_aligned_since = None
                phase_aligned_since = None
                align_samples.clear()
                if loss_started_at is None:
                    loss_started_at = now
                    relock_reset_done = False
                    loss_hold_enu = (float(pose.pose.position.x), float(pose.pose.position.y),
                                     float(pose.pose.position.z))
                    self.node.get_logger().warn(
                        "%s 目标暂时失效：%s；立即保持当前位置并进入重捕获流程"
                        % (dropper, loss_reason))
                loss_elapsed = now - loss_started_at
                self._aim_trace({
                    "wall": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    "dropper": dropper, "phase": phase, "valid": False,
                    "reason": loss_reason, "loss_s": round(loss_elapsed, 2),
                    "thr": round(self.coarse_align_threshold_m if phase == "COARSE"
                                 else (self.fine_align_threshold_m if phase == "FINE"
                                       else self.aim_cam_align_threshold_m), 4),
                    "count": int(getattr(aim, "count", 0)) if aim is not None else 0,
                    "frozen": self.frozen_target_ned is not None,
                    "locked_ned": ([round(float(v), 3) for v in locked_ned]
                                   if locked_ned is not None else None),
                    "preferred_ned": ([round(float(v), 3) for v in self.preferred_target_ned]
                                      if self.preferred_target_ned is not None else None),
                    "detections": self._detection_snapshot(),
                })
                if loss_elapsed >= self.target_loss_timeout_s:
                    self.node.get_logger().error(
                        "%s 搜索目标已持续 %.1fs，超过 %.1fs，自主任务超时"
                        % (dropper, loss_elapsed, self.target_loss_timeout_s))
                    return False
                if now >= next_setpoint_at:
                    if self.frozen_target_ned is not None:
                        if phase == "COARSE":
                            frozen_height = self.coarse_align_height_m
                        elif phase == "FINE":
                            frozen_height = self.fine_align_height_m
                        else:
                            frozen_height = self.target_drop_height_m
                        self._publish_ned_setpoint(
                            self.frozen_target_ned, frozen_height,
                            "{} 冻结目标丢失继续安全趋近".format(dropper))
                    elif loss_elapsed < self.loss_hover_s:
                        self._publish_hold_setpoint(
                            loss_hold_enu, "{} 目标丢失保持 {:.1f}s".format(dropper, loss_elapsed))
                    else:
                        center = locked_ned or search_center_ned
                        search_elapsed = loss_elapsed - self.loss_hover_s
                        search_ned = self._search_target_ned(center, search_elapsed)
                        self._publish_ned_setpoint(
                            search_ned, self.reacquire_height_m,
                            "{} 升高局部搜索 {:.1f}s".format(dropper, search_elapsed))
                    next_setpoint_at = now + self.setpoint_interval_s
                self.node.get_logger().warning(
                    "%s 重捕获 | 原因=%s 丢失=%.1f/%.1fs 阶段=%s"
                    % (dropper, loss_reason, loss_elapsed, self.target_loss_timeout_s,
                       "保持" if loss_elapsed < self.loss_hover_s else "升高搜索"),
                    throttle_duration_sec=1.0)
                self._publish_status(aiming=True)
                self._log_control_diagnostics(dropper, phase, locked_ned)
                time.sleep(0.1)
                continue
            if loss_started_at is not None:
                loss_duration = now - loss_started_at
                self.node.get_logger().warn(
                    "%s 目标重新捕获，丢失持续 %.2fs" % (dropper, loss_duration))
                if loss_duration >= self.brief_loss_s:
                    phase = "COARSE"
                    phase_aligned_since = None
                    self.node.get_logger().warn("%s 重捕获后回到 COARSE 阶段" % dropper)
                loss_started_at = None
                loss_hold_enu = None
                reacquire_valid_since = None
                relock_reset_done = False
                align_samples.clear()
                # 重捕获后制导点重新 snap 到新的锁定中心（不要在旧点慢爬）
                self._aim_guidance_ned = None
                self._next_guidance_at = 0.0
            search_center_ned = locked_ned
            if not aim_timer_started:
                aim_timer_started = True
                self.node.get_logger().warn(
                    "%s 获得合法目标并进入 COARSE，开始累计 %.1fs 瞄准超时"
                    % (dropper, self.aim_timeout_s))
            if aim_active_last is not None:
                aim_elapsed_s += max(0.0, now - aim_active_last)
            aim_active_last = now
            dx = float(getattr(aim, "{}_delta_x_m".format(prefix)))
            dy = float(getattr(aim, "{}_delta_y_m".format(prefix)))
            # 对准判据滑窗：单帧 dx/dy 噪声可达 ±0.1m（181016 实据），直接用单帧值
            # 判 3cm/0.5s 会永远不满足；窗口内取中位数再判（0=关闭，回落旧行为）。
            if self.align_filter_window_s > 0.0:
                align_samples.append((now, dx, dy))
                while (align_samples
                       and now - align_samples[0][0] > self.align_filter_window_s):
                    align_samples.popleft()
            else:
                align_samples.clear()
            if len(align_samples) >= 3:
                dx_filtered = float(median(sample[1] for sample in align_samples))
                dy_filtered = float(median(sample[2] for sample in align_samples))
            else:
                dx_filtered, dy_filtered = dx, dy
            if self.frozen_target_ned is None:
                if (abs(dx) < self.target_lock_center_threshold_m
                        and abs(dy) < self.target_lock_center_threshold_m
                        and measured_ned is not None):
                    target_lock_samples.append((now, measured_ned))
                else:
                    target_lock_samples.clear()
                while (target_lock_samples
                       and now - target_lock_samples[0][0] > self.target_lock_window_s):
                    target_lock_samples.popleft()
                if len(target_lock_samples) >= self.target_lock_min_samples:
                    ns = [sample[1][0] for sample in target_lock_samples]
                    es = [sample[1][1] for sample in target_lock_samples]
                    ds = [sample[1][2] for sample in target_lock_samples]
                    n_span = max(ns) - min(ns)
                    e_span = max(es) - min(es)
                    d_span = max(ds) - min(ds)
                    if (n_span <= self.target_lock_max_xy_span_m
                            and e_span <= self.target_lock_max_xy_span_m
                            and d_span <= self.target_lock_max_d_span_m):
                        frozen = (float(median(ns)), float(median(es)), float(median(ds)))
                        self.frozen_target_ned = frozen
                        locked_ned = frozen
                        self.node.get_logger().warn(
                            "%s TARGET LOCK | samples=%d span_N/E/D=(%.3f,%.3f,%.3f)m "
                            "raw=(%.3f,%.3f,%.3f) frozen=(%.3f,%.3f,%.3f)"
                            % (dropper, len(target_lock_samples), n_span, e_span, d_span,
                               measured_ned[0], measured_ned[1], measured_ned[2],
                               frozen[0], frozen[1], frozen[2]))
            current_z = float(pose.pose.position.z)
            surface_z = -float(locked_ned[2])
            actual_height = current_z - surface_z
            if phase == "COARSE":
                desired_height = self.coarse_align_height_m
                phase_threshold = self.coarse_align_threshold_m
            elif phase == "FINE":
                desired_height = self.fine_align_height_m
                phase_threshold = self.fine_align_threshold_m
            else:
                desired_height = self.target_drop_height_m
                # FINAL：用"相机系差值相对该投放器准星"的 3cm 触发（用户拍板）
                phase_threshold = self.aim_cam_align_threshold_m
            height_error = desired_height - actual_height
            if now >= next_setpoint_at:
                # 制导点 = 本投放器准星（前投 +off / 后投 -off）而非相机原点：
                # 与 _fill_aim_slot 的相机系判据同源，否则相机原点对桶时判据恒差
                # 一个 offset，FINAL 永远不会触发投放（T2 实据 2026-10-01 21:29）。
                # 带内保持（181016 实据）：滤波后偏差在阈值内时不再追单帧观测，
                # 保持上次 commanded 点让飞机停稳，破“追噪声”极限环。
                within_align_band = (
                    abs(dx_filtered) < phase_threshold
                    and abs(dy_filtered) < phase_threshold
                    and abs(height_error) < self.height_tolerance_m)
                hold_enu = self.last_setpoint_commanded_enu
                if within_align_band and hold_enu is not None:
                    self._publish_hold_setpoint(
                        hold_enu, "{} {} 对准带内保持".format(dropper, phase))
                else:
                    # 未冻结（COARSE 逼近）直接跟实时目标；FINAL 首次对准后直接用
                    # frozen NED 静态制导，不再跟随观测 EMA 漂移（2026-10-07 实据：
                    # EMA lock 被误检拖走 → 相机系 Y 恒偏 0.15m → 40s 拒投）。
                    # 未对齐前仍用慢速带死区跟踪，杜绝逐帧追测量导致的自激振荡。
                    if final_frozen and self.frozen_target_ned is not None:
                        guidance_ned = self.frozen_target_ned
                    elif self.frozen_target_ned is not None:
                        guidance_ned = self._slow_guidance_ned(now, locked_ned)
                    else:
                        guidance_ned = locked_ned
                    aim_setpoint_ned = self._aim_setpoint_ned(guidance_ned, dropper, pose)
                    self._publish_ned_setpoint(aim_setpoint_ned, desired_height,
                                               "{} {} 对准".format(dropper, phase))
                next_setpoint_at = now + self.setpoint_interval_s
            phase_aligned = (abs(dx_filtered) < phase_threshold
                             and abs(dy_filtered) < phase_threshold
                             and abs(height_error) < self.height_tolerance_m)
            if (phase == "FINAL" and phase_aligned and not final_frozen
                    and locked_ned is not None):
                # 对齐后制导直接用 frozen NED（2026-10-07 用户拍板）：首次进入
                # FINAL 对准带时，以当前滤波锁定点为 frozen 静态制导点；此后制导
                # 不再跟随观测 EMA 漂移，使相机系判据与制导同源。
                self.frozen_target_ned = (float(locked_ned[0]),
                                          float(locked_ned[1]),
                                          float(locked_ned[2]))
                final_frozen = True
                self.node.get_logger().warn(
                    "%s FINAL 首次对准：冻结目标 NED=(%.3f, %.3f, %.3f) 作为静态制导点"
                    % (dropper, self.frozen_target_ned[0],
                       self.frozen_target_ned[1], self.frozen_target_ned[2]))
            if phase_aligned:
                if phase_aligned_since is None:
                    phase_aligned_since = now
                phase_stable = now - phase_aligned_since
            else:
                phase_aligned_since = None
                phase_stable = 0.0
            velocity = self._fresh_velocity()
            horizontal_speed = float("inf")
            vertical_speed = float("inf")
            velocity_aligned = False
            if velocity is not None:
                horizontal_speed = math.hypot(float(velocity.twist.linear.x),
                                              float(velocity.twist.linear.y))
                vertical_speed = abs(float(velocity.twist.linear.z))
                velocity_aligned = (horizontal_speed < self.max_horizontal_speed_mps
                                    and vertical_speed < self.max_vertical_speed_mps)
            phase_velocity_aligned = (
                velocity is not None
                and horizontal_speed < self.phase_max_horizontal_speed_mps
                and vertical_speed < self.phase_max_vertical_speed_mps)
            if phase_aligned and not phase_velocity_aligned:
                phase_aligned_since = None
                phase_stable = 0.0
            if phase == "COARSE" and phase_stable >= self.phase_stable_time_s:
                phase = "FINE"
                phase_aligned_since = None
                self.node.get_logger().warn(
                    "%s COARSE 对准稳定 %.1fs，下降到 FINE 高度 %.2fm"
                    % (dropper, phase_stable, self.fine_align_height_m))
            elif phase == "FINE" and phase_stable >= self.phase_stable_time_s:
                phase = "FINAL"
                phase_aligned_since = None
                self.node.get_logger().warn(
                    "%s FINE 对准稳定 %.1fs，下降到最终投放高度 %.2fm"
                    % (dropper, phase_stable, self.target_drop_height_m))
            final_aligned = phase == "FINAL" and phase_aligned and velocity_aligned
            if final_aligned:
                if final_aligned_since is None:
                    final_aligned_since = now
                final_stable = now - final_aligned_since
            else:
                final_aligned_since = None
                final_stable = 0.0
            self._aim_trace({
                "wall": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "dropper": dropper, "phase": phase, "valid": True, "reason": "ok",
                "dx": round(dx, 4), "dy": round(dy, 4),
                "dxf": round(dx_filtered, 4), "dyf": round(dy_filtered, 4),
                "height": round(actual_height, 3), "h_err": round(height_error, 3),
                "hspeed": (round(horizontal_speed, 3)
                           if math.isfinite(horizontal_speed) else None),
                "vspeed": (round(vertical_speed, 3)
                           if math.isfinite(vertical_speed) else None),
                "aim_s": round(aim_elapsed_s, 2),
                # ---- 对准判断本身（实时记录，便于定论“为什么还没投”）----
                "thr": round(phase_threshold, 4),
                "h_tol": round(self.height_tolerance_m, 3),
                "ok_dx": bool(abs(dx_filtered) < phase_threshold),
                "ok_dy": bool(abs(dy_filtered) < phase_threshold),
                "ok_h": bool(abs(height_error) < self.height_tolerance_m),
                "phase_vel_ok": bool(phase_velocity_aligned),
                "vel_ok": bool(velocity_aligned),
                "h_lim": self.max_horizontal_speed_mps,
                "v_lim": self.max_vertical_speed_mps,
                "aligned": bool(phase_aligned),
                "phase_stable_s": round(phase_stable, 2),
                "final_ok": bool(final_aligned),
                "final_stable_s": round(final_stable, 2),
                "need_stable_s": self.stable_time_s,
                "frozen": self.frozen_target_ned is not None,
                "locked_ned": [round(float(v), 3) for v in locked_ned],
                "guide_ned": ([round(float(v), 3) for v in self._aim_guidance_ned]
                              if self._aim_guidance_ned is not None else None),
                "measured_ned": ([round(float(v), 3) for v in measured_ned]
                                 if measured_ned is not None else None),
                "preferred_ned": ([round(float(v), 3) for v in self.preferred_target_ned]
                                  if self.preferred_target_ned is not None else None),
                "detections": self._detection_snapshot(),
            })
            self._publish_status(aiming=True)
            throttle_log(
                self.node, 0.5,
                "%s %s | X=%+.3fm(Xf=%+.3fm) Y=%+.3fm(Yf=%+.3fm) 高度=%.2fm "
                "高度误差=%+.2fm 速度XY=%.2fm/s Z=%.2fm/s "
                "thr=%.3f ok=%d/%d/%d vel=%d 阶段稳定=%.1fs "
                "最终稳定=%.1f/%.1fs aim=%.1f/%.1fs frozen=%s",
                dropper, phase, dx, dx_filtered, dy, dy_filtered, actual_height,
                height_error, horizontal_speed, vertical_speed,
                phase_threshold,
                1 if abs(dx_filtered) < phase_threshold else 0,
                1 if abs(dy_filtered) < phase_threshold else 0,
                1 if abs(height_error) < self.height_tolerance_m else 0,
                1 if velocity_aligned else 0,
                phase_stable, final_stable, self.stable_time_s,
                aim_elapsed_s, self.aim_timeout_s, self.frozen_target_ned is not None)
            if aim_timer_started and aim_elapsed_s >= self.aim_timeout_s:
                hold = self._safe_hold_enu()
                if hold is None:
                    self.node.get_logger().error(
                        "%s 瞄准超时且没有安全保持 setpoint，拒绝投放" % dropper)
                    return False
                self._publish_hold_setpoint(hold, "{} FORCED_DROP 最后安全保持".format(dropper))
                self.node.get_logger().warn(
                    "%s 瞄准超时 %.2fs：保持安全位置并拒绝投放（原 FORCED_DROP 已禁用）"
                    % (dropper, aim_elapsed_s))
                self._publish_status(aiming=False)
                return False
            if final_aligned and final_stable >= self.stable_time_s:
                if self.frozen_target_ned is None:
                    self.frozen_target_ned = locked_ned
                    self.node.get_logger().warn(
                        "%s 正常投放门槛已满足；使用最后可信滤波 NED 作为冻结点" % dropper)
                self.last_aligned_target_ned = self.frozen_target_ned
                self.node.get_logger().warn(
                    "%s 最终位置、高度和速度连续稳定 %.1fs，瞄准完成" % (dropper, final_stable))
                self._publish_status(aiming=False)
                return True
            self._log_control_diagnostics(dropper, phase, locked_ned)
            time.sleep(0.1)
        return False

    # ==================================================================
    # 投放 / 收尾辅助（源 L3804-3971）
    # ==================================================================
    def _force_drop_safety_ok(self, dropper):
        state = self._fresh_state()
        if state is None or not state.connected or not state.armed or state.mode != "GUIDED":
            self.node.get_logger().error(
                "%s 强制投放被安全条件阻止: state=%s connected=%s armed=%s mode=%s"
                % (dropper, state is not None,
                   state.connected if state else None, state.armed if state else None,
                   state.mode if state else None))
            return False
        if (self.last_safe_setpoint_kind != "LOCAL"
                or self.last_setpoint_commanded_enu is None):
            self.node.get_logger().error(
                "%s 强制投放被阻止：没有最后安全本地保持 setpoint" % dropper)
            return False
        if self._fresh_pose() is None or self._fresh_velocity() is None:
            self.node.get_logger().error("%s 强制投放被阻止：pose/velocity 不新鲜" % dropper)
            return False
        if self.dry_run:
            return True
        with self._lock:
            ready = self.servo_ready
        if not ready or self.servo_cmd_pub.get_subscription_count() <= 0:
            self.node.get_logger().warn("%s 强制投放前 servo 不可用，尝试有界恢复" % dropper)
            if not self._restart_dead_children():
                return False
            return self._wait_for_servo_subscriber(timeout_s=self.servo_ready_timeout_s)
        return True

    def _wait_for_servo_subscriber(self, timeout_s=None):
        if self.flight_only_mode:
            self.node.get_logger().error("FLIGHT ONLY 禁止等待 /servo/cmd 订阅者")
            return False
        if self.dry_run:
            return True
        timeout_s = self.servo_wait_timeout_s if timeout_s is None else timeout_s
        deadline = monotonic_s() + timeout_s
        while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
            with self._lock:
                ready = self.servo_ready
            if ready and self.servo_cmd_pub.get_subscription_count() > 0:
                self.node.get_logger().info(
                    "servo_test.py 已完成初始化，/servo/cmd 控制链路就绪")
                return True
            throttle_log(self.node, 2.0, "等待 servo_test.py 完成初始化并发布 /servo/ready=true...")
            time.sleep(0.1)
        self.node.get_logger().error(
            "servo_test.py 未在 %.1fs 内就绪，拒绝开始投放测试" % timeout_s)
        return False

    def drop(self, dropper):
        if not self.drop_enabled:
            self.node.get_logger().error("FLIGHT ONLY 禁止发布任何 /servo/cmd 命令")
            return False
        if not self.ground_test:
            state = self._fresh_state()
            if (state is None or not state.connected or not state.armed
                    or state.mode != "GUIDED"):
                self.node.get_logger().error(
                    "%s 投放瞬间安全状态无效：connected=%s armed=%s mode=%s"
                    % (dropper, state.connected if state else None,
                       state.armed if state else None, state.mode if state else None))
                return False
        if self.dry_run:
            self.node.get_logger().warn(
                "dry_run=true：模拟投放 %s on（不发布 /servo/cmd，仅流程验证）" % dropper)
            if dropper == "A":
                self.ammo_a = 0
            else:
                self.ammo_b = 0
            # 干跑也在识别画面/终端显示 drop（发布 last_drop；识别节点经 /vision/mission_status 显示）
            for _ in range(3):
                self._publish_status(aiming=False, last_drop=dropper)
                time.sleep(0.05)
            self._publish_status(aiming=False)
            return True
        if self.servo_cmd_pub.get_subscription_count() <= 0:
            self.node.get_logger().error("投放瞬间 /servo/cmd 订阅者丢失，拒绝投放")
            return False
        command = "{} on".format(dropper)
        self.servo_cmd_pub.publish(String(data=command))
        if dropper == "A":
            self.ammo_a = 0
        else:
            self.ammo_b = 0
        self.node.get_logger().warn(
            "舵机命令已发布 /servo/cmd: %s | forced_drop=%s；当前接口没有 "
            "MAV_CMD_DO_SET_SERVO 执行回执，不能确认物理载荷已经释放"
            % (command, self.forced_drop))
        for _ in range(3):
            self._publish_status(aiming=False, last_drop=dropper)
            time.sleep(0.05)
        self._publish_status(aiming=False)
        if self.close_after_drop:
            time.sleep(max(0.0, self.close_delay_s))
            close_command = "{} off".format(dropper)
            self.servo_cmd_pub.publish(String(data=close_command))
            self.node.get_logger().warn(
                "close_after_drop=true，已发布 /servo/cmd: %s" % close_command)
        return True

    def _interruptible_delay(self, duration_s):
        deadline = monotonic_s() + max(0.0, duration_s)
        while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
            time.sleep(0.1)
        return rclpy.ok() and self.node.context.ok()

    def safe_loiter(self, reason):
        state = self._fresh_state()
        should_request = self.guided_was_requested or (
            state is not None and state.mode == "GUIDED")
        if not should_request:
            return True
        self.node.get_logger().warn("%s：尝试切回 LOITER，把控制权交还飞手" % reason)
        if self.set_mode_confirmed("LOITER"):
            return True
        self.node.get_logger().error("切回 LOITER 失败，请飞手立即用遥控器接管")
        return False

    def search_and_aim_dropper(self, dropper, require_confirmation, guided_already=False):
        if require_confirmation and not self.wait_for_area_search_confirm(dropper):
            self.node.get_logger().warn("操作者取消 %s 区域搜索" % dropper)
            return False
        if not guided_already and not self._validate_before_area_search(dropper):
            return False
        if not self._prepare_area_search_anchor(dropper):
            return False
        if not guided_already and not self.set_mode_confirmed("GUIDED"):
            self.node.get_logger().error("%s 区域搜索 GUIDED 切换或回读确认失败" % dropper)
            return False
        if not self.area_search_for_target(dropper):
            self.safe_loiter("{} 区域搜索未找到目标".format(dropper))
            return False
        return self.aim_dropper(dropper)

    def _stable_distinct_target_available(self, dropper, stable_s=None):
        stable_s = self.enroute_detect_stable_s if stable_s is None else stable_s
        deadline = monotonic_s() + stable_s
        while rclpy.ok() and self.node.context.ok() and monotonic_s() < deadline:
            aim = self._fresh_aim()
            target = self._aim_target_ned(aim, dropper)
            if target is None or self._target_is_blacklisted(target, dropper):
                return False
            time.sleep(0.1)
        aim = self._fresh_aim()
        target = self._aim_target_ned(aim, dropper)
        return target is not None and not self._target_is_blacklisted(target, dropper)

    def _mission_failure(self, reason):
        self._transition("FAILSAFE", reason)
        self.node.get_logger().error("任务级失败: %s" % reason)
        if self.airborne:
            state = self._fresh_state()
            if state is not None and not state.armed:
                return 1
            self.node.get_logger().error("飞机已离地，优先请求 RTL")
            if not self.set_mode_confirmed("RTL"):
                self.node.get_logger().error("RTL 请求失败，尝试 LAND")
                self.set_mode_confirmed("LAND")
        else:
            self.node.get_logger().warn("尚未确认离地，不无条件请求 RTL")
        return 1

    # ==================================================================
    # 顶层任务（源 L3973-4246）
    # ==================================================================
    def _run_flight_only_mission(self):
        self._transition("GUIDED_ARM_TAKEOFF", "操作者 Enter 授权 FLIGHT ONLY")
        if not self.set_mode_confirmed("GUIDED"):
            return self._mission_failure("FLIGHT ONLY GUIDED 切换失败")
        if not self.arm_confirmed():
            return self._mission_failure("FLIGHT ONLY 自动解锁失败")
        if not self.capture_ground_relative_altitude():
            return self._mission_failure("FLIGHT ONLY 地面 rel_alt 基准记录失败")
        if not self.command_takeoff_and_wait():
            return self._mission_failure("FLIGHT ONLY 起飞命令重试后仍未开始或未到高度")
        drop_target = self._make_global_target(
            self.drop_zone_wgs84[0], self.drop_zone_wgs84[1],
            self.drop_zone_transit_altitude_m)
        self._transition("GOTO_DROP_ZONE", "起飞高度已到达；视觉监控保持禁用")
        goto_result = self.fly_global_waypoint(
            drop_target, "GOTO_DROP_ZONE", detect_dropper=None, require_full_telemetry=True)
        if goto_result != "arrived":
            return self._mission_failure("FLIGHT ONLY 飞往投放区失败")
        self.drop_zone_reached = True
        self._transition("FLIGHT_ONLY_PATTERN",
                         "抵达投放区；停止全局 setpoint，切换本地保持和完整区域航线")
        if not self.fly_area_pattern_only():
            return self._mission_failure("FLIGHT ONLY 区域航线执行失败")
        if not self.flight_only_continue_to_recon:
            self._transition("RTL",
                             "FLIGHT ONLY 区域航线完成且 flight_only_continue_to_recon=false")
            if not self.request_rtl_and_wait():
                return self._mission_failure("FLIGHT ONLY RTL 请求/确认/等待失败")
            self._transition("COMPLETE", "FLIGHT ONLY 投放区航线和 RTL 完成")
            self.node.get_logger().warn("FLIGHT ONLY 任务完成；全程未启用视觉、瞄准或投放")
            return 0
        recon_target = self._make_global_target(
            self.recon_zone_wgs84[0], self.recon_zone_wgs84[1], self.recon_altitude_m)
        self._transition("GOTO_RECON_CLIMB",
                         "FLIGHT ONLY 区域航线完成；停止本地 setpoint，前往侦察区并爬升")
        if self.fly_global_waypoint(
                recon_target, "GOTO_RECON_CLIMB", detect_dropper=None,
                require_full_telemetry=True) != "arrived":
            return self._mission_failure("FLIGHT ONLY 飞往侦察区失败")
        self._transition("RECON_HOLD", "FLIGHT ONLY 已抵达侦察区")
        if not self.hold_global_target(recon_target, self.recon_hold_s, "RECON_HOLD",
                                       require_full_telemetry=True):
            return self._mission_failure("FLIGHT ONLY 侦察区保持失败")
        self._transition("RTL", "FLIGHT ONLY 侦察保持完成")
        if not self.request_rtl_and_wait():
            return self._mission_failure("FLIGHT ONLY RTL 请求/确认/等待失败")
        self._transition("COMPLETE", "FLIGHT ONLY 完整航线完成")
        self.node.get_logger().warn("FLIGHT ONLY 任务完成；全程未启用视觉、瞄准或投放")
        return 0

    def _run_ground_test(self):
        self._transition("A_AIM", "ground_test 等待 A 目标")
        if not self.wait_for_target_and_confirm("A") or not self.aim_dropper("A"):
            return self._mission_failure("ground_test A 瞄准失败")
        self._transition("A_DROP", "ground_test A 门槛满足")
        if not self.drop("A"):
            return self._mission_failure("ground_test A 命令发布失败")
        if self.enable_b_dropper:
            self._transition("B_AIM", "ground_test 继续 B")
            if not self.aim_dropper("B"):
                return self._mission_failure("ground_test B 瞄准失败")
            self._transition("B_DROP", "ground_test B 门槛满足")
            if not self.drop("B"):
                return self._mission_failure("ground_test B 命令发布失败")
        self._transition("COMPLETE", "ground_test 完成")
        return 0

    def run(self):
        if self.flight_only_mode:
            self.node.get_logger().warn("================ FLIGHT ONLY MODE ================")
            self.node.get_logger().warn(
                "vision=DISABLED servo=DISABLED aiming=DISABLED drop=DISABLED")
            self.node.get_logger().warn(
                "CUADC FLIGHT ONLY START | pattern=%s laps=%d drop=%.1fm recon=%.1fm "
                "takeoff=%.1fm drop_transit_alt=%.1fm pattern_alt=%.1fm recon_alt=%.1fm "
                "continue_recon=%s"
                % (self.area_search_pattern, self.flight_only_pattern_laps,
                   self.drop_zone_distance_m, self.recon_zone_distance_m,
                   self.takeoff_altitude_m, self.drop_zone_transit_altitude_m,
                   self.flight_only_search_altitude_m, self.recon_altitude_m,
                   self.flight_only_continue_to_recon))
        else:
            self.node.get_logger().warn(
                "CUADC FULL MISSION START | full_mission=%s ground_test=%s pattern=%s "
                "drop=%.1fm recon=%.1fm takeoff=%.1fm drop_transit_alt=%.1fm recon_alt=%.1fm "
                "A/B=%s dry_run=%s"
                % (self.full_mission_mode, self.ground_test, self.area_search_pattern,
                   self.drop_zone_distance_m, self.recon_zone_distance_m,
                   self.takeoff_altitude_m, self.drop_zone_transit_altitude_m,
                   self.recon_altitude_m, self.enable_b_dropper, self.dry_run))
        self._transition("INIT_DEPENDENCIES", "节点初始化完成")
        if not self.start_dependencies():
            return self._mission_failure("依赖启动失败")
        self._transition("WAIT_MAVROS", "依赖启动命令已发出")
        if not self.wait_for_mavros():
            return self._mission_failure("MAVROS/飞控连接超时")
        self.request_local_position_stream()
        if not self._wait_for_dependencies_ready():
            return self._mission_failure("运行依赖未就绪")
        self._transition("WAIT_NAVIGATION", "MAVROS 与依赖已就绪")
        if not self.wait_for_navigation():
            return self._mission_failure("飞行前导航数据不完整")
        self._transition("PLAN_WAYPOINTS", "导航数据新鲜")
        if not self.plan_waypoints():
            return self._mission_failure("航点解算失败")
        self.print_mission_summary()
        self._transition("WAIT_ENTER", "航点与任务摘要已冻结")
        if not self.wait_for_enter_authorization():
            self.node.get_logger().warn("操作者未授权；未切模式、未解锁、未起飞")
            return 2
        if self.flight_only_mode:
            if not self._navigation_is_fresh():
                if not self.wait_for_navigation():
                    return self._mission_failure("Enter 后导航数据无法恢复")
            if not self._ensure_flight_services():
                return self._mission_failure("MAVROS 飞行服务不可用")
            return self._run_flight_only_mission()
        if self.ground_test:
            return self._run_ground_test()
        if not self.full_mission_mode:
            self._transition("COMPLETE",
                             "full_mission_mode=false：仅完成依赖/航点/授权检查，不执行飞行")
            self.node.get_logger().warn("full_mission_mode=false：未切模式、未解锁、未起飞")
            return 0
        if not self._navigation_is_fresh():
            if not self.wait_for_navigation():
                return self._mission_failure("Enter 后导航数据无法恢复")
        if not self._ensure_flight_services():
            return self._mission_failure("MAVROS 飞行服务不可用")
        self._transition("GUIDED_ARM_TAKEOFF", "操作者 Enter 授权")
        if not self.set_mode_confirmed("GUIDED"):
            return self._mission_failure("GUIDED 切换失败")
        if not self.arm_confirmed():
            return self._mission_failure("自动解锁失败")
        if not self.capture_ground_relative_altitude():
            return self._mission_failure("地面 rel_alt 基准记录失败")
        if not self.command_takeoff_and_wait():
            return self._mission_failure("起飞命令重试后仍未开始或未到高度")
        # ---- T2：在搜索高度飞往投放区，然后做桶搜索+估计 ----
        drop_target = self._make_global_target(
            self.drop_zone_wgs84[0], self.drop_zone_wgs84[1],
            self.search_altitude_m)
        self._transition("GOTO_DROP_ZONE", "起飞高度已到达；在搜索高度飞往投放区")
        goto_result = self.fly_global_waypoint(
            drop_target, "GOTO_DROP_ZONE", detect_dropper=None)
        if goto_result == "failed":
            return self._mission_failure("飞往投放区失败")
        if goto_result != "arrived":
            return self._mission_failure("飞往投放区未到达（%s）" % goto_result)
        self.drop_zone_reached = True
        if not self.set_area_search_anchor_from_current("BUCKET"):
            return self._mission_failure("无法冻结投放区搜索中心")
        # 投放区上方连续搜索，累积多目标并估计桶坐标（不因单目标停下）
        survey_ok = self.bucket_survey()
        self.selected_buckets, self.bucket_notes = self._select_buckets()
        self._record_buckets()
        if not survey_ok:
            return self._mission_failure("投放区桶搜索超时/目标不足")

        # 仅搜索记录：视觉搜索 + 记录桶坐标后直接返航（不瞄准、不投放）。
        # 对齐"仅飞行模式也要开视觉识别"：只跑飞行+搜索+记录，不发任何 /servo/cmd。
        if self.survey_only_mode:
            self._transition("RTL", "survey_only_mode：已搜索并记录桶坐标，返航（未瞄准/未投放）")
            if not self.request_rtl_and_wait():
                return self._mission_failure("survey_only_mode RTL 请求/确认/等待失败")
            self._transition("COMPLETE", "survey_only_mode 完成：已搜索并记录，未瞄准未投放")
            self.node.get_logger().warn(
                "survey_only_mode 完成：仅搜索记录，未瞄准、未发布任何 /servo/cmd 投放命令")
            return 0

        # 仅飞行+仅瞄准：识别到 3 个桶后切入瞄准（不投放）；瞄准完成后返航
        # （对齐 P0 测试流程 5"仅做识别、记录桶坐标，然后返航"；不发布任何投放命令）。
        if self.aim_only_mode:
            self._transition("A_AIM", "aim_only_mode：已识别到 >=3 个桶，切入瞄准（不投放）")
            if not self.aim_dropper("A"):
                return self._mission_failure("aim_only_mode A 分层瞄准失败")
            self._transition("RTL", "aim_only_mode 瞄准完成，返航（未投放）")
            if not self.request_rtl_and_wait():
                return self._mission_failure("aim_only_mode RTL 请求/确认/等待失败")
            self._transition("COMPLETE", "aim_only_mode 完成：已瞄准并返航，未投放")
            self.node.get_logger().warn(
                "aim_only_mode 完成：仅瞄准，未发布任何 /servo/cmd 投放命令")
            return 0

        if not self.selected_buckets:
            return self._mission_failure("未选出可投放的桶")
        for entry in self.selected_buckets:
            bucket = self._bucket_by_id(entry["id"])
            if bucket is None:
                continue
            self.node.get_logger().warn(
                "选桶 #%d | class=%s ratio=%.4f diameter=%dmm samples=%d "
                "distance=%.2fm reason=%s"
                % (bucket["id"], bucket["size_class"], bucket["ratio"],
                   bucket["diameter_mm"], bucket["samples_count"],
                   bucket["distance_m"], entry.get("reason", "")))

        # 依选桶结果先近后远投放（A=最近，B=次近）；瞄准锁定"本槽选中的桶"
        # （preferred_target_ned），避免"最高置信度"落在未选中的桶上。
        drop_letters = ["A", "B"] if self.enable_b_dropper else ["A"]
        self._transition("BUCKET_DROP", "选桶完成，按先近后远依次投放")
        for index, entry in enumerate(self.selected_buckets):
            if index >= len(drop_letters):
                self.node.get_logger().warn(
                    "仅 %d 个投放通道，跳过剩余选桶 %s"
                    % (len(drop_letters), self.selected_buckets[index:]))
                break
            bucket = self._bucket_by_id(entry["id"])
            if bucket is None:
                continue
            letter = drop_letters[index]
            self.preferred_target_ned = (bucket["n"], bucket["e"], -bucket["u"])
            self._preferred_reanchor_since = None
            self._transition(
                "A_AIM" if letter == "A" else "B_AIM",
                "飞向桶#%d(%s, ratio=%.4f, dist=%.2fm)并瞄准"
                % (bucket["id"], bucket["size_class"], bucket["ratio"],
                   bucket["distance_m"]))
            if letter != "A" or bucket["distance_m"] > self.waypoint_tolerance_m * 3.0:
                approach = self._make_global_target(
                    bucket["lat"], bucket["lon"], self.search_altitude_m)
                if self.fly_global_waypoint(
                        approach, "GOTO_BUCKET_%s" % letter) != "arrived":
                    return self._mission_failure(
                        "飞往桶#%d(%s)失败" % (bucket["id"], letter))
            if not self.aim_dropper(letter):
                return self._mission_failure(
                    "%s 桶#%d 分层瞄准失败" % (letter, bucket["id"]))
            self._transition("%s_DROP" % letter,
                             "桶#%d(%s) 瞄准完成" % (bucket["id"], bucket["size_class"]))
            if not self.drop(letter):
                return self._mission_failure("%s 舵机命令发布失败" % letter)
            if self.last_aligned_target_ned is not None:
                self.completed_drop_targets.append(self.last_aligned_target_ned)
            if index + 1 < len(self.selected_buckets) and index + 1 < len(drop_letters):
                if not self._interruptible_delay(self.after_a_delay_s):
                    return self._mission_failure("A/B 间隔被中断")

        # 侦察区（可选）或直接 RTL
        self.preferred_target_ned = None
        self._preferred_reanchor_since = None
        if self.recon_enabled:
            recon_target = self._make_global_target(
                self.recon_zone_wgs84[0], self.recon_zone_wgs84[1], self.recon_altitude_m)
            self._transition("GOTO_RECON_CLIMB", "投放完成，前往预设侦察区并爬升")
            if self.fly_global_waypoint(recon_target, "GOTO_RECON_CLIMB") != "arrived":
                return self._mission_failure("飞往侦察区失败")
            self._transition("RECON_HOLD", "已抵达侦察区")
            if not self.hold_global_target(recon_target, self.recon_hold_s, "RECON_HOLD"):
                return self._mission_failure("侦察区保持失败")
            self._transition("RTL", "侦察保持完成")
        else:
            self._transition("RTL", "投放完成且 recon_enabled=false，直接返航")
        if not self.request_rtl_and_wait():
            return self._mission_failure("RTL 请求/确认/等待失败")
        self._transition("COMPLETE", "完整任务完成（搜索+选桶+投放+返航）")
        self.node.get_logger().warn(
            "CUADC 全自动任务完成；A/B 只确认舵机命令发布，不代表物理释放回执")
        return 0

    # ==================================================================
    # 收尾（源 L4248-4316）
    # ==================================================================
    def _stop_children(self):
        if self._cleanup_done:
            return
        self._cleanup_done = True
        if not self.auto_shutdown_children:
            self.node.get_logger().warn(
                "auto_shutdown_children=false：保留本脚本拉起的子进程（不自动收尾）")
            return
        for label, process in reversed(self._children):
            if process.poll() is not None:
                continue
            if label == "MAVROS" and self.airborne and self.top_state != "COMPLETE":
                self.node.get_logger().error(
                    "飞机仍在空中，保留本脚本自动启动的 MAVROS，避免 Ctrl+C 同时切断飞控链路")
                continue
            self.node.get_logger().info("停止本脚本自动启动的 %s" % label)
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                process.wait(timeout=5.0)
            except (OSError, subprocess.SubprocessError):
                try:
                    process.terminate()
                except OSError:
                    pass

    def _on_shutdown(self):
        try:
            if self.airborne and self.top_state not in ("RTL", "COMPLETE", "FAILSAFE"):
                with self._lock:
                    state = self.current_state
                    state_age = (monotonic_s() - self.state_received_at
                                 if state is not None else float("inf"))
                if (state is not None and state_age <= self.guided_state_grace_s
                        and state.connected and state.armed and state.mode == "GUIDED"):
                    try:
                        if self._set_mode_srv is None:
                            self._set_mode_srv = self.node.create_client(
                                SetMode, "/mavros/set_mode")
                        req = SetMode.Request()
                        req.base_mode = 0
                        req.custom_mode = "RTL"
                        ok, response = self._call_service(self._set_mode_srv, req, 5.0)
                        self.node.get_logger().error(
                            "ROS shutdown/Ctrl+C 空中保护：已请求 RTL，mode_sent=%s；"
                            "请飞手持续监控并准备接管"
                            % (getattr(response, "mode_sent", None) if ok else None))
                    except Exception as exc:
                        self.node.get_logger().error(
                            "ROS shutdown/Ctrl+C 空中 RTL 请求失败: %s；请飞手立即接管" % exc)
                else:
                    self.node.get_logger().error(
                        "ROS shutdown/Ctrl+C：未覆盖当前飞手模式/陈旧状态；停止任务 setpoint，"
                        "请飞手立即接管 | mode=%s armed=%s connected=%s state_age=%.2fs"
                        % (state.mode if state is not None else "NONE",
                           state.armed if state is not None else None,
                           state.connected if state is not None else None, state_age))
        except Exception as exc:
            self.node.get_logger().error("shutdown 安全处理异常: %s" % exc)
        self._stop_children()


# ======================================================================
# 每轮任务独立日志目录：logs/runs/<YYYYmmdd_HHMMSS>/
#   mission.log（fd 级 tee，含 rclpy C 层日志）、detector.log/camera.log/...
#   （本脚本拉起的子进程）、config.yaml、run_info.txt、buckets_latest.json
# ======================================================================
RUN_ROOT = os.path.join(os.path.expanduser("~/ROS2_CUADC2026"), "logs", "runs")
RUN_DIR = None          # 本轮日志目录（main 启动时创建）


def _install_run_logging():
    """建立本轮日志目录，并把 stdout/stderr 做 fd 级 tee 到 mission.log。"""
    global RUN_DIR
    run_dir = os.path.join(RUN_ROOT, datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(run_dir, exist_ok=True)
    latest = os.path.join(RUN_ROOT, "latest")
    try:
        if os.path.islink(latest) or os.path.exists(latest):
            os.unlink(latest)
        os.symlink(run_dir, latest)
    except OSError:
        pass
    try:
        r, w = os.pipe()
        orig = os.dup(1)
        logfh = open(os.path.join(run_dir, "mission.log"), "ab", buffering=0)

        def _pump():
            with os.fdopen(r, "rb", buffering=0) as rf:
                for chunk in iter(lambda: rf.read(4096), b""):
                    try:
                        os.write(orig, chunk)
                    except OSError:
                        pass
                    try:
                        logfh.write(chunk)
                    except Exception:
                        pass

        # 退出时还原 fd1/fd2（关闭管道写端）并等 pump 排空（有界，绝不挂死）
        th = threading.Thread(target=_pump, daemon=True, name="run_log_tee")

        def _restore():
            try:
                os.dup2(orig, 1)
                os.dup2(orig, 2)
            except OSError:
                pass
            try:
                th.join(timeout=1.0)
            except Exception:
                pass

        atexit.register(_restore)
        th.start()
        os.dup2(w, 1)
        os.dup2(w, 2)
        os.close(w)
    except Exception:
        pass
    RUN_DIR = run_dir
    return run_dir


def _copy_tail(src, dst, max_bytes=2 * 1024 * 1024):
    """把外部日志尾部 max_bytes 拷进本轮目录（尽力而为，不抛）。"""
    try:
        if not os.path.isfile(src):
            return
        size = os.path.getsize(src)
        with open(src, "rb") as fh:
            if size > max_bytes:
                fh.seek(size - max_bytes)
            data = fh.read()
        with open(dst, "wb") as out:
            out.write(data)
    except OSError:
        pass


def main():
    # YAML 配置入口（§3.B）：--config > CUADC_AUTO_DROP_CONFIG > 默认路径 > 内置默认。
    # 先摘除 --config 再 rclpy.init，避免非 ROS 参数被 rcl 解析；赋值到 sys.argv 供
    # rclpy.init() 使用。CLI `--ros-args -p` 由 rclpy 预先声明，优先级最高。
    cli_config = None
    if auto_drop_config is not None:
        cli_config, sys.argv = auto_drop_config.extract_config_arg(list(sys.argv))
    config_path = (auto_drop_config.resolve_config_path(cli_config)
                   if auto_drop_config is not None else None)

    # #29（两半）：入口堵 SIGINT/SIGTERM（其后线程继承掩码）→ 信号必投主线程；
    # init 后夺回 Python 级处理器 → SIGINT 由主线程持 GIL 跑 rclpy.try_shutdown()，
    # 杜绝 rclpy C 层 handler 在信号上下文 rcl_shutdown 与收尾并行的 FastDDS
    # std::terminate（rc=-6 竞态）。放开点在建齐 spin 线程之后。
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM})
    rclpy.init()


    def _py_shutdown_handler(signum, frame):
        if rclpy.ok():
            rclpy.try_shutdown()
        else:
            signal.signal(signum, signal.SIG_DFL)
            signal.raise_signal(signum)

    signal.signal(signal.SIGINT, _py_shutdown_handler)
    signal.signal(signal.SIGTERM, _py_shutdown_handler)
    node = rclpy.create_node("auto_drop_test")

    if auto_drop_config is not None:
        try:
            path, values, warns, applied, overridden = auto_drop_config.apply_to_node(
                node, config_path)
            node.get_logger().warn(
                "配置来源: %s（注入 %d 项，被命令行 -p 覆盖 %d 项）"
                % (path, applied, overridden))
            for warning in warns:
                node.get_logger().warn("配置警告: %s" % warning)
        except Exception as exc:
            node.get_logger().warn("读取配置文件失败，改用内置默认: %s" % exc)
    else:
        node.get_logger().warn("auto_drop_config 模块不可用，改用内置默认")

    # ---- 本轮独立日志目录（此后 stdout/stderr 同时进 logs/runs/<ts>/mission.log）----
    run_dir = _install_run_logging()
    node.get_logger().warn("本轮任务日志目录: %s" % run_dir)
    try:
        if config_path and os.path.isfile(config_path):
            shutil.copy2(config_path, os.path.join(run_dir, "config.yaml"))
    except OSError:
        pass
    try:
        with open(os.path.join(run_dir, "run_info.txt"), "w", encoding="utf-8") as fh:
            fh.write("start=%s\nconfig=%s\npid=%d\n"
                     % (datetime.now().isoformat(), config_path or "", os.getpid()))
    except OSError:
        pass

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _guarded_spin():
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception:
            if rclpy.ok():
                raise

    spin_thread = threading.Thread(target=_guarded_spin, daemon=True)
    spin_thread.start()
    signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT, signal.SIGTERM})

    exit_code = 2
    mission = None
    try:
        try:
            mission = AutoDropTest(node)
        except ValueError as exc:
            node.get_logger().fatal("参数错误: %s" % exc)
            exit_code = 2
        else:
            # 服务调用共用同一执行器，避免自建 executor 冲突（D1 同款）。
            exit_code = mission.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        # #28：rclpy C 层 SIGINT = rcl_shutdown + PyErr 挂起位 → 投递线程随机，
        # 落在主线程阻塞点以 KeyboardInterrupt 抛出；收口为本节点中断语义 rc=1。
        if mission is not None and mission.flight_only_mode:
            node.get_logger().warn("收到 Ctrl+C，取消 FLIGHT ONLY 航线验证")
        else:
            node.get_logger().warn("收到 Ctrl+C，取消 W/Z/I 搜索投放")
        exit_code = 1
    except Exception as exc:
        mission_name = ("FLIGHT ONLY 航线任务"
                        if mission is not None and mission.flight_only_mode
                        else "全自动任务")
        node.get_logger().error("%s异常: %s" % (mission_name, exc))
        if mission is not None:
            try:
                exit_code = mission._mission_failure("未处理异常: {}".format(exc))
            except Exception:
                exit_code = 1
    finally:
        if mission is not None:
            try:
                mission._on_shutdown()
            except Exception:
                pass
        # ---- 本轮日志收尾：写结束信息、拷桶记录、外部节点日志尾部 ----
        if RUN_DIR:
            try:
                with open(os.path.join(RUN_DIR, "run_info.txt"), "a",
                          encoding="utf-8") as fh:
                    fh.write("end=%s\nexit_code=%s\n"
                             % (datetime.now().isoformat(), exit_code))
            except OSError:
                pass
            _copy_tail(os.path.join(os.path.expanduser("~/ROS2_CUADC2026"),
                                    "logs", "buckets", "buckets_latest.json"),
                       os.path.join(RUN_DIR, "buckets_latest.json"))
            # 外部（WebUI 按钮）拉起的识别/探针日志：尾部快照进本轮目录
            for src_name, dst_name in (("webui_detector.log", "external_detector.log"),
                                       ("webui_ros_probe.log", "external_ros_probe.log"),
                                       ("webui_mavros_real.log", "external_mavros_real.log"),
                                       ("webui_mavros_sim.log", "external_mavros_sim.log")):
                _copy_tail(os.path.join("/tmp/opencode", src_name),
                           os.path.join(RUN_DIR, dst_name))
            try:
                os.makedirs("/tmp/opencode", exist_ok=True)
                with open("/tmp/opencode/webui_run_dir", "w", encoding="utf-8") as fh:
                    fh.write(RUN_DIR)
            except OSError:
                pass
            try:
                node.get_logger().warn("本轮日志已归档: %s" % RUN_DIR)
            except Exception:
                pass
        executor.shutdown(3.0)  # 有界兜底：shutdown() 默认无超时会 block forever
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
