#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CUADC 全自动 WGS84 航点、W/Z/I 搜索、A/B 瞄准投放与侦察 RTL 控制器。

本节点独占整场任务的 setpoint 发布职责。全局航点、本地保持、区域搜索和分层瞄准
均由一个显式顶层状态机串行调用；任一时刻只会发布一种 setpoint。操作者在航点和
任务摘要打印后只按一次 Enter，Enter 前不会切 GUIDED、解锁、起飞或动作舵机。
"""

import math
import os
import select
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from statistics import median

import rosnode
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, GlobalPositionTarget, RCIn, State, StatusText
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, ParamGet, SetMode
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64, String

from cuadc_vision.msg import BucketAimInfo, MissionStatus


def monotonic_s():
    return time.monotonic()


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class AutoDropTest:
    WGS84_A = 6378137.0
    WGS84_F = 1.0 / 298.257223563

    @staticmethod
    def _normalize_area_search_pattern(value):
        pattern = str(value).strip().upper()
        return "I" if pattern == "LINE" else pattern

    def __init__(self):
        self.flight_only_mode = get_bool_param("~flight_only_mode", False)
        self.flight_only_pattern_laps = int(
            rospy.get_param("~flight_only_pattern_laps", 1)
        )
        self.flight_only_search_altitude_m = float(
            rospy.get_param("~flight_only_search_altitude_m", 3.0)
        )
        self.flight_only_continue_to_recon = get_bool_param(
            "~flight_only_continue_to_recon", True
        )
        self.ground_test = get_bool_param("~ground_test", False)
        self.full_mission_mode = get_bool_param("~full_mission_mode", True)
        self.autonomous_mode = self.full_mission_mode
        self.enable_b_dropper = get_bool_param("~enable_b_dropper", True)
        self.takeoff_altitude_m = float(rospy.get_param("~takeoff_altitude_m", 3.0))
        self.drop_zone_transit_altitude_m = float(
            rospy.get_param(
                "~drop_zone_transit_altitude_m", self.takeoff_altitude_m
            )
        )
        self.drop_zone_distance_m = float(rospy.get_param("~drop_zone_distance_m", 32.0))
        self.recon_zone_distance_m = float(rospy.get_param("~recon_zone_distance_m", 60.0))
        self.recon_altitude_m = float(rospy.get_param("~recon_altitude_m", 5.0))
        self.recon_hold_s = float(rospy.get_param("~recon_hold_s", 5.0))
        self.waypoint_tolerance_m = float(rospy.get_param("~waypoint_tolerance_m", 1.0))
        self.waypoint_vertical_tolerance_m = float(
            rospy.get_param("~waypoint_vertical_tolerance_m", 0.4)
        )
        self.waypoint_arrival_hold_s = float(
            rospy.get_param("~waypoint_arrival_hold_s", 2.0)
        )
        self.waypoint_timeout_s = float(rospy.get_param("~waypoint_timeout_s", 120.0))
        self.auto_start_mavros = get_bool_param("~auto_start_mavros", True)
        self.mavros_fcu_url = str(
            rospy.get_param("~mavros_fcu_url", "/dev/ttyACM0:115200")
        )
        self.detector_show_window = get_bool_param("~detector_show_window", True)
        self.dependency_restart_attempts = max(
            0, int(rospy.get_param("~dependency_restart_attempts", 3))
        )
        self.dependency_restart_backoff_s = max(
            0.0, float(rospy.get_param("~dependency_restart_backoff_s", 2.0))
        )
        self.detector_ready_timeout_s = float(
            rospy.get_param("~detector_ready_timeout_s", 30.0)
        )
        self.servo_ready_timeout_s = float(
            rospy.get_param("~servo_ready_timeout_s", 20.0)
        )
        self.enroute_detect_stable_s = float(
            rospy.get_param("~enroute_detect_stable_s", 0.3)
        )
        self.second_target_search_timeout_s = float(
            rospy.get_param("~second_target_search_timeout_s", 30.0)
        )
        self.aim_timeout_s = float(rospy.get_param("~aim_timeout_s", 20.0))
        self.target_lock_center_threshold_m = float(
            rospy.get_param("~target_lock_center_threshold_m", 0.20)
        )
        self.target_lock_window_s = float(
            rospy.get_param("~target_lock_window_s", 0.5)
        )
        self.target_lock_min_samples = max(
            2, int(rospy.get_param("~target_lock_min_samples", 5))
        )
        self.target_lock_max_xy_span_m = float(
            rospy.get_param("~target_lock_max_xy_span_m", 0.10)
        )
        self.target_lock_max_d_span_m = float(
            rospy.get_param("~target_lock_max_d_span_m", 0.15)
        )
        self.navigation_timeout_s = float(
            rospy.get_param("~navigation_timeout_s", 2.0)
        )
        self.navigation_grace_s = float(
            rospy.get_param("~navigation_grace_s", 5.0)
        )
        self.connection_loss_grace_s = float(
            rospy.get_param("~connection_loss_grace_s", 10.0)
        )
        self.service_retry_attempts = max(
            1, int(rospy.get_param("~service_retry_attempts", 3))
        )
        self.service_retry_backoff_s = max(
            0.0, float(rospy.get_param("~service_retry_backoff_s", 0.5))
        )
        self.takeoff_timeout_s = float(rospy.get_param("~takeoff_timeout_s", 40.0))
        self.takeoff_started_threshold_m = float(
            rospy.get_param("~takeoff_started_threshold_m", 0.30)
        )
        self.rtl_timeout_s = float(rospy.get_param("~rtl_timeout_s", 240.0))
        self.align_threshold_m = float(rospy.get_param("~align_threshold_m", 0.15))
        self.target_drop_height_m = float(
            rospy.get_param("~target_drop_height_m", 1.5)
        )
        self.height_tolerance_m = float(
            rospy.get_param("~height_tolerance_m", 0.15)
        )
        self.stable_time_s = float(rospy.get_param("~stable_time_s", 0.5))
        self.setpoint_interval_s = float(
            rospy.get_param("~setpoint_interval_s", 0.1)
        )
        self.after_a_delay_s = float(rospy.get_param("~after_a_delay_s", 3.0))
        self.auto_start_detector = get_bool_param("~auto_start_detector", True)
        self.auto_start_servo_test = get_bool_param(
            "~auto_start_servo_test", True
        )
        # 保留 ROS 参数服务器中的原始值，仅在节点内部计算本次运行的有效配置。
        self.detector_enabled = (
            self.auto_start_detector and not self.flight_only_mode
        )
        self.servo_enabled = (
            self.auto_start_servo_test and not self.flight_only_mode
        )
        self.b_dropper_enabled = (
            self.enable_b_dropper and not self.flight_only_mode
        )
        self.vision_enabled = not self.flight_only_mode
        self.servo_interface_enabled = not self.flight_only_mode
        self.aiming_enabled = not self.flight_only_mode
        self.drop_enabled = not self.flight_only_mode
        self.close_after_drop = get_bool_param("~close_after_drop", False)
        self.dry_run = get_bool_param("~dry_run", False)
        self.min_bucket_count = max(1, int(rospy.get_param("~min_bucket_count", 1)))

        # 自主瞄准状态机参数。高度表示飞机相对识别到的桶顶/地面目标点高度。
        # D435 画面中心深度是独立实测量，待现场标定后再决定如何进入控制闭环。
        self.acquire_stable_time_s = float(
            rospy.get_param("~acquire_stable_time_s", 0.8)
        )
        self.coarse_align_height_m = float(
            rospy.get_param("~coarse_align_height_m", 2.0)
        )
        self.fine_align_height_m = float(
            rospy.get_param("~fine_align_height_m", 1.7)
        )
        self.coarse_align_threshold_m = float(
            rospy.get_param("~coarse_align_threshold_m", 0.45)
        )
        self.fine_align_threshold_m = float(
            rospy.get_param("~fine_align_threshold_m", 0.20)
        )
        self.phase_stable_time_s = float(
            rospy.get_param("~phase_stable_time_s", 0.5)
        )
        self.target_filter_alpha = float(
            rospy.get_param("~target_filter_alpha", 0.25)
        )
        self.max_target_jump_m = float(
            rospy.get_param("~max_target_jump_m", 0.80)
        )
        self.brief_loss_s = float(rospy.get_param("~brief_loss_s", 0.5))
        self.loss_hover_s = float(rospy.get_param("~loss_hover_s", 2.0))
        self.reacquire_height_m = float(
            rospy.get_param("~reacquire_height_m", 2.0)
        )
        self.search_start_radius_m = float(
            rospy.get_param("~search_start_radius_m", 0.30)
        )
        self.search_radius_step_m = float(
            rospy.get_param("~search_radius_step_m", 0.30)
        )
        self.search_max_radius_m = float(
            rospy.get_param("~search_max_radius_m", 1.20)
        )
        self.search_dwell_s = float(rospy.get_param("~search_dwell_s", 1.5))
        self.target_loss_timeout_s = float(
            rospy.get_param("~target_loss_timeout_s", 30.0)
        )
        self.max_horizontal_setpoint_distance_m = float(
            rospy.get_param("~max_horizontal_setpoint_distance_m", 0.60)
        )
        self.max_vertical_setpoint_distance_m = float(
            rospy.get_param("~max_vertical_setpoint_distance_m", 0.20)
        )
        self.max_horizontal_setpoint_rate_mps = float(
            rospy.get_param("~max_horizontal_setpoint_rate_mps", 0.25)
        )
        self.max_vertical_setpoint_rate_mps = float(
            rospy.get_param("~max_vertical_setpoint_rate_mps", 0.20)
        )
        self.phase_max_horizontal_speed_mps = float(
            rospy.get_param("~phase_max_horizontal_speed_mps", 0.20)
        )
        self.phase_max_vertical_speed_mps = float(
            rospy.get_param("~phase_max_vertical_speed_mps", 0.15)
        )
        self.max_horizontal_speed_mps = float(
            rospy.get_param("~max_horizontal_speed_mps", 0.10)
        )
        self.max_vertical_speed_mps = float(
            rospy.get_param("~max_vertical_speed_mps", 0.10)
        )
        self.require_distinct_b_target = get_bool_param(
            "~require_distinct_b_target", True
        )
        self.distinct_target_distance_m = float(
            rospy.get_param("~distinct_target_distance_m", 0.60)
        )

        # 投放区快速搜索参数。区域中心和机头方向在 A 搜索授权时冻结；B 默认复用。
        self.area_search_pattern = self._normalize_area_search_pattern(
            rospy.get_param("~area_search_pattern", "W")
        )
        self.area_search_width_m = float(
            rospy.get_param("~area_search_width_m", 8.0)
        )
        self.area_search_length_m = float(
            rospy.get_param("~area_search_length_m", 5.0)
        )
        self.area_search_footprint_width_m = float(
            rospy.get_param("~area_search_footprint_width_m", 2.25)
        )
        self.area_search_footprint_length_m = float(
            rospy.get_param("~area_search_footprint_length_m", 1.5)
        )
        self.area_search_timeout_s = float(
            rospy.get_param("~area_search_timeout_s", 120.0)
        )
        self.area_search_waypoint_tolerance_m = float(
            rospy.get_param("~area_search_waypoint_tolerance_m", 0.25)
        )
        self.area_search_height_tolerance_m = float(
            rospy.get_param("~area_search_height_tolerance_m", 0.20)
        )
        self.area_search_dwell_s = float(
            rospy.get_param("~area_search_dwell_s", 0.5)
        )
        self.reuse_area_search_anchor_for_b = get_bool_param(
            "~reuse_area_search_anchor_for_b", True
        )

        self.aim_info_timeout_s = float(
            rospy.get_param("~aim_info_timeout_s", 1.0)
        )
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 2.0))
        self.guided_state_grace_s = float(
            rospy.get_param("~guided_state_grace_s", 5.0)
        )
        self.pose_timeout_s = float(
            rospy.get_param("~pose_timeout_s", self.state_timeout_s)
        )
        self.guided_pose_grace_s = float(
            rospy.get_param("~guided_pose_grace_s", 5.0)
        )
        self.local_position_rate_hz = float(
            rospy.get_param("~local_position_rate_hz", 10.0)
        )
        self.diagnostic_log_interval_s = float(
            rospy.get_param("~diagnostic_log_interval_s", 1.0)
        )
        self.connection_timeout_s = float(
            rospy.get_param("~connection_timeout_s", 30.0)
        )
        self.mode_timeout_s = float(rospy.get_param("~mode_timeout_s", 8.0))
        self.servo_wait_timeout_s = float(
            rospy.get_param("~servo_wait_timeout_s", 20.0)
        )
        self.close_delay_s = float(rospy.get_param("~close_delay_s", 0.8))
        self.detector_package = rospy.get_param(
            "~detector_package", "cuadc_vision"
        )
        self.detector_launch = rospy.get_param(
            "~detector_launch", "detector_node.launch"
        )

        if self.align_threshold_m <= 0.0:
            raise ValueError("align_threshold_m 必须大于 0")
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
        positive_values = (
            self.coarse_align_threshold_m,
            self.fine_align_threshold_m,
            self.max_target_jump_m,
            self.search_start_radius_m,
            self.search_radius_step_m,
            self.search_max_radius_m,
            self.search_dwell_s,
            self.target_loss_timeout_s,
            self.max_horizontal_setpoint_distance_m,
            self.max_vertical_setpoint_distance_m,
            self.max_horizontal_setpoint_rate_mps,
            self.max_vertical_setpoint_rate_mps,
            self.phase_max_horizontal_speed_mps,
            self.phase_max_vertical_speed_mps,
            self.max_horizontal_speed_mps,
            self.max_vertical_speed_mps,
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
                "flight_only_search_altitude_m 必须小于等于 recon_altitude_m"
            )
        area_positive_values = (
            self.area_search_width_m,
            self.area_search_length_m,
            self.area_search_footprint_width_m,
            self.area_search_footprint_length_m,
            self.area_search_timeout_s,
            self.area_search_waypoint_tolerance_m,
            self.area_search_height_tolerance_m,
            self.area_search_dwell_s,
        )
        if any(value <= 0.0 for value in area_positive_values):
            raise ValueError("区域搜索尺寸、超时、容差和停留时间必须大于 0")
        if self.area_search_footprint_width_m >= self.area_search_width_m:
            raise ValueError("area_search_footprint_width_m 必须小于搜索区宽度")
        if self.area_search_footprint_length_m >= self.area_search_length_m:
            raise ValueError("area_search_footprint_length_m 必须小于搜索区长度")
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
            raise ValueError(
                "recon_altitude_m 必须大于等于 drop_zone_transit_altitude_m"
            )
        mission_positive = (
            self.recon_hold_s,
            self.waypoint_tolerance_m,
            self.waypoint_vertical_tolerance_m,
            self.waypoint_arrival_hold_s,
            self.waypoint_timeout_s,
            self.detector_ready_timeout_s,
            self.servo_ready_timeout_s,
            self.enroute_detect_stable_s,
            self.second_target_search_timeout_s,
            self.aim_timeout_s,
            self.target_lock_center_threshold_m,
            self.target_lock_window_s,
            self.target_lock_max_xy_span_m,
            self.target_lock_max_d_span_m,
            self.navigation_timeout_s,
            self.navigation_grace_s,
            self.connection_loss_grace_s,
            self.takeoff_timeout_s,
            self.takeoff_started_threshold_m,
            self.rtl_timeout_s,
        )
        if any(value <= 0.0 for value in mission_positive):
            raise ValueError("完整任务航点、恢复、瞄准和超时参数必须大于 0")
        if self.navigation_grace_s <= self.navigation_timeout_s:
            raise ValueError("navigation_grace_s 必须大于 navigation_timeout_s")

        self._lock = threading.RLock()
        self.current_state = None
        self.state_received_at = 0.0
        self.latest_aim = None
        self.aim_received_at = 0.0
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
        self._param_get_srv = None
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

        if self.flight_only_mode:
            rospy.logwarn("================ FLIGHT ONLY MODE ================")
            rospy.logwarn(
                "vision=DISABLED servo=DISABLED aiming=DISABLED drop=DISABLED"
            )
            if self.auto_start_detector:
                rospy.logwarn(
                    "flight_only_mode=true：忽略 auto_start_detector=true；"
                    "不会检查、启动或等待 camera/detector"
                )
            if self.auto_start_servo_test:
                rospy.logwarn(
                    "flight_only_mode=true：忽略 auto_start_servo_test=true；"
                    "不会检查、启动或等待 servo_test.py"
                )
            if self.enable_b_dropper:
                rospy.logwarn(
                    "flight_only_mode=true：忽略 enable_b_dropper=true；"
                    "A/B 瞄准与投放均禁用"
                )
            if self.ground_test:
                rospy.logwarn(
                    "flight_only_mode=true：忽略 ground_test=true，按仅飞行任务流程执行"
                )
            if not self.full_mission_mode:
                rospy.logwarn(
                    "flight_only_mode=true：忽略 full_mission_mode=false，"
                    "Enter 后执行仅飞行航线"
                )

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self._extended_state_cb,
            queue_size=10,
        )
        rospy.Subscriber("/mavros/rc/in", RCIn, self._rc_cb, queue_size=10)
        rospy.Subscriber(
            "/mavros/statustext/recv",
            StatusText,
            self._statustext_cb,
            queue_size=20,
        )
        if self.vision_enabled:
            rospy.Subscriber(
                "/vision/bucket/aim_info",
                BucketAimInfo,
                self._aim_cb,
                queue_size=10,
            )
        rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            self._pose_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            "/mavros/local_position/velocity_local",
            TwistStamped,
            self._velocity_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            "/mavros/global_position/global",
            NavSatFix,
            self._fix_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            "/mavros/global_position/compass_hdg",
            Float64,
            self._heading_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            "/mavros/global_position/rel_alt",
            Float64,
            self._relative_altitude_cb,
            queue_size=10,
        )
        if self.servo_interface_enabled:
            rospy.Subscriber(
                "/servo/ready", Bool, self._servo_ready_cb, queue_size=1
            )
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
        )
        self.global_setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=20
        )
        self.servo_cmd_pub = rospy.Publisher(
            "/servo/cmd", String, queue_size=10
        )
        self.mission_status_pub = rospy.Publisher(
            "/vision/mission_status", MissionStatus, queue_size=10
        )

        rospy.on_shutdown(self._on_shutdown)

    def _state_cb(self, msg):
        with self._lock:
            previous = self.current_state
            self.current_state = msg
            self.state_received_at = monotonic_s()
            rc_summary = self._rc_summary_unlocked()
        if previous is None:
            rospy.loginfo(
                "MAVROS 首个状态 | connected=%s armed=%s mode=%s system_status=%s RC=%s",
                msg.connected,
                msg.armed,
                msg.mode,
                msg.system_status,
                rc_summary,
            )
        elif (
            previous.connected != msg.connected
            or previous.armed != msg.armed
            or previous.mode != msg.mode
            or previous.system_status != msg.system_status
        ):
            rospy.logwarn(
                "MAVROS 状态变化 | connected %s->%s armed %s->%s "
                "mode %s->%s system_status %s->%s RC=%s recent_fcu=%s",
                previous.connected,
                msg.connected,
                previous.armed,
                msg.armed,
                previous.mode,
                msg.mode,
                previous.system_status,
                msg.system_status,
                rc_summary,
                self._recent_statustext_summary(),
            )

    def _aim_cb(self, msg):
        if self.flight_only_mode:
            return
        with self._lock:
            self.latest_aim = msg
            self.aim_received_at = monotonic_s()

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
        rospy.logwarn(
            "FCU STATUSTEXT | severity=%d text=%s", msg.severity, msg.text
        )

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
                pose.pose.position.y, pose.pose.position.x, -pose.pose.position.z
            )
        wgs_text = "none"
        if fix is not None:
            wgs_text = "({:.8f},{:.8f},{:.2f})".format(
                fix.latitude, fix.longitude, fix.altitude
            )
        local_setpoint_text = "none"
        if self.last_setpoint_commanded_enu is not None:
            local_setpoint_text = "({:+.2f},{:+.2f},{:+.2f})".format(
                *self.last_setpoint_commanded_enu
            )
        global_setpoint_text = "none"
        if self.last_safe_global_target is not None:
            global_setpoint_text = "({:.8f},{:.8f},{:.2f})".format(
                self.last_safe_global_target.latitude,
                self.last_safe_global_target.longitude,
                self.last_safe_global_target.altitude,
            )
        rospy.logwarn(
            "MISSION TRANSITION | %s -> %s | reason=%s | mode=%s armed=%s "
            "connected=%s state_age=%.2fs NED=%s WGS84=%s rel_alt=%s "
            "ammo=A%d/B%d safe=%s:%s local_sp=%s global_sp=%s "
            "pattern=%s lap=%d/%d point=%d/%d",
            old_state,
            self.top_state,
            reason,
            state.mode if state is not None else "NONE",
            state.armed if state is not None else None,
            state.connected if state is not None else None,
            state_age,
            ned_text,
            wgs_text,
            "{:.2f}".format(rel_alt) if rel_alt is not None else "none",
            self.ammo_a,
            self.ammo_b,
            self.last_safe_setpoint_kind,
            self.last_setpoint_label,
            local_setpoint_text,
            global_setpoint_text,
            self.area_search_pattern,
            self.flight_only_current_lap,
            self.flight_only_pattern_laps,
            self.flight_only_current_point,
            self.flight_only_total_points,
        )

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
            return any(
                self._node_name_matches(name, expected_basename)
                for name in rosnode.get_node_names()
            )
        except Exception as exc:
            rospy.logwarn("读取 ROS 节点列表失败: %s", exc)
            return False

    def _spawn(self, command, label, restartable=True):
        rospy.logwarn("自动启动 %s: %s", label, " ".join(command))
        try:
            # 子节点不读取本脚本的交互终端，避免 servo_test.py 抢走 Enter。
            process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, preexec_fn=os.setsid
            )
        except (OSError, subprocess.SubprocessError) as exc:
            rospy.logerr("启动 %s 失败: %s", label, exc)
            return False
        self._children.append((label, process))
        if restartable:
            self._child_specs[label] = list(command)
            self._dependency_restart_counts.setdefault(label, 0)
        return True

    def start_dependencies(self):
        if not self._node_is_running("mavros") and not self._mavros_topic_exists():
            if not self.auto_start_mavros:
                rospy.logerr("MAVROS 未运行且 auto_start_mavros=false")
                return False
            if not self._spawn(
                [
                    "roslaunch",
                    "mavros",
                    "apm.launch",
                    "fcu_url:={}".format(self.mavros_fcu_url),
                ],
                "MAVROS",
            ):
                return False
            deadline = monotonic_s() + self.connection_timeout_s
            while not rospy.is_shutdown() and monotonic_s() < deadline:
                if self._mavros_topic_exists():
                    break
                rospy.loginfo_throttle(1.0, "等待自动启动的 MAVROS 发布 /mavros/state...")
                rospy.sleep(0.1)
            else:
                rospy.logerr("自动启动 MAVROS 后 /mavros/state 未出现")
                return False
        else:
            rospy.loginfo("MAVROS 已运行，不重复启动")

        if self.detector_enabled:
            detector_running = self._node_is_running("detector_node")
            camera_running = self._node_is_running("camera_node")

            if not detector_running:
                if not self._spawn(
                    [
                        "roslaunch",
                        self.detector_package,
                        self.detector_launch,
                        "start_camera:={}".format(
                            "false" if camera_running else "true"
                        ),
                        "show_window:={}".format(
                            "true" if self.detector_show_window else "false"
                        ),
                    ],
                    "识别 launch",
                ):
                    return False
            elif not camera_running:
                if not self._spawn(
                    ["roslaunch", self.detector_package, "camera_node.launch"],
                    "camera_node launch",
                ):
                    return False
            else:
                rospy.loginfo("detector_node 与 camera_node 已运行，不重复启动")

        if self.servo_enabled:
            if self._node_is_running("servo_controller"):
                rospy.loginfo("servo_test.py 已运行，不重复启动")
            elif not self._spawn(
                ["rosrun", "cuadc_vision", "servo_test.py"],
                "servo_test.py",
            ):
                return False
        return True

    def _mavros_topic_exists(self):
        try:
            return any(
                topic == "/mavros/state"
                for topic, _type in rospy.get_published_topics()
            )
        except Exception:
            return False

    def _wait_for_dependencies_ready(self):
        if self.flight_only_mode:
            rospy.logwarn(
                "FLIGHT ONLY：仅等待 MAVROS；视觉、相机和舵机就绪检查已跳过"
            )
            return True
        if self.detector_enabled:
            deadline = monotonic_s() + self.detector_ready_timeout_s
            while not rospy.is_shutdown() and monotonic_s() < deadline:
                with self._lock:
                    aim_seen = self.latest_aim is not None
                if (
                    self._node_is_running("camera_node")
                    and self._node_is_running("detector_node")
                    and aim_seen
                ):
                    rospy.loginfo("camera_node、detector_node 与视觉消息流已就绪")
                    break
                rospy.loginfo_throttle(2.0, "等待 camera/detector/aim_info 就绪...")
                rospy.sleep(0.1)
            else:
                rospy.logerr("视觉依赖未在 %.1fs 内就绪", self.detector_ready_timeout_s)
                return False
        if self.servo_enabled and not self._wait_for_servo_subscriber(
            timeout_s=self.servo_ready_timeout_s
        ):
            return False
        return True

    def _restart_dead_children(self):
        """有界重启本节点自动启动且意外退出的依赖。"""
        for index, (label, process) in enumerate(list(self._children)):
            if process.poll() is None or label not in self._child_specs:
                continue
            attempts = self._dependency_restart_counts.get(label, 0)
            if attempts >= self.dependency_restart_attempts:
                rospy.logerr_throttle(
                    1.0,
                    "%s 已退出且重启次数耗尽 %d/%d",
                    label,
                    attempts,
                    self.dependency_restart_attempts,
                )
                return False
            if self.airborne:
                hold = self._safe_hold_enu()
                if hold is None:
                    rospy.logerr("依赖退出且没有最后安全本地保持 setpoint")
                    return False
                self._publish_hold_setpoint(hold, "RECOVERY 依赖重启保持")
            rospy.logwarn(
                "%s 意外退出(code=%s)，%.1fs 后执行第 %d/%d 次重启",
                label,
                process.poll(),
                self.dependency_restart_backoff_s,
                attempts + 1,
                self.dependency_restart_attempts,
            )
            if not self._interruptible_delay(self.dependency_restart_backoff_s):
                return False
            command = self._child_specs[label]
            try:
                replacement = subprocess.Popen(
                    command, stdin=subprocess.DEVNULL, preexec_fn=os.setsid
                )
            except (OSError, subprocess.SubprocessError) as exc:
                rospy.logerr("重启 %s 失败: %s", label, exc)
                self._dependency_restart_counts[label] = attempts + 1
                return True
            self._children[index] = (label, replacement)
            self._dependency_restart_counts[label] = attempts + 1
        return True

    def _fresh_state(self):
        with self._lock:
            msg = self.current_state
            age = monotonic_s() - self.state_received_at
        if msg is None or age > self.state_timeout_s:
            return None
        return msg

    def _fresh_aim(self, require_count=True):
        if self.flight_only_mode:
            return None
        with self._lock:
            msg = self.latest_aim
            age = monotonic_s() - self.aim_received_at
        if msg is None or age > self.aim_info_timeout_s:
            return None
        if require_count and (msg.count < self.min_bucket_count or not msg.valid):
            return None
        return msg

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
                "state_age": (
                    now - self.state_received_at
                    if state is not None
                    else float("inf")
                ),
                "pose": pose,
                "pose_age": (
                    now - self.pose_received_at
                    if pose is not None
                    else float("inf")
                ),
                "velocity": velocity,
                "velocity_age": (
                    now - self.velocity_received_at
                    if velocity is not None
                    else float("inf")
                ),
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
            for stamp, severity, text in recent[-4:]
        )

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
            velocity_age = (
                now - self.velocity_received_at
                if velocity is not None
                else float("inf")
            )
            aim = self.latest_aim
            aim_age = now - self.aim_received_at if aim is not None else float("inf")
            extended = self.current_extended_state
            extended_age = (
                now - self.extended_state_received_at
                if extended is not None
                else float("inf")
            )
            rc = self._rc_summary_unlocked()
            rc_age = now - self.rc_received_at if self.current_rc is not None else float("inf")
            desired = self.last_setpoint_desired_enu
            commanded = self.last_setpoint_commanded_enu
            setpoint_label = self.last_setpoint_label
            setpoint_age = (
                now - self.last_setpoint_at
                if self.last_setpoint_at > 0.0
                else float("inf")
            )

        state_text = "none age=inf"
        if state is not None:
            state_text = (
                "mode={} connected={} armed={} system_status={} age={:.3f}s"
            ).format(
                state.mode,
                state.connected,
                state.armed,
                state.system_status,
                state_age,
            )
        pose_text = "none age=inf"
        if pose is not None:
            pos = pose.pose.position
            pose_text = "ENU=({:+.3f},{:+.3f},{:+.3f}) age={:.3f}s".format(
                pos.x, pos.y, pos.z, pose_age
            )
        velocity_text = "none age=inf"
        if velocity is not None:
            linear = velocity.twist.linear
            velocity_text = "ENU=({:+.3f},{:+.3f},{:+.3f}) age={:.3f}s".format(
                linear.x, linear.y, linear.z, velocity_age
            )
        aim_text = "none age=inf"
        if aim is not None:
            aim_text = (
                "count={} valid={} A_ned={} B_ned={} age={:.3f}s"
            ).format(
                aim.count,
                aim.valid,
                aim.a_ned_valid,
                aim.b_ned_valid,
                aim_age,
            )
        landed_text = "none age=inf"
        if extended is not None:
            landed_text = "{} age={:.3f}s".format(
                self._landed_state_name(extended.landed_state), extended_age
            )
        setpoint_text = "none"
        if desired is not None and commanded is not None:
            setpoint_text = (
                "label={} desired=({:+.3f},{:+.3f},{:+.3f}) "
                "commanded=({:+.3f},{:+.3f},{:+.3f}) age={:.3f}s"
            ).format(
                setpoint_label,
                desired[0],
                desired[1],
                desired[2],
                commanded[0],
                commanded[1],
                commanded[2],
                setpoint_age,
            )
        lock_text = "none" if locked_ned is None else "({:+.3f},{:+.3f},{:+.3f})".format(
            locked_ned[0], locked_ned[1], locked_ned[2]
        )
        return (
            "dropper={} phase={} | STATE[{}] | POSE[{}] | VEL[{}] | "
            "AIM[{}] | EXT[landed={}] | RC[{} age={:.3f}s] | LOCK_NED={} | "
            "SETPOINT[{}] | FCU_RECENT[{}]"
        ).format(
            dropper,
            phase,
            state_text,
            pose_text,
            velocity_text,
            aim_text,
            landed_text,
            rc,
            rc_age,
            lock_text,
            setpoint_text,
            self._recent_statustext_summary(),
        )

    def _log_control_diagnostics(self, dropper, phase, locked_ned=None, error=None):
        text = self._control_diagnostic_text(dropper, phase, locked_ned)
        if error:
            rospy.logerr("CONTROL DIAGNOSTIC | reason=%s | %s", error, text)
        else:
            rospy.loginfo_throttle(
                self.diagnostic_log_interval_s, "CONTROL TRACE | %s", text
            )

    def _aim_invalid_reason(self, dropper=None):
        """细分视觉目标为何当前不能用于飞行控制。"""
        with self._lock:
            msg = self.latest_aim
            age = monotonic_s() - self.aim_received_at
        if msg is None:
            return "尚未收到 /vision/bucket/aim_info"
        if age > self.aim_info_timeout_s:
            return "视觉消息停更 {:.2f}s".format(age)
        if msg.count < self.min_bucket_count:
            return "未检测到足够桶（count={}，需要≥{}）".format(
                msg.count, self.min_bucket_count
            )
        if not msg.valid:
            return "检测到桶但深度、相机内参或米制瞄准点无效"
        if dropper is not None and not getattr(
            msg, "{}_ned_valid".format(dropper.lower())
        ):
            return "{} 视觉有效但绝对 NED 无效".format(dropper)
        return "视觉目标有效"

    @staticmethod
    def _aim_target_ned(aim, dropper):
        prefix = dropper.lower()
        if aim is None or not getattr(aim, "{}_ned_valid".format(prefix)):
            return None
        values = (
            float(getattr(aim, "{}_ned_n".format(prefix))),
            float(getattr(aim, "{}_ned_e".format(prefix))),
            float(getattr(aim, "{}_ned_d".format(prefix))),
        )
        if not all(math.isfinite(value) for value in values):
            return None
        return values

    @staticmethod
    def _ned_xy_distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _target_is_blacklisted(self, target_ned, dropper):
        if dropper != "B" or not self.require_distinct_b_target:
            return False
        return any(
            self._ned_xy_distance(target_ned, old_target)
            < self.distinct_target_distance_m
            for old_target in self.completed_drop_targets
        )

    def _update_target_lock(self, locked_ned, measured_ned, dropper):
        if self._target_is_blacklisted(measured_ned, dropper):
            rospy.logwarn_throttle(
                1.0,
                "%s 当前目标与已投放目标距离小于 %.2fm，继续搜索新桶",
                dropper,
                self.distinct_target_distance_m,
            )
            return locked_ned, False
        if locked_ned is None:
            return measured_ned, True
        jump = self._ned_xy_distance(locked_ned, measured_ned)
        if jump > self.max_target_jump_m:
            rospy.logwarn_throttle(
                1.0,
                "%s 目标 NED 单帧跳变 %.2fm，超过 %.2fm，忽略该观测",
                dropper,
                jump,
                self.max_target_jump_m,
            )
            return locked_ned, False
        alpha = self.target_filter_alpha
        filtered = tuple(
            old + alpha * (new - old)
            for old, new in zip(locked_ned, measured_ned)
        )
        return filtered, True

    def _aim_wait_message(self):
        """返回面向现场操作者的视觉等待原因。"""
        with self._lock:
            msg = self.latest_aim
            age = monotonic_s() - self.aim_received_at
        if msg is None:
            return "尚未收到视觉瞄准消息，请检查 detector_node"
        if age > self.aim_info_timeout_s:
            return "视觉瞄准消息已超时，请检查相机和识别节点是否仍在运行"
        if msg.count < self.min_bucket_count:
            return "当前未识别到桶（检测到 {} 个，至少需要 {} 个）".format(
                msg.count, self.min_bucket_count
            )
        if not msg.valid:
            return (
                "已识别到桶，但深度或相机内参无效；请调整距离、角度，"
                "并检查桶中心是否有有效深度"
            )
        return "视觉目标有效"

    def _navigation_snapshot(self):
        now = monotonic_s()
        with self._lock:
            return {
                "pose": self.current_pose,
                "pose_age": now - self.pose_received_at if self.current_pose else float("inf"),
                "fix": self.current_fix,
                "fix_age": now - self.fix_received_at if self.current_fix else float("inf"),
                "heading": self.heading_deg,
                "heading_age": now - self.heading_received_at if self.heading_deg is not None else float("inf"),
                "rel_alt": self.relative_altitude,
                "rel_alt_age": now - self.relative_altitude_received_at if self.relative_altitude is not None else float("inf"),
            }

    def _navigation_is_fresh(self):
        nav = self._navigation_snapshot()
        return (
            nav["pose"] is not None
            and nav["fix"] is not None
            and nav["heading"] is not None
            and nav["rel_alt"] is not None
            and nav["pose_age"] <= self.navigation_timeout_s
            and nav["fix_age"] <= self.navigation_timeout_s
            and nav["heading_age"] <= self.navigation_timeout_s
            and nav["rel_alt_age"] <= self.navigation_timeout_s
        )

    def wait_for_navigation(self):
        deadline = monotonic_s() + self.connection_timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            if self._navigation_is_fresh() and self._fresh_velocity() is not None:
                rospy.loginfo("GPS、本地位姿、速度、航向和相对高度均已就绪")
                return True
            nav = self._navigation_snapshot()
            rospy.loginfo_throttle(
                1.0,
                "等待导航数据 | pose=%.2fs fix=%.2fs heading=%.2fs rel_alt=%.2fs vel=%.2fs",
                nav["pose_age"], nav["fix_age"], nav["heading_age"],
                nav["rel_alt_age"], self._control_stream_snapshot()["velocity_age"],
            )
            rate.sleep()
        rospy.logerr("等待完整导航数据超时")
        return False

    @classmethod
    def _destination_wgs84(cls, latitude, longitude, distance, heading_deg):
        lat_rad = math.radians(latitude)
        heading_rad = math.radians(heading_deg)
        eccentricity_sq = cls.WGS84_F * (2.0 - cls.WGS84_F)
        sin_lat = math.sin(lat_rad)
        denominator = math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
        prime_vertical_radius = cls.WGS84_A / denominator
        meridian_radius = (
            cls.WGS84_A * (1.0 - eccentricity_sq) / denominator ** 3
        )
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

    @classmethod
    def _horizontal_wgs84_distance(cls, lat1, lon1, lat2, lon2):
        mean_lat = math.radians((lat1 + lat2) * 0.5)
        eccentricity_sq = cls.WGS84_F * (2.0 - cls.WGS84_F)
        sin_lat = math.sin(mean_lat)
        denominator = math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
        prime_vertical_radius = cls.WGS84_A / denominator
        meridian_radius = (
            cls.WGS84_A * (1.0 - eccentricity_sq) / denominator ** 3
        )
        north = math.radians(lat2 - lat1) * meridian_radius
        dlon = math.radians((lon2 - lon1 + 180.0) % 360.0 - 180.0)
        east = dlon * prime_vertical_radius * math.cos(mean_lat)
        return math.hypot(north, east)

    def plan_waypoints(self):
        if not self._navigation_is_fresh():
            rospy.logerr("导航数据不新鲜，不能冻结起点和航点")
            return False
        with self._lock:
            pose = self.current_pose.pose.position
            fix = self.current_fix
            self.start_ned = (float(pose.y), float(pose.x), float(-pose.z))
            self.start_wgs84 = (
                float(fix.latitude), float(fix.longitude), float(fix.altitude)
            )
            self.start_heading_deg = float(self.heading_deg)
            self.start_relative_altitude = float(self.relative_altitude)
        drop_lat, drop_lon = self._destination_wgs84(
            self.start_wgs84[0], self.start_wgs84[1],
            self.drop_zone_distance_m, self.start_heading_deg,
        )
        recon_lat, recon_lon = self._destination_wgs84(
            self.start_wgs84[0], self.start_wgs84[1],
            self.recon_zone_distance_m, self.start_heading_deg,
        )
        self.drop_zone_wgs84 = (drop_lat, drop_lon)
        self.recon_zone_wgs84 = (recon_lat, recon_lon)
        return True

    def print_mission_summary(self):
        if self.flight_only_mode:
            drop_distance = self._horizontal_wgs84_distance(
                self.start_wgs84[0],
                self.start_wgs84[1],
                self.drop_zone_wgs84[0],
                self.drop_zone_wgs84[1],
            )
            recon_distance = self._horizontal_wgs84_distance(
                self.start_wgs84[0],
                self.start_wgs84[1],
                self.recon_zone_wgs84[0],
                self.recon_zone_wgs84[1],
            )
            route = (
                "GUIDED -> ARM -> TAKEOFF -> 投放区 -> {} 字完整航线 x{} -> "
                "侦察区爬升/保持 -> RTL"
            ).format(self.area_search_pattern, self.flight_only_pattern_laps)
            if not self.flight_only_continue_to_recon:
                route = (
                    "GUIDED -> ARM -> TAKEOFF -> 投放区 -> {} 字完整航线 x{} -> RTL"
                ).format(self.area_search_pattern, self.flight_only_pattern_laps)
            lines = [
                "",
                "================ FLIGHT ONLY 任务摘要 ================",
                "MODE          = FLIGHT_ONLY",
                "VISION        = DISABLED",
                "SERVO         = DISABLED",
                "AIM/DROP      = DISABLED",
                "起飞点 WGS84  : lat={:.9f}, lon={:.9f}, alt={:.3f}m".format(
                    *self.start_wgs84
                ),
                "起始 NED       : N={:.3f}, E={:.3f}, D={:.3f}m".format(
                    *self.start_ned
                ),
                "启动航向       : {:.2f}deg（0=北，顺时针）".format(
                    self.start_heading_deg
                ),
                "投放区 WGS84  : distance={:.2f}m lat={:.9f}, lon={:.9f}".format(
                    drop_distance, *self.drop_zone_wgs84
                ),
                "侦察区 WGS84  : distance={:.2f}m lat={:.9f}, lon={:.9f}".format(
                    recon_distance, *self.recon_zone_wgs84
                ),
                "起飞高度       : ground rel_alt + {:.2f}m".format(
                    self.takeoff_altitude_m
                ),
                "投放区航程高度 : ground rel_alt + {:.2f}m".format(
                    self.drop_zone_transit_altitude_m
                ),
                "区域航线高度   : ground rel_alt + {:.2f}m".format(
                    self.flight_only_search_altitude_m
                ),
                "区域航线模式   : {}（W/Z/I，LINE 等同 I）".format(
                    self.area_search_pattern
                ),
                "区域航线圈数   : {}".format(self.flight_only_pattern_laps),
                "侦察高度/保持  : ground rel_alt + {:.2f}m / {:.1f}s".format(
                    self.recon_altitude_m, self.recon_hold_s
                ),
                "continue_recon : {}".format(
                    self.flight_only_continue_to_recon
                ),
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
            "投放区 WGS84 : {:.2f}m, lat={:.9f}, lon={:.9f}".format(
                self.drop_zone_distance_m, *self.drop_zone_wgs84
            ),
            "侦察区 WGS84 : {:.2f}m, lat={:.9f}, lon={:.9f}".format(
                self.recon_zone_distance_m, *self.recon_zone_wgs84
            ),
            "区域中心间距  : 约 {:.2f}m".format(
                self.recon_zone_distance_m - self.drop_zone_distance_m
            ),
            "起飞高度      : 地面 rel_alt + {:.2f}m".format(
                self.takeoff_altitude_m
            ),
            "投放区航程高度: 地面 rel_alt + {:.2f}m".format(
                self.drop_zone_transit_altitude_m
            ),
            "侦察航高      : 地面 rel_alt + {:.2f}m；保持 {:.1f}s".format(
                self.recon_altitude_m, self.recon_hold_s
            ),
            "W/Z/I 搜索    : {}，区域 {:.1f}m x {:.1f}m".format(
                self.area_search_pattern, self.area_search_width_m, self.area_search_length_m
            ),
            "A/B 余弹      : A={} B={}".format(self.ammo_a, self.ammo_b),
            "dry-run       : {}（true 时完整飞行但不发布舵机打开命令）".format(self.dry_run),
            "流程           : GUIDED/ARM/TAKEOFF -> 投放区/途中截获 -> A/B -> 侦察区 -> RTL",
            "Enter 前不会切模式、解锁、起飞、发布移动 setpoint 或打开舵机。",
            "=======================================================",
        ]
        print("\n".join(lines), flush=True)

    @staticmethod
    def _read_enter(prompt):
        try:
            sys.stdout.write(prompt)
            sys.stdout.flush()
            try:
                with open("/dev/tty", "r", encoding="utf-8") as terminal:
                    return terminal.readline() is not None
            except OSError:
                return sys.stdin.readline() is not None
        except (EOFError, KeyboardInterrupt, OSError):
            return False

    def wait_for_enter_authorization(self):
        if self.ground_test and not self.flight_only_mode:
            rospy.logwarn("ground_test=true：启动命令视为地面测试授权，不实际起飞")
            self.authorized = True
            return True
        mission_name = "仅飞行航线验证" if self.flight_only_mode else "完整任务"
        ok = self._read_enter(
            "确认空域和载荷安全后按 Enter，授权执行{}（Ctrl+C 取消）: ".format(
                mission_name
            )
        )
        self.authorized = bool(ok and not rospy.is_shutdown())
        if self.authorized:
            rospy.logwarn("操作者已按 Enter，授权执行%s", mission_name)
        return self.authorized

    def _ensure_flight_services(self):
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=10.0)
            rospy.wait_for_service("/mavros/cmd/arming", timeout=10.0)
            rospy.wait_for_service("/mavros/cmd/takeoff", timeout=10.0)
            self._set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self._arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            self._takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
            try:
                rospy.wait_for_service("/mavros/param/get", timeout=2.0)
                self._param_get_srv = rospy.ServiceProxy("/mavros/param/get", ParamGet)
            except rospy.ROSException:
                self._param_get_srv = None
            return True
        except rospy.ROSException as exc:
            rospy.logerr("等待 MAVROS 飞行服务失败: %s", exc)
            return False

    def arm_confirmed(self):
        for attempt in range(1, self.service_retry_attempts + 1):
            state = self._fresh_state()
            if state is not None and state.armed:
                return True
            try:
                response = self._arming_srv(value=True)
            except rospy.ServiceException as exc:
                response = None
                rospy.logwarn("解锁服务第 %d/%d 次异常: %s", attempt, self.service_retry_attempts, exc)
            deadline = monotonic_s() + self.mode_timeout_s
            while not rospy.is_shutdown() and monotonic_s() < deadline:
                state = self._fresh_state()
                if state is not None and state.armed:
                    rospy.logwarn("已回读确认 armed=true")
                    return True
                rospy.sleep(0.1)
            rospy.logwarn("解锁第 %d/%d 次未确认，response=%s", attempt, self.service_retry_attempts, response)
            if attempt < self.service_retry_attempts:
                rospy.sleep(self.service_retry_backoff_s)
        return False

    def capture_ground_relative_altitude(self):
        deadline = monotonic_s() + self.connection_timeout_s
        fresh_since = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            state = self._fresh_state()
            nav = self._navigation_snapshot()
            if state is None or not state.connected or not state.armed:
                rospy.logerr("记录地面 rel_alt 时飞控状态无效")
                return False
            if nav["rel_alt"] is not None and nav["rel_alt_age"] <= self.navigation_timeout_s:
                if fresh_since is None:
                    fresh_since = monotonic_s()
                if monotonic_s() - fresh_since >= 1.0:
                    self.ground_relative_altitude = float(nav["rel_alt"])
                    rospy.logwarn(
                        "地面相对高度基准已记录: %.3fm；起飞目标=%.3fm "
                        "投放区航程目标=%.3fm 侦察目标=%.3fm",
                        self.ground_relative_altitude,
                        self.ground_relative_altitude + self.takeoff_altitude_m,
                        self.ground_relative_altitude
                        + self.drop_zone_transit_altitude_m,
                        self.ground_relative_altitude + self.recon_altitude_m,
                    )
                    return True
            else:
                fresh_since = None
            rate.sleep()
        return False

    def _takeoff_has_started(self):
        nav = self._navigation_snapshot()
        return (
            self.ground_relative_altitude is not None
            and nav["rel_alt"] is not None
            and nav["rel_alt_age"] <= self.navigation_grace_s
            and nav["rel_alt"] - self.ground_relative_altitude >= self.takeoff_started_threshold_m
        )

    def command_takeoff_and_wait(self):
        target_rel_alt = self.ground_relative_altitude + self.takeoff_altitude_m
        command_accepted = False
        for attempt in range(1, self.service_retry_attempts + 1):
            try:
                response = self._takeoff_srv(
                    altitude=target_rel_alt, latitude=0.0, longitude=0.0,
                    min_pitch=0.0, yaw=0.0,
                )
            except rospy.ServiceException as exc:
                response = None
                rospy.logwarn("起飞服务第 %d/%d 次异常: %s", attempt, self.service_retry_attempts, exc)
            if response is not None and response.success:
                command_accepted = True
                break
            rospy.sleep(min(1.0, self.service_retry_backoff_s))
            if self._takeoff_has_started():
                rospy.logwarn("起飞 ACK 不可靠，但检测到飞机已开始上升，继续监控")
                command_accepted = True
                break
        if not command_accepted:
            rospy.logerr("起飞命令有界重试后仍未确认，且未检测到起飞开始")
            return False
        deadline = monotonic_s() + self.takeoff_timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            state = self._fresh_state()
            nav = self._navigation_snapshot()
            if state is not None and not state.armed:
                return False
            if nav["rel_alt"] is not None and nav["rel_alt_age"] <= self.navigation_grace_s:
                climbed = nav["rel_alt"] - self.ground_relative_altitude
                if climbed >= self.takeoff_altitude_m - 0.20:
                    self.airborne = True
                    rospy.logwarn("已到达起飞高度: 离地 %.2f/%.2fm", climbed, self.takeoff_altitude_m)
                    return True
                rospy.loginfo_throttle(1.0, "起飞爬升中: 离地 %.2f/%.2fm", climbed, self.takeoff_altitude_m)
            rate.sleep()
        return False

    def _make_global_target(self, latitude, longitude, height_above_ground_m):
        msg = GlobalPositionTarget()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        msg.latitude = float(latitude)
        msg.longitude = float(longitude)
        msg.altitude = self.ground_relative_altitude + float(height_above_ground_m)
        return msg

    def _publish_global_target(self, target, label):
        target.header.stamp = rospy.Time.now()
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
        stale_grace = self.connection_loss_grace_s if not state.connected else self.guided_state_grace_s
        if snapshot["state_age"] <= stale_grace and state.armed and state.mode == "GUIDED":
            return "recover", "state age {:.2f}s".format(snapshot["state_age"])
        return "fatal", "state 超过恢复宽限 age={:.2f}s".format(snapshot["state_age"])

    def fly_global_waypoint(
        self, target, label, detect_dropper=None, require_full_telemetry=False
    ):
        rate = rospy.Rate(10)
        started_at = monotonic_s()
        inside_since = None
        detect_since = None
        recovery_started = None
        recovery_hold = None
        recovery_use_global = False
        last_recovery_log = 0.0
        while not rospy.is_shutdown():
            now = monotonic_s()
            if not self._restart_dead_children():
                return "failed"
            health, reason = self._flight_state_health()
            nav = self._navigation_snapshot()
            control = self._control_stream_snapshot()
            nav_fresh = (
                nav["fix"] is not None and nav["rel_alt"] is not None
                and nav["fix_age"] <= self.navigation_timeout_s
                and nav["rel_alt_age"] <= self.navigation_timeout_s
            )
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
                if (
                    control["velocity"] is None
                    or control["velocity_age"] > self.pose_timeout_s
                ):
                    stale_fields.append(
                        "velocity={:.2f}s".format(control["velocity_age"])
                    )
                    if (
                        control["velocity"] is None
                        or control["velocity_age"] > self.guided_pose_grace_s
                    ):
                        expired_fields.append(stale_fields[-1])
                if nav["fix"] is None or nav["fix_age"] > self.navigation_grace_s:
                    expired_fields.append("fix={:.2f}s".format(nav["fix_age"]))
                if (
                    nav["rel_alt"] is None
                    or nav["rel_alt_age"] > self.navigation_grace_s
                ):
                    expired_fields.append(
                        "rel_alt={:.2f}s".format(nav["rel_alt_age"])
                    )
            full_telemetry_fresh = not stale_fields
            if health == "fatal":
                rospy.logerr("%s 安全退出: %s", label, reason)
                return "failed"
            if expired_fields:
                rospy.logerr(
                    "%s 飞行遥测超过恢复宽限: %s",
                    label,
                    " ".join(expired_fields),
                )
                return "failed"
            if health == "recover" or not nav_fresh or not full_telemetry_fresh:
                if health == "recover":
                    recovery_reason = reason
                elif not nav_fresh:
                    recovery_reason = (
                        "导航不新鲜 fix={:.2f}s rel_alt={:.2f}s"
                    ).format(nav["fix_age"], nav["rel_alt_age"])
                else:
                    recovery_reason = "飞行遥测不新鲜 " + " ".join(stale_fields)
                if recovery_started is None:
                    recovery_started = now
                    self._begin_recovery("{}: {}".format(label, recovery_reason))
                    recovery_hold = self._safe_hold_enu()
                    if recovery_hold is None:
                        if self.last_safe_global_target is None:
                            rospy.logerr("%s 进入恢复但没有最后安全 setpoint", label)
                            return "failed"
                        recovery_use_global = True
                    else:
                        recovery_use_global = False
                if "connected" in recovery_reason:
                    grace = self.connection_loss_grace_s
                elif health == "recover":
                    grace = self.guided_state_grace_s
                elif any(field.startswith(("pose=", "velocity=")) for field in stale_fields):
                    grace = max(self.navigation_grace_s, self.guided_pose_grace_s)
                else:
                    grace = self.navigation_grace_s
                elapsed = now - recovery_started
                if elapsed > grace:
                    rospy.logerr("%s 恢复超时 %.1f/%.1fs: %s", label, elapsed, grace, recovery_reason)
                    return "failed"
                if recovery_use_global:
                    self._publish_global_target(
                        self.last_safe_global_target,
                        "{} RECOVERY GLOBAL HOLD".format(label),
                    )
                else:
                    self._publish_hold_setpoint(
                        recovery_hold, "{} RECOVERY HOLD".format(label)
                    )
                if now - last_recovery_log >= 1.0:
                    rospy.logwarn(
                        "%s RECOVERY | %s | 已恢复等待 %.1fs 剩余宽限 %.1fs",
                        label, recovery_reason, elapsed, max(0.0, grace - elapsed),
                    )
                    last_recovery_log = now
                rate.sleep()
                continue
            if recovery_started is not None:
                paused = now - recovery_started
                started_at += paused
                rospy.logwarn("%s 遥测已恢复，继续原航点；暂停计时 %.2fs", label, paused)
                recovery_started = None
                recovery_hold = None
                recovery_use_global = False
                self._end_recovery("{} 遥测恢复，继续原航点".format(label))

            if detect_dropper is not None:
                aim = self._fresh_aim()
                target_ned = self._aim_target_ned(aim, detect_dropper)
                eligible = target_ned is not None and not self._target_is_blacklisted(target_ned, detect_dropper)
                if eligible:
                    if detect_since is None:
                        detect_since = now
                    if now - detect_since >= self.enroute_detect_stable_s:
                        pose = self._fresh_pose()
                        if pose is None:
                            detect_since = None
                        else:
                            hold = self._pose_position_enu(pose)
                            remaining = self._horizontal_wgs84_distance(
                                nav["fix"].latitude, nav["fix"].longitude,
                                target.latitude, target.longitude,
                            )
                            aim_age = monotonic_s() - self.aim_received_at
                            rospy.logwarn(
                                "%s 途中稳定发现目标 | aircraft_NED=(%.2f,%.2f,%.2f) "
                                "aircraft_WGS84=(%.8f,%.8f) bucket_NED=(%.2f,%.2f,%.2f) "
                                "aim_age=%.3fs remaining=%.2fm；停止全局 setpoint",
                                detect_dropper,
                                pose.pose.position.y, pose.pose.position.x, -pose.pose.position.z,
                                nav["fix"].latitude, nav["fix"].longitude,
                                target_ned[0], target_ned[1], target_ned[2], aim_age, remaining,
                            )
                            self._publish_hold_setpoint(hold, "{} 途中截获本地保持".format(detect_dropper))
                            rospy.sleep(self.setpoint_interval_s)
                            return "detected"
                else:
                    detect_since = None

            self._publish_global_target(target, label)
            horizontal_error = self._horizontal_wgs84_distance(
                nav["fix"].latitude, nav["fix"].longitude,
                target.latitude, target.longitude,
            )
            vertical_error = abs(nav["rel_alt"] - target.altitude)
            inside = (
                horizontal_error <= self.waypoint_tolerance_m
                and vertical_error <= self.waypoint_vertical_tolerance_m
            )
            if inside:
                if inside_since is None:
                    inside_since = now
                if now - inside_since >= self.waypoint_arrival_hold_s:
                    rospy.logwarn("%s 已连续到达: horizontal=%.2fm vertical=%.2fm", label, horizontal_error, vertical_error)
                    return "arrived"
            else:
                inside_since = None
            if now - started_at >= self.waypoint_timeout_s:
                rospy.logerr("%s 超时: horizontal=%.2fm vertical=%.2fm", label, horizontal_error, vertical_error)
                return "failed"
            if detect_dropper is None:
                rospy.loginfo_throttle(
                    1.0,
                    "%s | horizontal=%.2fm vertical=%.2fm vision_monitor=DISABLED",
                    label,
                    horizontal_error,
                    vertical_error,
                )
            else:
                rospy.loginfo_throttle(
                    1.0, "%s | horizontal=%.2fm vertical=%.2fm detect_stable=%.2f/%.2fs",
                    label, horizontal_error, vertical_error,
                    0.0 if detect_since is None else now - detect_since,
                    self.enroute_detect_stable_s,
                )
            rate.sleep()
        return "failed"

    def hold_global_target(
        self, target, duration_s, label, require_full_telemetry=False
    ):
        rate = rospy.Rate(10)
        held = 0.0
        last = monotonic_s()
        recovery_started = None
        while not rospy.is_shutdown() and held < duration_s:
            now = monotonic_s()
            if not self._restart_dead_children():
                rospy.logerr("%s 期间依赖有界重启失败", label)
                return False
            health, reason = self._flight_state_health()
            nav = self._navigation_snapshot()
            control = self._control_stream_snapshot()
            telemetry_reasons = []
            if require_full_telemetry:
                if nav["fix"] is None or nav["fix_age"] > self.navigation_timeout_s:
                    telemetry_reasons.append("fix={:.2f}s".format(nav["fix_age"]))
                if nav["heading"] is None or nav["heading_age"] > self.navigation_timeout_s:
                    telemetry_reasons.append(
                        "heading={:.2f}s".format(nav["heading_age"])
                    )
                if nav["rel_alt"] is None or nav["rel_alt_age"] > self.navigation_timeout_s:
                    telemetry_reasons.append(
                        "rel_alt={:.2f}s".format(nav["rel_alt_age"])
                    )
                if nav["pose"] is None or nav["pose_age"] > self.pose_timeout_s:
                    telemetry_reasons.append("pose={:.2f}s".format(nav["pose_age"]))
                if (
                    control["velocity"] is None
                    or control["velocity_age"] > self.pose_timeout_s
                ):
                    telemetry_reasons.append(
                        "velocity={:.2f}s".format(control["velocity_age"])
                    )
            if health == "fatal":
                rospy.logerr("%s: %s", label, reason)
                return False
            if health == "ok" and not telemetry_reasons:
                if recovery_started is not None:
                    rospy.logwarn("%s 遥测恢复，继续累计保持时间", label)
                    recovery_started = None
                self._publish_global_target(target, label)
                held += max(0.0, now - last)
            elif self.last_safe_global_target is not None:
                if recovery_started is None:
                    recovery_started = now
                self._publish_global_target(self.last_safe_global_target, label + " RECOVERY")
                recovery_text = reason or "飞行遥测不新鲜 " + " ".join(telemetry_reasons)
                max_age = max(
                    [
                        nav["fix_age"],
                        nav["heading_age"],
                        nav["rel_alt_age"],
                        nav["pose_age"],
                        control["velocity_age"],
                    ]
                )
                grace = max(
                    self.navigation_grace_s,
                    self.guided_pose_grace_s,
                    self.guided_state_grace_s,
                )
                if "connected" in recovery_text:
                    grace = self.connection_loss_grace_s
                recovery_elapsed = now - recovery_started
                if (
                    (max_age > grace and telemetry_reasons)
                    or recovery_elapsed > grace
                ):
                    rospy.logerr("%s 恢复宽限耗尽: %s", label, recovery_text)
                    return False
                rospy.logwarn_throttle(
                    1.0,
                    "%s 暂停保持计时 %.2f/%.2fs: %s",
                    label,
                    recovery_elapsed,
                    grace,
                    recovery_text,
                )
            else:
                return False
            last = now
            rate.sleep()
        return not rospy.is_shutdown()

    def set_area_search_anchor_from_current(self, label):
        pose = self._fresh_pose()
        if pose is None or self.start_heading_deg is None:
            return False
        # compass: 0=North clockwise；ROS ENU yaw: 0=East counter-clockwise。
        yaw_enu = math.radians(90.0 - self.start_heading_deg)
        p = pose.pose.position
        self.area_search_anchor = (float(p.x), float(p.y), float(p.z), yaw_enu)
        rospy.logwarn(
            "%s 搜索中心已冻结: ENU=(%.2f,%.2f,%.2f), start_heading=%.2fdeg",
            label, p.x, p.y, p.z, self.start_heading_deg,
        )
        return True

    def request_rtl_and_wait(self):
        if not self.set_mode_confirmed("RTL"):
            rospy.logerr("RTL 请求或回读失败")
            return False
        deadline = monotonic_s() + self.rtl_timeout_s
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            with self._lock:
                state = self.current_state
                state_age = monotonic_s() - self.state_received_at if state else float("inf")
            if state is not None and state_age <= self.connection_loss_grace_s:
                if not state.armed:
                    self.airborne = False
                    rospy.logwarn("RTL 已完成并上锁")
                    return True
                if state.mode not in ("RTL", "LAND"):
                    rospy.logerr("RTL 等待期间收到模式 %s，停止自动控制", state.mode)
                    return False
            rospy.loginfo_throttle(2.0, "RTL 中等待返航/降落 | mode=%s armed=%s state_age=%.2fs", state.mode if state else "NONE", state.armed if state else None, state_age)
            rate.sleep()
        rospy.logerr("RTL 等待超时；保持飞控当前 RTL，不在异地强制 LAND")
        return False

    def wait_for_mavros(self):
        deadline = monotonic_s() + self.connection_timeout_s
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            state = self._fresh_state()
            if state is not None and state.connected:
                rospy.loginfo(
                    "MAVROS 已连接: armed=%s mode=%s", state.armed, state.mode
                )
                return True
            rospy.loginfo_throttle(2.0, "等待 /mavros/state 与飞控连接...")
            rospy.sleep(0.1)
        rospy.logerr("等待 MAVROS 飞控连接超时")
        return False

    def request_local_position_stream(self):
        """请求飞控以稳定频率发送 MAVLink LOCAL_POSITION_NED（消息 32）。"""
        try:
            rospy.wait_for_service("/mavros/set_message_interval", timeout=3.0)
            if self._message_interval_srv is None:
                self._message_interval_srv = rospy.ServiceProxy(
                    "/mavros/set_message_interval", MessageInterval
                )
            response = self._message_interval_srv(
                message_id=32, message_rate=self.local_position_rate_hz
            )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            self._message_interval_srv = None
            rospy.logwarn(
                "请求 LOCAL_POSITION_NED %.1fHz 失败，将依靠遥测恢复宽限: %s",
                self.local_position_rate_hz,
                exc,
            )
            return False
        if response.success:
            rospy.logwarn(
                "已请求 MAVLink LOCAL_POSITION_NED(message_id=32) 频率 %.1fHz",
                self.local_position_rate_hz,
            )
            return True
        rospy.logwarn(
            "飞控未接受 LOCAL_POSITION_NED %.1fHz 请求，将依靠遥测恢复宽限",
            self.local_position_rate_hz,
        )
        return False

    def wait_for_target_and_confirm(self, dropper="A"):
        """等待稳定目标；半自动模式等待期间丢失目标会重新获取而非退出。"""
        if self.flight_only_mode:
            rospy.logerr("FLIGHT ONLY 禁止等待或确认视觉目标")
            return False
        rate = rospy.Rate(10)
        valid_since = None
        while not rospy.is_shutdown():
            aim = self._fresh_aim()
            target_ok = aim is not None and (
                self.ground_test or self._aim_target_ned(aim, dropper) is not None
            )
            if not target_ok:
                valid_since = None
                rospy.loginfo_throttle(
                    2.0,
                    "等待 %s 稳定视觉目标：%s",
                    dropper,
                    self._aim_invalid_reason(None if self.ground_test else dropper),
                )
                rate.sleep()
                continue

            now = monotonic_s()
            if valid_since is None:
                valid_since = now
            stable = now - valid_since
            if stable < self.acquire_stable_time_s:
                rospy.loginfo_throttle(
                    0.5,
                    "%s 目标获取稳定时间 %.1f/%.1fs",
                    dropper,
                    stable,
                    self.acquire_stable_time_s,
                )
                rate.sleep()
                continue

            if self.ground_test:
                rospy.logwarn(
                    "地面测试已由启动命令授权：检测到稳定有效桶目标，"
                    "不等待 Enter，直接开始 %s 抛投器对准监测",
                    dropper,
                )
                return True
            if self.autonomous_mode:
                rospy.logwarn(
                    "autonomous_mode=true：%s 目标稳定 %.1fs，自动进入 GUIDED",
                    dropper,
                    stable,
                )
                return True

            prompt = (
                "检测到稳定桶目标，按 Enter 后进入 GUIDED，并执行分层自动瞄准；"
                "等待期间目标失效将自动重新获取，Ctrl+C 取消。"
            )
            rospy.logwarn(prompt)
            try:
                sys.stdout.write("\n{}\n".format(prompt))
                sys.stdout.flush()
                tty = None
                try:
                    tty = open(
                        "/dev/tty", "r", encoding="utf-8", errors="replace"
                    )
                    stream = tty
                except OSError:
                    stream = sys.stdin
                try:
                    while not rospy.is_shutdown():
                        current = self._fresh_aim()
                        current_ok = current is not None and (
                            self._aim_target_ned(current, dropper) is not None
                        )
                        if not current_ok:
                            rospy.logwarn(
                                "等待 Enter 期间目标失效：%s；回到目标获取状态",
                                self._aim_invalid_reason(dropper),
                            )
                            valid_since = None
                            break
                        ready, _, _ = select.select([stream], [], [], 0.1)
                        if ready:
                            return bool(stream.readline()) and not rospy.is_shutdown()
                finally:
                    if tty is not None:
                        tty.close()
            except (EOFError, KeyboardInterrupt, OSError):
                return False
        return False

    def _validate_before_guided(self, dropper):
        if self.ground_test:
            # 地面测试没有 GUIDED 前置条件。尤其是 B 流程开始瞬间即使目标
            # 正好丢失，也必须进入持续瞄准循环等待恢复，不能一次性判失败。
            state = self._fresh_state()
            if state is None:
                rospy.logwarn(
                    "ground_test=true：当前飞控状态消息不新鲜，但地面瞄准不依赖模式/定位"
                )
            else:
                rospy.logwarn(
                    "ground_test=true：忽略 armed=%s mode=%s，不要求 NED，不进入 GUIDED",
                    state.armed,
                    state.mode,
                )
            rospy.logwarn(
                "%s 地面流程直接进入持续瞄准循环；目标无效时只等待并清零计时",
                dropper,
            )
            return True

        state = self._fresh_state()
        if state is None or not state.connected:
            rospy.logerr("飞控状态无效，无法执行舵机测试")
            return False

        if not state.armed:
            rospy.logerr("current_state.armed == false，拒绝进入投放流程")
            return False
        if state.mode != "LOITER":
            rospy.logerr("当前模式不是 LOITER（实际为 %s），拒绝接管", state.mode)
            return False
        if self._fresh_pose() is None:
            rospy.logerr("本地位姿无效，无法确认 %.2fm 投放高度", self.target_drop_height_m)
            return False
        aim = self._fresh_aim()
        if aim is None or not getattr(
            aim, "{}_ned_valid".format(dropper.lower())
        ):
            if self.autonomous_mode:
                rospy.logwarn(
                    "autonomous_mode=true：进入 GUIDED 前暂无有效 %s 目标（%s）；"
                    "将从当前位置升高并搜索",
                    dropper,
                    self._aim_invalid_reason(dropper),
                )
                return True
            rospy.logerr(
                "未收到新鲜有效的 %s /vision/bucket/aim_info，拒绝开始瞄准",
                dropper,
            )
            return False
        return True

    def _ensure_set_mode_service(self):
        if self._set_mode_srv is not None:
            return True
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=10.0)
            self._set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            return True
        except rospy.ROSException as exc:
            rospy.logerr("/mavros/set_mode 不可用: %s", exc)
            return False

    def set_mode_confirmed(self, mode):
        if not self._ensure_set_mode_service():
            return False
        if mode == "GUIDED":
            self.guided_was_requested = True
        actual = "UNKNOWN"
        for attempt in range(1, self.service_retry_attempts + 1):
            try:
                response = self._set_mode_srv(base_mode=0, custom_mode=mode)
            except rospy.ServiceException as exc:
                rospy.logwarn(
                    "调用 set_mode(%s) 第 %d/%d 次失败: %s",
                    mode, attempt, self.service_retry_attempts, exc,
                )
                self._set_mode_srv = None
                if not self._ensure_set_mode_service():
                    continue
                response = None
            deadline = monotonic_s() + self.mode_timeout_s
            while monotonic_s() < deadline and not rospy.is_shutdown():
                state = self._fresh_state()
                if state is not None:
                    actual = state.mode
                    if actual == mode:
                        rospy.logwarn("已回读 /mavros/state，确认模式为 %s", mode)
                        if mode == "LOITER":
                            self.guided_was_requested = False
                        return True
                time.sleep(0.1)
            rospy.logwarn(
                "模式第 %d/%d 次未确认: expected=%s actual=%s mode_sent=%s",
                attempt, self.service_retry_attempts, mode, actual,
                getattr(response, "mode_sent", None),
            )
            if attempt < self.service_retry_attempts:
                rospy.sleep(self.service_retry_backoff_s)
        rospy.logerr("模式确认重试耗尽: expected=%s actual=%s", mode, actual)
        return False

    @staticmethod
    def _ned_to_enu(north, east, down):
        return float(east), float(north), -float(down)

    def _target_enu_from_ned(self, target_ned, height_m):
        x, y, surface_z = self._ned_to_enu(*target_ned)
        return x, y, surface_z + float(height_m)

    def _target_enu(self, aim, dropper):
        """返回抛投点水平坐标与配置高度语义对应的 ENU 位置目标。"""
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
        dz = max(
            -self.max_vertical_setpoint_distance_m,
            min(self.max_vertical_setpoint_distance_m, dz),
        )
        candidate = (
            float(current.x) + dx,
            float(current.y) + dy,
            float(current.z) + dz,
        )

        # 在“相对当前位置限幅”之外再限制 setpoint 随时间的移动速度，避免
        # COARSE->FINE 等阶段切换时目标高度瞬间跳变，引起快速升降和摆动。
        with self._lock:
            previous = self.last_setpoint_commanded_enu
            previous_at = self.last_setpoint_at
        if previous is None or previous_at <= 0.0:
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
        return (
            previous[0] + step_x,
            previous[1] + step_y,
            previous[2] + step_z,
        )

    def _publish_enu_setpoint(self, desired_enu, label, yaw=None):
        commanded = self._limit_setpoint_from_current_pose(desired_enu)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
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
        rospy.loginfo_throttle(
            0.5,
            "%s | desired ENU=(%.3f, %.3f, %.3f) limited=(%.3f, %.3f, %.3f)",
            label,
            desired_enu[0],
            desired_enu[1],
            desired_enu[2],
            commanded[0],
            commanded[1],
            commanded[2],
        )
        return commanded

    def _publish_ned_setpoint(self, target_ned, height_m, label):
        return self._publish_enu_setpoint(
            self._target_enu_from_ned(target_ned, height_m), label
        )

    def _publish_hold_setpoint(self, hold_enu, label):
        return self._publish_enu_setpoint(hold_enu, label)

    def _search_target_ned(self, center_ned, search_elapsed_s):
        points = (
            (0.0, 0.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (-1.0, 1.0),
            (-1.0, 0.0),
            (-1.0, -1.0),
            (0.0, -1.0),
            (1.0, -1.0),
        )
        step_index = int(max(0.0, search_elapsed_s) / self.search_dwell_s)
        ring = step_index // len(points)
        point = points[step_index % len(points)]
        radius = min(
            self.search_start_radius_m + ring * self.search_radius_step_m,
            self.search_max_radius_m,
        )
        return (
            center_ned[0] + point[0] * radius,
            center_ned[1] + point[1] * radius,
            center_ned[2],
        )

    def _publish_setpoint(self, aim, dropper):
        prefix = dropper.lower()
        if not getattr(aim, "{}_ned_valid".format(prefix)):
            rospy.logerr("%s NED 无效，拒绝发送 setpoint", dropper)
            return False
        north = getattr(aim, "{}_ned_n".format(prefix))
        east = getattr(aim, "{}_ned_e".format(prefix))
        down = getattr(aim, "{}_ned_d".format(prefix))
        target_ned = (float(north), float(east), float(down))
        x, y, z = self._target_enu_from_ned(
            target_ned, self.target_drop_height_m
        )
        self._publish_ned_setpoint(
            target_ned,
            self.target_drop_height_m,
            "{} 兼容投放 setpoint".format(dropper),
        )
        rospy.loginfo(
            "%s 桶目标 NED=(%.3f, %.3f, %.3f)，发送桶上方 %.2fm "
            "ENU setpoint=(%.3f, %.3f, %.3f)",
            dropper,
            north,
            east,
            down,
            self.target_drop_height_m,
            x,
            y,
            z,
        )
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
        siny_cosp = 2.0 * (
            float(quaternion.w) * float(quaternion.z)
            + float(quaternion.x) * float(quaternion.y)
        )
        cosy_cosp = 1.0 - 2.0 * (
            float(quaternion.y) ** 2 + float(quaternion.z) ** 2
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _validate_before_area_search(self, dropper):
        state = self._fresh_state()
        if state is None or not state.connected:
            rospy.logerr("%s 区域搜索前飞控状态无效", dropper)
            return False
        if not state.armed:
            rospy.logerr("%s 区域搜索前 armed=false，拒绝接管", dropper)
            return False
        if state.mode != "LOITER":
            rospy.logerr(
                "%s 区域搜索要求飞手先保持 LOITER，当前模式=%s",
                dropper,
                state.mode,
            )
            return False
        if self._fresh_pose() is None:
            rospy.logerr("%s 区域搜索前本地位姿无效", dropper)
            return False
        return True

    def wait_for_area_search_confirm(self, dropper):
        if not self._validate_before_area_search(dropper):
            return False
        anchor_text = "复用 A 搜索区域中心" if (
            dropper == "B"
            and self.reuse_area_search_anchor_for_b
            and self.area_search_anchor is not None
        ) else "把当前 LOITER 位置作为搜索区域中心"
        prompt = (
            "准备执行 {pattern} 字搜索 {dropper} 目标：{anchor}，区域 {width:.1f}m×"
            "{length:.1f}m。按 Enter 后请求 GUIDED；Ctrl+C 取消。"
        ).format(
            pattern=self.area_search_pattern,
            dropper=dropper,
            anchor=anchor_text,
            width=self.area_search_width_m,
            length=self.area_search_length_m,
        )
        rospy.logwarn(prompt)
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
                return bool(stream.readline()) and not rospy.is_shutdown()
            finally:
                if tty is not None:
                    tty.close()
        except (EOFError, KeyboardInterrupt, OSError):
            return False

    def _prepare_area_search_anchor(self, dropper):
        if (
            dropper == "B"
            and self.reuse_area_search_anchor_for_b
            and self.area_search_anchor is not None
        ):
            rospy.logwarn("B 搜索复用 A 已冻结的区域中心和机头方向")
            return True
        pose = self._fresh_pose()
        if pose is None:
            rospy.logerr("%s 无法冻结搜索区域：本地位姿不新鲜", dropper)
            return False
        position = pose.pose.position
        yaw = self._yaw_from_quaternion(pose.pose.orientation)
        values = (float(position.x), float(position.y), float(position.z), yaw)
        if not all(math.isfinite(value) for value in values):
            rospy.logerr("%s 无法冻结搜索区域：位姿或航向包含非有限值", dropper)
            return False
        self.area_search_anchor = values
        rospy.logwarn(
            "%s 搜索区域已冻结 | center_ENU=(%.3f, %.3f, %.3f) yaw=%.1fdeg "
            "pattern=%s size=%.1fx%.1fm footprint=%.2fx%.2fm",
            dropper,
            values[0],
            values[1],
            values[2],
            math.degrees(values[3]),
            self.area_search_pattern,
            self.area_search_width_m,
            self.area_search_length_m,
            self.area_search_footprint_width_m,
            self.area_search_footprint_length_m,
        )
        return True

    def _area_search_local_points(self):
        lateral = 0.5 * (
            self.area_search_width_m - self.area_search_footprint_width_m
        )
        forward = 0.5 * (
            self.area_search_length_m - self.area_search_footprint_length_m
        )
        if self.area_search_pattern == "I":
            # 以启动航向为前向，right<0 为左。先从中心飞到有效搜索宽度左端，
            # 再横穿到右端；相机覆盖宽度用于给区域左右边界留出半幅覆盖。
            return (
                (0.0, -lateral),
                (0.0, lateral),
            )
        if self.area_search_pattern == "W":
            return (
                (-forward, -lateral),
                (-forward / 3.0, lateral),
                (forward / 3.0, -lateral),
                (forward, lateral),
            )
        return (
            (-forward, -lateral),
            (-forward, lateral),
            (forward, -lateral),
            (forward, lateral),
        )

    def _area_search_enu_points(self):
        if self.area_search_anchor is None:
            raise ValueError("搜索区域中心尚未冻结")
        center_e, center_n, center_u, _yaw = self.area_search_anchor
        center_d = -center_u
        heading = math.radians(self.start_heading_deg)
        points = []
        for forward, right in self._area_search_local_points():
            # 先在冻结的 NED 搜索区内构造路径点：前向沿启动航向，右向为其顺时针 90°。
            north = center_n + forward * math.cos(heading) - right * math.sin(heading)
            east = center_e + forward * math.sin(heading) + right * math.cos(heading)
            down = center_d
            # 发布 MAVROS local ENU 前显式转换：ENU.x=E, ENU.y=N, ENU.z=-D。
            points.append((east, north, -down))
        return tuple(points)

    def _flight_only_local_health(self):
        """检查仅飞行本地航线需要的全部飞行遥测及其有界恢复条件。"""
        nav = self._navigation_snapshot()
        control = self._control_stream_snapshot()
        health, state_reason = self._flight_state_health()
        if health == "fatal":
            return "fatal", state_reason, nav, control, 0.0

        stale_reasons = []
        grace = 0.0
        if health == "recover":
            stale_reasons.append("STATE {}".format(state_reason))
            grace = max(
                grace,
                self.connection_loss_grace_s
                if "connected" in state_reason
                else self.guided_state_grace_s,
            )

        local_fields = (
            ("POSE", nav["pose"], nav["pose_age"]),
            ("VELOCITY", control["velocity"], control["velocity_age"]),
        )
        for name, message, age in local_fields:
            if message is not None and age <= self.pose_timeout_s:
                continue
            if message is None or age > self.guided_pose_grace_s:
                return (
                    "fatal",
                    "{} 超过恢复宽限 age={:.2f}s grace={:.2f}s".format(
                        name, age, self.guided_pose_grace_s
                    ),
                    nav,
                    control,
                    0.0,
                )
            stale_reasons.append("{}={:.2f}s".format(name, age))
            grace = max(grace, self.guided_pose_grace_s)

        navigation_fields = (
            ("GPS", nav["fix"], nav["fix_age"]),
            ("HEADING", nav["heading"], nav["heading_age"]),
            ("REL_ALT", nav["rel_alt"], nav["rel_alt_age"]),
        )
        for name, value, age in navigation_fields:
            if value is not None and age <= self.navigation_timeout_s:
                continue
            if value is None or age > self.navigation_grace_s:
                return (
                    "fatal",
                    "{} 超过恢复宽限 age={:.2f}s grace={:.2f}s".format(
                        name, age, self.navigation_grace_s
                    ),
                    nav,
                    control,
                    0.0,
                )
            stale_reasons.append("{}={:.2f}s".format(name, age))
            grace = max(grace, self.navigation_grace_s)

        if stale_reasons:
            return "recover", " + ".join(stale_reasons), nav, control, grace
        return "ok", "", nav, control, 0.0

    def fly_area_pattern_only(self):
        """完整飞行指定圈数的 W/Z/I 航线，绝不读取或响应视觉/弹药状态。"""
        if not self.flight_only_mode:
            rospy.logerr("fly_area_pattern_only 仅允许在 flight_only_mode=true 时调用")
            return False
        if self.ground_relative_altitude is None or self.start_heading_deg is None:
            rospy.logerr("FLIGHT ONLY 航线缺少地面 rel_alt 或冻结启动航向")
            return False

        health, reason, nav, _control, _grace = self._flight_only_local_health()
        if health != "ok":
            rospy.logerr("FLIGHT ONLY 航线启动前遥测无效: %s", reason)
            return False
        current_enu = self._pose_position_enu(nav["pose"])
        if current_enu is None:
            rospy.logerr("FLIGHT ONLY 无法读取抵达投放区时的本地 ENU")
            return False

        # 全局航点循环已经结束。先用当前本地位置建立保持，再开始有界垂直移动。
        self._publish_hold_setpoint(current_enu, "FLIGHT_ONLY GLOBAL_TO_LOCAL HOLD")
        rospy.sleep(self.setpoint_interval_s)

        target_rel_alt = (
            self.ground_relative_altitude + self.flight_only_search_altitude_m
        )
        search_center_z = current_enu[2] + (target_rel_alt - float(nav["rel_alt"]))
        yaw_enu = math.radians(90.0 - self.start_heading_deg)
        self.area_search_anchor = (
            current_enu[0],
            current_enu[1],
            search_center_z,
            yaw_enu,
        )
        points = self._area_search_enu_points()
        self.flight_only_total_points = len(points)
        self.flight_only_current_lap = 0
        self.flight_only_current_point = 0
        rospy.logwarn(
            "FLIGHT ONLY 区域航线中心已冻结 | center_ENU=(%.3f, %.3f) "
            "arrival_ENU_z=%.3f target_ENU_z=%.3f target_rel_alt=%.3f "
            "start_heading=%.2fdeg pattern=%s points=%d laps=%d",
            current_enu[0],
            current_enu[1],
            current_enu[2],
            search_center_z,
            target_rel_alt,
            self.start_heading_deg,
            self.area_search_pattern,
            len(points),
            self.flight_only_pattern_laps,
        )

        rate = rospy.Rate(10)
        altitude_phase = True
        lap_index = 0
        point_index = 0
        inside_since = None
        next_setpoint_at = 0.0
        recovery_started = None
        recovery_hold = None

        while not rospy.is_shutdown():
            now = monotonic_s()
            if not self._restart_dead_children():
                rospy.logerr("FLIGHT ONLY 航线期间依赖有界重启失败")
                return False

            health, reason, nav, control, grace = self._flight_only_local_health()
            if health == "fatal":
                rospy.logerr("FLIGHT ONLY 航线安全退出: %s", reason)
                return False
            if health == "recover":
                if recovery_started is None:
                    recovery_started = now
                    recovery_hold = self._safe_hold_enu(
                        control, fallback=self.area_search_anchor[:3]
                    )
                    if recovery_hold is None:
                        rospy.logerr("FLIGHT ONLY 遥测恢复没有可用安全保持 setpoint")
                        return False
                    self._begin_recovery("FLIGHT_ONLY_PATTERN: {}".format(reason))
                elapsed = now - recovery_started
                if elapsed > grace:
                    rospy.logerr(
                        "FLIGHT ONLY 遥测恢复超时 %.2f/%.2fs: %s",
                        elapsed,
                        grace,
                        reason,
                    )
                    return False
                inside_since = None
                if now >= next_setpoint_at:
                    self._publish_hold_setpoint(
                        recovery_hold, "FLIGHT_ONLY_PATTERN RECOVERY HOLD"
                    )
                    next_setpoint_at = now + self.setpoint_interval_s
                state = control["state"]
                rospy.logwarn_throttle(
                    1.0,
                    "FLIGHT ONLY RECOVERY | reason=%s elapsed=%.2f/%.2fs "
                    "state_age=%.2fs pose_age=%.2fs velocity_age=%.2fs "
                    "mode=%s RC=%s hold_ENU=(%.3f,%.3f,%.3f)",
                    reason,
                    elapsed,
                    grace,
                    control["state_age"],
                    control["pose_age"],
                    control["velocity_age"],
                    state.mode if state is not None else "NONE",
                    self._rc_summary(),
                    recovery_hold[0],
                    recovery_hold[1],
                    recovery_hold[2],
                )
                rate.sleep()
                continue

            if recovery_started is not None:
                rospy.logwarn(
                    "FLIGHT ONLY 遥测恢复，继续圈 %d/%d 点 %d/%d；到达停留重新计时",
                    self.flight_only_current_lap,
                    self.flight_only_pattern_laps,
                    self.flight_only_current_point,
                    self.flight_only_total_points,
                )
                recovery_started = None
                recovery_hold = None
                inside_since = None
                self._end_recovery("FLIGHT_ONLY_PATTERN 遥测恢复")

            if altitude_phase:
                target_enu = (
                    self.area_search_anchor[0],
                    self.area_search_anchor[1],
                    self.area_search_anchor[2],
                )
                phase_text = "ALTITUDE"
                shown_lap = 0
                shown_point = 0
            else:
                if lap_index >= self.flight_only_pattern_laps:
                    self.flight_only_current_lap = self.flight_only_pattern_laps
                    self.flight_only_current_point = self.flight_only_total_points
                    rospy.logwarn(
                        "FLIGHT ONLY %s 字航线全部完成，共 %d 圈",
                        self.area_search_pattern,
                        self.flight_only_pattern_laps,
                    )
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
                        self.area_search_pattern,
                        phase_text,
                        shown_lap,
                        self.flight_only_pattern_laps,
                        shown_point,
                        self.flight_only_total_points,
                    ),
                    yaw=self.area_search_anchor[3],
                )
                next_setpoint_at = now + self.setpoint_interval_s

            current_enu = self._pose_position_enu(nav["pose"])
            if current_enu is None:
                rospy.logerr("FLIGHT ONLY 收到非有限本地 ENU")
                return False
            horizontal_error = math.hypot(
                target_enu[0] - current_enu[0], target_enu[1] - current_enu[1]
            )
            vertical_error = abs(float(nav["rel_alt"]) - target_rel_alt)
            inside = (
                horizontal_error <= self.area_search_waypoint_tolerance_m
                and vertical_error <= self.area_search_height_tolerance_m
            )
            if inside:
                if inside_since is None:
                    inside_since = now
                if now - inside_since >= self.area_search_dwell_s:
                    if altitude_phase:
                        altitude_phase = False
                        inside_since = None
                        rospy.logwarn(
                            "FLIGHT ONLY 已到区域航线高度：rel_alt=%.3f target=%.3f；"
                            "开始第一段",
                            nav["rel_alt"],
                            target_rel_alt,
                        )
                    else:
                        rospy.logwarn(
                            "FLIGHT ONLY %s | 圈 %d/%d 到达点 %d/%d，"
                            "horizontal=%.3fm vertical=%.3fm dwell=%.2fs",
                            self.area_search_pattern,
                            shown_lap,
                            self.flight_only_pattern_laps,
                            shown_point,
                            self.flight_only_total_points,
                            horizontal_error,
                            vertical_error,
                            self.area_search_dwell_s,
                        )
                        point_index += 1
                        inside_since = None
                        if point_index >= len(points):
                            lap_index += 1
                            rospy.logwarn(
                                "FLIGHT ONLY %s 字第 %d/%d 圈完整完成",
                                self.area_search_pattern,
                                lap_index,
                                self.flight_only_pattern_laps,
                            )
                            point_index = 0
            else:
                inside_since = None

            state = control["state"]
            rospy.loginfo_throttle(
                1.0,
                "FLIGHT ONLY TRACE | pattern=%s phase=%s lap=%d/%d point=%d/%d "
                "target_ENU=(%.3f,%.3f,%.3f) current_ENU=(%.3f,%.3f,%.3f) "
                "horizontal_error=%.3fm vertical_error=%.3fm "
                "state_age=%.3fs pose_age=%.3fs velocity_age=%.3fs mode=%s RC=%s",
                self.area_search_pattern,
                phase_text,
                shown_lap,
                self.flight_only_pattern_laps,
                shown_point,
                self.flight_only_total_points,
                target_enu[0],
                target_enu[1],
                target_enu[2],
                current_enu[0],
                current_enu[1],
                current_enu[2],
                horizontal_error,
                vertical_error,
                control["state_age"],
                control["pose_age"],
                control["velocity_age"],
                state.mode if state is not None else "NONE",
                self._rc_summary(),
            )
            rate.sleep()
        return False

    def area_search_for_target(self, dropper):
        if self.flight_only_mode:
            rospy.logerr("FLIGHT ONLY 禁止进入目标区域搜索")
            return False
        points = self._area_search_enu_points()
        rate = rospy.Rate(10)
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

        rospy.logwarn(
            "%s 开始 %s 字区域搜索，共 %d 个路径点；航段中持续检测目标",
            dropper,
            self.area_search_pattern,
            len(points),
        )
        while not rospy.is_shutdown():
            now = monotonic_s()
            if not self._restart_dead_children():
                rospy.logerr("%s 区域搜索期间依赖重启失败", dropper)
                return False
            phase_label = "AREA_{}".format(self.area_search_pattern)
            state = self._fresh_state()
            pose = self._fresh_pose()
            snapshot = self._control_stream_snapshot()
            stale_reasons = []
            fatal_reason = None
            if state is None:
                raw_state = snapshot["state"]
                state_grace = (
                    self.connection_loss_grace_s
                    if raw_state is not None and not raw_state.connected
                    else self.guided_state_grace_s
                )
                if not (
                    raw_state is not None
                    and raw_state.armed
                    and raw_state.mode == "GUIDED"
                    and snapshot["state_age"] <= state_grace
                ):
                    fatal_reason = (
                        "区域搜索 /mavros/state 恢复失败：last_mode={} "
                        "connected={} armed={} age={:.3f}s grace={:.3f}s"
                    ).format(
                        raw_state.mode if raw_state is not None else "NONE",
                        raw_state.connected if raw_state is not None else None,
                        raw_state.armed if raw_state is not None else None,
                        snapshot["state_age"],
                        self.guided_state_grace_s,
                    )
                else:
                    stale_reasons.append(
                        "STATE age={:.3f}/{:.3f}s".format(
                            snapshot["state_age"], self.guided_state_grace_s
                        )
                    )
            elif not state.armed or state.mode != "GUIDED":
                fatal_reason = (
                    "区域搜索安全状态失效：connected={} armed={} mode={}"
                ).format(state.connected, state.armed, state.mode)
            elif not state.connected:
                stale_reasons.append("CONNECTION connected=false")

            if pose is None:
                if (
                    snapshot["pose"] is None
                    or snapshot["pose_age"] > self.guided_pose_grace_s
                ):
                    fatal_reason = (
                        "区域搜索本地位姿恢复失败：pose_age={:.3f}s "
                        "timeout={:.3f}s grace={:.3f}s"
                    ).format(
                        snapshot["pose_age"],
                        self.pose_timeout_s,
                        self.guided_pose_grace_s,
                    )
                else:
                    stale_reasons.append(
                        "POSE age={:.3f}/{:.3f}s".format(
                            snapshot["pose_age"], self.guided_pose_grace_s
                        )
                    )

            if fatal_reason is not None:
                self._log_control_diagnostics(
                    dropper, phase_label, candidate_ned, fatal_reason
                )
                return False

            if stale_reasons:
                if telemetry_stale_started_at is None:
                    telemetry_stale_started_at = now
                    telemetry_stale_reason = " + ".join(stale_reasons)
                    self._begin_recovery("{} 区域搜索: {}".format(dropper, telemetry_stale_reason))
                    telemetry_stale_hold_enu = self._safe_hold_enu(
                        snapshot, fallback=self.area_search_anchor[:3]
                    )
                    if telemetry_stale_hold_enu is None:
                        self._log_control_diagnostics(
                            dropper,
                            phase_label,
                            candidate_ned,
                            "区域搜索遥测短暂不新鲜，但没有可安全保持的 ENU setpoint",
                        )
                        return False
                stale_grace = (
                    self.connection_loss_grace_s
                    if any("CONNECTION" in reason for reason in stale_reasons)
                    else max(self.guided_state_grace_s, self.guided_pose_grace_s)
                )
                if now - telemetry_stale_started_at > stale_grace:
                    self._log_control_diagnostics(
                        dropper, phase_label, candidate_ned,
                        "区域搜索恢复超时 {:.1f}/{:.1f}s: {}".format(
                            now - telemetry_stale_started_at, stale_grace, " + ".join(stale_reasons)
                        ),
                    )
                    return False
                    rospy.logwarn(
                        "%s 区域搜索遥测短暂不新鲜（%s）；冻结搜索计时并保持最后安全位置，"
                        "STATE/POSE 宽限分别为 %.1f/%.1fs",
                        dropper,
                        telemetry_stale_reason,
                        self.guided_state_grace_s,
                        self.guided_pose_grace_s,
                    )
                point_inside_since = None
                candidate_valid_since = None
                candidate_ned = None
                candidate_hold_enu = None
                if now >= next_setpoint_at:
                    self._publish_enu_setpoint(
                        telemetry_stale_hold_enu,
                        "{} AREA_{} TELEMETRY_STALE 保持".format(
                            dropper, self.area_search_pattern
                        ),
                        yaw=self.area_search_anchor[3],
                    )
                    next_setpoint_at = now + self.setpoint_interval_s
                rospy.logwarn_throttle(
                    0.5,
                    "%s AREA_%s TELEMETRY_STALE 恢复等待 | %s | "
                    "保持ENU=(%.3f, %.3f, %.3f)",
                    dropper,
                    self.area_search_pattern,
                    " + ".join(stale_reasons),
                    telemetry_stale_hold_enu[0],
                    telemetry_stale_hold_enu[1],
                    telemetry_stale_hold_enu[2],
                )
                self._publish_status(aiming=True)
                self._log_control_diagnostics(dropper, phase_label, candidate_ned)
                rate.sleep()
                continue

            if telemetry_stale_started_at is not None:
                stale_duration = now - telemetry_stale_started_at
                started_at += stale_duration
                rospy.logwarn(
                    "%s 区域搜索遥测已恢复，停滞 %.2fs；继续原路径点 %d/%d，"
                    "搜索/停留/候选稳定计时重新开始",
                    dropper,
                    stale_duration,
                    point_index + 1,
                    len(points),
                )
                telemetry_stale_started_at = None
                telemetry_stale_hold_enu = None
                telemetry_stale_reason = None
                self._end_recovery("{} 区域搜索遥测恢复".format(dropper))

            elapsed = now - started_at
            if elapsed >= self.area_search_timeout_s:
                rospy.logerr(
                    "%s %s 字区域搜索超时 %.1f/%.1fs，未获得稳定目标",
                    dropper,
                    self.area_search_pattern,
                    elapsed,
                    self.area_search_timeout_s,
                )
                return False

            aim = self._fresh_aim()
            measured_ned = self._aim_target_ned(aim, dropper)
            accepted = False
            if measured_ned is not None:
                candidate_ned, accepted = self._update_target_lock(
                    candidate_ned, measured_ned, dropper
                )
            if accepted:
                if candidate_valid_since is None:
                    candidate_valid_since = now
                    candidate_hold_enu = (
                        float(pose.pose.position.x),
                        float(pose.pose.position.y),
                        float(pose.pose.position.z),
                    )
                    rospy.logwarn(
                        "%s 区域搜索发现候选目标，停止航线并保持当前位置确认",
                        dropper,
                    )
                stable = now - candidate_valid_since
                if now >= next_setpoint_at:
                    self._publish_enu_setpoint(
                        candidate_hold_enu,
                        "{} AREA_{} 目标确认保持".format(
                            dropper, self.area_search_pattern
                        ),
                        yaw=self.area_search_anchor[3],
                    )
                    next_setpoint_at = now + self.setpoint_interval_s
                rospy.logwarn_throttle(
                    0.5,
                    "%s 候选目标稳定 %.1f/%.1fs",
                    dropper,
                    stable,
                    self.acquire_stable_time_s,
                )
                self._publish_status(aiming=True)
                if stable >= self.acquire_stable_time_s:
                    rospy.logwarn(
                        "%s 区域搜索目标连续稳定 %.1fs，转入分层瞄准",
                        dropper,
                        stable,
                    )
                    return True
                rate.sleep()
                continue

            if candidate_valid_since is not None:
                rospy.logwarn("%s 候选目标确认期间失效，恢复原搜索航线", dropper)
            candidate_ned = None
            candidate_valid_since = None
            candidate_hold_enu = None

            if point_index >= len(points):
                rospy.logwarn(
                    "%s %s 字航线一轮完成但未找到稳定目标，返回第一个路径点继续搜索至超时",
                    dropper,
                    self.area_search_pattern,
                )
                point_index = 0
                point_inside_since = None

            target_enu = points[point_index]
            if now >= next_setpoint_at:
                self._publish_enu_setpoint(
                    target_enu,
                    "{} AREA_{} 点 {}/{}".format(
                        dropper,
                        self.area_search_pattern,
                        point_index + 1,
                        len(points),
                    ),
                    yaw=self.area_search_anchor[3],
                )
                next_setpoint_at = now + self.setpoint_interval_s

            position = pose.pose.position
            horizontal_error = math.hypot(
                target_enu[0] - float(position.x),
                target_enu[1] - float(position.y),
            )
            vertical_error = abs(target_enu[2] - float(position.z))
            inside = (
                horizontal_error <= self.area_search_waypoint_tolerance_m
                and vertical_error <= self.area_search_height_tolerance_m
            )
            if inside:
                if point_inside_since is None:
                    point_inside_since = now
                if now - point_inside_since >= self.area_search_dwell_s:
                    rospy.logwarn(
                        "%s %s 字搜索到达点 %d/%d，继续下一航段",
                        dropper,
                        self.area_search_pattern,
                        point_index + 1,
                        len(points),
                    )
                    point_index += 1
                    point_inside_since = None
            else:
                point_inside_since = None

            rospy.loginfo_throttle(
                0.5,
                "%s AREA_%s | 点=%d/%d 平面误差=%.2fm 高度误差=%.2fm "
                "已搜索=%.1f/%.1fs",
                dropper,
                self.area_search_pattern,
                min(point_index + 1, len(points)),
                len(points),
                horizontal_error,
                vertical_error,
                elapsed,
                self.area_search_timeout_s,
            )
            self._publish_status(aiming=True)
            self._log_control_diagnostics(
                dropper,
                "AREA_{}_{}".format(
                    self.area_search_pattern, min(point_index + 1, len(points))
                ),
                candidate_ned,
            )
            rate.sleep()
        return False

    def aim_dropper(self, dropper):
        if self.flight_only_mode:
            rospy.logerr("FLIGHT ONLY 禁止进入 A/B 瞄准")
            return False
        prefix = dropper.lower()
        final_aligned_since = None
        rate = rospy.Rate(10)
        self.frozen_target_ned = None
        self.forced_drop = False

        if self.ground_test:
            rospy.logwarn(
                "开始 %s 抛投器地面瞄准：目标丢失或偏差超限只会清零计时，不会退出",
                dropper,
            )
            while not rospy.is_shutdown():
                aim = self._fresh_aim()
                if aim is None:
                    final_aligned_since = None
                    self._publish_status(aiming=True)
                    rospy.logwarn_throttle(
                        1.0,
                        "%s 地面瞄准目标无效：%s；连续计时已清零",
                        dropper,
                        self._aim_invalid_reason(),
                    )
                    rate.sleep()
                    continue
                now = monotonic_s()
                dx = float(getattr(aim, "{}_delta_x_m".format(prefix)))
                dy = float(getattr(aim, "{}_delta_y_m".format(prefix)))
                aligned = (
                    abs(dx) < self.align_threshold_m
                    and abs(dy) < self.align_threshold_m
                )
                if aligned:
                    if final_aligned_since is None:
                        final_aligned_since = now
                    stable = now - final_aligned_since
                else:
                    final_aligned_since = None
                    stable = 0.0
                self._publish_status(aiming=True)
                rospy.loginfo_throttle(
                    0.5,
                    "地面 %s 瞄准 | X偏差=%+.3fm Y偏差=%+.3fm 稳定=%.1f/%.1fs",
                    dropper,
                    dx,
                    dy,
                    stable,
                    self.stable_time_s,
                )
                if aligned and stable >= self.stable_time_s:
                    rospy.logwarn("%s 连续对准 %.1fs，瞄准完成", dropper, stable)
                    self._publish_status(aiming=False)
                    return True
                rate.sleep()
            return False

        control_mode = "AUTONOMOUS" if self.autonomous_mode else "SEMI_AUTO"
        rospy.logwarn(
            "开始 %s 分层瞄准 | control_mode=%s COARSE=%.2fm FINE=%.2fm DROP=%.2fm",
            dropper,
            control_mode,
            self.coarse_align_height_m,
            self.fine_align_height_m,
            self.target_drop_height_m,
        )
        phase = "COARSE"
        phase_aligned_since = None
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
        aim_elapsed_s = 0.0
        aim_timer_started = False
        aim_active_last = None

        start_pose = self._fresh_pose()
        if start_pose is None:
            rospy.logerr("%s 开始瞄准时没有本地位姿", dropper)
            return False
        search_center_ned = (
            float(start_pose.pose.position.y),
            float(start_pose.pose.position.x),
            float(-start_pose.pose.position.z),
        )
        if self.autonomous_mode:
            loss_started_at = monotonic_s()
            loss_hold_enu = (
                float(start_pose.pose.position.x),
                float(start_pose.pose.position.y),
                float(start_pose.pose.position.z),
            )
            rospy.logwarn(
                "%s 自主 ACQUIRE：先保持当前位置，目标连续有效 %.1fs 后开始移动",
                dropper,
                self.acquire_stable_time_s,
            )
        if dropper == "B" and self.completed_drop_targets:
            search_center_ned = self.completed_drop_targets[-1]

        while not rospy.is_shutdown():
            now = monotonic_s()
            if not self._restart_dead_children():
                rospy.logerr("%s 瞄准期间依赖重启失败", dropper)
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
                state_grace = (
                    self.connection_loss_grace_s
                    if raw_state is not None and not raw_state.connected
                    else self.guided_state_grace_s
                )
                if not (
                    raw_state is not None
                    and raw_state.armed
                    and raw_state.mode == "GUIDED"
                    and raw_state_age <= state_grace
                ):
                    fatal_reason = (
                        "/mavros/state 恢复失败：last_mode={} connected={} armed={} "
                        "age={:.3f}s timeout={:.3f}s grace={:.3f}s"
                    ).format(
                        raw_mode,
                        raw_state.connected if raw_state is not None else None,
                        raw_state.armed if raw_state is not None else None,
                        raw_state_age,
                        self.state_timeout_s,
                        self.guided_state_grace_s,
                    )
                else:
                    stale_reasons.append(
                        "STATE age={:.3f}/{:.3f}s".format(
                            raw_state_age, self.guided_state_grace_s
                        )
                    )
            elif state.mode != "GUIDED":
                fatal_reason = (
                    "收到新鲜状态但实际模式不是 GUIDED：mode={} connected={} "
                    "armed={} system_status={}"
                ).format(
                    state.mode,
                    state.connected,
                    state.armed,
                    state.system_status,
                )
            elif not state.armed:
                fatal_reason = "瞄准期间 armed 变为 false"
            elif not state.connected:
                stale_reasons.append("CONNECTION connected=false")

            if pose is None:
                if (
                    snapshot["pose"] is None
                    or snapshot["pose_age"] > self.guided_pose_grace_s
                ):
                    fatal_reason = (
                        "本地位姿恢复失败：pose_age={:.3f}s timeout={:.3f}s "
                        "grace={:.3f}s"
                    ).format(
                        snapshot["pose_age"],
                        self.pose_timeout_s,
                        self.guided_pose_grace_s,
                    )
                else:
                    stale_reasons.append(
                        "POSE age={:.3f}/{:.3f}s".format(
                            snapshot["pose_age"], self.guided_pose_grace_s
                        )
                    )
            if velocity is None:
                if (
                    snapshot["velocity"] is None
                    or snapshot["velocity_age"] > self.guided_pose_grace_s
                ):
                    fatal_reason = (
                        "本地速度恢复失败：velocity_age={:.3f}s timeout={:.3f}s "
                        "grace={:.3f}s"
                    ).format(
                        snapshot["velocity_age"],
                        self.pose_timeout_s,
                        self.guided_pose_grace_s,
                    )
                else:
                    stale_reasons.append(
                        "VEL age={:.3f}/{:.3f}s".format(
                            snapshot["velocity_age"], self.guided_pose_grace_s
                        )
                    )

            if fatal_reason is not None:
                self._log_control_diagnostics(
                    dropper, phase, locked_ned, fatal_reason
                )
                return False

            if stale_reasons:
                aim_active_last = None
                if telemetry_stale_started_at is None:
                    telemetry_stale_started_at = now
                    telemetry_stale_reason = " + ".join(stale_reasons)
                    self._begin_recovery("{} 瞄准: {}".format(dropper, telemetry_stale_reason))
                    telemetry_stale_hold_enu = self._safe_hold_enu(
                        snapshot, fallback=self._pose_position_enu(start_pose)
                    )
                    if telemetry_stale_hold_enu is None:
                        self._log_control_diagnostics(
                            dropper,
                            phase,
                            locked_ned,
                            "遥测短暂不新鲜，但没有可安全保持的 ENU setpoint",
                        )
                        return False
                stale_grace = (
                    self.connection_loss_grace_s
                    if any("CONNECTION" in reason for reason in stale_reasons)
                    else max(self.guided_state_grace_s, self.guided_pose_grace_s)
                )
                if now - telemetry_stale_started_at > stale_grace:
                    self._log_control_diagnostics(
                        dropper, phase, locked_ned,
                        "瞄准恢复超时 {:.1f}/{:.1f}s: {}".format(
                            now - telemetry_stale_started_at, stale_grace, " + ".join(stale_reasons)
                        ),
                    )
                    return False
                    rospy.logwarn(
                        "%s 遥测短暂不新鲜（%s）；冻结 %s 阶段并保持最后安全位置，"
                        "STATE/POSE 宽限分别为 %.1f/%.1fs",
                        dropper,
                        telemetry_stale_reason,
                        phase,
                        self.guided_state_grace_s,
                        self.guided_pose_grace_s,
                    )
                phase_aligned_since = None
                final_aligned_since = None
                reacquire_valid_since = None
                if now >= next_setpoint_at:
                    self._publish_hold_setpoint(
                        telemetry_stale_hold_enu,
                        "{} TELEMETRY_STALE 保持".format(dropper),
                    )
                    next_setpoint_at = now + self.setpoint_interval_s
                rospy.logwarn_throttle(
                    0.5,
                    "%s TELEMETRY_STALE 恢复等待 | %s | "
                    "保持ENU=(%.3f, %.3f, %.3f)",
                    dropper,
                    " + ".join(stale_reasons),
                    telemetry_stale_hold_enu[0],
                    telemetry_stale_hold_enu[1],
                    telemetry_stale_hold_enu[2],
                )
                self._publish_status(aiming=True)
                self._log_control_diagnostics(dropper, phase, locked_ned)
                rate.sleep()
                continue

            if telemetry_stale_started_at is not None:
                stale_duration = now - telemetry_stale_started_at
                if loss_started_at is not None:
                    loss_started_at += stale_duration
                rospy.logwarn(
                    "%s 遥测已恢复，停滞 %.2fs；保持原阶段 %s，稳定计时重新开始",
                    dropper,
                    stale_duration,
                    phase,
                )
                telemetry_stale_started_at = None
                telemetry_stale_hold_enu = None
                telemetry_stale_reason = None
                self._end_recovery("{} 瞄准遥测恢复，继续 {}".format(dropper, phase))

            aim = self._fresh_aim()
            measured_ned = self._aim_target_ned(aim, dropper)
            accepted = False
            loss_reason = self._aim_invalid_reason(dropper)
            if self.frozen_target_ned is not None:
                locked_ned = self.frozen_target_ned
                accepted = measured_ned is not None
                if accepted and self._target_is_blacklisted(measured_ned, dropper):
                    accepted = False
                    loss_reason = "当前观测命中已完成目标黑名单"
                elif accepted and self._ned_xy_distance(measured_ned, self.frozen_target_ned) > self.max_target_jump_m:
                    accepted = False
                    loss_reason = "冻结后观测与 frozen NED 距离过大，拒绝覆盖/误用"
            elif measured_ned is not None:
                # 长时间搜索后允许用重新捕获的观测重置旧锁定，避免一次错误初值
                # 使后续真实目标持续被当作跳变拒绝。
                update_base = locked_ned
                if (
                    loss_started_at is not None
                    and now - loss_started_at >= self.loss_hover_s
                    and not relock_reset_done
                    and not self._target_is_blacklisted(measured_ned, dropper)
                ):
                    update_base = None
                    relock_reset_done = True
                locked_ned, accepted = self._update_target_lock(
                    update_base, measured_ned, dropper
                )
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
                            reacquire_stable, self.acquire_stable_time_s
                        )
            else:
                reacquire_valid_since = None

            if not accepted:
                aim_active_last = None
                final_aligned_since = None
                phase_aligned_since = None
                if loss_started_at is None:
                    loss_started_at = now
                    relock_reset_done = False
                    loss_hold_enu = (
                        float(pose.pose.position.x),
                        float(pose.pose.position.y),
                        float(pose.pose.position.z),
                    )
                    rospy.logwarn(
                        "%s 目标暂时失效：%s；立即保持当前位置并进入重捕获流程",
                        dropper,
                        loss_reason,
                    )
                loss_elapsed = now - loss_started_at
                if loss_elapsed >= self.target_loss_timeout_s:
                    rospy.logerr(
                        "%s 搜索目标已持续 %.1fs，超过 %.1fs，自主任务超时",
                        dropper,
                        loss_elapsed,
                        self.target_loss_timeout_s,
                    )
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
                            self.frozen_target_ned,
                            frozen_height,
                            "{} 冻结目标丢失继续安全趋近".format(dropper),
                        )
                    elif loss_elapsed < self.loss_hover_s:
                        self._publish_hold_setpoint(
                            loss_hold_enu,
                            "{} 目标丢失保持 {:.1f}s".format(dropper, loss_elapsed),
                        )
                    else:
                        center = locked_ned or search_center_ned
                        search_elapsed = loss_elapsed - self.loss_hover_s
                        search_ned = self._search_target_ned(center, search_elapsed)
                        self._publish_ned_setpoint(
                            search_ned,
                            self.reacquire_height_m,
                            "{} 升高局部搜索 {:.1f}s".format(dropper, search_elapsed),
                        )
                    next_setpoint_at = now + self.setpoint_interval_s
                rospy.logwarn_throttle(
                    1.0,
                    "%s 重捕获 | 原因=%s 丢失=%.1f/%.1fs 阶段=%s",
                    dropper,
                    loss_reason,
                    loss_elapsed,
                    self.target_loss_timeout_s,
                    "保持" if loss_elapsed < self.loss_hover_s else "升高搜索",
                )
                self._publish_status(aiming=True)
                self._log_control_diagnostics(dropper, phase, locked_ned)
                rate.sleep()
                continue

            if loss_started_at is not None:
                loss_duration = now - loss_started_at
                rospy.logwarn("%s 目标重新捕获，丢失持续 %.2fs", dropper, loss_duration)
                if loss_duration >= self.brief_loss_s:
                    phase = "COARSE"
                    phase_aligned_since = None
                    rospy.logwarn("%s 重捕获后回到 COARSE 阶段", dropper)
                loss_started_at = None
                loss_hold_enu = None
                reacquire_valid_since = None
                relock_reset_done = False
            search_center_ned = locked_ned

            if not aim_timer_started:
                aim_timer_started = True
                rospy.logwarn("%s 获得合法目标并进入 COARSE，开始累计 %.1fs 瞄准超时", dropper, self.aim_timeout_s)
            if aim_active_last is not None:
                aim_elapsed_s += max(0.0, now - aim_active_last)
            aim_active_last = now

            dx = float(getattr(aim, "{}_delta_x_m".format(prefix)))
            dy = float(getattr(aim, "{}_delta_y_m".format(prefix)))

            if self.frozen_target_ned is None:
                if (
                    abs(dx) < self.target_lock_center_threshold_m
                    and abs(dy) < self.target_lock_center_threshold_m
                    and measured_ned is not None
                ):
                    target_lock_samples.append((now, measured_ned))
                else:
                    target_lock_samples.clear()
                while target_lock_samples and now - target_lock_samples[0][0] > self.target_lock_window_s:
                    target_lock_samples.popleft()
                if len(target_lock_samples) >= self.target_lock_min_samples:
                    ns = [sample[1][0] for sample in target_lock_samples]
                    es = [sample[1][1] for sample in target_lock_samples]
                    ds = [sample[1][2] for sample in target_lock_samples]
                    n_span = max(ns) - min(ns)
                    e_span = max(es) - min(es)
                    d_span = max(ds) - min(ds)
                    if (
                        n_span <= self.target_lock_max_xy_span_m
                        and e_span <= self.target_lock_max_xy_span_m
                        and d_span <= self.target_lock_max_d_span_m
                    ):
                        frozen = (float(median(ns)), float(median(es)), float(median(ds)))
                        self.frozen_target_ned = frozen
                        locked_ned = frozen
                        rospy.logwarn(
                            "%s TARGET LOCK | samples=%d span_N/E/D=(%.3f,%.3f,%.3f)m "
                            "raw=(%.3f,%.3f,%.3f) frozen=(%.3f,%.3f,%.3f)",
                            dropper, len(target_lock_samples), n_span, e_span, d_span,
                            measured_ned[0], measured_ned[1], measured_ned[2],
                            frozen[0], frozen[1], frozen[2],
                        )
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
                phase_threshold = self.align_threshold_m

            height_error = desired_height - actual_height
            if now >= next_setpoint_at:
                self._publish_ned_setpoint(
                    locked_ned,
                    desired_height,
                    "{} {} 对准".format(dropper, phase),
                )
                next_setpoint_at = now + self.setpoint_interval_s

            phase_aligned = (
                abs(dx) < phase_threshold
                and abs(dy) < phase_threshold
                and abs(height_error) < self.height_tolerance_m
            )
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
                horizontal_speed = math.hypot(
                    float(velocity.twist.linear.x),
                    float(velocity.twist.linear.y),
                )
                vertical_speed = abs(float(velocity.twist.linear.z))
                velocity_aligned = (
                    horizontal_speed < self.max_horizontal_speed_mps
                    and vertical_speed < self.max_vertical_speed_mps
                )

            phase_velocity_aligned = (
                velocity is not None
                and horizontal_speed < self.phase_max_horizontal_speed_mps
                and vertical_speed < self.phase_max_vertical_speed_mps
            )
            if phase_aligned and not phase_velocity_aligned:
                phase_aligned_since = None
                phase_stable = 0.0

            if phase == "COARSE" and phase_stable >= self.phase_stable_time_s:
                phase = "FINE"
                phase_aligned_since = None
                rospy.logwarn(
                    "%s COARSE 对准稳定 %.1fs，下降到 FINE 高度 %.2fm",
                    dropper,
                    phase_stable,
                    self.fine_align_height_m,
                )
            elif phase == "FINE" and phase_stable >= self.phase_stable_time_s:
                phase = "FINAL"
                phase_aligned_since = None
                rospy.logwarn(
                    "%s FINE 对准稳定 %.1fs，下降到最终投放高度 %.2fm",
                    dropper,
                    phase_stable,
                    self.target_drop_height_m,
                )

            final_aligned = phase == "FINAL" and phase_aligned and velocity_aligned
            if final_aligned:
                if final_aligned_since is None:
                    final_aligned_since = now
                final_stable = now - final_aligned_since
            else:
                final_aligned_since = None
                final_stable = 0.0

            self._publish_status(aiming=True)
            rospy.loginfo_throttle(
                0.5,
                "%s %s | X=%+.3fm Y=%+.3fm 高度=%.2fm 高度误差=%+.2fm "
                "速度XY=%.2fm/s Z=%.2fm/s 阶段稳定=%.1fs 最终稳定=%.1f/%.1fs "
                "aim=%.1f/%.1fs frozen=%s",
                dropper,
                phase,
                dx,
                dy,
                actual_height,
                height_error,
                horizontal_speed,
                vertical_speed,
                phase_stable,
                final_stable,
                self.stable_time_s,
                aim_elapsed_s,
                self.aim_timeout_s,
                self.frozen_target_ned is not None,
            )
            if aim_timer_started and aim_elapsed_s >= self.aim_timeout_s:
                chosen = self.frozen_target_ned or locked_ned
                hold = self._safe_hold_enu()
                if chosen is None or hold is None:
                    rospy.logerr("%s 强制投放到期但没有可信目标或安全保持 setpoint", dropper)
                    return False
                if not self._force_drop_safety_ok(dropper):
                    return False
                self._publish_hold_setpoint(hold, "{} FORCED_DROP 最后安全保持".format(dropper))
                self.frozen_target_ned = chosen
                self.last_aligned_target_ned = chosen
                self.forced_drop = True
                rospy.logwarn(
                    "%s FORCED DROP | aim=%.2fs X=%+.3f Y=%+.3f height_error=%+.3f "
                    "vxy=%.3f vz=%.3f target_NED=(%.3f,%.3f,%.3f) hold_ENU=(%.3f,%.3f,%.3f)",
                    dropper, aim_elapsed_s, dx, dy, height_error,
                    horizontal_speed, vertical_speed,
                    chosen[0], chosen[1], chosen[2], hold[0], hold[1], hold[2],
                )
                self._publish_status(aiming=False)
                return True
            if final_aligned and final_stable >= self.stable_time_s:
                if self.frozen_target_ned is None:
                    self.frozen_target_ned = locked_ned
                    rospy.logwarn("%s 正常投放门槛已满足；使用最后可信滤波 NED 作为冻结点", dropper)
                self.last_aligned_target_ned = self.frozen_target_ned
                rospy.logwarn(
                    "%s 最终位置、高度和速度连续稳定 %.1fs，瞄准完成",
                    dropper,
                    final_stable,
                )
                self._publish_status(aiming=False)
                return True
            self._log_control_diagnostics(dropper, phase, locked_ned)
            rate.sleep()
        return False

    def _force_drop_safety_ok(self, dropper):
        state = self._fresh_state()
        if state is None or not state.connected or not state.armed or state.mode != "GUIDED":
            rospy.logerr(
                "%s 强制投放被安全条件阻止: state=%s connected=%s armed=%s mode=%s",
                dropper,
                state is not None,
                state.connected if state else None,
                state.armed if state else None,
                state.mode if state else None,
            )
            return False
        if self.last_safe_setpoint_kind != "LOCAL" or self.last_setpoint_commanded_enu is None:
            rospy.logerr("%s 强制投放被阻止：没有最后安全本地保持 setpoint", dropper)
            return False
        if self._fresh_pose() is None or self._fresh_velocity() is None:
            rospy.logerr("%s 强制投放被阻止：pose/velocity 不新鲜", dropper)
            return False
        if self.dry_run:
            return True
        with self._lock:
            ready = self.servo_ready
        if not ready or self.servo_cmd_pub.get_num_connections() <= 0:
            rospy.logwarn("%s 强制投放前 servo 不可用，尝试有界恢复", dropper)
            if not self._restart_dead_children():
                return False
            return self._wait_for_servo_subscriber(timeout_s=self.servo_ready_timeout_s)
        return True

    def _wait_for_servo_subscriber(self, timeout_s=None):
        if self.flight_only_mode:
            rospy.logerr("FLIGHT ONLY 禁止等待 /servo/cmd 订阅者")
            return False
        if self.dry_run:
            return True
        timeout_s = self.servo_wait_timeout_s if timeout_s is None else timeout_s
        deadline = monotonic_s() + timeout_s
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            with self._lock:
                ready = self.servo_ready
            if ready and self.servo_cmd_pub.get_num_connections() > 0:
                rospy.loginfo("servo_test.py 已完成初始化，/servo/cmd 控制链路就绪")
                return True
            rospy.loginfo_throttle(
                2.0,
                "等待 servo_test.py 完成初始化并发布 /servo/ready=true...",
            )
            rospy.sleep(0.1)
        rospy.logerr("servo_test.py 未在 %.1fs 内就绪，拒绝开始投放测试", timeout_s)
        return False

    def drop(self, dropper):
        if not self.drop_enabled:
            rospy.logerr("FLIGHT ONLY 禁止发布任何 /servo/cmd 命令")
            return False
        if not self.ground_test:
            state = self._fresh_state()
            if state is None or not state.connected or not state.armed or state.mode != "GUIDED":
                rospy.logerr(
                    "%s 投放瞬间安全状态无效：connected=%s armed=%s mode=%s",
                    dropper,
                    state.connected if state else None,
                    state.armed if state else None,
                    state.mode if state else None,
                )
                return False
        if self.dry_run:
            rospy.logwarn("dry_run=true：将要执行 %s on，但不发布 /servo/cmd", dropper)
            if dropper == "A":
                self.ammo_a = 0
            else:
                self.ammo_b = 0
            return True
        if self.servo_cmd_pub.get_num_connections() <= 0:
            rospy.logerr("投放瞬间 /servo/cmd 订阅者丢失，拒绝投放")
            return False

        command = "{} on".format(dropper)
        self.servo_cmd_pub.publish(String(data=command))
        if dropper == "A":
            self.ammo_a = 0
        else:
            self.ammo_b = 0
        rospy.logwarn(
            "舵机命令已发布 /servo/cmd: %s | forced_drop=%s；"
            "当前接口没有 MAV_CMD_DO_SET_SERVO 执行回执，不能确认物理载荷已经释放",
            command,
            self.forced_drop,
        )

        # 重复数次任务事件，降低 detector_node 启动瞬间错过显示事件的概率。
        for _ in range(3):
            self._publish_status(aiming=False, last_drop=dropper)
            rospy.sleep(0.05)
        self._publish_status(aiming=False)

        if self.close_after_drop:
            rospy.sleep(max(0.0, self.close_delay_s))
            close_command = "{} off".format(dropper)
            self.servo_cmd_pub.publish(String(data=close_command))
            rospy.logwarn("close_after_drop=true，已发布 /servo/cmd: %s", close_command)
        return True

    def _interruptible_delay(self, duration_s):
        deadline = monotonic_s() + max(0.0, duration_s)
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            rospy.sleep(0.1)
        return not rospy.is_shutdown()

    def safe_loiter(self, reason):
        state = self._fresh_state()
        should_request = self.guided_was_requested or (
            state is not None and state.mode == "GUIDED"
        )
        if not should_request:
            return True
        rospy.logwarn("%s：尝试切回 LOITER，把控制权交还飞手", reason)
        if self.set_mode_confirmed("LOITER"):
            return True
        rospy.logerr("切回 LOITER 失败，请飞手立即用遥控器接管")
        return False

    def search_and_aim_dropper(
        self, dropper, require_confirmation, guided_already=False
    ):
        if require_confirmation and not self.wait_for_area_search_confirm(dropper):
            rospy.logwarn("操作者取消 %s 区域搜索", dropper)
            return False
        if not guided_already and not self._validate_before_area_search(dropper):
            return False
        if not self._prepare_area_search_anchor(dropper):
            return False
        if not guided_already and not self.set_mode_confirmed("GUIDED"):
            rospy.logerr("%s 区域搜索 GUIDED 切换或回读确认失败", dropper)
            return False
        if not self.area_search_for_target(dropper):
            self.safe_loiter("{} 区域搜索未找到目标".format(dropper))
            return False
        return self.aim_dropper(dropper)

    def _stable_distinct_target_available(self, dropper, stable_s=None):
        stable_s = self.enroute_detect_stable_s if stable_s is None else stable_s
        deadline = monotonic_s() + stable_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            aim = self._fresh_aim()
            target = self._aim_target_ned(aim, dropper)
            if target is None or self._target_is_blacklisted(target, dropper):
                return False
            rate.sleep()
        aim = self._fresh_aim()
        target = self._aim_target_ned(aim, dropper)
        return target is not None and not self._target_is_blacklisted(target, dropper)

    def _mission_failure(self, reason):
        self._transition("FAILSAFE", reason)
        rospy.logerr("任务级失败: %s", reason)
        if self.airborne:
            state = self._fresh_state()
            if state is not None and not state.armed:
                return 1
            rospy.logerr("飞机已离地，优先请求 RTL")
            if not self.set_mode_confirmed("RTL"):
                rospy.logerr("RTL 请求失败，尝试 LAND")
                self.set_mode_confirmed("LAND")
        else:
            rospy.logwarn("尚未确认离地，不无条件请求 RTL")
        return 1

    def _run_flight_only_mission(self):
        """执行与视觉、瞄准、弹药和舵机完全隔离的航线验证任务。"""
        self._transition("GUIDED_ARM_TAKEOFF", "操作者 Enter 授权 FLIGHT ONLY")
        if not self.set_mode_confirmed("GUIDED"):
            return self._mission_failure("FLIGHT ONLY GUIDED 切换失败")
        if not self.arm_confirmed():
            return self._mission_failure("FLIGHT ONLY 自动解锁失败")
        if not self.capture_ground_relative_altitude():
            return self._mission_failure("FLIGHT ONLY 地面 rel_alt 基准记录失败")
        if not self.command_takeoff_and_wait():
            return self._mission_failure(
                "FLIGHT ONLY 起飞命令重试后仍未开始或未到高度"
            )

        drop_target = self._make_global_target(
            self.drop_zone_wgs84[0],
            self.drop_zone_wgs84[1],
            self.drop_zone_transit_altitude_m,
        )
        self._transition("GOTO_DROP_ZONE", "起飞高度已到达；视觉监控保持禁用")
        goto_result = self.fly_global_waypoint(
            drop_target,
            "GOTO_DROP_ZONE",
            detect_dropper=None,
            require_full_telemetry=True,
        )
        if goto_result != "arrived":
            return self._mission_failure("FLIGHT ONLY 飞往投放区失败")
        self.drop_zone_reached = True

        self._transition(
            "FLIGHT_ONLY_PATTERN",
            "抵达投放区；停止全局 setpoint，切换本地保持和完整区域航线",
        )
        if not self.fly_area_pattern_only():
            return self._mission_failure("FLIGHT ONLY 区域航线执行失败")

        if not self.flight_only_continue_to_recon:
            self._transition(
                "RTL",
                "FLIGHT ONLY 区域航线完成且 flight_only_continue_to_recon=false",
            )
            if not self.request_rtl_and_wait():
                return self._mission_failure("FLIGHT ONLY RTL 请求/确认/等待失败")
            self._transition("COMPLETE", "FLIGHT ONLY 投放区航线和 RTL 完成")
            rospy.logwarn("FLIGHT ONLY 任务完成；全程未启用视觉、瞄准或投放")
            return 0

        recon_target = self._make_global_target(
            self.recon_zone_wgs84[0],
            self.recon_zone_wgs84[1],
            self.recon_altitude_m,
        )
        self._transition(
            "GOTO_RECON_CLIMB",
            "FLIGHT ONLY 区域航线完成；停止本地 setpoint，前往侦察区并爬升",
        )
        if self.fly_global_waypoint(
            recon_target,
            "GOTO_RECON_CLIMB",
            detect_dropper=None,
            require_full_telemetry=True,
        ) != "arrived":
            return self._mission_failure("FLIGHT ONLY 飞往侦察区失败")
        self._transition("RECON_HOLD", "FLIGHT ONLY 已抵达侦察区")
        if not self.hold_global_target(
            recon_target,
            self.recon_hold_s,
            "RECON_HOLD",
            require_full_telemetry=True,
        ):
            return self._mission_failure("FLIGHT ONLY 侦察区保持失败")
        self._transition("RTL", "FLIGHT ONLY 侦察保持完成")
        if not self.request_rtl_and_wait():
            return self._mission_failure("FLIGHT ONLY RTL 请求/确认/等待失败")
        self._transition("COMPLETE", "FLIGHT ONLY 完整航线完成")
        rospy.logwarn("FLIGHT ONLY 任务完成；全程未启用视觉、瞄准或投放")
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
            rospy.logwarn("================ FLIGHT ONLY MODE ================")
            rospy.logwarn(
                "vision=DISABLED servo=DISABLED aiming=DISABLED drop=DISABLED"
            )
            rospy.logwarn(
                "CUADC FLIGHT ONLY START | pattern=%s laps=%d drop=%.1fm "
                "recon=%.1fm takeoff=%.1fm drop_transit_alt=%.1fm "
                "pattern_alt=%.1fm recon_alt=%.1fm "
                "continue_recon=%s",
                self.area_search_pattern,
                self.flight_only_pattern_laps,
                self.drop_zone_distance_m,
                self.recon_zone_distance_m,
                self.takeoff_altitude_m,
                self.drop_zone_transit_altitude_m,
                self.flight_only_search_altitude_m,
                self.recon_altitude_m,
                self.flight_only_continue_to_recon,
            )
        else:
            rospy.logwarn(
                "CUADC FULL MISSION START | full_mission=%s ground_test=%s pattern=%s "
                "drop=%.1fm recon=%.1fm takeoff=%.1fm drop_transit_alt=%.1fm "
                "recon_alt=%.1fm A/B=%s dry_run=%s",
                self.full_mission_mode, self.ground_test, self.area_search_pattern,
                self.drop_zone_distance_m, self.recon_zone_distance_m,
                self.takeoff_altitude_m, self.drop_zone_transit_altitude_m,
                self.recon_altitude_m,
                self.enable_b_dropper, self.dry_run,
            )
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
            rospy.logwarn("操作者未授权；未切模式、未解锁、未起飞")
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
            self._transition("COMPLETE", "full_mission_mode=false：仅完成依赖/航点/授权检查，不执行飞行")
            rospy.logwarn("full_mission_mode=false：未切模式、未解锁、未起飞")
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

        drop_target = self._make_global_target(
            self.drop_zone_wgs84[0],
            self.drop_zone_wgs84[1],
            self.drop_zone_transit_altitude_m,
        )
        self._transition("GOTO_DROP_ZONE", "起飞高度已到达")
        goto_result = self.fly_global_waypoint(drop_target, "GOTO_DROP_ZONE", detect_dropper="A")
        if goto_result == "failed":
            return self._mission_failure("飞往投放区失败")
        self.enroute_a_intercepted = goto_result == "detected"
        self.drop_zone_reached = goto_result == "arrived"

        if self.drop_zone_reached:
            if not self.set_area_search_anchor_from_current("A"):
                return self._mission_failure("无法冻结投放区搜索中心")
            if self._stable_distinct_target_available("A"):
                self._transition("A_AIM", "抵达投放区且已有稳定 A 目标")
            else:
                self._transition("A_SEARCH", "抵达投放区仍无稳定 A 目标")
                if not self.area_search_for_target("A"):
                    return self._mission_failure("A 区域航线搜索超时")
                self._transition("A_AIM", "A 区域搜索发现稳定目标")
        else:
            self._transition("A_AIM", "途中稳定发现 A 并已切到本地保持")
        if not self.aim_dropper("A"):
            return self._mission_failure("A 分层瞄准失败")
        self._transition("A_DROP", "A 正常或强制投放条件成立")
        if not self.drop("A"):
            return self._mission_failure("A 舵机命令发布失败")
        if self.last_aligned_target_ned is not None:
            self.completed_drop_targets.append(self.last_aligned_target_ned)

        if self.enable_b_dropper:
            self.forced_drop = False
            if not self._interruptible_delay(self.after_a_delay_s):
                return self._mission_failure("A/B 间隔被中断")
            b_direct = self._stable_distinct_target_available("B")
            if not b_direct and self.enroute_a_intercepted and not self.drop_zone_reached:
                self._transition("RETURN_DROP_ZONE_FOR_B", "A 途中投放且 B 需要区域搜索")
                return_result = self.fly_global_waypoint(drop_target, "RETURN_DROP_ZONE_FOR_B")
                if return_result != "arrived":
                    return self._mission_failure("返回预设投放区中心失败")
                self.drop_zone_reached = True
            if b_direct:
                self._transition("B_AIM", "已发现不同于 A 的稳定 B 目标")
            else:
                if not self.set_area_search_anchor_from_current("B"):
                    return self._mission_failure("无法冻结 B 搜索中心")
                self._transition("B_SEARCH", "尚未发现不同 B 目标")
                original_timeout = self.area_search_timeout_s
                self.area_search_timeout_s = self.second_target_search_timeout_s
                try:
                    b_found = self.area_search_for_target("B")
                finally:
                    self.area_search_timeout_s = original_timeout
                if b_found:
                    self._transition("B_AIM", "B 搜索发现不同目标")
                else:
                    hold = self._safe_hold_enu()
                    if hold is None or not self._force_drop_safety_ok("B"):
                        return self._mission_failure("B 搜索超时且强制释放安全条件不成立")
                    self._publish_hold_setpoint(hold, "B SEARCH TIMEOUT 强制释放保持")
                    self.forced_drop = True
                    rospy.logwarn(
                        "B 搜索 %.1fs 未发现不同桶；保持最后安全位置并强制发布 B on",
                        self.second_target_search_timeout_s,
                    )
                    self._transition("B_DROP", "B 搜索超时强制命令")
                    if not self.drop("B"):
                        return self._mission_failure("B 搜索超时后的舵机命令发布失败")
                    b_direct = None
            if b_direct is not None and self.top_state == "B_AIM":
                if not self.aim_dropper("B"):
                    return self._mission_failure("B 分层瞄准失败")
                self._transition("B_DROP", "B 正常或 20s 强制投放条件成立")
                if not self.drop("B"):
                    return self._mission_failure("B 舵机命令发布失败")
                if self.last_aligned_target_ned is not None:
                    self.completed_drop_targets.append(self.last_aligned_target_ned)
        else:
            rospy.logwarn("enable_b_dropper=false：A 完成后直接进入侦察区")

        recon_target = self._make_global_target(
            self.recon_zone_wgs84[0], self.recon_zone_wgs84[1], self.recon_altitude_m
        )
        self._transition("GOTO_RECON_CLIMB", "A/B 处理完成，前往预设侦察区并爬升")
        if self.fly_global_waypoint(recon_target, "GOTO_RECON_CLIMB") != "arrived":
            return self._mission_failure("飞往侦察区失败")
        self._transition("RECON_HOLD", "已抵达侦察区")
        if not self.hold_global_target(recon_target, self.recon_hold_s, "RECON_HOLD"):
            return self._mission_failure("侦察区保持失败")
        self._transition("RTL", "侦察保持完成")
        if not self.request_rtl_and_wait():
            return self._mission_failure("RTL 请求/确认/等待失败")
        self._transition("COMPLETE", "完整任务完成")
        rospy.logwarn("CUADC 全自动任务完成；A/B 只确认舵机命令发布，不代表物理释放回执")
        return 0

    def _stop_children(self):
        if self._cleanup_done:
            return
        self._cleanup_done = True
        for label, process in reversed(self._children):
            if process.poll() is not None:
                continue
            if label == "MAVROS" and self.airborne and self.top_state != "COMPLETE":
                rospy.logerr("飞机仍在空中，保留本脚本自动启动的 MAVROS，避免 Ctrl+C 同时切断飞控链路")
                continue
            rospy.loginfo("停止本脚本自动启动的 %s", label)
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
                    state_age = (
                        monotonic_s() - self.state_received_at
                        if state is not None
                        else float("inf")
                    )
                if (
                    state is not None
                    and state_age <= self.guided_state_grace_s
                    and state.connected
                    and state.armed
                    and state.mode == "GUIDED"
                ):
                    try:
                        if self._set_mode_srv is None:
                            self._set_mode_srv = rospy.ServiceProxy(
                                "/mavros/set_mode", SetMode
                            )
                        response = self._set_mode_srv(
                            base_mode=0, custom_mode="RTL"
                        )
                        rospy.logerr(
                            "ROS shutdown/Ctrl+C 空中保护：已请求 RTL，mode_sent=%s；"
                            "请飞手持续监控并准备接管",
                            getattr(response, "mode_sent", None),
                        )
                    except (rospy.ServiceException, rospy.ROSException) as exc:
                        rospy.logerr(
                            "ROS shutdown/Ctrl+C 空中 RTL 请求失败: %s；"
                            "请飞手立即接管",
                            exc,
                        )
                else:
                    rospy.logerr(
                        "ROS shutdown/Ctrl+C：未覆盖当前飞手模式/陈旧状态；"
                        "停止任务 setpoint，请飞手立即接管 | mode=%s armed=%s "
                        "connected=%s state_age=%.2fs",
                        state.mode if state is not None else "NONE",
                        state.armed if state is not None else None,
                        state.connected if state is not None else None,
                        state_age,
                    )
        except Exception as exc:
            rospy.logerr("shutdown 安全处理异常: %s", exc)
        self._stop_children()


def main():
    rospy.init_node("auto_drop_test")
    node = None
    exit_code = 1
    try:
        node = AutoDropTest()
        exit_code = node.run()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        if node is not None and node.flight_only_mode:
            rospy.logwarn("收到 Ctrl+C，取消 FLIGHT ONLY 航线验证")
        else:
            rospy.logwarn("收到 Ctrl+C，取消 W/Z/I 搜索投放")
    except Exception as exc:
        mission_name = (
            "FLIGHT ONLY 航线任务"
            if node is not None and node.flight_only_mode
            else "全自动任务"
        )
        rospy.logerr("%s异常: %s", mission_name, exc)
        if node is not None:
            exit_code = node._mission_failure("未处理异常: {}".format(exc))
    finally:
        if node is not None:
            node._stop_children()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
