#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CUADC 半自动/自主 A/B 瞄准投放测试。

飞手负责手动解锁、起飞并在桶附近保持 LOITER。半自动模式由操作者按 Enter
授权；自主模式在目标稳定后自动接管 GUIDED。节点使用 detector_node 发布的
绝对 NED 瞄准点执行分层高度对准，并在目标丢失时保持、升高和局部搜索，最终
通过 /servo/cmd 请求 servo_test.py 投放。
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

import rosnode
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import ExtendedState, RCIn, State, StatusText
from mavros_msgs.srv import SetMode
from std_msgs.msg import Bool, String

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


class SemiAutoDropTest:
    def __init__(self):
        self.ground_test = get_bool_param("~ground_test", False)
        self.autonomous_mode = get_bool_param("~autonomous_mode", False)
        self.enable_b_dropper = get_bool_param("~enable_b_dropper", False)
        self.align_threshold_m = float(rospy.get_param("~align_threshold_m", 0.15))
        self.target_drop_height_m = float(
            rospy.get_param("~target_drop_height_m", 1.5)
        )
        self.height_tolerance_m = float(
            rospy.get_param("~height_tolerance_m", 0.15)
        )
        self.stable_time_s = float(rospy.get_param("~stable_time_s", 1.0))
        self.setpoint_interval_s = float(
            rospy.get_param("~setpoint_interval_s", 0.1)
        )
        self.after_a_delay_s = float(rospy.get_param("~after_a_delay_s", 3.0))
        self.auto_start_detector = get_bool_param("~auto_start_detector", True)
        self.auto_start_servo_test = get_bool_param(
            "~auto_start_servo_test", True
        )
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
            rospy.get_param("~phase_stable_time_s", 1.0)
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
            "~require_distinct_b_target", False
        )
        self.distinct_target_distance_m = float(
            rospy.get_param("~distinct_target_distance_m", 0.60)
        )

        self.aim_info_timeout_s = float(
            rospy.get_param("~aim_info_timeout_s", 1.0)
        )
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 2.0))
        self.guided_state_grace_s = float(
            rospy.get_param("~guided_state_grace_s", 5.0)
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

        self._lock = threading.RLock()
        self.current_state = None
        self.state_received_at = 0.0
        self.latest_aim = None
        self.aim_received_at = 0.0
        self.current_pose = None
        self.pose_received_at = 0.0
        self.current_velocity = None
        self.velocity_received_at = 0.0
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
        self._children = []
        self._cleanup_done = False
        self.ammo_a = 1
        self.ammo_b = 1 if self.enable_b_dropper else 0
        self.completed_drop_targets = []
        self.last_aligned_target_ned = None

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
            "/servo/ready", Bool, self._servo_ready_cb, queue_size=1
        )
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10
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

    def _spawn(self, command, label):
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
        return True

    def start_dependencies(self):
        if self.auto_start_detector:
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

        if self.auto_start_servo_test:
            if self._node_is_running("servo_controller"):
                rospy.loginfo("servo_test.py 已运行，不重复启动")
            elif not self._spawn(
                ["rosrun", "cuadc_vision", "servo_test.py"],
                "servo_test.py",
            ):
                return False
        return True

    def _fresh_state(self):
        with self._lock:
            msg = self.current_state
            age = monotonic_s() - self.state_received_at
        if msg is None or age > self.state_timeout_s:
            return None
        return msg

    def _fresh_aim(self, require_count=True):
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
        if msg is None or age > self.state_timeout_s:
            return None
        return msg

    def _fresh_velocity(self):
        with self._lock:
            msg = self.current_velocity
            age = monotonic_s() - self.velocity_received_at
        if msg is None or age > self.state_timeout_s:
            return None
        return msg

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

    def wait_for_target_and_confirm(self, dropper="A"):
        """等待稳定目标；半自动模式等待期间丢失目标会重新获取而非退出。"""
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
        try:
            response = self._set_mode_srv(base_mode=0, custom_mode=mode)
        except rospy.ServiceException as exc:
            rospy.logerr("调用 set_mode(%s) 失败: %s", mode, exc)
            self._set_mode_srv = None
            return False
        if not response.mode_sent:
            rospy.logerr("飞控拒绝模式请求: %s", mode)
            return False

        deadline = monotonic_s() + self.mode_timeout_s
        actual = "UNKNOWN"
        while monotonic_s() < deadline:
            state = self._fresh_state()
            if state is not None:
                actual = state.mode
                if actual == mode:
                    rospy.logwarn("已回读 /mavros/state，确认模式为 %s", mode)
                    if mode == "LOITER":
                        self.guided_was_requested = False
                    return True
            time.sleep(0.1)
        rospy.logerr("模式确认超时: expected=%s actual=%s", mode, actual)
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

    def _publish_enu_setpoint(self, desired_enu, label):
        commanded = self._limit_setpoint_from_current_pose(desired_enu)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = commanded[0]
        msg.pose.position.y = commanded[1]
        msg.pose.position.z = commanded[2]
        pose = self._fresh_pose()
        if pose is not None:
            msg.pose.orientation = pose.pose.orientation
        else:
            msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)
        with self._lock:
            self.last_setpoint_desired_enu = tuple(float(v) for v in desired_enu)
            self.last_setpoint_commanded_enu = tuple(float(v) for v in commanded)
            self.last_setpoint_label = str(label)
            self.last_setpoint_at = monotonic_s()
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
        msg = MissionStatus()
        msg.ammo_a = self.ammo_a
        msg.ammo_b = self.ammo_b
        msg.aiming = aiming
        msg.last_drop = last_drop
        self.mission_status_pub.publish(msg)

    def aim_dropper(self, dropper):
        prefix = dropper.lower()
        final_aligned_since = None
        rate = rospy.Rate(10)

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
        state_stale_started_at = None
        state_stale_hold_enu = None
        next_setpoint_at = 0.0

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
            state = self._fresh_state()
            if state is None:
                with self._lock:
                    raw_state = self.current_state
                    raw_state_age = (
                        now - self.state_received_at
                        if raw_state is not None
                        else float("inf")
                    )
                raw_mode = raw_state.mode if raw_state is not None else "NONE"
                recoverable = (
                    raw_state is not None
                    and raw_state.connected
                    and raw_state.armed
                    and raw_state.mode == "GUIDED"
                    and raw_state_age <= self.guided_state_grace_s
                )
                if recoverable:
                    pose = self._fresh_pose()
                    if pose is None:
                        error = (
                            "/mavros/state 超时宽限期间本地位姿也不新鲜："
                            "state_age={:.3f}s"
                        ).format(raw_state_age)
                        self._log_control_diagnostics(
                            dropper, phase, locked_ned, error
                        )
                        return False
                    if state_stale_started_at is None:
                        state_stale_started_at = now
                        state_stale_hold_enu = (
                            float(pose.pose.position.x),
                            float(pose.pose.position.y),
                            float(pose.pose.position.z),
                        )
                        rospy.logwarn(
                            "%s /mavros/state 超过 %.2fs 未刷新；最后模式仍为 GUIDED，"
                            "冻结阶段并保持当前位置，最长等待到 %.2fs",
                            dropper,
                            self.state_timeout_s,
                            self.guided_state_grace_s,
                        )
                    phase_aligned_since = None
                    final_aligned_since = None
                    if now >= next_setpoint_at:
                        self._publish_hold_setpoint(
                            state_stale_hold_enu,
                            "{} STATE_STALE 保持".format(dropper),
                        )
                        next_setpoint_at = now + self.setpoint_interval_s
                    rospy.logwarn_throttle(
                        0.5,
                        "%s STATE_STALE 恢复等待 | last_mode=%s age=%.3f/%.3fs "
                        "保持ENU=(%.3f, %.3f, %.3f)",
                        dropper,
                        raw_mode,
                        raw_state_age,
                        self.guided_state_grace_s,
                        state_stale_hold_enu[0],
                        state_stale_hold_enu[1],
                        state_stale_hold_enu[2],
                    )
                    self._publish_status(aiming=True)
                    self._log_control_diagnostics(dropper, phase, locked_ned)
                    rate.sleep()
                    continue
                error = (
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
                self._log_control_diagnostics(dropper, phase, locked_ned, error)
                return False
            if state_stale_started_at is not None:
                stale_duration = now - state_stale_started_at
                rospy.logwarn(
                    "%s /mavros/state 已恢复，当前 mode=%s，停滞 %.2fs；"
                    "保持原阶段 %s 并重新累计稳定时间",
                    dropper,
                    state.mode,
                    stale_duration,
                    phase,
                )
                state_stale_started_at = None
                state_stale_hold_enu = None
            if state.mode != "GUIDED":
                error = (
                    "收到新鲜状态但实际模式不是 GUIDED：mode={} connected={} "
                    "armed={} system_status={}"
                ).format(
                    state.mode,
                    state.connected,
                    state.armed,
                    state.system_status,
                )
                self._log_control_diagnostics(dropper, phase, locked_ned, error)
                return False
            if not state.armed:
                self._log_control_diagnostics(
                    dropper,
                    phase,
                    locked_ned,
                    "瞄准期间 armed 变为 false",
                )
                return False
            pose = self._fresh_pose()
            if pose is None:
                self._log_control_diagnostics(
                    dropper,
                    phase,
                    locked_ned,
                    "本地位姿消息不新鲜",
                )
                return False

            aim = self._fresh_aim()
            measured_ned = self._aim_target_ned(aim, dropper)
            accepted = False
            loss_reason = self._aim_invalid_reason(dropper)
            if measured_ned is not None:
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
                    if loss_elapsed < self.loss_hover_s:
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

            dx = float(getattr(aim, "{}_delta_x_m".format(prefix)))
            dy = float(getattr(aim, "{}_delta_y_m".format(prefix)))
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
                "速度XY=%.2fm/s Z=%.2fm/s 阶段稳定=%.1fs 最终稳定=%.1f/%.1fs",
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
            )
            if final_aligned and final_stable >= self.stable_time_s:
                self.last_aligned_target_ned = locked_ned
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

    def _wait_for_servo_subscriber(self):
        if self.dry_run:
            return True
        deadline = monotonic_s() + self.servo_wait_timeout_s
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
        rospy.logerr("servo_test.py 未在超时时间内就绪，拒绝开始投放测试")
        return False

    def drop(self, dropper):
        if self.dry_run:
            rospy.logwarn("dry_run=true：将要执行 %s on，但不发布 /servo/cmd", dropper)
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
        rospy.logwarn("已发布 /servo/cmd: %s", command)

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

    def run(self):
        rospy.logwarn(
            "瞄准投放测试启动 | ground_test=%s autonomous=%s B=%s dry_run=%s "
            "xy_threshold=%.3fm target_height=%.2fm height_tolerance=%.2fm "
            "stable=%.1fs min_bucket_count=%d",
            self.ground_test,
            self.autonomous_mode,
            self.enable_b_dropper,
            self.dry_run,
            self.align_threshold_m,
            self.target_drop_height_m,
            self.height_tolerance_m,
            self.stable_time_s,
            self.min_bucket_count,
        )
        if self.ground_test:
            rospy.logwarn(
                "ground_test=true：脚本启动即视为地面测试授权；"
                "目标满足阈值并稳定 %.1fs 后将自动控制真实舵机",
                self.stable_time_s,
            )
        elif self.autonomous_mode:
            rospy.logwarn(
                "autonomous_mode=true：启动命令已授权 GUIDED 和首次搜索；"
                "目标丢失将保持、升高并执行局部搜索，搜索超时才退出"
            )
        else:
            rospy.logwarn(
                "control_mode=SEMI_AUTO：默认无附加参数模式；稳定检测后等待 Enter，"
                "Enter 后才请求 GUIDED"
            )
        if not self.start_dependencies():
            return 1
        if not self.wait_for_mavros():
            return 1
        if not self._wait_for_servo_subscriber():
            return 1
        if not self.autonomous_mode:
            if not self.wait_for_target_and_confirm("A"):
                rospy.logwarn("操作者取消半自动投放")
                return 1

        if not self._validate_before_guided("A"):
            return 1
        if not self.ground_test and not self.set_mode_confirmed("GUIDED"):
            rospy.logerr("GUIDED 切换或回读确认失败，不继续投放")
            return 1
        if not self.aim_dropper("A") or not self.drop("A"):
            return 1
        if self.last_aligned_target_ned is not None:
            self.completed_drop_targets.append(self.last_aligned_target_ned)

        if not self.enable_b_dropper:
            if not self.safe_loiter("A 投放完成"):
                return 1
            rospy.logwarn("A 流程完成，enable_b_dropper=false，测试结束")
            return 0

        if self.autonomous_mode and not self.ground_test:
            rospy.logwarn(
                "A 流程完成，自主模式保持 GUIDED，等待 %.1fs 后继续 B",
                self.after_a_delay_s,
            )
        else:
            if not self.safe_loiter("A 投放完成"):
                return 1
            rospy.logwarn("A 流程完成，等待 %.1fs 后开始 B", self.after_a_delay_s)
        if not self._interruptible_delay(self.after_a_delay_s):
            return 1
        if self.autonomous_mode and not self.ground_test:
            state = self._fresh_state()
            if state is None or not state.connected or not state.armed:
                rospy.logerr("B 流程开始前飞控连接或解锁状态无效")
                return 1
            if state.mode != "GUIDED":
                rospy.logerr("B 流程开始前不再处于 GUIDED")
                return 1
            if self._fresh_pose() is None:
                rospy.logerr("B 流程开始前本地位姿无效")
                return 1
        else:
            if not self._validate_before_guided("B"):
                return 1
            if not self.ground_test and not self.set_mode_confirmed("GUIDED"):
                rospy.logerr("B 流程 GUIDED 切换或回读确认失败，不继续投放")
                return 1
        if not self.aim_dropper("B") or not self.drop("B"):
            return 1
        if self.last_aligned_target_ned is not None:
            self.completed_drop_targets.append(self.last_aligned_target_ned)
        if not self.safe_loiter("B 投放完成"):
            return 1

        rospy.logwarn("A/B 半自动投放测试完成，飞机已交还飞手")
        return 0

    def _stop_children(self):
        if self._cleanup_done:
            return
        self._cleanup_done = True
        for label, process in reversed(self._children):
            if process.poll() is not None:
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
            self.safe_loiter("ROS shutdown/Ctrl+C")
        except Exception as exc:
            rospy.logerr("shutdown 时请求 LOITER 发生异常: %s", exc)
        self._stop_children()


def main():
    rospy.init_node("semi_auto_drop_test")
    node = None
    exit_code = 1
    try:
        node = SemiAutoDropTest()
        exit_code = node.run()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        rospy.logwarn("收到 Ctrl+C，取消半自动投放")
    except Exception as exc:
        rospy.logerr("半自动投放异常: %s", exc)
    finally:
        if node is not None:
            node.safe_loiter("脚本退出")
            node._stop_children()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
