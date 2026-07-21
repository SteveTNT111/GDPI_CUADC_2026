#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CUADC 半自动 A/B 抛投测试。

飞手负责手动解锁、起飞并在桶附近保持 LOITER。本节点只在操作者按 Enter
确认后短时接管 GUIDED，使用 detector_node 发布的绝对 NED 瞄准点发送
MAVROS ENU 位置 setpoint，并通过 /servo/cmd 请求 servo_test.py 投放。
启用 B 抛投器时，A 和 B 在同一次 GUIDED 接管中依次完成，只在两个投放
都完成后切回 LOITER 并把控制权交还飞手。
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
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
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


def limit_xy_step(current_x, current_y, target_x, target_y, max_step_m):
    """把一次水平位置目标增量限制在 max_step_m 内。"""
    dx = float(target_x) - float(current_x)
    dy = float(target_y) - float(current_y)
    distance = math.hypot(dx, dy)
    if distance <= max_step_m or distance <= 1e-9:
        return float(target_x), float(target_y)
    scale = float(max_step_m) / distance
    return float(current_x) + dx * scale, float(current_y) + dy * scale


class TargetLossGuard:
    """按视觉消息帧计数的目标丢失容错器。

    连续无效帧未达阈值时只视为短暂丢帧；达到阈值后开始重获计时。
    重获超时一旦发生就锁存，避免在控制循环来不及读取时，超时后的一帧
    有效消息又把任务恢复。
    """

    VALID = "valid"
    TOLERATING = "tolerating"
    LOST = "lost"
    RECOVERED = "recovered"
    REACQUIRED = "reacquired"
    TIMEOUT = "timeout"

    def __init__(
        self,
        lost_frame_threshold,
        reacquire_timeout_s,
        vision_frame_rate_hz=30.0,
    ):
        self.lost_frame_threshold = int(lost_frame_threshold)
        self.reacquire_timeout_s = float(reacquire_timeout_s)
        self.vision_frame_rate_hz = float(vision_frame_rate_hz)
        if self.lost_frame_threshold <= 0:
            raise ValueError("lost_frame_threshold 必须大于 0")
        if self.reacquire_timeout_s <= 0.0:
            raise ValueError("target_reacquire_timeout_s 必须大于 0")
        if self.vision_frame_rate_hz <= 0.0:
            raise ValueError("vision_frame_rate_hz 必须大于 0")
        self.loss_frame_window_s = (
            self.lost_frame_threshold / self.vision_frame_rate_hz
        )
        self.reset()

    def reset(self):
        self.invalid_frames = 0
        self.invalid_since = None
        self.lost_since = None
        self.timed_out = False

    def observe(self, valid, now):
        now = float(now)
        if self.timed_out:
            return self.TIMEOUT

        if valid:
            if (
                self.lost_since is not None
                and now - self.lost_since >= self.reacquire_timeout_s
            ):
                self.timed_out = True
                return self.TIMEOUT
            previous_invalid_frames = self.invalid_frames
            was_lost = self.lost_since is not None
            self.invalid_frames = 0
            self.invalid_since = None
            self.lost_since = None
            if was_lost:
                return self.REACQUIRED
            if previous_invalid_frames > 0:
                return self.RECOVERED
            return self.VALID

        if self.invalid_frames == 0:
            self.invalid_since = now
        self.invalid_frames += 1
        return self.status(now)

    def force_lost(self, now):
        """视觉消息流停止时，直接进入重获倒计。"""
        if self.timed_out:
            return self.TIMEOUT
        self.invalid_frames = max(
            self.invalid_frames, self.lost_frame_threshold
        )
        if self.invalid_since is None:
            self.invalid_since = float(now)
        if self.lost_since is None:
            self.lost_since = float(now)
        return self.status(now)

    def status(self, now):
        if self.timed_out:
            return self.TIMEOUT
        if (
            self.lost_since is None
            and self.invalid_frames > 0
            and (
                self.invalid_frames >= self.lost_frame_threshold
                or float(now) - self.invalid_since >= self.loss_frame_window_s
            )
        ):
            # 检测节点实际推理帧率可能低于相机 30 FPS。
            # 因此除逐帧计数外，还使用 10/30s 时间等价阈值。
            self.lost_since = float(now)
        if self.lost_since is not None:
            if float(now) - self.lost_since >= self.reacquire_timeout_s:
                self.timed_out = True
                return self.TIMEOUT
            return self.LOST
        if self.invalid_frames > 0:
            return self.TOLERATING
        return self.VALID

    def remaining_s(self, now):
        if self.lost_since is None:
            return self.reacquire_timeout_s
        return max(
            0.0,
            self.reacquire_timeout_s - (float(now) - self.lost_since),
        )


class LateralDivergenceGuard:
    """检测水平二维误差在一段时间内是否持续增大。

    使用窗口首尾各五分之一样本的中位数，避免单帧深度或检测抖动误触发，
    同时保留足够的首尾时间跨度来识别 0.05m/s 量级的反向移动。
    一旦确认发散，调用方应立即退出 GUIDED 瞄准并交还飞手。
    """

    def __init__(self, growth_m, window_s, buffer_m=0.03, min_samples=6):
        self.growth_m = float(growth_m)
        self.window_s = float(window_s)
        self.buffer_m = float(buffer_m)
        self.min_samples = int(min_samples)
        if self.growth_m <= 0.0:
            raise ValueError("lateral_divergence_growth_m 必须大于 0")
        if self.window_s <= 0.0:
            raise ValueError("lateral_divergence_window_s 必须大于 0")
        if self.buffer_m < 0.0:
            raise ValueError("command_error_buffer_m 不能小于 0")
        if self.min_samples < 4:
            raise ValueError("min_samples 不能小于 4")
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    @staticmethod
    def _median(values):
        ordered = sorted(float(value) for value in values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])

    def observe(self, lateral_error_m, now):
        now = float(now)
        error = abs(float(lateral_error_m))
        if not math.isfinite(now) or not math.isfinite(error):
            self.reset()
            return False

        self.samples.append((now, error))
        cutoff = now - self.window_s
        # 保留截止点之前的最后一个样本，以便窗口跨度能达到 window_s。
        while len(self.samples) >= 2 and self.samples[1][0] <= cutoff:
            self.samples.popleft()

        if (
            len(self.samples) < self.min_samples
            or now - self.samples[0][0] < self.window_s
        ):
            return False

        edge_count = max(2, len(self.samples) // 5)
        baseline = self._median(
            sample[1] for sample in list(self.samples)[:edge_count]
        )
        recent = self._median(
            sample[1] for sample in list(self.samples)[-edge_count:]
        )
        return recent - baseline >= self.growth_m + self.buffer_m


class SemiAutoDropTest:
    def __init__(self):
        self.ground_test = get_bool_param("~ground_test", False)
        self.enable_b_dropper = get_bool_param("~enable_b_dropper", False)
        self.align_threshold_m = float(rospy.get_param("~align_threshold_m", 0.15))
        self.target_drop_height_m = float(
            rospy.get_param("~target_drop_height_m", 1.5)
        )
        self.height_tolerance_m = float(
            rospy.get_param("~height_tolerance_m", 0.15)
        )
        self.stable_time_s = float(rospy.get_param("~stable_time_s", 2.0))
        self.setpoint_interval_s = float(
            rospy.get_param("~setpoint_interval_s", 0.1)
        )
        self.max_horizontal_speed_mps = float(
            rospy.get_param("~max_horizontal_speed_mps", 0.05)
        )
        self.max_target_offset_m = float(
            rospy.get_param("~max_target_offset_m", 2.0)
        )
        self.max_surface_z_drift_m = float(
            rospy.get_param("~max_surface_z_drift_m", 0.50)
        )
        self.lateral_divergence_growth_m = float(
            rospy.get_param("~lateral_divergence_growth_m", 0.10)
        )
        self.lateral_divergence_window_s = float(
            rospy.get_param("~lateral_divergence_window_s", 3.0)
        )
        self.command_error_buffer_m = float(
            rospy.get_param("~command_error_buffer_m", 0.03)
        )
        self.after_a_delay_s = float(rospy.get_param("~after_a_delay_s", 3.0))
        self.auto_start_detector = get_bool_param("~auto_start_detector", True)
        self.auto_start_servo_test = get_bool_param(
            "~auto_start_servo_test", True
        )
        self.close_after_drop = get_bool_param("~close_after_drop", False)
        self.require_disarmed_ground_test = get_bool_param(
            "~require_disarmed_ground_test", True
        )
        self.dry_run = get_bool_param("~dry_run", False)
        self.min_bucket_count = max(1, int(rospy.get_param("~min_bucket_count", 1)))

        self.aim_info_timeout_s = float(
            rospy.get_param("~aim_info_timeout_s", 1.0)
        )
        self.lost_frame_threshold = int(
            rospy.get_param("~lost_frame_threshold", 10)
        )
        self.target_reacquire_timeout_s = float(
            rospy.get_param("~target_reacquire_timeout_s", 4.0)
        )
        self.vision_frame_rate_hz = float(
            rospy.get_param("~vision_frame_rate_hz", 30.0)
        )
        self.state_timeout_s = float(rospy.get_param("~state_timeout_s", 2.0))
        self.connection_timeout_s = float(
            rospy.get_param("~connection_timeout_s", 30.0)
        )
        self.mode_timeout_s = float(rospy.get_param("~mode_timeout_s", 8.0))
        self.servo_wait_timeout_s = float(
            rospy.get_param("~servo_wait_timeout_s", 20.0)
        )
        self.close_delay_s = float(rospy.get_param("~close_delay_s", 0.8))
        self.post_drop_display_s = float(
            rospy.get_param("~post_drop_display_s", 0.0)
        )
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
        if self.max_horizontal_speed_mps <= 0.0:
            raise ValueError("max_horizontal_speed_mps 必须大于 0")
        if self.max_target_offset_m <= 0.0:
            raise ValueError("max_target_offset_m 必须大于 0")
        if self.max_surface_z_drift_m <= 0.0:
            raise ValueError("max_surface_z_drift_m 必须大于 0")
        if self.lateral_divergence_growth_m <= 0.0:
            raise ValueError("lateral_divergence_growth_m 必须大于 0")
        if self.lateral_divergence_window_s <= 0.0:
            raise ValueError("lateral_divergence_window_s 必须大于 0")
        if self.command_error_buffer_m < 0.03:
            raise ValueError("command_error_buffer_m 不能小于 0.03m")
        if self.lost_frame_threshold <= 0:
            raise ValueError("lost_frame_threshold 必须大于 0")
        if self.target_reacquire_timeout_s <= 0.0:
            raise ValueError("target_reacquire_timeout_s 必须大于 0")
        if self.vision_frame_rate_hz <= 0.0:
            raise ValueError("vision_frame_rate_hz 必须大于 0")
        if self.post_drop_display_s < 0.0:
            raise ValueError("post_drop_display_s 不能小于 0")

        self._lock = threading.RLock()
        self.current_state = None
        self.state_received_at = 0.0
        self.latest_aim = None
        self.aim_received_at = 0.0
        self.target_loss_guard = TargetLossGuard(
            self.lost_frame_threshold,
            self.target_reacquire_timeout_s,
            self.vision_frame_rate_hz,
        )
        self.current_pose = None
        self.pose_received_at = 0.0
        self.servo_ready = False
        self.guided_was_requested = False
        self._set_mode_srv = None
        self._children = []
        self._cleanup_done = False
        self.ammo_a = 1
        self.ammo_b = 1 if self.enable_b_dropper else 0

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
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
            self.current_state = msg
            self.state_received_at = monotonic_s()

    def _aim_cb(self, msg):
        now = monotonic_s()
        with self._lock:
            self.latest_aim = msg
            self.aim_received_at = now
            transition = self.target_loss_guard.observe(
                msg.count >= self.min_bucket_count and bool(msg.valid),
                now,
            )
            invalid_frames = self.target_loss_guard.invalid_frames
        if transition == TargetLossGuard.RECOVERED:
            rospy.loginfo(
                "视觉目标在丢失判定前恢复，无效帧计数已清零"
            )
        elif transition == TargetLossGuard.REACQUIRED:
            rospy.logwarn(
                "视觉目标已在 %.1fs 容错时间内重新找到，继续瞄准",
                self.target_reacquire_timeout_s,
            )
        elif (
            transition == TargetLossGuard.LOST
            and invalid_frames == self.lost_frame_threshold
        ):
            rospy.logwarn(
                "视觉目标已连续无效 %d 帧，正式判定丢失；"
                "开始 %.1fs 重获倒计",
                self.lost_frame_threshold,
                self.target_reacquire_timeout_s,
            )

    def _pose_cb(self, msg):
        with self._lock:
            self.current_pose = msg
            self.pose_received_at = monotonic_s()

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

    def _reset_target_loss_guard(self):
        with self._lock:
            self.target_loss_guard.reset()
            msg = self.latest_aim
            if msg is not None:
                self.target_loss_guard.observe(
                    msg.count >= self.min_bucket_count and bool(msg.valid),
                    monotonic_s(),
                )

    def _target_loss_snapshot(self, now):
        with self._lock:
            message_age = now - self.aim_received_at
            if (
                self.latest_aim is None
                or message_age > self.aim_info_timeout_s
            ):
                state = self.target_loss_guard.force_lost(now)
            else:
                state = self.target_loss_guard.status(now)
            return (
                state,
                self.target_loss_guard.invalid_frames,
                self.target_loss_guard.remaining_s(now),
                message_age,
            )

    def _fresh_pose(self):
        with self._lock:
            msg = self.current_pose
            age = monotonic_s() - self.pose_received_at
        if msg is None or age > self.state_timeout_s:
            return None
        return msg

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

    def wait_for_target_and_confirm(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            aim = self._fresh_aim()
            if aim is None:
                rospy.loginfo_throttle(
                    2.0,
                    "等待视觉识别：%s",
                    self._aim_wait_message(),
                )
                rate.sleep()
                continue

            if self.ground_test:
                rospy.logwarn(
                    "地面测试已由启动命令授权：检测到有效桶目标，"
                    "不等待 Enter，直接开始 A 抛投器对准监测"
                )
                return True

            pose = self._fresh_pose()
            height_result = self._height_over_surface(aim, "A", pose)
            if height_result is None:
                rospy.loginfo_throttle(
                    1.0,
                    "目标有效，但暂时无法估算桶面上方高度；保持 LOITER 等待",
                )
                rate.sleep()
                continue
            actual_height, height_error = height_result
            height_limit = self.height_tolerance_m + self.command_error_buffer_m
            if abs(height_error) >= height_limit:
                rospy.logwarn_throttle(
                    1.0,
                    "暂不进入 GUIDED：当前桶面上方高度 %.2fm，目标 %.2fm，"
                    "误差 %+.2fm 超过 %.2fm（含3cm缓冲）；"
                    "请飞手在 LOITER 调整高度",
                    actual_height,
                    self.target_drop_height_m,
                    height_error,
                    height_limit,
                )
                rate.sleep()
                continue

            prompt = (
                "检测到桶，按 Enter 后进入 GUIDED：锁定当前高度和航向，"
                "以最大 {:.2f}m/s 仅水平对中；按 Ctrl+C 取消。"
            ).format(self.max_horizontal_speed_mps)
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
                        ready, _, _ = select.select([stream], [], [], 0.2)
                        if ready:
                            return bool(stream.readline()) and not rospy.is_shutdown()
                finally:
                    if tty is not None:
                        tty.close()
                return False
            except (EOFError, KeyboardInterrupt, OSError):
                return False
        return False

    def _validate_before_guided(
        self,
        dropper,
        expected_mode="LOITER",
        allow_target_reacquire=False,
    ):
        if self.ground_test:
            # 地面测试没有 GUIDED 前置条件。尤其是 B 流程开始瞬间即使目标
            # 正好丢失，也先进入瞄准循环，再由帧计数和重获超时统一判定。
            state = self._fresh_state()
            if state is None:
                rospy.logwarn(
                    "ground_test=true：当前飞控状态消息不新鲜，但地面瞄准不依赖模式/定位"
                )
                if not self.dry_run and self.require_disarmed_ground_test:
                    rospy.logerr(
                        "无法确认飞机处于未解锁状态，拒绝真实地面舵机投放"
                    )
                    return False
            else:
                rospy.logwarn(
                    "ground_test=true：忽略 armed=%s mode=%s，不要求 NED，不进入 GUIDED",
                    state.armed,
                    state.mode,
                )
                if (
                    not self.dry_run
                    and self.require_disarmed_ground_test
                    and state.armed
                ):
                    rospy.logerr(
                        "真实地面舵机测试要求飞机保持未解锁，拒绝投放"
                    )
                    return False
            rospy.logwarn(
                "%s 地面流程直接进入瞄准循环；目标连续无效 %d 帧后"
                "最多再等待 %.1fs",
                dropper,
                self.lost_frame_threshold,
                self.target_reacquire_timeout_s,
            )
            return True

        state = self._fresh_state()
        if state is None or not state.connected:
            rospy.logerr("飞控状态无效，无法执行舵机测试")
            return False

        aim = self._fresh_aim()
        if aim is None and not allow_target_reacquire:
            rospy.logerr("未收到新鲜有效的 /vision/bucket/aim_info，拒绝开始瞄准")
            return False

        if not state.armed:
            rospy.logerr("current_state.armed == false，拒绝进入投放流程")
            return False
        if state.mode != expected_mode:
            rospy.logerr(
                "当前模式不是 %s（实际为 %s），拒绝继续投放",
                expected_mode,
                state.mode,
            )
            return False
        if aim is not None and not getattr(
            aim, "{}_ned_valid".format(dropper.lower())
        ):
            rospy.logerr("%s 的 NED 数据无效，拒绝进入 GUIDED", dropper)
            return False
        pose = self._fresh_pose()
        if pose is None:
            rospy.logerr("本地位姿无效，无法确认 %.2fm 投放高度", self.target_drop_height_m)
            return False
        if aim is not None:
            height_result = self._height_over_surface(aim, dropper, pose)
            if height_result is None:
                rospy.logerr("%s 无法估算桶面上方高度，拒绝进入 GUIDED", dropper)
                return False
            actual_height, height_error = height_result
            height_limit = self.height_tolerance_m + self.command_error_buffer_m
            if abs(height_error) >= height_limit:
                rospy.logerr(
                    "%s 当前桶面上方高度 %.2fm，目标 %.2fm，误差 %+.2fm "
                    "超过 %.2fm（含3cm缓冲）；保持 %s，请飞手调整高度",
                    dropper,
                    actual_height,
                    self.target_drop_height_m,
                    height_error,
                    height_limit,
                    expected_mode,
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

    def _target_enu(self, aim, dropper):
        """返回抛投点正上方指定高度的 ENU 位置目标。"""
        prefix = dropper.lower()
        north = getattr(aim, "{}_ned_n".format(prefix))
        east = getattr(aim, "{}_ned_e".format(prefix))
        down = getattr(aim, "{}_ned_d".format(prefix))
        x, y, target_surface_z = self._ned_to_enu(north, east, down)
        return x, y, target_surface_z + self.target_drop_height_m

    def _height_over_surface(self, aim, dropper, pose):
        """返回 ``(当前桶面上方高度, 目标高度误差)``。"""
        if aim is None or pose is None:
            return None
        prefix = dropper.lower()
        if not getattr(aim, "{}_ned_valid".format(prefix)):
            return None
        _, _, target_z = self._target_enu(aim, dropper)
        surface_z = target_z - self.target_drop_height_m
        actual_height = float(pose.pose.position.z) - surface_z
        if not math.isfinite(actual_height):
            return None
        return actual_height, self.target_drop_height_m - actual_height

    def _publish_pose_setpoint(self, x, y, z, orientation):
        """发布 ENU 位置目标，并保持接管时的航向。"""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation = orientation
        self.setpoint_pub.publish(msg)

    def _publish_limited_setpoint(
        self,
        aim,
        dropper,
        pose,
        commanded_xy,
        hold_z,
        hold_orientation,
        elapsed_s,
    ):
        """仅0.05m/s水平追踪目标，不改变接管时高度。"""
        prefix = dropper.lower()
        if not getattr(aim, "{}_ned_valid".format(prefix)):
            rospy.logerr("%s NED 无效，拒绝发送 setpoint", dropper)
            return None
        north = getattr(aim, "{}_ned_n".format(prefix))
        east = getattr(aim, "{}_ned_e".format(prefix))
        down = getattr(aim, "{}_ned_d".format(prefix))
        target_x, target_y, target_z = self._target_enu(aim, dropper)
        p = pose.pose.position
        target_distance = math.hypot(target_x - float(p.x), target_y - float(p.y))
        if target_distance > self.max_target_offset_m:
            rospy.logerr(
                "%s 水平目标跳变 %.2fm，超过安全上限 %.2fm，拒绝继续自主移动",
                dropper,
                target_distance,
                self.max_target_offset_m,
            )
            return None

        max_step = self.max_horizontal_speed_mps * max(0.0, float(elapsed_s))
        next_x, next_y = limit_xy_step(
            commanded_xy[0], commanded_xy[1], target_x, target_y, max_step
        )
        self._publish_pose_setpoint(next_x, next_y, hold_z, hold_orientation)
        rospy.loginfo_throttle(
            0.5,
            "%s 桶目标 NED=(%.3f, %.3f, %.3f) | 水平限速 %.2fm/s "
            "| raw_xy=(%.3f, %.3f) cmd=(%.3f, %.3f, %.3f)",
            dropper,
            north,
            east,
            down,
            self.max_horizontal_speed_mps,
            target_x,
            target_y,
            next_x,
            next_y,
            hold_z,
        )
        return (next_x, next_y), target_z - self.target_drop_height_m

    def _publish_status(self, aiming=False, last_drop=""):
        msg = MissionStatus()
        msg.ammo_a = self.ammo_a
        msg.ammo_b = self.ammo_b
        msg.aiming = aiming
        msg.last_drop = last_drop
        self.mission_status_pub.publish(msg)

    def aim_dropper(self, dropper, reset_loss_guard=True):
        prefix = dropper.lower()
        aligned_since = None
        next_setpoint_at = 0.0
        last_setpoint_at = monotonic_s()
        commanded_xy = None
        hold_z = None
        hold_orientation = None
        hover_xyz = None
        xy_hold = None
        target_surface_z_ref = None
        lateral_divergence_guard = LateralDivergenceGuard(
            self.lateral_divergence_growth_m,
            self.lateral_divergence_window_s,
            self.command_error_buffer_m,
        )
        rate = rospy.Rate(10)
        if reset_loss_guard:
            self._reset_target_loss_guard()
        if self.ground_test:
            rospy.logwarn(
                "开始 %s 抛投器地面瞄准：连续无效 %d 帧判定丢失，"
                "%.1fs 内未重获则结束流程并模拟交还飞手",
                dropper,
                self.lost_frame_threshold,
                self.target_reacquire_timeout_s,
            )
        else:
            initial_pose = self._fresh_pose()
            if initial_pose is None:
                rospy.logerr("进入 %s 瞄准时本地位姿无效", dropper)
                return False
            p = initial_pose.pose.position
            commanded_xy = (float(p.x), float(p.y))
            hold_z = float(p.z)
            hold_orientation = initial_pose.pose.orientation
            self._publish_pose_setpoint(
                commanded_xy[0], commanded_xy[1], hold_z, hold_orientation
            )
            last_setpoint_at = monotonic_s()
            next_setpoint_at = last_setpoint_at + self.setpoint_interval_s
            rospy.logwarn(
                "开始 %s 抛投器 GUIDED 瞄准：锁定接管高度 %.3fm和航向，"
                "仅水平移动，最大速度 %.2fm/s",
                dropper,
                hold_z,
                self.max_horizontal_speed_mps,
            )

        while not rospy.is_shutdown():
            now = monotonic_s()
            loss_state, invalid_frames, remaining_s, message_age = (
                self._target_loss_snapshot(now)
            )
            pose = None
            if not self.ground_test:
                state = self._fresh_state()
                if state is None or state.mode != "GUIDED":
                    rospy.logerr("%s 瞄准期间 GUIDED 状态丢失", dropper)
                    return False
                if not state.armed:
                    rospy.logerr("%s 瞄准期间飞机已变为未解锁状态", dropper)
                    return False
                pose = self._fresh_pose()
                if pose is None:
                    rospy.logerr("%s 瞄准期间本地位姿丢失", dropper)
                    return False
            if loss_state == TargetLossGuard.TIMEOUT:
                self._publish_status(aiming=False)
                if self.ground_test:
                    rospy.logerr(
                        "%s 目标连续无效至少 %d 帧，且 %.1fs 内"
                        "未重新找到；地面测试结束，模拟交还飞手",
                        dropper,
                        self.lost_frame_threshold,
                        self.target_reacquire_timeout_s,
                    )
                else:
                    rospy.logerr(
                        "%s 目标连续无效至少 %d 帧，且 %.1fs 内"
                        "未重新找到；退出瞄准并切回 LOITER",
                        dropper,
                        self.lost_frame_threshold,
                        self.target_reacquire_timeout_s,
                    )
                return False
            aim = self._fresh_aim()
            if aim is None:
                aligned_since = None
                xy_hold = None
                lateral_divergence_guard.reset()
                self._publish_status(aiming=True)
                if not self.ground_test:
                    if hover_xyz is None:
                        p = pose.pose.position
                        hover_xyz = (float(p.x), float(p.y), hold_z)
                        commanded_xy = (hover_xyz[0], hover_xyz[1])
                        rospy.logwarn(
                            "%s 视觉中断：冻结当前水平位置 "
                            "(%.3f, %.3f)，保持高度 %.3fm",
                            dropper,
                            hover_xyz[0],
                            hover_xyz[1],
                            hover_xyz[2],
                        )
                    if now >= next_setpoint_at:
                        self._publish_pose_setpoint(
                            hover_xyz[0], hover_xyz[1], hover_xyz[2], hold_orientation
                        )
                        last_setpoint_at = now
                        next_setpoint_at = now + self.setpoint_interval_s
                if loss_state == TargetLossGuard.TOLERATING:
                    rospy.logwarn_throttle(
                        0.5,
                        "%s 短暂丢帧 %d/%d，已清零投放稳定计时；"
                        "尚未判定目标丢失",
                        dropper,
                        invalid_frames,
                        self.lost_frame_threshold,
                    )
                else:
                    rospy.logwarn_throttle(
                        0.5,
                        "%s 目标已丢失，等待重获；剩余 %.1fs"
                        "（aim_info age=%.2fs）",
                        dropper,
                        remaining_s,
                        message_age,
                    )
                rate.sleep()
                continue

            if not self.ground_test:
                if not getattr(aim, "{}_ned_valid".format(prefix)):
                    rospy.logerr("%s NED 数据无效，拒绝发送 setpoint", dropper)
                    return False
                if hover_xyz is not None:
                    p = pose.pose.position
                    commanded_xy = (float(p.x), float(p.y))
                    hover_xyz = None
                    xy_hold = None
                    lateral_divergence_guard.reset()
                    last_setpoint_at = now
                    rospy.logwarn(
                        "%s 目标已重新找到，从当前悬停点继续 %.2fm/s 限速对中",
                        dropper,
                        self.max_horizontal_speed_mps,
                    )

            dx = float(getattr(aim, "{}_delta_x_m".format(prefix)))
            dy = float(getattr(aim, "{}_delta_y_m".format(prefix)))
            horizontal_error = math.hypot(dx, dy)
            raw_xy_aligned = (
                abs(dx) < self.align_threshold_m
                and abs(dy) < self.align_threshold_m
            )
            xy_aligned = raw_xy_aligned

            if not self.ground_test:
                release_threshold = (
                    self.align_threshold_m + self.command_error_buffer_m
                )
                p = pose.pose.position
                if xy_hold is not None:
                    if (
                        abs(dx) >= release_threshold
                        or abs(dy) >= release_threshold
                    ):
                        rospy.logwarn(
                            "%s 水平偏差超出 %.2fm 释放阈值，解除对中冻结并继续限速寻找",
                            dropper,
                            release_threshold,
                        )
                        xy_hold = None
                        commanded_xy = (float(p.x), float(p.y))
                        aligned_since = None
                        lateral_divergence_guard.reset()
                        last_setpoint_at = now
                        xy_aligned = False
                    else:
                        xy_aligned = True
                elif raw_xy_aligned:
                    xy_hold = (float(p.x), float(p.y))
                    commanded_xy = xy_hold
                    lateral_divergence_guard.reset()
                    xy_aligned = True
                    rospy.logwarn(
                        "%s 水平已进入 %.2fm 对准范围，冻结当前位置 "
                        "(%.3f, %.3f)；允许额外 %.2fm 指令误差缓冲",
                        dropper,
                        self.align_threshold_m,
                        xy_hold[0],
                        xy_hold[1],
                        self.command_error_buffer_m,
                    )

            if not self.ground_test:
                if now >= next_setpoint_at:
                    elapsed_s = max(
                        self.setpoint_interval_s, now - last_setpoint_at
                    )
                    if xy_hold is not None:
                        _, _, target_z = self._target_enu(aim, dropper)
                        observed_surface_z = target_z - self.target_drop_height_m
                        self._publish_pose_setpoint(
                            xy_hold[0], xy_hold[1], hold_z, hold_orientation
                        )
                        result = (xy_hold, observed_surface_z)
                    else:
                        result = self._publish_limited_setpoint(
                            aim,
                            dropper,
                            pose,
                            commanded_xy,
                            hold_z,
                            hold_orientation,
                            elapsed_s,
                        )
                    if result is None:
                        return False
                    commanded_xy, observed_surface_z = result
                    if target_surface_z_ref is None:
                        target_surface_z_ref = observed_surface_z
                        rospy.loginfo(
                            "%s 锁定首帧桶面 ENU Z=%.3fm，仅用于高度安全判定",
                            dropper,
                            target_surface_z_ref,
                        )
                    elif abs(observed_surface_z - target_surface_z_ref) > self.max_surface_z_drift_m:
                        rospy.logerr(
                            "%s 桶面 Z 漂移 %.2fm，超过 %.2fm 安全上限，退出自主瞄准",
                            dropper,
                            abs(observed_surface_z - target_surface_z_ref),
                            self.max_surface_z_drift_m,
                        )
                        return False
                    last_setpoint_at = now
                    next_setpoint_at = now + self.setpoint_interval_s

            if (
                not self.ground_test
                and xy_hold is None
                and lateral_divergence_guard.observe(horizontal_error, now)
            ):
                rospy.logerr(
                    "%s 水平二维误差在 %.1fs 内持续增大至少 %.2fm"
                    "（基础 %.2fm + 缓冲 %.2fm，当前 XY总偏差=%.3fm）；"
                    "判定方向或坐标异常，"
                    "立即退出 GUIDED 瞄准并切回 LOITER",
                    dropper,
                    self.lateral_divergence_window_s,
                    self.lateral_divergence_growth_m + self.command_error_buffer_m,
                    self.lateral_divergence_growth_m,
                    self.command_error_buffer_m,
                    horizontal_error,
                )
                return False
            height_error = 0.0
            actual_height = self.target_drop_height_m
            height_aligned = True
            if not self.ground_test:
                current_z = float(pose.pose.position.z)
                if target_surface_z_ref is None:
                    actual_height = 0.0
                    height_error = 0.0
                    height_aligned = False
                else:
                    actual_height = current_z - target_surface_z_ref
                    height_error = self.target_drop_height_m - actual_height
                    height_aligned = abs(height_error) < (
                        self.height_tolerance_m + self.command_error_buffer_m
                    )
            aligned = (
                xy_aligned and height_aligned
            )
            if aligned:
                if aligned_since is None:
                    aligned_since = now
                stable = now - aligned_since
            else:
                aligned_since = None
                stable = 0.0

            self._publish_status(aiming=True)
            rospy.loginfo_throttle(
                0.5,
                "%s %s 瞄准 | X偏差=%+.3fm Y偏差=%+.3fm "
                "目标上方高度=%.2fm 高度误差=%+.2fm 稳定时间=%.1f/%.1fs",
                "地面" if self.ground_test else "空中",
                dropper,
                dx,
                dy,
                actual_height,
                height_error,
                stable,
                self.stable_time_s,
            )
            if aligned and stable >= self.stable_time_s:
                rospy.logwarn("%s 连续对准 %.1fs，瞄准完成", dropper, stable)
                self._publish_status(aiming=False)
                return True
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

        # 投放的同时立即通知 detector_node 显示红色 DROP 提示。
        for _ in range(3):
            self._publish_status(aiming=False, last_drop=dropper)
            rospy.sleep(0.05)

        if self.close_after_drop:
            rospy.sleep(max(0.0, self.close_delay_s))
            close_command = "{} off".format(dropper)
            self.servo_cmd_pub.publish(String(data=close_command))
            rospy.logwarn("close_after_drop=true，已发布 /servo/cmd: %s", close_command)

        # 舵机关闭后刷新一次显示计时，确保窗口不会随任务节点
        # 立即退出，而看不清 "A DROP!!!"。
        if self.post_drop_display_s > 0.0:
            self._publish_status(aiming=False, last_drop=dropper)
            rospy.loginfo(
                "保持 %s DROP!!! 红色提示 %.1fs",
                dropper,
                self.post_drop_display_s,
            )
            if not self._interruptible_delay(self.post_drop_display_s):
                return False
        self._publish_status(aiming=False)
        return True

    def _interruptible_delay(self, duration_s):
        deadline = monotonic_s() + max(0.0, duration_s)
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            rospy.sleep(0.1)
        return not rospy.is_shutdown()

    def _wait_between_droppers(self):
        """A 投放后保持同一次 GUIDED 接管，安全等待 B 流程。"""
        deadline = monotonic_s() + max(0.0, self.after_a_delay_s)
        while not rospy.is_shutdown() and monotonic_s() < deadline:
            if not self.ground_test:
                state = self._fresh_state()
                if state is None or not state.connected:
                    rospy.logerr("A/B 间隔期间飞控状态丢失")
                    return False
                if not state.armed:
                    rospy.logerr("A/B 间隔期间飞机已变为未解锁状态")
                    return False
                if state.mode != "GUIDED":
                    rospy.logerr(
                        "A/B 间隔期间 GUIDED 丢失（实际为 %s），"
                        "视为飞手接管",
                        state.mode,
                    )
                    return False

            loss_state, _, remaining_s, _ = self._target_loss_snapshot(
                monotonic_s()
            )
            if loss_state == TargetLossGuard.TIMEOUT:
                rospy.logerr(
                    "A 投放后目标在 %.1fs 内未重新找到，"
                    "不继续 B 投放",
                    self.target_reacquire_timeout_s,
                )
                return False
            if loss_state == TargetLossGuard.LOST:
                rospy.logwarn_throttle(
                    0.5,
                    "A/B 切换期间目标丢失，继续等待重获；剩余 %.1fs",
                    remaining_s,
                )
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
            "半自动投放测试启动 | ground_test=%s B=%s dry_run=%s "
            "xy_threshold=%.3fm target_height=%.2fm height_tolerance=%.2fm "
            "stable=%.1fs min_bucket_count=%d lost_frames=%d vision_fps=%.1f "
            "reacquire=%.1fs xy_speed_limit=%.2fm/s "
            "lateral_divergence=%.2fm+%.2fm_buffer/%.1fs",
            self.ground_test,
            self.enable_b_dropper,
            self.dry_run,
            self.align_threshold_m,
            self.target_drop_height_m,
            self.height_tolerance_m,
            self.stable_time_s,
            self.min_bucket_count,
            self.lost_frame_threshold,
            self.vision_frame_rate_hz,
            self.target_reacquire_timeout_s,
            self.max_horizontal_speed_mps,
            self.lateral_divergence_growth_m,
            self.command_error_buffer_m,
            self.lateral_divergence_window_s,
        )
        if self.ground_test:
            rospy.logwarn(
                "ground_test=true：脚本启动即视为地面测试授权；"
                "目标满足阈值并稳定 %.1fs 后将自动控制真实舵机",
                self.stable_time_s,
            )
        if not self.start_dependencies():
            return 1
        if not self.wait_for_mavros():
            return 1
        if not self._wait_for_servo_subscriber():
            return 1
        if not self.wait_for_target_and_confirm():
            rospy.logwarn("操作者取消半自动投放")
            return 1

        if not self._validate_before_guided("A", expected_mode="LOITER"):
            return 1
        if not self.ground_test and not self.set_mode_confirmed("GUIDED"):
            rospy.logerr("GUIDED 切换或回读确认失败，不继续投放")
            return 1
        if not self.aim_dropper("A", reset_loss_guard=True) or not self.drop("A"):
            return 1

        if not self.enable_b_dropper:
            if not self.safe_loiter("A 投放完成"):
                return 1
            rospy.logwarn("A 流程完成，enable_b_dropper=false，测试结束")
            return 0

        rospy.logwarn(
            "A 投放完成，保持 GUIDED，等待 %.1fs 后在同一次"
            "自动接管中开始 B 瞄准",
            self.after_a_delay_s,
        )
        if not self._wait_between_droppers():
            return 1
        if not self._validate_before_guided(
            "B",
            expected_mode="GUIDED",
            allow_target_reacquire=True,
        ):
            return 1
        # 不重置目标丢失状态：A 投放到 B 瞄准之间仍然连续使用
        # “10 帧判丢 + 4 秒重获”容错，避免切换抛投器时重新起算。
        if not self.aim_dropper("B", reset_loss_guard=False) or not self.drop("B"):
            return 1
        if not self.safe_loiter("A/B 两个投放均完成"):
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
