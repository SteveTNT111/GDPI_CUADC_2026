#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
main.py — CUADC 2026 主控节点

功能：
  1. 通过 MAVROS 连接飞控
  2. 状态机驱动：待命 → 解锁 → 起飞 → 巡航 → 任务 → 返航 → 着陆
  3. 切换飞行模式（GUIDED / AUTO / RTL / LOITER 等）
  4. 发送起飞指令
  5. 监听检测结果，自动切换 GUIDED 模式 + 对准后自动抛投
  6. 集成舵机控制（MAV_CMD_DO_SET_SERVO）

依赖：
  - rospy
  - mavros_msgs (State, SetMode, CommandBool, CommandTOL, CommandLong)
  - geometry_msgs
  - cuadc_vision.msg (BucketInfo, YoloDetection)
  - std_msgs

运行方式：
  roslaunch cuadc_vision run_main.launch
"""

import os
import subprocess
import sys
import time
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from std_msgs.msg import String

from cuadc_vision.msg import BucketInfo, MissionStatus, YoloDetection


# ========================================================================
#  环境自检与自动启动 (roscore / MAVROS)
# ========================================================================

def _ros_master_is_alive():
    """检查 ROS master 是否可达"""
    try:
        import xmlrpc.client
        master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
        master = xmlrpc.client.ServerProxy(master_uri)
        master.getPid('/main_startup_check')
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


def get_bool_param(name, default):
    """解析 ROS 参数为 bool（兼容字符串 "true"/"1" 等写法）"""
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class FlightController:
    """无人机飞行主控 —— 状态机驱动"""

    # 状态机枚举
    STATE_INIT = "INIT"             # 初始化：等待 MAVROS 连接
    STATE_PREARM = "PREARM"         # 预解锁：等待 GPS / EKF 就绪
    STATE_ARMED = "ARMED"           # 已解锁，等待起飞指令
    STATE_TAKEOFF = "TAKEOFF"       # 起飞中
    STATE_HOLD = "HOLD"             # 悬停待命
    STATE_MISSION = "MISSION"       # 任务执行中（巡航 / 识别 / 对准 / 投放）
    STATE_LAND = "LAND"             # 着陆
    STATE_RTL = "RTL"               # 返航
    STATE_COMPLETE = "COMPLETE"     # 任务完成

    def __init__(self):
        # ---------- 参数 ----------
        self.takeoff_altitude = rospy.get_param("~takeoff_altitude", 10.0)  # 起飞高度 (m)
        self.auto_arm = rospy.get_param("~auto_arm", False)                  # 是否自动解锁
        self.auto_takeoff = rospy.get_param("~auto_takeoff", False)          # 是否自动起飞
        self.loiter_time = rospy.get_param("~loiter_time", 5.0)             # 起飞后悬停时间 (s)
        self.ground_test = rospy.get_param("~ground_test", False)            # 地面测试模式
        self.auto_start_mavros = get_bool_param("~auto_start_mavros", True)  # 是否自动启动 MAVROS
        self.mavros_fcu_url = rospy.get_param("~mavros_fcu_url", "/dev/ttyACM0:921600")  # 飞控串口地址

        # ---------- 自动动作 ----------
        self.enable_auto_guided = rospy.get_param("~enable_auto_guided", True)
        self.enable_auto_drop = rospy.get_param("~enable_auto_drop", True)
        self.drop_align_threshold_m = rospy.get_param("~drop_align_threshold_m", 0.10)
        self.drop_stable_time_s = rospy.get_param("~drop_stable_time_s", 3.0)
        self.drop_cooldown_s = rospy.get_param("~drop_cooldown_s", 5.0)

        # ---------- 舵机 ----------
        self.front_servo_channel = int(rospy.get_param("~front_servo_channel", 5))
        self.rear_servo_channel = int(rospy.get_param("~rear_servo_channel", 6))
        self.servo_a_label = "A舵机(前)"
        self.servo_b_label = "B舵机(后)"
        self.pwm_open = rospy.get_param("~pwm_open", 1500)
        self.pwm_close = rospy.get_param("~pwm_close", 1000)
        self.servo_hold_s = rospy.get_param("~servo_hold_s", 0.8)

        # ---------- 状态 ----------
        self.state = self.STATE_INIT
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.state_lock = threading.Lock()

        # ---------- 检测数据 ----------
        self.detect_lock = threading.Lock()
        self.latest_bucket = None       # BucketInfo — 目标数量 + 像素偏差
        self.latest_detection = None    # YoloDetection — 最佳目标相机系坐标

        # ---------- 自动抛投状态 ----------
        self._aligned_since = None      # 对准开始时间 (float seconds)
        self._drop_cooldown_until = 0.0 # 抛投冷却结束时间
        self._drop_triggered = False    # 本轮任务是否已触发过抛投

        # ---------- 舵机状态 ----------
        self.servo_states = {
            self.front_servo_channel: False,
            self.rear_servo_channel: False,
        }
        self.servo_open = False

        # ---------- 弹药 ----------
        self.ammo_a = rospy.get_param("~ammo_a", 1)   # 前抛投器弹药数
        self.ammo_b = rospy.get_param("~ammo_b", 0)   # 后抛投器弹药数 (0=未挂载)

        # ---------- 抛投事件（用于 detector_node 画面显示）----------
        self._last_drop_label = ""            # 当前显示的抛投标签 ("A" / "B" / "")
        self._drop_display_until = 0.0        # 抛投标签显示截止时间

        # ---------- MAVROS 订阅 ----------
        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=10
        )
        self.local_pos_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._pos_cb, queue_size=10
        )
        # 检测结果
        self.bucket_sub = rospy.Subscriber(
            "/vision/bucket/info", BucketInfo, self._bucket_cb, queue_size=5
        )
        self.detection_sub = rospy.Subscriber(
            "/vision/yolo/detection", YoloDetection, self._detection_cb, queue_size=5
        )
        # 外部舵机指令
        self.servo_cmd_sub = rospy.Subscriber(
            "/servo/cmd", String, self._servo_cmd_cb, queue_size=5
        )

        # ---------- MAVROS 服务代理（延迟连接） ----------
        self._set_mode_srv = None
        self._arming_srv = None
        self._takeoff_srv = None
        self._cmd_srv = None     # MAV_CMD_DO_SET_SERVO

        # ---------- 发布 ----------
        self.vel_pub = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "/vision/mission_status", MissionStatus, queue_size=1
        )

        rospy.loginfo("FlightController 初始化完成，等待 MAVROS 连接...")

    # ==================== 回调 ====================

    def _state_cb(self, msg):
        self.current_state = msg

    def _pos_cb(self, msg):
        self.current_pose = msg

    def _bucket_cb(self, msg):
        with self.detect_lock:
            self.latest_bucket = msg

    def _detection_cb(self, msg):
        with self.detect_lock:
            self.latest_detection = msg

    def _servo_cmd_cb(self, msg):
        """外部舵机指令回调（兼容 servo_test 的话题接口）"""
        cmd = " ".join(msg.data.strip().lower().split())
        if cmd in ("qdfs", "all on", "all open", "on", "open", "1", "true"):
            self._set_servo(True)
        elif cmd in ("all off", "off", "close", "0", "false"):
            self._set_servo(False)
        elif cmd == "toggle":
            self._set_servo(not self.servo_open)
        elif cmd.startswith(("a ", "ch5 ", "5 ", "front ")):
            self._handle_target_servo_command(
                cmd.split(" ", 1)[1],
                self.front_servo_channel,
                self.servo_a_label,
            )
        elif cmd.startswith(("b ", "ch6 ", "6 ", "rear ")):
            self._handle_target_servo_command(
                cmd.split(" ", 1)[1],
                self.rear_servo_channel,
                self.servo_b_label,
            )

    def _handle_target_servo_command(self, action, servo_num, label):
        if action in ("on", "open", "1", "true"):
            self._set_single_servo(servo_num, label, True)
        elif action in ("off", "close", "0", "false"):
            self._set_single_servo(servo_num, label, False)
        elif action == "toggle":
            self._set_single_servo(servo_num, label, not self.servo_states[servo_num])

    # ==================== 服务连接 ====================

    def _ensure_services(self, timeout=10.0):
        """确保所有 MAVROS 服务已连接"""
        if self._set_mode_srv is not None:
            return True
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout)
            rospy.wait_for_service("/mavros/cmd/arming", timeout)
            rospy.wait_for_service("/mavros/cmd/takeoff", timeout)
            self._set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self._arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            self._takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
            # 舵机命令服务（可选，不阻塞启动）
            try:
                rospy.wait_for_service("/mavros/cmd/command", timeout=3.0)
                self._cmd_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
            except rospy.ROSException:
                rospy.loginfo("舵机服务 /mavros/cmd/command 不可用（抛投功能将禁用）")

            rospy.loginfo("所有 MAVROS 服务已连接")
            return True
        except rospy.ROSException as e:
            rospy.logwarn_throttle(5.0, "等待 MAVROS 服务: %s", e)
            return False

    # ==================== 飞控指令 ====================

    def set_mode(self, mode, timeout=5.0):
        """切换飞行模式

        Args:
            mode: PX4 模式名，如 "GUIDED", "AUTO", "RTL", "LOITER", "LAND"
            timeout: 服务调用超时 (s)
        """
        if not self._ensure_services():
            rospy.logerr("无法切换模式 —— MAVROS 服务未就绪")
            return False
        try:
            resp = self._set_mode_srv(custom_mode=mode)
            if resp.mode_sent:
                rospy.loginfo("飞行模式切换至: %s", mode)
                return True
            else:
                rospy.logerr("飞行模式切换失败: %s", mode)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("set_mode 服务调用失败: %s", e)
            return False

    def arm(self, timeout=5.0):
        """解锁无人机"""
        if not self._ensure_services():
            return False
        if self.current_state.armed:
            rospy.loginfo("无人机已解锁")
            return True
        try:
            resp = self._arming_srv(value=True)
            if resp.success:
                rospy.loginfo("解锁成功")
                return True
            else:
                rospy.logerr("解锁失败: result=%d", resp.result)
                return False
        except rospy.ServiceException as e:
            rospy.logerr("arming 服务调用失败: %s", e)
            return False

    def disarm(self, timeout=5.0):
        """上锁无人机"""
        if not self._ensure_services():
            return False
        try:
            resp = self._arming_srv(value=False)
            if resp.success:
                rospy.loginfo("上锁成功")
                return True
            else:
                rospy.logerr("上锁失败")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("disarm 服务调用失败: %s", e)
            return False

    def takeoff(self, altitude=None):
        """发送起飞指令（需先切换到 GUIDED 模式并解锁）

        Args:
            altitude: 起飞目标高度 (m)，默认使用参数 takeoff_altitude
        """
        if not self._ensure_services():
            return False
        if altitude is None:
            altitude = self.takeoff_altitude
        try:
            resp = self._takeoff_srv(
                altitude=altitude,
                latitude=0.0,
                longitude=0.0,
                min_pitch=0.0,
                yaw=0.0,
            )
            if resp.success:
                rospy.loginfo("起飞指令已发送，目标高度 %.1f m", altitude)
                return True
            else:
                rospy.logerr("起飞指令发送失败")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("takeoff 服务调用失败: %s", e)
            return False

    def land(self):
        """切换到 LAND 模式"""
        return self.set_mode("LAND")

    def rtl(self):
        """切换到 RTL（返航）模式"""
        return self.set_mode("RTL")

    # ==================== 舵机控制（servo_test.py 核心逻辑） ====================

    def _ensure_cmd_service(self):
        """确保 /mavros/cmd/command 服务可用"""
        if self._cmd_srv is not None:
            return True
        try:
            rospy.wait_for_service("/mavros/cmd/command", timeout=3.0)
            self._cmd_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
            return True
        except rospy.ROSException:
            return False

    def _set_servo(self, state):
        """同时设置前后两个舵机的开关状态。"""
        return self._set_servo_targets(
            (
                (self.front_servo_channel, self.servo_a_label),
                (self.rear_servo_channel, self.servo_b_label),
            ),
            state,
        )

    def _set_single_servo(self, servo_num, label, state):
        """通过 MAV_CMD_DO_SET_SERVO (command=183) 设置单个舵机 PWM。"""
        pwm = self.pwm_open if state else self.pwm_close
        action = "打开" if state else "关闭"

        if not self._ensure_cmd_service():
            rospy.logerr("舵机 %s(CH%d) → %s 失败: /mavros/cmd/command 服务不可用",
                         label, servo_num, action)
            return False

        ok = self._call_set_servo(servo_num, pwm)
        if ok:
            self.servo_states[servo_num] = state
            self.servo_open = any(self.servo_states.values())
            rospy.loginfo("舵机 %s(CH%d) → %s (PWM=%d)", label, servo_num, action, pwm)
        else:
            rospy.logerr("舵机 %s(CH%d) → %s 失败", label, servo_num, action)
        return ok

    def _set_servo_targets(self, targets, state):
        """批量设置指定舵机。"""
        ok = True
        for servo_num, label in targets:
            ok = self._set_single_servo(servo_num, label, state) and ok
        return ok

    def _call_set_servo(self, servo_num, pwm):
        """发送 MAV_CMD_DO_SET_SERVO 指令

        command=183: MAV_CMD_DO_SET_SERVO
        param1=servo_num: 舵机编号 (5=SERVO5, 6=SERVO6)
        param2=pwm: PWM 微秒值
        """
        try:
            resp = self._cmd_srv(
                broadcast=False,
                command=183,
                confirmation=0,
                param1=float(servo_num),
                param2=float(pwm),
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0,
            )
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("MAV_CMD_DO_SET_SERVO(%d, %d) 失败: %s", servo_num, pwm, e)
            self._cmd_srv = None
            return False

    # ==================== 自动任务逻辑 ====================

    def _check_auto_guided(self):
        """检测到目标 → 自动切换 GUIDED 模式"""
        if not self.enable_auto_guided:
            return
        with self.detect_lock:
            bucket = self.latest_bucket
        if bucket is None:
            return
        if bucket.count > 0 and self.current_state.mode != "GUIDED":
            rospy.loginfo("检测到 %d 个目标 → 自动切换 GUIDED 模式", bucket.count)
            self.set_mode("GUIDED")

    def _check_auto_drop(self):
        """对准判断 + 稳定计时 + 自动抛投"""
        if not self.enable_auto_drop:
            return

        now = time.time()
        if now < self._drop_cooldown_until:
            return

        with self.detect_lock:
            bucket = self.latest_bucket
            det = self.latest_detection

        if bucket is None or det is None:
            return
        if bucket.count <= 0:
            if self._aligned_since is not None:
                rospy.loginfo("目标丢失，重置对准计时")
            self._aligned_since = None
            return
        if not det.position_valid:
            if self._aligned_since is not None:
                rospy.loginfo("深度无效，重置对准计时")
            self._aligned_since = None
            return

        # 检查相机系 XY 偏移是否在阈值内
        cx = abs(det.camera_x_m)
        cy = abs(det.camera_y_m)
        aligned = cx < self.drop_align_threshold_m and cy < self.drop_align_threshold_m

        if aligned:
            if self._aligned_since is None:
                self._aligned_since = now
                rospy.loginfo(
                    "目标已对准 | x=%.3f y=%.3f (< %.0fcm) | 开始 %.1fs 稳定计时...",
                    det.camera_x_m, det.camera_y_m,
                    self.drop_align_threshold_m * 100, self.drop_stable_time_s,
                )
            else:
                elapsed = now - self._aligned_since
                if elapsed >= self.drop_stable_time_s:
                    rospy.loginfo(
                        "稳定对准 %.1fs | x=%.3f y=%.3f | 触发抛投！",
                        elapsed, det.camera_x_m, det.camera_y_m,
                    )
                    self._do_drop()
        else:
            if self._aligned_since is not None:
                rospy.loginfo(
                    "目标偏离阈值 | x=%.3f(%.0fcm) y=%.3f(%.0fcm) | 重置计时",
                    det.camera_x_m, abs(det.camera_x_m) * 100,
                    det.camera_y_m, abs(det.camera_y_m) * 100,
                )
            self._aligned_since = None

    def _do_drop(self):
        """执行抛投动作：打开 → 保持 → 关闭"""
        now = time.time()

        # 确定使用哪个抛投器（优先使用前抛投器 A）
        if self.ammo_a > 0:
            dropper = "A"
            servo_num = self.front_servo_channel
            servo_label = self.servo_a_label
        elif self.ammo_b > 0:
            dropper = "B"
            servo_num = self.rear_servo_channel
            servo_label = self.servo_b_label
        else:
            rospy.logwarn("弹药耗尽，无法抛投！")
            return

        if not self._set_single_servo(servo_num, servo_label, True):
            rospy.logerr("%s 抛投失败：打开舵机失败", dropper)
            return

        rospy.sleep(self.servo_hold_s)
        if not self._set_single_servo(servo_num, servo_label, False):
            rospy.logwarn("%s 抛投结束后关闭舵机失败，请检查通道 %d", dropper, servo_num)

        if dropper == "A":
            self.ammo_a -= 1
        else:
            self.ammo_b -= 1

        self._drop_cooldown_until = now + self.drop_cooldown_s
        self._aligned_since = None
        self._drop_triggered = True

        # 设置抛投显示事件（detector_node 将显示 "A DROP!!!" 3 秒）
        self._last_drop_label = dropper
        self._drop_display_until = now + 3.0

        rospy.loginfo("抛投完成！%s 抛投器 | 剩余 A=%d B=%d | 冷却 %.1fs",
                      dropper, self.ammo_a, self.ammo_b, self.drop_cooldown_s)

    def _publish_status(self):
        """向 detector_node 发布任务状态（弹药 + 瞄准 + 抛投事件）"""
        msg = MissionStatus()
        msg.ammo_a = self.ammo_a
        msg.ammo_b = self.ammo_b
        msg.aiming = (self.state == self.STATE_MISSION and
                      self.current_state.mode == "GUIDED" and
                      self.current_state.connected)
        # 抛投显示标签：3 秒后自动清除
        if self._last_drop_label and time.time() < self._drop_display_until:
            msg.last_drop = self._last_drop_label
        else:
            msg.last_drop = ""
            self._last_drop_label = ""
        self.status_pub.publish(msg)

    # ==================== 状态机 ====================

    def _wait_for_connection(self):
        """等待与飞控建立 MAVROS 连接"""
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.current_state.connected:
                rospy.loginfo("飞控已连接")
                return True
            rospy.loginfo_throttle(5.0, "等待飞控连接...")
            rate.sleep()
        return False

    def _wait_for_ekf(self):
        """等待 EKF 融合就绪（GPS + IMU）

        通过检查 /mavros/local_position/pose 数据是否有效来判断 EKF 是否收敛。
        PoseStamped 不含协方差字段，改为检查位置数值是否已更新（非全零）。

        地面测试模式（ground_test=true）下直接跳过，室内无 GPS 也能测。
        """
        if self.ground_test:
            rospy.loginfo("地面测试模式：跳过 EKF 检查")
            return True

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            pos = self.current_pose.pose.position
            # EKF 收敛 = 已收到有效位置数据（至少 z 不为 0）
            has_pose = abs(pos.x) > 0.001 or abs(pos.y) > 0.001 or abs(pos.z) > 0.001
            if has_pose and self.current_state.connected:
                rospy.loginfo("EKF 就绪 (x=%.3f y=%.3f z=%.3f)", pos.x, pos.y, pos.z)
                return True
            rospy.loginfo_throttle(5.0, "等待 EKF 收敛... (x=%.3f y=%.3f z=%.3f)",
                                   pos.x, pos.y, pos.z)
            rate.sleep()
        return False

    def _transition(self, new_state):
        """线程安全的状态切换"""
        with self.state_lock:
            old = self.state
            self.state = new_state
        rospy.loginfo("状态切换: %s → %s", old, new_state)

    def run(self):
        """主循环 —— 状态机"""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # ========== INIT: 等待 MAVROS 连接 ==========
            if self.state == self.STATE_INIT:
                if self._wait_for_connection():
                    if not self._ensure_services():
                        rate.sleep()
                        continue
                    self._transition(self.STATE_PREARM)

            # ========== PREARM: 等待 EKF / GPS 就绪 ==========
            elif self.state == self.STATE_PREARM:
                self._wait_for_ekf()
                if self.auto_arm:
                    self._transition(self.STATE_ARMED)
                else:
                    rospy.loginfo("auto_arm=false，等待手动 unlocking...")
                    # 循环等待直到解锁
                    while not rospy.is_shutdown():
                        if self.current_state.armed:
                            self._transition(self.STATE_ARMED)
                            break
                        rate.sleep()

            # ========== ARMED: 起飞 ==========
            elif self.state == self.STATE_ARMED:
                if self.ground_test:
                    # 地面测试模式：跳过 GUIDED 切换和起飞，直接进入 MISSION
                    rospy.loginfo("地面测试模式：解锁后直接进入 MISSION 状态")
                    self._transition(self.STATE_MISSION)
                    continue

                # 确保 GUIDED 模式
                if self.current_state.mode != "GUIDED":
                    self.set_mode("GUIDED")
                    rospy.sleep(1.0)
                    continue

                if self.auto_takeoff:
                    if self.takeoff():
                        self._transition(self.STATE_TAKEOFF)
                    else:
                        rospy.sleep(1.0)
                else:
                    rospy.loginfo("auto_takeoff=false，等待手动起飞指令...")
                    rospy.sleep(1.0)

            # ========== TAKEOFF: 等待到达目标高度 ==========
            elif self.state == self.STATE_TAKEOFF:
                current_alt = self.current_pose.pose.position.z
                if current_alt >= self.takeoff_altitude * 0.9:
                    rospy.loginfo("到达目标高度 %.1f m，进入悬停", current_alt)
                    self._transition(self.STATE_HOLD)
                else:
                    rospy.loginfo_throttle(2.0, "爬升中: %.1f / %.1f m",
                                           current_alt, self.takeoff_altitude)

            # ========== HOLD: 悬停待命 ==========
            elif self.state == self.STATE_HOLD:
                rospy.loginfo_throttle(5.0, "悬停中... (mode=%s, armed=%s)",
                                       self.current_state.mode, self.current_state.armed)

            # ========== MISSION: 任务执行 ==========
            elif self.state == self.STATE_MISSION:
                self._check_auto_guided()
                self._check_auto_drop()
                rospy.loginfo_throttle(5.0, "任务执行中... | mode=%s armed=%s",
                                       self.current_state.mode, self.current_state.armed)

            # ========== LAND: 着陆 ==========
            elif self.state == self.STATE_LAND:
                self.land()
                # 等待触地后上锁
                rospy.sleep(3.0)
                self._transition(self.STATE_COMPLETE)

            # ========== RTL: 返航 ==========
            elif self.state == self.STATE_RTL:
                self.rtl()
                self._transition(self.STATE_COMPLETE)

            # ========== COMPLETE: 任务完成 ==========
            elif self.state == self.STATE_COMPLETE:
                rospy.loginfo("任务流程完成")
                break

            self._publish_status()
            rate.sleep()

        # 主循环退出
        rospy.loginfo("FlightController 退出")

    # ==================== 外部控制接口（ROS 服务/话题可调用） ====================

    def trigger_takeoff(self):
        """外部触发起飞（可由服务调用）"""
        if self.state in (self.STATE_ARMED, self.STATE_HOLD):
            self._transition(self.STATE_TAKEOFF)
            return self.takeoff()
        else:
            rospy.logwarn("当前状态 %s 不允许起飞", self.state)
            return False

    def trigger_land(self):
        """外部触发着陆"""
        self._transition(self.STATE_LAND)

    def trigger_rtl(self):
        """外部触发返航"""
        self._transition(self.STATE_RTL)

    def set_mission(self):
        """进入任务状态"""
        if self.state == self.STATE_HOLD:
            self._transition(self.STATE_MISSION)
        else:
            rospy.logwarn("当前状态 %s 不允许进入任务", self.state)


def main():
    # ---- 第 0 步：环境自检，确保 roscore 在运行 ----
    _ensure_roscore()

    # ---- 第 1 步：初始化节点 ----
    rospy.init_node("flight_controller")

    # ---- 第 2 步：按需启动 MAVROS ----
    auto_start = get_bool_param("~auto_start_mavros", True)
    if auto_start:
        fcu_url = rospy.get_param("~mavros_fcu_url", "/dev/ttyACM0:921600")
        _ensure_mavros(fcu_url)

    # ---- 第 3 步：创建主控并运行 ----
    fc = FlightController()
    try:
        fc.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
