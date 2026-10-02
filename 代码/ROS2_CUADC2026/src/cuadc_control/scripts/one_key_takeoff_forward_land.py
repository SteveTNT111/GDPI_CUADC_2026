#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
one_key_takeoff_forward_land.py — 起飞 → 前飞 → 降落（ROS2/Humble 版）
========================================================================
由 ROS1 版 `代码/比赛main工作版本/src/cuadc_control/scripts/
one_key_takeoff_forward_land.py` 逐行翻译而来。

流程（前提：满足解锁条件后，脚本在终端按 Enter 自解锁；SITL 无遥控器）：
  GUIDED -> 按 Enter 解锁 -> 起飞 2 m -> 悬停 2 s -> 沿当前罗盘航向前飞 1 m
  -> 悬停 3 s -> LAND -> 等飞控自动落地并上锁。

安全约束（2026-09-28 修订：按用户要求支持终端 Enter 自解锁，免遥控器）：
  - 解锁只由 `_request_arm_by_enter()` 在操作者按 Enter 后触发；绝不自动静默解锁。
  - 起飞前必须从 /mavros/state 确认 armed=True，否则立即退出。
  - 飞行阶段失败时仅尝试切换 LAND；不发送上锁命令。

坐标约定（与原版一致）：
  - 飞控本地坐标按 NED 解算：x=North, y=East, z=Down。
  - /mavros/local_position/pose 使用 ROS ENU：x=East, y=North, z=Up。
  - 航点先在 NED 中计算，再转换为 ENU 发布到
    /mavros/setpoint_position/local。

高度参考系（2026-09-29 统一，错题 #40）：
  - 一切高度判据/指令都用 **相对起飞点**：`rel_alt`（相对 Home）+ 解锁后地面
    记录的 `ground_rel_alt` 零点；起飞命令发 `ground_rel_alt + 计划爬升`。
  - `pose.z`（EKF 局部原点以上）仅用于本地 ENU setpoint 坐标；把目标高度换算
    回本地 z 时用桥接 `pose.z + target_rel_alt − rel_alt`，不直接假定 local z=高度。

ROS1 -> ROS2 翻译要点：
  - `rospy.init_node` -> `rclpy.init() + create_node`；类构造改为显式传入 rclpy Node。
  - 私有参数 `rospy.get_param("~x", d)` -> `declare_parameter/get_parameter`（参数名去掉 `~`，
    运行时用 `--ros-args -p x:=v` 覆盖）。
  - `rospy.Subscriber(queue_size=)` -> `node.create_subscription`（MAVROS 导航话题为
    BEST_EFFORT，统一用 BEST_EFFORT QoS；/mavros/state 为 RELIABLE，BEST_EFFORT 订阅兼容）。
    新增 `/mavros/global_position/compass_hdg`、`/mavros/global_position/rel_alt`（Float64）同 QoS。
  - `rospy.Publisher(..., queue_size=20)` -> `create_publisher(depth=20, RELIABLE)`：
    RELIABLE 发布对 BEST_EFFORT / RELIABLE 订阅方均兼容（DDS offer >= request）。
  - `rospy.ServiceProxy 同步调用` -> `create_client + call_async` + 轮询 future
    （本节点由后台 MultiThreadedExecutor spin，轮询避免与 spin() 冲突丢回调）。
  - `rospy.Rate` -> `node.create_rate`；`rospy.is_shutdown()` -> `rclpy.ok() and node.context.ok()`；
    `rospy.Time.now()` -> `node.get_clock().now().to_msg()`；`rospy.log*` -> `node.get_logger().*`
    （节流日志用本地 `throttle_log`）。
  - ROS1 自动启动 roscore 的逻辑删除（ROS2 无 master）；MAVROS 未运行且
    auto_start_mavros=true 时自动 `ros2 launch mavros apm.launch fcu_url:=...`
    （ROS2 图发现异步，先给 mavros_discovery_grace 宽限窗口，避免重复启动）。
  - `rospy.get_published_topics()` 检查 MAVROS -> `node.count_publishers("/mavros/state") > 0`。

运行方式（终端按 Enter 自解锁；验证脚本亦可）：
  ros2 run cuadc_control one_key_takeoff_forward_land.py
  ros2 run cuadc_control one_key_takeoff_forward_land.py --ros-args -p forward_distance:=2.0
"""

import copy
import math
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


def now_s():
    return time.monotonic()


def node_param(node, name, default):
    """声明并读取节点参数（等价于 ROS1 的 rospy.get_param('~name', default)）。"""
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    try:
        return node.get_parameter(name).value
    except Exception:
        return default


def throttle_log(node, interval_s, fmt, *args):
    """简单节流日志：等价 ROS1 的 rospy.loginfo_throttle(interval_s, fmt, *args)。"""
    name = "_throttle_last_" + str(abs(hash(fmt)))
    last = getattr(node, name, 0.0)
    if now_s() - last >= interval_s:
        setattr(node, name, now_s())
        node.get_logger().info(fmt % args if args else fmt)


class TakeoffForwardLandNode:
    """执行已解锁前提下的起飞、前飞和降落流程。"""

    def __init__(self, node):
        self.node = node

        self.takeoff_altitude = float(node_param(node, "takeoff_altitude", 2.0))
        self.forward_distance = float(node_param(node, "forward_distance", 1.0))
        self.takeoff_hover_time = float(node_param(node, "takeoff_hover_time", 2.0))
        self.waypoint_hover_time = float(node_param(node, "waypoint_hover_time", 3.0))

        self.auto_start_mavros = bool(node_param(node, "auto_start_mavros", False))
        self.mavros_fcu_url = node_param(node, "mavros_fcu_url", "/dev/ttyACM0:115200")
        # ROS2 的图发现是异步的：新建节点后 count_publishers("/mavros/state")
        # 可能短暂返回 0。给一个宽限窗口，避免把"已运行的 MAVROS"误判为未运行
        # 而重复启动（错题本：ROS1 get_published_topics 经 master 同步无此问题）。
        self.mavros_discovery_grace = float(
            node_param(node, "mavros_discovery_grace", 3.0)
        )

        self.connection_timeout = float(node_param(node, "connection_timeout", 30.0))
        self.service_timeout = float(node_param(node, "service_timeout", 10.0))
        self.data_timeout = float(node_param(node, "data_timeout", 30.0))
        self.data_max_age = float(node_param(node, "data_max_age", 2.0))
        self.mode_timeout = float(node_param(node, "mode_timeout", 5.0))
        self.takeoff_timeout = float(node_param(node, "takeoff_timeout", 30.0))
        self.waypoint_timeout = float(node_param(node, "waypoint_timeout", 20.0))
        self.landing_timeout = float(node_param(node, "landing_timeout", 90.0))

        self.takeoff_tolerance = max(
            0.02, float(node_param(node, "takeoff_tolerance", 0.15))
        )
        self.waypoint_tolerance = max(
            0.02, float(node_param(node, "waypoint_tolerance", 0.15))
        )
        self.vertical_tolerance = max(
            0.02, float(node_param(node, "vertical_tolerance", 0.20))
        )
        self.setpoint_rate = max(5.0, float(node_param(node, "setpoint_rate", 20.0)))

        if self.takeoff_altitude <= 0.0:
            raise ValueError("takeoff_altitude 必须大于 0")
        if self.forward_distance <= 0.0:
            raise ValueError("forward_distance 必须大于 0")

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.heading_deg = None
        self.relative_altitude = None
        # 高度统一为"相对起飞点"：rel_alt 相对 Home，MAV_CMD_NAV_TAKEOFF 亦然。
        # 解锁后在地面记录 ground_rel_alt 作零点，命令/门控/悬停航点共用它。
        self.ground_rel_alt = None
        self.target_rel_alt = None

        self._state_received = False
        self._pose_received = False
        self._heading_received = False
        self._relative_altitude_received = False
        self._pose_received_at = 0.0
        self._heading_received_at = 0.0
        self._relative_altitude_received_at = 0.0

        self._set_mode_srv = None
        self._takeoff_srv = None
        self._arming_srv = None

        # MAVROS 传感器类话题（local_position/pose、global_position/*）以 BEST_EFFORT
        # 发布；统一用 BEST_EFFORT + depth 10：BEST_EFFORT 订阅对 RELIABLE 与
        # BEST_EFFORT 发布方均兼容（/mavros/state 为 RELIABLE）。
        nav_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_sub = self.node.create_subscription(
            State, "/mavros/state", self._state_cb, nav_qos
        )
        self.pose_sub = self.node.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, nav_qos
        )
        self.heading_sub = self.node.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self._heading_cb, nav_qos
        )
        self.relative_altitude_sub = self.node.create_subscription(
            Float64, "/mavros/global_position/rel_alt",
            self._relative_altitude_cb, nav_qos,
        )
        # 位置 setpoint 流：RELIABLE 发布对 BEST_EFFORT/RELIABLE 订阅均兼容。
        setpoint_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.setpoint_pub = self.node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", setpoint_qos
        )

        # 可选：外部 MultiThreadedExecutor（后台 spin 时使用），避免内部
        # spin_until_future_complete 与已有 spin 线程冲突。
        self._executor = None

    # ------------------------------------------------------------------
    # 订阅回调（由后台 spin 线程驱动）
    # ------------------------------------------------------------------
    def _state_cb(self, msg):
        self.current_state = msg
        self._state_received = True

    def _pose_cb(self, msg):
        self.current_pose = msg
        self._pose_received = True
        self._pose_received_at = time.monotonic()

    def _heading_cb(self, msg):
        if math.isfinite(msg.data):
            self.heading_deg = msg.data % 360.0
            self._heading_received = True
            self._heading_received_at = time.monotonic()

    def _relative_altitude_cb(self, msg):
        if math.isfinite(msg.data):
            self.relative_altitude = msg.data
            self._relative_altitude_received = True
            self._relative_altitude_received_at = time.monotonic()

    # ------------------------------------------------------------------
    # MAVROS 检测 / 自动启动
    # ------------------------------------------------------------------
    def _mavros_is_alive(self):
        """检查 MAVROS 是否在运行（等价 ROS1 的 get_published_topics 检查）。"""
        try:
            return self.node.count_publishers("/mavros/state") > 0
        except Exception:
            return False

    def _wait_for_mavros(self, timeout=30.0):
        """等待 MAVROS 上线。"""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.node.context.ok():
            if self._mavros_is_alive():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(1.0)
        return False

    def _ensure_mavros(self):
        """确保 MAVROS 在运行（先给图发现一个宽限窗口，见 __init__ 中说明）。"""
        if self._wait_for_mavros(timeout=self.mavros_discovery_grace):
            self.node.get_logger().info("[auto-start] MAVROS 已在运行")
            return True
        if not self.auto_start_mavros:
            self.node.get_logger().error("MAVROS 未运行，且 auto_start_mavros=false")
            return False

        self.node.get_logger().info(
            "[auto-start] 启动 MAVROS: fcu_url=%s" % self.mavros_fcu_url
        )
        try:
            subprocess.Popen(
                [
                    "ros2", "launch", "mavros", "apm.launch",
                    "fcu_url:={}".format(self.mavros_fcu_url),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            self.node.get_logger().error("[auto-start] 启动 MAVROS 失败: %s" % exc)
            return False

        if self._wait_for_mavros(timeout=self.connection_timeout):
            self.node.get_logger().info("[auto-start] MAVROS 已就绪")
            return True
        self.node.get_logger().error(
            "[auto-start] MAVROS 启动超时（%.1fs）" % self.connection_timeout
        )
        return False

    # ------------------------------------------------------------------
    # 等待连接 / 导航数据
    # ------------------------------------------------------------------
    def _wait_for_connection(self):
        """等待飞控连接建立。"""
        self.node.get_logger().info("等待飞控连接 ...")
        deadline = time.monotonic() + self.connection_timeout
        rate = self.node.create_rate(5)
        while rclpy.ok() and self.node.context.ok():
            if self._state_received and self.current_state.connected:
                self.node.get_logger().info("飞控已连接")
                return True
            if time.monotonic() >= deadline:
                self.node.get_logger().error(
                    "等待飞控连接超时（%.1fs）" % self.connection_timeout
                )
                return False
            rate.sleep()
        return False

    def _wait_for_navigation_data(self):
        """等待本地位置与飞控航向；零坐标也是有效数据。"""
        self.node.get_logger().info("等待本地位置和航向数据 ...")
        deadline = time.monotonic() + self.data_timeout
        rate = self.node.create_rate(10)
        while rclpy.ok() and self.node.context.ok():
            if (
                self._pose_received
                and self._heading_received
                and self._relative_altitude_received
            ):
                pos = self.current_pose.pose.position
                self.node.get_logger().info(
                    "导航数据就绪: ENU=(%.3f, %.3f, %.3f), "
                    "heading=%.2f deg, rel_alt=%.2f m"
                    % (
                        pos.x,
                        pos.y,
                        pos.z,
                        self.heading_deg,
                        self.relative_altitude,
                    )
                )
                return True
            if time.monotonic() >= deadline:
                self.node.get_logger().error(
                    "等待导航数据超时: local_pose=%s compass_hdg=%s rel_alt=%s"
                    % (
                        self._pose_received,
                        self._heading_received,
                        self._relative_altitude_received,
                    )
                )
                return False
            rate.sleep()
        return False

    def _navigation_data_is_fresh(self):
        now = time.monotonic()
        return (
            self._pose_received
            and self._heading_received
            and self._relative_altitude_received
            and now - self._pose_received_at <= self.data_max_age
            and now - self._heading_received_at <= self.data_max_age
            and now - self._relative_altitude_received_at <= self.data_max_age
        )

    # ------------------------------------------------------------------
    # 服务
    # ------------------------------------------------------------------
    def _ensure_services(self):
        """连接模式、起飞和解锁服务（房规：满足条件后由脚本按 Enter 自解锁）。"""
        if self._set_mode_srv is not None:
            return True
        try:
            self._set_mode_srv = self.node.create_client(SetMode, "/mavros/set_mode")
            self._takeoff_srv = self.node.create_client(CommandTOL, "/mavros/cmd/takeoff")
            self._arming_srv = self.node.create_client(CommandBool, "/mavros/cmd/arming")
            if not (
                self._wait_service(self._set_mode_srv, self.service_timeout)
                and self._wait_service(self._takeoff_srv, self.service_timeout)
                and self._wait_service(self._arming_srv, self.service_timeout)
            ):
                self.node.get_logger().error("MAVROS basic services unavailable")
                return False
            self.node.get_logger().info("模式、起飞与解锁服务已连接")
            return True
        except Exception as exc:
            self.node.get_logger().error("等待 MAVROS 服务失败: %s" % exc)
            return False

    def _wait_service(self, client, timeout_s):
        """等价 rospy.wait_for_service；已存在或超时均返回 bool。"""
        if client is None:
            return False
        try:
            if client.service_is_ready():
                return True
            return client.wait_for_service(timeout_sec=timeout_s)
        except Exception:
            return False

    def _call_service(self, client, request, timeout_s):
        """等价 ROS1 ServiceProxy(...)(req) 的同步调用，阻塞直到完成或超时。"""
        if client is None:
            return (False, None)
        if not self._wait_service(client, timeout_s):
            self.node.get_logger().warn("service %s not available" % client.srv_name)
            return (False, None)
        try:
            future = client.call_async(request)
            if self._executor is not None:
                # 后台 spin 线程统一处理订阅回调与服务响应；
                # 这里只轮询 future 完成状态，避免同一 executor 上
                # 再调用 spin_until_future_complete 造成与 spin() 冲突而丢回调。
                deadline = time.monotonic() + timeout_s
                while not future.done():
                    if time.monotonic() >= deadline:
                        break
                    time.sleep(0.02)
            else:
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        except Exception as exc:
            self.node.get_logger().error("service call failed: %s" % exc)
            return (False, None)
        if future.result() is None:
            return (False, None)
        return (True, future.result())

    # ------------------------------------------------------------------
    # 状态等待 / 模式切换
    # ------------------------------------------------------------------
    def _wait_for_mode(self, expected_mode):
        deadline = time.monotonic() + self.mode_timeout
        rate = self.node.create_rate(10)
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.mode == expected_mode:
                return True
            if time.monotonic() >= deadline:
                return False
            rate.sleep()
        return False

    def set_mode(self, mode):
        """切换飞行模式。"""
        if not self._ensure_services():
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        ok, resp = self._call_service(self._set_mode_srv, req, self.service_timeout)
        if not ok:
            self.node.get_logger().error("set_mode 服务调用失败: %s" % mode)
            return False
        if not resp.mode_sent:
            self.node.get_logger().error("模式切换请求被拒绝: %s" % mode)
            return False
        if not self._wait_for_mode(mode):
            self.node.get_logger().error(
                "模式未切换成功: expected=%s current=%s"
                % (mode, self.current_state.mode)
            )
            return False
        self.node.get_logger().info("飞行模式已切换为 %s" % mode)
        return True

    # ------------------------------------------------------------------
    # 起飞
    # ------------------------------------------------------------------
    def _request_arm_by_enter(self):
        """满足解锁条件后，等操作者按 Enter 再解锁（房规：脚本自解锁，SITL 无遥控器）。"""
        prompt = "飞控已就绪（GUIDED，未解锁）。按 Enter 执行解锁（Ctrl+C 取消）: "
        try:
            sys.stdout.write(prompt)
            sys.stdout.flush()
            try:
                with open("/dev/tty", "r", encoding="utf-8") as terminal:
                    return terminal.readline() != ""
            except OSError:
                return sys.stdin.readline() != ""
        except (EOFError, KeyboardInterrupt, OSError):
            return False

    def arm(self):
        """按 Enter 授权后执行解锁，并回读确认 armed=True。"""
        if not self._ensure_services():
            return False
        if self.current_state.armed:
            self.node.get_logger().info("飞机已处于 armed=True，跳过重复解锁")
            return True
        req = CommandBool.Request()
        req.value = True
        ok, resp = self._call_service(self._arming_srv, req, self.service_timeout)
        if not ok:
            self.node.get_logger().error("arming 服务调用失败")
            return False
        if not resp.success:
            self.node.get_logger().error("解锁失败: result=%d" % getattr(resp, "result", -1))
            return False
        deadline = time.monotonic() + self.mode_timeout
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.armed:
                self.node.get_logger().info("解锁成功")
                return True
            if time.monotonic() >= deadline:
                break
            time.sleep(0.1)
        self.node.get_logger().error("解锁请求已发送，但状态未更新为 armed")
        return False

    def _confirm_prearmed(self):
        """确认已解锁（本脚本会按 Enter 自解锁，见 run()）。"""
        if not self.current_state.connected:
            self.node.get_logger().error("飞控连接已断开，禁止起飞")
            return False
        if not self.current_state.armed:
            self.node.get_logger().error("飞机尚未解锁，禁止起飞")
            return False
        if not self._navigation_data_is_fresh():
            self.node.get_logger().error("本地位置或航向数据已超时，禁止起飞")
            return False
        self.node.get_logger().info("起飞前确认完成：connected=True, armed=True")
        return True

    def _capture_ground_altitude_reference(self):
        """解锁后在地面记录 rel_alt 地面零点，换算实际起飞目标高度。

        门控与起飞命令必须同参考系：rel_alt 相对 Home，MAV_CMD_NAV_TAKEOFF 的
        altitude 也相对 Home。命令高度取 ground_rel_alt + 计划爬升，门控同样比它，
        悬停航点再换算回本地 ENU z（见 run() 的桥接）。
        """
        self.node.get_logger().info("记录起飞地面高度基准（rel_alt）...")
        deadline = time.monotonic() + self.data_timeout
        samples = []
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.connected and self._relative_altitude_received:
                samples.append(self.relative_altitude)
                if len(samples) >= 3:
                    break
            if time.monotonic() >= deadline:
                break
            time.sleep(0.5)
        if not samples:
            self.node.get_logger().error(
                "未收到 /mavros/global_position/rel_alt，无法建立地面高度基准")
            return False
        samples.sort()
        self.ground_rel_alt = samples[len(samples) // 2]
        self.target_rel_alt = self.ground_rel_alt + self.takeoff_altitude
        self.node.get_logger().info(
            "地面高度基准: ground_rel_alt=%.2f m, 计划爬升=%.2f m, "
            "target_rel_alt=%.2f m（pose.z=%.2f m 仅作本地坐标，不作高度判据）"
            % (self.ground_rel_alt, self.takeoff_altitude, self.target_rel_alt,
               self.current_pose.pose.position.z))
        return True

    def takeoff(self):
        """发送一次 MAV_CMD_NAV_TAKEOFF；调用前再次确认 armed=True。"""
        if not self._confirm_prearmed():
            return False
        if self.target_rel_alt is None:
            self.node.get_logger().error("尚未记录地面高度基准，拒绝发送起飞命令")
            return False
        req = CommandTOL.Request()
        req.altitude = float(self.target_rel_alt)
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0
        ok, resp = self._call_service(self._takeoff_srv, req, self.service_timeout)
        if not ok:
            self.node.get_logger().error("takeoff 服务调用失败")
            return False
        if not resp.success:
            self.node.get_logger().error(
                "飞控拒绝起飞命令: result=%d" % getattr(resp, "result", -1)
            )
            return False
        self.node.get_logger().info(
            "起飞命令已发送，目标相对 Home 高度 %.2f m（离启动地面上升 %.2f m）"
            % (self.target_rel_alt, self.takeoff_altitude)
        )
        return True

    def _wait_until_takeoff_altitude(self):
        """用相对起飞点高度（rel_alt + 地面基准）判断起飞是否到达目标。"""
        accepted_altitude = self.target_rel_alt - self.takeoff_tolerance
        deadline = time.monotonic() + self.takeoff_timeout
        rate = self.node.create_rate(10)
        while rclpy.ok() and self.node.context.ok():
            if not self.current_state.connected or not self.current_state.armed:
                self.node.get_logger().error("爬升过程中飞控断开或飞机已上锁")
                return False
            if self.relative_altitude >= accepted_altitude:
                self.node.get_logger().info(
                    "已到达起飞高度（离地 %.2f / %.2f m）: rel_alt=%.2f m, pose.z=%.2f m"
                    % (self.relative_altitude - self.ground_rel_alt,
                       self.takeoff_altitude, self.relative_altitude,
                       self.current_pose.pose.position.z)
                )
                return True
            if time.monotonic() >= deadline:
                self.node.get_logger().error(
                    "等待起飞高度超时: rel_alt %.2f / %.2f m"
                    % (self.relative_altitude, self.target_rel_alt)
                )
                return False
            throttle_log(
                self.node, 2.0,
                "爬升中: rel_alt=%.2f m（离地 %.2f） / 目标 %.2f m",
                self.relative_altitude,
                self.relative_altitude - self.ground_rel_alt,
                self.takeoff_altitude,
            )
            rate.sleep()
        return False

    # ------------------------------------------------------------------
    # setpoint 发布 / 航点保持与飞行
    # ------------------------------------------------------------------
    @staticmethod
    def _make_enu_target(east, north, up, orientation):
        target = PoseStamped()
        target.header.frame_id = "map"
        target.pose.position.x = east
        target.pose.position.y = north
        target.pose.position.z = up
        target.pose.orientation = copy.deepcopy(orientation)
        return target

    def _publish_target(self, target):
        target.header.stamp = self.node.get_clock().now().to_msg()
        self.setpoint_pub.publish(target)

    def hold_target(self, target, duration, description):
        self.node.get_logger().info("%s，持续 %.1f s" % (description, duration))
        deadline = time.monotonic() + duration
        rate = self.node.create_rate(self.setpoint_rate)
        while rclpy.ok() and self.node.context.ok() and time.monotonic() < deadline:
            if not self.current_state.connected or not self.current_state.armed:
                self.node.get_logger().error("保持航点时飞控断开或飞机已上锁")
                return False
            self._publish_target(target)
            rate.sleep()
        return rclpy.ok() and self.node.context.ok()

    def fly_to_target(self, target):
        deadline = time.monotonic() + self.waypoint_timeout
        rate = self.node.create_rate(self.setpoint_rate)
        while rclpy.ok() and self.node.context.ok():
            if not self.current_state.connected or not self.current_state.armed:
                self.node.get_logger().error("飞向航点时飞控断开或飞机已上锁")
                return False

            self._publish_target(target)
            current = self.current_pose.pose.position
            dx = target.pose.position.x - current.x
            dy = target.pose.position.y - current.y
            dz = target.pose.position.z - current.z
            horizontal_error = math.hypot(dx, dy)
            vertical_error = abs(dz)
            if (
                horizontal_error <= self.waypoint_tolerance
                and vertical_error <= self.vertical_tolerance
            ):
                self.node.get_logger().info(
                    "到达前方航点: horizontal_error=%.3f m vertical_error=%.3f m"
                    % (horizontal_error, vertical_error)
                )
                return True
            if time.monotonic() >= deadline:
                self.node.get_logger().error(
                    "飞往航点超时: horizontal_error=%.3f m vertical_error=%.3f m"
                    % (horizontal_error, vertical_error)
                )
                return False
            throttle_log(
                self.node, 1.0,
                "飞往前方航点: horizontal_error=%.3f m vertical_error=%.3f m",
                horizontal_error, vertical_error,
            )
            rate.sleep()
        return False

    def _calculate_forward_waypoint(self, target_up):
        """根据当前 ENU 位置和罗盘航向，在 NED 中计算前方航点。"""
        current = self.current_pose.pose.position

        # MAVROS ENU -> 飞控 NED。
        current_north = current.y
        current_east = current.x
        current_down = -current.z

        heading_deg = self.heading_deg % 360.0
        heading_rad = math.radians(heading_deg)
        target_north = (
            current_north + self.forward_distance * math.cos(heading_rad)
        )
        target_east = (
            current_east + self.forward_distance * math.sin(heading_rad)
        )
        target_down = -target_up

        self.node.get_logger().info(
            "当前 NED=(N %.3f, E %.3f, D %.3f), heading=%.2f deg"
            % (current_north, current_east, current_down, heading_deg)
        )
        self.node.get_logger().info(
            "目标 NED=(N %.3f, E %.3f, D %.3f), 前进距离=%.2f m"
            % (target_north, target_east, target_down, self.forward_distance)
        )

        # 飞控 NED -> MAVROS/ROS ENU：E->x, N->y, -D->z。
        return self._make_enu_target(
            east=target_east,
            north=target_north,
            up=-target_down,
            orientation=self.current_pose.pose.orientation,
        )

    # ------------------------------------------------------------------
    # 降落 / 异常处置
    # ------------------------------------------------------------------
    def land(self):
        if not self.set_mode("LAND"):
            return False
        self.node.get_logger().info("LAND 已生效，等待飞控自动落地并上锁 ...")
        deadline = time.monotonic() + self.landing_timeout
        rate = self.node.create_rate(5)
        while rclpy.ok() and self.node.context.ok():
            if not self.current_state.armed:
                self.node.get_logger().info("降落完成：飞控已自动上锁")
                return True
            if time.monotonic() >= deadline:
                self.node.get_logger().error(
                    "等待自动落地超时（%.1fs），当前模式=%s armed=%s"
                    % (
                        self.landing_timeout,
                        self.current_state.mode,
                        self.current_state.armed,
                    )
                )
                return False
            rate.sleep()
        return False

    def _abort_to_land(self):
        """起飞后的流程失败时请求 LAND；不发送解锁或上锁命令。"""
        if self.current_state.connected and self.current_state.armed:
            self.node.get_logger().error("飞行流程异常，尝试切换 LAND")
            self.set_mode("LAND")

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def run(self):
        if not self._ensure_mavros():
            return 1
        if not self._wait_for_connection():
            return 1
        if not self._wait_for_navigation_data():
            return 1
        if not self._ensure_services():
            return 1

        if not self.set_mode("GUIDED"):
            return 1
        if not self._request_arm_by_enter():
            self.node.get_logger().warn("操作者未按 Enter，取消；未解锁、未起飞")
            return 2
        if not self.arm():
            return 1
        if not self._confirm_prearmed():
            return 1
        if not self._capture_ground_altitude_reference():
            self._abort_to_land()
            return 1
        if not self.takeoff():
            return 1

        if not self._wait_until_takeoff_altitude():
            self._abort_to_land()
            return 1

        reached_pose = copy.deepcopy(self.current_pose)
        # rel_alt 与 local_position 的零点可能不同。用两者当前差值把
        # "相对起飞点 target_rel_alt"换算到本地 ENU z，而不是假定
        # local z == 目标高度（本地 z 只是内部 setpoint 坐标）。
        target_up = (
            reached_pose.pose.position.z
            + self.target_rel_alt
            - self.relative_altitude
        )
        takeoff_hold_target = self._make_enu_target(
            east=reached_pose.pose.position.x,
            north=reached_pose.pose.position.y,
            up=target_up,
            orientation=reached_pose.pose.orientation,
        )
        forward_target = self._calculate_forward_waypoint(target_up)
        if not self.hold_target(
            takeoff_hold_target,
            self.takeoff_hover_time,
            "已到达 %.2f m 起飞航点，原地悬停" % self.takeoff_altitude,
        ):
            self._abort_to_land()
            return 1

        if not self._navigation_data_is_fresh():
            self.node.get_logger().error("计算前方航点前导航数据已超时")
            self._abort_to_land()
            return 1
        if not self.fly_to_target(forward_target):
            self._abort_to_land()
            return 1
        if not self.hold_target(
            forward_target,
            self.waypoint_hover_time,
            "已到达前方 %.2f m 航点，悬停" % self.forward_distance,
        ):
            self._abort_to_land()
            return 1
        if not self.land():
            return 1
        return 0


def main():
    rclpy.init()
    node = rclpy.create_node("one_key_takeoff_forward_land")

    # 后台 spin：订阅回调（state/pose/heading/rel_alt）必须由 executor 驱动，
    # ROS2 的 rate.sleep() 不会处理回调。
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    exit_code = 2
    try:
        try:
            c1_node = TakeoffForwardLandNode(node)
        except ValueError as exc:
            node.get_logger().fatal("参数错误: %s" % exc)
            exit_code = 2
        else:
            c1_node._executor = executor  # 同一执行器，避免自建 executor 冲突
            exit_code = c1_node.run()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
