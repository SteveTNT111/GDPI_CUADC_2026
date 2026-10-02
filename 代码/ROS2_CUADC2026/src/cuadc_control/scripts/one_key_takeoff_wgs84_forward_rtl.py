#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
one_key_takeoff_wgs84_forward_rtl.py — WGS84 前飞 + RTL 返航（ROS2/Humble 版）
================================================================================
由 ROS1 版 `代码/比赛main工作版本/src/cuadc_control/scripts/
one_key_takeoff_wgs84_forward_rtl.py`（1195 行）逐行翻译而来。

流程：GUIDED -> ARM（飞手终端按 Enter 授权）-> 起飞 2 m（相对起飞点）
-> 沿启动航向飞行 1 m 对应的 WGS84 航点 -> 悬停 5 s -> RTL
-> 必要时在 Home 切 LAND。
（默认起飞 2.0 m、前飞 1.0 m，与 one_key_takeoff_forward_land.py 对齐；
  可用 --ros-args -p takeoff_altitude:=X -p forward_distance:=Y 覆盖。）

安全约束（2026-09-28 修订：默认"按 Enter 授权"，免遥控器）：
  - 程序只允许在飞机确定停在地面时启动；启动后冻结起始航向、位置和目标航点。
  - 解锁后、起飞前重新记录地面的相对 Home 高度，允许该高度不等于 0。
  - 切换模式或解锁前，必须由飞手在终端按 Enter（或输入 authorization_phrase 短语）授权。
  - RTL 未自动降落时，只有确认飞机已回到起飞点附近才请求 LAND。
  - 飞行阶段发生异常时优先请求 RTL；RTL 请求失败才请求 LAND。

坐标约定（与原版一致）：
  - /mavros/local_position/pose 为 ROS ENU，打印时转换为 NED。
  - 航向来自 /mavros/global_position/compass_hdg，0 度为北、顺时针为正。
  - 经纬度目标使用 WGS84 椭球局部曲率计算。
  - 全局 setpoint 使用 MAV_FRAME_GLOBAL_RELATIVE_ALT_INT（FRAME_GLOBAL_REL_ALT），
    高度相对 Home，发布到 /mavros/setpoint_raw/global。

ROS1 -> ROS2 翻译要点：
  - `rospy.init_node` -> `rclpy.init() + create_node`；类构造改为显式传入 rclpy Node。
  - 私有参数 `rospy.get_param("~x", d)` -> `declare_parameter/get_parameter`（去 `~`，
    运行时 `--ros-args -p x:=v` 覆盖）。
  - `rospy.Subscriber(queue_size=)` -> `create_subscription`（导航话题 BEST_EFFORT QoS；
    /mavros/state RELIABLE，BEST_EFFORT 订阅兼容）；全局 fix 同 QoS。
  - `rospy.Publisher(queue_size=20)` -> `create_publisher(depth=20, RELIABLE)`。
  - `rospy.ServiceProxy 同步调用` -> `create_client + call_async` + 轮询 future
    （节点由后台 MultiThreadedExecutor spin，避免与 spin() 冲突丢回调）。
  - `rospy.Rate` -> `node.create_rate`；`rospy.is_shutdown()` -> `rclpy.ok() and
    node.context.ok()`；`rospy.Time.now()` -> `node.get_clock().now().to_msg()`；
    `rospy.log*` -> `node.get_logger().*`（节流日志用本地 `throttle_log`）。
  - ROS1 自动启动 roscore 的逻辑删除（ROS2 无 master）；MAVROS 未运行且
    auto_start_mavros=true 时自动 `ros2 launch mavros apm.launch fcu_url:=...`
    （ROS2 图发现异步，先给 mavros_discovery_grace 宽限窗口，防重复启动——C1 教训）。
  - `rospy.get_published_topics()` 检查 MAVROS -> `node.count_publishers("/mavros/state") > 0`。
  - 飞手授权：默认按 Enter 授权（authorization_phrase 留空）；如显式设置
    authorization_phrase 则需输入该短语。优先读 /dev/tty（真终端），失败回退 sys.stdin
    （验证脚本用 stdin=PIPE 喂 "\\n" 或短语；start_new_session=True 保证 /dev/tty 打不开）。

运行方式（终端按 Enter 授权；如设 authorization_phrase 则输入该短语）：
  # 默认：起飞 2.0 m、前飞 1.0 m
  ros2 run cuadc_control one_key_takeoff_wgs84_forward_rtl.py
  # 覆盖默认值示例：
  ros2 run cuadc_control one_key_takeoff_wgs84_forward_rtl.py --ros-args \
      -p forward_distance:=5.0 -p takeoff_altitude:=3.0
"""

import math
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, ParamGet, SetMode
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
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


def throttle_log(node, interval_s, fmt, *args, level="info"):
    """简单节流日志：等价 ROS1 的 rospy.log{info,warn}_throttle(...)。"""
    name = "_throttle_last_" + str(abs(hash(fmt)))
    last = getattr(node, name, 0.0)
    if now_s() - last >= interval_s:
        setattr(node, name, now_s())
        msg = fmt % args if args else fmt
        if level == "warn":
            node.get_logger().warn(msg)
        else:
            node.get_logger().info(msg)


class Wgs84ForwardRtlNode:
    """执行 WGS84 前向航点和 RTL 任务。"""

    WGS84_A = 6378137.0
    WGS84_F = 1.0 / 298.257223563

    def __init__(self, node):
        self.node = node

        self.takeoff_altitude = float(node_param(node, "takeoff_altitude", 2.0))
        self.forward_distance = float(node_param(node, "forward_distance", 1.0))
        self.waypoint_hover_time = float(
            node_param(node, "waypoint_hover_time", 5.0)
        )
        # 2026-09-28：默认改为"按 Enter 授权"（authorization_phrase 留空）；
        # 若显式传入非空短语，则仍要求输入该短语。
        self.authorization_phrase = str(
            node_param(node, "authorization_phrase", "")
        ).strip()

        self.auto_start_mavros = bool(
            node_param(node, "auto_start_mavros", False)
        )
        self.mavros_fcu_url = node_param(
            node, "mavros_fcu_url", "/dev/ttyACM0:115200"
        )
        # ROS2 图发现异步：新节点 count_publishers 可能短暂返回 0。给宽限窗口，
        # 避免把"已运行的 MAVROS"误判为未运行而重复启动（C1 教训）。
        self.mavros_discovery_grace = float(
            node_param(node, "mavros_discovery_grace", 3.0)
        )
        self.connection_timeout = float(
            node_param(node, "connection_timeout", 30.0)
        )
        self.connection_loss_timeout = max(
            1.0, float(node_param(node, "connection_loss_timeout", 10.0))
        )
        self.service_timeout = float(node_param(node, "service_timeout", 10.0))
        self.data_timeout = float(node_param(node, "data_timeout", 30.0))
        self.data_max_age = float(node_param(node, "data_max_age", 2.0))
        self.navigation_loss_timeout = max(
            self.data_max_age,
            float(node_param(node, "navigation_loss_timeout", 5.0)),
        )
        self.mode_timeout = float(node_param(node, "mode_timeout", 6.0))
        self.arm_timeout = float(node_param(node, "arm_timeout", 6.0))
        self.takeoff_command_attempts = max(
            1, int(node_param(node, "takeoff_command_attempts", 2))
        )
        self.takeoff_command_retry_delay = max(
            0.0, float(node_param(node, "takeoff_command_retry_delay", 1.0))
        )
        self.takeoff_ack_timeout_threshold = max(
            1.0, float(node_param(node, "takeoff_ack_timeout_threshold", 4.0))
        )
        self.takeoff_started_threshold = max(
            0.10, float(node_param(node, "takeoff_started_threshold", 0.30))
        )
        self.takeoff_timeout = float(node_param(node, "takeoff_timeout", 40.0))
        self.waypoint_timeout = float(node_param(node, "waypoint_timeout", 90.0))
        self.rtl_timeout = float(node_param(node, "rtl_timeout", 240.0))
        self.rtl_auto_land_timeout = float(
            node_param(node, "rtl_auto_land_timeout", 90.0)
        )
        self.landing_timeout = float(node_param(node, "landing_timeout", 120.0))

        self.takeoff_tolerance = max(
            0.05, float(node_param(node, "takeoff_tolerance", 0.20))
        )
        # 到点水平容差：默认前飞仅 1 m，容差若 ≥1 m 会"未移动即判到达"，
        # 故从 1.0 收小到 0.30（C2 实测到达误差 0.26 m，留余量）。
        self.waypoint_tolerance = max(
            0.20, float(node_param(node, "waypoint_tolerance", 0.30))
        )
        self.vertical_tolerance = max(
            0.10, float(node_param(node, "vertical_tolerance", 0.40))
        )
        self.arrival_hold_time = max(
            0.0, float(node_param(node, "arrival_hold_time", 2.0))
        )
        self.rtl_home_tolerance = max(
            0.50, float(node_param(node, "rtl_home_tolerance", 2.0))
        )
        self.rtl_home_hold_time = max(
            0.0, float(node_param(node, "rtl_home_hold_time", 3.0))
        )
        self.authorization_max_drift = max(
            0.20, float(node_param(node, "authorization_max_drift", 2.0))
        )
        self.post_arm_altitude_settle_time = max(
            0.0, float(node_param(node, "post_arm_altitude_settle_time", 2.0))
        )
        self.setpoint_rate = max(
            2.0, float(node_param(node, "setpoint_rate", 10.0))
        )

        if self.takeoff_altitude <= 0.0:
            raise ValueError("takeoff_altitude 必须大于 0")
        if self.forward_distance <= 0.0:
            raise ValueError("forward_distance 必须大于 0")
        # authorization_phrase 允许为空：为空时按 Enter 即授权（2026-09-28）。

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_fix = NavSatFix()
        self.heading_deg = None
        self.relative_altitude = None

        self._state_received = False
        self._pose_received = False
        self._fix_received = False
        self._heading_received = False
        self._relative_altitude_received = False
        self._pose_received_at = 0.0
        self._fix_received_at = 0.0
        self._heading_received_at = 0.0
        self._relative_altitude_received_at = 0.0

        self.start_north = None
        self.start_east = None
        self.start_down = None
        self.start_latitude = None
        self.start_longitude = None
        self.start_wgs84_altitude = None
        self.start_heading_deg = None
        self.start_relative_altitude = None
        self.ground_relative_altitude = None
        self.target_relative_altitude = None
        self.target_latitude = None
        self.target_longitude = None

        self._set_mode_srv = None
        self._arming_srv = None
        self._takeoff_srv = None
        self._param_get_srv = None

        # MAVROS 话题 BEST_EFFORT 订阅对 RELIABLE/BEST_EFFORT 发布均兼容。
        nav_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_sub = self.node.create_subscription(
            State, "/mavros/state", self._state_cb, nav_qos
        )
        self.pose_sub = self.node.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, nav_qos
        )
        self.fix_sub = self.node.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._fix_cb, nav_qos
        )
        self.heading_sub = self.node.create_subscription(
            Float64, "/mavros/global_position/compass_hdg",
            self._heading_cb, nav_qos,
        )
        self.relative_altitude_sub = self.node.create_subscription(
            Float64, "/mavros/global_position/rel_alt",
            self._relative_altitude_cb, nav_qos,
        )
        # 全局 setpoint 流：RELIABLE 发布对 BEST_EFFORT/RELIABLE 订阅均兼容。
        setpoint_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.global_setpoint_pub = self.node.create_publisher(
            GlobalPositionTarget, "/mavros/setpoint_raw/global", setpoint_qos
        )

        # 可选：外部 MultiThreadedExecutor（后台 spin 时使用），_call_service
        # 用它区分轮询与 spin_until_future_complete。
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
        self._pose_received_at = now_s()

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
            self.current_fix = msg
            self._fix_received = True
            self._fix_received_at = now_s()

    def _heading_cb(self, msg):
        if math.isfinite(msg.data):
            self.heading_deg = msg.data % 360.0
            self._heading_received = True
            self._heading_received_at = now_s()

    def _relative_altitude_cb(self, msg):
        if math.isfinite(msg.data):
            self.relative_altitude = msg.data
            self._relative_altitude_received = True
            self._relative_altitude_received_at = now_s()

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
        deadline = now_s() + timeout
        while rclpy.ok() and self.node.context.ok():
            if self._mavros_is_alive():
                return True
            if now_s() >= deadline:
                return False
            time.sleep(1.0)
        return False

    def _ensure_mavros(self):
        """确保 MAVROS 在运行（先给图发现宽限窗口，见 __init__ 中说明）。"""
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
        self.node.get_logger().info("等待飞控连接 ...")
        deadline = now_s() + self.connection_timeout
        rate = self.node.create_rate(5)
        while rclpy.ok() and self.node.context.ok():
            if self._state_received and self.current_state.connected:
                self.node.get_logger().info("飞控已连接")
                return True
            if now_s() >= deadline:
                self.node.get_logger().error(
                    "等待飞控连接超时（%.1fs）" % self.connection_timeout
                )
                return False
            rate.sleep()
        return False

    def _wait_for_navigation_data(self):
        self.node.get_logger().info("等待 NED、WGS84、航向和相对高度数据 ...")
        deadline = now_s() + self.data_timeout
        rate = self.node.create_rate(10)
        while rclpy.ok() and self.node.context.ok():
            if self._navigation_data_is_fresh():
                self.node.get_logger().info("NED、WGS84、航向和相对高度数据已刷新")
                return True
            if now_s() >= deadline:
                self.node.get_logger().error(
                    "等待导航数据超时: pose=%s fix=%s heading=%s rel_alt=%s"
                    % (
                        self._pose_received,
                        self._fix_received,
                        self._heading_received,
                        self._relative_altitude_received,
                    )
                )
                return False
            rate.sleep()
        return False

    def _navigation_data_is_fresh(self):
        now = now_s()
        return (
            self._pose_received
            and self._fix_received
            and self._heading_received
            and self._relative_altitude_received
            and now - self._pose_received_at <= self.data_max_age
            and now - self._fix_received_at <= self.data_max_age
            and now - self._heading_received_at <= self.data_max_age
            and now - self._relative_altitude_received_at <= self.data_max_age
        )

    def _relative_altitude_is_fresh(self):
        return (
            self._relative_altitude_received
            and now_s() - self._relative_altitude_received_at
            <= self.data_max_age
        )

    # ------------------------------------------------------------------
    # 服务
    # ------------------------------------------------------------------
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
                # 后台 spin 线程统一处理回调；这里只轮询 future，避免同一
                # executor 上再 spin_until_future_complete 冲突丢回调。
                deadline = now_s() + timeout_s
                while not future.done():
                    if now_s() >= deadline:
                        break
                    time.sleep(0.02)
            else:
                rclpy.spin_until_future_complete(
                    self.node, future, timeout_sec=timeout_s
                )
        except Exception as exc:
            self.node.get_logger().error("service call failed: %s" % exc)
            return (False, None)
        if future.result() is None:
            return (False, None)
        return (True, future.result())

    def _ensure_services(self):
        if self._set_mode_srv is not None:
            return True
        try:
            self._set_mode_srv = self.node.create_client(SetMode, "/mavros/set_mode")
            self._arming_srv = self.node.create_client(CommandBool, "/mavros/cmd/arming")
            self._takeoff_srv = self.node.create_client(CommandTOL, "/mavros/cmd/takeoff")
            if not (
                self._wait_service(self._set_mode_srv, self.service_timeout)
                and self._wait_service(self._arming_srv, self.service_timeout)
                and self._wait_service(self._takeoff_srv, self.service_timeout)
            ):
                self.node.get_logger().error("等待 MAVROS 服务失败: 基础服务不可用")
                return False
            try:
                param_get = self.node.create_client(ParamGet, "/mavros/param/get")
                if self._wait_service(param_get, self.service_timeout):
                    self._param_get_srv = param_get
                else:
                    self.node.get_logger().warn(
                        "参数服务不可用，将通过 RTL 状态超时判断是否需要 LAND"
                    )
            except Exception:
                self.node.get_logger().warn(
                    "参数服务不可用，将通过 RTL 状态超时判断是否需要 LAND"
                )
            return True
        except Exception as exc:
            self.node.get_logger().error("等待 MAVROS 服务失败: %s" % exc)
            return False

    # ------------------------------------------------------------------
    # WGS84 解算（纯数学，与原版逐行一致）
    # ------------------------------------------------------------------
    @classmethod
    def _destination_wgs84(cls, latitude, longitude, distance, heading_deg):
        """用 WGS84 椭球在起点处的曲率半径解算短距离目标。"""
        lat_rad = math.radians(latitude)
        heading_rad = math.radians(heading_deg)
        flattening_term = cls.WGS84_F * (2.0 - cls.WGS84_F)
        sin_lat = math.sin(lat_rad)
        denominator = math.sqrt(1.0 - flattening_term * sin_lat * sin_lat)
        prime_vertical_radius = cls.WGS84_A / denominator
        meridian_radius = (
            cls.WGS84_A * (1.0 - flattening_term) / denominator ** 3
        )

        north = distance * math.cos(heading_rad)
        east = distance * math.sin(heading_rad)
        target_lat = latitude + math.degrees(north / meridian_radius)
        cos_lat = math.cos(lat_rad)
        if abs(cos_lat) < 1e-12:
            raise ValueError("当前位置过于接近极点，无法解算经度目标")
        target_lon = longitude + math.degrees(
            east / (prime_vertical_radius * cos_lat)
        )
        target_lon = (target_lon + 180.0) % 360.0 - 180.0
        return target_lat, target_lon

    @classmethod
    def _horizontal_wgs84_distance(cls, lat1, lon1, lat2, lon2):
        """用 WGS84 局部曲率计算两相近经纬度点的水平距离。"""
        mean_lat = math.radians((lat1 + lat2) * 0.5)
        flattening_term = cls.WGS84_F * (2.0 - cls.WGS84_F)
        sin_lat = math.sin(mean_lat)
        denominator = math.sqrt(1.0 - flattening_term * sin_lat * sin_lat)
        prime_vertical_radius = cls.WGS84_A / denominator
        meridian_radius = (
            cls.WGS84_A * (1.0 - flattening_term) / denominator ** 3
        )
        dlat = math.radians(lat2 - lat1)
        dlon_deg = (lon2 - lon1 + 180.0) % 360.0 - 180.0
        dlon = math.radians(dlon_deg)
        north = dlat * meridian_radius
        east = dlon * prime_vertical_radius * math.cos(mean_lat)
        return math.hypot(north, east)

    # ------------------------------------------------------------------
    # 启动冻结 / 授权
    # ------------------------------------------------------------------
    def _freeze_start_and_target(self):
        pose = self.current_pose.pose.position
        self.start_north = pose.y
        self.start_east = pose.x
        self.start_down = -pose.z
        self.start_latitude = self.current_fix.latitude
        self.start_longitude = self.current_fix.longitude
        self.start_wgs84_altitude = self.current_fix.altitude
        self.start_heading_deg = self.heading_deg
        self.start_relative_altitude = self.relative_altitude
        self.target_latitude, self.target_longitude = self._destination_wgs84(
            self.start_latitude,
            self.start_longitude,
            self.forward_distance,
            self.start_heading_deg,
        )

    def _print_authorization_summary(self):
        lines = [
            "",
            "================ 飞行授权确认 ================",
            "当前 NED     : N={:.3f} m, E={:.3f} m, D={:.3f} m".format(
                self.start_north, self.start_east, self.start_down
            ),
            "当前 WGS84   : lat={:.9f}, lon={:.9f}, alt={:.3f} m".format(
                self.start_latitude,
                self.start_longitude,
                self.start_wgs84_altitude,
            ),
            "当前 rel_alt : {:.3f} m（仅记录，不用于判断飞机是否在地面）".format(
                self.start_relative_altitude
            ),
            "启动时航向   : {:.2f} deg（0=北，顺时针）".format(
                self.start_heading_deg
            ),
            "目标 WGS84   : lat={:.9f}, lon={:.9f}".format(
                self.target_latitude,
                self.target_longitude,
            ),
            "目标高度     : 起飞点以上 {:.2f} m（解锁后以地面 rel_alt 标定，"
            "命令/门控/setpoint 同源）".format(self.takeoff_altitude),
            "飞行计划     : 自动解锁 -> 起飞 {:.2f} m -> 前飞 {:.2f} m".format(
                self.takeoff_altitude, self.forward_distance
            ),
            "               -> 悬停 {:.1f} s -> RTL -> 必要时 Home 点 LAND".format(
                self.waypoint_hover_time
            ),
            "注意：解锁后将重新记录地面 rel_alt，目标命令高度=地面 rel_alt+{:.2f} m。".format(
                self.takeoff_altitude
            ),
            "      上面的 WGS84 alt 为参考值；请确保执行脚本时飞机确实在地面。",
            "================================================",
        ]
        print("\n".join(lines), flush=True)

    def _request_authorization(self):
        if self.authorization_phrase:
            prompt = (
                "确认空域安全后输入 {!r} 授权飞行（英文字母不区分大小写，其他输入取消）: "
            ).format(self.authorization_phrase)
        else:
            prompt = "确认空域和载荷安全后按 Enter 授权飞行（Ctrl+C 取消）: "
        try:
            sys.stdout.write(prompt)
            sys.stdout.flush()
            try:
                with open("/dev/tty", "r", encoding="utf-8") as terminal:
                    response = terminal.readline()
            except OSError:
                response = sys.stdin.readline()
        except (EOFError, KeyboardInterrupt):
            print("")
            return False
        entered_phrase = response.strip()
        if self.authorization_phrase:
            if entered_phrase.casefold() != self.authorization_phrase.strip().casefold():
                self.node.get_logger().warn(
                    "飞手未授权：输入=%r，期望=%r；任务取消，未发送模式、解锁或起飞命令"
                    % (entered_phrase, self.authorization_phrase)
                )
                return False
        elif response == "":
            # Enter-only 模式：readline 返回空串表示 EOF/无输入
            self.node.get_logger().warn(
                "未检测到输入（EOF），任务取消，未发送模式、解锁或起飞命令")
            return False
        self.node.get_logger().info("飞手已授权，开始执行飞行任务")
        return True

    def _validate_after_authorization(self):
        if not self.current_state.connected:
            self.node.get_logger().error("授权后飞控连接已断开")
            return False
        if not self._navigation_data_is_fresh():
            self.node.get_logger().warn("授权等待期间导航数据已变旧，等待下一批数据刷新 ...")
            if not self._wait_for_navigation_data():
                self.node.get_logger().error("授权后无法取得最新导航数据")
                return False
        drift = self._horizontal_wgs84_distance(
            self.start_latitude,
            self.start_longitude,
            self.current_fix.latitude,
            self.current_fix.longitude,
        )
        if drift > self.authorization_max_drift:
            self.node.get_logger().error(
                "授权期间飞机位置漂移 %.2f m，超过 %.2f m；请重新启动并确认"
                % (drift, self.authorization_max_drift)
            )
            return False
        return True

    # ------------------------------------------------------------------
    # 模式 / 解锁
    # ------------------------------------------------------------------
    def _wait_for_mode(self, expected_mode):
        deadline = now_s() + self.mode_timeout
        rate = self.node.create_rate(10)
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.mode == expected_mode:
                return True
            if now_s() >= deadline:
                return False
            rate.sleep()
        return False

    def set_mode(self, mode):
        if not self._ensure_services():
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        ok, resp = self._call_service(self._set_mode_srv, req, self.service_timeout)
        if not ok:
            self.node.get_logger().error("set_mode 服务调用失败: %s" % mode)
            return False
        if not resp.mode_sent or not self._wait_for_mode(mode):
            self.node.get_logger().error(
                "模式切换失败: expected=%s current=%s" % (mode, self.current_state.mode)
            )
            return False
        self.node.get_logger().info("飞行模式已切换为 %s" % mode)
        return True

    def arm(self):
        if self.current_state.armed:
            self.node.get_logger().warn("飞机在授权前已处于 armed=True，跳过重复解锁")
            return True
        req = CommandBool.Request()
        req.value = True
        ok, resp = self._call_service(self._arming_srv, req, self.service_timeout)
        if not ok:
            self.node.get_logger().error("arming 服务调用失败")
            return False
        if not resp.success:
            self.node.get_logger().error(
                "飞控拒绝解锁: result=%d" % getattr(resp, "result", -1)
            )
            return False
        deadline = now_s() + self.arm_timeout
        rate = self.node.create_rate(10)
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.armed:
                self.node.get_logger().info("飞机已解锁")
                return True
            if now_s() >= deadline:
                self.node.get_logger().error("等待 armed=True 超时")
                return False
            rate.sleep()
        return False

    # ------------------------------------------------------------------
    # 地面高度基准 / 起飞
    # ------------------------------------------------------------------
    def _capture_ground_altitude_reference(self):
        """解锁后在地面记录相对 Home 高度，并换算实际起飞目标高度。"""
        self.node.get_logger().info(
            "飞机应保持在地面，等待相对高度数据持续有效 %.1f s ..."
            % self.post_arm_altitude_settle_time
        )
        deadline = now_s() + self.data_timeout
        fresh_since = None
        rate = self.node.create_rate(10)
        while rclpy.ok() and self.node.context.ok():
            if not self.current_state.connected or not self.current_state.armed:
                self.node.get_logger().error("记录地面高度基准时飞控断开或飞机已上锁")
                return False

            now = now_s()
            if self._relative_altitude_is_fresh():
                if fresh_since is None:
                    fresh_since = now
                if now - fresh_since >= self.post_arm_altitude_settle_time:
                    self.ground_relative_altitude = self.relative_altitude
                    self.target_relative_altitude = (
                        self.ground_relative_altitude + self.takeoff_altitude
                    )
                    self.node.get_logger().info(
                        "地面高度基准已记录: ground_rel_alt=%.2f m, "
                        "计划爬升=%.2f m, target_rel_alt=%.2f m"
                        % (
                            self.ground_relative_altitude,
                            self.takeoff_altitude,
                            self.target_relative_altitude,
                        )
                    )
                    return True
            else:
                fresh_since = None

            if now >= deadline:
                self.node.get_logger().error(
                    "等待最新相对高度数据超时（%.1fs）: received=%s age=%.2fs"
                    % (
                        self.data_timeout,
                        self._relative_altitude_received,
                        now - self._relative_altitude_received_at,
                    )
                )
                return False
            rate.sleep()
        return False

    def takeoff(self):
        if self.target_relative_altitude is None:
            self.node.get_logger().error("尚未记录地面高度基准，拒绝发送起飞命令")
            return False

        result_names = {
            0: "ACCEPTED",
            1: "TEMPORARILY_REJECTED",
            2: "DENIED",
            3: "UNSUPPORTED",
            4: "FAILED",
            5: "IN_PROGRESS",
            6: "CANCELLED",
        }
        for attempt in range(1, self.takeoff_command_attempts + 1):
            if not self.current_state.connected:
                if not self._wait_for_connection_recovery():
                    return False
            if not self.current_state.armed:
                self.node.get_logger().error("发送起飞命令前飞机已上锁")
                return False
            if self.current_state.mode != "GUIDED":
                self.node.get_logger().warn(
                    "发送起飞命令前模式为 %r，重新确认 GUIDED"
                    % self.current_state.mode
                )
                if not self.set_mode("GUIDED"):
                    return False

            self.node.get_logger().info(
                "发送起飞命令（第 %d/%d 次）: target_rel_alt=%.2f m"
                % (attempt, self.takeoff_command_attempts, self.target_relative_altitude)
            )
            call_started_at = now_s()
            service_exception = None
            response = None
            req = CommandTOL.Request()
            req.altitude = float(self.target_relative_altitude)
            req.latitude = 0.0
            req.longitude = 0.0
            req.min_pitch = 0.0
            req.yaw = 0.0
            ok, response = self._call_service(
                self._takeoff_srv, req, self.service_timeout
            )
            if not ok:
                service_exception = "service call failed"
                response = None
            call_duration = now_s() - call_started_at

            if response is not None and response.success:
                self.node.get_logger().info(
                    "起飞命令已确认，目标相对 Home 高度 %.2f m"
                    "（离启动地面上升 %.2f m）"
                    % (self.target_relative_altitude, self.takeoff_altitude)
                )
                return True

            result = getattr(response, "result", -1) if response is not None else -1
            result_name = result_names.get(result, "UNKNOWN")
            transport_failure = (
                service_exception is not None
                or not self.current_state.connected
                or not self.current_state.mode
                or call_duration >= self.takeoff_ack_timeout_threshold
            )
            if not transport_failure:
                self.node.get_logger().error(
                    "飞控明确拒绝起飞命令: result=%d (%s), mode=%s, armed=%s"
                    % (
                        result,
                        result_name,
                        self.current_state.mode,
                        self.current_state.armed,
                    )
                )
                return False

            self.node.get_logger().warn(
                "起飞命令确认异常: result=%d (%s), duration=%.2fs, "
                "connected=%s, mode=%r, exception=%s；命令可能已执行，先检查高度"
                % (
                    result,
                    result_name,
                    call_duration,
                    self.current_state.connected,
                    self.current_state.mode,
                    service_exception,
                )
            )
            if not self.current_state.connected:
                if not self._wait_for_connection_recovery():
                    return False

            altitude_deadline = now_s() + min(5.0, self.data_timeout)
            rate = self.node.create_rate(10)
            while (
                rclpy.ok() and self.node.context.ok()
                and now_s() < altitude_deadline
            ):
                if self._relative_altitude_is_fresh():
                    break
                rate.sleep()
            if self._takeoff_has_started():
                self.node.get_logger().warn(
                    "虽然未收到可靠 ACK，但飞机已上升 %.2f m，按起飞命令已执行继续监控"
                    % (self.relative_altitude - self.ground_relative_altitude)
                )
                return True

            if attempt < self.takeoff_command_attempts:
                self.node.get_logger().warn(
                    "连接已恢复且飞机仍在地面，%.1f s 后重试起飞命令"
                    % self.takeoff_command_retry_delay
                )
                time.sleep(self.takeoff_command_retry_delay)

        self.node.get_logger().error(
            "起飞命令重试 %d 次后仍未确认执行" % self.takeoff_command_attempts
        )
        return False

    def _takeoff_has_started(self):
        return (
            self.ground_relative_altitude is not None
            and self.relative_altitude is not None
            and self._relative_altitude_is_fresh()
            and self.relative_altitude - self.ground_relative_altitude
            >= self.takeoff_started_threshold
        )

    def _wait_until_takeoff_altitude(self):
        accepted_altitude = self.target_relative_altitude - self.takeoff_tolerance
        deadline = now_s() + self.takeoff_timeout
        rate = self.node.create_rate(10)
        while rclpy.ok() and self.node.context.ok():
            if not self.current_state.connected or not self.current_state.armed:
                self.node.get_logger().error("爬升过程中飞控断开或飞机已上锁")
                return False
            if self.relative_altitude >= accepted_altitude:
                self.node.get_logger().info(
                    "已到达起飞高度: rel_alt=%.2f / %.2f m"
                    % (self.relative_altitude, self.target_relative_altitude)
                )
                return True
            if now_s() >= deadline:
                self.node.get_logger().error(
                    "等待起飞高度超时: rel_alt=%.2f / %.2f m"
                    % (self.relative_altitude, self.target_relative_altitude)
                )
                return False
            throttle_log(
                self.node, 2.0,
                "爬升中: rel_alt=%.2f / %.2f m",
                self.relative_altitude, self.target_relative_altitude,
            )
            rate.sleep()
        return False

    # ------------------------------------------------------------------
    # 全局 setpoint / 飞往航点 / 悬停
    # ------------------------------------------------------------------
    def _make_global_target(self):
        target = GlobalPositionTarget()
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = (
            GlobalPositionTarget.IGNORE_VX
            | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ
            | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY
            | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW
            | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        target.latitude = self.target_latitude
        target.longitude = self.target_longitude
        target.altitude = self.target_relative_altitude
        return target

    def _publish_global_target(self, target):
        target.header.stamp = self.node.get_clock().now().to_msg()
        self.global_setpoint_pub.publish(target)

    def fly_to_target(self, target):
        self.node.get_logger().info(
            "发送 WGS84 全局位置目标: lat=%.9f lon=%.9f rel_alt=%.2f m"
            % (target.latitude, target.longitude, target.altitude)
        )
        deadline = now_s() + self.waypoint_timeout
        inside_since = None
        navigation_stale_since = None
        connection_lost_since = None
        rate = self.node.create_rate(self.setpoint_rate)
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.connected and not self.current_state.armed:
                self.node.get_logger().error("飞往航点时飞机已上锁")
                return False
            self._publish_global_target(target)

            if not self.current_state.connected:
                now = now_s()
                if connection_lost_since is None:
                    connection_lost_since = now
                    self.node.get_logger().warn(
                        "飞往航点时飞控连接中断，继续发送 setpoint 并等待恢复"
                    )
                lost_duration = now - connection_lost_since
                if lost_duration >= self.connection_loss_timeout:
                    self.node.get_logger().error(
                        "飞往航点时飞控连续断开 %.1f s，超过允许的 %.1f s"
                        % (lost_duration, self.connection_loss_timeout)
                    )
                    return False
                rate.sleep()
                continue
            if connection_lost_since is not None:
                self.node.get_logger().info("飞控连接已恢复，继续飞往 WGS84 航点")
                connection_lost_since = None

            if not self._navigation_data_is_fresh():
                now = now_s()
                if navigation_stale_since is None:
                    navigation_stale_since = now
                stale_duration = now - navigation_stale_since
                if stale_duration >= self.navigation_loss_timeout:
                    self.node.get_logger().error(
                        "飞往航点时导航数据连续中断 %.1f s，超过允许的 %.1f s"
                        % (stale_duration, self.navigation_loss_timeout)
                    )
                    return False
                throttle_log(
                    self.node, 1.0,
                    "飞往航点时等待导航数据刷新: 已中断 %.1f / %.1f s",
                    stale_duration, self.navigation_loss_timeout,
                    level="warn",
                )
                rate.sleep()
                continue
            navigation_stale_since = None

            horizontal_error = self._horizontal_wgs84_distance(
                self.current_fix.latitude,
                self.current_fix.longitude,
                target.latitude,
                target.longitude,
            )
            vertical_error = abs(self.relative_altitude - target.altitude)
            inside = (
                horizontal_error <= self.waypoint_tolerance
                and vertical_error <= self.vertical_tolerance
            )
            if inside:
                if inside_since is None:
                    inside_since = now_s()
                if now_s() - inside_since >= self.arrival_hold_time:
                    self.node.get_logger().info(
                        "已抵达 WGS84 航点: horizontal_error=%.2f m "
                        "vertical_error=%.2f m"
                        % (horizontal_error, vertical_error)
                    )
                    return True
            else:
                inside_since = None

            if now_s() >= deadline:
                self.node.get_logger().error(
                    "飞往 WGS84 航点超时: horizontal_error=%.2f m "
                    "vertical_error=%.2f m"
                    % (horizontal_error, vertical_error)
                )
                return False
            throttle_log(
                self.node, 1.0,
                "飞往 WGS84 航点: horizontal_error=%.2f m vertical_error=%.2f m",
                horizontal_error, vertical_error,
            )
            rate.sleep()
        return False

    def hold_global_target(self, target):
        self.node.get_logger().info(
            "已抵达目标，持续发送 setpoint 悬停 %.1f s" % self.waypoint_hover_time
        )
        held_time = 0.0
        last_loop_at = now_s()
        connection_lost_since = None
        rate = self.node.create_rate(self.setpoint_rate)
        while (
            rclpy.ok() and self.node.context.ok()
            and held_time < self.waypoint_hover_time
        ):
            now = now_s()
            elapsed = max(0.0, now - last_loop_at)
            last_loop_at = now

            if self.current_state.connected and not self.current_state.armed:
                self.node.get_logger().error("航点悬停时飞机已上锁")
                return False
            self._publish_global_target(target)

            if not self.current_state.connected:
                if connection_lost_since is None:
                    connection_lost_since = now
                    self.node.get_logger().warn(
                        "航点悬停时飞控连接中断，继续发送 setpoint 并等待恢复"
                    )
                lost_duration = now - connection_lost_since
                if lost_duration >= self.connection_loss_timeout:
                    self.node.get_logger().error(
                        "航点悬停时飞控连续断开 %.1f s，超过允许的 %.1f s"
                        % (lost_duration, self.connection_loss_timeout)
                    )
                    return False
                rate.sleep()
                continue

            if connection_lost_since is not None:
                self.node.get_logger().info("飞控连接已恢复，继续完成航点悬停")
                connection_lost_since = None
            held_time += elapsed
            rate.sleep()
        return rclpy.ok() and self.node.context.ok()

    def _wait_for_connection_recovery(self):
        if self.current_state.connected:
            return True
        self.node.get_logger().warn(
            "等待飞控连接恢复，最长 %.1f s ..." % self.connection_timeout
        )
        deadline = now_s() + self.connection_timeout
        rate = self.node.create_rate(5)
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.connected:
                self.node.get_logger().info("飞控连接已恢复")
                return True
            if now_s() >= deadline:
                self.node.get_logger().error(
                    "等待飞控连接恢复超时，无法通过 MAVROS 请求 RTL"
                )
                return False
            rate.sleep()
        return False

    # ------------------------------------------------------------------
    # RTL / LAND
    # ------------------------------------------------------------------
    def _read_rtl_alt_final(self):
        if self._param_get_srv is None:
            return None
        req = ParamGet.Request()
        req.param_id = "RTL_ALT_FINAL"
        ok, response = self._call_service(
            self._param_get_srv, req, self.service_timeout
        )
        if not ok:
            self.node.get_logger().warn("读取 RTL_ALT_FINAL 失败")
            return None
        if not response.success:
            self.node.get_logger().warn("飞控未返回 RTL_ALT_FINAL")
            return None
        value = (
            response.value.integer if response.value.integer != 0
            else response.value.real
        )
        self.node.get_logger().info("RTL_ALT_FINAL=%.3f cm" % float(value))
        return float(value)

    def _distance_from_start(self):
        gps_distance = self._horizontal_wgs84_distance(
            self.start_latitude,
            self.start_longitude,
            self.current_fix.latitude,
            self.current_fix.longitude,
        )
        position = self.current_pose.pose.position
        local_distance = math.hypot(
            position.y - self.start_north,
            position.x - self.start_east,
        )
        return gps_distance, local_distance

    def land_and_wait(self):
        if not self.set_mode("LAND"):
            return False
        self.node.get_logger().info("LAND 已生效，等待自动落地并上锁 ...")
        deadline = now_s() + self.landing_timeout
        rate = self.node.create_rate(5)
        while rclpy.ok() and self.node.context.ok():
            if not self.current_state.armed:
                self.node.get_logger().info("降落完成：飞控已自动上锁")
                return True
            if now_s() >= deadline:
                self.node.get_logger().error(
                    "等待 LAND 自动上锁超时（%.1fs）" % self.landing_timeout
                )
                return False
            rate.sleep()
        return False

    def rtl_and_wait(self):
        rtl_alt_final = self._read_rtl_alt_final()
        if not self.set_mode("RTL"):
            return False

        if rtl_alt_final is not None and rtl_alt_final > 0.0:
            self.node.get_logger().warn(
                "RTL_ALT_FINAL > 0：RTL 回到 Home 后不会自动降落，届时将请求 LAND"
            )
        else:
            self.node.get_logger().info(
                "RTL 应自动返回并降落；若回到 Home 后长时间未降落，将请求 LAND"
            )

        deadline = now_s() + self.rtl_timeout
        home_inside_since = None
        home_confirmed_at = None
        connection_lost_since = None
        rate = self.node.create_rate(5)
        while rclpy.ok() and self.node.context.ok():
            if not self.current_state.connected:
                if connection_lost_since is None:
                    connection_lost_since = now_s()
                    self.node.get_logger().warn(
                        "RTL 过程中飞控连接断开；飞控已处于 RTL，等待连接恢复"
                    )
                throttle_log(
                    self.node, 5.0,
                    "RTL 中等待飞控连接恢复，已断开 %.1f s",
                    now_s() - connection_lost_since,
                    level="warn",
                )
                if now_s() >= deadline:
                    self.node.get_logger().error("RTL 等待超时且飞控连接仍未恢复")
                    return False
                rate.sleep()
                continue
            if connection_lost_since is not None:
                self.node.get_logger().info("RTL 过程中飞控连接已恢复")
                connection_lost_since = None
            if not self.current_state.armed:
                self.node.get_logger().info("RTL 已完成自动降落并上锁")
                return True

            if self._pose_received and self._fix_received:
                gps_distance, local_distance = self._distance_from_start()
                inside_home = (
                    gps_distance <= self.rtl_home_tolerance
                    and local_distance <= self.rtl_home_tolerance
                )
                if inside_home:
                    if home_inside_since is None:
                        home_inside_since = now_s()
                    if (
                        home_confirmed_at is None
                        and now_s() - home_inside_since >= self.rtl_home_hold_time
                    ):
                        home_confirmed_at = now_s()
                        self.node.get_logger().info(
                            "已确认回到起飞点附近: GPS=%.2f m local=%.2f m"
                            % (gps_distance, local_distance)
                        )
                else:
                    home_inside_since = None

                if home_confirmed_at is not None:
                    no_auto_land = (
                        rtl_alt_final is not None and rtl_alt_final > 0.0
                    )
                    auto_land_wait_expired = (
                        now_s() - home_confirmed_at >= self.rtl_auto_land_timeout
                    )
                    if no_auto_land or auto_land_wait_expired:
                        self.node.get_logger().warn(
                            "飞机已在 Home 附近但仍未上锁，切换 LAND"
                        )
                        return self.land_and_wait()

                throttle_log(
                    self.node, 2.0,
                    "RTL 中: 距起飞点 GPS=%.2f m local=%.2f m rel_alt=%.2f m mode=%s",
                    gps_distance, local_distance,
                    self.relative_altitude, self.current_state.mode,
                )

            if now_s() >= deadline:
                if home_confirmed_at is not None:
                    self.node.get_logger().warn(
                        "RTL 总等待超时，但飞机已在 Home 附近，切换 LAND"
                    )
                    return self.land_and_wait()
                self.node.get_logger().error(
                    "RTL 等待超时且未确认回到起飞点；保持 RTL，不在异地强制 LAND"
                )
                return False
            rate.sleep()
        return False

    # ------------------------------------------------------------------
    # 异常处置
    # ------------------------------------------------------------------
    def _abort_to_rtl_or_land(self):
        if not self.current_state.connected and not self._wait_for_connection_recovery():
            return
        if not self.current_state.armed:
            return
        self.node.get_logger().error("飞行流程异常，优先尝试切换 RTL")
        if not self.set_mode("RTL"):
            self.node.get_logger().error("RTL 切换失败，尝试切换 LAND")
            self.set_mode("LAND")

    def _abort_before_takeoff(self):
        """尚未发出起飞命令时不请求 RTL，只让已解锁飞机进入 LAND。"""
        if not self.current_state.connected and not self._wait_for_connection_recovery():
            self.node.get_logger().error(
                "连接未恢复，无法确认起飞状态或发送安全模式指令"
            )
            return
        if not self.current_state.armed:
            return
        if self._takeoff_has_started():
            self.node.get_logger().error("检测到飞机已经离地，改为请求 RTL")
            self._abort_to_rtl_or_land()
            return
        self.node.get_logger().error("起飞命令尚未发送，切换 LAND 等待地面安全处置")
        if not self.set_mode("LAND"):
            self.node.get_logger().error("LAND 切换失败，请立即使用遥控器接管并上锁")

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

        try:
            self._freeze_start_and_target()
        except ValueError as exc:
            self.node.get_logger().error("起飞前检查失败: %s" % exc)
            return 1
        self._print_authorization_summary()
        if not self._request_authorization():
            return 2
        if not self._validate_after_authorization():
            return 1

        if not self.set_mode("GUIDED"):
            return 1
        if not self.arm():
            return 1
        if not self._capture_ground_altitude_reference():
            self._abort_before_takeoff()
            return 1
        if not self.takeoff():
            self._abort_before_takeoff()
            return 1
        if not self._wait_until_takeoff_altitude():
            self._abort_to_rtl_or_land()
            return 1

        target = self._make_global_target()
        if not self.fly_to_target(target):
            self._abort_to_rtl_or_land()
            return 1
        if not self.hold_global_target(target):
            self._abort_to_rtl_or_land()
            return 1
        if not self.rtl_and_wait():
            return 1
        return 0


def main():
    rclpy.init()
    node = rclpy.create_node("one_key_takeoff_wgs84_forward_rtl")

    # 后台 spin：订阅回调（state/pose/fix/heading/rel_alt）必须由 executor 驱动，
    # ROS2 的 rate.sleep() 不会处理回调。
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    exit_code = 2
    try:
        try:
            c2_node = Wgs84ForwardRtlNode(node)
        except ValueError as exc:
            node.get_logger().fatal("参数错误: %s" % exc)
            exit_code = 2
        else:
            c2_node._executor = executor  # 同一执行器，避免自建 executor 冲突
            exit_code = c2_node.run()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
