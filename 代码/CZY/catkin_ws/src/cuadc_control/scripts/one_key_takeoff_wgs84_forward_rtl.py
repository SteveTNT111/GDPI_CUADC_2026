#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
飞手确认后自动执行：GUIDED -> ARM -> 起飞 3 m -> 沿启动航向飞行
10 m 对应的 WGS84 航点 -> 悬停 5 s -> RTL -> 必要时在 Home 切 LAND。

安全约束：
  - 程序只允许在飞机确定停在地面时启动；启动后冻结起始航向、位置和目标航点。
  - 解锁后、起飞前重新记录地面的相对 Home 高度，允许该高度不等于 0。
  - 切换模式或解锁前，必须由飞手在终端输入授权短语。
  - RTL 未自动降落时，只有确认飞机已回到起飞点附近才请求 LAND。
  - 飞行阶段发生异常时优先请求 RTL；RTL 请求失败才请求 LAND。

坐标约定：
  - /mavros/local_position/pose 为 ROS ENU，打印时转换为 NED。
  - 航向来自 /mavros/global_position/compass_hdg，0 度为北、顺时针为正。
  - 经纬度目标使用 WGS84 椭球局部曲率计算。
  - 全局 setpoint 使用 MAV_FRAME_GLOBAL_RELATIVE_ALT_INT，高度相对 Home。
"""

import math
import os
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, ParamGet, SetMode
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64


def _ros_master_is_alive():
    try:
        import xmlrpc.client

        master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        master = xmlrpc.client.ServerProxy(master_uri)
        master.getPid("/one_key_takeoff_wgs84_forward_rtl_startup_check")
        return True
    except Exception:
        return False


def _start_roscore():
    print("[auto-start] 正在启动 roscore ...")
    try:
        subprocess.Popen(
            ["roscore"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
    except Exception as exc:
        print("[auto-start] 启动 roscore 失败: {}".format(exc))
        sys.exit(1)


def _ensure_roscore(timeout=30.0):
    if _ros_master_is_alive():
        return
    _start_roscore()
    deadline = time.monotonic() + timeout
    while not _ros_master_is_alive():
        if time.monotonic() >= deadline:
            print("[auto-start] roscore 启动超时（{:.1f}s）".format(timeout))
            sys.exit(1)
        time.sleep(1.0)
    print("[auto-start] roscore 已就绪")


class Wgs84ForwardRtlNode:
    """执行 WGS84 前向航点和 RTL 任务。"""

    WGS84_A = 6378137.0
    WGS84_F = 1.0 / 298.257223563

    def __init__(self):
        self.takeoff_altitude = float(rospy.get_param("~takeoff_altitude", 3.0))
        self.forward_distance = float(rospy.get_param("~forward_distance", 10.0))
        self.waypoint_hover_time = float(
            rospy.get_param("~waypoint_hover_time", 5.0)
        )
        self.authorization_phrase = str(
            rospy.get_param("~authorization_phrase", "YES")
        )

        self.auto_start_mavros = bool(
            rospy.get_param("~auto_start_mavros", True)
        )
        self.mavros_fcu_url = rospy.get_param(
            "~mavros_fcu_url", "/dev/ttyACM0:115200"
        )
        self.connection_timeout = float(
            rospy.get_param("~connection_timeout", 30.0)
        )
        self.connection_loss_timeout = max(
            1.0, float(rospy.get_param("~connection_loss_timeout", 10.0))
        )
        self.service_timeout = float(rospy.get_param("~service_timeout", 10.0))
        self.data_timeout = float(rospy.get_param("~data_timeout", 30.0))
        self.data_max_age = float(rospy.get_param("~data_max_age", 2.0))
        self.navigation_loss_timeout = max(
            self.data_max_age,
            float(rospy.get_param("~navigation_loss_timeout", 5.0)),
        )
        self.mode_timeout = float(rospy.get_param("~mode_timeout", 6.0))
        self.arm_timeout = float(rospy.get_param("~arm_timeout", 6.0))
        self.takeoff_command_attempts = max(
            1, int(rospy.get_param("~takeoff_command_attempts", 2))
        )
        self.takeoff_command_retry_delay = max(
            0.0, float(rospy.get_param("~takeoff_command_retry_delay", 1.0))
        )
        self.takeoff_ack_timeout_threshold = max(
            1.0, float(rospy.get_param("~takeoff_ack_timeout_threshold", 4.0))
        )
        self.takeoff_started_threshold = max(
            0.10, float(rospy.get_param("~takeoff_started_threshold", 0.30))
        )
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 40.0))
        self.waypoint_timeout = float(rospy.get_param("~waypoint_timeout", 90.0))
        self.rtl_timeout = float(rospy.get_param("~rtl_timeout", 240.0))
        self.rtl_auto_land_timeout = float(
            rospy.get_param("~rtl_auto_land_timeout", 90.0)
        )
        self.landing_timeout = float(rospy.get_param("~landing_timeout", 120.0))

        self.takeoff_tolerance = max(
            0.05, float(rospy.get_param("~takeoff_tolerance", 0.20))
        )
        self.waypoint_tolerance = max(
            0.20, float(rospy.get_param("~waypoint_tolerance", 1.0))
        )
        self.vertical_tolerance = max(
            0.10, float(rospy.get_param("~vertical_tolerance", 0.40))
        )
        self.arrival_hold_time = max(
            0.0, float(rospy.get_param("~arrival_hold_time", 2.0))
        )
        self.rtl_home_tolerance = max(
            0.50, float(rospy.get_param("~rtl_home_tolerance", 2.0))
        )
        self.rtl_home_hold_time = max(
            0.0, float(rospy.get_param("~rtl_home_hold_time", 3.0))
        )
        self.authorization_max_drift = max(
            0.20, float(rospy.get_param("~authorization_max_drift", 2.0))
        )
        self.post_arm_altitude_settle_time = max(
            0.0, float(rospy.get_param("~post_arm_altitude_settle_time", 2.0))
        )
        self.setpoint_rate = max(
            2.0, float(rospy.get_param("~setpoint_rate", 10.0))
        )

        if self.takeoff_altitude <= 0.0:
            raise ValueError("takeoff_altitude 必须大于 0")
        if self.forward_distance <= 0.0:
            raise ValueError("forward_distance 必须大于 0")
        if not self.authorization_phrase:
            raise ValueError("authorization_phrase 不能为空")

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

        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=10
        )
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._pose_cb, queue_size=10
        )
        self.fix_sub = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self._fix_cb, queue_size=10
        )
        self.heading_sub = rospy.Subscriber(
            "/mavros/global_position/compass_hdg",
            Float64,
            self._heading_cb,
            queue_size=10,
        )
        self.relative_altitude_sub = rospy.Subscriber(
            "/mavros/global_position/rel_alt",
            Float64,
            self._relative_altitude_cb,
            queue_size=10,
        )
        self.global_setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_raw/global",
            GlobalPositionTarget,
            queue_size=20,
        )

    def _state_cb(self, msg):
        self.current_state = msg
        self._state_received = True

    def _pose_cb(self, msg):
        self.current_pose = msg
        self._pose_received = True
        self._pose_received_at = time.monotonic()

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
            self._fix_received_at = time.monotonic()

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

    def _mavros_is_alive(self):
        try:
            return any(
                topic == "/mavros/state"
                for topic, _msg_type in rospy.get_published_topics()
            )
        except Exception:
            return False

    def _wait_for_mavros(self):
        deadline = time.monotonic() + self.connection_timeout
        while not rospy.is_shutdown():
            if self._mavros_is_alive():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(1.0)
        return False

    def _ensure_mavros(self):
        if self._mavros_is_alive():
            rospy.loginfo("[auto-start] MAVROS 已在运行")
            return True
        if not self.auto_start_mavros:
            rospy.logerr("MAVROS 未运行，且 auto_start_mavros=false")
            return False
        rospy.loginfo("[auto-start] 启动 MAVROS: fcu_url=%s", self.mavros_fcu_url)
        try:
            subprocess.Popen(
                [
                    "roslaunch",
                    "mavros",
                    "apm.launch",
                    "fcu_url:={}".format(self.mavros_fcu_url),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            rospy.logerr("启动 MAVROS 失败: %s", exc)
            return False
        if self._wait_for_mavros():
            rospy.loginfo("[auto-start] MAVROS 已就绪")
            return True
        rospy.logerr("MAVROS 启动超时（%.1fs）", self.connection_timeout)
        return False

    def _wait_for_connection(self):
        rospy.loginfo("等待飞控连接 ...")
        deadline = time.monotonic() + self.connection_timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self._state_received and self.current_state.connected:
                rospy.loginfo("飞控已连接")
                return True
            if time.monotonic() >= deadline:
                rospy.logerr("等待飞控连接超时（%.1fs）", self.connection_timeout)
                return False
            rate.sleep()
        return False

    def _wait_for_navigation_data(self):
        rospy.loginfo("等待 NED、WGS84、航向和相对高度数据 ...")
        deadline = time.monotonic() + self.data_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._navigation_data_is_fresh():
                rospy.loginfo("NED、WGS84、航向和相对高度数据已刷新")
                return True
            if time.monotonic() >= deadline:
                rospy.logerr(
                    "等待导航数据超时: pose=%s fix=%s heading=%s rel_alt=%s",
                    self._pose_received,
                    self._fix_received,
                    self._heading_received,
                    self._relative_altitude_received,
                )
                return False
            rate.sleep()
        return False

    def _navigation_data_is_fresh(self):
        now = time.monotonic()
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
            and time.monotonic() - self._relative_altitude_received_at
            <= self.data_max_age
        )

    def _ensure_services(self):
        if self._set_mode_srv is not None:
            return True
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=self.service_timeout)
            rospy.wait_for_service("/mavros/cmd/arming", timeout=self.service_timeout)
            rospy.wait_for_service("/mavros/cmd/takeoff", timeout=self.service_timeout)
            self._set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self._arming_srv = rospy.ServiceProxy(
                "/mavros/cmd/arming", CommandBool
            )
            self._takeoff_srv = rospy.ServiceProxy(
                "/mavros/cmd/takeoff", CommandTOL
            )
            try:
                rospy.wait_for_service(
                    "/mavros/param/get", timeout=self.service_timeout
                )
                self._param_get_srv = rospy.ServiceProxy(
                    "/mavros/param/get", ParamGet
                )
            except rospy.ROSException:
                rospy.logwarn("参数服务不可用，将通过 RTL 状态超时判断是否需要 LAND")
            return True
        except rospy.ROSException as exc:
            rospy.logerr("等待 MAVROS 服务失败: %s", exc)
            return False

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
        target_wgs84_altitude = self.start_wgs84_altitude + self.takeoff_altitude
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
            "目标 WGS84   : lat={:.9f}, lon={:.9f}, alt≈{:.3f} m".format(
                self.target_latitude,
                self.target_longitude,
                target_wgs84_altitude,
            ),
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
        prompt = (
            "确认空域安全后输入 {!r} 授权飞行（英文字母不区分大小写，其他输入取消）: "
        ).format(self.authorization_phrase)
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
        if entered_phrase.casefold() != self.authorization_phrase.strip().casefold():
            rospy.logwarn(
                "飞手未授权：输入=%r，期望=%r；任务取消，未发送模式、解锁或起飞命令",
                entered_phrase,
                self.authorization_phrase,
            )
            return False
        rospy.loginfo("飞手已授权，开始执行飞行任务")
        return True

    def _validate_after_authorization(self):
        if not self.current_state.connected:
            rospy.logerr("授权后飞控连接已断开")
            return False
        if not self._navigation_data_is_fresh():
            rospy.logwarn("授权等待期间导航数据已变旧，等待下一批数据刷新 ...")
            if not self._wait_for_navigation_data():
                rospy.logerr("授权后无法取得最新导航数据")
                return False
        drift = self._horizontal_wgs84_distance(
            self.start_latitude,
            self.start_longitude,
            self.current_fix.latitude,
            self.current_fix.longitude,
        )
        if drift > self.authorization_max_drift:
            rospy.logerr(
                "授权期间飞机位置漂移 %.2f m，超过 %.2f m；请重新启动并确认",
                drift,
                self.authorization_max_drift,
            )
            return False
        return True

    def _wait_for_mode(self, expected_mode):
        deadline = time.monotonic() + self.mode_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state.mode == expected_mode:
                return True
            if time.monotonic() >= deadline:
                return False
            rate.sleep()
        return False

    def set_mode(self, mode):
        try:
            response = self._set_mode_srv(custom_mode=mode)
        except rospy.ServiceException as exc:
            rospy.logerr("set_mode 服务调用失败: %s", exc)
            return False
        if not response.mode_sent or not self._wait_for_mode(mode):
            rospy.logerr(
                "模式切换失败: expected=%s current=%s", mode, self.current_state.mode
            )
            return False
        rospy.loginfo("飞行模式已切换为 %s", mode)
        return True

    def arm(self):
        if self.current_state.armed:
            rospy.logwarn("飞机在授权前已处于 armed=True，跳过重复解锁")
            return True
        try:
            response = self._arming_srv(value=True)
        except rospy.ServiceException as exc:
            rospy.logerr("arming 服务调用失败: %s", exc)
            return False
        if not response.success:
            rospy.logerr("飞控拒绝解锁: result=%d", getattr(response, "result", -1))
            return False
        deadline = time.monotonic() + self.arm_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state.armed:
                rospy.loginfo("飞机已解锁")
                return True
            if time.monotonic() >= deadline:
                rospy.logerr("等待 armed=True 超时")
                return False
            rate.sleep()
        return False

    def _capture_ground_altitude_reference(self):
        """解锁后在地面记录相对 Home 高度，并换算实际起飞目标高度。"""
        rospy.loginfo(
            "飞机应保持在地面，等待相对高度数据持续有效 %.1f s ...",
            self.post_arm_altitude_settle_time,
        )
        deadline = time.monotonic() + self.data_timeout
        fresh_since = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.current_state.connected or not self.current_state.armed:
                rospy.logerr("记录地面高度基准时飞控断开或飞机已上锁")
                return False

            now = time.monotonic()
            if self._relative_altitude_is_fresh():
                if fresh_since is None:
                    fresh_since = now
                if now - fresh_since >= self.post_arm_altitude_settle_time:
                    self.ground_relative_altitude = self.relative_altitude
                    self.target_relative_altitude = (
                        self.ground_relative_altitude + self.takeoff_altitude
                    )
                    rospy.loginfo(
                        "地面高度基准已记录: ground_rel_alt=%.2f m, "
                        "计划爬升=%.2f m, target_rel_alt=%.2f m",
                        self.ground_relative_altitude,
                        self.takeoff_altitude,
                        self.target_relative_altitude,
                    )
                    return True
            else:
                fresh_since = None

            if now >= deadline:
                rospy.logerr(
                    "等待最新相对高度数据超时（%.1fs）: received=%s age=%.2fs",
                    self.data_timeout,
                    self._relative_altitude_received,
                    now - self._relative_altitude_received_at,
                )
                return False
            rate.sleep()
        return False

    def takeoff(self):
        if self.target_relative_altitude is None:
            rospy.logerr("尚未记录地面高度基准，拒绝发送起飞命令")
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
                rospy.logerr("发送起飞命令前飞机已上锁")
                return False
            if self.current_state.mode != "GUIDED":
                rospy.logwarn(
                    "发送起飞命令前模式为 %r，重新确认 GUIDED",
                    self.current_state.mode,
                )
                if not self.set_mode("GUIDED"):
                    return False

            rospy.loginfo(
                "发送起飞命令（第 %d/%d 次）: target_rel_alt=%.2f m",
                attempt,
                self.takeoff_command_attempts,
                self.target_relative_altitude,
            )
            call_started_at = time.monotonic()
            service_exception = None
            response = None
            try:
                response = self._takeoff_srv(
                    altitude=self.target_relative_altitude,
                    latitude=0.0,
                    longitude=0.0,
                    min_pitch=0.0,
                    yaw=0.0,
                )
            except rospy.ServiceException as exc:
                service_exception = exc
            call_duration = time.monotonic() - call_started_at

            if response is not None and response.success:
                rospy.loginfo(
                    "起飞命令已确认，目标相对 Home 高度 %.2f m"
                    "（离启动地面上升 %.2f m）",
                    self.target_relative_altitude,
                    self.takeoff_altitude,
                )
                return True

            result = getattr(response, "result", -1)
            result_name = result_names.get(result, "UNKNOWN")
            transport_failure = (
                service_exception is not None
                or not self.current_state.connected
                or not self.current_state.mode
                or call_duration >= self.takeoff_ack_timeout_threshold
            )
            if not transport_failure:
                rospy.logerr(
                    "飞控明确拒绝起飞命令: result=%d (%s), mode=%s, armed=%s",
                    result,
                    result_name,
                    self.current_state.mode,
                    self.current_state.armed,
                )
                return False

            rospy.logwarn(
                "起飞命令确认异常: result=%d (%s), duration=%.2fs, "
                "connected=%s, mode=%r, exception=%s；命令可能已执行，先检查高度",
                result,
                result_name,
                call_duration,
                self.current_state.connected,
                self.current_state.mode,
                service_exception,
            )
            if not self.current_state.connected:
                if not self._wait_for_connection_recovery():
                    return False

            altitude_deadline = time.monotonic() + min(5.0, self.data_timeout)
            rate = rospy.Rate(10)
            while not rospy.is_shutdown() and time.monotonic() < altitude_deadline:
                if self._relative_altitude_is_fresh():
                    break
                rate.sleep()
            if self._takeoff_has_started():
                rospy.logwarn(
                    "虽然未收到可靠 ACK，但飞机已上升 %.2f m，按起飞命令已执行继续监控",
                    self.relative_altitude - self.ground_relative_altitude,
                )
                return True

            if attempt < self.takeoff_command_attempts:
                rospy.logwarn(
                    "连接已恢复且飞机仍在地面，%.1f s 后重试起飞命令",
                    self.takeoff_command_retry_delay,
                )
                rospy.sleep(self.takeoff_command_retry_delay)

        rospy.logerr("起飞命令重试 %d 次后仍未确认执行", self.takeoff_command_attempts)
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
        deadline = time.monotonic() + self.takeoff_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.current_state.connected or not self.current_state.armed:
                rospy.logerr("爬升过程中飞控断开或飞机已上锁")
                return False
            if self.relative_altitude >= accepted_altitude:
                rospy.loginfo(
                    "已到达起飞高度: rel_alt=%.2f / %.2f m",
                    self.relative_altitude,
                    self.target_relative_altitude,
                )
                return True
            if time.monotonic() >= deadline:
                rospy.logerr(
                    "等待起飞高度超时: rel_alt=%.2f / %.2f m",
                    self.relative_altitude,
                    self.target_relative_altitude,
                )
                return False
            rospy.loginfo_throttle(
                2.0,
                "爬升中: rel_alt=%.2f / %.2f m",
                self.relative_altitude,
                self.target_relative_altitude,
            )
            rate.sleep()
        return False

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
        target.header.stamp = rospy.Time.now()
        self.global_setpoint_pub.publish(target)

    def fly_to_target(self, target):
        rospy.loginfo(
            "发送 WGS84 全局位置目标: lat=%.9f lon=%.9f rel_alt=%.2f m",
            target.latitude,
            target.longitude,
            target.altitude,
        )
        deadline = time.monotonic() + self.waypoint_timeout
        inside_since = None
        navigation_stale_since = None
        connection_lost_since = None
        rate = rospy.Rate(self.setpoint_rate)
        while not rospy.is_shutdown():
            if self.current_state.connected and not self.current_state.armed:
                rospy.logerr("飞往航点时飞机已上锁")
                return False
            self._publish_global_target(target)

            if not self.current_state.connected:
                now = time.monotonic()
                if connection_lost_since is None:
                    connection_lost_since = now
                    rospy.logwarn("飞往航点时飞控连接中断，继续发送 setpoint 并等待恢复")
                lost_duration = now - connection_lost_since
                if lost_duration >= self.connection_loss_timeout:
                    rospy.logerr(
                        "飞往航点时飞控连续断开 %.1f s，超过允许的 %.1f s",
                        lost_duration,
                        self.connection_loss_timeout,
                    )
                    return False
                rate.sleep()
                continue
            if connection_lost_since is not None:
                rospy.loginfo("飞控连接已恢复，继续飞往 WGS84 航点")
                connection_lost_since = None

            if not self._navigation_data_is_fresh():
                now = time.monotonic()
                if navigation_stale_since is None:
                    navigation_stale_since = now
                stale_duration = now - navigation_stale_since
                if stale_duration >= self.navigation_loss_timeout:
                    rospy.logerr(
                        "飞往航点时导航数据连续中断 %.1f s，超过允许的 %.1f s",
                        stale_duration,
                        self.navigation_loss_timeout,
                    )
                    return False
                rospy.logwarn_throttle(
                    1.0,
                    "飞往航点时等待导航数据刷新: 已中断 %.1f / %.1f s",
                    stale_duration,
                    self.navigation_loss_timeout,
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
                    inside_since = time.monotonic()
                if time.monotonic() - inside_since >= self.arrival_hold_time:
                    rospy.loginfo(
                        "已抵达 WGS84 航点: horizontal_error=%.2f m "
                        "vertical_error=%.2f m",
                        horizontal_error,
                        vertical_error,
                    )
                    return True
            else:
                inside_since = None

            if time.monotonic() >= deadline:
                rospy.logerr(
                    "飞往 WGS84 航点超时: horizontal_error=%.2f m "
                    "vertical_error=%.2f m",
                    horizontal_error,
                    vertical_error,
                )
                return False
            rospy.loginfo_throttle(
                1.0,
                "飞往 WGS84 航点: horizontal_error=%.2f m "
                "vertical_error=%.2f m",
                horizontal_error,
                vertical_error,
            )
            rate.sleep()
        return False

    def hold_global_target(self, target):
        rospy.loginfo("已抵达目标，持续发送 setpoint 悬停 %.1f s", self.waypoint_hover_time)
        held_time = 0.0
        last_loop_at = time.monotonic()
        connection_lost_since = None
        rate = rospy.Rate(self.setpoint_rate)
        while not rospy.is_shutdown() and held_time < self.waypoint_hover_time:
            now = time.monotonic()
            elapsed = max(0.0, now - last_loop_at)
            last_loop_at = now

            if self.current_state.connected and not self.current_state.armed:
                rospy.logerr("航点悬停时飞机已上锁")
                return False
            self._publish_global_target(target)

            if not self.current_state.connected:
                if connection_lost_since is None:
                    connection_lost_since = now
                    rospy.logwarn("航点悬停时飞控连接中断，继续发送 setpoint 并等待恢复")
                lost_duration = now - connection_lost_since
                if lost_duration >= self.connection_loss_timeout:
                    rospy.logerr(
                        "航点悬停时飞控连续断开 %.1f s，超过允许的 %.1f s",
                        lost_duration,
                        self.connection_loss_timeout,
                    )
                    return False
                rate.sleep()
                continue

            if connection_lost_since is not None:
                rospy.loginfo("飞控连接已恢复，继续完成航点悬停")
                connection_lost_since = None
            held_time += elapsed
            rate.sleep()
        return not rospy.is_shutdown()

    def _wait_for_connection_recovery(self):
        if self.current_state.connected:
            return True
        rospy.logwarn(
            "等待飞控连接恢复，最长 %.1f s ...", self.connection_timeout
        )
        deadline = time.monotonic() + self.connection_timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.current_state.connected:
                rospy.loginfo("飞控连接已恢复")
                return True
            if time.monotonic() >= deadline:
                rospy.logerr("等待飞控连接恢复超时，无法通过 MAVROS 请求 RTL")
                return False
            rate.sleep()
        return False

    def _read_rtl_alt_final(self):
        if self._param_get_srv is None:
            return None
        try:
            response = self._param_get_srv(param_id="RTL_ALT_FINAL")
        except rospy.ServiceException as exc:
            rospy.logwarn("读取 RTL_ALT_FINAL 失败: %s", exc)
            return None
        if not response.success:
            rospy.logwarn("飞控未返回 RTL_ALT_FINAL")
            return None
        value = response.value.integer if response.value.integer != 0 else response.value.real
        rospy.loginfo("RTL_ALT_FINAL=%.3f cm", float(value))
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
        rospy.loginfo("LAND 已生效，等待自动落地并上锁 ...")
        deadline = time.monotonic() + self.landing_timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.current_state.armed:
                rospy.loginfo("降落完成：飞控已自动上锁")
                return True
            if time.monotonic() >= deadline:
                rospy.logerr("等待 LAND 自动上锁超时（%.1fs）", self.landing_timeout)
                return False
            rate.sleep()
        return False

    def rtl_and_wait(self):
        rtl_alt_final = self._read_rtl_alt_final()
        if not self.set_mode("RTL"):
            return False

        if rtl_alt_final is not None and rtl_alt_final > 0.0:
            rospy.logwarn(
                "RTL_ALT_FINAL > 0：RTL 回到 Home 后不会自动降落，届时将请求 LAND"
            )
        else:
            rospy.loginfo(
                "RTL 应自动返回并降落；若回到 Home 后长时间未降落，将请求 LAND"
            )

        deadline = time.monotonic() + self.rtl_timeout
        home_inside_since = None
        home_confirmed_at = None
        connection_lost_since = None
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.current_state.connected:
                if connection_lost_since is None:
                    connection_lost_since = time.monotonic()
                    rospy.logwarn("RTL 过程中飞控连接断开；飞控已处于 RTL，等待连接恢复")
                rospy.logwarn_throttle(
                    5.0,
                    "RTL 中等待飞控连接恢复，已断开 %.1f s",
                    time.monotonic() - connection_lost_since,
                )
                if time.monotonic() >= deadline:
                    rospy.logerr("RTL 等待超时且飞控连接仍未恢复")
                    return False
                rate.sleep()
                continue
            if connection_lost_since is not None:
                rospy.loginfo("RTL 过程中飞控连接已恢复")
                connection_lost_since = None
            if not self.current_state.armed:
                rospy.loginfo("RTL 已完成自动降落并上锁")
                return True

            if self._pose_received and self._fix_received:
                gps_distance, local_distance = self._distance_from_start()
                inside_home = (
                    gps_distance <= self.rtl_home_tolerance
                    and local_distance <= self.rtl_home_tolerance
                )
                if inside_home:
                    if home_inside_since is None:
                        home_inside_since = time.monotonic()
                    if (
                        home_confirmed_at is None
                        and time.monotonic() - home_inside_since
                        >= self.rtl_home_hold_time
                    ):
                        home_confirmed_at = time.monotonic()
                        rospy.loginfo(
                            "已确认回到起飞点附近: GPS=%.2f m local=%.2f m",
                            gps_distance,
                            local_distance,
                        )
                else:
                    home_inside_since = None

                if home_confirmed_at is not None:
                    no_auto_land = rtl_alt_final is not None and rtl_alt_final > 0.0
                    auto_land_wait_expired = (
                        time.monotonic() - home_confirmed_at
                        >= self.rtl_auto_land_timeout
                    )
                    if no_auto_land or auto_land_wait_expired:
                        rospy.logwarn("飞机已在 Home 附近但仍未上锁，切换 LAND")
                        return self.land_and_wait()

                rospy.loginfo_throttle(
                    2.0,
                    "RTL 中: 距起飞点 GPS=%.2f m local=%.2f m "
                    "rel_alt=%.2f m mode=%s",
                    gps_distance,
                    local_distance,
                    self.relative_altitude,
                    self.current_state.mode,
                )

            if time.monotonic() >= deadline:
                if home_confirmed_at is not None:
                    rospy.logwarn("RTL 总等待超时，但飞机已在 Home 附近，切换 LAND")
                    return self.land_and_wait()
                rospy.logerr(
                    "RTL 等待超时且未确认回到起飞点；保持 RTL，不在异地强制 LAND"
                )
                return False
            rate.sleep()
        return False

    def _abort_to_rtl_or_land(self):
        if not self.current_state.connected and not self._wait_for_connection_recovery():
            return
        if not self.current_state.armed:
            return
        rospy.logerr("飞行流程异常，优先尝试切换 RTL")
        if not self.set_mode("RTL"):
            rospy.logerr("RTL 切换失败，尝试切换 LAND")
            self.set_mode("LAND")

    def _abort_before_takeoff(self):
        """尚未发出起飞命令时不请求 RTL，只让已解锁飞机进入 LAND。"""
        if not self.current_state.connected and not self._wait_for_connection_recovery():
            rospy.logerr("连接未恢复，无法确认起飞状态或发送安全模式指令")
            return
        if not self.current_state.armed:
            return
        if self._takeoff_has_started():
            rospy.logerr("检测到飞机已经离地，改为请求 RTL")
            self._abort_to_rtl_or_land()
            return
        rospy.logerr("起飞命令尚未发送，切换 LAND 等待地面安全处置")
        if not self.set_mode("LAND"):
            rospy.logerr("LAND 切换失败，请立即使用遥控器接管并上锁")

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
            rospy.logerr("起飞前检查失败: %s", exc)
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
    _ensure_roscore()
    rospy.init_node("one_key_takeoff_wgs84_forward_rtl")
    try:
        node = Wgs84ForwardRtlNode()
    except ValueError as exc:
        rospy.logfatal("参数错误: %s", exc)
        raise SystemExit(2)
    exit_code = node.run()
    if exit_code != 0:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
