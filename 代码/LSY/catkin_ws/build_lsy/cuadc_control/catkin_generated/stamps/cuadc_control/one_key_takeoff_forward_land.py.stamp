#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
已解锁前提下执行：GUIDED 起飞 2 m -> 悬停 2 s -> 沿当前航向前飞 1 m
-> 悬停 3 s -> LAND。

安全约束：
  - 本脚本不会创建或调用 /mavros/cmd/arming。
  - 起飞前必须从 /mavros/state 确认 armed=True，否则立即退出。
  - 飞行阶段失败时仅尝试切换 LAND，不发送上锁/解锁命令。

坐标约定：
  - 飞控本地坐标按 NED 解算：x=North, y=East, z=Down。
  - /mavros/local_position/pose 使用 ROS ENU：x=East, y=North, z=Up。
  - 航点先在 NED 中计算，再转换为 ENU 发布到
    /mavros/setpoint_position/local。MAVROS 最终发送本地位置目标。
"""

import copy
import math
import os
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, SetMode
from std_msgs.msg import Float64


def _ros_master_is_alive():
    """检查 ROS master 是否可达。"""
    try:
        import xmlrpc.client

        master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        master = xmlrpc.client.ServerProxy(master_uri)
        master.getPid("/one_key_takeoff_forward_land_startup_check")
        return True
    except Exception:
        return False


def _start_roscore():
    """后台启动 roscore。"""
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
    """确保 roscore 已运行。"""
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


class TakeoffForwardLandNode:
    """执行已解锁前提下的起飞、前飞和降落流程。"""

    def __init__(self):
        self.takeoff_altitude = float(
            rospy.get_param("~takeoff_altitude", 2.0)
        )
        self.forward_distance = float(
            rospy.get_param("~forward_distance", 1.0)
        )
        self.takeoff_hover_time = float(
            rospy.get_param("~takeoff_hover_time", 2.0)
        )
        self.waypoint_hover_time = float(
            rospy.get_param("~waypoint_hover_time", 3.0)
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
        self.service_timeout = float(rospy.get_param("~service_timeout", 10.0))
        self.data_timeout = float(rospy.get_param("~data_timeout", 30.0))
        self.data_max_age = float(rospy.get_param("~data_max_age", 2.0))
        self.mode_timeout = float(rospy.get_param("~mode_timeout", 5.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 30.0))
        self.waypoint_timeout = float(
            rospy.get_param("~waypoint_timeout", 20.0)
        )
        self.landing_timeout = float(rospy.get_param("~landing_timeout", 90.0))

        self.takeoff_tolerance = max(
            0.02, float(rospy.get_param("~takeoff_tolerance", 0.15))
        )
        self.waypoint_tolerance = max(
            0.02, float(rospy.get_param("~waypoint_tolerance", 0.15))
        )
        self.vertical_tolerance = max(
            0.02, float(rospy.get_param("~vertical_tolerance", 0.20))
        )
        self.setpoint_rate = max(
            5.0, float(rospy.get_param("~setpoint_rate", 20.0))
        )

        if self.takeoff_altitude <= 0.0:
            raise ValueError("takeoff_altitude 必须大于 0")
        if self.forward_distance <= 0.0:
            raise ValueError("forward_distance 必须大于 0")

        self.current_state = State()
        self.current_pose = PoseStamped()
        self.heading_deg = None
        self.relative_altitude = None

        self._state_received = False
        self._pose_received = False
        self._heading_received = False
        self._relative_altitude_received = False
        self._pose_received_at = 0.0
        self._heading_received_at = 0.0
        self._relative_altitude_received_at = 0.0

        self._set_mode_srv = None
        self._takeoff_srv = None

        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=10
        )
        self.pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose",
            PoseStamped,
            self._pose_cb,
            queue_size=10,
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
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=20
        )

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

        rospy.loginfo(
            "[auto-start] 启动 MAVROS: fcu_url=%s", self.mavros_fcu_url
        )
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
        """等待本地位置与飞控航向；零坐标也是有效数据。"""
        rospy.loginfo("等待本地位置和航向数据 ...")
        deadline = time.monotonic() + self.data_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (
                self._pose_received
                and self._heading_received
                and self._relative_altitude_received
            ):
                pos = self.current_pose.pose.position
                rospy.loginfo(
                    "导航数据就绪: ENU=(%.3f, %.3f, %.3f), "
                    "heading=%.2f deg, rel_alt=%.2f m",
                    pos.x,
                    pos.y,
                    pos.z,
                    self.heading_deg,
                    self.relative_altitude,
                )
                return True
            if time.monotonic() >= deadline:
                rospy.logerr(
                    "等待导航数据超时: local_pose=%s compass_hdg=%s "
                    "rel_alt=%s",
                    self._pose_received,
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
            and self._heading_received
            and self._relative_altitude_received
            and now - self._pose_received_at <= self.data_max_age
            and now - self._heading_received_at <= self.data_max_age
            and now - self._relative_altitude_received_at <= self.data_max_age
        )

    def _ensure_services(self):
        """只连接模式和起飞服务；这里刻意没有 arming 服务。"""
        if self._set_mode_srv is not None:
            return True
        try:
            rospy.wait_for_service(
                "/mavros/set_mode", timeout=self.service_timeout
            )
            rospy.wait_for_service(
                "/mavros/cmd/takeoff", timeout=self.service_timeout
            )
            self._set_mode_srv = rospy.ServiceProxy(
                "/mavros/set_mode", SetMode
            )
            self._takeoff_srv = rospy.ServiceProxy(
                "/mavros/cmd/takeoff", CommandTOL
            )
            rospy.loginfo("模式与起飞服务已连接（未连接 arming 服务）")
            return True
        except rospy.ROSException as exc:
            rospy.logerr("等待 MAVROS 服务失败: %s", exc)
            return False

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
        if not self._ensure_services():
            return False
        try:
            resp = self._set_mode_srv(custom_mode=mode)
        except rospy.ServiceException as exc:
            rospy.logerr("set_mode 服务调用失败: %s", exc)
            return False
        if not resp.mode_sent:
            rospy.logerr("模式切换请求被拒绝: %s", mode)
            return False
        if not self._wait_for_mode(mode):
            rospy.logerr(
                "模式未切换成功: expected=%s current=%s",
                mode,
                self.current_state.mode,
            )
            return False
        rospy.loginfo("飞行模式已切换为 %s", mode)
        return True

    def _confirm_prearmed(self):
        """确认操作者已解锁；绝不主动发送解锁命令。"""
        if not self.current_state.connected:
            rospy.logerr("飞控连接已断开，禁止起飞")
            return False
        if not self.current_state.armed:
            rospy.logerr(
                "飞机尚未解锁：请由操作者手动解锁后重新运行；脚本不会发送解锁命令"
            )
            return False
        if not self._navigation_data_is_fresh():
            rospy.logerr("本地位置或航向数据已超时，禁止起飞")
            return False
        rospy.loginfo("起飞前确认完成：connected=True, armed=True")
        return True

    def takeoff(self):
        """发送一次 MAV_CMD_NAV_TAKEOFF；调用前再次确认 armed=True。"""
        if not self._confirm_prearmed():
            return False
        try:
            resp = self._takeoff_srv(
                altitude=self.takeoff_altitude,
                latitude=0.0,
                longitude=0.0,
                min_pitch=0.0,
                yaw=0.0,
            )
        except rospy.ServiceException as exc:
            rospy.logerr("takeoff 服务调用失败: %s", exc)
            return False
        if not resp.success:
            rospy.logerr(
                "飞控拒绝起飞命令: result=%d", getattr(resp, "result", -1)
            )
            return False
        rospy.loginfo("起飞命令已发送，目标相对高度 %.2f m", self.takeoff_altitude)
        return True

    def _wait_until_takeoff_altitude(self):
        """用相对 Home 高度判断 MAV_CMD_NAV_TAKEOFF 是否到达目标。"""
        accepted_altitude = self.takeoff_altitude - self.takeoff_tolerance
        deadline = time.monotonic() + self.takeoff_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (
                not self.current_state.connected
                or not self.current_state.armed
            ):
                rospy.logerr("爬升过程中飞控断开或飞机已上锁")
                return False
            if self.relative_altitude >= accepted_altitude:
                rospy.loginfo(
                    "已到达起飞高度: relative_altitude %.2f / %.2f m",
                    self.relative_altitude,
                    self.takeoff_altitude,
                )
                return True
            if time.monotonic() >= deadline:
                rospy.logerr(
                    "等待起飞高度超时: relative_altitude %.2f / %.2f m",
                    self.relative_altitude,
                    self.takeoff_altitude,
                )
                return False
            rospy.loginfo_throttle(
                2.0,
                "爬升中: relative_altitude %.2f / %.2f m",
                self.relative_altitude,
                self.takeoff_altitude,
            )
            rate.sleep()
        return False

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
        target.header.stamp = rospy.Time.now()
        self.setpoint_pub.publish(target)

    def hold_target(self, target, duration, description):
        rospy.loginfo("%s，持续 %.1f s", description, duration)
        deadline = time.monotonic() + duration
        rate = rospy.Rate(self.setpoint_rate)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if (
                not self.current_state.connected
                or not self.current_state.armed
            ):
                rospy.logerr("保持航点时飞控断开或飞机已上锁")
                return False
            self._publish_target(target)
            rate.sleep()
        return not rospy.is_shutdown()

    def fly_to_target(self, target):
        deadline = time.monotonic() + self.waypoint_timeout
        rate = rospy.Rate(self.setpoint_rate)
        while not rospy.is_shutdown():
            if (
                not self.current_state.connected
                or not self.current_state.armed
            ):
                rospy.logerr("飞向航点时飞控断开或飞机已上锁")
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
                rospy.loginfo(
                    "到达前方航点: horizontal_error=%.3f m "
                    "vertical_error=%.3f m",
                    horizontal_error,
                    vertical_error,
                )
                return True
            if time.monotonic() >= deadline:
                rospy.logerr(
                    "飞往航点超时: horizontal_error=%.3f m "
                    "vertical_error=%.3f m",
                    horizontal_error,
                    vertical_error,
                )
                return False
            rospy.loginfo_throttle(
                1.0,
                "飞往前方航点: horizontal_error=%.3f m "
                "vertical_error=%.3f m",
                horizontal_error,
                vertical_error,
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

        rospy.loginfo(
            "当前 NED=(N %.3f, E %.3f, D %.3f), heading=%.2f deg",
            current_north,
            current_east,
            current_down,
            heading_deg,
        )
        rospy.loginfo(
            "目标 NED=(N %.3f, E %.3f, D %.3f), 前进距离=%.2f m",
            target_north,
            target_east,
            target_down,
            self.forward_distance,
        )

        # 飞控 NED -> MAVROS/ROS ENU：E->x, N->y, -D->z。
        return self._make_enu_target(
            east=target_east,
            north=target_north,
            up=-target_down,
            orientation=self.current_pose.pose.orientation,
        )

    def land(self):
        if not self.set_mode("LAND"):
            return False
        rospy.loginfo("LAND 已生效，等待飞控自动落地并上锁 ...")
        deadline = time.monotonic() + self.landing_timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not self.current_state.armed:
                rospy.loginfo("降落完成：飞控已自动上锁")
                return True
            if time.monotonic() >= deadline:
                rospy.logerr(
                    "等待自动落地超时（%.1fs），当前模式=%s armed=%s",
                    self.landing_timeout,
                    self.current_state.mode,
                    self.current_state.armed,
                )
                return False
            rate.sleep()
        return False

    def _abort_to_land(self):
        """起飞后的流程失败时请求 LAND；不发送解锁或上锁命令。"""
        if self.current_state.connected and self.current_state.armed:
            rospy.logerr("飞行流程异常，尝试切换 LAND")
            self.set_mode("LAND")

    def run(self):
        if not self._ensure_mavros():
            return 1
        if not self._wait_for_connection():
            return 1
        if not self._wait_for_navigation_data():
            return 1
        if not self._ensure_services():
            return 1
        if not self._confirm_prearmed():
            return 1

        if not self.set_mode("GUIDED"):
            return 1
        if not self._confirm_prearmed():
            return 1
        if not self.takeoff():
            return 1

        if not self._wait_until_takeoff_altitude():
            self._abort_to_land()
            return 1

        reached_pose = copy.deepcopy(self.current_pose)
        # rel_alt 与 local_position 的零点可能不同。用两者当前差值把
        # “相对 Home 的 2 m”换算到本地 ENU z，而不是假定 local z=2。
        target_up = (
            reached_pose.pose.position.z
            + self.takeoff_altitude
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
            "已到达 {:.2f} m 起飞航点，原地悬停".format(
                self.takeoff_altitude
            ),
        ):
            self._abort_to_land()
            return 1

        if not self._navigation_data_is_fresh():
            rospy.logerr("计算前方航点前导航数据已超时")
            self._abort_to_land()
            return 1
        if not self.fly_to_target(forward_target):
            self._abort_to_land()
            return 1
        if not self.hold_target(
            forward_target,
            self.waypoint_hover_time,
            "已到达前方 {:.2f} m 航点，悬停".format(
                self.forward_distance
            ),
        ):
            self._abort_to_land()
            return 1
        if not self.land():
            return 1
        return 0


def main():
    _ensure_roscore()
    rospy.init_node("one_key_takeoff_forward_land")
    try:
        node = TakeoffForwardLandNode()
    except ValueError as exc:
        rospy.logfatal("参数错误: %s", exc)
        raise SystemExit(2)

    exit_code = node.run()
    if exit_code != 0:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
