#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
one_key_takeoff.py — 一键起飞并切换到 LOITER 悬停

流程：
  1. 自动检查并启动 roscore / MAVROS（可关闭）
  2. 等待飞控连接与本地位姿可用
  3. 切换到 GUIDED
  4. 解锁
  5. 发送起飞到目标高度（默认 3.0 m）
  6. 到达高度后切换到 LOITER

运行方式：
  python3 one_key_takeoff.py
  rosrun cuadc_control one_key_takeoff.py _takeoff_altitude:=3.0
"""

import os
import subprocess
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


def _ros_master_is_alive():
    """检查 ROS master 是否可达"""
    try:
        import xmlrpc.client
        master_uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
        master = xmlrpc.client.ServerProxy(master_uri)
        master.getPid("/one_key_takeoff_startup_check")
        return True
    except Exception:
        return False


def _start_roscore():
    """后台启动 roscore"""
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


def _wait_for_ros_master(timeout=30.0):
    """等待 ROS master 上线"""
    deadline = time.time() + timeout
    while not _ros_master_is_alive():
        if time.time() >= deadline:
            return False
        time.sleep(1.0)
    return True


def _ensure_roscore():
    """确保 roscore 在运行"""
    if _ros_master_is_alive():
        return
    print("[auto-start] ROS master 未运行，自动启动 roscore ...")
    _start_roscore()
    if not _wait_for_ros_master():
        print("[auto-start] 错误：roscore 启动超时（30s），请手动检查")
        sys.exit(1)
    print("[auto-start] roscore 已就绪")


class OneKeyTakeoffNode:
    """执行一键起飞并切换 LOITER 的控制节点"""

    def __init__(self):
        self.takeoff_altitude = float(rospy.get_param("~takeoff_altitude", 3.0))
        self.hover_mode = rospy.get_param("~hover_mode", "LOITER")
        self.auto_start_mavros = bool(rospy.get_param("~auto_start_mavros", True))
        self.mavros_fcu_url = rospy.get_param("~mavros_fcu_url", "/dev/ttyACM0:115200")
        self.connection_timeout = float(rospy.get_param("~connection_timeout", 30.0))
        self.service_timeout = float(rospy.get_param("~service_timeout", 10.0))
        self.ekf_timeout = float(rospy.get_param("~ekf_timeout", 30.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 30.0))
        self.altitude_reach_ratio = max(
            1.0, float(rospy.get_param("~altitude_reach_ratio", 1.0))
        )
        self.mode_settle_time = float(rospy.get_param("~mode_settle_time", 1.5))
        self.arm_settle_time = float(rospy.get_param("~arm_settle_time", 1.5))
        self.monitor_only = bool(rospy.get_param("~monitor_after_loiter", True))

        self.current_state = State()
        self.current_pose = PoseStamped()

        self._set_mode_srv = None
        self._arming_srv = None
        self._takeoff_srv = None

        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=10
        )
        self.local_pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._pose_cb, queue_size=10
        )

    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _mavros_is_alive(self):
        """检查 MAVROS 是否在运行"""
        try:
            return any(topic == "/mavros/state" for topic, _ in rospy.get_published_topics())
        except Exception:
            return False

    def _wait_for_mavros(self, timeout=30.0):
        """等待 MAVROS 上线"""
        deadline = time.time() + timeout
        while not rospy.is_shutdown():
            if self._mavros_is_alive():
                return True
            if time.time() >= deadline:
                return False
            time.sleep(1.0)
        return False

    def _ensure_mavros(self):
        """确保 MAVROS 在运行"""
        if self._mavros_is_alive():
            rospy.loginfo("[auto-start] MAVROS 已在运行")
            return True
        if not self.auto_start_mavros:
            rospy.logerr("MAVROS 未运行，且 auto_start_mavros=false")
            return False

        rospy.loginfo("[auto-start] MAVROS 未运行，自动启动 mavros apm.launch ...")
        try:
            subprocess.Popen(
                ["roslaunch", "mavros", "apm.launch", "fcu_url:={}".format(self.mavros_fcu_url)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except Exception as exc:
            rospy.logerr("[auto-start] 启动 MAVROS 失败: %s", exc)
            return False

        if self._wait_for_mavros(timeout=self.connection_timeout):
            rospy.loginfo("[auto-start] MAVROS 已就绪")
            return True

        rospy.logerr("[auto-start] MAVROS 启动超时（%.1fs）", self.connection_timeout)
        return False

    def _wait_for_connection(self):
        """等待飞控连接建立"""
        rospy.loginfo("等待飞控连接 ...")
        deadline = time.time() + self.connection_timeout
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.current_state.connected:
                rospy.loginfo("飞控已连接")
                return True
            if time.time() >= deadline:
                rospy.logerr("等待飞控连接超时（%.1fs）", self.connection_timeout)
                return False
            rate.sleep()
        return False

    def _wait_for_local_pose(self):
        """等待本地位姿可用，作为起飞前的 EKF 就绪条件"""
        rospy.loginfo("等待本地位姿 /mavros/local_position/pose ...")
        deadline = time.time() + self.ekf_timeout
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            pos = self.current_pose.pose.position
            has_pose = (
                abs(pos.x) > 0.001 or
                abs(pos.y) > 0.001 or
                abs(pos.z) > 0.001
            )
            if has_pose and self.current_state.connected:
                rospy.loginfo("本地位姿已就绪 (x=%.3f y=%.3f z=%.3f)", pos.x, pos.y, pos.z)
                return True
            if time.time() >= deadline:
                rospy.logerr(
                    "等待本地位姿超时（%.1fs），当前 (x=%.3f y=%.3f z=%.3f)",
                    self.ekf_timeout, pos.x, pos.y, pos.z
                )
                return False
            rate.sleep()
        return False

    def _ensure_services(self):
        """连接 MAVROS 所需服务"""
        if self._set_mode_srv is not None:
            return True
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=self.service_timeout)
            rospy.wait_for_service("/mavros/cmd/arming", timeout=self.service_timeout)
            rospy.wait_for_service("/mavros/cmd/takeoff", timeout=self.service_timeout)
            self._set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self._arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            self._takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
            rospy.loginfo("MAVROS 服务已连接")
            return True
        except rospy.ROSException as exc:
            rospy.logerr("等待 MAVROS 服务失败: %s", exc)
            return False

    def _wait_for_mode(self, expected_mode, timeout=5.0):
        """等待模式切换完成"""
        deadline = time.time() + timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.current_state.mode == expected_mode:
                return True
            if time.time() >= deadline:
                return False
            rate.sleep()
        return False

    def _wait_for_arm_state(self, expected_armed, timeout=5.0):
        """等待解锁状态切换完成"""
        deadline = time.time() + timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.current_state.armed == expected_armed:
                return True
            if time.time() >= deadline:
                return False
            rate.sleep()
        return False

    def set_mode(self, mode):
        """切换飞行模式"""
        if not self._ensure_services():
            return False
        try:
            resp = self._set_mode_srv(custom_mode=mode)
        except rospy.ServiceException as exc:
            rospy.logerr("set_mode 服务调用失败: %s", exc)
            return False

        if not resp.mode_sent:
            rospy.logerr("飞行模式切换请求被拒绝: %s", mode)
            return False

        if not self._wait_for_mode(mode, timeout=self.mode_settle_time + 3.0):
            rospy.logerr(
                "模式请求已发送，但目标模式未生效: expected=%s current=%s",
                mode, self.current_state.mode
            )
            return False
        rospy.loginfo("飞行模式切换至: %s", mode)
        return True

    def arm(self):
        """解锁无人机"""
        if not self._ensure_services():
            return False
        if self.current_state.armed:
            rospy.loginfo("无人机已解锁")
            return True
        try:
            resp = self._arming_srv(value=True)
        except rospy.ServiceException as exc:
            rospy.logerr("arming 服务调用失败: %s", exc)
            return False

        if not resp.success:
            rospy.logerr("解锁失败: result=%d", getattr(resp, "result", -1))
            return False

        if not self._wait_for_arm_state(True, timeout=self.arm_settle_time + 3.0):
            rospy.logerr("解锁请求已发送，但状态未更新为 armed")
            return False
        rospy.loginfo("解锁成功")
        return True

    def takeoff(self):
        """发送起飞指令"""
        if not self._ensure_services():
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
            rospy.logerr("起飞指令发送失败")
            return False

        rospy.loginfo("起飞指令已发送，目标高度 %.2f m", self.takeoff_altitude)
        return True

    def wait_until_altitude_reached(self):
        """等待上升到目标高度"""
        target = self.takeoff_altitude * self.altitude_reach_ratio
        deadline = time.time() + self.takeoff_timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            current_alt = self.current_pose.pose.position.z
            if current_alt >= target:
                rospy.loginfo("已到达目标高度附近: %.2f / %.2f m", current_alt, self.takeoff_altitude)
                return True
            if time.time() >= deadline:
                rospy.logerr(
                    "等待起飞高度超时（%.1fs），当前高度 %.2f m，目标 %.2f m",
                    self.takeoff_timeout, current_alt, self.takeoff_altitude
                )
                return False
            rospy.loginfo_throttle(
                2.0, "爬升中: %.2f / %.2f m", current_alt, self.takeoff_altitude
            )
            rate.sleep()
        return False

    def run(self):
        """执行一键起飞流程"""
        if not self._ensure_mavros():
            return 1
        if not self._wait_for_connection():
            return 1
        if not self._ensure_services():
            return 1
        if not self._wait_for_local_pose():
            return 1
        if not self.set_mode("GUIDED"):
            return 1
        rospy.sleep(self.mode_settle_time)
        if not self.arm():
            return 1
        rospy.sleep(self.arm_settle_time)
        if not self.takeoff():
            return 1
        if not self.wait_until_altitude_reached():
            return 1
        rospy.sleep(1.0)
        if not self.set_mode(self.hover_mode):
            return 1

        rospy.loginfo("流程完成：无人机已切换到 %s 模式保持悬停", self.hover_mode)

        if not self.monitor_only:
            return 0

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pos = self.current_pose.pose.position
            rospy.loginfo_throttle(
                5.0,
                "悬停监控中... mode=%s armed=%s z=%.2f m",
                self.current_state.mode,
                self.current_state.armed,
                pos.z,
            )
            rate.sleep()
        return 0


def main():
    _ensure_roscore()
    rospy.init_node("one_key_takeoff")
    node = OneKeyTakeoffNode()
    exit_code = node.run()
    if exit_code != 0:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
