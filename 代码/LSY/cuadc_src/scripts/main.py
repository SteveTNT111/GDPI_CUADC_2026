#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
main.py — CUADC 2026 主控节点

功能：
  1. 通过 MAVROS 连接飞控
  2. 状态机驱动：待命 → 解锁 → 起飞 → 巡航 → 任务 → 返航 → 着陆
  3. 切换飞行模式（GUIDED / AUTO / RTL / LOITER 等）
  4. 发送起飞指令

依赖：
  - rospy
  - mavros_msgs (State, SetMode, CommandBool, CommandTOL, OverrideRCIn)
  - geometry_msgs

运行方式：
  roslaunch cuadc_vision runmain.launch
"""

import time
import threading

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


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

        # ---------- 状态 ----------
        self.state = self.STATE_INIT
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.state_lock = threading.Lock()

        # ---------- MAVROS 订阅 ----------
        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=10
        )
        self.local_pos_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._pos_cb, queue_size=10
        )

        # ---------- MAVROS 服务代理（延迟连接） ----------
        self._set_mode_srv = None
        self._arming_srv = None
        self._takeoff_srv = None

        # ---------- 发布 ----------
        self.vel_pub = rospy.Publisher(
            "/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10
        )

        rospy.loginfo("FlightController 初始化完成，等待 MAVROS 连接...")

    # ==================== 回调 ====================

    def _state_cb(self, msg):
        self.current_state = msg

    def _pos_cb(self, msg):
        self.current_pose = msg

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
        """等待 EKF 融合就绪（GPS + IMU）"""
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            # PX4 的 EKF 通过本地位置协方差判断
            pose = self.current_pose
            cov = pose.pose.covariance
            # 简化判断：位置协方差矩阵非零即认为 EKF 有一定收敛
            if cov[0] > 0.0 and not rospy.get_param("/mavros/state", self.current_state).system_status == 0:
                rospy.loginfo("EKF 就绪")
                return True
            rospy.loginfo_throttle(5.0, "等待 EKF 收敛...")
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
                # 此处由检测/投放逻辑接管
                rospy.loginfo_throttle(5.0, "任务执行中...")

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
    rospy.init_node("flight_controller")
    fc = FlightController()
    try:
        fc.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
