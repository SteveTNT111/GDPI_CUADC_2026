#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
safe_flight_test.py — 绝对安全的 ENU 坐标系 setpoint 飞行验证

用法：
  终端1: roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
  终端2: rosrun cuadc_vision safe_flight_test.py _test_distance:=2.0

安全策略：
  - 不自动解锁、不自动起飞（用遥控器手动起飞）
  - 遥控器切 GUIDED 后代码才开始发 setpoint
  - 目标 = 当前位置 + 正北 N 米（ENU坐标系，你想往哪飞就改参数）
  - 到达后自动切 LOITER 让你接管
  - ⚠️ 全程遥控器 CH5 开关 = 紧急切断，不对劲立刻切回 LOITER！

坐标系说明：
  发的是 ENU（东北天）坐标系，原点 = 起飞点，单位 = 米
  x = 东(East)，y = 北(North)，z = 上(Up)
  和 /mavros/local_position/pose 是同一个坐标系
"""

import rospy
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

# ==================== 你只需要改这里 ====================
TEST_DISTANCE = rospy.get_param("~test_distance", 2.0)   # 飞多远（米）
TEST_DIRECTION = rospy.get_param("~test_direction", "north")  # north/east/south/west
TARGET_HEIGHT = rospy.get_param("~target_height", 0.0)   # 高度变化（0=保持当前高度）
ARRIVE_THRESHOLD = 0.5  # 到达判定距离（米）
# =======================================================


class SafeFlightTest:
    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = None
        self.target_set = False
        self.arrived = False
        self.start_pose = None

        # 方向到 ENU 偏移的映射
        direction_map = {
            "north": (0, TEST_DISTANCE),
            "south": (0, -TEST_DISTANCE),
            "east":  (TEST_DISTANCE, 0),
            "west":  (-TEST_DISTANCE, 0),
        }
        if TEST_DIRECTION not in direction_map:
            rospy.logerr("test_direction 必须是 north/south/east/west 之一，收到: %s", TEST_DIRECTION)
            rospy.signal_shutdown("invalid direction")
            return
        self.dx, self.dy = direction_map[TEST_DIRECTION]

        # 订阅
        rospy.Subscriber("/mavros/state", State, self._state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pos_cb)

        # 发布 setpoint（ENU 坐标系）
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=10)

        # 模式切换服务
        rospy.wait_for_service("/mavros/set_mode")
        self.set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        rospy.loginfo("=" * 60)
        rospy.loginfo("安全飞行测试初始化完成")
        rospy.loginfo("飞向: %s 方向 %.1f 米", TEST_DIRECTION.upper(), TEST_DISTANCE)
        rospy.loginfo("ENU 偏移: dx=%.1f (东) dy=%.1f (北)", self.dx, self.dy)
        rospy.loginfo("=" * 60)
        rospy.loginfo("⚠️  安全提醒:")
        rospy.loginfo("  1. 用遥控器手动起飞到安全高度")
        rospy.loginfo("  2. 遥控器切 GUIDED 模式后代码自动开始发 setpoint")
        rospy.loginfo("  3. 手放在 CH5 开关上！不对劲立刻切 LOITER！")
        rospy.loginfo("=" * 60)

    def _state_cb(self, msg):
        self.current_state = msg

    def _pos_cb(self, msg):
        self.current_pose = msg

    def run(self):
        rate = rospy.Rate(20)  # 20Hz，稳定发送 setpoint

        # ========== 阶段1：等待 GUIDED 模式 ==========
        rospy.loginfo("等待遥控器切到 GUIDED 模式...")
        rospy.loginfo("（当前模式: %s，武装: %s）",
                      self.current_state.mode, self.current_state.armed)
        while not rospy.is_shutdown():
            if self.current_state.mode == "GUIDED":
                rospy.loginfo("✅ 检测到 GUIDED 模式！")
                break
            rospy.loginfo_throttle(3.0, "当前模式: %s，请在遥控器上切 GUIDED",
                                   self.current_state.mode)
            rate.sleep()

        # ========== 阶段2：记录当前位置，计算目标 ==========
        rospy.sleep(1.0)  # 等飞控稳定
        self.start_pose = self.current_pose
        start_x = self.start_pose.pose.position.x
        start_y = self.start_pose.pose.position.y
        start_z = self.start_pose.pose.position.z

        target_z = start_z if TARGET_HEIGHT == 0 else TARGET_HEIGHT

        rospy.loginfo("=" * 60)
        rospy.loginfo("当前位置 (ENU): x=%.2f  y=%.2f  z=%.2f", start_x, start_y, start_z)
        rospy.loginfo("目标位置 (ENU): x=%.2f  y=%.2f  z=%.2f",
                      start_x + self.dx, start_y + self.dy, target_z)
        rospy.loginfo("偏移: Δx=%.2f(东)  Δy=%.2f(北)  距离≈%.2fm",
                      self.dx, self.dy,
                      (self.dx**2 + self.dy**2)**0.5)
        rospy.loginfo("=" * 60)
        rospy.loginfo("开始发送 setpoint，观察飞机动作...")
        rospy.loginfo("手放在 CH5 开关上！不对劲立刻切 LOITER！")

        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = start_x + self.dx
        self.target_pose.pose.position.y = start_y + self.dy
        self.target_pose.pose.position.z = target_z

        # ========== 阶段3：持续发 setpoint 直到到达 ==========
        while not rospy.is_shutdown():
            # 如果遥控器切出了 GUIDED，立即停止
            if self.current_state.mode != "GUIDED":
                rospy.logwarn("⚠️  检测到飞行模式切换为 %s，停止发送 setpoint",
                              self.current_state.mode)
                break

            # 发 setpoint
            self.target_pose.header.stamp = rospy.Time.now()
            self.setpoint_pub.publish(self.target_pose)

            # 检查是否到达
            cur_x = self.current_pose.pose.position.x
            cur_y = self.current_pose.pose.position.y
            dist = ((cur_x - self.target_pose.pose.position.x)**2 +
                    (cur_y - self.target_pose.pose.position.y)**2)**0.5

            if dist < ARRIVE_THRESHOLD:
                if not self.arrived:
                    self.arrived = True
                    rospy.loginfo("✅ 已到达目标！距离误差: %.2fm", dist)
                    rospy.loginfo("3秒后自动切换 LOITER 模式...")
                    rospy.sleep(3.0)
                break

            rospy.loginfo_throttle(1.0, "飞行中... 距目标: %.2fm | ENU(%.1f,%.1f) → (%.1f,%.1f)",
                                   dist, cur_x, cur_y,
                                   self.target_pose.pose.position.x,
                                   self.target_pose.pose.position.y)
            rate.sleep()

        # ========== 阶段4：切回 LOITER，把控制权还给你 ==========
        rospy.loginfo("切换 LOITER 模式，控制权交还遥控器")
        self.set_mode_srv(custom_mode="LOITER")
        rospy.loginfo("=" * 60)
        rospy.loginfo("测试完成。飞机现在由遥控器控制。")
        rospy.loginfo("可以手动飞回来降落了。")
        rospy.loginfo("=" * 60)


def main():
    rospy.init_node("safe_flight_test")
    tester = SafeFlightTest()
    try:
        tester.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
