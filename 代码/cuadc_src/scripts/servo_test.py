#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
servo_test.py — CUADC 2026 抛投器舵机测试节点

功能：
  1. 通过 MAVROS RC Override 控制飞控 5/6 通道舵机
  2. 终端交互模式：手动输入 on/off 控制舵机开关
  3. ROS 话题控制：订阅 /servo/cmd 话题接收控制指令
  4. PWM: 1100 = 关闭, 1400 = 打开

通道说明：
  - 5 通道 (CH5)：抛投器 1 舵机  [默认启用]
  - 6 通道 (CH6)：抛投器 2 舵机  [默认禁用]
  - 可使用一个舵机通过机械联动同时驱动两侧抛投器，
    也可使用两个舵机独立控制（启用 CH6 即可）

依赖：
  - rospy
  - mavros_msgs (OverrideRCIn)
  - std_msgs

运行方式：
  roslaunch cuadc_vision run_servo_test.launch

终端命令：
  on  / open   → 打开抛投器（PWM=1400）
  off / close  → 关闭抛投器（PWM=1100）
  q   / quit   → 退出
"""

import sys
import threading

import rospy
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Bool, String

# Python 2/3 兼容
try:
    input_func = raw_input
except NameError:
    input_func = input


class ServoController:
    """抛投器舵机控制"""

    def __init__(self):
        # ---------- 参数 ----------
        # 通道开关
        self.enable_ch5 = rospy.get_param("~enable_ch5", True)
        self.enable_ch6 = rospy.get_param("~enable_ch6", False)

        # PWM 值（μs）
        self.pwm_open = rospy.get_param("~pwm_open", 1400)
        self.pwm_close = rospy.get_param("~pwm_close", 1100)

        # 初始状态
        self.servo_open = False  # False = 关闭, True = 打开
        self.lock = threading.Lock()

        # ---------- MAVROS RC Override 发布 ----------
        self.rc_pub = rospy.Publisher(
            "/mavros/rc/override", OverrideRCIn, queue_size=10
        )

        # ---------- ROS 话题订阅 ----------
        # /servo/cmd  (std_msgs/String): "on" / "off" / "open" / "close"
        self.cmd_sub = rospy.Subscriber(
            "/servo/cmd", String, self._string_cmd_cb, queue_size=10
        )
        # /servo/open  (std_msgs/Bool): True=打开, False=关闭
        self.open_sub = rospy.Subscriber(
            "/servo/open", Bool, self._bool_cmd_cb, queue_size=10
        )

        # 启动时先发送一次关闭信号
        rospy.sleep(0.5)
        self._set_servo(False)

        rospy.loginfo(
            "ServoController 就绪 | CH5=%s CH6=%s | PWM_close=%d PWM_open=%d",
            "启用" if self.enable_ch5 else "禁用",
            "启用" if self.enable_ch6 else "禁用",
            self.pwm_close,
            self.pwm_open,
        )
        rospy.loginfo("终端命令: on/open = 打开  |  off/close = 关闭  |  q/quit = 退出")

    # ==================== RC Override 核心 ====================

    def _set_servo(self, state):
        """设置舵机状态

        Args:
            state: True = 打开 (PWM=1400), False = 关闭 (PWM=1100)
        """
        pwm = self.pwm_open if state else self.pwm_close

        msg = OverrideRCIn()
        # MAVROS RC Override 默认值是 0 和 65535 (未覆盖)，
        # 需要先填充默认值（UINT16_MAX 表示不覆盖该通道）
        msg.channels = [65535] * 18

        if self.enable_ch5:
            msg.channels[4] = pwm  # CH5 = index 4
        if self.enable_ch6:
            msg.channels[5] = pwm  # CH6 = index 5

        self.rc_pub.publish(msg)

        with self.lock:
            self.servo_open = state

        action = "打开" if state else "关闭"
        channels = []
        if self.enable_ch5:
            channels.append("CH5")
        if self.enable_ch6:
            channels.append("CH6")
        rospy.loginfo("舵机 %s → %s (PWM=%d)", "+".join(channels), action, pwm)

    # ==================== 回调 ====================

    def _string_cmd_cb(self, msg):
        """/servo/cmd 话题回调（String 指令）"""
        cmd = msg.data.strip().lower()
        self._handle_command(cmd)

    def _bool_cmd_cb(self, msg):
        """/servo/open 话题回调（Bool 指令）"""
        self._set_servo(msg.data)

    def _handle_command(self, cmd):
        """解析并执行指令"""
        if cmd in ("on", "open", "1", "true"):
            self._set_servo(True)
        elif cmd in ("off", "close", "0", "false"):
            self._set_servo(False)
        elif cmd == "toggle":
            with self.lock:
                self._set_servo(not self.servo_open)
        else:
            rospy.logwarn("无法识别的指令: '%s' (支持: on/off/open/close/toggle)", cmd)

    # ==================== 终端交互 ====================

    def _terminal_loop(self):
        """终端输入线程 —— 手动控制舵机"""
        print("")
        print("=" * 50)
        print("  舵机测试终端")
        print("  输入 on/open  → 打开抛投器 (PWM=%d)" % self.pwm_open)
        print("  输入 off/close → 关闭抛投器 (PWM=%d)" % self.pwm_close)
        print("  输入 q/quit    → 退出")
        print("=" * 50)
        print("")

        while not rospy.is_shutdown():
            try:
                cmd = input_func("servo> ").strip().lower()
                if not cmd:
                    continue
                if cmd in ("q", "quit", "exit"):
                    rospy.loginfo("用户退出舵机终端")
                    rospy.signal_shutdown("用户退出")
                    break
                self._handle_command(cmd)
            except (EOFError, KeyboardInterrupt):
                rospy.loginfo("终端输入中断")
                break

    # ==================== 主入口 ====================

    def run(self):
        """启动终端交互线程并进入 ROS spin"""
        terminal_thread = threading.Thread(target=self._terminal_loop)
        terminal_thread.daemon = True
        terminal_thread.start()

        rospy.spin()


def main():
    rospy.init_node("servo_controller")
    controller = ServoController()
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
