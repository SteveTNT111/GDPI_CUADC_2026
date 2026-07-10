#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
servo_test.py — CUADC 2026 抛投器舵机测试节点

功能：
  1. 通过 MAVROS MAV_CMD_DO_SET_SERVO 直接控制飞控舵机输出（不依赖 SERVOx_FUNCTION）
  2. 终端交互模式：手动输入命令分别控制前/后舵机
  3. ROS 话题控制：订阅 /servo/cmd 话题接收控制指令
  4. 实时显示飞控连接状态和心跳

舵机配置（前后两个独立舵机）：
  - 前舵机：SERVO5
  - 后舵机：SERVO6
  - PWM 1000：关闭
  - PWM 1500：打开

飞控参数要求（ArduPilot）：
  - RC_OVERRIDE_TIME > 0（允许外部 MAVLink 指令，默认 3s 即可）
  - SERVO5_FUNCTION / SERVO6_FUNCTION 可以保持 0 (Disabled)，
    因为 MAV_CMD_DO_SET_SERVO 不经过飞控逻辑，直接操作输出引脚

依赖：
  - rospy
  - mavros_msgs (CommandLong, State)
  - std_msgs

运行方式：
  roslaunch cuadc_vision run_servo_test.launch

终端命令：
  on / open               → 打开前后全部舵机
  off / close             → 关闭前后全部舵机
  front open / front off  → 控制前舵机（CH5）
  rear open / rear off    → 控制后舵机（CH6）
  q / quit                → 退出
"""

import sys
import threading

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Bool, String

# Python 2/3 兼容
try:
    input_func = raw_input
except NameError:
    input_func = input


class ServoController:
    """抛投器舵机控制 —— 使用 MAV_CMD_DO_SET_SERVO"""

    def __init__(self):
        # ---------- 参数 ----------
        self.enable_ch5 = rospy.get_param("~enable_ch5", True)
        self.enable_ch6 = rospy.get_param("~enable_ch6", True)

        # PWM 值（μs）—— 前后两个独立舵机
        self.pwm_open = rospy.get_param("~pwm_open", 1500)
        self.pwm_close = rospy.get_param("~pwm_close", 1000)
        self.front_servo_channel = 5
        self.rear_servo_channel = 6

        # 初始状态
        self.servo_states = {
            self.front_servo_channel: False,
            self.rear_servo_channel: False,
        }
        self.servo_open = False
        self.lock = threading.Lock()

        # ---------- 飞控连接状态 ----------
        self.fc_connected = False
        self.fc_armed = False
        self.fc_mode = "unknown"
        self._state_sub = rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=10
        )

        # ---------- MAVROS 命令服务 ----------
        self._cmd_srv = None

        # ---------- ROS 话题订阅 ----------
        self.cmd_sub = rospy.Subscriber(
            "/servo/cmd", String, self._string_cmd_cb, queue_size=10
        )
        self.open_sub = rospy.Subscriber(
            "/servo/open", Bool, self._bool_cmd_cb, queue_size=10
        )

        # 启动时先复位舵机
        rospy.sleep(0.5)
        self._set_all_servos(False)

        rospy.loginfo(
            "ServoController 就绪 | 前舵机 CH5=%s 后舵机 CH6=%s | PWM_close=%d PWM_open=%d",
            "启用" if self.enable_ch5 else "禁用",
            "启用" if self.enable_ch6 else "禁用",
            self.pwm_close,
            self.pwm_open,
        )
        rospy.loginfo(
            "终端命令: on/off 控制全部 | front open/off 控制前舵机 | rear open/off 控制后舵机"
        )

    # ==================== 飞控连接 ====================

    def _state_cb(self, msg):
        prev = self.fc_connected
        self.fc_connected = msg.connected
        self.fc_armed = msg.armed
        self.fc_mode = msg.mode
        if prev != self.fc_connected:
            if self.fc_connected:
                rospy.loginfo("✅ 飞控已连接 | armed=%s mode=%s", self.fc_armed, self.fc_mode)
            else:
                rospy.logwarn("❌ 飞控连接断开！")

    def _fc_status_str(self):
        if self.fc_connected:
            return "飞控: ✅ 已连接 | armed=%s | mode=%s" % (self.fc_armed, self.fc_mode)
        else:
            return "飞控: ❌ 未连接 —— 检查 MAVROS 和飞控连线"

    def _ensure_service(self):
        """确保 /mavros/cmd/command 服务可用"""
        if self._cmd_srv is not None:
            return True
        try:
            rospy.wait_for_service("/mavros/cmd/command", timeout=3.0)
            self._cmd_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
            return True
        except rospy.ROSException:
            return False

    # ==================== 舵机控制核心 ====================

    def _set_servo(self, state):
        """同时设置前后两个舵机的开关状态。"""
        return self._set_all_servos(state)

    def _set_single_servo(self, servo_num, label, enabled, state):
        """通过 MAV_CMD_DO_SET_SERVO (command=183) 直接设置单个舵机 PWM。"""
        pwm = self.pwm_open if state else self.pwm_close
        action = "打开" if state else "关闭"

        # 检查飞控连接
        if not self.fc_connected:
            rospy.logwarn("⚠️ 飞控未连接！无法控制舵机")
            rospy.logwarn("   请检查: 1) MAVROS 是否运行  2) 飞控 USB 是否插好")
            rospy.logwarn("   验证: rostopic echo /mavros/state")
            return False

        if not enabled:
            rospy.logwarn("⚠️ %s(CH%d) 已禁用，跳过本次 %s 指令", label, servo_num, action)
            return True

        # 检查命令服务
        if not self._ensure_service():
            rospy.logerr("❌ /mavros/cmd/command 服务不可用，MAVROS 是否正常运行？")
            return False

        ok = self._call_set_servo(servo_num, pwm)
        if ok:
            with self.lock:
                self.servo_states[servo_num] = state
                self.servo_open = any(self.servo_states.values())
            rospy.loginfo(
                "舵机 %s(CH%d) → %s (PWM=%d) | %s",
                label, servo_num, action, pwm, self._fc_status_str()
            )
        else:
            rospy.logerr(
                "舵机 %s(CH%d) → %s 失败 | %s",
                label, servo_num, action, self._fc_status_str()
            )
        return ok

    def _set_all_servos(self, state):
        ok = True
        ok = self._set_single_servo(
            self.front_servo_channel, "前舵机", self.enable_ch5, state
        ) and ok
        ok = self._set_single_servo(
            self.rear_servo_channel, "后舵机", self.enable_ch6, state
        ) and ok
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
                command=183,         # MAV_CMD_DO_SET_SERVO
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
            rospy.logerr("MAV_CMD_DO_SET_SERVO(SERVO%d, %d) 调用失败: %s", servo_num, pwm, e)
            self._cmd_srv = None
            return False

    # ==================== 回调 ====================

    def _string_cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        self._handle_command(cmd)

    def _bool_cmd_cb(self, msg):
        self._set_servo(msg.data)

    def _handle_command(self, cmd):
        cmd = " ".join(cmd.split())
        if cmd in ("on", "open", "1", "true"):
            self._set_all_servos(True)
        elif cmd in ("off", "close", "0", "false"):
            self._set_all_servos(False)
        elif cmd == "toggle":
            with self.lock:
                next_state = not self.servo_open
            self._set_all_servos(next_state)
        elif cmd.startswith(("front ", "ch5 ", "5 ", "a ")):
            self._handle_target_command(
                cmd.split(" ", 1)[1],
                self.front_servo_channel,
                "前舵机",
                self.enable_ch5,
            )
        elif cmd.startswith(("rear ", "ch6 ", "6 ", "b ")):
            self._handle_target_command(
                cmd.split(" ", 1)[1],
                self.rear_servo_channel,
                "后舵机",
                self.enable_ch6,
            )
        else:
            rospy.logwarn(
                "无法识别的指令: '%s' (支持: on/off/open/close/toggle/front open/front off/rear open/rear off)",
                cmd,
            )

    def _handle_target_command(self, action, servo_num, label, enabled):
        if action in ("on", "open", "1", "true"):
            self._set_single_servo(servo_num, label, enabled, True)
        elif action in ("off", "close", "0", "false"):
            self._set_single_servo(servo_num, label, enabled, False)
        elif action == "toggle":
            with self.lock:
                next_state = not self.servo_states[servo_num]
            self._set_single_servo(servo_num, label, enabled, next_state)
        else:
            rospy.logwarn("无法识别的 %s 指令: '%s'", label, action)

    # ==================== 终端交互 ====================

    def _terminal_loop(self):
        rospy.sleep(1.0)  # 等 state 回调更新
        print("")
        print("=" * 55)
        print("  舵机测试终端  —  MAV_CMD_DO_SET_SERVO")
        print("  %s" % self._fc_status_str())
        print("  输入 on/open        → 打开前后舵机 (PWM=%d)" % self.pwm_open)
        print("  输入 off/close      → 关闭前后舵机 (PWM=%d)" % self.pwm_close)
        print("  输入 front open/off → 控制前舵机 (CH5)")
        print("  输入 rear open/off  → 控制后舵机 (CH6)")
        print("  输入 status    → 刷新飞控连接状态")
        print("  输入 q/quit    → 退出")
        print("=" * 55)
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
                if cmd == "status":
                    print("  %s" % self._fc_status_str())
                    continue
                self._handle_command(cmd)
            except (EOFError, KeyboardInterrupt):
                rospy.loginfo("终端输入中断")
                break

    # ==================== 主入口 ====================

    def run(self):
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
