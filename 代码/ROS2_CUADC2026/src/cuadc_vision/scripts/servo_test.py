#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
servo_test.py — CUADC 2026 抛投器舵机测试节点（ROS2 / Humble 译本）
====================================================================
源文件（只读）：
  代码/NUC上面的测试功能包（不可以随便修改,可以复制里面的内容）/
  20260722外场测试版本，自动飞行良好/src/cuadc_vision/scripts/servo_test.py（334 行）

功能（与源一致）：
  1. 通过 MAVROS MAV_CMD_DO_SET_SERVO 直接控制飞控舵机输出（不依赖 SERVOx_FUNCTION）
  2. 终端交互模式：手动输入命令分别控制 A/B 舵机
  3. ROS 话题控制：订阅 /servo/cmd 话题接收控制指令
  4. 实时显示飞控连接状态

舵机配置（前后两个独立舵机）：
  - A 舵机（前）：默认 SERVO9（AUX1）   - B 舵机（后）：默认 SERVO10（AUX2）
  - 通道可用参数覆盖：servo_a_channel / servo_b_channel（MAIN=1-8, AUX=9-16）
  - PWM 1000：关闭            - PWM 1500：打开

飞控参数要求（ArduPilot）：
  - RC_OVERRIDE_TIME > 0（允许外部 MAVLink 指令，默认 3s 即可）
  - SERVO9_FUNCTION / SERVO10_FUNCTION 不能为 0（Disabled 不发 PWM，DO_SET_SERVO 也驱动不了）；
    真机 V6X 已按 #43 设为 RC 直通 59/60，DO_SET_SERVO 属临时覆盖（受 RC_OVERRIDE_TIME 限制）

运行方式：
  ros2 run cuadc_vision servo_test.py
  ros2 run cuadc_vision servo_test.py --ros-args -p enable_ch6:=false

终端命令：
  A on / A off / B on / B off / QDFS / all off / status / q

默认不拉起 MAVROS（本译本要求）
--------------------------------
  `auto_start_mavros` 默认 **false**：节点只等待外部（手动 / demo_up / launch）
  已启动的 MAVROS，不自行 `ros2 launch mavros`。若确实需要自启，显式传
  `-p auto_start_mavros:=true`。

ROS1 → ROS2 对应 + 房规编号
---------------------------
  rospy.init_node("servo_controller") → rclpy.create_node("servo_controller")
                                        （节点名保持 servo_controller：auto_drop_test.py
                                         用 `ros2 node list` 判重/就绪）
  rospy.get_param("~x", d)            → node_param(node, "x", d)（declare_parameter）
  rospy.Subscriber/Publisher          → create_subscription/create_publisher
  rospy.ServiceProxy 同步调用         → create_client + call_async + 轮询 future
                                        （后台 MultiThreadedExecutor 投递响应）
  rospy.sleep / rospy.Rate            → time.sleep（#27：Rate.sleep 在 SIGINT 下挂死）
  rospy.loginfo("fmt", a)             → logger.info("fmt" % (a,))（#8：rclpy 不吃 printf 多参）
  rospy.signal_shutdown("用户退出")    → rclpy.try_shutdown()
  rospy.spin()                        → executor.spin()（后台线程建齐后主线程放行信号）
  latch=True 的 /servo/ready          → QoS TRANSIENT_LOCAL（ROS2 的 latch 等价物）
                                        + 低周期心跳，兼容后加入的 VOLATILE 订阅者
  #28 主线程阻塞点 KI 防护           → main() 外层 except (KeyboardInterrupt,
                                        ExternalShutdownException) 收口；工具类 Ctrl-C → rc=0
  #29 SIGINT 治理                    → 入口 pthread_sigmask 先堵 → rclpy.init 后
                                        signal.signal 夺回 Python 级处理器 → 线程建齐后
                                        主线程 SIG_UNBLOCK；双击 Ctrl-C 走 SIG_DFL 保底
  #23 关停竞态护栏                   → 后台 spin 包 _guarded_spin + 有界 executor.shutdown(3.0)
  #20 新脚本执行位                   → CMakeLists install(PROGRAMS) + chmod +x
  #37 /servo/ready                   → 本译本补发布（源有 latch 发布；旧译本遗漏致
                                        auto_drop/semi_auto 等不到舵机就绪）

接口
----
  订阅：/mavros/state(State)、/servo/cmd(String)、/servo/open(Bool)
  发布：/servo/ready(Bool, TRANSIENT_LOCAL)
  服务：/mavros/cmd/command(CommandLong)

rc 语义：0=正常退出（q / Ctrl-C / 终端 EOF 后停止）；1=未预期异常；2=参数错误。
"""

import select
import signal
import subprocess
import sys
import threading
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandLong
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


# ---------------------------------------------------------------------------
# QoS（房规：MAVROS 导航/状态类 BEST_EFFORT；命令/话题类 RELIABLE；
#      /servo/ready 用 TRANSIENT_LOCAL 复刻 ROS1 latch=True）
# ---------------------------------------------------------------------------
NAV_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
MSG_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
READY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

SERVO_A_CHANNEL = 9   # 默认通道（可被参数 servo_a_channel 覆盖）；#43 真机 AUX1=SERVO9
SERVO_B_CHANNEL = 10  # 默认通道（可被参数 servo_b_channel 覆盖）；#43 真机 AUX2=SERVO10
SERVO_A_LABEL = "A舵机(前)"
SERVO_B_LABEL = "B舵机(后)"
MAV_CMD_DO_SET_SERVO = 183   # MAV_CMD_DO_SET_SERVO
CMD_SERVICE = "/mavros/cmd/command"
SERVICE_CALL_TIMEOUT_S = 3.0


def node_param(node, name, default):
    """等价 rospy.get_param("~"+name, default)，declare-once。"""
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    try:
        return node.get_parameter(name).value
    except Exception:
        return default


def get_bool_param(node, name, default):
    value = node_param(node, name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class ServoController:
    """抛投器舵机控制 —— 使用 MAV_CMD_DO_SET_SERVO。"""

    def __init__(self, node):
        self.node = node

        # ---------- 参数（源 ~enable_ch5 / ~enable_ch6 / ~pwm_open / ~pwm_close）----------
        self.enable_ch5 = get_bool_param(node, "enable_ch5", True)
        self.enable_ch6 = get_bool_param(node, "enable_ch6", True)
        self.pwm_open = int(node_param(node, "pwm_open", 1500))
        self.pwm_close = int(node_param(node, "pwm_close", 1000))
        # 输出通道可配置（#43：真机 V6X AUX1/A2 = SERVO9/10；MAIN1-8=SERVO1-8）。
        self.servo_a_channel = int(node_param(node, "servo_a_channel", SERVO_A_CHANNEL))
        self.servo_b_channel = int(node_param(node, "servo_b_channel", SERVO_B_CHANNEL))

        # ---------- 依赖自启（房规：默认不拉 MAVROS，由操作者/demo_up 手动管理）----------
        self.auto_start_mavros = get_bool_param(node, "auto_start_mavros", False)
        self.mavros_fcu_url = str(node_param(node, "mavros_fcu_url", "/dev/ttyACM0:115200"))
        self.mavros_connection_timeout_s = float(
            node_param(node, "mavros_connection_timeout_s", 30.0))
        # 启动时等待 /mavros/state 首帧的最长时间（真机 state ~1Hz，源固定 sleep 0.5s
        # 会漏掉首帧 → 启动复位被跳过；此处改为有界等待）。
        self.initial_state_wait_s = float(node_param(node, "initial_state_wait_s", 5.0))

        # ---------- 就绪心跳（ROS2 兼容后加入的 VOLATILE 订阅者；<=0 关闭）----------
        self.ready_heartbeat_s = float(node_param(node, "ready_heartbeat_s", 2.0))
        self._ready_state = False

        self._validate_params()

        # 初始状态
        self.servo_states = {
            self.servo_a_channel: False,
            self.servo_b_channel: False,
        }
        self.servo_open = False
        self.lock = threading.Lock()
        self._stopping = False
        self._terminal_thread = None
        self._children = []

        # ---------- 飞控连接状态 ----------
        self.fc_connected = False
        self.fc_armed = False
        self.fc_mode = "unknown"
        self._state_sub = node.create_subscription(
            State, "/mavros/state", self._state_cb, NAV_QOS
        )

        # ---------- MAVROS 命令服务（惰性建立）----------
        self._cmd_srv = None

        # ---------- ROS 话题 ----------
        self.cmd_sub = node.create_subscription(
            String, "/servo/cmd", self._string_cmd_cb, MSG_QOS
        )
        self.open_sub = node.create_subscription(
            Bool, "/servo/open", self._bool_cmd_cb, MSG_QOS
        )
        self.ready_pub = node.create_publisher(Bool, "/servo/ready", READY_QOS)
        self._publish_ready(False)

        self._ready_timer = None
        if self.ready_heartbeat_s > 0.0:
            self._ready_timer = node.create_timer(
                self.ready_heartbeat_s, self._ready_heartbeat_cb
            )

    def _validate_params(self):
        if not 1 <= self.servo_a_channel <= 16:
            raise ValueError("servo_a_channel 应在 1~16（MAIN=1-8, AUX=9-16）")
        if not 1 <= self.servo_b_channel <= 16:
            raise ValueError("servo_b_channel 应在 1~16（MAIN=1-8, AUX=9-16）")
        if not 800 <= self.pwm_close <= 2200:
            raise ValueError("pwm_close 应在 800~2200 μs 之间")
        if not 800 <= self.pwm_open <= 2200:
            raise ValueError("pwm_open 应在 800~2200 μs 之间")
        if self.pwm_close == self.pwm_open:
            raise ValueError("pwm_close 与 pwm_open 不能相同")
        if self.mavros_connection_timeout_s <= 0.0:
            raise ValueError("mavros_connection_timeout_s 必须大于 0")
        if self.initial_state_wait_s < 0.0:
            raise ValueError("initial_state_wait_s 不能为负")

    # ==================== 飞控连接 ====================

    def _state_cb(self, msg):
        prev = self.fc_connected
        self.fc_connected = bool(msg.connected)
        self.fc_armed = bool(msg.armed)
        self.fc_mode = str(msg.mode)
        if prev != self.fc_connected:
            if self.fc_connected:
                self.node.get_logger().info(
                    "✅ 飞控已连接 | armed=%s mode=%s" % (self.fc_armed, self.fc_mode))
            else:
                self.node.get_logger().warn("❌ 飞控连接断开！")

    def _fc_status_str(self):
        if self.fc_connected:
            return "飞控: ✅ 已连接 | armed=%s | mode=%s" % (self.fc_armed, self.fc_mode)
        return "飞控: ❌ 未连接 —— 检查 MAVROS 和飞控连线"

    def _mavros_present(self):
        try:
            if any(name == "/mavros/state"
                   for name, _types in self.node.get_topic_names_and_types()):
                return True
            return any(name.rstrip("/").split("/")[-1] == "mavros"
                       for name in self.node.get_node_names())
        except Exception:
            return False

    def _ensure_service(self):
        """确保 /mavros/cmd/command 服务可用。"""
        if self._cmd_srv is not None:
            return True
        try:
            self._cmd_srv = self.node.create_client(CommandLong, CMD_SERVICE)
            if not self._cmd_srv.wait_for_service(timeout_sec=SERVICE_CALL_TIMEOUT_S):
                self._cmd_srv = None
                return False
            return True
        except Exception as exc:
            self.node.get_logger().error("建立 %s 客户端异常: %s" % (CMD_SERVICE, exc))
            self._cmd_srv = None
            return False

    # ==================== 依赖自启（默认关闭） ====================

    def _spawn(self, command, label):
        self.node.get_logger().warn("自动启动 %s: %s" % (label, " ".join(command)))
        try:
            # 子进程不读取本脚本的交互终端，避免抢走终端输入。
            process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, start_new_session=True
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.node.get_logger().error("启动 %s 失败: %s" % (label, exc))
            return False
        self._children.append((label, process))
        return True

    def start_dependencies(self):
        if self._mavros_present():
            self.node.get_logger().info("MAVROS 已运行，不重复启动")
            return True
        if not self.auto_start_mavros:
            self.node.get_logger().warn(
                "MAVROS 未运行且 auto_start_mavros=false（默认）：按约定由操作者/"
                "demo_up 手动启动；本节点只等待其就绪")
            return True
        if not self._spawn(
            ["ros2", "launch", "mavros", "apm.launch.py",
             "fcu_url:=%s" % self.mavros_fcu_url],
            "MAVROS",
        ):
            return False
        deadline = time.monotonic() + self.mavros_connection_timeout_s
        while rclpy.ok() and self.node.context.ok() and time.monotonic() < deadline:
            if self._mavros_present():
                self.node.get_logger().info("自动启动的 MAVROS 已出现")
                return True
            time.sleep(0.1)
        self.node.get_logger().error("自动启动 MAVROS 后仍未出现 /mavros/state")
        return False

    def _stop_children(self):
        for label, process in reversed(self._children):
            if process.poll() is not None:
                continue
            self.node.get_logger().info("停止本脚本自动启动的 %s" % label)
            try:
                process.terminate()
                process.wait(timeout=5.0)
            except (OSError, subprocess.SubprocessError):
                pass

    # ==================== 舵机控制核心 ====================

    def _set_servo(self, state):
        """同时设置前后两个舵机的开关状态。"""
        return self._set_all_servos(state)

    def _set_single_servo(self, servo_num, label, enabled, state):
        """通过 MAV_CMD_DO_SET_SERVO (command=183) 直接设置单个舵机 PWM。"""
        pwm = self.pwm_open if state else self.pwm_close
        action = "打开" if state else "关闭"

        if not self.fc_connected:
            self.node.get_logger().warn("⚠️ 飞控未连接！无法控制舵机")
            self.node.get_logger().warn(
                "   请检查: 1) MAVROS 是否运行  2) 飞控 USB/网口是否连好")
            self.node.get_logger().warn("   验证: ros2 topic hz /mavros/state")
            return False

        if not enabled:
            self.node.get_logger().warn(
                "⚠️ %s(CH%d) 已禁用，跳过本次 %s 指令" % (label, servo_num, action))
            return True

        if not self._ensure_service():
            self.node.get_logger().error(
                "❌ %s 服务不可用，MAVROS 是否正常运行？" % CMD_SERVICE)
            return False

        ok = self._call_set_servo(servo_num, pwm)
        if ok:
            with self.lock:
                self.servo_states[servo_num] = state
                self.servo_open = any(self.servo_states.values())
            self.node.get_logger().info(
                "舵机 %s(CH%d) → %s (PWM=%d) | %s"
                % (label, servo_num, action, pwm, self._fc_status_str()))
        else:
            self.node.get_logger().error(
                "舵机 %s(CH%d) → %s 失败 | %s"
                % (label, servo_num, action, self._fc_status_str()))
        return ok

    def _set_all_servos(self, state):
        ok = True
        ok = self._set_single_servo(
            self.servo_a_channel, SERVO_A_LABEL, self.enable_ch5, state) and ok
        ok = self._set_single_servo(
            self.servo_b_channel, SERVO_B_LABEL, self.enable_ch6, state) and ok
        return ok

    def _call_set_servo(self, servo_num, pwm):
        """发送 MAV_CMD_DO_SET_SERVO 指令。

        command=183: MAV_CMD_DO_SET_SERVO
        param1=servo_num: 舵机输出号（1-8=MAIN/SERVO1-8，9-16=AUX/SERVO9-16）
        param2=pwm: PWM 微秒值
        """
        try:
            if self._cmd_srv is None:
                self._cmd_srv = self.node.create_client(CommandLong, CMD_SERVICE)
            req = CommandLong.Request()
            req.broadcast = False
            req.command = MAV_CMD_DO_SET_SERVO
            req.confirmation = 0
            req.param1 = float(servo_num)
            req.param2 = float(pwm)
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0
            future = self._cmd_srv.call_async(req)
            deadline = time.monotonic() + SERVICE_CALL_TIMEOUT_S
            while not future.done():
                if time.monotonic() >= deadline:
                    break
                if not (rclpy.ok() and self.node.context.ok()):
                    break
                time.sleep(0.02)
            if future.done():
                resp = future.result()
                return bool(getattr(resp, "success", False))
            self.node.get_logger().error(
                "MAV_CMD_DO_SET_SERVO(SERVO%d, %d) 服务调用超时" % (servo_num, pwm))
            self._cmd_srv = None
            return False
        except Exception as exc:
            self.node.get_logger().error(
                "MAV_CMD_DO_SET_SERVO(SERVO%d, %d) 调用失败: %s" % (servo_num, pwm, exc))
            self._cmd_srv = None
            return False

    # ==================== 回调 ====================

    def _string_cmd_cb(self, msg):
        try:
            cmd = str(msg.data).strip().lower()
            self._handle_command(cmd)
        except Exception as exc:
            self.node.get_logger().error("/servo/cmd 回调异常: %s" % exc)

    def _bool_cmd_cb(self, msg):
        try:
            self._set_servo(bool(msg.data))
        except Exception as exc:
            self.node.get_logger().error("/servo/open 回调异常: %s" % exc)

    def _handle_command(self, cmd):
        cmd = " ".join(cmd.split())
        if cmd in ("qdfs", "all on", "all open", "on", "open", "1", "true"):
            self._set_all_servos(True)
        elif cmd in ("all off", "off", "close", "0", "false"):
            self._set_all_servos(False)
        elif cmd == "toggle":
            with self.lock:
                next_state = not self.servo_open
            self._set_all_servos(next_state)
        elif cmd.startswith(("a ", "ch5 ", "5 ", "front ")):
            self._handle_target_command(
                cmd.split(" ", 1)[1], self.servo_a_channel, SERVO_A_LABEL, self.enable_ch5)
        elif cmd.startswith(("b ", "ch6 ", "6 ", "rear ")):
            self._handle_target_command(
                cmd.split(" ", 1)[1], self.servo_b_channel, SERVO_B_LABEL, self.enable_ch6)
        else:
            self.node.get_logger().warn(
                "无法识别的指令: '%s' (支持: A on/A off/B on/B off/QDFS/all off)" % cmd)

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
            self.node.get_logger().warn("无法识别的 %s 指令: '%s'" % (label, action))

    # ==================== 就绪广播 ====================

    def _publish_ready(self, state):
        self._ready_state = bool(state)
        try:
            self.ready_pub.publish(Bool(data=self._ready_state))
        except Exception:
            pass

    def _ready_heartbeat_cb(self):
        # 心跳只重复"当前就绪值"；TRANSIENT_LOCAL 覆盖本节点启动前即已订阅者，
        # 心跳覆盖本节点启动后才加入的 VOLATILE 订阅者（如 auto_drop_test.py）。
        try:
            self.ready_pub.publish(Bool(data=self._ready_state))
        except Exception:
            pass

    # ==================== 终端交互 ====================

    def _print_banner(self):
        print("")
        print("=" * 55)
        print("  舵机测试终端  —  MAV_CMD_DO_SET_SERVO")
        print("  %s" % self._fc_status_str())
        print("  A on/off   → %s(CH%d)" % (SERVO_A_LABEL, self.servo_a_channel))
        print("  B on/off   → %s(CH%d)" % (SERVO_B_LABEL, self.servo_b_channel))
        print("  QDFS       → 全弹发射 (同时打开)")
        print("  all off    → 全部关闭 (PWM=%d)" % self.pwm_close)
        print("  status     → 刷新状态 | q 退出")
        print("=" * 55)
        print("")

    def _wait_for_initial_fc_state(self):
        """有界等待 /mavros/state 首帧（真机 ~1Hz；源固定 sleep 0.5s 会漏帧）。"""
        if self.fc_connected:
            return True
        deadline = time.monotonic() + self.initial_state_wait_s
        while (rclpy.ok() and self.node.context.ok()
               and time.monotonic() < deadline):
            if self.fc_connected:
                return True
            time.sleep(0.1)
        return self.fc_connected

    def _terminal_loop(self):
        time.sleep(1.0)   # 等 state 回调更新
        self._print_banner()
        while rclpy.ok() and self.node.context.ok() and not self._stopping:
            try:
                sys.stdout.write("ab> ")
                sys.stdout.flush()
            except (OSError, ValueError):
                return
            line = None
            # 单次提示符 → 等待一行输入（轮询以便响应外部关停，不重复刷提示符）。
            while rclpy.ok() and self.node.context.ok() and not self._stopping:
                try:
                    ready, _, _ = select.select([sys.stdin], [], [], 0.2)
                except (OSError, ValueError):
                    self.node.get_logger().warn(
                        "终端输入流不可用，停止交互输入（节点继续运行）")
                    return
                if ready:
                    try:
                        line = sys.stdin.readline()
                    except (OSError, ValueError):
                        return
                    break
            if line is None:
                return
            if line == "":
                self.node.get_logger().info("终端输入中断（EOF）")
                return
            cmd = line.strip().lower()
            if not cmd:
                continue
            if cmd in ("q", "quit", "exit"):
                self.node.get_logger().info("用户退出舵机终端")
                rclpy.try_shutdown()
                return
            if cmd == "status":
                print("  %s" % self._fc_status_str(), flush=True)
                continue
            try:
                self._handle_command(cmd)
            except Exception as exc:
                self.node.get_logger().error("处理终端指令异常: %s" % exc)

    # ==================== 主入口 ====================

    def run(self):
        if not self.start_dependencies():
            return 1
        # 有界等待 /mavros/state 首帧，确保启动复位真能下发（真机 state ~1Hz）。
        if not self._wait_for_initial_fc_state():
            self.node.get_logger().warn(
                "启动后 %.1fs 内未收到 /mavros/state 首帧；启动复位将跳过，请检查 MAVROS 链路"
                % self.initial_state_wait_s)
        reset_ok = self._set_all_servos(False)
        if not reset_ok:
            self.node.get_logger().warn(
                "舵机启动复位未全部成功；节点已完成初始化，但请检查 MAVROS 命令响应")
        self._publish_ready(True)

        self.node.get_logger().info(
            "ServoController 就绪 | A舵机 CH5=%s B舵机 CH6=%s | PWM_close=%d PWM_open=%d"
            % ("启用" if self.enable_ch5 else "禁用",
               "启用" if self.enable_ch6 else "禁用",
               self.pwm_close, self.pwm_open))
        self.node.get_logger().info(
            "终端命令: A on/off | B on/off | QDFS | all off")

        # 终端线程在建齐线程、主线程放行信号之前创建（#29）。
        self._terminal_thread = threading.Thread(target=self._terminal_loop, daemon=True)
        self._terminal_thread.start()
        signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT, signal.SIGTERM})

        # 主线程等待关停（后台 spin 线程驱动回调；主线程不放行 spin 以免与
        # MultiThreadedExecutor 的共享回调生成器冲突）。
        while rclpy.ok() and self.node.context.ok() and not self._stopping:
            time.sleep(0.2)
        return 0


def _guarded_spin(executor, node):
    """后台 spin：关停竞态（#23）与真异常分流。"""
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            import traceback
            node.get_logger().error("spin 线程异常退出:\n%s" % traceback.format_exc())


def _make_signal_handler():
    """#29：夺回 Python 级 SIGINT/SIGTERM 处理器；双击 Ctrl-C 走 SIG_DFL 保底。"""
    state = {"count": 0}

    def handler(signum, frame):
        state["count"] += 1
        if state["count"] >= 2:
            try:
                signal.signal(signum, signal.SIG_DFL)
                signal.raise_signal(signum)
            except Exception:
                pass
            return
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

    return handler


def main(args=None):
    # #29 第一半：入口先堵住 SIGINT/SIGTERM，其后创建的线程继承被掩码。
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM})
    rclpy.init(args=args)
    # #29 第二半：rclpy.init 装好 C 处理器后，用 Python 级处理器夺回控制权。
    signal.signal(signal.SIGINT, _make_signal_handler())
    signal.signal(signal.SIGTERM, _make_signal_handler())

    node = rclpy.create_node("servo_controller")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=_guarded_spin, args=(executor, node), daemon=True
    )
    spin_thread.start()

    exit_code = 2
    controller = None
    try:
        try:
            controller = ServoController(node)
        except ValueError as exc:
            node.get_logger().fatal("参数错误: %s" % exc)
            exit_code = 2
        else:
            exit_code = controller.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        # #28：工具类 Ctrl-C 视为正常关停。
        node.get_logger().warn("收到 Ctrl+C，退出舵机终端")
        exit_code = 0
    except Exception as exc:
        import traceback
        node.get_logger().error("未处理异常: %s\n%s" % (exc, traceback.format_exc()))
        exit_code = 1
    finally:
        if controller is not None:
            controller._stopping = True
            try:
                controller._publish_ready(False)
            except Exception:
                pass
            try:
                controller._stop_children()
            except Exception:
                pass
            if controller._terminal_thread is not None:
                controller._terminal_thread.join(timeout=1.0)
        executor.shutdown(3.0)   # 有界兜底：默认无超时会 block forever
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
