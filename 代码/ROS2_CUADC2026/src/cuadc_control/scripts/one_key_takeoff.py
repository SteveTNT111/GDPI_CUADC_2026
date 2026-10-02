#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
one_key_takeoff.py — 一键起飞并切换到 LOITER 悬停（ROS2/Humble 版）
====================================================================
由 ROS1 版 `代码/CZY/比赛main工作版本/src/cuadc_control/scripts/one_key_takeoff.py`
逐行翻译而来。

流程：
  1. 检查 / 自动启动 MAVROS（可关闭）
  2. 等待飞控连接与本地位姿可用
  3. 切换到 GUIDED
  4. 解锁
  5. 记录地面 rel_alt 零点 → 发送起飞到 target_rel_alt（默认离地 3.0 m）
  6. 到达目标高度后切换到 LOITER

高度门控参考系（2026-09-29 修复，错题 #40）：
  原版门控用 `/mavros/local_position/pose.z`（EKF 局部原点以上高度），
  与 MAV_CMD_NAV_TAKEOFF 的参考系（相对 Home）不一致；两者恒定相差
  `home.alt − gp_origin.alt`（仿真 ~0.19m、真机可达 ~1m）→ 门控提前触发，
  飞机比目标低 `offset` 悬停（= "目视/终端高度不一致"）。
  现统一：门控与命令都用 `rel_alt`（相对 Home），并在地面记录
  `ground_rel_alt` 作为零点基准（与 one_key_takeoff_forward_land.py /
  one_key_takeoff_wgs84_forward_rtl.py 同款）。

ROS1 -> ROS2 翻译要点：
  - `rospy.init_node` -> `rclpy.init() + create_node`；类构造改为显式传入 rclpy Node。
  - 私有参数 `rospy.get_param("~x", d)` -> `declare_parameter/get_parameter`（参数名去掉 `~`，
    运行时用 `--ros-args -p x:=v` 覆盖）。
  - `rospy.Subscriber(queue_size=)` -> `node.create_subscription`（MAVROS 导航话题为
    BEST_EFFORT，统一用 BEST_EFFORT QoS；/mavros/state 为 RELIABLE，BEST_EFFORT 订阅兼容）。
  - `rospy.ServiceProxy 同步调用` -> `create_client + call_async` + 轮询 future
    （本节点由后台 MultiThreadedExecutor spin，轮询避免与 spin() 冲突丢回调）。
  - `rospy.Rate` -> `node.create_rate`；`rospy.is_shutdown()` -> `rclpy.ok()`；
    `rospy.sleep` -> `time.sleep`；`rospy.log*` -> `node.get_logger().*`
    （节流日志用本地 `throttle_log`）。
  - ROS1 自动启动 roscore 的逻辑删除（ROS2 无 master）；MAVROS 未运行且
    auto_start_mavros=true 时自动 `ros2 launch mavros apm.launch fcu_url:=...`。

运行方式：
  ros2 run cuadc_control one_key_takeoff.py
  ros2 run cuadc_control one_key_takeoff.py --ros-args -p takeoff_altitude:=3.0
"""

import math
import os
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


class OneKeyTakeoffNode:
    """执行一键起飞并切换 LOITER 的控制节点"""

    def __init__(self, node):
        self.node = node
        self.takeoff_altitude = float(node_param(node, "takeoff_altitude", 3.0))
        self.hover_mode = node_param(node, "hover_mode", "LOITER")
        self.auto_start_mavros = bool(node_param(node, "auto_start_mavros", False))
        self.mavros_fcu_url = node_param(node, "mavros_fcu_url", "/dev/ttyACM0:115200")
        # ROS2 的图发现是异步的：新建节点后 count_publishers("/mavros/state")
        # 可能短暂返回 0。给一个宽限窗口，避免把"已运行的 MAVROS"误判为未运行
        # 而重复启动（ROS1 的 get_published_topics 经 master 同步，无此问题）。
        self.mavros_discovery_grace = float(
            node_param(node, "mavros_discovery_grace", 3.0)
        )
        self.connection_timeout = float(node_param(node, "connection_timeout", 30.0))
        self.service_timeout = float(node_param(node, "service_timeout", 10.0))
        self.ekf_timeout = float(node_param(node, "ekf_timeout", 30.0))
        self.takeoff_timeout = float(node_param(node, "takeoff_timeout", 30.0))
        self.altitude_reach_ratio = max(
            1.0, float(node_param(node, "altitude_reach_ratio", 1.0))
        )
        self.mode_settle_time = float(node_param(node, "mode_settle_time", 1.5))
        self.arm_settle_time = float(node_param(node, "arm_settle_time", 1.5))
        self.monitor_only = bool(node_param(node, "monitor_after_loiter", True))

        self.current_state = State()
        self.current_pose = PoseStamped()
        # 门控高度统一用 rel_alt（相对 Home，= MAV_CMD_NAV_TAKEOFF 的参考系）。
        # pose.z 是 "EKF 局部原点以上高度"，其地面零点非 0（= home_alt −
        # gp_origin_alt，仿真 ~0.19m、真机可达 ~1m），裸用会提前触发门控。
        self.relative_altitude = None
        self._relative_altitude_received = False
        self.ground_rel_alt = None
        self.target_rel_alt = None

        self._set_mode_srv = None
        self._arming_srv = None
        self._takeoff_srv = None

        # MAVROS 传感器类话题（local_position/pose）以 BEST_EFFORT 发布；
        # 统一用 BEST_EFFORT + depth 10：BEST_EFFORT 订阅对 RELIABLE 与
        # BEST_EFFORT 发布方均兼容（/mavros/state 为 RELIABLE）。
        nav_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.state_sub = self.node.create_subscription(
            State, "/mavros/state", self._state_cb, nav_qos
        )
        self.local_pose_sub = self.node.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_cb, nav_qos
        )
        self.relative_altitude_sub = self.node.create_subscription(
            Float64, "/mavros/global_position/rel_alt",
            self._relative_altitude_cb, nav_qos,
        )

        # 可选：外部 MultiThreadedExecutor（后台 spin 时使用），避免内部
        # spin_until_future_complete 与已有 spin 线程冲突。
        self._executor = None

    # ------------------------------------------------------------------
    # 订阅回调（由后台 spin 线程驱动）
    # ------------------------------------------------------------------
    def _state_cb(self, msg):
        self.current_state = msg

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _relative_altitude_cb(self, msg):
        if math.isfinite(msg.data):
            self.relative_altitude = float(msg.data)
            self._relative_altitude_received = True

    # ------------------------------------------------------------------
    # MAVROS 检测 / 自动启动
    # ------------------------------------------------------------------
    def _mavros_is_alive(self):
        """检查 MAVROS 是否在运行（等价 ROS1 的 get_published_topics 检查）"""
        try:
            return self.node.count_publishers("/mavros/state") > 0
        except Exception:
            return False

    def _wait_for_mavros(self, timeout=30.0):
        """等待 MAVROS 上线"""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.node.context.ok():
            if self._mavros_is_alive():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(1.0)
        return False

    def _ensure_mavros(self):
        """确保 MAVROS 在运行"""
        # 先给图发现一个宽限窗口（见 __init__ 中说明），避免误判后重复启动。
        if self._wait_for_mavros(timeout=self.mavros_discovery_grace):
            self.node.get_logger().info("[auto-start] MAVROS 已在运行")
            return True
        if not self.auto_start_mavros:
            self.node.get_logger().error("MAVROS 未运行，且 auto_start_mavros=false")
            return False

        self.node.get_logger().info("[auto-start] MAVROS 未运行，自动启动 mavros apm.launch ...")
        try:
            subprocess.Popen(
                ["ros2", "launch", "mavros", "apm.launch",
                 "fcu_url:={}".format(self.mavros_fcu_url)],
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

        self.node.get_logger().error("[auto-start] MAVROS 启动超时（%.1fs）" % self.connection_timeout)
        return False

    # ------------------------------------------------------------------
    # 等待连接 / 本地位姿
    # ------------------------------------------------------------------
    def _wait_for_connection(self):
        """等待飞控连接建立"""
        self.node.get_logger().info("等待飞控连接 ...")
        deadline = time.monotonic() + self.connection_timeout
        rate = self.node.create_rate(2)
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.connected:
                self.node.get_logger().info("飞控已连接")
                return True
            if time.monotonic() >= deadline:
                self.node.get_logger().error("等待飞控连接超时（%.1fs）" % self.connection_timeout)
                return False
            rate.sleep()
        return False

    def _wait_for_local_pose(self):
        """等待本地位姿可用，作为起飞前的 EKF 就绪条件"""
        self.node.get_logger().info("等待本地位姿 /mavros/local_position/pose ...")
        deadline = time.monotonic() + self.ekf_timeout
        rate = self.node.create_rate(2)
        while rclpy.ok() and self.node.context.ok():
            pos = self.current_pose.pose.position
            has_pose = (
                abs(pos.x) > 0.001 or
                abs(pos.y) > 0.001 or
                abs(pos.z) > 0.001
            )
            if has_pose and self.current_state.connected:
                self.node.get_logger().info(
                    "本地位姿已就绪 (x=%.3f y=%.3f z=%.3f)" % (pos.x, pos.y, pos.z)
                )
                return True
            if time.monotonic() >= deadline:
                self.node.get_logger().error(
                    "等待本地位姿超时（%.1fs），当前 (x=%.3f y=%.3f z=%.3f)"
                    % (self.ekf_timeout, pos.x, pos.y, pos.z)
                )
                return False
            rate.sleep()
        return False

    # ------------------------------------------------------------------
    # 服务
    # ------------------------------------------------------------------
    def _ensure_services(self):
        """连接 MAVROS 所需服务"""
        if self._set_mode_srv is not None:
            return True
        try:
            self._set_mode_srv = self.node.create_client(SetMode, "/mavros/set_mode")
            self._arming_srv = self.node.create_client(CommandBool, "/mavros/cmd/arming")
            self._takeoff_srv = self.node.create_client(CommandTOL, "/mavros/cmd/takeoff")
            if not (self._wait_service(self._set_mode_srv, self.service_timeout)
                    and self._wait_service(self._arming_srv, self.service_timeout)
                    and self._wait_service(self._takeoff_srv, self.service_timeout)):
                self.node.get_logger().error("MAVROS basic services unavailable")
                return False
            self.node.get_logger().info("MAVROS 服务已连接")
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
    # 状态等待
    # ------------------------------------------------------------------
    def _wait_for_mode(self, expected_mode, timeout=5.0):
        """等待模式切换完成"""
        deadline = time.monotonic() + timeout
        rate = self.node.create_rate(5)
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.mode == expected_mode:
                return True
            if time.monotonic() >= deadline:
                return False
            rate.sleep()
        return False

    def _wait_for_arm_state(self, expected_armed, timeout=5.0):
        """等待解锁状态切换完成"""
        deadline = time.monotonic() + timeout
        rate = self.node.create_rate(5)
        while rclpy.ok() and self.node.context.ok():
            if self.current_state.armed == expected_armed:
                return True
            if time.monotonic() >= deadline:
                return False
            rate.sleep()
        return False

    # ------------------------------------------------------------------
    # 飞行动作
    # ------------------------------------------------------------------
    def set_mode(self, mode):
        """切换飞行模式"""
        if not self._ensure_services():
            return False
        req = SetMode.Request()
        req.custom_mode = mode
        ok, resp = self._call_service(self._set_mode_srv, req, self.service_timeout)
        if not ok:
            self.node.get_logger().error("set_mode 服务调用失败: %s" % mode)
            return False

        if not resp.mode_sent:
            self.node.get_logger().error("飞行模式切换请求被拒绝: %s" % mode)
            return False

        if not self._wait_for_mode(mode, timeout=self.mode_settle_time + 3.0):
            self.node.get_logger().error(
                "模式请求已发送，但目标模式未生效: expected=%s current=%s"
                % (mode, self.current_state.mode)
            )
            return False
        self.node.get_logger().info("飞行模式切换至: %s" % mode)
        return True

    def arm(self):
        """解锁无人机"""
        if not self._ensure_services():
            return False
        if self.current_state.armed:
            self.node.get_logger().info("无人机已解锁")
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

        if not self._wait_for_arm_state(True, timeout=self.arm_settle_time + 3.0):
            self.node.get_logger().error("解锁请求已发送，但状态未更新为 armed")
            return False
        self.node.get_logger().info("解锁成功")
        return True

    def _capture_ground_altitude_reference(self):
        """解锁后在地面记录 rel_alt 地面零点，换算实际起飞目标高度。

        门控与起飞命令必须同参考系：rel_alt 相对 Home，MAV_CMD_NAV_TAKEOFF 的
        altitude 也相对 Home。记录 ground_rel_alt 后，命令高度取
        ground_rel_alt + 计划爬升，门控同样比 ground_rel_alt + 计划爬升。
        """
        self.node.get_logger().info("记录起飞地面高度基准（rel_alt）...")
        deadline = time.monotonic() + self.ekf_timeout
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
            "地面高度基准: ground_rel_alt=%.2f m, 计划爬升=%.2f m, target_rel_alt=%.2f m"
            % (self.ground_rel_alt, self.takeoff_altitude, self.target_rel_alt))
        return True

    def takeoff(self):
        """发送起飞指令（目标高度相对 Home，与门控参考系一致）"""
        if not self._ensure_services():
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
            self.node.get_logger().error("起飞指令发送失败")
            return False

        self.node.get_logger().info(
            "起飞指令已发送，目标相对 Home 高度 %.2f m（离启动地面上升 %.2f m）"
            % (self.target_rel_alt, self.takeoff_altitude))
        return True

    def wait_until_altitude_reached(self):
        """等待上升到目标高度（门控用 rel_alt，与起飞命令同参考系）"""
        base = self.ground_rel_alt if self.ground_rel_alt is not None else 0.0
        target = base + self.takeoff_altitude * self.altitude_reach_ratio
        deadline = time.monotonic() + self.takeoff_timeout
        rate = self.node.create_rate(5)
        while rclpy.ok() and self.node.context.ok():
            current_alt = self.relative_altitude
            if current_alt is not None and current_alt >= target:
                self.node.get_logger().info(
                    "已到达目标高度附近: rel_alt=%.2f / %.2f m"
                    % (current_alt, target)
                )
                return True
            if time.monotonic() >= deadline:
                self.node.get_logger().error(
                    "等待起飞高度超时（%.1fs），当前 rel_alt=%s，目标 %.2f m"
                    % (self.takeoff_timeout, current_alt, target)
                )
                return False
            throttle_log(
                self.node, 2.0, "爬升中: rel_alt=%.2f / %.2f m",
                current_alt if current_alt is not None else float("nan"), target,
            )
            rate.sleep()
        return False

    def _request_arm_by_enter(self):
        """满足解锁条件后，等操作者按 Enter 再解锁（房规：脚本自解锁，SITL 无遥控器）。"""
        prompt = "飞控已就绪（GUIDED，未解锁）。按 Enter 执行解锁并起飞（Ctrl+C 取消）: "
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
        time.sleep(self.mode_settle_time)
        if not self._request_arm_by_enter():
            self.node.get_logger().warn("操作者未按 Enter，取消；未解锁、未起飞")
            return 2
        if not self.arm():
            return 1
        time.sleep(self.arm_settle_time)
        if not self._capture_ground_altitude_reference():
            return 1
        if not self.takeoff():
            return 1
        if not self.wait_until_altitude_reached():
            return 1
        time.sleep(1.0)
        if not self.set_mode(self.hover_mode):
            return 1

        self.node.get_logger().info("流程完成：无人机已切换到 %s 模式保持悬停" % self.hover_mode)

        if not self.monitor_only:
            return 0

        rate = self.node.create_rate(1)
        while rclpy.ok() and self.node.context.ok():
            pos = self.current_pose.pose.position
            throttle_log(
                self.node, 5.0,
                "悬停监控中... mode=%s armed=%s rel_alt(真高)=%.2f m z(pose)=%.2f m",
                self.current_state.mode, self.current_state.armed,
                (self.relative_altitude if self.relative_altitude is not None
                 else float("nan")), pos.z,
            )
            rate.sleep()
        return 0


def main():
    rclpy.init()
    node = rclpy.create_node("one_key_takeoff")

    # 后台 spin：订阅回调（/mavros/state、/mavros/local_position/pose）
    # 必须由 executor 驱动，ROS2 的 rate.sleep() 不会处理回调。
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        takeoff_node = OneKeyTakeoffNode(node)
        takeoff_node._executor = executor  # 使用同一执行器，避免内部自建 executor 冲突
        exit_code = takeoff_node.run()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())