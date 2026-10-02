#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fc_apply_nav_streams.py — 飞控导航速度 + MAVLink 定位流率配置工具（CUADC 2026 ROS2）
=====================================================================================

用途
----
按需（默认仿真先用）向飞控写入两类参数，并把 MAVROS 链路的消息间隔同步到位，
使 detector_node「反求地面桶坐标」所需的定位话题频率达标：

  1) 导航模式最大平飞速度：`WP_SPD`（ArduCopter 4.7 SI 名，m/s；旧名 WPNAV_SPEED cm/s）。
  2) MAVROS 链路（SERIAL0=5760，MAV1）定位流率：`MAV1_POSITION` / `MAV1_EXTRA1` /
     `MAV1_EXT_STAT`（Hz）。这些参数 `@RebootRequired`，写后需重启飞控才生效；
     本工具可同时通过 `/mavros/cmd/command` 的 `SET_MESSAGE_INTERVAL` 立即生效
     （request_streams 同款机制，per-channel、即时、可逆）。

detector_node 内置要求（墙钟 / 实据 src/cuadc_vision/scripts/detector_node.py）
  ID32 LOCAL_POSITION_NED  → /mavros/local_position/pose    ≥ 10 Hz
  ID33 GLOBAL_POSITION_INT → /mavros/global_position/global ≥  5 Hz
  ID24 GPS_RAW_INT         → /mavros/gpsstatus/gps1/raw     ≥  2 Hz
  ID31 ATTITUDE_QUATERNION → /mavros/imu/data               ≥ 10 Hz
  流组归属：POSITION={ID32,ID33}  EXT_STAT={ID24}  EXTRA1={ID30;ID31 需显式请求}

⚠ 仿真 RTF<1 的说明（错题本 #31）
  SITL 消息按 sim-time 调度 → 墙钟频率 = 配置频率 × GazeboRTF。仿真 RTF≈0.2~0.3，
  故按“要求值”配置时墙钟仍不到 10Hz。`compensate:=true` 会读 Gazebo RTF 并把目标
  频率放大 1/RTF（上限 50Hz）后再配置，使**仿真墙钟**达标；真机 RTF=1，用默认
  （不补偿）的 10/10/5 即可。

真机写动作
----------
  非本机 fcu_url 默认拒绝；真机执行须用户单独确认后加 `-p allow_fc_write:=true`。
  写前自动把原值备份到 `<out_dir>/fc_nav_streams_backup_<日期>_<时刻>.csv`。

用法
----
  source /opt/ros/humble/setup.bash && source ~/ROS2_CUADC2026/install/setup.bash

  # 仿真：最大平飞 3 m/s + 定位流率按 detector 要求（10/10/5）并立即生效
  python3 scripts/fc_apply_nav_streams.py --ros-args \
      -p fcu_url:=tcp://127.0.0.1:5763 -p wp_spd:=3.0

  # 仿真：补偿 RTF，使墙钟频率也达标
  python3 scripts/fc_apply_nav_streams.py --ros-args \
      -p fcu_url:=tcp://127.0.0.1:5763 -p wp_spd:=3.0 -p compensate:=true

  # 实机（用户确认后）：网口 + 显式放行
  python3 scripts/fc_apply_nav_streams.py --ros-args \
      -p fcu_url:=udp://192.168.144.10:14550 -p wp_spd:=3.0 -p allow_fc_write:=true

参数（一律 `--ros-args -p k:=v`）
  fcu_url                  MAVLink 连接串（tcp:// / udp:// / udpin:// / /dev/xxx:baud）
  wp_spd                   导航模式最大平飞速度 m/s（默认 3.0）
  position_hz              MAV1_POSITION / ID32+ID33 目标 Hz（默认 10）
  extra1_hz                MAV1_EXTRA1 / ID31 目标 Hz（默认 10）
  ext_stat_hz              MAV1_EXT_STAT / ID24 目标 Hz（默认 5）
  compensate               读 Gazebo RTF 并把目标 Hz ×1/RTF（上限 50）后再配置（默认 false）
  apply_mavros_intervals   同步用 SET_MESSAGE_INTERVAL 立即生效（默认 true）
  mav_instance             MAVLink 通道实例号（1-based，参数前缀 MAV<n>_；
                           仿真 MAVROS 走 SERIAL0=MAV1；实机 CUAV V6X 以太网 NET_P1=MAV6）
  intervals_via            SET_MESSAGE_INTERVAL 下发路径：mavros（默认，走 /mavros/cmd/command，
                           适合仿真）/ direct（走本工具直连的 MAVLink，适合实机）/ none
  reboot                   写完后发送飞控重启命令使 MAVn_* 生效（默认 false；也可留给下次 SITL 启动）
  allow_fc_write           放行非本机 fcu_url 的写动作（默认 false）
  gz_stats_topic           Gazebo 统计话题（读 RTF；默认 /world/cuadc_2026_field/stats）
  out_dir                  备份/日志目录（默认 ~/ROS2_CUADC2026/logs）

房规：#26（--ros-args -p）、#28/#29（SIGINT 两半，收尾 executor.shutdown(3.0)）、
      #15（按 pid 精确杀，本工具不杀进程）、#16c（不读 /mavros/state）。

退出码：0=成功；1=部分步骤失败；2=连接/放行校验失败。
"""

import csv
import os
import re
import signal
import subprocess
import sys
import threading
import time

try:
    from pymavlink import mavutil
except Exception as exc:  # pragma: no cover
    print("FATAL: 需要 pymavlink: %s" % exc)
    sys.exit(2)

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    try:
        from mavros_msgs.srv import CommandLong
        _HAS_MAVROS = True
    except Exception:
        CommandLong = None
        _HAS_MAVROS = False
except Exception as exc:  # pragma: no cover
    print("FATAL: 需要 ROS2 Humble（先 source setup.bash 两行）: %s" % exc)
    sys.exit(2)

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246

# 需要写/备份的飞控参数（name -> 说明）
TARGET_FC_PARAMS = [
    "WP_SPD", "WPNAV_SPEED",
    "MAV1_POSITION", "MAV1_EXTRA1", "MAV1_EXT_STAT",
    "SR0_POSITION", "SR0_EXTRA1", "SR0_EXT_STAT",
]
# MAVROS 链路消息间隔：msg_id -> (哪个目标 Hz 字段, 名称)
MAVROS_INTERVALS = [
    (32, "position_hz", "LOCAL_POSITION_NED"),
    (33, "position_hz", "GLOBAL_POSITION_INT"),
    (31, "extra1_hz", "ATTITUDE_QUATERNION"),
    (24, "ext_stat_hz", "GPS_RAW_INT"),
]


def parse_fcu_url(url):
    s = (url or "").strip()
    if not s:
        return "tcp:127.0.0.1:5763", None
    if s.startswith("tcp://"):
        return "tcp:" + s[len("tcp://"):], None
    if s.startswith("udpin://"):
        return "udpin:" + s[len("udpin://"):], None
    if s.startswith("udpout://"):
        return "udpout:" + s[len("udpout://"):], None
    if s.startswith("udp://"):
        return "udpout:" + s[len("udp://"):], None
    if s.startswith("serial://"):
        rest = s[len("serial://"):]
        dev, sep, baud = rest.rpartition(":")
        if sep and dev and baud.isdigit():
            return dev, int(baud)
        return rest, None
    if s.startswith("/dev/"):
        dev, sep, baud = s.rpartition(":")
        if sep and dev.startswith("/dev/") and baud.isdigit():
            return dev, int(baud)
        return s, None
    return s, None


def is_local_url(url):
    s = (url or "").strip().lower()
    return any(h in s for h in ("127.0.0.1", "localhost", "::1", "0.0.0.0"))


def read_gz_rtf(topic):
    try:
        env = dict(os.environ)
        env.setdefault("GZ_VERSION", "harmonic")
        r = subprocess.run(["timeout", "4", "gz", "topic", "-e", "-t", topic],
                           capture_output=True, text=True, timeout=8, env=env)
        vals = re.findall(r"real_time_factor:\s*([0-9.]+)", r.stdout or "")
        if vals:
            return float(vals[-1])
    except Exception:
        pass
    return None


def _fmt(v):
    if v is None:
        return "N/A"
    if isinstance(v, float):
        return ("%g" % v)
    return str(v)


def main():
    # #29 第一半：入口堵 SIGINT/SIGTERM
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM})
    rclpy.init()

    def _h(signum, frame):  # #29 第二半：夺回 Python 级处理器 + #28 收口
        if rclpy.ok():
            rclpy.try_shutdown()
            raise KeyboardInterrupt
        else:
            signal.signal(signum, signal.SIG_DFL)
            signal.raise_signal(signum)

    signal.signal(signal.SIGINT, _h)
    signal.signal(signal.SIGTERM, _h)

    node = rclpy.create_node("fc_apply_nav_streams")
    g = node.declare_parameter
    g("fcu_url", "tcp://127.0.0.1:5763")
    g("wp_spd", 3.0)
    g("position_hz", 10.0)
    g("extra1_hz", 10.0)
    g("ext_stat_hz", 5.0)
    g("compensate", False)
    g("apply_mavros_intervals", True)
    g("mav_instance", 1)
    g("intervals_via", "mavros")
    g("reboot", False)
    g("allow_fc_write", False)
    g("gz_stats_topic", "/world/cuadc_2026_field/stats")
    g("out_dir", "~/ROS2_CUADC2026/logs")
    fcu_url = node.get_parameter("fcu_url").value
    wp_spd = float(node.get_parameter("wp_spd").value)
    position_hz = float(node.get_parameter("position_hz").value)
    extra1_hz = float(node.get_parameter("extra1_hz").value)
    ext_stat_hz = float(node.get_parameter("ext_stat_hz").value)
    compensate = bool(node.get_parameter("compensate").value)
    apply_intervals = bool(node.get_parameter("apply_mavros_intervals").value)
    mav_instance = int(node.get_parameter("mav_instance").value)
    intervals_via = str(node.get_parameter("intervals_via").value)
    reboot = bool(node.get_parameter("reboot").value)
    allow_fc_write = bool(node.get_parameter("allow_fc_write").value)
    gz_stats_topic = str(node.get_parameter("gz_stats_topic").value)
    out_dir = os.path.expanduser(str(node.get_parameter("out_dir").value))

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        except Exception:
            if rclpy.ok():
                raise

    th = threading.Thread(target=_spin, daemon=True)
    th.start()
    signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT, signal.SIGTERM})

    rc = 2
    link = None
    try:
        rc = _run(node, fcu_url, wp_spd, position_hz, extra1_hz, ext_stat_hz,
                  compensate, apply_intervals, reboot, allow_fc_write,
                  gz_stats_topic, out_dir, mav_instance, intervals_via)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n[apply] 收到中断信号，退出")
        rc = 1
    except Exception:
        if rclpy.ok():
            raise
    finally:
        if link is not None:
            link.close()
        executor.shutdown(3.0)
        th.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


def _run(node, fcu_url, wp_spd, position_hz, extra1_hz, ext_stat_hz,
         compensate, apply_intervals, reboot, allow_fc_write,
         gz_stats_topic, out_dir, mav_instance=1, intervals_via="mavros"):
    print("=" * 78)
    print("CUADC 飞控 导航速度 + MAVLink 定位流率 配置  @ %s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    print("fcu_url = %s" % fcu_url)
    print("=" * 78)

    if not is_local_url(fcu_url) and not allow_fc_write:
        print("[!] fcu_url 非本机且未 allow_fc_write —— 拒绝写飞控。")
        print("    仿真请用本机 URL；真机须用户确认后加 -p allow_fc_write:=true")
        return 2

    # MAVLink 通道实例 → 参数前缀（1-based）。仿真 MAVROS 走 SERIAL0=MAV1；
    # 实机 CUAV V6X 以太网 NET_P1 是第 6 个 MAVLink 通道 = MAV6（见错题本 #31 补注）。
    mav_names = ("MAV%d_POSITION" % mav_instance,
                 "MAV%d_EXTRA1" % mav_instance,
                 "MAV%d_EXT_STAT" % mav_instance)

    # ---- RTF 补偿 ----
    rtf = read_gz_rtf(gz_stats_topic) if compensate else None
    factor = 1.0
    if compensate:
        if not rtf:
            print("[!] compensate=true 但读不到 Gazebo RTF，退化为不补偿")
        else:
            factor = min(50.0, 1.0 / max(rtf, 0.01))
            print("[rtf] Gazebo RTF=%.3f → 频率补偿 ×%.2f（上限 50）" % (rtf, factor))
    tgt = {
        "position_hz": min(50.0, position_hz * factor),
        "extra1_hz": min(50.0, extra1_hz * factor),
        "ext_stat_hz": min(50.0, ext_stat_hz * factor),
    }
    print("[tgt] FC MAV%d_* 写入：POSITION=%.0fHz EXTRA1=%.0fHz EXT_STAT=%.0fHz"
          % (mav_instance, tgt["position_hz"], tgt["extra1_hz"], tgt["ext_stat_hz"]))
    print("[tgt] WP_SPD = %.2f m/s（导航模式最大平飞速度）" % wp_spd)

    # ---- 连接 + 备份 + 写入 ----
    link = MavLinkWrite(fcu_url)
    if not link.connect(timeout=10.0):
        print("[!] 无法建立 MAVLink 连接")
        return 2

    # 备份：目标参数 + 旧命名（存在才写），并附全量快照
    backup_names = ["WP_SPD", "WPNAV_SPEED",
                    "SR%d_POSITION" % (mav_instance - 1),
                    "SR%d_EXTRA1" % (mav_instance - 1),
                    "SR%d_EXT_STAT" % (mav_instance - 1)] + list(mav_names)
    for n in list(backup_names):
        if n not in link.params:
            v = link.read_param(n)
            if v is None:
                backup_names.remove(n)
    backup = link.backup(backup_names, out_dir)
    print("[backup] 目标参数原值已备份: %s" % backup)

    ok_all = True
    # 1) WP_SPD
    if "WP_SPD" in link.params:
        ok, msg = link.set_param("WP_SPD", wp_spd)
        print("  %s WP_SPD = %.3f m/s  %s" % ("✅" if ok else "❌", wp_spd, msg))
        if not ok:
            ok_all = False
    else:
        print("  ❌ 本固件无 WP_SPD（回退 WPNAV_SPEED? %s）"
              % ("存在" if "WPNAV_SPEED" in link.params else "也无"))
        ok_all = False
    # 2) MAVn_* 流率（n=mav_instance）
    for name, key in zip(mav_names, ("position_hz", "extra1_hz", "ext_stat_hz")):
        if name in link.params:
            ok, msg = link.set_param(name, tgt[key])
            print("  %s %-16s = %3.0f Hz  %s" % ("✅" if ok else "❌", name, tgt[key], msg))
            if not ok:
                ok_all = False
        else:
            print("  ❌ 本固件无 %s" % name)
            ok_all = False
    print("  （MAVn_* 为 @RebootRequired：写后需重启飞控；本工具可 SET_MESSAGE_INTERVAL 立即生效）")

    # 3) 立即生效：SET_MESSAGE_INTERVAL
    if apply_intervals:
        if intervals_via == "none":
            print("[intervals] intervals_via=none，跳过")
        elif intervals_via == "direct":
            print("[direct] 在 MAVLink 直连链路上 SET_MESSAGE_INTERVAL（即时，作用于本 GCS 通道）:")
            for mid, key, nm in MAVROS_INTERVALS:
                ok, msg = link.set_message_interval(mid, tgt[key])
                print("  %s ID%-3d %-22s @ %3.0f Hz %s"
                      % ("✅" if ok else "❌", mid, nm, tgt[key], msg))
                if not ok:
                    ok_all = False
        else:  # mavros
            if not _HAS_MAVROS:
                print("[!] mavros_msgs 不可用，跳过 SET_MESSAGE_INTERVAL")
            else:
                print("[mavros] 设置 /mavros 链路消息间隔（立即生效，可逆）:")
                for mid, key, nm in MAVROS_INTERVALS:
                    ok, msg = _mavros_set_interval(node, mid, tgt[key])
                    print("  %s ID%-3d %-22s @ %3.0f Hz %s"
                          % ("✅" if ok else "❌", mid, nm, tgt[key], msg))
                    if not ok:
                        ok_all = False

    # 4) 可选重启飞控使 MAVn_* 生效
    if reboot:
        print("[reboot] 发送飞控重启命令（MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN）...")
        if link.reboot():
            print("  ✅ 重启命令已发送（等待重连）")
        else:
            print("  ❌ 重启命令发送失败")
            ok_all = False

    # 5) 读回校验
    print("[verify] 读回:")
    for name, want in (("WP_SPD", wp_spd),
                       (mav_names[0], tgt["position_hz"]),
                       (mav_names[1], tgt["extra1_hz"]),
                       (mav_names[2], tgt["ext_stat_hz"])):
        v = link.read_param(name)
        mark = "✅" if (v is not None and abs(v - want) < max(0.01, want * 0.02)) else "❌"
        print("  %s %-16s = %s（期望 %s）" % (mark, name, _fmt(v), _fmt(want)))
        if mark == "❌":
            ok_all = False

    print("\n[done] %s" % ("全部写入并校验通过 ✅" if ok_all else "存在失败项 ❌"))
    print("  真机（RTF=1，无需补偿）：WP_SPD 按需；%s=%.0f、%s=%.0f、%s=%.0f"
          "（改后重启 FC 生效）。" % (mav_names[0], tgt["position_hz"],
                                     mav_names[1], tgt["extra1_hz"],
                                     mav_names[2], tgt["ext_stat_hz"]))
    return 0 if ok_all else 1


class MavLinkWrite:
    """同步式 MAVLink 写客户端：读参数用请求-应答，写参数后读回。"""

    def __init__(self, url):
        self.url = url
        device, baud = parse_fcu_url(url)
        self.device = device
        self.baud = baud
        self.master = None
        self.params = {}

    def connect(self, timeout=10.0):
        kwargs = dict(source_system=250, source_component=191)
        if self.baud:
            self.master = mavutil.mavlink_connection(self.device, baud=self.baud, **kwargs)
        else:
            self.master = mavutil.mavlink_connection(self.device, **kwargs)
        # 必须主动收/发：mavutil 在收到 HEARTBEAT 后才会设置 target_system/component；
        # UDP server（实机 NET_P1）还需我们先发心跳以登记回程地址。
        got_hb = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                pass
            try:
                msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.6)
            except Exception:
                msg = None
            if msg is not None:
                got_hb = True
                break
        if not got_hb or not self.master.target_system:
            return False
        print("[link] HEARTBEAT OK sys=%d comp=%d"
              % (self.master.target_system, self.master.target_component))
        self.params = self.pull_all(timeout=45.0)
        print("[link] 参数总数=%d" % len(self.params))
        return True

    def close(self):
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass

    def pull_all(self, timeout=45.0):
        # 只请求一次：每次 PARAM_REQUEST_LIST 都会让飞控从头重发；
        # 频繁重发会把接收窗口永远卡在前缀（实机 UDP 实测只拿到 0..249）。
        self.master.mav.param_request_list_send(
            self.master.target_system, self.master.target_component)
        params = {}
        t0 = time.monotonic()
        last_n = 0
        last_p = t0
        while time.monotonic() - t0 < timeout and rclpy.ok():
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
            if msg:
                params[msg.param_id] = (float(msg.param_value), int(msg.param_type))
            if len(params) > last_n:
                last_n = len(params)
                last_p = time.monotonic()
            elif params and (time.monotonic() - last_p) > 8.0:
                break
        return params

    def read_param(self, name):
        try:
            self.master.mav.param_request_read_send(
                self.master.target_system, self.master.target_component,
                name.encode(), -1)
        except Exception:
            return None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
            if msg and msg.param_id == name:
                self.params[name] = (float(msg.param_value), int(msg.param_type))
                return float(msg.param_value)
        return None

    def set_param(self, name, value):
        if name not in self.params:
            return False, "参数不存在"
        ptype = self.params[name][1]
        for _ in range(3):
            try:
                self.master.param_set_send(name, float(value), ptype)
            except Exception as exc:
                return False, str(exc)
            time.sleep(0.3)
            v = self.read_param(name)
            if v is not None and abs(v - value) < max(0.01, abs(value) * 0.02):
                return True, ""
        return False, "读回不一致"

    def set_message_interval(self, msg_id, rate_hz):
        """在直连 MAVLink 链路上设置消息间隔（即时；作用于本 GCS 通道）。"""
        us = 0 if rate_hz <= 0 else int(1e6 / rate_hz)
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                float(msg_id), float(us), 0, 0, 0, 0, 0)
        except Exception as exc:
            return False, str(exc)
        time.sleep(0.25)
        return True, ""

    def backup(self, names, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, "fc_nav_streams_backup_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["name", "value", "type"])
            for n in names:
                if n in self.params:
                    v, t = self.params[n]
                    w.writerow([n, v, t])
        return path

    def reboot(self):
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        except Exception:
            return False
        # 等重连
        t0 = time.monotonic()
        while time.monotonic() - t0 < 30.0 and rclpy.ok():
            try:
                msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            except Exception:
                msg = None
            if msg and time.monotonic() - t0 > 3.0:
                return True
        return True


def _mavros_set_interval(node, msg_id, rate_hz, timeout=4.0):
    if not _HAS_MAVROS:
        return False, "mavros_msgs 不可用"
    cli = node.create_client(CommandLong, "/mavros/cmd/command")
    try:
        if not cli.wait_for_service(timeout_sec=3.0):
            return False, "/mavros/cmd/command 不可用"
        req = CommandLong.Request()
        req.broadcast = False
        req.command = MAV_CMD_SET_MESSAGE_INTERVAL
        req.param1 = float(msg_id)
        req.param2 = float(1e6 / rate_hz) if rate_hz > 0 else 0.0
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while rclpy.ok() and not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        if fut.done() and fut.result() is not None:
            return bool(fut.result().success), ""
        return False, "调用超时"
    finally:
        try:
            node.destroy_client(cli)
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
