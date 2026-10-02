#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_takeoff_alt.py — one_key_takeoff.py 高度门控参考系排查/量化脚本
======================================================================
目的：定位"终端判定高度 vs 目视/真高不一致（目视 3m、终端 ~2m 就悬停）"
      的根因，判定 1m 级常量偏差来自哪一路高度 / 哪个零点。

被测：cuadc_control/scripts/one_key_takeoff.py（门控 = /mavros/local_position/pose.z）
对照：one_key_takeoff_forward_land.py（门控 = rel_alt + 桥接），可选 --child forward。

测量通道（全部带墙钟时间戳）：
  A. /mavros/local_position/pose.z            —— ENU，EKF 局部原点
  B. /mavros/global_position/rel_alt          —— Float64，相对 Home
  C. /mavros/global_position/global.altitude  —— GPS AMSL（WGS84）
  D. /mavros/global_position/gp_origin.alt    —— 全局原点 AMSL
  E. /mavros/home_position/home.geo.altitude  —— 飞控 Home AMSL
  F. Gazebo 真值（不依赖飞控）：gz model -p 的 iris_demo.z 与 iris_with_ardupilot.z
     （world 系，地面 z≈0；= 用户目视的高度基准）
  （/mavros/altitude.* 已确认被 apm_pluginlists denylist 屏蔽，不在测量面）

流程：
  P1 地面零基准：解锁前采样 ground_settle_s 秒，取各通道地面中位数。
  P2 拉起被测脚本（stdin 喂 Enter），全程采样；
     标注脚本打印"已到达目标高度附近"的时刻 t*（门控触发点）。
  P3 到达/悬停段真高 vs 各通道；算两两差，找出常数偏移所在的那一对。
  P4 输出 CSV + 偏差汇总表 + 判定。

用法：
  source /opt/ros/humble/setup.bash && source ~/ROS2_CUADC2026/install/setup.bash
  python3 ~/ROS2_CUADC2026/scripts/verify_takeoff_alt.py \
      --ros-args -p takeoff_altitude:=3.0

rc：0=已量化且有结论 / 1=数据不足或结论未定 / 2=环境前提缺。
日志：/tmp/opencode/takeoff_alt_run.log、/tmp/opencode/takeoff_alt_samples.csv
"""

import csv
import math
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import HomePosition, State
from mavros_msgs.srv import SetMode
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

HOME = os.path.expanduser("~")
INSTALL = os.path.join(
    HOME,
    "ROS2_CUADC2026/install/cuadc_control/lib/cuadc_control",
)
CHILD_DEFAULT = os.path.join(INSTALL, "one_key_takeoff.py")
CHILD_FORWARD = os.path.join(INSTALL, "one_key_takeoff_forward_land.py")
LOG = "/tmp/opencode/takeoff_alt_run.log"
CSV = "/tmp/opencode/takeoff_alt_samples.csv"


def now_s():
    return time.monotonic()


def wall_s():
    return time.time()


def median(xs):
    if not xs:
        return None
    ys = sorted(xs)
    n = len(ys)
    if n % 2:
        return ys[n // 2]
    return 0.5 * (ys[n // 2 - 1] + ys[n // 2])


def fnum(x, nd=3):
    return "n/a" if x is None else ("%.*f" % (nd, x))


def nearest(samples, t):
    """samples: list of (t, value). 返回距 t 最近的样本值。"""
    if not samples:
        return None
    best = min(samples, key=lambda s: abs(s[0] - t))
    if abs(best[0] - t) > 3.0:
        return None
    return best[1]


def window_values(samples, t0, t1):
    return [v for (t, v) in samples if t0 <= t <= t1 and v is not None]


class Recorder:
    def __init__(self, node):
        self.node = node
        nav = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pose_z = []          # (t, z)
        self.rel_alt = []         # (t, v)
        self.amsl = []            # (t, alt)
        self.gp_origin = []       # (t, alt)
        self.home_alt = []        # (t, alt)
        self.armed = []           # (t, bool)
        self.mode = []            # (t, str)

        node.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            lambda m: self.pose_z.append((now_s(), m.pose.position.z)), nav)
        node.create_subscription(
            Float64, "/mavros/global_position/rel_alt",
            lambda m: self.rel_alt.append((now_s(), float(m.data))), nav)
        node.create_subscription(
            NavSatFix, "/mavros/global_position/global",
            lambda m: self.amsl.append((now_s(), float(m.altitude))), nav)
        node.create_subscription(
            GeoPointStamped, "/mavros/global_position/gp_origin",
            lambda m: self.gp_origin.append((now_s(), float(m.position.altitude))), nav)
        node.create_subscription(
            HomePosition, "/mavros/home_position/home",
            lambda m: self.home_alt.append((now_s(), float(m.geo.altitude))), nav)

        def state_cb(m):
            self.armed.append((now_s(), bool(m.armed)))
            self.mode.append((now_s(), str(m.mode or "")))

        node.create_subscription(State, "/mavros/state", state_cb, nav)


class GzPoller(threading.Thread):
    """用 gz.transport13 订阅 dynamic_pose，采 iris_demo 世界 z（目视真值）。

    注意（verify 前两版踩坑）：
      - `gz model -m iris_demo -p` 低 RTF 下单次 ~5s，采样太慢 → 弃用；
      - `iris_with_ardupilot` 是 iris_demo 内部嵌套空壳（世界 z 恒 0），非真值；
      - gz Python 绑定是 `gz.msgs10`（不是 msgs13），消息 `pose_v_pb2.Pose_V`。
    """

    def __init__(self, stop_evt):
        super().__init__(daemon=True)
        self.stop_evt = stop_evt
        self.demo = []   # (t, z) iris_demo 世界 z
        self.err = None

    def run(self):
        try:
            import gz.transport13 as gz_transport
            from gz.msgs10.pose_v_pb2 import Pose_V
        except Exception as exc:  # 绑定不可用 → 真值通道缺，其余测量照旧
            self.err = repr(exc)
            return
        try:
            node = gz_transport.Node()

            def cb(msg):
                for p in msg.pose:
                    if p.name == "iris_demo":
                        self.demo.append((now_s(), float(p.position.z)))
                        break

            node.subscribe(Pose_V, "/world/cuadc_2026_field/dynamic_pose/info", cb)
            while not self.stop_evt.is_set():
                self.stop_evt.wait(0.5)
        except Exception as exc:
            self.err = repr(exc)


def parse_child_log(path):
    """从被测脚本日志提取关键事件时刻（相对 monotonic 基准）。"""
    ev = {}
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if "已到达目标高度附近" in line and "reached" not in ev:
                    ev["reached"] = line.strip()
                if "起飞指令已发送" in line and "takeoff_cmd" not in ev:
                    ev["takeoff_cmd"] = line.strip()
                if "飞行模式切换至: LOITER" in line and "loiter" not in ev:
                    ev["loiter"] = line.strip()
        return ev
    except OSError:
        return ev


def run_child(child, target, log_path, t_base):
    """拉起被测脚本并喂 Enter；返回 (rc, 事件字典)。事件里带采样基准时刻。"""
    args = [sys.executable, child, "--ros-args",
            "-p", "takeoff_altitude:=%s" % target,
            "-p", "auto_start_mavros:=false",
            "-p", "monitor_after_loiter:=false"]
    logf = open(log_path, "w", encoding="utf-8")
    proc = subprocess.Popen(args, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            stdin=subprocess.PIPE, text=True,
                            bufsize=1, start_new_session=True)
    events = {}
    fed = {"enter": False}

    def reader():
        # 逐字符读：提示符 "…按 Enter…: " 无换行，行式读取会永远拿不到
        # （verify 首版踩坑：90s 未喂 Enter → 子进程自行取消）。
        buf = ""
        while True:
            ch = proc.stdout.read(1)
            if ch == "":
                break
            buf += ch
            if "按 Enter" in buf and not fed["enter"]:
                try:
                    proc.stdin.write("\n")
                    proc.stdin.flush()
                except Exception:
                    pass
                fed["enter"] = True
                events["enter"] = now_s()
            if ch == "\n":
                logf.write(buf)
                logf.flush()
                t = now_s()
                if "起飞指令已发送" in buf and "takeoff_cmd" not in events:
                    events["takeoff_cmd"] = t
                if (("已到达目标高度附近" in buf
                     or "已到达起飞高度" in buf)
                        and "reached" not in events):
                    events["reached"] = t
                if ("飞行模式切换至: LOITER" in buf
                        and "loiter" not in events):
                    events["loiter"] = t
                buf = ""

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    return proc, th, logf, events, fed


def analyze(rec, gz, events, target, log, gate_tol=0.05):
    """核心分析：地面零基准 + 到达点真高 + 两两差。"""
    ground_t0 = 0.0
    ground_t1 = events.get("takeoff_cmd", events.get("enter"))
    if ground_t1 is None:
        ground_t1 = min([t for (t, _) in rec.pose_z], default=0.0) + 1e6
    ground_t1 = ground_t1 - 0.5  # 留出起飞指令前的余量

    # 首帧时刻，作为地面窗起点
    first = min([t for (t, _) in rec.pose_z], default=None)
    if first is None:
        log.append("FAIL: 未收到 /mavros/local_position/pose 样本")
        return None
    ground_t0 = first

    def gz_ground(samples):
        vals = window_values(samples, ground_t0, ground_t1)
        return median(vals)

    def chan_ground(samples):
        vals = window_values(samples, ground_t0, ground_t1)
        return median(vals)

    g = {
        "pose_z": chan_ground(rec.pose_z),
        "rel_alt": chan_ground(rec.rel_alt),
        "amsl": chan_ground(rec.amsl),
        "gp_origin": chan_ground(rec.gp_origin),
        "home_alt": chan_ground(rec.home_alt),
        "demo_z": gz_ground(gz.demo),
    }

    t_star = events.get("reached")
    if t_star is None:
        log.append("FAIL: 未捕获'已到达目标高度附近'事件（门控未触发？）")

    def at(samples, t):
        return nearest(samples, t)

    def h_true(t):
        """真高 = 机体世界 z − 地面 z（优先 iris_demo）。"""
        if t is None or g["demo_z"] is None:
            return None
        z = at(gz.demo, t)
        if z is None:
            return None
        return z - g["demo_z"]

    r = {}
    r["pose_z_star"] = at(rec.pose_z, t_star) if t_star else None
    r["rel_alt_star"] = at(rec.rel_alt, t_star) if t_star else None
    r["amsl_star"] = at(rec.amsl, t_star) if t_star else None
    r["h_true_star"] = h_true(t_star) if t_star else None

    # 悬停段（到达后到脚本退出前 2s）中位数
    t_end = max([t for (t, _) in rec.pose_z], default=None)
    if t_star and t_end and t_end - t_star > 3.0:
        # 到达后短窗（覆盖 one_key_takeoff 的 LOITER 悬停与 forward_land 的
        # "起飞高度悬停"，此时垂直基本恒定）
        h0 = t_star + 0.5
        h1 = min(t_end - 0.5, t_star + 8.0)
        r["pose_z_hold"] = median(window_values(rec.pose_z, h0, h1))
        r["rel_alt_hold"] = median(window_values(rec.rel_alt, h0, h1))
        r["h_true_hold"] = None
        zs = window_values(gz.demo, h0, h1)
        if zs and g["demo_z"] is not None:
            r["h_true_hold"] = median(zs) - g["demo_z"]

    # 两两差（地面 / 到达点 / 悬停）
    def diff(a, b):
        if a is None or b is None:
            return None
        return a - b

    # 判定
    verdict = []
    pose_off = g["pose_z"] if g["pose_z"] is not None else None
    rel_off = g["rel_alt"] if g["rel_alt"] is not None else None
    pair_off = diff(g["pose_z"], g["rel_alt"])

    log.append("== 地面零基准（解锁前）==")
    log.append("  pose.z       = %s m" % fnum(g["pose_z"]))
    log.append("  rel_alt      = %s m" % fnum(g["rel_alt"]))
    log.append("  GPS AMSL     = %s m" % fnum(g["amsl"]))
    log.append("  gp_origin.alt= %s m" % fnum(g["gp_origin"]))
    log.append("  home.geo.alt = %s m" % fnum(g["home_alt"]))
    log.append("  gz iris_demo.z         = %s m（真值/目视基准）" % fnum(g["demo_z"]))

    log.append("== 到达点 t*（脚本打印'已到达(目标/起飞)高度'）==")
    log.append("  pose.z @t*   = %s m（门控量，目标 %.2f）"
               % (fnum(r["pose_z_star"]), target))
    log.append("  rel_alt @t*  = %s m" % fnum(r["rel_alt_star"]))
    log.append("  真高 @t*     = %s m（gz 目视基准）" % fnum(r["h_true_star"]))

    log.append("== 常量差（哪一对不一致）==")
    log.append("  pose.z0 − rel_alt0        = %s m" % fnum(pair_off))
    log.append("  pose.z @t* − rel_alt @t*  = %s m"
               % fnum(diff(r["pose_z_star"], r["rel_alt_star"])))
    if r["h_true_star"] is not None:
        log.append("  pose.z @t* − 真高 @t*     = %s m（门控高估量）"
                   % fnum(diff(r["pose_z_star"], r["h_true_star"])))
        log.append("  rel_alt @t* − 真高 @t*    = %s m"
                   % fnum(diff(r["rel_alt_star"], r["h_true_star"])))
        log.append("  真高 @t* − 目标          = %s m（提前/滞后量）"
                   % fnum(diff(r["h_true_star"], target)))
    log.append("  (GPS AMSL @t* − AMSL0)    = %s m"
               % fnum(diff(r["amsl_star"], g["amsl"])))

    log.append("== 悬停段（到达后）==")
    log.append("  pose.z_hold  = %s m" % fnum(r.get("pose_z_hold")))
    log.append("  rel_alt_hold = %s m" % fnum(r.get("rel_alt_hold")))
    log.append("  真高_hold    = %s m" % fnum(r.get("h_true_hold")))

    # 参考系恒等式：pose.z − rel_alt ≡ home_alt − gp_origin_alt
    ident = None
    if (g["home_alt"] is not None and g["gp_origin"] is not None):
        ident = g["home_alt"] - g["gp_origin"]
        r["ident"] = ident
        log.append("  恒等式 home.alt − gp_origin.alt = %s m"
                   "（应 == pose.z0 − rel_alt0 = %s m）"
                   % (fnum(ident), fnum(pair_off)))
        if pair_off is not None and abs(ident - pair_off) < 0.05:
            log.append("  → 已证实：pose.z(EKF 局部原点) 与 rel_alt(Home) 的恒定差"
                       " = Home 与 EKF 原点的海拔差")

    # 结论判定
    if pose_off is not None and abs(pose_off) >= 0.05:
        verdict.append("pose.z 地面零点 = %s m（gz 真值地面 = %s m）→ pose.z 不是"
                       "'离地高度'，而是'EKF 原点以上高度'；裸用它当门控必然偏 %s m"
                       % (fnum(pose_off), fnum(g["demo_z"]), fnum(pose_off)))
    if (r["h_true_star"] is not None
            and abs(r["h_true_star"] - target) >= gate_tol):
        verdict.append("门控触发时真高 %s ≠ 目标 %.2f（差 %s m，容差 %.2f）→ 门控"
                       "参考系与命令参考系不一致"
                       % (fnum(r["h_true_star"]), target,
                          fnum(r["h_true_star"] - target), gate_tol))
    if pair_off is not None and abs(pair_off) >= 0.05:
        verdict.append("pose.z 与 rel_alt 恒定相差 %s m（= home.alt − gp_origin.alt）"
                       "→ 两路高度零点不同，必须统一" % fnum(pair_off))

    return {"g": g, "r": r, "verdict": verdict, "t_star": t_star,
            "ground": (ground_t0, ground_t1)}


def dump_csv(rec, gz):
    try:
        rows = []
        for t, v in rec.pose_z:
            rows.append((t, "pose_z", v))
        for t, v in rec.rel_alt:
            rows.append((t, "rel_alt", v))
        for t, v in rec.amsl:
            rows.append((t, "amsl", v))
        for t, v in gz.demo:
            rows.append((t, "gz_iris_demo_z", v))
        rows.sort(key=lambda r: r[0])
        with open(CSV, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_monotonic", "channel", "value"])
            w.writerows(rows)
    except OSError:
        pass


def main():
    rclpy.init()
    node = rclpy.create_node("verify_takeoff_alt")
    node.declare_parameter("takeoff_altitude", 3.0)
    node.declare_parameter("ground_settle_s", 8.0)
    node.declare_parameter("child", "takeoff")  # takeoff | forward
    node.declare_parameter("child_timeout_s", 90.0)
    node.declare_parameter("hold_s", 12.0)
    node.declare_parameter("gate_tolerance", 0.05)
    target = float(node.get_parameter("takeoff_altitude").value)
    ground_settle = float(node.get_parameter("ground_settle_s").value)
    hold_s = float(node.get_parameter("hold_s").value)
    gate_tol = float(node.get_parameter("gate_tolerance").value)
    child_kind = str(node.get_parameter("child").value)
    child_timeout = float(node.get_parameter("child_timeout_s").value)
    child = CHILD_FORWARD if child_kind == "forward" else CHILD_DEFAULT

    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    rec = Recorder(node)
    stop_evt = threading.Event()
    gz = GzPoller(stop_evt)
    gz.start()

    log = []
    rc = 2
    proc = None
    logf = None
    try:
        # 等首帧
        t0 = now_s()
        while rclpy.ok() and now_s() - t0 < 30.0:
            if rec.pose_z and rec.rel_alt:
                break
            time.sleep(0.2)
        if not rec.pose_z:
            log.append("FAIL: 30s 内无 /mavros/local_position/pose 流（环境未就绪）")
            raise SystemExit(2)

        # 起步清空：若上一轮留下的飞机仍在空中（LOITER），先 LAND 回地面，
        # 保证每轮从地面零基准开始（同一时刻只跑一个用例）。
        # 注意：/mavros/state 是 RELIABLE 且首帧可能晚于 pose，必须等一条 state
        # 再判 armed（否则误判 False → 跳过归零 → 子进程带着已解锁飞机起飞失败）。
        t_state = now_s()
        while rclpy.ok() and not rec.armed and now_s() - t_state < 15.0:
            time.sleep(0.2)
        armed_now = rec.armed[-1][1] if rec.armed else False
        if armed_now:
            log.append("== P0 检测到飞机已解锁/在空中，先切 LAND 归零 ==")
            try:
                cli = node.create_client(SetMode, "/mavros/set_mode")
                cli.wait_for_service(timeout_sec=10.0)
                req = SetMode.Request()
                req.custom_mode = "LAND"
                fut = cli.call_async(req)
                t_land = now_s()
                while not fut.done() and now_s() - t_land < 10.0:
                    time.sleep(0.1)
                log.append("  LAND 请求: %s"
                           % ("已发送" if fut.done() else "超时"))
            except Exception as exc:
                log.append("  LAND 请求异常: %s" % exc)
            t_wait = now_s()
            while rclpy.ok() and now_s() - t_wait < 120.0:
                if rec.armed and rec.armed[-1][1] is False:
                    break
                time.sleep(0.5)
            log.append("  已回到地面 armed=%s（用时 %.1fs）"
                       % (bool(rec.armed[-1][1]) if rec.armed else None,
                          now_s() - t_wait))
            time.sleep(3.0)

        log.append("== P1 地面零基准采样 %.1fs（解锁前）==" % ground_settle)
        time.sleep(ground_settle)

        log.append("== P2 拉起被测脚本 %s（目标 %.2f m）==" % (os.path.basename(child), target))
        proc, rth, logf, events, fed = run_child(child, target, LOG, now_s())

        deadline = now_s() + child_timeout
        while proc.poll() is None and now_s() < deadline:
            time.sleep(0.2)
        if proc.poll() is None:
            log.append("FAIL: 被测脚本 %.0fs 未退出，kill" % child_timeout)
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            time.sleep(2.0)
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            child_rc = -999
        else:
            child_rc = proc.returncode
        log.append("被测脚本 rc=%s，事件: takeoff_cmd=%s reached=%s loiter=%s"
                   % (child_rc,
                      "有" if "takeoff_cmd" in events else "无",
                      "有" if "reached" in events else "无",
                      "有" if "loiter" in events else "无"))

        time.sleep(hold_s)  # 收尾多采悬停段（脚本退出后飞机仍在 LOITER 保持）
        res = analyze(rec, gz, events, target, log, gate_tol=gate_tol)
        dump_csv(rec, gz)

        if child_rc not in (0, None):
            log.append("FAIL: 被测脚本 rc=%s（非正常退出）" % child_rc)

        # 收尾：把飞机落回地面，避免悬停残留影响下一用例
        if rec.armed and rec.armed[-1][1]:
            try:
                cli2 = node.create_client(SetMode, "/mavros/set_mode")
                cli2.wait_for_service(timeout_sec=5.0)
                rq = SetMode.Request()
                rq.custom_mode = "LAND"
                cli2.call_async(rq)
                t_l = now_s()
                while rclpy.ok() and now_s() - t_l < 120.0:
                    if rec.armed and rec.armed[-1][1] is False:
                        break
                    time.sleep(0.5)
                log.append("收尾 LAND：armed=%s"
                           % (bool(rec.armed[-1][1]) if rec.armed else None))
            except Exception as exc:
                log.append("收尾 LAND 异常: %s" % exc)

        if res is None:
            rc = 1
        else:
            rc = 0 if res["verdict"] else 1
            log.append("")
            log.append("== 结论 ==")
            if res["verdict"]:
                for v in res["verdict"]:
                    log.append("  * " + v)
            else:
                log.append("  未发现 ≥0.3m 的常量偏移（可能需要更长悬停窗/更高 RTF）")
    except SystemExit as e:
        rc = int(e.code) if e.code else 1
    finally:
        stop_evt.set()
        if proc is not None:
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        if logf is not None:
            logf.close()
        for line in log:
            print(line)
            sys.stdout.flush()
        print("CSV: %s" % CSV)
        ex.shutdown(3.0)
        th.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
