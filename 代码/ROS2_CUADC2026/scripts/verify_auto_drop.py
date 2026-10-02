#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_auto_drop.py — E8 (auto_drop_test.py) 验收脚本
=====================================================
对照 verify_d1.py / verify_e2.py 判据风格。被测：cuadc_control/scripts/auto_drop_test.py。

验收（计划书 §2 + 任务书）：
  RUN A flight_only 正例：全航线飞完（GUIDED→…→FLIGHT_ONLY_PATTERN→
        GOTO_RECON_CLIMB→RECON_HOLD→RTL→COMPLETE）、**/servo/cmd 零发布**、
        mission_status 恒 aiming=false/last_drop=""、模式序列合理、rc=0。
  RUN B 负例（无 detector/无目标超时）：完整任务无注入检测、area_search_timeout_s
        调小 → rc=1、**零 /servo/cmd**、进入 FAILSAFE。
  RUN C 参数错误负例：flight_only_search_altitude_m>recon_altitude_m → rc=2。
  RUN D 完整任务正例（ground_test + dry_run + 注入合成 YoloDetections）：
        A_AIM→A_DROP→B_AIM→B_DROP→COMPLETE、rc=0、零 /servo/cmd（dry 不下发舵机）。
  日志：各正例 0 [ERROR]、全部 0 Traceback（RUN B 的 [ERROR] 属预期失败路径）。
  清理：无残留进程。

rc：0=全过 / 1=有 FAIL / 2=环境前提缺。
日志：/tmp/opencode/e8_run{a,b,c}.log
"""

import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, State
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, NavSatFix
from std_msgs.msg import Float64, String

from cuadc_vision.msg import MissionStatus, YoloDetection, YoloDetections

HOME = os.path.expanduser("~")
ADT = os.path.join(HOME, "ROS2_CUADC2026/install/cuadc_control/lib/cuadc_control/auto_drop_test.py")
LOGS = {"a": "/tmp/opencode/e8_runa.log",
        "b": "/tmp/opencode/e8_runb.log",
        "c": "/tmp/opencode/e8_runc.log",
        "d": "/tmp/opencode/e8_rund.log"}

WARN_OK = [
    "FLIGHT ONLY MODE", "vision=DISABLED", "CUADC FLIGHT ONLY START",
    "MISSION TRANSITION", "已回读", "MAVROS 已连接", "MAVROS 首个状态",
    "MAVROS 状态变化", "地面相对高度基准", "起飞爬升中", "已到达起飞高度",
    "已连续到达", "搜索中心已冻结", "区域航线中心已冻结", "开始", "到达点",
    "圈完整完成", "全部完成", "遥测", "RECOVERY", "RECON", "任务完成",
    "操作者已按 Enter", "sync", "遥控", "等待", "Dry", "dry_run", "投票",
]


def main():
    rclpy.init()
    node = rclpy.create_node("verify_auto_drop")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    nav_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    now = time.monotonic
    rec = {"connected": False, "pose": None, "fix": False, "hdg": False,
           "modes": [], "sp_local": [], "sp_global": [], "servo": [], "mission": [],
           "cam": (320.0, 240.0)}

    def state_cb(m):
        rec["connected"] = bool(m.connected)
        rec["modes"].append((now(), m.mode or "", bool(m.armed), float(m.system_status)))

    node.create_subscription(State, "/mavros/state", state_cb, nav_qos)

    def pose_cb(m):
        rec["pose"] = m
    node.create_subscription(PoseStamped, "/mavros/local_position/pose", pose_cb, nav_qos)

    def fix_cb(m):
        if m.status.status >= 0 and abs(m.latitude) < 90:
            rec["fix"] = True
    node.create_subscription(NavSatFix, "/mavros/global_position/global", fix_cb, nav_qos)

    def hdg_cb(m):
        rec["hdg"] = True
    node.create_subscription(Float64, "/mavros/global_position/compass_hdg", hdg_cb, nav_qos)

    def spl_cb(m):
        rec["sp_local"].append((now(), float(m.pose.position.z)))
    node.create_subscription(PoseStamped, "/mavros/setpoint_position/local", spl_cb, 10)

    def spg_cb(m):
        rec["sp_global"].append((now(), float(m.latitude), float(m.longitude),
                                 float(m.altitude)))
    node.create_subscription(GlobalPositionTarget, "/mavros/setpoint_raw/global", spg_cb, 10)

    def servo_cb(m):
        rec["servo"].append((now(), m.data))
    node.create_subscription(String, "/servo/cmd", servo_cb, 10)

    def mission_cb(m):
        rec["mission"].append((now(), bool(m.aiming), m.last_drop))
    node.create_subscription(MissionStatus, "/vision/mission_status", mission_cb, 10)

    def cam_cb(m):
        try:
            if len(m.k) >= 6:
                rec["cam"] = (float(m.k[2]), float(m.k[5]))
        except Exception:
            pass
    node.create_subscription(CameraInfo, "/vision/color/camera_info", cam_cb, 1)

    det_pub = node.create_publisher(YoloDetections, "/vision/yolo/detections", 5)

    def make_det():
        d = YoloDetections()
        d.header.stamp = node.get_clock().now().to_msg()
        x = YoloDetection()
        x.header = d.header
        x.detected = True
        x.class_id = 3
        x.class_name = "cylinder"
        x.confidence = 0.9
        x.center_x = int(rec["cam"][0])
        x.center_y = int(rec["cam"][1])
        x.position_valid = True
        x.camera_x_m, x.camera_y_m, x.camera_z_m = 0.0, 0.0, 2.0
        x.depth_m = 2.0
        d.detections = [x]
        return d

    checks = []
    procs = {}

    def report(code):
        for p in procs.values():
            if p is not None and p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    pass
        print("\n================ E8 验证报告 ================", flush=True)
        all_pass = True
        for name, ok, detail in checks:
            all_pass = all_pass and ok
            print("  [%s] %-56s %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)
        print("\n>>> E8 验证 %s <<<" % ("通过 ✅" if all_pass else "失败 ❌"), flush=True)
        ex.shutdown()
        th.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0 if all_pass else code)

    def read_log(path):
        try:
            with open(path, "r", errors="replace") as f:
                return f.read()
        except OSError:
            return ""

    def transitions(tag):
        out = []
        for m in re.finditer(r"MISSION TRANSITION \| (\w+) -> (\w+) \| reason=([^|]*)",
                             read_log(LOGS[tag])):
            out.append((m.group(1), m.group(2), m.group(3).strip()))
        return out

    def subseq(items, expected):
        it = iter(items)
        return all(any(x == e for x in it) for e in expected)

    def win(t0, t1, seq):
        return [x for x in seq if t0 <= x[0] <= t1]

    def wait_flow(flag, timeout_s):
        t0 = now()
        while now() - t0 < timeout_s:
            if flag():
                return True
            time.sleep(0.2)
        return False

    def set_mode(mode, timeout_s=8.0):
        try:
            r = subprocess.run(
                ["ros2", "service", "call", "/mavros/set_mode",
                 "mavros_msgs/srv/SetMode",
                 "{base_mode: 0, custom_mode: '%s'}" % mode],
                capture_output=True, text=True, timeout=timeout_s)
            return "mode_sent=True" in r.stdout
        except Exception:
            return False

    def ensure_grounded(timeout_s=120.0):
        """确保落地并上锁，避免上一轮残留污染本轮（错峰/串跑必备）。"""
        t0 = now()
        while now() - t0 < timeout_s:
            if rec["pose"] is not None and not any(
                    m[2] for m in win(now() - 3.0, now(), rec["modes"])):
                z = float(rec["pose"].pose.position.z)
                if z < 0.6:
                    return True
            set_mode("RTL")
            time.sleep(3.0)
        return False

    def import_ok():
        r = subprocess.run(
            [sys.executable, "-c",
             "from cuadc_vision.msg import MissionStatus, YoloDetections"],
            capture_output=True)
        return r.returncode == 0

    def run_node(tag, args, timeout_s, feed="\n", inject=False):
        for attempt in range(2):
            t0 = now()
            logf = open(LOGS[tag], "wb")
            p = subprocess.Popen(
                [sys.executable, ADT, "--ros-args"] + args,
                stdin=subprocess.PIPE, stdout=logf, stderr=subprocess.STDOUT,
                start_new_session=True)
            procs[tag] = p
            try:
                p.stdin.write(feed.encode())
                p.stdin.flush()
            except Exception:
                pass
            while p.poll() is None and now() - t0 < timeout_s:
                if inject:
                    det_pub.publish(make_det())
                time.sleep(0.3)
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except Exception:
                    pass
                p.wait(timeout=5)
            # 并发 agent 重建 cuadc_vision 期间 import 会瞬断（ModuleNotFoundError）
            # → 等其重建稳定后重试一次，避免把环境竞态记成被测失败。
            if attempt == 0 and "ModuleNotFoundError" in read_log(LOGS[tag]):
                self_settle = now()
                while now() - self_settle < 90 and not import_ok():
                    time.sleep(2)
                time.sleep(3)
                continue
            time.sleep(3.0)
            return p.returncode, t0, now()
        return p.returncode, t0, now()

    # ---- 1. 环境前提 ----
    nav_ok = wait_flow(
        lambda: rec["connected"] and rec["pose"] is not None
        and rec["fix"] and rec["hdg"], 12)
    checks.append(("环境前提（connected+pose/fix/hdg 流）", nav_ok,
                   "conn=%s pose=%s fix=%s hdg=%s" % (
                       rec["connected"], rec["pose"] is not None, rec["fix"], rec["hdg"])))
    if not nav_ok:
        print("FAIL: 环境前提缺", flush=True)
        report(2)
    grounded = ensure_grounded()
    checks.append(("起跑前已落地/上锁（防串跑污染）", grounded,
                   "armed现在=%s" % any(m[2] for m in win(now() - 3.0, now(), rec["modes"]))))

    # ================= RUN A：flight_only 正例全航线 =================
    rc_a, ta0, ta1 = run_node(
        "a",
        ["-p", "flight_only_mode:=true",
         "-p", "auto_start_detector:=false",
         "-p", "auto_start_servo_test:=false",
         "-p", "drop_zone_distance_m:=10.0",
         "-p", "recon_zone_distance_m:=20.0",
         "-p", "takeoff_altitude_m:=2.5",
         "-p", "flight_only_search_altitude_m:=2.5",
         "-p", "recon_altitude_m:=4.0",
         "-p", "recon_hold_s:=2.0",
         "-p", "drop_zone_width_m:=6.0",
         "-p", "drop_zone_length_m:=4.0",
         "-p", "area_search_dwell_s:=0.3",
         "-p", "area_search_waypoint_tolerance_m:=0.4",
         "-p", "area_search_height_tolerance_m:=0.3",
         "-p", "state_timeout_s:=5.0", "-p", "guided_state_grace_s:=10.0",
         "-p", "pose_timeout_s:=5.0", "-p", "guided_pose_grace_s:=10.0",
         "-p", "navigation_timeout_s:=5.0", "-p", "navigation_grace_s:=10.0",
         "-p", "mode_timeout_s:=20.0"],
        timeout_s=330)
    log_a = read_log(LOGS["a"])
    tr_a = [(o, n) for o, n, _ in transitions("a")]
    exp_a = [("INIT_DEPENDENCIES", "WAIT_MAVROS"), ("WAIT_MAVROS", "WAIT_NAVIGATION"),
             ("WAIT_NAVIGATION", "PLAN_WAYPOINTS"), ("PLAN_WAYPOINTS", "WAIT_ENTER"),
             ("WAIT_ENTER", "GUIDED_ARM_TAKEOFF"), ("GUIDED_ARM_TAKEOFF", "GOTO_DROP_ZONE"),
             ("GOTO_DROP_ZONE", "FLIGHT_ONLY_PATTERN"),
             ("FLIGHT_ONLY_PATTERN", "GOTO_RECON_CLIMB"),
             ("GOTO_RECON_CLIMB", "RECON_HOLD"), ("RECON_HOLD", "RTL"), ("RTL", "COMPLETE")]
    checks.append(("RUN A rc=0（COMPLETE 正常退出）", rc_a == 0, "rc=%s" % rc_a))
    checks.append(("RUN A 全航线状态链完整", subseq(tr_a, exp_a),
                   "链尾=%s" % "→".join("%s→%s" % t for t in tr_a[-6:])))
    checks.append(("RUN A 无 FAILSAFE", not any(n == "FAILSAFE" for _, n in tr_a),
                   "含FAILSAFE=%s" % any(n == "FAILSAFE" for _, n in tr_a)))
    servo_a = win(ta0, ta1, rec["servo"])
    checks.append(("RUN A /servo/cmd 零发布", len(servo_a) == 0,
                   "servo发布=%d" % len(servo_a)))
    miss_a = win(ta0, ta1, rec["mission"])
    # flight_only 允许不发布 mission_status（源 §9）；若发布则必须恒 aiming=false/last_drop=''
    checks.append(("RUN A mission_status 若发布则恒 aiming=false / last_drop=''",
                   all((not ai) and (ld == "") for _, ai, ld in miss_a),
                   "n=%d 违规=%d" % (len(miss_a),
                                     sum(1 for _, ai, ld in miss_a if ai or ld))))
    mloc_a = win(ta0, ta1, rec["sp_local"])
    mglo_a = win(ta0, ta1, rec["sp_global"])
    checks.append(("RUN A 本地+全局 setpoint 均有流", len(mloc_a) > 0 and len(mglo_a) > 0,
                   "local=%d global=%d" % (len(mloc_a), len(mglo_a))))
    modes_a = [m for _, m, _, _ in win(ta0, ta1, rec["modes"])]
    checks.append(("RUN A 模式含 GUIDED 且收尾 RTL", "GUIDED" in modes_a and "RTL" in modes_a,
                   "模式集=%s" % sorted(set(modes_a))))
    # 起飞高度：从日志判定（#18.1 避免 RTL 爬升污染；此处日志即离地高度）
    m_tk = re.search(r"已到达起飞高度: 离地 ([\d.]+)/", log_a)
    checks.append(("RUN A 起飞高度达标（≥目标-0.4m）",
                   bool(m_tk) and float(m_tk.group(1)) >= 2.5 - 0.4,
                   "离地=%s" % (m_tk.group(1) if m_tk else "无")))

    # ================= RUN B：无目标超时负例 =================
    ensure_grounded()
    rc_b, tb0, tb1 = run_node(
        "b",
        ["-p", "auto_start_detector:=false",
         "-p", "auto_start_servo_test:=false",
         "-p", "area_search_timeout_s:=8.0",
         "-p", "drop_zone_distance_m:=8.0",
         "-p", "recon_zone_distance_m:=16.0",
         "-p", "takeoff_altitude_m:=2.0",
         "-p", "recon_altitude_m:=3.0",
         "-p", "waypoint_timeout_s:=60.0",
         "-p", "state_timeout_s:=5.0", "-p", "guided_state_grace_s:=10.0",
         "-p", "pose_timeout_s:=5.0", "-p", "guided_pose_grace_s:=10.0",
         "-p", "navigation_timeout_s:=5.0", "-p", "navigation_grace_s:=10.0",
         "-p", "mode_timeout_s:=20.0"],
        timeout_s=240)
    tr_b = [(o, n) for o, n, _ in transitions("b")]
    servo_b = win(tb0, tb1, rec["servo"])
    b_ok = (rc_b == 1 and any(n == "FAILSAFE" for _, n in tr_b)
            and not any(n == "COMPLETE" for _, n in tr_b) and len(servo_b) == 0)
    checks.append(("RUN B 无目标超时负例（rc=1·FAILSAFE·零servo·无COMPLETE）", b_ok,
                   "rc=%s FAILSAFE=%s servo=%d COMPLETE=%s" % (
                       rc_b, any(n == "FAILSAFE" for _, n in tr_b), len(servo_b),
                       any(n == "COMPLETE" for _, n in tr_b))))

    # ================= RUN C：参数错误负例 =================
    rc_c, tc0, tc1 = run_node(
        "c",
        ["-p", "flight_only_mode:=true",
         "-p", "auto_start_detector:=false",
         "-p", "auto_start_servo_test:=false",
         "-p", "flight_only_search_altitude_m:=9.0",
         "-p", "recon_altitude_m:=5.0"],
        timeout_s=30)
    log_c = read_log(LOGS["c"])
    c_ok = (rc_c == 2 and "参数错误" in log_c and len(win(tc0, tc1, rec["servo"])) == 0)
    checks.append(("RUN C 参数错误负例（rc=2·零副作用）", c_ok, "rc=%s" % rc_c))

    # ============ RUN D：完整任务正例（ground_test + dry_run + 注入检测）============
    ensure_grounded()
    rc_d, td0, td1 = run_node(
        "d",
        ["-p", "ground_test:=true",
         "-p", "dry_run:=true",
         "-p", "enable_b_dropper:=true",
         "-p", "auto_start_detector:=false",
         "-p", "auto_start_servo_test:=false",
         "-p", "state_timeout_s:=5.0", "-p", "guided_state_grace_s:=10.0",
         "-p", "pose_timeout_s:=5.0", "-p", "guided_pose_grace_s:=10.0"],
        timeout_s=90, inject=True)
    log_d = read_log(LOGS["d"])
    tr_d = [(o, n) for o, n, _ in transitions("d")]
    exp_d = [("A_AIM", "A_DROP"), ("A_DROP", "B_AIM"), ("B_AIM", "B_DROP"),
             ("B_DROP", "COMPLETE")]
    servo_d = win(td0, td1, rec["servo"])
    checks.append(("RUN D 完整任务 dry_run（A/B 投放链→COMPLETE·零servo）",
                   rc_d == 0 and subseq(tr_d, exp_d) and len(servo_d) == 0,
                   "rc=%s 链=%s servo=%d" % (
                       rc_d, "→".join("%s→%s" % t for t in tr_d), len(servo_d))))
    miss_d = win(td0, td1, rec["mission"])
    # dry_run 的 drop() 早返回（源同构），不发布 last_drop；只校验瞄准期有 aiming=true
    checks.append(("RUN D mission_status 有 aiming 事件（dry 链）",
                   bool(miss_d) and any(ai for _, ai, _ in miss_d),
                   "n=%d aiming帧=%d" % (len(miss_d),
                                         sum(1 for _, ai, _ in miss_d if ai))))

    # ================= 日志与清理 =================
    errs_a = [l for l in log_a.splitlines() if "[ERROR]" in l]
    tb_a = [l for l in log_a.splitlines() if l.startswith("Traceback (most recent call last)")]
    errs_c = [l for l in log_c.splitlines() if "[ERROR]" in l]
    errs_d = [l for l in log_d.splitlines() if "[ERROR]" in l]
    tb_b = [l for l in read_log(LOGS["b"]).splitlines()
            if l.startswith("Traceback (most recent call last)")]
    tb_d = [l for l in log_d.splitlines() if l.startswith("Traceback (most recent call last)")]
    checks.append(("RUN A/C/D 0 [ERROR]·A/B/D 0 Traceback",
                   not errs_a and not errs_c and not errs_d and not tb_a and not tb_b and not tb_d,
                   "A_err=%d C_err=%d D_err=%d A_tb=%d B_tb=%d D_tb=%d" % (
                       len(errs_a), len(errs_c), len(errs_d),
                       len(tb_a), len(tb_b), len(tb_d))))

    left = subprocess.run(["pgrep", "-af", "auto_drop_test"],
                          capture_output=True, text=True).stdout
    left = [l for l in left.splitlines()
            if "bash -c" not in l and "awk" not in l and "pgrep" not in l
            and "verify_auto_drop" not in l]
    checks.append(("清理：无 auto_drop_test 残留进程", not left, "残留=%s" % left))

    report(1)


if __name__ == "__main__":
    main()
