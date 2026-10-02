#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线几何自检：注入器坐标 ⇄ auto_drop._build_aim_view（FRD→ENU 修正后）互为逆。

不连飞控：构造一个假位姿（其中包含 yaw/俯仰/横滚各一例），按 t2_vision_inject
的公式生成 YoloDetections，塞进 AutoDropTest._build_aim_view，检查其米制 NED
是否回到真值。用法：source ROS2 + install 后 python3 本文件。
"""
import math
import sys
import types

sys.path.insert(0, "/home/lab/ROS2_CUADC2026/src/cuadc_control/scripts")
sys.path.insert(0, "/home/lab/ROS2_CUADC2026/scripts")

import t2_vision_inject as inj  # noqa: E402
from auto_drop_test import AutoDropTest  # noqa: E402


class FakeLogger:
    def info(self, *a, **k):
        pass

    warn = warning = error = info


class FakeNode:
    def get_logger(self):
        return FakeLogger()


class FakePose:
    def __init__(self, e, n, u, q):
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=e, y=n, z=u),
            orientation=types.SimpleNamespace(x=q[0], y=q[1], z=q[2], w=q[3]))


def yaw_q(deg):
    r = math.radians(deg) / 2.0
    return (0.0, 0.0, math.sin(r), math.cos(r))


def rpy_q(roll_deg, pitch_deg, yaw_deg):
    cr, sr = math.cos(math.radians(roll_deg) / 2), math.sin(math.radians(roll_deg) / 2)
    cp, sp = math.cos(math.radians(pitch_deg) / 2), math.sin(math.radians(pitch_deg) / 2)
    cy, sy = math.cos(math.radians(yaw_deg) / 2), math.sin(math.radians(yaw_deg) / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


class Det:
    pass


def inject_det(pose_e, pose_n, pose_u, q, target, fx=648.4, fy=648.4, cx=320.0, cy=240.0,
               ratio=0.058):
    """按注入器公式生成一条 detection/geo（返回 (det, geo_delta)）。"""
    delta = (target[0] - pose_e, target[1] - pose_n, target[2] - pose_u)
    ux, uy, uz = inj.q_rotate(inj.q_conj(q), delta)
    depth = -uz
    cam = (uy, -ux, depth)
    u_px = cx + fx * cam[0] / depth
    v_px = cy + fy * cam[1] / depth
    diag = depth / ratio
    half = diag / math.sqrt(2.0) / 2.0
    d = Det()
    d.detected = True
    d.class_name = "bucket"
    d.confidence = 0.9
    d.position_valid = True
    d.depth_m = depth
    d.camera_x_m, d.camera_y_m, d.camera_z_m = cam
    d.center_x, d.center_y = int(round(u_px)), int(round(v_px))
    d.x_min, d.y_min = int(u_px - half), int(v_px - half)
    d.x_max, d.y_max = int(u_px + half), int(v_px + half)
    return d, delta


def make_handle(pose):
    h = object.__new__(AutoDropTest)
    h.node = FakeNode()
    h.vision_min_confidence = 0.30
    h.current_pose = pose
    h.camera_fx = 648.4
    h.camera_fy = 648.4
    h.camera_center_x = 320.0
    h.camera_center_y = 240.0
    h.preferred_target_ned = None
    h.preferred_target_gate_m = 2.0
    h.front_dropper = "A"
    h.crosshair_offset_m = 0.05
    import threading
    h._lock = threading.RLock()
    h._finite_number = AutoDropTest._finite_number
    return h


def run_case(desc, pose_e, pose_n, pose_u, q, target):
    pose = FakePose(pose_e, pose_n, pose_u, q)
    det, delta = inject_det(pose_e, pose_n, pose_u, q, target)
    msg = types.SimpleNamespace(detections=[det])
    h = make_handle(pose)
    view = h._build_aim_view(msg)
    ok = view.valid
    got = (view.a_ned_n, view.a_ned_e, view.a_ned_d)
    want = (target[1], target[0], -target[2])
    err = max(abs(got[i] - want[i]) for i in range(3))
    ok = ok and err < 0.30     # tolerance: pixel rounding + no mount offset
    print("  %s %s | got NED=(%+.3f,%+.3f,%+.3f) want=(%+.3f,%+.3f,%+.3f) err=%.3fm"
          % ("PASS" if ok else "FAIL", desc, got[0], got[1], got[2],
             want[0], want[1], want[2], err))
    return ok


def run_ab_case():
    """A/B 槽位：3 条检测 → A=最高置信度、B=次高且像素可区分。"""
    pose = FakePose(31.0, 0.6, 3.0, yaw_q(90))
    targets = [(32.3, 1.4, 0.30, 0.85), (31.0, 0.6, 0.30, 0.90), (32.6, -0.9, 0.30, 0.80)]
    dets = []
    for (e, n, u, conf) in targets:
        d, _ = inject_det(31.0, 0.6, 3.0, yaw_q(90), (e, n, u))
        d.confidence = conf
        dets.append(d)
    h = make_handle(pose)
    view = h._build_aim_view(types.SimpleNamespace(detections=dets))
    ok = True
    ok_a = (view.a_ned_valid and abs(view.a_ned_n - 0.6) < 0.05
            and abs(view.a_ned_e - 31.0) < 0.05)
    ok_b = (view.b_ned_valid and abs(view.b_ned_n - 1.4) < 0.05
            and abs(view.b_ned_e - 32.3) < 0.05)
    print("  %s A/B 槽位：A(mid)=(%+.2f,%+.2f) B(small)=(%+.2f,%+.2f) valid=%s/%s"
          % ("PASS" if (ok_a and ok_b) else "FAIL", view.a_ned_n, view.a_ned_e,
             view.b_ned_n, view.b_ned_e, view.a_ned_valid, view.b_ned_valid))
    ok = ok and ok_a and ok_b
    # 单目标 → B 槽 invalid
    h2 = make_handle(pose)
    v2 = h2._build_aim_view(types.SimpleNamespace(detections=[dets[1]]))
    ok_c = v2.a_ned_valid and not v2.b_ned_valid
    print("  %s 单目标帧：A valid / B invalid = %s/%s"
          % ("PASS" if ok_c else "FAIL", v2.a_ned_valid, v2.b_ned_valid))
    ok = ok and ok_c
    # 偏好目标：锁定"选中的桶"（小号），而非最高置信度（中号）
    h3 = make_handle(pose)
    h3.preferred_target_ned = (1.4, 32.3, -0.30)   # small 桶 NED
    v3 = h3._build_aim_view(types.SimpleNamespace(detections=dets))
    ok_d = (v3.valid and abs(v3.a_ned_e - 32.3) < 0.05 and abs(v3.a_ned_n - 1.4) < 0.05
            and abs(v3.b_ned_e - 32.3) < 0.05)
    print("  %s 偏好桶=small：A/B 均锁 small (e=%+.2f,n=%+.2f)"
          % ("PASS" if ok_d else "FAIL", v3.a_ned_e, v3.a_ned_n))
    # 偏好目标不在视野 → valid=false（不投未选中的桶）
    h4 = make_handle(pose)
    h4.preferred_target_ned = (50.0, 50.0, -0.30)
    v4 = h4._build_aim_view(types.SimpleNamespace(detections=dets))
    ok_e = not v4.valid
    print("  %s 偏好桶不在视野：valid=%s（期望 False）"
          % ("PASS" if ok_e else "FAIL", v4.valid))
    return ok and ok_d and ok_e


def run_crosshair_case():
    """制导点=投放器准星：飞机按 _aim_setpoint_ned 飞到位后，相机系差值应为 0。

    回归：曾把 setpoint 设为桶正上方（相机原点对桶），FINAL 判据 Y 恒差
    ±crosshair_offset（实飞 2026-10-01 21:29：Y=+0.050m，20s 超时 FAILSAFE）。
    本用例同时验证"旧做法必然差一个 offset"，防止制导点又被改回相机原点。
    """
    ok = True
    bucket_enu = (32.5, 0.3, 0.30)               # E,N,U
    bucket_ned = (bucket_enu[1], bucket_enu[0], -bucket_enu[2])
    flight_u = 1.5                               # 投放高度（相对地面）
    for dropper, yaw in (("A", 90), ("B", 270)):  # A=前投（默认），B=后投
        prefix = dropper.lower()
        pose = FakePose(30.0, 0.0, 3.0, yaw_q(yaw))
        h = make_handle(pose)
        h.preferred_target_ned = bucket_ned
        sp_ned = h._aim_setpoint_ned(bucket_ned, dropper, pose)
        sp_enu = (sp_ned[1], sp_ned[0], flight_u)
        # 飞机位于制导点：注入该桶检测（偏好桶=两槽同填）→ 该投放器准星差值应 ≈ 0
        det, _ = inject_det(sp_enu[0], sp_enu[1], flight_u, yaw_q(yaw), bucket_enu)
        h2 = make_handle(FakePose(sp_enu[0], sp_enu[1], flight_u, yaw_q(yaw)))
        h2.preferred_target_ned = bucket_ned
        view = h2._build_aim_view(types.SimpleNamespace(detections=[det]))
        dx = float(getattr(view, prefix + "_delta_x_m"))
        dy = float(getattr(view, prefix + "_delta_y_m"))
        good = view.valid and abs(dx) < 0.03 and abs(dy) < 0.03
        print("  %s 制导点 %s(yaw=%d) 到位后 X=%+.3fm Y=%+.3fm（期望≈0）"
              % ("PASS" if good else "FAIL", dropper, yaw, dx, dy))
        ok = ok and good
        # 旧做法（相机原点对桶）按同源判据必然残留 ±0.05
        det0, _ = inject_det(bucket_enu[0], bucket_enu[1], flight_u,
                             yaw_q(yaw), bucket_enu)
        h3 = make_handle(FakePose(bucket_enu[0], bucket_enu[1], flight_u, yaw_q(yaw)))
        h3.preferred_target_ned = bucket_ned
        v0 = h3._build_aim_view(types.SimpleNamespace(detections=[det0]))
        legacy = float(getattr(v0, prefix + "_delta_y_m"))
        expect = (0.05 if dropper == "A" else -0.05)
        good0 = abs(legacy - expect) < 0.01
        print("  %s 旧做法（相机原点对桶）%s 残留 Y=%+.3fm（期望 %+.3f，即超时根因）"
              % ("PASS" if good0 else "FAIL", dropper, legacy, expect))
        ok = ok and good0
    return ok


def main():
    checks = []
    # 目标：投放区三桶位置（ENU 绝对）
    T = (32.5, 0.3, 0.30)
    checks.append(run_case("平飞悬停 yaw=90", 30.0, 0.0, 3.0, yaw_q(90), T))
    checks.append(run_case("平飞悬停 yaw=0 ", 30.0, 0.0, 3.0, yaw_q(0), T))
    checks.append(run_case("偏航+俯仰     ", 30.5, -0.5, 3.0, rpy_q(0, 8, 45), T))
    checks.append(run_case("偏航+横滚     ", 30.5, -0.5, 2.6, rpy_q(-10, 0, 200), T))
    checks.append(run_case("低空 (1.8m)   ", 32.0, 0.0, 1.8, rpy_q(3, -5, 90), T))
    # 目标在正下方：yaw 任意
    checks.append(run_case("正下方目标    ", 32.5, 0.3, 3.0, yaw_q(33), T))
    checks.append(run_ab_case())
    checks.append(run_crosshair_case())
    print("\n==== 注入器几何自检：%s ====" % ("ALL PASS" if all(checks) else "FAIL"))
    return 0 if all(checks) else 1


if __name__ == "__main__":
    sys.exit(main())
