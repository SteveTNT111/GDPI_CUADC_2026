#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场地世界自检：投放区/侦察区 5m(沿航线) × 8m(横向) 与竞赛规则/图1 一致 +（--sim）Gazebo 实载复核。

背景（2026-09-30）：原 world 把投放区/侦察区的长宽画反了（沿航线 8m × 横向 5m）。
竞赛规则图1 把 8m 标在横向、5m 标在沿航线方向，且与文本自洽：
起降点前 30m 起投放区（沿航线 5m）＋20m = 侦察区距起降点 55m。
本脚本离线断言 SDF 几何；--sim 时再查运行中的 GZ server，用模型实际位姿复核同一组数值。

用法:
  python3 scripts/verify_world_field.py            # 离线（只读文件）
  python3 scripts/verify_world_field.py --sim      # 额外连运行中的 GZ server 复核
退出码: 0=无 FAIL（WARN 不算），1=有 FAIL
"""
import argparse
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

WORLD_DEFAULT = "/home/lab/ROS2_CUADC2026/worlds/cuadc_2026_field.world"
CONFIG_DEFAULT = "/home/lab/ROS2_CUADC2026/config/auto_drop.yaml"
GZ_WORLD_DEFAULT = "cuadc_2026_field"

# 规则/图1 基准：中心 X、沿航线 X 区间、横向半宽
ZONES = {
    "drop": {"center": 32.5, "x0": 30.0, "x1": 35.0,
             "cyl": ["drop_cylinder_1", "drop_cylinder_2", "drop_cylinder_3"]},
    "recon": {"center": 57.5, "x0": 55.0, "x1": 60.0,
              "cyl": ["recon_cylinder_1", "recon_cylinder_2", "recon_cylinder_3",
                      "recon_cylinder_4", "recon_cylinder_5"]},
}
HALF_W = 4.0          # 横向半宽（8m/2）
ALONG_L = 5.0         # 沿航线长度

checks = []


def check(ok, name, detail=""):
    ok = bool(ok)
    checks.append((ok, name))
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", name,
                           ("  " + detail) if detail else ""))
    return ok


def warn(name, detail=""):
    print("  [WARN] %s%s" % (name, ("  " + detail) if detail else ""))


def find_model(root, name):
    for m in root.findall(".//model"):
        if m.get("name") == name:
            return m
    return None


def box_size(model):
    node = model.find("./link/visual/geometry/box/size")
    vals = [float(v) for v in node.text.split()]
    return tuple(vals[:2])


def pose_xy(model):
    node = model.find("pose")
    vals = [float(v) for v in node.text.split()]
    return vals[0], vals[1]


def include_pose(root, name):
    for inc in root.findall(".//include"):
        n = inc.find("name")
        if n is not None and n.text.strip() == name:
            vals = [float(v) for v in inc.find("pose").text.split()]
            return vals[0], vals[1], vals[2]
    return None


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


def check_zone(root, key):
    z = ZONES[key]
    c, x0, x1 = z["center"], z["x0"], z["x1"]
    print("[%s] 区域: 中心 X=%.1f, X: %.1f~%.1f, 横向 ±%.0fm（%s）"
          % (key, c, x0, x1, HALF_W, "5m(沿航线)×8m(横向)"))
    m = find_model(root, key + "_zone")
    if m is None:
        return check(False, "%s_zone 存在" % key)
    px, py = pose_xy(m)
    check(approx(px, c) and approx(py, 0.0), "%s_zone 中心位姿" % key,
          "pose=(%.2f, %.2f)" % (px, py))
    check(box_size(m) == (ALONG_L, 2 * HALF_W), "%s_zone 地面尺寸=5×8" % key,
          "size=%s" % (box_size(m),))
    # 四边框：N/S 在 y=±4、沿航线长 5；E/W 在 x=x0/x1、横向长 8
    borders = [("N", c, HALF_W, (ALONG_L, 0.08)), ("S", c, -HALF_W, (ALONG_L, 0.08)),
               ("E", x1, 0.0, (0.08, 2 * HALF_W)), ("W", x0, 0.0, (0.08, 2 * HALF_W))]
    for tag, bx, by, bsize in borders:
        bm = find_model(root, "%s_border_%s" % (key, tag))
        if bm is None:
            check(False, "%s_border_%s 存在" % (key, tag))
            continue
        bpx, bpy = pose_xy(bm)
        check(approx(bpx, bx, 1e-4) and approx(bpy, by, 1e-4)
              and box_size(bm) == bsize,
              "%s_border_%s" % (key, tag),
              "pose=(%.1f, %.1f) size=%s" % (bpx, bpy, box_size(bm)))
    corners = [("NE", x1, HALF_W), ("NW", x0, HALF_W),
               ("SE", x1, -HALF_W), ("SW", x0, -HALF_W)]
    for tag, bx, by in corners:
        cm = find_model(root, "%s_corner_%s" % (key, tag))
        if cm is None:
            check(False, "%s_corner_%s 存在" % (key, tag))
            continue
        cpx, cpy = pose_xy(cm)
        check(approx(cpx, bx, 1e-4) and approx(cpy, by, 1e-4),
              "%s_corner_%s" % (key, tag), "pose=(%.1f, %.1f)" % (cpx, cpy))
    # 圆筒必须在区域矩形内
    for cyl in z["cyl"]:
        p = include_pose(root, cyl)
        if p is None:
            check(False, "%s 存在" % cyl)
            continue
        check(x0 <= p[0] <= x1 and -HALF_W <= p[1] <= HALF_W,
              "%s 在区域内" % cyl, "pose=(%.2f, %.2f)" % (p[0], p[1]))


def check_takeoff(root):
    print("[takeoff] 起降区 33×8 地面块与边框对齐（X: -3~30）")
    m = find_model(root, "takeoff_zone")
    if m is None:
        return check(False, "takeoff_zone 存在")
    px, py = pose_xy(m)
    check(approx(px, 13.5) and approx(py, 0.0), "takeoff_zone 居中位姿",
          "pose=(%.2f, %.2f)（应 X=13.5 对应 -3~30）" % (px, py))
    check(box_size(m) == (33.0, 8.0), "takeoff_zone 尺寸=33×8",
          "size=%s" % (box_size(m),))


def check_rules_chain(root):
    print("[chain] 规则自洽链: 30m + 5m(投放区沿航线) + 20m = 55m")
    check(approx(ZONES["drop"]["x0"], 30.0), "投放区起点=起降点前 30m")
    check(approx(ZONES["recon"]["x0"] - ZONES["drop"]["x1"], 20.0), "两区沿航线间距=20m",
          "%.1f~%.1f" % (ZONES["drop"]["x1"], ZONES["recon"]["x0"]))
    check(approx(ZONES["recon"]["x0"], 55.0), "侦察区起点=起降点前 55m")
    check(approx(ZONES["recon"]["center"] - ZONES["drop"]["center"], 25.0),
          "两区中心间距=25m")


def check_config(path):
    print("[config] 全自动脚本搜索区应与区域同向（横向 8m / 沿航线 5m）")
    try:
        text = open(path, "r", encoding="utf-8").read()
    except OSError as exc:
        return warn("读不到 %s: %s" % (path, exc))
    mw = re.search(r"^\s*drop_zone_width_m:\s*([0-9.]+)", text, re.M)
    ml = re.search(r"^\s*drop_zone_length_m:\s*([0-9.]+)", text, re.M)
    if not (mw and ml):
        return warn("config 里没有 drop_zone_width_m/length_m，跳过")
    w, l = float(mw.group(1)), float(ml.group(1))
    check(approx(w, 2 * HALF_W, 1e-3) and approx(l, ALONG_L, 1e-3),
          "area_search 尺寸=8(横向)×5(沿航线)", "解析到 %.2f×%.2f" % (w, l))


def sim_pose(gz_world, model):
    try:
        out = subprocess.run(["gz", "model", "-m", model, "--pose"],
                             capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired) as exc:
        return None, "gz model 调用失败: %s" % exc
    # 输出样例： "  - Pose [ XYZ (m) ] [ RPY (rad) ]:" / "    [32.5 4.0 0.004]" / "    [0 0 0]"
    for line in out.splitlines():
        nums = re.findall(r"[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?", line)
        if len(nums) >= 3:
            return [float(v) for v in nums[:3]], out
    return None, out


def check_sim(gz_world):
    print("[--sim] 运行中的 Gazebo（world=%s）实载复核" % gz_world)
    try:
        out = subprocess.run(["gz", "model", "--list"],
                             capture_output=True, text=True, timeout=15).stdout
    except (OSError, subprocess.TimeoutExpired) as exc:
        return check(False, "gz model --list", str(exc))
    names = [n.strip() for n in out.split() if n.strip()]
    if not names:
        return check(False, "gz model --list 有输出", "world 未起或查询失败")
    expect = ["drop_zone", "drop_border_N", "drop_border_E",
              "recon_zone", "recon_border_N", "recon_border_E",
              "takeoff_zone"]
    for name in expect:
        check(name in names, "模型 %s 已加载" % name)
    for name, ex, ey in [("drop_zone", ZONES["drop"]["center"], 0.0),
                         ("drop_border_N", ZONES["drop"]["center"], HALF_W),
                         ("drop_border_E", ZONES["drop"]["x1"], 0.0),
                         ("recon_zone", ZONES["recon"]["center"], 0.0),
                         ("recon_border_N", ZONES["recon"]["center"], HALF_W),
                         ("recon_border_E", ZONES["recon"]["x1"], 0.0),
                         ("takeoff_zone", 13.5, 0.0)]:
        vals, raw = sim_pose(gz_world, name)
        if vals is None or len(vals) < 2:
            check(False, " %s 位姿" % name, "解析失败: %r" % raw[:120])
            continue
        check(approx(vals[0], ex, 1e-3) and approx(vals[1], ey, 1e-3),
              " %s 位姿" % name,
              "实载=(%.3f, %.3f) 期望=(%.1f, %.1f)" % (vals[0], vals[1], ex, ey))


def main():
    ap = argparse.ArgumentParser(description="场地 world 几何自检")
    ap.add_argument("--world", default=WORLD_DEFAULT, help="world 文件路径")
    ap.add_argument("--config", default=CONFIG_DEFAULT, help="auto_drop.yaml 路径")
    ap.add_argument("--gz-world", default=GZ_WORLD_DEFAULT, help="Gazebo world 名（--sim 用）")
    ap.add_argument("--sim", action="store_true", help="连运行中的 GZ server 复核")
    args = ap.parse_args()

    print("== 离线：SDF 几何断言 ==")
    root = ET.parse(args.world).getroot()
    world = root.find("world")
    print("[world] %s（name=%s）" % (args.world, world.get("name") if world is not None else "?"))
    check(world is not None and world.get("name") == GZ_WORLD_DEFAULT, "world 名称")
    for key in ZONES:
        check_zone(root, key)
    check_takeoff(root)
    check_rules_chain(root)
    check_config(args.config)

    if args.sim:
        check_sim(args.gz_world)

    npass = sum(1 for ok, _ in checks if ok)
    nfail = len(checks) - npass
    print("\n== 结果: PASS=%d FAIL=%d ==" % (npass, nfail))
    if nfail:
        print("== 场地自检 FAIL ❌ ==")
        return 1
    print("== 场地自检全绿 ✅ ==")
    return 0


if __name__ == "__main__":
    sys.exit(main())
