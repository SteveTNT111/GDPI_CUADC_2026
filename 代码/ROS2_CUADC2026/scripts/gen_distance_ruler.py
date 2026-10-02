#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_distance_ruler.py — 生成场地距离标尺模型（models/distance_ruler/）。

用途：沿 Y=+4m（投放区/侦察区北侧角点连线）从 X=0（起飞点）到 X=60m，
     画一条"连续标尺"：连续基线 + 每 1m 短刻度 + 每 5m 中刻度 + 每 10m 长刻度与数字，
     用于在 Gazebo 俯视/机载画面里目视确认飞机前进距离。

约定：
  - 深色（避免与白色桶混淆，影响视觉识别）；
  - 标尺整体 z=0.008~0.0085（高于场地所有地面/边框，防 z-fighting）；
  - 数字为 7 段码盒体拼字；数字"上"方向 = +X（机头方向），飞行画面里正读。
参数都在文件头常量里，改完重跑本脚本即可（会重写 models/distance_ruler/model.sdf）。
"""
import os

# ===== 布局参数（m）=====
Y_LINE = 4.0          # 标尺所在 Y（北侧角点连线）
X_START = 0.0         # 起飞点
X_END = 60.0          # 侦察区末端（北侧角点）
Z_BASE = 0.008        # 基线高度（高于所有场地视觉）
Z_MARK = 0.0084       # 刻度/数字高度（略高于基线，防同层闪烁）
BASE_W = 0.06         # 基线宽
TICK1 = 0.30          # 1m 刻度长（沿 Y，居中于基线）
TICK5 = 0.80          # 5m 刻度长
TICK10 = 1.50         # 10m 刻度长
THICK = 0.05          # 刻度厚（沿 X）
THICK10 = 0.06

# ===== 数字（7 段码）=====
DIG_H = 0.60          # 数字高（沿 X）
DIG_W = 0.34          # 数字宽（沿 Y）
DIG_T = 0.07          # 段厚
DIG_GAP = 0.02        # 段间隙
DIG_SPACING = 0.14    # 数字间距
DIG_Y_CENTER = 5.15   # 数字中心行（基线北侧，避开刻度端）
DIGITS = {            # a=上 b=右上 c=右下 d=下 e=左下 f=左上 g=中
    0: "abcdef", 1: "bc", 2: "abgde", 3: "abgcd", 4: "fgbc",
    5: "afgcd", 6: "afgedc", 7: "abc", 8: "abcdefg", 9: "abcdfg",
}
MARKS_EVERY_10 = [0, 10, 20, 30, 40, 50, 60]

DARK = ("0.05 0.05 0.05 1", "0.10 0.10 0.10 1")  # ambient / diffuse

OUTDIR = "/home/lab/ROS2_CUADC2026/models/distance_ruler"


def seg_boxes(digit, du, dv):
    """返回某一位数字的段盒体列表：[(cx, cy, sx, sy)]。

    数字局部坐标系：u=读数方向（世界 -Y），v=数字上方向（世界 +X）。
    """
    a, b, c, d, e, f, g = "a" in digit, "b" in digit, "c" in digit, "d" in digit, \
        "e" in digit, "f" in digit, "g" in digit
    top = (DIG_H - DIG_T) / 2.0
    vert_dv = (DIG_H - DIG_T) / 4.0
    vert_h = (DIG_H / 2.0 - 1.5 * DIG_T) - 2 * DIG_GAP
    left_u = -(DIG_W - DIG_T) / 2.0
    right_u = +(DIG_W - DIG_T) / 2.0
    boxes = []
    if a:
        boxes.append((du, dv + top, DIG_W, DIG_T))
    if g:
        boxes.append((du, dv, DIG_W, DIG_T))
    if d:
        boxes.append((du, dv - top, DIG_W, DIG_T))
    if f:
        boxes.append((du + left_u, dv + vert_dv, DIG_T, vert_h))
    if b:
        boxes.append((du + right_u, dv + vert_dv, DIG_T, vert_h))
    if e:
        boxes.append((du + left_u, dv - vert_dv, DIG_T, vert_h))
    if c:
        boxes.append((du + right_u, dv - vert_dv, DIG_T, vert_h))
    return boxes


def emit_visual(name, cx, cy, sx, sy, sz, z, dark=DARK):
    return (
        '      <visual name="%s">\n'
        '        <pose>%.4f %.4f %.4f 0 0 0</pose>\n'
        '        <geometry><box><size>%.4f %.4f %.4f</size></box></geometry>\n'
        '        <material><ambient>%s</ambient><diffuse>%s</diffuse></material>\n'
        '      </visual>\n' % (name, cx, cy, z, sx, sy, sz, dark[0], dark[1])
    )


def main():
    visuals = []
    # 1) 连续基线
    visuals.append(emit_visual(
        "base", (X_START + X_END) / 2.0, Y_LINE, X_END - X_START, BASE_W, 0.0025, Z_BASE))
    # 2) 刻度
    marks = []
    x = X_START
    while x <= X_END + 1e-6:
        n = int(round(x))
        if n % 10 == 0:
            ln, th = TICK10, THICK10
            tag = "t10_%d" % n
        elif n % 5 == 0:
            ln, th = TICK5, THICK
            tag = "t5_%d" % n
        else:
            ln, th = TICK1, THICK
            tag = "t1_%d" % n
        visuals.append(emit_visual(tag, float(n), Y_LINE, th, ln, 0.0025, Z_MARK))
        marks.append(n)
        x += 1.0
    # 3) 数字（每 10m）
    for n in MARKS_EVERY_10:
        text = str(n)
        k = len(text)
        for i, ch in enumerate(text):
            du = (i - (k - 1) / 2.0) * (DIG_W + DIG_SPACING)
            for j, (su, sv, s_w, s_h) in enumerate(seg_boxes(DIGITS[int(ch)], du, 0.0)):
                # 数字系 (u,v) -> 世界 (x,y)：u-> -Y, v-> +X
                cx = float(n) + sv
                cy = DIG_Y_CENTER - su
                visuals.append(emit_visual(
                    "d%d_%d_%d" % (n, i, j), cx, cy, s_h, s_w, 0.0025, Z_MARK))

    os.makedirs(OUTDIR, exist_ok=True)
    lines = ['<?xml version="1.0" ?>', '<sdf version="1.8">',
             '  <!-- 自动生成：scripts/gen_distance_ruler.py 勿手改 -->',
             '  <model name="distance_ruler">', '    <static>true</static>',
             '    <link name="link">']
    lines.extend(v.rstrip("\n") for v in visuals)
    lines += ['    </link>', '  </model>', '</sdf>', '']
    with open(os.path.join(OUTDIR, "model.sdf"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines))
    with open(os.path.join(OUTDIR, "model.config"), "w", encoding="utf-8") as fh:
        fh.write('<?xml version="1.0"?>\n<model>\n  <name>distance_ruler</name>\n'
                 '  <version>1.0</version>\n  <sdf version="1.8">model.sdf</sdf>\n'
                 '  <description>Y=+4m 距离标尺：1m 刻度/5m/10m+数字，目视确认前进距离'
                 '（scripts/gen_distance_ruler.py 生成）</description>\n</model>\n')
    print("saved %s/model.sdf（%d 个 visual）" % (OUTDIR, len(visuals)))


if __name__ == "__main__":
    main()
