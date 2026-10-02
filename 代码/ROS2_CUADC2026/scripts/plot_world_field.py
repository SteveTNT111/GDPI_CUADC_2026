#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把场地 world 画成俯视平面图（PNG），用于对照竞赛规则图1 检查几何。

只画几何、不连仿真：读 worlds/cuadc_2026_field.world 里各 model 的 pose/size，
输出 /tmp/opencode/field_topdown.png（+X 向右、+Y 向上，单位 m）。
用法: python3 scripts/plot_world_field.py [--world X.world] [--out Y.png]
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont

WORLD = "/home/lab/ROS2_CUADC2026/worlds/cuadc_2026_field.world"
OUT = "/tmp/opencode/field_topdown.png"
SCALE = 16          # px / m
PAD = 40


def parse_model(world, name):
    for m in world.findall(".//model"):
        if m.get("name") == name:
            vals = [float(v) for v in m.find("pose").text.split()]
            geo = m.find(".//geometry/box/size")
            size = tuple(float(v) for v in geo.text.split()[:2]) if geo is not None else None
            return {"pose": vals, "size": size, "kind": "model", "el": m}
    return None


def parse_include(world, name):
    for inc in world.findall(".//include"):
        n = inc.find("name")
        if n is not None and n.text.strip() == name:
            vals = [float(v) for v in inc.find("pose").text.split()]
            return {"pose": vals, "uri": inc.find("uri").text, "el": inc}
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=WORLD)
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()

    root = ET.parse(args.world).getroot()
    img = Image.new("RGB", (int(70 * SCALE) + 2 * PAD, int(12 * SCALE) + 2 * PAD), (245, 245, 245))
    d = ImageDraw.Draw(img)
    try:
        cjk = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
        dejavu = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
        font = ImageFont.truetype(cjk if os.path.exists(cjk) else dejavu, 13)
        font_s = ImageFont.truetype(cjk if os.path.exists(cjk) else dejavu, 10)
    except OSError:
        font = font_s = ImageFont.load_default()

    def to_px(x, y):
        return PAD + (x + 5) * SCALE, PAD + (6 - y) * SCALE

    def rect(cx, cy, sx, sy, fill, outline):
        x0, y0 = to_px(cx - sx / 2, cy + sy / 2)
        x1, y1 = to_px(cx + sx / 2, cy - sy / 2)
        d.rectangle([x0, y0, x1, y1], fill=fill, outline=outline, width=1)

    # 地面与起降区
    rect(13.5, 0, 33, 8, (222, 214, 122), (170, 150, 40))
    for x, y in ((0, 0),):
        px, py = to_px(x, y)
        d.ellipse([px - 6, py - 6, px + 6, py + 6], outline=(30, 160, 60), width=2)
        d.text((px - 3, py - 5), "H", fill=(30, 120, 60), font=font_s)
    px, _ = to_px(2, 4)
    d.line([px, to_px(0, 4)[1], px, to_px(0, -4)[1]], fill=(200, 30, 30), width=2)

    # 投放区
    rect(32.5, 0, 5, 8, (170, 190, 240), (40, 70, 180))
    # 侦察区
    rect(57.5, 0, 5, 8, (205, 205, 205), (120, 120, 120))

    # 圆筒（按物理直径画，最小视觉半径 3px）
    for name, diameter in [("drop_cylinder_1", 0.15), ("drop_cylinder_2", 0.20),
                           ("drop_cylinder_3", 0.25), ("recon_cylinder_1", 0.20),
                           ("recon_cylinder_2", 0.20), ("recon_cylinder_3", 0.20),
                           ("recon_cylinder_4", 0.20), ("recon_cylinder_5", 0.20)]:
        inc = parse_include(root, name)
        if inc is None:
            continue
        x, y = inc["pose"][0], inc["pose"][1]
        px, py = to_px(x, y)
        r = max(3, diameter * SCALE / 2)
        d.ellipse([px - r, py - r, px + r, py + r], fill=(255, 255, 255), outline=(60, 60, 60))

    # 尺寸标注：5m 沿航线、8m 横向
    for cx, label in ((32.5, "投放区 5m(沿航线)×8m(横向)"), (57.5, "侦察区 5m(沿航线)×8m(横向)")):
        x0, y0 = to_px(cx - 2.5, 4)
        x1, y1 = to_px(cx + 2.5, 4)
        d.line([x0, y0 - 14, x1, y1 - 14], fill=(0, 0, 0))
        d.text((x0 - 8, y0 - 30), "5m", fill=(0, 0, 0), font=font_s)
        x2, y2 = to_px(cx + 2.5, 4)
        x3, y3 = to_px(cx + 2.5, -4)
        d.line([x2 + 14, y2, x3 + 14, y3], fill=(0, 0, 0))
        d.text((x2 + 16, (y2 + y3) / 2 - 12), "8m", fill=(0, 0, 0), font=font_s)

    # 距离标尺
    d.line([to_px(0, -5.2)[0], to_px(0, -5.2)[1], to_px(30, -5.2)[0], to_px(30, -5.2)[1]],
           fill=(0, 0, 200))
    d.text(to_px(13, -5.4), "30m（起降点→投放区）", fill=(0, 0, 200), font=font_s)
    d.line([to_px(35, -5.2)[0], to_px(35, -5.2)[1], to_px(55, -5.2)[0], to_px(55, -5.2)[1]],
           fill=(0, 0, 200))
    d.text(to_px(36, -5.4), "20m", fill=(0, 0, 200), font=font_s)

    for x in range(0, 65, 5):
        px, py = to_px(x, -4.6)
        if x % 10 == 0:
            d.text((px - 5, py + 6), str(x), fill=(90, 90, 90), font=font_s)
    d.text((PAD, 8), "CUADC2026 场地俯视图（+X=飞行方向；由 worlds/cuadc_2026_field.world 生成）",
           fill=(20, 20, 20), font=font)
    img.save(args.out)
    print("saved", args.out, img.size)


if __name__ == "__main__":
    main()
