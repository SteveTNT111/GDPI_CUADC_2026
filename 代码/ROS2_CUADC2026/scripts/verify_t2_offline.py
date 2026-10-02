#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""T2 离线构造级验证：比值分档 / 选桶(含降级) / 桶记录原子写 / 配置解析。

不连飞控、不起仿真：直接对 AutoDropTest 的纯函数与构造对象做断言。
用法：source /opt/ros/humble/setup.bash && source ~/ROS2_CUADC2026/install/setup.bash
      python3 /tmp/opencode/verify_t2_offline.py
"""
import json
import os
import sys
import tempfile

sys.path.insert(0, "/home/lab/ROS2_CUADC2026/src/cuadc_control/scripts")

import auto_drop_config as cfg  # noqa: E402
from auto_drop_test import AutoDropTest  # noqa: E402


class FakeLogger:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(("INFO", m % a if a else m))

    def warn(self, m, *a, **k):
        self.lines.append(("WARN", m % a if a else m))

    def warning(self, m, *a, **k):
        self.lines.append(("WARN", m % a if a else m))

    def error(self, m, *a, **k):
        self.lines.append(("ERROR", m % a if a else m))


class FakeNode:
    def __init__(self):
        self.log = FakeLogger()

    def get_logger(self):
        return self.log


class FakePose:
    class _P:
        class _Pos:
            def __init__(self, x, y):
                self.x, self.y = x, y
        def __init__(self, x, y):
            self.position = FakePose._P._Pos(x, y)
    def __init__(self, x, y):
        self.pose = FakePose._P(x, y)


class Det:
    def __init__(self, x_min, y_min, x_max, y_max, depth, conf=0.9,
                 cx=None, cy=None, valid=True):
        self.x_min, self.y_min, self.x_max, self.y_max = x_min, y_min, x_max, y_max
        self.depth_m = depth
        self.confidence = conf
        self.detected = True
        self.position_valid = valid
        self.center_x = cx if cx is not None else (x_min + x_max) // 2
        self.center_y = cy if cy is not None else (y_min + y_max) // 2


def make_bucket(handle, e, n, u, ratio, samples=5):
    b = {"id": 0, "samples": __import__("collections").deque(maxlen=15)}
    for _ in range(samples):
        b["samples"].append((e, n, u, 30.0 + e * 1e-6, 120.0 + n * 1e-6, 5.0 + u, ratio))
    b["samples_count"] = samples
    b["updated_at"] = "2026-09-30T12:00:00+08:00"
    handle.buckets.append(b)
    handle._recompute_bucket(b)
    return b


def main():
    checks = []

    def check(cond, desc):
        checks.append((cond, desc))
        print(("  PASS " if cond else "  FAIL ") + desc)

    handle = object.__new__(AutoDropTest)
    handle.node = FakeNode()
    handle.bucket_ratio_ref_200 = 0.20
    handle.bucket_ratio_tol = 0.15
    handle.bucket_ratio_invert = False
    handle.bucket_assoc_distance_m = 0.6   # #59 新默认（=config bucket_assoc_distance_m）
    handle.vision_min_confidence = 0.30
    handle.area_search_pattern = "W"
    handle.search_altitude_m = 3.0
    handle.recon_enabled = False
    handle.record_buckets = True
    handle.bucket_record_dir = tempfile.mkdtemp(prefix="t2_buckets_")
    handle.drop_zone_wgs84 = (30.5, 120.5)
    handle.recon_zone_wgs84 = (30.6, 120.6)
    handle.buckets = []
    handle.selected_buckets = []
    handle.bucket_notes = []
    import threading
    import time as _time
    handle._lock = threading.RLock()
    handle.current_pose = FakePose(0.0, 0.0)
    handle.pose_received_at = _time.monotonic()
    handle.pose_timeout_s = 2.0

    # ---------- 1. ratio 公式 ----------
    handle.camera_fx = 500.0
    handle.camera_fy = 500.0
    print("[1] ratio = depth_m*sqrt((w/fx)^2+(h/fy)^2)")
    det = Det(100, 100, 140, 130, 6.0)  # diag=hypot(40,30)=50
    ratio = handle._bucket_ratio(det)
    expected = 6.0 * ((40.0 / 500.0) ** 2 + (30.0 / 500.0) ** 2) ** 0.5
    check(ratio is not None and abs(ratio - expected) < 1e-12,
          "ratio=%.6f (期望 %.6f)" % (ratio if ratio else -1, expected))
    check(handle._bucket_ratio(Det(0, 0, 0, 0, 6.0)) is None, "diag=0 返回 None")
    check(handle._bucket_ratio(Det(0, 0, 40, 30, 0.0)) is None, "depth=0 返回 None")

    # ---------- 2. 分档方向 ----------
    print("[2] 分档：物理尺寸代理小=小号/大=大号")
    check(handle._bucket_size_class(0.10) == "small", "ratio=0.10 -> small")
    check(handle._bucket_size_class(0.16) == "small", "ratio=0.16 -> small (<0.17)")
    check(handle._bucket_size_class(0.20) == "mid", "ratio=0.20 -> mid")
    check(handle._bucket_size_class(0.225) == "mid", "ratio=0.225 -> mid")
    check(handle._bucket_size_class(0.24) == "large", "ratio=0.24 -> large (>0.23)")
    handle.bucket_ratio_invert = True
    check(handle._bucket_size_class(0.08) == "large", "invert: 0.08 -> large")
    check(handle._bucket_size_class(0.50) == "small", "invert: 0.50 -> small")
    handle.bucket_ratio_invert = False

    # ---------- 3. 关联 GeoTarget↔YoloDetections ----------
    print("[3] 目标关联（索引对齐 + 像素/置信度校验）")
    class Geo:
        def __init__(self, cx, cy, conf):
            self.center_x, self.center_y, self.confidence = cx, cy, conf
    dets = [Det(280, 200, 360, 260, 6.0, 0.95, cx=320, cy=230),
            Det(120, 200, 200, 260, 6.0, 0.80, cx=160, cy=230)]
    got = handle._associate_detection(Geo(320, 230, 0.95), dets)
    check(got is dets[0], "id1 关联到中心(320,230)")
    got = handle._associate_detection(Geo(160, 230, 0.80), dets)
    check(got is dets[1], "id2 关联到中心(160,230)")
    got = handle._associate_detection(Geo(500, 500, 0.95), dets)
    check(got is None, "像素距过大 -> None")

    # ---------- 4. 选桶：三桶 mid+small、先近后远 ----------
    print("[4] 选桶：三桶 (large/mid/small)")
    handle.buckets = []
    b_large = make_bucket(handle, 10.0, 0.0, 5.0, 0.26)   # ratio>0.23 -> large
    b_mid = make_bucket(handle, 2.0, 0.0, 5.0, 0.20)      # mid
    b_small = make_bucket(handle, 5.0, 0.0, 5.0, 0.14)    # small
    handle._prune_buckets()
    sel, notes = handle._select_buckets()
    got = [(s["id"], s["size_class"]) for s in sel]
    print("   selected:", got, "notes:", notes)
    check(len(sel) == 2, "选出 2 个桶")
    sel_classes = {s["size_class"] for s in sel}
    check("large" not in sel_classes, "不含 large（只取 mid+small）")
    check(any(s["reason"] == "closest_to_ref" for s in sel), "含 closest_to_ref")

    # ---------- 5. 降级：只有两个桶 ----------
    print("[5] 降级：只搜索到两个桶")
    handle.buckets = []
    make_bucket(handle, 2.0, 0.0, 5.0, 0.20)
    make_bucket(handle, 8.0, 0.0, 5.0, 0.26)
    handle._prune_buckets()
    sel, notes = handle._select_buckets()
    check(len(sel) == 2, "两桶全部选中")
    check(any("两个桶" in n for n in notes), "notes 记录两桶降级：%s" % notes)

    # ---------- 6. 降级：档位不可辨（两桶同档） ----------
    print("[6] 降级：两桶同档不可辨")
    handle.buckets = []
    make_bucket(handle, 2.0, 0.0, 5.0, 0.20)
    make_bucket(handle, 8.0, 0.0, 5.0, 0.22)
    handle._prune_buckets()
    sel, notes = handle._select_buckets()
    ids = [s["id"] for s in sel]
    check(len(sel) == 2 and len(set(ids)) == 2, "两桶选出且 id 不重复")
    check(any("不可辨" in n for n in notes), "notes 记录档位不可辨：%s" % notes)
    # 最接近 ref 的桶 + 另一个候选（同档位时只记录不确定）
    ratios = {b["id"]: b["ratio"] for b in handle.buckets}
    ref_entry = [s for s in sel if s["reason"] == "closest_to_ref"][0]
    other = [s for s in sel if s is not ref_entry][0]
    check(ratios[other["id"]] != ratios[ref_entry["id"]],
          "另一个候选比值可复核")

    # ---------- 7. 单桶降级 ----------
    print("[7] 降级：只搜索到一个桶")
    handle.buckets = []
    make_bucket(handle, 2.0, 0.0, 5.0, 1.00)
    handle._prune_buckets()
    sel, notes = handle._select_buckets()
    check(len(sel) == 1, "单桶只选 1 个")
    check(any("一个桶" in n for n in notes), "notes 记录单桶降级")

    # ---------- 8. 先近后远排序 ----------
    print("[8] 先近后远（distance 升序）")
    handle.buckets = []
    make_bucket(handle, 20.0, 0.0, 5.0, 1.00)   # 远
    make_bucket(handle, 1.0, 0.0, 5.0, 1.30)    # 近
    handle._prune_buckets()
    sel, _ = handle._select_buckets()
    check(sel[0]["distance_m"] <= sel[1]["distance_m"],
          "第一个投放更近 (%.2f <= %.2f)" % (sel[0]["distance_m"], sel[1]["distance_m"]))

    # ---------- 8b. 整帧一对一关联（错题 #59：近距桶不合并） ----------
    print("[8b] 整帧一对一关联（#59：1.35m 间距三桶不合并）")
    handle.buckets = []
    frame = [(31.5, -1.2, 5.0, 30.0, 120.0, 5.0, 0.15),   # 与 32.5/-0.3 相距 1.35m
             (33.5, 0.8, 5.0, 30.0, 120.0, 5.0, 0.20),
             (32.5, -0.3, 5.0, 30.0, 120.0, 5.0, 0.25)]
    handle._associate_frame(list(frame))
    check(len(handle.buckets) == 3, "一帧 3 条观测 → 3 个桶（旧 1.5m 门限会并成 2 个）")
    handle._associate_frame(list(frame))
    check(all(b["samples_count"] == 2 for b in handle.buckets), "第二帧各自归位（每桶 2 样本）")
    ratios = sorted(b["ratio"] for b in handle.buckets)
    classes = sorted(b["size_class"] for b in handle.buckets)
    check(ratios == [0.15, 0.20, 0.25], "ratio 未被混合：%s" % ratios)
    check(classes == ["large", "mid", "small"], "档位=small/mid/large：%s" % classes)
    sel, _notes = handle._select_buckets()
    sel_ratios = sorted(handle._bucket_by_id(s["id"])["ratio"] for s in sel)
    check(sel_ratios == [0.15, 0.20], "选桶=小号 0.15 + 中号 0.20：%s" % sel_ratios)
    handle.buckets = []
    handle._associate_frame([(10.0, 0.0, 5.0, 30.0, 120.0, 5.0, 0.20)])
    handle._associate_frame([(10.02, 0.01, 5.0, 30.0, 120.0, 5.0, 0.20),
                             (10.03, -0.01, 5.0, 30.0, 120.0, 5.0, 0.20)])
    check(len(handle.buckets) == 1 and handle.buckets[0]["samples_count"] == 3,
          "同桶重复观测仍并入（1 桶 3 样本）")

    # ---------- 9. 记录文件原子写 ----------
    print("[9] 桶记录原子写 + schema")
    handle.buckets = []
    handle.bucket_notes = ["测试降级说明"]
    make_bucket(handle, 1.0, 2.0, 5.0, 1.00)
    make_bucket(handle, 5.0, 2.0, 5.0, 1.30)
    handle._prune_buckets()
    handle.selected_buckets, handle.bucket_notes = handle._select_buckets()
    ok = handle._record_buckets()
    files = sorted(os.listdir(handle.bucket_record_dir))
    check(ok and "buckets_latest.json" in files, "写成功，文件列表=%s" % files)
    latest = os.path.join(handle.bucket_record_dir, "buckets_latest.json")
    with open(latest, "r", encoding="utf-8") as fh:
        payload = json.load(fh)
    print("   latest:", json.dumps(payload, ensure_ascii=False)[:400])
    check(payload.get("schema") == 1, "schema==1")
    check(len(payload.get("buckets", [])) == 2, "buckets 有 2 条")
    check(set(payload["buckets"][0].keys()) == {
        "id", "lat", "lon", "alt", "ratio", "size_class", "diameter_mm",
        "samples", "updated_at", "distance_m"}, "buckets 字段集冻结一致")
    check(payload.get("drop_order") == [s["id"] for s in payload.get("selected", [])],
          "drop_order == selected ids")
    check(payload["mission"]["recon_enabled"] is False, "mission.recon_enabled")
    check("notes" in payload and payload["notes"], "notes 非空")
    # 无残留 tmp
    check(not any(f.endswith(".tmp") for f in files), "无 .tmp 残留")

    # ---------- 10. 配置解析（工作区 YAML 新键） ----------
    print("[10] 配置解析（load_file）")
    values, warns = cfg.load_file("/home/lab/ROS2_CUADC2026/config/auto_drop.yaml")
    for k in ("search_altitude_m", "recon_enabled", "aim_only_mode", "record_buckets",
              "bucket_ratio_ref_200", "bucket_ratio_tol", "bucket_ratio_invert",
              "bucket_assoc_distance_m", "bucket_record_dir"):
        check(k in values, "YAML 解析到 %s=%r" % (k, values.get(k)))
    check(values.get("bucket_record_dir", "").endswith("logs/buckets"),
          "bucket_record_dir 结尾 logs/buckets")

    # ---------- 11. 校验：非法值被拒 ----------
    print("[11] validate 拒绝非法值")
    bad = dict(cfg.effective_defaults())
    bad["bucket_ratio_tol"] = 2.0
    errs, _ = cfg.validate(bad)
    check(any("bucket_ratio_tol" in e for e in errs), "tol=2.0 被拒")
    bad = dict(cfg.effective_defaults())
    bad["bucket_ratio_ref_200"] = 0.0
    errs, _ = cfg.validate(bad)
    check(any("bucket_ratio_ref_200" in e for e in errs), "ref=0 被拒")
    bad = dict(cfg.effective_defaults())
    bad["search_altitude_m"] = 0.0
    errs, _ = cfg.validate(bad)
    check(any("search_altitude_m" in e for e in errs), "search_altitude_m=0 被拒")
    bad = dict(cfg.effective_defaults())
    bad["search_altitude_m"] = 99.0
    errs, _ = cfg.validate(bad)
    check(any("search_altitude_m" in e for e in errs), "search_altitude_m>recon 被拒")

    ok = all(c for c, _ in checks)
    print("\n==== T2 离线结果:", "ALL PASS" if ok else "FAIL",
          "(%d/%d)" % (sum(1 for c, _ in checks if c), len(checks)), "====")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
