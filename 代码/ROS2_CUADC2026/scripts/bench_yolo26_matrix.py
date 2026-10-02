#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_yolo26_matrix.py — 关键配置矩阵基准（重复取稳健值）

背景：本机有桌面/NoMachine/OpenCode 后台负载，单次测量 run-to-run 波动可达 ±35%。
本脚本对每个配置重复 R 次（每次 n 帧、去 warmup），报告各次 mean 的 min/median/max，
以 **min（最干净一次）** 代表"机器安静时可达"，median 代表常态。

用法：
  taskset -c 0-7 python3 scripts/bench_yolo26_matrix.py \
      --reps 3 --n 60 --out /tmp/opencode/bench_matrix.json
"""

import argparse
import json
import os
import statistics
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_yolo26_fps as B  # noqa: E402

M = "/tmp/opencode/bench_models"
REAL_IMG = "/tmp/opencode/frames/d435_frame1.png"
PROVIDED = "/home/lab/ROS2_CUADC2026/src/cuadc_visionY26/models/best_yolo26.onnx"


def model(sz, kind):
    if kind == "fp32":
        return f"{M}/best_y26_{sz}_end2end.onnx"
    if kind == "ortint8":
        return f"{M}/best_y26_{sz}_int8.onnx"
    if kind == "ovint8":
        return f"{M}/best_y26_{sz}_int8_e2e_openvino_model/best.xml"
    raise ValueError(kind)


def configs(include_ultra=True):
    cfgs = []
    for sz in (640, 480, 416):
        cfgs.append(("ORT-FP32-%d" % sz, "ort", model(sz, "fp32"), sz, None, 8))
        cfgs.append(("ORT-INT8-%d" % sz, "ort", model(sz, "ortint8"), sz, None, 8))
        cfgs.append(("OV-FP32-%d" % sz, "ov", model(sz, "fp32"), sz, None, None))
        cfgs.append(("OV-INT8-%d" % sz, "ov", model(sz, "ovint8"), sz, None, None))
    if include_ultra:
        cfgs.append(("ultralytics-FP32-640", "ultra", PROVIDED, 640, None, None))
        cfgs.append(("ultralytics-INT8-640", "ultra",
                     f"{M}/best_y26_640_int8_e2e_openvino_model", 640, None, None))
    return cfgs


def run_once(backend, path, size, threads, mode, n, warmup, frame):
    if backend == "ort":
        s, _ = B.bench_ort(path, size, mode, n, warmup, threads=threads, frame=frame)
    elif backend == "ov":
        s, _ = B.bench_ov(path, size, mode, n, warmup, device="CPU", frame=frame)
    elif backend == "ultra":
        s, _ = B.bench_ultralytics(path, size, n, warmup, frame=frame)
    else:
        raise ValueError(backend)
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--warmup", type=int, default=15)
    ap.add_argument("--mode", default="pure")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    import cv2
    frame = cv2.imread(REAL_IMG) if os.path.exists(REAL_IMG) else None

    rows = {}
    print("%-24s %9s %9s %9s   %s" % ("config", "min_FPS", "med_FPS", "max_FPS", "rep_means_ms"))
    for name, backend, path, size, _, threads in configs():
        if not os.path.exists(path):
            print("%-24s SKIP (missing %s)" % (name, path))
            continue
        means = []
        for _ in range(args.reps):
            s = run_once(backend, path, size, threads, args.mode, args.n, args.warmup, frame)
            means.append(s["mean_ms"])
        fps = [1000.0 / m for m in means]
        rows[name] = {
            "backend": backend, "size": size, "mode": args.mode,
            "rep_mean_ms": means,
            "min_ms": min(means), "median_ms": statistics.median(means), "max_ms": max(means),
            "min_fps": 1000.0 / min(means), "median_fps": 1000.0 / statistics.median(means),
            "max_fps": 1000.0 / max(means),
        }
        print("%-24s %9.2f %9.2f %9.2f   %s"
              % (name, max(fps), statistics.median(fps), min(fps),
                 [round(m, 1) for m in means]))

    if args.out:
        with open(args.out, "w") as f:
            json.dump({"ts": time.strftime("%Y-%m-%dT%H:%M:%S"), "rows": rows}, f, indent=2)
        print("written", args.out)


if __name__ == "__main__":
    main()
