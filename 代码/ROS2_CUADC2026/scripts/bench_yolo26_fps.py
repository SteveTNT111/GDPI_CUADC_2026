#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bench_yolo26_fps.py — YOLO26 ONNX 推理帧率基准（独立于 ROS/相机）

用途：在**空闲机器**上对 best_yolo26.onnx 做可复现的 FPS 基准与优化对比。
所有测量都基于固定输入张量、去 warmup、N 帧统计（mean/median/p95/min/max），
不做 ROS 发布，避免图像消息构造等旁路开销污染推理耗时。

计时口径（三种，务必区分，写进结论文档时注明）：
  pure  —— 仅 session.run / infer（输入张量预分配，不含预处理）
  e2e   —— letterbox 预处理 + 推理 + 解析输出（对齐 ultralytics predict 语义）
  ultra —— 直接调用 ultralytics YOLO.predict（含其前后处理，文档旧基线口径）

后端：
  ort   —— onnxruntime InferenceSession（可扫 intra_op_num_threads）
  ov    —— OpenVINO 原生 API（读 ONNX 或 IR，CPU/GPU 可选，同步/异步）

用法示例：
  # ORT 线程扫描（纯推理）
  taskset -c 0-7 python3 scripts/bench_yolo26_fps.py threads --n 100
  # 三后端对比 + e2e + ultralytics
  taskset -c 0-7 python3 scripts/bench_yolo26_fps.py compare
  # 单点
  python3 scripts/bench_yolo26_fps.py run --backend ort --threads 4 --mode pure
  # OpenVINO（IR 首次自动转换并缓存到 /tmp/opencode）
  python3 scripts/bench_yolo26_fps.py run --backend ov --device CPU --mode pure

设计纪律（呼应项目约定）：
  - 只读源仓库，产物落 ~/ROS2_CUADC2026；本脚本不 import rclpy。
  - 不改交付源码；本脚本仅测量。
  - 日志由调用方 tee 落盘 /tmp/opencode/（#4）。
"""

import argparse
import json
import os
import statistics
import sys
import time

import numpy as np

DEFAULT_MODEL = os.path.expanduser(
    "~/ROS2_CUADC2026/src/cuadc_visionY26/models/best_yolo26.onnx"
)
DEFAULT_IMAGE_SIZE = (640, 480)  # 相机出图尺寸 H,W（bgr8）
IR_CACHE_DIR = "/tmp/opencode/bench_ir"


# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------

def make_frame(h=DEFAULT_IMAGE_SIZE[1], w=DEFAULT_IMAGE_SIZE[0], seed=0):
    """生成一张确定性的合成 bgr8 帧（梯度 + 噪声），保证跨前后完全可复现。"""
    rng = np.random.default_rng(seed)
    gy = np.linspace(0, 255, h, dtype=np.float32)[:, None]
    gx = np.linspace(0, 255, w, dtype=np.float32)[None, :]
    frame = np.empty((h, w, 3), np.float32)
    frame[..., 0] = gx
    frame[..., 1] = gy
    frame[..., 2] = (gx + gy) * 0.5
    frame += rng.normal(0, 8, frame.shape).astype(np.float32)
    return np.clip(frame, 0, 255).astype(np.uint8)


def letterbox(img, new_shape=640, color=(114, 114, 114)):
    """ultralytics 风格 letterbox（保持比例 + 灰边），返回 (img, r, (dw,dh))。"""
    import cv2
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    h, w = img.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    new_unpad = (int(round(w * r)), int(round(h * r)))
    dw, dh = (new_shape[1] - new_unpad[0]) / 2, (new_shape[0] - new_unpad[1]) / 2
    if (w, h) != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return np.ascontiguousarray(img), r, (dw, dh)


def preprocess(frame_bgr, size):
    """bgr8 → NCHW float32 RGB /255（ORT 输入张量）。"""
    import cv2
    img, _, _ = letterbox(frame_bgr, size)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    x = img.transpose(2, 0, 1).astype(np.float32) / 255.0
    return np.ascontiguousarray(x[None, ...])


def decode_end2end(out, conf_thr):
    """解析 NMS-free 端到端输出 [1,300,6] → Nx6 (x1,y1,x2,y2,conf,cls)。"""
    rows = np.asarray(out).reshape(-1, 6)
    return rows[rows[:, 4] >= conf_thr]


def nms_numpy(boxes, scores, iou_thr=0.45):
    """简单 NMS（用于 84x8400 非端到端模型的后处理计时）。"""
    if boxes.size == 0:
        return np.empty(0, dtype=np.int64)
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_thr]
    return np.asarray(keep, dtype=np.int64)


def stats(times_ms):
    t = np.asarray(times_ms, dtype=np.float64)
    return {
        "n": int(t.size),
        "mean_ms": float(t.mean()),
        "median_ms": float(np.median(t)),
        "p95_ms": float(np.percentile(t, 95)),
        "min_ms": float(t.min()),
        "max_ms": float(t.max()),
        "fps_mean": float(1000.0 / t.mean()),
        "fps_median": float(1000.0 / np.median(t)),
        "stdev_ms": float(statistics.pstdev(t)) if t.size > 1 else 0.0,
    }


def fmt_stats(tag, s):
    return ("%-34s n=%3d  mean=%7.2fms (%5.2f FPS)  median=%7.2f  "
            "p95=%7.2f  min=%7.2f  max=%7.2f  sd=%5.2f"
            % (tag, s["n"], s["mean_ms"], s["fps_mean"], s["median_ms"],
               s["p95_ms"], s["min_ms"], s["max_ms"], s["stdev_ms"]))


# ---------------------------------------------------------------------------
# ORT
# ---------------------------------------------------------------------------

def load_ort(path, threads=None, providers=("CPUExecutionProvider",)):
    import onnxruntime as ort
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    if threads is not None:
        so.intra_op_num_threads = int(threads)
        so.inter_op_num_threads = 1
    sess = ort.InferenceSession(path, sess_options=so, providers=list(providers))
    return sess


def bench_ort(path, size, mode, n, warmup, threads=None,
              providers=("CPUExecutionProvider",), frame=None, conf_thr=0.5):
    sess = load_ort(path, threads, providers)
    iname = sess.get_inputs()[0].name
    if size is None:
        shp = sess.get_inputs()[0].shape
        size = shp[2] if isinstance(shp[2], int) else 640
    if mode == "pure":
        x = np.zeros((1, 3, size, size), np.float32)
        def one():
            return sess.run(None, {iname: x})
    elif mode == "e2e":
        fr = frame if frame is not None else make_frame()
        x = preprocess(fr, size)
        def one():
            out = sess.run(None, {iname: x})[0]
            return decode_end2end(out, conf_thr)
    else:
        raise ValueError("mode must be pure/e2e for ort")
    for _ in range(warmup):
        one()
    times = []
    for _ in range(n):
        t0 = time.perf_counter()
        one()
        times.append((time.perf_counter() - t0) * 1000.0)
    return stats(times), sess


# ---------------------------------------------------------------------------
# OpenVINO（原生 API；不依赖 onnxruntime-openvino）
# ---------------------------------------------------------------------------

def ov_ir_path(onnx_path, size):
    os.makedirs(IR_CACHE_DIR, exist_ok=True)
    base = os.path.splitext(os.path.basename(onnx_path))[0]
    return os.path.join(IR_CACHE_DIR, "%s_%d.xml" % (base, size or 0))


def load_ov(onnx_path, size, device="CPU", cache_dir=IR_CACHE_DIR):
    import openvino as ov
    core = ov.Core()
    # 已量化/已转好的 IR（.xml）直读，不再做 ONNX→IR 转换/缓存
    if onnx_path.endswith(".xml") and os.path.exists(onnx_path):
        model = core.read_model(onnx_path)
        compiled = core.compile_model(model, device)
        return core, compiled, model
    xml = ov_ir_path(onnx_path, size)
    if os.path.exists(xml):
        model = core.read_model(xml)
    else:
        model = core.read_model(onnx_path)
        if size is not None:
            try:
                model.reshape({model.input(0).any_name: [1, 3, size, size]})
            except Exception as exc:
                print("[ov] reshape %s failed: %s (keep original)" % (size, exc))
        os.makedirs(cache_dir, exist_ok=True)
        ov.save_model(model, xml, compress_to_fp16=False)
    compiled = core.compile_model(model, device)
    return core, compiled, model


def bench_ov(model_path, size, mode, n, warmup, device="CPU",
             frame=None, conf_thr=0.5, async_depth=2):
    _, compiled, model = load_ov(model_path, size)
    if size is None:
        shp = model.input(0).shape
        size = shp[2] if isinstance(shp[2], int) else 640
    ireq = compiled.create_infer_request()
    if mode == "pure":
        x = np.zeros((1, 3, size, size), np.float32)
    elif mode == "e2e":
        fr = frame if frame is not None else make_frame()
        x = preprocess(fr, size)
    else:
        raise ValueError("mode must be pure/e2e for ov")

    def one():
        out = ireq.infer([x])
        if mode == "e2e":
            arr = list(out.values())[0]
            return decode_end2end(arr, conf_thr)
        return None

    for _ in range(warmup):
        one()
    times = []
    for _ in range(n):
        t0 = time.perf_counter()
        one()
        times.append((time.perf_counter() - t0) * 1000.0)
    return stats(times), compiled


# ---------------------------------------------------------------------------
# ultralytics（文档旧基线口径：含其前后处理）
# ---------------------------------------------------------------------------

def bench_ultralytics(model_path, size, n, warmup, frame=None, conf_thr=0.5):
    os.environ.setdefault("ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS", "1")
    from ultralytics import YOLO
    fr = frame if frame is not None else make_frame()
    m = YOLO(model_path, task="detect")
    for _ in range(warmup):
        m(fr, conf=conf_thr, imgsz=size, device="cpu", verbose=False)
    times = []
    for _ in range(n):
        t0 = time.perf_counter()
        m(fr, conf=conf_thr, imgsz=size, device="cpu", verbose=False)
        times.append((time.perf_counter() - t0) * 1000.0)
    return stats(times), m


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def model_size(path):
    try:
        import onnx
        m = onnx.load(path, load_external_data=False)
        d = m.graph.input[0].type.tensor_type.shape.dim
        h, w = d[2].dim_value, d[3].dim_value
        return h if h and h == w else None
    except Exception:
        return None


def load_frame_arg(args):
    if args.image:
        import cv2
        fr = cv2.imread(os.path.expanduser(args.image))
        if fr is None:
            print("[warn] cannot read image %s; use synthetic" % args.image)
        return fr
    return None


def cmd_run(args):
    frame = load_frame_arg(args)
    result = {}
    if args.backend == "ort":
        s, _ = bench_ort(args.model, args.size, args.mode, args.n, args.warmup,
                         threads=args.threads, frame=frame, conf_thr=args.conf)
        tag = "ort t=%s %s" % (args.threads, args.mode)
    elif args.backend == "ov":
        s, _ = bench_ov(args.model, args.size, args.mode, args.n, args.warmup,
                        device=args.device, frame=frame, conf_thr=args.conf)
        tag = "ov %s %s" % (args.device, args.mode)
    elif args.backend == "ultralytics":
        s, _ = bench_ultralytics(args.model, args.size or 640, args.n, args.warmup,
                                 frame=frame, conf_thr=args.conf)
        tag = "ultralytics %s" % args.mode
    else:
        raise SystemExit("unknown backend")
    print(fmt_stats(tag, s))
    result[tag] = s
    if args.json:
        with open(args.json, "w") as f:
            json.dump(result, f, indent=2)
    return s


def cmd_threads(args):
    frame = load_frame_arg(args)
    threads_list = [int(x) for x in args.threads_list.split(",")]
    print("== ORT intra_op_num_threads 扫描 (mode=%s, model=%s) ==" % (args.mode, args.model))
    print("   cpu_count=%d  OMP_NUM_THREADS=%s" % (os.cpu_count(), os.environ.get("OMP_NUM_THREADS")))
    results = {}
    for t in threads_list + [None]:
        s, _ = bench_ort(args.model, args.size, args.mode, args.n, args.warmup,
                         threads=t, frame=frame, conf_thr=args.conf)
        tag = "threads=%s" % ("default" if t is None else t)
        results[tag] = s
        print(fmt_stats(tag, s))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(results, f, indent=2)
    return results


def cmd_compare(args):
    frame = load_frame_arg(args)
    size = args.size or model_size(args.model) or 640
    print("== 三后端对比 (model=%s size=%s) ==" % (args.model, size))
    out = {}
    for threads in [4, 6, 8, None]:
        s, _ = bench_ort(args.model, size, "pure", args.n, args.warmup, threads=threads)
        tag = "ort_pure_t%s" % ("def" if threads is None else threads)
        out[tag] = s
        print(fmt_stats(tag, s))
    for mode in ["pure", "e2e"]:
        s, _ = bench_ov(args.model, size, mode, args.n, args.warmup, device=args.device)
        tag = "ov_%s_%s" % (args.device, mode)
        out[tag] = s
        print(fmt_stats(tag, s))
    s, _ = bench_ultralytics(args.model, size, args.n, args.warmup)
    out["ultralytics"] = s
    print(fmt_stats("ultralytics", s))
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
    return out


def build_parser():
    p = argparse.ArgumentParser(description="YOLO26 ONNX 推理帧率基准")
    p.add_argument("--model", default=DEFAULT_MODEL)
    p.add_argument("--json", default=None, help="结果写 JSON")
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp):
        sp.add_argument("--model", default=DEFAULT_MODEL)
        sp.add_argument("--json", default=None)
        sp.add_argument("--n", type=int, default=100)
        sp.add_argument("--warmup", type=int, default=20)
        sp.add_argument("--size", type=int, default=None)
        sp.add_argument("--mode", choices=["pure", "e2e"], default="pure")
        sp.add_argument("--conf", type=float, default=0.5)
        sp.add_argument("--image", default=None, help="真实帧(可选)，默认合成图")
        sp.add_argument("--device", default="CPU")

    r = sub.add_parser("run")
    common(r)
    r.add_argument("--backend", choices=["ort", "ov", "ultralytics"], default="ort")
    r.add_argument("--threads", type=int, default=None)
    r.set_defaults(func=cmd_run)

    t = sub.add_parser("threads")
    common(t)
    t.add_argument("--threads-list", default="1,2,4,6,8,16")
    t.set_defaults(func=cmd_threads)

    c = sub.add_parser("compare")
    common(c)
    c.set_defaults(func=cmd_compare)
    return p


def main():
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
