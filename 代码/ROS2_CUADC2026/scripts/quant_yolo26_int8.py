#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""quant_yolo26_int8.py — YOLO26 ONNX → INT8 量化（ORT 静态 QDQ / OpenVINO NNCF）

CPU 为 i5-1340P（AVX2 + AVX-VNNI），INT8 可借 VNNI 提速。本脚本只做量化并落盘，
计时/精度分别用 scripts/bench_yolo26_fps.py 与 ultralytics val 复核。

子命令：
  ort  —— onnxruntime 静态量化（QDQ, per-channel），输出 *_int8.onnx
  ov   —— OpenVINO NNCF INT8，输出 *_int8.xml/.bin（需 pip install --user nncf）
  gen-calib —— 仅生成一份校准 npy（默认从 bucket 验证集取图）

用法：
  python3 scripts/quant_yolo26_int8.py ort --model M.onnx --out /tmp/opencode/x_int8.onnx \
      --calib-dir /tmp/opencode/bucket_val/images/val --size 640 --num 200
"""

import argparse
import glob
import os

import numpy as np

DEFAULT_CALIB = "/tmp/opencode/bucket_val/images/val"


def preprocess_path(path, size):
    import cv2
    img = cv2.imread(path)
    if img is None:
        return None
    img, _, _ = _letterbox(img, size)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(img.transpose(2, 0, 1).astype(np.float32) / 255.0)[None, ...]


def _letterbox(img, new_shape, color=(114, 114, 114)):
    import cv2
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    h, w = img.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    unpad = (int(round(w * r)), int(round(h * r)))
    dw, dh = (new_shape[1] - unpad[0]) / 2, (new_shape[0] - unpad[1]) / 2
    if (w, h) != unpad:
        img = cv2.resize(img, unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return np.ascontiguousarray(img), r, (dw, dh)


def iter_calib(calib_dir, size, num, exts=(".jpg", ".jpeg", ".png", ".bmp")):
    files = []
    for e in exts:
        files += glob.glob(os.path.join(calib_dir, "*" + e))
        files += glob.glob(os.path.join(calib_dir, "*" + e.upper()))
    files = sorted(set(files))[:num]
    if not files:
        raise SystemExit("no calibration images in %s" % calib_dir)
    for f in files:
        x = preprocess_path(f, size)
        if x is not None:
            yield x


def cmd_ort(args):
    from onnxruntime.quantization import (
        CalibrationDataReader, CalibrationMethod, QuantFormat, QuantType,
        quantize_static,
    )
    import onnx

    g = onnx.load(args.model, load_external_data=False)
    iname = g.graph.input[0].name

    imgs = list(iter_calib(args.calib_dir, args.size, args.num))

    class Reader(CalibrationDataReader):
        def __init__(self):
            self.it = iter(imgs)

        def get_next(self):
            try:
                return {iname: next(self.it)}
            except StopIteration:
                return None

    quantize_static(
        args.model, args.out, Reader(),
        quant_format=QuantFormat.QDQ,
        per_channel=True,
        activation_type=QuantType.QUInt8,
        weight_type=QuantType.QInt8,
        calibrate_method=CalibrationMethod.MinMax,
    )
    print("ORT INT8 written:", args.out, "%.1fMB" % (os.path.getsize(args.out) / 1e6))


def cmd_ov(args):
    import openvino as ov
    import nncf

    core = ov.Core()
    ov_model = core.read_model(args.model)
    if args.size:
        try:
            ov_model.reshape({ov_model.input(0).any_name: [1, 3, args.size, args.size]})
        except Exception as exc:
            print("[ov] reshape skip:", exc)

    imgs = list(iter_calib(args.calib_dir, args.size, args.num))

    def transform(item):
        return item

    dataset = nncf.Dataset(imgs, transform)
    quantized = nncf.quantize(ov_model, dataset, subset_size=min(len(imgs), 200))
    xml = args.out if args.out.endswith(".xml") else args.out + ".xml"
    ov.save_model(quantized, xml, compress_to_fp16=False)
    print("OpenVINO INT8 written:", xml)


def build_parser():
    p = argparse.ArgumentParser(description="YOLO26 INT8 量化")
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp):
        sp.add_argument("--model", required=True)
        sp.add_argument("--out", required=True)
        sp.add_argument("--size", type=int, default=640)
        sp.add_argument("--calib-dir", default=DEFAULT_CALIB)
        sp.add_argument("--num", type=int, default=200)

    o = sub.add_parser("ort"); common(o); o.set_defaults(func=cmd_ort)
    v = sub.add_parser("ov"); common(v); v.set_defaults(func=cmd_ov)
    return p


def main():
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
