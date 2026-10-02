#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ros_probe.py — Web UI 的 ROS 状态探针（常驻，逐行输出 JSON）。

给 webui/server.py 读取，用于状态区：
  - /mavros/state 的 connected/armed/mode：**VOLATILE 实时订阅**（错题 #16c：
    不用 `ros2 topic echo --once`，那是 transient_local 缓存假象）。
  - 流实据：pose / compass_hdg / rel_alt / vision color / servo/ready 的窗口内 Hz。
  - state 的 age（新鲜度）：断链后 state 停发，UI 据此报"陈旧/异常"，不误报已连接。
  - 实时桶数量：订阅 `/vision/target_global`(GeoTarget) 的 `count`（T1 冻结契约：
    每帧每目标一条，count=本帧全部目标数）。**不消费**被停 agent 遗留的
    `/vision/bucket/estimates`(BucketEstimates) —— 其 ratio 定义与 T2 契约不同（见 AGENTS 落地注记）。
  - 识别画面（带标注）：**仅当存在观看者**（server 落 `webui_stream_on` 标记）时，才订阅
    `/vision/annotated_image` 并把最新帧编码 JPEG 原子写 `webui_frame.jpg`，供 server 以 MJPEG 推流。
    无观看者即退订 → 不触发识别节点发布标注大图（错题 #45：别让探针无谓拖慢相机）。

用法: python3 ros_probe.py [--interval 2.0]
输出: 每 interval 一行 JSON（stdout, flush）。
铁律：链路真伪只看流实据（Hz），state 仅作显示。
"""

import argparse
import json
import os
import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from mavros_msgs.msg import GPSRAW, VfrHud
from sensor_msgs.msg import BatteryState, Image, NavSatFix
from std_msgs.msg import Bool, Float64

try:  # 轻量目标话题（仅数量/小结构，非大图，#45）
    from cuadc_vision.msg import GeoTarget
except Exception:  # 识别包未构建/未 source 时不致命，探针其余状态照常
    GeoTarget = None

try:  # 识别画面（带标注）转 JPEG 用；缺 cv2 时视频功能降级，其余状态照常
    import cv2
    import numpy as np
except Exception:  # pragma: no cover
    cv2 = None
    np = None

NAV_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
STATE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
IMG_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
READY_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

LOGDIR = "/tmp/opencode"
FRAME_FILE = os.path.join(LOGDIR, "webui_frame.jpg")   # 最新标注帧（server 读它推流）
STREAM_FLAG = os.path.join(LOGDIR, "webui_stream_on")   # 有人看画面时由 server 落此标记

TOPICS = {
    "/mavros/local_position/pose": "geometry_msgs/msg/PoseStamped",
    "/mavros/global_position/compass_hdg": "std_msgs/msg/Float64",
    "/mavros/global_position/rel_alt": "std_msgs/msg/Float64",
    "/vision/color/image_raw": "sensor_msgs/msg/Image",
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("webui_ros_probe")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()

    lock = threading.Lock()
    counts = {t: 0 for t in TOPICS}
    state = {"msg": None, "at": 0.0}
    servo_ready = {"value": None, "at": 0.0}
    bucket = {"count": None, "dx": None, "dy": None, "at": 0.0, "targets": {}}
    fc = {}   # 飞控值（供电量/高度/GPS/NED），供 WebUI「飞机状态栏」

    def make_cb(topic):
        def _cb(_msg):
            with lock:
                counts[topic] += 1
        return _cb

    def pose_cb(msg):
        with lock:
            counts["/mavros/local_position/pose"] += 1
            try:
                fc["pose"] = {"x": float(msg.pose.position.x), "y": float(msg.pose.position.y),
                              "z": float(msg.pose.position.z), "at": time.monotonic()}
            except Exception:
                pass
    node.create_subscription(PoseStamped, "/mavros/local_position/pose", pose_cb, NAV_QOS)
    def hdg_cb(msg):
        with lock:
            counts["/mavros/global_position/compass_hdg"] += 1
            try:
                fc["heading"] = float(msg.data) % 360.0
                fc["heading_at"] = time.monotonic()
            except Exception:
                pass
    node.create_subscription(
        Float64, "/mavros/global_position/compass_hdg", hdg_cb, NAV_QOS)

    def vfr_cb(msg):
        with lock:
            try:
                fc["vfr_heading"] = float(msg.heading) % 360.0
                fc["vfr_heading_at"] = time.monotonic()
            except Exception:
                pass
    node.create_subscription(VfrHud, "/mavros/vfr_hud", vfr_cb, NAV_QOS)

    def rel_alt_cb(msg):
        with lock:
            counts["/mavros/global_position/rel_alt"] += 1
            try:
                fc["rel_alt"] = float(msg.data)
                fc["rel_alt_at"] = time.monotonic()
            except Exception:
                pass
    node.create_subscription(Float64, "/mavros/global_position/rel_alt", rel_alt_cb, NAV_QOS)
    node.create_subscription(
        Image, "/vision/color/image_raw", make_cb("/vision/color/image_raw"), NAV_QOS)

    def batt_cb(msg):
        with lock:
            try:
                fc["voltage"] = float(msg.voltage)
                fc["current"] = float(msg.current)
                fc["batt_at"] = time.monotonic()
            except Exception:
                pass
    node.create_subscription(BatteryState, "/mavros/battery", batt_cb, NAV_QOS)

    def gps_cb(msg):
        with lock:
            try:
                fc["lat"] = float(msg.latitude)
                fc["lon"] = float(msg.longitude)
                fc["alt"] = float(msg.altitude)
                fc["gps_status"] = int(msg.status.status)   # NavSatStatus.status（int8）
                fc["gps_at"] = time.monotonic()
            except Exception:
                pass
    node.create_subscription(NavSatFix, "/mavros/global_position/global", gps_cb, NAV_QOS)

    def gpsraw_cb(msg):
        with lock:
            try:
                fc["satellites"] = int(msg.satellites_visible)
                fc["fix_type"] = int(msg.fix_type)
                fc["gpsraw_at"] = time.monotonic()
            except Exception:
                pass
    node.create_subscription(GPSRAW, "/mavros/gpsstatus/gps1/raw", gpsraw_cb, NAV_QOS)

    def state_cb(msg):
        with lock:
            state["msg"] = msg
            state["at"] = time.monotonic()
    node.create_subscription(State, "/mavros/state", state_cb, STATE_QOS)

    def ready_cb(msg):
        with lock:
            servo_ready["value"] = bool(msg.data)
            servo_ready["at"] = time.monotonic()
    node.create_subscription(Bool, "/servo/ready", ready_cb, READY_QOS)

    def _geo_target_cb(msg):
        # T1 契约：GeoTarget.count = 本帧识别到的桶总数（每帧每条消息相同）。
        # 本回调同时缓存 target_id 1/2/3 的完整解（含双投放点 front/rear），供 WebUI 桶面板。
        now = time.monotonic()
        with lock:
            try:
                bucket["count"] = int(msg.count)
            except (TypeError, ValueError):
                bucket["count"] = 0
            bucket["at"] = now
            tid = int(getattr(msg, "target_id", 0) or 0)
            if tid <= 0:
                return

            def trip(*names):
                return [round(float(getattr(msg, n, 0.0) or 0.0), 7) for n in names]

            bucket["targets"][tid] = {
                "id": tid,
                "count": int(getattr(msg, "count", 0) or 0),
                "valid": bool(getattr(msg, "valid", False)),
                "status": str(getattr(msg, "status", "")),
                "class_name": str(getattr(msg, "class_name", "")),
                "confidence": round(float(getattr(msg, "confidence", 0.0) or 0.0), 3),
                "center": [int(getattr(msg, "center_x", 0) or 0),
                           int(getattr(msg, "center_y", 0) or 0)],
                "camera": trip("camera_x_m", "camera_y_m", "camera_z_m"),
                "body": trip("body_x_m", "body_y_m", "body_z_m"),
                "front_cam": trip("front_cam_x_m", "front_cam_y_m", "front_cam_z_m"),
                "rear_cam": trip("rear_cam_x_m", "rear_cam_y_m", "rear_cam_z_m"),
                "front_ned": trip("front_ned_n_m", "front_ned_e_m", "front_ned_d_m"),
                "rear_ned": trip("rear_ned_n_m", "rear_ned_e_m", "rear_ned_d_m"),
                "front_wgs84": trip("front_latitude", "front_longitude", "front_altitude"),
                "rear_wgs84": trip("rear_latitude", "rear_longitude", "rear_altitude"),
                "size_proxy_m": round(float(getattr(msg, "size_proxy_m", 0.0) or 0.0), 4),
                "at": now,
            }

    if GeoTarget is not None:
        node.create_subscription(GeoTarget, "/vision/target_global",
                                 _geo_target_cb, NAV_QOS)

    # ---- 识别画面（带标注）：仅当有人观看（STREAM_FLAG 存在）时才订阅 ----
    ann = {"sub": None, "at": 0.0, "count": 0}

    def _encode_jpeg(msg):
        if cv2 is None or np is None:
            return None
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == "bgr8":
                frame = raw.reshape((msg.height, msg.step))[:, :msg.width * 3] \
                           .reshape((msg.height, msg.width, 3))
            elif msg.encoding == "rgb8":
                frame = cv2.cvtColor(
                    raw.reshape((msg.height, msg.step))[:, :msg.width * 3]
                       .reshape((msg.height, msg.width, 3)),
                    cv2.COLOR_RGB2BGR)
            else:
                return None
            ok, enc = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
            return enc.tobytes() if ok else None
        except Exception:
            return None

    def _ann_cb(msg):
        now = time.monotonic()
        with lock:
            if now - ann["at"] < 0.12:          # 限 ~8fps，别拖慢识别
                return
            ann["at"] = now
            ann["count"] += 1
        data = _encode_jpeg(msg)
        if not data:
            return
        try:
            tmp = FRAME_FILE + ".tmp"
            with open(tmp, "wb") as fh:
                fh.write(data)
            os.replace(tmp, FRAME_FILE)
        except OSError:
            pass

    def _stream_wanted():
        # server 推流期间每个循环 utime 标记文件；mtime 超过 3s 即视为无人看
        # （即便流线程卡在阻塞写也能可靠退订）。st_mtime 是墙钟，用 time.time() 比。
        try:
            return (time.time() - os.stat(STREAM_FLAG).st_mtime) < 3.0
        except OSError:
            return False

    def _ensure_annotated():
        want = _stream_wanted()
        with lock:
            sub = ann["sub"]
            if want and sub is None and cv2 is not None:
                try:
                    ann["sub"] = node.create_subscription(
                        Image, "/vision/annotated_image", _ann_cb, IMG_QOS)
                except Exception:
                    ann["sub"] = None
            elif not want and sub is not None:
                try:
                    node.destroy_subscription(sub)
                except Exception:
                    pass
                ann["sub"] = None
                ann["at"] = 0.0

    stop = {"flag": False}
    signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT, signal.SIGTERM})

    def _handler(signum, frame):
        stop["flag"] = True
        if rclpy.ok():
            rclpy.try_shutdown()
    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT, signal.SIGTERM})

    interval = max(0.5, args.interval)
    try:
        while rclpy.ok() and not stop["flag"]:
            time.sleep(interval)
            now = time.monotonic()
            _ensure_annotated()
            with lock:
                snap = dict(counts)
                st = state["msg"]
                st_at = state["at"]
                sr = dict(servo_ready)
                bk = dict(bucket)
                fcs = dict(fc)
                targets_snap = {k: dict(v) for k, v in bucket["targets"].items()}
                ann_at = ann["at"]
                ann_cnt = ann["count"]
                ann_on = ann["sub"] is not None
                ann["count"] = 0
                for t in counts:
                    counts[t] = 0
            payload = {
                "t": now,
                "interval": interval,
                "hz": {t: round(n / interval, 2) for t, n in snap.items()},
                "state": {
                    "connected": bool(st.connected) if st is not None else None,
                    "armed": bool(st.armed) if st is not None else None,
                    "mode": str(st.mode) if st is not None else None,
                    "age_s": round(now - st_at, 2) if st_at else None,
                },
                "servo_ready": sr["value"],
                "servo_ready_age_s": (round(now - sr["at"], 2) if sr["at"] else None),
                "bucket_count": bk["count"],
                "bucket_count_age_s": (round(now - bk["at"], 2) if bk["at"] else None),
                "stream_on": ann_on,
                "frame_age_s": (round(now - ann_at, 2) if ann_at else None),
                "frame_hz": (round(ann_cnt / interval, 2) if ann_on else 0.0),
                "frame_available": bool(ann_at),
                "fc": {
                    "rel_alt": fcs.get("rel_alt"),
                    "rel_alt_age_s": (round(now - fcs["rel_alt_at"], 2)
                                      if fcs.get("rel_alt_at") else None),
                    "heading": (fcs.get("heading") if fcs.get("heading") is not None
                                else fcs.get("vfr_heading")),
                    "heading_age_s": (round(now - fcs["heading_at"], 2)
                                      if fcs.get("heading_at")
                                      else (round(now - fcs["vfr_heading_at"], 2)
                                            if fcs.get("vfr_heading_at") else None)),
                    "heading_src": ("compass_hdg" if fcs.get("heading") is not None
                                    else ("vfr_hud" if fcs.get("vfr_heading") is not None
                                          else None)),
                    "voltage": fcs.get("voltage"),
                    "current": fcs.get("current"),
                    "satellites": fcs.get("satellites"),
                    "fix_type": fcs.get("fix_type"),
                    "lat": fcs.get("lat"),
                    "lon": fcs.get("lon"),
                    "alt": fcs.get("alt"),
                    "pose_age_s": (round(now - fcs["pose"]["at"], 2)
                                   if fcs.get("pose") else None),
                    "ned": ({"n": round(fcs["pose"]["y"], 2),
                             "e": round(fcs["pose"]["x"], 2),
                             "d": round(-fcs["pose"]["z"], 2)}
                            if fcs.get("pose") else None),
                },
                "buckets_live": [
                    dict(t, age_s=round(now - t["at"], 2))
                    for _tid, t in sorted(targets_snap.items())
                    if (now - t["at"]) < 1.5
                ],
            }
            try:
                sys.stdout.write(json.dumps(payload) + "\n")
                sys.stdout.flush()
            except BrokenPipeError:
                # 读取端（服务器）已关闭：静默退出，避免 Traceback（房规：0 Traceback）。
                try:
                    devnull = os.open(os.devnull, os.O_WRONLY)
                    os.dup2(devnull, sys.stdout.fileno())
                except OSError:
                    pass
                break
            except OSError:
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            ex.shutdown(2.0)
            spin_thread.join(timeout=2.0)
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
