#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""server.py — CUADC 桌面 Web UI 控制台后端（纯标准库，零依赖、离线可用）。

职责：
  - 静态页服务（webui/static）。
  - 配置读写：GET/POST /api/config（校验 -> 备份 -> 原子写 config/auto_drop.yaml）。
  - 进程启停：自维护 pid 表，setsid 起、killpg 停（**按 pid 精确，禁用 pkill**）。
  - 状态区：读常驻 ros_probe.py 的 JSON（流实据 Hz + state 仅显示，#16c）。
  - 舵机指令：ros2 topic pub --once /servo/cmd。
  - 一键体检 / 一键拉起仿真。

安全（房规）：
  - 默认绑定 0.0.0.0 + 口令 token（LAN 访问；见 webui_up.sh 生成 config/webui.token）。
  - 真机动作（mavros_real / mission）要求请求体 confirm=true（前端二次确认）。
  - 停止只允许：本服务拉起过的 pid，或命令行匹配白名单正则的 pid；一律 killpg。
  - 日志落盘 /tmp/opencode/webui_<name>.log。

用法: python3 server.py [--host 0.0.0.0] [--port 8765] [--token XXX]
"""

import argparse
import glob
import json
import os
import re
import secrets
import select
import shlex
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

WS = os.path.expanduser("~/ROS2_CUADC2026")
CONFIG = os.path.join(WS, "config", "auto_drop.yaml")
TOKEN_FILE = os.path.join(WS, "config", "webui.token")
BACKUP_DIR = os.path.join(WS, "logs")
LOGDIR = "/tmp/opencode"
STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
# 识别画面（带标注）推流：ros_probe 写最新帧，server 读它做 MJPEG；
# 有人看画面时落 STREAM_FLAG（探针据此才订阅 /vision/annotated_image，避免无谓负载）。
FRAME_FILE = os.path.join(LOGDIR, "webui_frame.jpg")
STREAM_FLAG = os.path.join(LOGDIR, "webui_stream_on")
PLACEHOLDER_JPEG = os.path.join(STATIC_DIR, "no_signal.jpg")

# auto_drop_config：优先安装产物，其次源码树。
for _cand in (
    os.path.join(WS, "install", "cuadc_control", "lib", "cuadc_control"),
    os.path.join(WS, "src", "cuadc_control", "scripts"),
):
    if os.path.isfile(os.path.join(_cand, "auto_drop_config.py")) and _cand not in sys.path:
        sys.path.insert(0, _cand)
try:
    import auto_drop_config as cfgmod
except Exception as exc:  # pragma: no cover
    sys.stderr.write("无法导入 auto_drop_config: %s\n" % exc)
    raise

# 外部进程（非本服务拉起）的精确识别模式（正则匹配 /proc/<pid>/cmdline）。
DETECT = {
    "sitl": r"/build/sitl/bin/arducopter/",
    "gz_server": r"/gz sim -s",
    "gz_gui": r"/gz sim -g",
    "mavros": r"/ros2 launch mavros|mavros_node",
    # 仿真/真机区分：sim 走 tcp 5760 或 sim 启动器；real 走 udp 或 real 启动器
    "mavros_sim": r"run_sim_mavros\.sh|apm\.launch.*tcp://127\.0\.0\.1:5760",
    "mavros_real": r"run_real_fc_mavros\.sh|apm\.launch.*udp://",
    "camera": r"/camera_node\.py|cuadc_visionY26 camera_node",
    "detector": r"/detector_node\.py|detector_node\.launch",
    "servo_test": r"/servo_test\.py",
    "mission": r"/auto_drop_test\.py",
    "rc_center": r"/sitl_rc_center\.py",
    "ros_probe": r"/ros_probe\.py",
}
# 桶记录（P0 §六.5 冻结契约，T2 写 / T3 读）：
#   logs/buckets/buckets_latest.json（实时）+ logs/buckets/buckets_<ts>.json（历史）
# 兼容旧的 logs/bucket_*.json（早期约定）。
BUCKET_DIR = os.path.join(BACKUP_DIR, "buckets")
BUCKET_LATEST = os.path.join(BUCKET_DIR, "buckets_latest.json")
BUCKET_GLOB = os.path.join(BUCKET_DIR, "buckets_*.json")
BUCKET_LEGACY_GLOB = os.path.join(BACKUP_DIR, "bucket_*.json")
BUCKET_DIAMETERS_MM = (150, 200, 250)
SIZE_CLASS_MM = {"small": 150, "s": 150, "mid": 200, "m": 200, "medium": 200,
                 "large": 250, "l": 250}

DANGEROUS = {"mavros_real", "mission"}
SERVO_ACTIONS = {
    "a_on": "A on", "a_off": "A off", "b_on": "B on", "b_off": "B off",
    "qdfs": "QDFS", "all_off": "all off", "status": "status",
}


def log_path(name):
    safe = re.sub(r"[^A-Za-z0-9_.-]", "_", name)
    return os.path.join(LOGDIR, "webui_%s.log" % safe)


def ts():
    return time.strftime("%Y-%m-%d_%H%M%S")


def read_cfg():
    values, _ = cfgmod.load_file(CONFIG)
    merged = cfgmod.builtin_defaults()
    merged.update(values)
    return merged


def _fnum(value):
    """宽松转 float；失败返回 None（用于正常化外部记录字段）。"""
    try:
        if value is None or value == "":
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _normalize_bucket(b, idx):
    """正常化单个桶记录：补算 bbox 对角线、直径档位；容错缺失/异构字段名。"""
    bbox = b.get("bbox") or b.get("bbox_px")
    if bbox is None and all(k in b for k in ("x_min", "y_min", "x_max", "y_max")):
        bbox = [b.get("x_min"), b.get("y_min"), b.get("x_max"), b.get("y_max")]
    diag = _fnum(b.get("bbox_diagonal_px") if b.get("bbox_diagonal_px") is not None
                 else b.get("diag_px"))
    if diag is None and isinstance(bbox, (list, tuple)) and len(bbox) == 4:
        x1, y1, x2, y2 = (_fnum(v) for v in bbox)
        if None not in (x1, y1, x2, y2):
            diag = round(((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5, 2)
    diameter_mm = _fnum(b.get("diameter_mm") if b.get("diameter_mm") is not None
                        else b.get("estimated_diameter_mm"))
    if diameter_mm is None:
        est_m = _fnum(b.get("estimated_diameter_m"))
        if est_m is not None:
            diameter_mm = round(est_m * 1000.0, 1)
    size = b.get("size") or b.get("size_class") or b.get("class") or b.get("diameter_class")
    class_mm = SIZE_CLASS_MM.get(str(size).strip().lower()) if size is not None else None
    if diameter_mm is None and class_mm is not None:
        diameter_mm = float(class_mm)
    if class_mm is None and diameter_mm is not None:
        class_mm = min(BUCKET_DIAMETERS_MM, key=lambda c: abs(c - diameter_mm))
    return {
        "id": str(b.get("id") or b.get("name") or b.get("target_id")
                  or ("bucket_%d" % (idx + 1))),
        "lat": b.get("lat", b.get("latitude")),
        "lon": b.get("lon", b.get("longitude")),
        "alt": b.get("alt", b.get("altitude")),
        "depth_m": b.get("depth_m"),
        "bbox": list(bbox) if isinstance(bbox, (list, tuple)) else None,
        "bbox_diagonal_px": diag,
        "ratio": _fnum(b.get("ratio")),
        "diameter_mm": diameter_mm,
        "diameter_class_mm": class_mm,
        "distance_m": _fnum(b.get("distance_m") if b.get("distance_m") is not None
                            else b.get("distance")),
        "samples": b.get("samples"),
        "updated_at": b.get("updated_at") or b.get("time"),
        "selected": bool(b.get("selected", False)),
        "confidence": b.get("confidence"),
    }


def _bucket_files():
    files = set(glob.glob(BUCKET_GLOB)) | set(glob.glob(BUCKET_LEGACY_GLOB))
    return [p for p in files if os.path.isfile(p)]


def list_bucket_records():
    """历史记录文件列表（最新在前），供 WebUI 选择回看。"""
    out = []
    for p in _bucket_files():
        if os.path.basename(p) == "buckets_latest.json":
            continue
        try:
            mtime = os.path.getmtime(p)
        except OSError:
            continue
        out.append({"name": os.path.basename(p), "mtime": mtime,
                    "time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mtime))})
    out.sort(key=lambda x: x["mtime"], reverse=True)
    return out


def _empty_buckets():
    return {"count": 0, "buckets": [], "requested_drop": None, "selected": [],
            "selected_detail": [], "drop_order": [], "notes": [], "schema": None,
            "mission": None, "source": None, "ts": None, "warnings": [],
            "diameters_mm": list(BUCKET_DIAMETERS_MM), "history": list_bucket_records(),
            "drop_zone": None, "recon_zone": None}


def load_buckets(path=None):
    """读取桶记录（P0 §六.5：logs/buckets/buckets_latest.json / buckets_<ts>.json）。

    宽松解析：字段名兼容 lat/latitude、buckets/targets、size/class/size_class 等；
    也接受旧约定 logs/bucket_*.json 与单桶/数组顶层。无文件返回空结构（不报错）。
    """
    empty = _empty_buckets()
    if not path:
        if os.path.isfile(BUCKET_LATEST):
            path = BUCKET_LATEST
        else:
            files = _bucket_files()
            if not files:
                return empty
            path = max(files, key=os.path.getmtime)
    else:
        if not os.path.isabs(path):
            cand = os.path.join(BUCKET_DIR, os.path.basename(path))
            path = cand if os.path.isfile(cand) else path
        if not os.path.isfile(path):
            empty["warnings"].append("记录文件不存在: %s" % path)
            return empty
    try:
        with open(path, "r", encoding="utf-8") as fh:
            raw = json.load(fh)
    except (OSError, ValueError) as exc:
        empty["warnings"].append("桶记录解析失败: %s（%s）" % (exc, os.path.basename(path)))
        empty["source"] = path
        return empty
    if isinstance(raw, list):
        raw = {"buckets": raw}
    if not isinstance(raw, dict):
        empty["warnings"].append("桶记录顶层不是对象: %s" % os.path.basename(path))
        empty["source"] = path
        return empty
    buckets_in = raw.get("buckets")
    if buckets_in is None:
        buckets_in = raw.get("targets")
    if not isinstance(buckets_in, list):
        buckets_in = [raw] if any(k in raw for k in ("depth_m", "bbox", "lat", "latitude")) else []
    buckets = [_normalize_bucket(b, i) for i, b in enumerate(buckets_in)
               if isinstance(b, dict)]

    # T2 冻结 schema：selected = [{id,size_class,reason,distance_m}]（已按距飞机升序）。
    # 兼容旧约定 selected_ids / drop_order / 顶层单值。
    sel = raw.get("selected")
    if sel is None:
        sel = raw.get("selected_buckets")
    if sel is None:
        sel = raw.get("drop_first") or raw.get("first_drop") or raw.get("requested_drop")
    selected_ids = []
    selected_detail = []
    if isinstance(sel, list):
        for s in sel:
            if isinstance(s, dict):
                v = s.get("id") or s.get("name") or s.get("target_id")
                if v is not None:
                    selected_ids.append(str(v))
                    selected_detail.append({
                        "id": str(v),
                        "size_class": s.get("size_class") or s.get("size"),
                        "reason": s.get("reason"),
                        "distance_m": _fnum(s.get("distance_m")),
                    })
            elif s is not None:
                selected_ids.append(str(s))
                selected_detail.append({"id": str(s)})
    elif sel is not None:
        selected_ids = [str(sel)]
        selected_detail = [{"id": str(sel)}]
    drop_order = raw.get("drop_order")
    if isinstance(drop_order, list) and drop_order:
        drop_order_ids = [str(x.get("id") if isinstance(x, dict) else x)
                          for x in drop_order]
        if not selected_ids:
            selected_ids = drop_order_ids
    else:
        drop_order_ids = []
    for b in buckets:
        if b["id"] in selected_ids:
            b["selected"] = True
    if not selected_ids:
        selected_ids = [b["id"] for b in buckets if b["selected"]]
        drop_order_ids = list(selected_ids)

    count = raw.get("count")
    if count is None:
        count = len(buckets)

    # 投放区/侦察区锁定结果：T2 写在 mission 块内（兼容顶层旧写法）。
    mission = raw.get("mission") if isinstance(raw.get("mission"), dict) else {}
    drop_zone = (mission.get("drop_zone_wgs84") or raw.get("drop_zone_wgs84")
                 or raw.get("drop_zone"))
    recon_zone = (mission.get("recon_zone_wgs84") or raw.get("recon_zone_wgs84")
                  or raw.get("recon_zone"))
    notes = raw.get("notes")
    return {
        "count": count,
        "buckets": buckets,
        "requested_drop": selected_ids[0] if selected_ids else None,
        "selected": selected_ids,
        "selected_detail": selected_detail,
        "drop_order": drop_order_ids,
        "notes": list(notes) if isinstance(notes, list) else ([str(notes)] if notes else []),
        "schema": raw.get("schema"),
        "mission": {
            "pattern": mission.get("pattern"),
            "search_altitude_m": _fnum(mission.get("search_altitude_m")),
            "recon_enabled": mission.get("recon_enabled"),
        } if mission else None,
        "source": path,
        "ts": raw.get("generated_at") or raw.get("ts") or raw.get("timestamp"),
        "warnings": [],
        "diameters_mm": list(BUCKET_DIAMETERS_MM),
        "history": list_bucket_records(),
        "drop_zone": drop_zone,
        "recon_zone": recon_zone,
    }


def _spawn_detached(cmd, name):
    """setsid 拉起 + 日志落盘；返回 Popen。"""
    os.makedirs(LOGDIR, exist_ok=True)
    fh = open(log_path(name), "ab", buffering=0)
    fh.write(("\n===== [%s] start: %s =====\n"
              % (time.strftime("%F %T"), " ".join(shlex.quote(c) for c in cmd))).encode())
    env = dict(os.environ)
    proc = subprocess.Popen(
        cmd, stdin=subprocess.DEVNULL, stdout=fh, stderr=subprocess.STDOUT,
        start_new_session=True, cwd=WS, env=env, close_fds=True)
    fh.close()
    return proc


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def pid_cmdline(pid):
    try:
        with open("/proc/%d/cmdline" % pid, "rb") as fh:
            raw = fh.read()
        return " ".join(a.decode("utf-8", "replace") for a in raw.split(b"\x00") if a)
    except OSError:
        return ""


def scan_external():
    """扫描 /proc，按 DETECT 精确识别外部进程。返回 {name: [pids]}。"""
    found = {}
    me = os.getpid()
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == me:
            continue
        cmd = pid_cmdline(pid)
        if not cmd:
            continue
        for name, pat in DETECT.items():
            if re.search(pat, cmd):
                found.setdefault(name, []).append(pid)
    return found


class Registry:
    def __init__(self):
        self.lock = threading.RLock()
        self.procs = {}          # name -> {"proc":Popen,"cmd":list,"label":str,"started":float}
        self.mission = {"proc": None}   # WebUI 启动的全自动任务（stdin 管道，供 Enter 授权）
        self.probe = None
        self.status = {"probe_ok": False, "data": None, "updated": 0.0}
        self._stop = False

    # ---------------- 进程规格（依赖当前 YAML）----------------
    def spec(self, name):
        c = read_cfg()
        pkg = str(c.get("detector_package", "cuadc_vision"))
        src = str(c.get("camera_source", "gazebo"))
        show = bool(c.get("detector_show_window", True))
        scripts = os.path.join(WS, "scripts")
        if name == "mavros_sim":
            return ("仿真 MAVROS", ["bash", os.path.join(scripts, "launchers", "run_sim_mavros.sh")])
        if name == "mavros_real":
            return ("真机 MAVROS(V6X)", ["bash", os.path.join(scripts, "launchers", "run_real_fc_mavros.sh")])
        if name == "camera":
            if pkg == "cuadc_visionY26":
                return ("相机(Y26)", ["ros2", "launch", pkg, "camera_node.launch.py",
                                      "source:=%s" % src, "show_rgb:=false"])
            return ("相机", ["ros2", "run", "cuadc_vision", "camera_node.py",
                             "--ros-args", "-p", "source:=%s" % src])
        if name == "detector":
            cmd = ["ros2", "launch", pkg, "detector_node.launch.py", "auto_start_mavros:=false"]
            if pkg == "cuadc_visionY26":
                cmd += ["auto_start_camera:=false", "camera_source:=%s" % src,
                        "show_window:=%s" % ("true" if show else "false")]
            return ("识别(%s)" % pkg, cmd)
        if name == "servo_test":
            return ("舵机测试", [
                "ros2", "run", "cuadc_vision", "servo_test.py", "--ros-args",
                "-p", "servo_a_channel:=%d" % int(c.get("servo_a_channel", 9)),
                "-p", "servo_b_channel:=%d" % int(c.get("servo_b_channel", 10)),
                "-p", "pwm_open:=%d" % int(c.get("pwm_open", 1500)),
                "-p", "pwm_close:=%d" % int(c.get("pwm_close", 1000)),
                "-p", "enable_ch5:=%s" % ("true" if c.get("enable_ch5", True) else "false"),
                "-p", "enable_ch6:=%s" % ("true" if c.get("enable_ch6", True) else "false"),
            ])
        if name == "mission":
            return ("全自动任务", ["ros2", "run", "cuadc_control", "auto_drop_test.py",
                                   "--config", CONFIG])
        if name == "preflight":
            return ("一键体检", ["bash", os.path.join(scripts, "preflight.sh")])
        if name == "demo_up":
            return ("一键拉起仿真", ["bash", os.path.join(scripts, "demo_up.sh")])
        if name == "sim_up":
            return ("一键启动仿真(清场+拉起)",
                    ["bash", os.path.join(scripts, "launchers", "run_sim.sh")])
        if name == "kill_sim":
            return ("一键杀后台(清场)",
                    ["bash", os.path.join(scripts, "launchers", "run_kill_sim.sh")])
        return (None, None)

    # ---------------- 启停 ----------------
    def start(self, name, confirm=False):
        if name == "mission":
            return {"ok": False,
                    "error": "全自动任务必须在终端启动并按 Enter 授权；WebUI 只启动任务依赖"}
        if name in DANGEROUS and not confirm:
            return {"ok": False, "need_confirm": True,
                    "error": "该动作需二次确认（真实飞行/真机链路）"}
        with self.lock:
            label, cmd = self.spec(name)
            if cmd is None:
                return {"ok": False, "error": "未知进程: %s" % name}
            entry = self.procs.get(name)
            if entry and pid_alive(entry["proc"].pid):
                return {"ok": False, "error": "%s 已在运行 (pid=%d)" % (label, entry["proc"].pid)}
            try:
                proc = _spawn_detached(cmd, name)
            except Exception as exc:
                return {"ok": False, "error": "启动失败: %s" % exc}
            self.procs[name] = {"proc": proc, "cmd": cmd, "label": label,
                                "started": time.time()}
            return {"ok": True, "name": name, "pid": proc.pid, "label": label,
                    "log": log_path(name), "cmd": " ".join(shlex.quote(c) for c in cmd)}

    def start_bundle(self, link="sim"):
        """启动全自动任务依赖，不启动 mission，保留终端 Enter 起飞授权。"""
        link = str(link or "sim").lower()
        if link not in ("sim", "real"):
            return {"ok": False, "error": "link 必须是 sim 或 real"}
        names = ["mavros_sim" if link == "sim" else "mavros_real",
                 "camera", "detector", "servo_test"]
        results = []
        for name in names:
            result = self.start(name, confirm=(link == "real"))
            results.append(result)
            if not result.get("ok") and "已在运行" not in result.get("error", ""):
                return {"ok": False, "link": link, "results": results,
                        "error": result.get("error", "启动依赖失败")}
        # 终端命令同步 tee 到 WebUI 读的日志文件（免看混乱终端）。
        command = ("source /opt/ros/humble/setup.bash && "
                   "source ~/ROS2_CUADC2026/install/setup.bash && "
                   "PYTHONUNBUFFERED=1 ros2 run cuadc_control auto_drop_test.py --config "
                   + CONFIG + " 2>&1 | tee -a /tmp/opencode/webui_mission.log")
        return {"ok": True, "link": link, "results": results,
                "command": command,
                "message": "依赖已启动；请在终端运行命令并按 Enter 授权起飞"}

    # ---------------- WebUI 启动全流程 + Enter 授权 ----------------
    def start_mission(self, confirm=False):
        """从 WebUI 启动全自动任务：stdin 走管道，页面「Enter 确认」即可授权起飞。"""
        with self.lock:
            proc = self.mission.get("proc")
            if proc is not None and pid_alive(proc.pid):
                return {"ok": False, "error": "任务已在运行 (pid=%d)" % proc.pid}
            _, cmd = self.spec("mission")
            if not cmd:
                return {"ok": False, "error": "未知任务命令"}
            os.makedirs(LOGDIR, exist_ok=True)
            fh = open(log_path("mission"), "ab", buffering=0)
            fh.write(("\n===== [%s] WebUI 启动全自动任务: %s =====\n"
                      % (time.strftime("%F %T"),
                         " ".join(shlex.quote(c) for c in cmd))).encode())
            env = dict(os.environ)
            env["PYTHONUNBUFFERED"] = "1"
            try:
                proc = subprocess.Popen(
                    cmd, stdin=subprocess.PIPE, stdout=fh, stderr=subprocess.STDOUT,
                    start_new_session=True, cwd=WS, env=env, text=True, bufsize=1)
            except Exception as exc:
                fh.close()
                return {"ok": False, "error": "启动失败: %s" % exc}
            fh.close()
            self.mission = {"proc": proc}
            return {"ok": True, "pid": proc.pid, "log": log_path("mission"),
                    "cmd": " ".join(shlex.quote(c) for c in cmd)}

    def confirm_mission(self):
        """向正在运行的任务写入一个换行 = 终端按 Enter 授权起飞。"""
        with self.lock:
            proc = self.mission.get("proc")
            if proc is None or proc.poll() is not None:
                return {"ok": False, "error": "没有正在运行的任务"}
            if proc.stdin is None:
                return {"ok": False, "error": "任务 stdin 不可用"}
            try:
                proc.stdin.write("\n")
                proc.stdin.flush()
            except Exception as exc:
                return {"ok": False, "error": "写入确认失败: %s" % exc}
            return {"ok": True, "pid": proc.pid}

    def stop_mission(self):
        """停止 WebUI 启动的全自动任务：按 pgid 精确 SIGINT（禁 pkill）。"""
        with self.lock:
            proc = self.mission.get("proc")
            if proc is None or proc.poll() is not None:
                self.mission = {"proc": None}
                return {"ok": False, "error": "没有正在运行的任务"}
            try:
                pgid = os.getpgid(proc.pid)
            except OSError:
                pgid = proc.pid
            try:
                os.killpg(pgid, signal.SIGINT)
            except OSError as exc:
                return {"ok": False, "error": "停止失败: %s" % exc}
            deadline = time.time() + 5.0
            while time.time() < deadline and proc.poll() is None:
                time.sleep(0.2)
            if proc.poll() is None:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except OSError:
                    pass
            alive = proc.poll() is None
            if not alive:
                self.mission = {"proc": None}
            return {"ok": True, "pid": proc.pid, "alive": alive}

    def mission_status(self):
        proc = self.mission.get("proc")
        running = bool(proc is not None and proc.poll() is None)
        return {"running": running, "pid": (proc.pid if running else None)}

    def stop_mavros_all(self):
        """关闭所有 MAVROS（仿真 + 真机）：本服务托管的按名字停，外部的按 pid 逐个停（禁 pkill）。"""
        results = []
        for name in ("mavros_sim", "mavros_real"):
            entry = self.procs.get(name)
            if entry and pid_alive(entry["proc"].pid):
                results.append({"name": name, **self.stop(name=name)})
            else:
                results.append({"name": name, "ok": True, "skipped": "未在本服务托管"})
        ext = scan_external().get("mavros", [])   # DETECT 键名就是 "mavros"（含 sim/real 两种命令）
        if ext:
            for pid in ext:
                results.append({"name": "mavros(外部)", "pid": pid, **self.stop(pid=pid)})
        else:
            results.append({"name": "mavros(外部)", "ok": True, "skipped": "无外部 MAVROS"})
        ok = all(r.get("ok") for r in results)
        return {"ok": ok, "results": results}

    def stop(self, name=None, pid=None):
        with self.lock:
            target = None
            if name and name in self.procs:
                target = self.procs[name]["proc"].pid
            elif name:
                ext = scan_external().get(name) or []
                if len(ext) == 1:
                    target = ext[0]
                elif len(ext) > 1:
                    return {"ok": False, "error": "%s 有多个进程 %s，请按 pid 停止"
                            % (name, ext)}
                else:
                    return {"ok": False, "error": "%s 未在运行" % name}
            elif pid:
                try:
                    target = int(pid)
                except (TypeError, ValueError):
                    return {"ok": False, "error": "pid 非法"}
            if target is None:
                return {"ok": False, "error": "缺少 name 或 pid"}
            if not pid_alive(target):
                return {"ok": False, "error": "pid=%d 已不存在" % target}
            # 白名单校验：只允许匹配已知模式的进程（防误杀）
            if pid is not None:
                cmd = pid_cmdline(target)
                if not any(re.search(p, cmd) for p in DETECT.values()):
                    return {"ok": False, "error": "拒绝停止：pid=%d 不匹配任何已知节点" % target}
            try:
                pgid = os.getpgid(target)
            except OSError:
                pgid = target
            if pgid == os.getpgid(os.getpid()):
                return {"ok": False, "error": "拒绝：目标进程组即本服务进程组"}
            try:
                os.killpg(pgid, signal.SIGINT)
            except OSError as exc:
                return {"ok": False, "error": "killpg 失败: %s" % exc}
            deadline = time.time() + 4.0
            while time.time() < deadline and pid_alive(target):
                time.sleep(0.2)
            if pid_alive(target):
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except OSError:
                    pass
                deadline = time.time() + 2.0
                while time.time() < deadline and pid_alive(target):
                    time.sleep(0.2)
            for key, entry in list(self.procs.items()):
                if entry["proc"].pid == target:
                    self.procs.pop(key, None)
            return {"ok": True, "pid": target, "alive": pid_alive(target)}

    def servo(self, action):
        text = SERVO_ACTIONS.get(action, action)
        if not text:
            return {"ok": False, "error": "未知舵机动作"}
        helper = os.path.join(os.path.dirname(os.path.abspath(__file__)), "servo_pub.py")
        try:
            res = subprocess.run(
                [sys.executable, helper, text], cwd=WS, capture_output=True, text=True,
                timeout=12)
        except subprocess.TimeoutExpired:
            return {"ok": False, "error": "发送超时（订阅者发现异常）"}
        except Exception as exc:
            return {"ok": False, "error": "发送失败: %s" % exc}
        out = (res.stdout + res.stderr).strip()
        if res.returncode == 0:
            return {"ok": True, "action": text, "rc": 0, "output": out[-400:]}
        return {"ok": False, "action": text, "rc": res.returncode, "error": out[-400:]}

    # ---------------- ros_probe ----------------
    def start_probe(self):
        cmd = [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                            "ros_probe.py"), "--interval", "2.0"]
        os.makedirs(LOGDIR, exist_ok=True)
        errfh = open(log_path("ros_probe"), "ab", buffering=0)
        self.probe = subprocess.Popen(
            cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=errfh, start_new_session=True, cwd=WS, text=True, bufsize=1)
        threading.Thread(target=self._probe_reader, daemon=True).start()

    def _probe_reader(self):
        proc = self.probe
        try:
            for line in proc.stdout:
                line = line.strip()
                if not line:
                    continue
                try:
                    data = json.loads(line)
                except ValueError:
                    continue
                self.status = {"probe_ok": True, "data": data, "updated": time.time()}
        except Exception:
            pass
        self.status["probe_ok"] = False

    def probe_status(self):
        st = dict(self.status)
        if st.get("data") and st.get("updated"):
            st["age_s"] = round(time.time() - st["updated"], 2)
        else:
            st["age_s"] = None
        if self.probe is not None and self.probe.poll() is not None:
            st["probe_ok"] = False
            if not self._stop:
                try:
                    self.start_probe()
                except Exception:
                    pass
        return st

    def processes(self):
        ext = scan_external()
        with self.lock:
            out = []
            names = ["mavros_sim", "mavros_real", "camera", "detector", "servo_test",
                     "mission", "preflight", "demo_up", "sim_up", "kill_sim"]
            for name in names:
                label, _ = self.spec(name)
                entry = self.procs.get(name)
                managed_pid = entry["proc"].pid if entry and pid_alive(entry["proc"].pid) else None
                # 外部等价节点（如 demo_up 起的 mavros/camera/detector/servo/mission）
                det_name = {"mavros_sim": "mavros_sim", "mavros_real": "mavros_real",
                            "servo_test": "servo_test", "mission": "mission",
                            "camera": "camera", "detector": "detector"}.get(name)
                detected = ext.get(det_name, []) if det_name else []
                running = bool(managed_pid) or bool(detected)
                out.append({
                    "name": name, "label": label, "running": running,
                    "managed": bool(managed_pid), "pid": managed_pid,
                    "detected_pids": detected,
                    "uptime_s": (round(time.time() - entry["started"], 1)
                                 if entry and managed_pid else None),
                    "log": log_path(name),
                })
            extras = []
            for dname in ("sitl", "gz_server", "gz_gui", "rc_center"):
                pids = ext.get(dname, [])
                extras.append({"name": dname, "running": bool(pids), "detected_pids": pids})
            return {"processes": out, "extras": extras}

    def shutdown_children(self):
        self._stop = True
        if self.probe is not None and self.probe.poll() is None:
            try:
                os.killpg(os.getpgid(self.probe.pid), signal.SIGTERM)
            except OSError:
                pass
        # 网页关闭/服务退出：按用户拍板——不杀已 detach 的其它子进程。


REG = Registry()


class Handler(BaseHTTPRequestHandler):
    server_version = "CUADCWebUI/1.0"
    token = ""

    def log_message(self, fmt, *args):
        sys.stderr.write("[webui] %s - %s\n" % (self.address_string(), fmt % args))

    # ---------------- 工具 ----------------
    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass  # 客户端提前断开（如请求超时离开）

    def _text(self, text, code=200, ctype="text/plain; charset=utf-8"):
        body = text.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    # ---------------- 识别画面（带标注）：前端轮询单帧 ----------------
    # 不用长连接 MJPEG（曾有"客户端断开后流线程不退出"的坑）；改为前端每 ~150ms
    # 轮询 /api/frame，每次请求刷新 STREAM_FLAG 的 mtime；探针只在 mtime 新鲜
    # (<3s) 时订阅 /vision/annotated_image，没人看就自动退订（免无谓负载）。
    def _placeholder(self):
        try:
            with open(PLACEHOLDER_JPEG, "rb") as fh:
                return fh.read()
        except OSError:
            return b""

    def _frame(self):
        """/api/frame：最新标注帧 JPEG（无则 NO SIGNAL 占位），并刷新观看心跳。"""
        try:
            os.utime(STREAM_FLAG, None)
        except OSError:
            try:
                os.makedirs(LOGDIR, exist_ok=True)
                open(STREAM_FLAG, "w").close()
            except OSError:
                pass
        try:
            with open(FRAME_FILE, "rb") as fh:
                data = fh.read()
        except OSError:
            data = b""
        if not data:
            data = self._placeholder()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _authed(self, query):
        if not self.token:
            return True
        supplied = self.headers.get("X-Auth-Token") or (query.get("token", [""])[0])
        return bool(supplied) and secrets.compare_digest(str(supplied), self.token)

    def _body(self):
        try:
            length = int(self.headers.get("Content-Length") or 0)
        except ValueError:
            length = 0
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return {}

    # ---------------- 路由 ----------------
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)
        if path.startswith("/api/"):
            if not self._authed(query):
                return self._json({"ok": False, "error": "口令无效或缺失"}, 401)
            if path == "/api/frame":
                return self._frame()
            return self._api_get(path, query)
        return self._static(path)

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        query = parse_qs(parsed.query)
        if not path.startswith("/api/"):
            return self._json({"ok": False, "error": "not found"}, 404)
        if not self._authed(query):
            return self._json({"ok": False, "error": "口令无效或缺失"}, 401)
        body = self._body()
        try:
            if path == "/api/config":
                return self._save_config(body)
            if path == "/api/start_bundle":
                return self._json(REG.start_bundle(body.get("link", "sim")))
            if path == "/api/mission/start":
                return self._json(REG.start_mission(bool(body.get("confirm"))))
            if path == "/api/mission/confirm":
                return self._json(REG.confirm_mission())
            if path == "/api/mission/stop":
                return self._json(REG.stop_mission())
            if path == "/api/mavros/stop":
                return self._json(REG.stop_mavros_all())
            if path == "/api/process/start":
                return self._json(REG.start(str(body.get("name", "")), bool(body.get("confirm"))))
            if path == "/api/process/stop":
                return self._json(REG.stop(name=body.get("name"), pid=body.get("pid")))
            if path == "/api/servo":
                return self._json(REG.servo(str(body.get("action", ""))))
            return self._json({"ok": False, "error": "未知接口"}, 404)
        except Exception as exc:
            return self._json({"ok": False, "error": "服务内部错误: %s" % exc}, 500)

    def _api_get(self, path, query):
        try:
            if path == "/api/config":
                values, warnings = cfgmod.load_file(CONFIG)
                merged = cfgmod.builtin_defaults()
                merged.update(values)
                errors, warns = cfgmod.validate(merged)
                return self._json({"ok": True, "values": merged,
                                   "schema": cfgmod.describe(),
                                   "path": CONFIG, "warnings": warnings + warns,
                                   "errors": errors})
            if path == "/api/processes":
                return self._json({"ok": True, **REG.processes()})
            if path == "/api/status":
                return self._json({"ok": True, "status": REG.probe_status()})
            if path == "/api/mission/status":
                return self._json({"ok": True, **REG.mission_status()})
            if path == "/api/buckets":
                # 只读：logs/buckets/buckets_latest.json（或 ?file= 指定历史）+ 探针实时 count。
                file_arg = (query.get("file", [""])[0]) or None
                try:
                    info = load_buckets(file_arg)
                except Exception as exc:
                    info = _empty_buckets()
                    info["warnings"] = ["读取桶记录失败: %s" % exc]
                probe = (REG.probe_status().get("data") or {})
                payload = {"ok": True}
                payload.update(info)
                payload["live_count"] = probe.get("bucket_count")
                payload["live_count_age_s"] = probe.get("bucket_count_age_s")
                return self._json(payload)
            if path == "/api/log":
                name = (query.get("name", [""])[0])
                try:
                    tail = int(query.get("tail", ["200"])[0])
                except ValueError:
                    tail = 200
                path_log = log_path(name)
                if not os.path.isfile(path_log):
                    return self._text("（暂无日志: %s）" % path_log)
                with open(path_log, "r", encoding="utf-8", errors="replace") as fh:
                    lines = fh.readlines()[-max(1, tail):]
                return self._text("".join(lines))
            if path == "/api/ping":
                return self._json({"ok": True, "ws": WS, "config": CONFIG})
            return self._json({"ok": False, "error": "未知接口"}, 404)
        except Exception as exc:
            return self._json({"ok": False, "error": "服务内部错误: %s" % exc}, 500)

    def _save_config(self, body):
        values = body.get("values") or {}
        if not isinstance(values, dict) or not values:
            return self._json({"ok": False, "errors": ["未收到配置内容"]}, 400)
        errors, warnings = cfgmod.validate(values)
        if errors:
            return self._json({"ok": False, "errors": errors, "warnings": warnings}, 400)
        os.makedirs(BACKUP_DIR, exist_ok=True)
        os.makedirs(os.path.dirname(CONFIG), exist_ok=True)
        backup = None
        if os.path.isfile(CONFIG):
            backup = os.path.join(BACKUP_DIR, "config_backup_%s.yaml" % ts())
            try:
                with open(CONFIG, "rb") as src, open(backup, "wb") as dst:
                    dst.write(src.read())
            except OSError as exc:
                return self._json({"ok": False, "errors": ["备份失败: %s" % exc]}, 500)
        text = cfgmod.emit(values)
        tmp = CONFIG + ".tmp"
        with open(tmp, "w", encoding="utf-8") as fh:
            fh.write(text)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, CONFIG)
        return self._json({"ok": True, "backup": backup, "warnings": warnings,
                           "path": CONFIG})

    def _static(self, path):
        if path in ("/", "/index.html"):
            rel = "index.html"
        else:
            rel = path.lstrip("/")
            if rel.startswith("static/"):
                rel = rel[len("static/"):]
        base = os.path.normpath(STATIC_DIR)
        target = os.path.normpath(os.path.join(base, rel))
        if not target.startswith(base) or not os.path.isfile(target):
            return self._text("404 not found", 404)
        ctype = {
            ".html": "text/html; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".svg": "image/svg+xml",
            ".woff2": "font/woff2",
            ".woff": "font/woff",
            ".ttf": "font/ttf",
            ".otf": "font/otf",
            ".jpg": "image/jpeg",
            ".jpeg": "image/jpeg",
            ".png": "image/png",
        }.get(os.path.splitext(target)[1], "application/octet-stream")
        with open(target, "rb") as fh:
            body = fh.read()
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")   # 改前端后无需清缓存
        self.end_headers()
        self.wfile.write(body)


def resolve_token(cli_token):
    if cli_token:
        return cli_token
    env = os.environ.get("CUADC_WEBUI_TOKEN")
    if env:
        return env
    if os.path.isfile(TOKEN_FILE):
        with open(TOKEN_FILE, "r", encoding="utf-8") as fh:
            return fh.read().strip()
    token = secrets.token_urlsafe(16)
    os.makedirs(os.path.dirname(TOKEN_FILE), exist_ok=True)
    with open(TOKEN_FILE, "w", encoding="utf-8") as fh:
        fh.write(token)
    os.chmod(TOKEN_FILE, 0o600)
    return token


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--token", default="")
    args = ap.parse_args()

    cfgmod.ensure_config(CONFIG)
    Handler.token = resolve_token(args.token)
    REG.start_probe()

    class Server(ThreadingHTTPServer):
        def handle_error(self, request, client_address):
            # 客户端提前断开（请求超时离开/刷新）属正常，不打印 Traceback（房规：0 Traceback）。
            exc = sys.exc_info()[1]
            if isinstance(exc, (BrokenPipeError, ConnectionResetError)):
                return
            super().handle_error(request, client_address)

    httpd = Server((args.host, args.port), Handler)
    sys.stderr.write("[webui] listening on http://%s:%d  (config=%s)\n"
                     % (args.host, args.port, CONFIG))
    if Handler.token:
        sys.stderr.write("[webui] token 已启用（%s）\n" % TOKEN_FILE)

    def _sig(signum, frame):
        threading.Thread(target=httpd.shutdown, daemon=True).start()
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    try:
        httpd.serve_forever()
    finally:
        REG.shutdown_children()
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
