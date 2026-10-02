#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""verify_webui_api.py — Web UI 后端（webui/server.py）接口/进程/配置 验收（B 组 + D 无飞行部分）。

不飞行、不碰真机动作：
  - 服务器在测试端口 8899 独立起（不碰 8765 与真机 MAVROS）。
  - 仅测：静态页 / 鉴权 / 读配置 / 非法拒绝 / 备份+原子写+还原 / 进程启停(用只读 preflight)
    / /servo 指令发布（当前无订阅者，无副作用）/ /status 流实据 / /log。
  - 退出：killpg 精确停测试服务器，校验端口释放、无残留。

用法: bash -lc 'source /opt/ros/humble/setup.bash && source ~/ROS2_CUADC2026/install/setup.bash && \
      python3 ~/ROS2_CUADC2026/scripts/verify_webui_api.py'
rc: 0=全过 / 1=有 FAIL
"""
import glob
import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import quote

WS = os.path.expanduser("~/ROS2_CUADC2026")
CONFIG = os.path.join(WS, "config", "auto_drop.yaml")
PORT = 8899
TOKEN = "verify-token"
BASE = "http://127.0.0.1:%d" % PORT
LOG = "/tmp/opencode/webui_verify_server.log"

CHECKS = []


def check(name, ok, detail=""):
    CHECKS.append((name, bool(ok), detail))
    print("  [%s] %-46s %s" % ("PASS" if ok else "FAIL", name, detail))


def req(path, method="GET", body=None, token=TOKEN, timeout=15):
    headers = {"Content-Type": "application/json"}
    if token:
        headers["X-Auth-Token"] = token
    data = json.dumps(body).encode() if body is not None else None
    r = urllib.request.Request(BASE + path, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(r, timeout=timeout) as resp:
            return resp.status, resp.read().decode("utf-8", "replace")
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode("utf-8", "replace")


def jreq(path, method="GET", body=None, token=TOKEN, timeout=15):
    code, raw = req(path, method, body, token, timeout)
    try:
        return code, json.loads(raw)
    except ValueError:
        return code, {"_raw": raw}


def port_busy(port):
    out = subprocess.run(["ss", "-ltn", "sport = :%d" % port],
                         capture_output=True, text=True).stdout
    return (":%d" % port) in out


def server_pids():
    out = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True).stdout
    pids = []
    for line in out.splitlines():
        if "webui/server.py" in line and "--port %d" % PORT in line and "grep" not in line:
            try:
                pids.append(int(line.split()[0]))
            except ValueError:
                pass
    return pids


def run_checks():
    up = False
    for _ in range(60):
        if port_busy(PORT):
            up = True
            break
        time.sleep(0.2)
    check("服务器启动/监听 %d" % PORT, up)
    if not up:
        return

    code, raw = req("/", token=None)
    check("静态页 GET /", code == 200 and "CUADC" in raw, "http=%s" % code)
    code, raw = req("/static/app.js", token=None)
    check("静态资源 app.js", code == 200 and "loadConfig" in raw, "http=%s" % code)

    code, _ = req("/api/config", token=None)
    check("无口令访问 /api/config -> 401", code == 401, "http=%s" % code)
    code, _ = req("/api/config", token="wrong")
    check("错误口令 -> 401", code == 401, "http=%s" % code)

    code, d = jreq("/api/config")
    vals = (d.get("values") or {})
    schema = (d.get("schema") or {})
    check("读配置 values/schema", code == 200 and d.get("ok")
          and "area_search_pattern" in vals and len(schema.get("fields", [])) >= 100,
          "fields=%d" % len(schema.get("fields", [])))

    bad = dict(vals)
    bad["recon_zone_distance_m"] = bad.get("drop_zone_distance_m", 32.0) - 1.0
    code, d = jreq("/api/config", "POST", {"values": bad})
    check("非法值被拒绝(400+errors)", code == 400 and d.get("errors"),
          "http=%s errs=%s" % (code, (d.get("errors") or [None])[0]))

    before_backups = set(glob.glob(os.path.join(WS, "logs", "config_backup_*.yaml")))
    good = dict(vals)
    good["area_search_pattern"] = "Z" if vals.get("area_search_pattern") != "Z" else "W"
    code, d = jreq("/api/config", "POST", {"values": good})
    saved = False
    with open(CONFIG, "r", encoding="utf-8") as fh:
        saved = ("area_search_pattern: %s" % good["area_search_pattern"]) in fh.read()
    after_backups = set(glob.glob(os.path.join(WS, "logs", "config_backup_*.yaml")))
    check("合法保存(ok+备份+生效)",
          code == 200 and d.get("ok") and d.get("backup") and saved
          and (after_backups - before_backups),
          "backup=%s" % os.path.basename(d.get("backup") or "无"))

    code, d = jreq("/api/processes")
    names = [p["name"] for p in d.get("processes", [])]
    check("进程列表含关键节点", code == 200 and {"mission", "camera", "detector",
          "servo_test", "mavros_sim", "preflight"} <= set(names), ",".join(names))

    code, d = jreq("/api/process/start", "POST", {"name": "preflight"})
    pid = d.get("pid")
    check("启动 preflight(managed pid)", code == 200 and d.get("ok") and pid,
          "pid=%s" % pid)
    time.sleep(3)
    code, d = jreq("/api/processes")
    entry = next((p for p in d.get("processes", []) if p["name"] == "preflight"), {})
    check("preflight 运行中", entry.get("running") is True,
          "detected=%s" % entry.get("detected_pids"))
    code, raw = req("/api/log?name=preflight&tail=50")
    check("preflight 日志可见", code == 200 and len(raw.strip()) > 0, "%dB" % len(raw))

    code, d = jreq("/api/process/stop", "POST", {"name": "preflight"}, timeout=30)
    time.sleep(1)
    alive = False
    if pid:
        try:
            os.kill(int(pid), 0)
            alive = True
        except OSError:
            alive = False
    check("停止 preflight(精确 pid)", code == 200 and d.get("ok") and not alive,
          "alive=%s" % alive)

    # /servo 指令：起一个 ros2 topic echo 订阅者作证，验证真发到 /servo/cmd（不碰舵机硬件）
    echo_log = "/tmp/opencode/webui_verify_echo.log"
    efh = open(echo_log, "wb")
    echo = subprocess.Popen(
        ["ros2", "topic", "echo", "--once", "/servo/cmd", "std_msgs/String"],
        stdin=subprocess.DEVNULL, stdout=efh, stderr=subprocess.STDOUT,
        start_new_session=True, cwd=WS, env=dict(os.environ))
    time.sleep(2.5)
    code, d = jreq("/api/servo", "POST", {"action": "all_off"}, timeout=20)
    time.sleep(1.5)
    try:
        efh.close()
    except Exception:
        pass
    with open(echo_log, "r", encoding="utf-8", errors="replace") as fh:
        echo_text = fh.read()
    try:
        os.killpg(os.getpgid(echo.pid), signal.SIGTERM)
    except OSError:
        pass
    check("/servo 指令发布到 /servo/cmd", code == 200 and d.get("ok")
          and "all off" in echo_text,
          "ok=%s action=%s echo=%s" % (d.get("ok"), d.get("action"),
                                       "all off" in echo_text))

    time.sleep(3)
    code, d = jreq("/api/status")
    st = (d.get("status") or {})
    data = st.get("data") or {}
    check("/status 有流实据结构", code == 200 and "hz" in data and "state" in data,
          "probe_ok=%s" % st.get("probe_ok"))

    # /api/buckets（只读：鉴权 + 结构 + 坏文件容错，0 Traceback）
    code, _ = req("/api/buckets", token=None)
    check("无口令 /api/buckets -> 401", code == 401, "http=%s" % code)
    code, d = jreq("/api/buckets")
    check("/api/buckets 结构(ok/buckets/count)",
          code == 200 and d.get("ok") and "buckets" in d and "count" in d
          and "diameters_mm" in d and "selected_detail" in d and "drop_order" in d,
          "count=%s src=%s" % (d.get("count"), os.path.basename(d.get("source") or "无")))
    check("/api/buckets 带实时 count 字段(探针 GeoTarget)",
          code == 200 and "live_count" in d and "live_count_age_s" in d,
          "live_count=%s" % d.get("live_count"))

    # 坏文件：显式 ?file= 绝对路径 → 走真实解析路径，应 ok + warning（0 Traceback）
    bad_rec = os.path.join("/tmp/opencode", "bucket_verify_bad_%d.json" % int(time.time()))
    with open(bad_rec, "w", encoding="utf-8") as fh:
        fh.write("{ 这不是合法 JSON")
    try:
        code, d = jreq("/api/buckets?file=" + quote(bad_rec, safe=""))
        check("/api/buckets 坏文件不崩(ok+warning)",
              code == 200 and d.get("ok") and d.get("warnings"),
              "http=%s warn=%s" % (code, (d.get("warnings") or [None])[0]))
    finally:
        try:
            os.remove(bad_rec)
        except OSError:
            pass

    # 合法 T2 冻结 schema（mission 区坐标 / generated_at / selected 顺序）→ 解析正确
    good_rec = os.path.join("/tmp/opencode", "bucket_verify_good_%d.json" % int(time.time()))
    with open(good_rec, "w", encoding="utf-8") as fh:
        json.dump({
            "schema": 1,
            "generated_at": "2026-10-06T12:00:00+08:00",
            "mission": {
                "pattern": "Z", "search_altitude_m": 3.0,
                "drop_zone_wgs84": [29.1, 104.1],
                "recon_zone_wgs84": [29.2, 104.2],
                "recon_enabled": True,
            },
            "buckets": [
                {"id": 1, "lat": 29.3, "lon": 104.3, "alt": 603.0, "ratio": 0.95,
                 "size_class": "mid", "diameter_mm": 200, "samples": 5,
                 "updated_at": "2026-10-06T11:59:00+08:00", "distance_m": 8.2},
                {"id": 2, "lat": 29.4, "lon": 104.4, "alt": 603.0, "ratio": 0.72,
                 "size_class": "small", "diameter_mm": 150, "samples": 4,
                 "updated_at": "2026-10-06T11:59:30+08:00", "distance_m": 5.4},
            ],
            "selected": [
                {"id": 2, "size_class": "small", "reason": "smaller_bucket", "distance_m": 5.4},
                {"id": 1, "size_class": "mid", "reason": "closest_to_ref", "distance_m": 8.2},
            ],
            "drop_order": [2, 1],
            "notes": ["先小号(近)后中号(远)"],
        }, fh, ensure_ascii=False)
    try:
        code, d = jreq("/api/buckets?file=" + quote(good_rec, safe=""))
        zones_ok = d.get("drop_zone") == [29.1, 104.1] and d.get("recon_zone") == [29.2, 104.2]
        ts_ok = d.get("ts") == "2026-10-06T12:00:00+08:00"
        order_ok = d.get("requested_drop") == "2" and d.get("selected") == ["2", "1"]
        cls_ok = next((b.get("diameter_class_mm") for b in d.get("buckets", [])
                       if str(b.get("id")) == "1"), None) == 200
        check("/api/buckets 对齐 T2 schema(区坐标/generated_at/选桶顺序/档位)",
              code == 200 and d.get("ok") and zones_ok and ts_ok and order_ok and cls_ok,
              "zones=%s ts=%s order=%s cls200=%s" % (zones_ok, ts_ok, order_ok, cls_ok))
    finally:
        try:
            os.remove(good_rec)
        except OSError:
            pass


def stop_server(proc):
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except OSError:
        pass
    for _ in range(30):
        if proc.poll() is not None:
            break
        time.sleep(0.2)
    time.sleep(0.5)
    check("测试服务器已退出", proc.poll() is not None, "rc=%s" % proc.poll())
    check("端口已释放", not port_busy(PORT))
    check("无残留 server.py", not server_pids(), "pids=%s" % server_pids())


def main():
    with open(CONFIG, "r", encoding="utf-8") as fh:
        original = fh.read()
    logfh = open(LOG, "ab", buffering=0)
    proc = subprocess.Popen(
        [sys.executable, os.path.join(WS, "webui", "server.py"),
         "--host", "127.0.0.1", "--port", str(PORT), "--token", TOKEN],
        stdin=subprocess.DEVNULL, stdout=logfh, stderr=subprocess.STDOUT,
        start_new_session=True, cwd=WS, env=dict(os.environ))
    try:
        run_checks()
    except Exception as exc:
        check("未预期异常", False, repr(exc))
    finally:
        stop_server(proc)
        try:
            logfh.close()
        except Exception:
            pass
        with open(CONFIG, "w", encoding="utf-8") as fh:
            fh.write(original)
        # 兜底：清理可能残留的测试服务器/其子节点
        leftover = server_pids()
        for p in leftover:
            try:
                os.killpg(os.getpgid(p), signal.SIGTERM)
            except OSError:
                pass

    npass = sum(1 for _, ok, _ in CHECKS if ok)
    nfail = len(CHECKS) - npass
    print("\n== 结果: PASS=%d FAIL=%d ==" % (npass, nfail))
    return 0 if nfail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
