(function () {
  "use strict";
  var state = { values: {}, schema: null, fields: [], byName: {} };
  var authToken = new URLSearchParams(window.location.search).get("token") ||
    window.sessionStorage.getItem("cuadc_webui_token") || "";

  /* 3 个大步骤（全自动任务，均开启视觉识别）：
     ① 仅搜索记录           -> survey_only_mode=true（视觉搜索 + 记录桶坐标 → 返航；不瞄准、不投放）
     ② 搜索 + 瞄准 + 投放    -> full_mission_mode=true + dry_run=false（完整走瞄准/投放流程并发布 /servo/cmd）
     ③ 搜索 + 瞄准投放 + 侦察 -> full_mission_mode=true + dry_run=false + recon_enabled（实投 + 是否去侦察区）
     说明：完整任务必须关闭 flight_only_mode；仅飞行模式仍可单独打开该开关。
     keys = 卡片上直接可见/可改的开关；set = "载入本步默认参数"写入的建议值。 */
  var STEPS = [
    {
      id: "survey", title: "1. 仅搜索记录（视觉开）",
      text: "飞往投放区做 W/Z/I 视觉搜索并记录桶坐标后返航；不瞄准、不投放（视觉识别开启）。",
      keys: ["survey_only_mode", "min_bucket_count", "record_buckets",
        "bucket_ratio_ref_200"],
      set: { flight_only_mode: false, survey_only_mode: true, aim_only_mode: false,
        full_mission_mode: true, ground_test: false, recon_enabled: false, dry_run: false,
        record_buckets: true, area_search_pattern: "W", search_altitude_m: 3.0 }
    },
    {
      id: "drop", title: "2. 搜索 + 瞄准 + 投放",
      text: "完整走 搜索 → 选桶 → 瞄准 → 投放 流程；会发布 /servo/cmd。需要只演练时再手动打开 dry_run。",
      keys: ["flight_only_mode", "dry_run", "bucket_ratio_ref_200", "bucket_ratio_tol", "record_buckets"],
      set: { flight_only_mode: false, survey_only_mode: false, aim_only_mode: false,
        full_mission_mode: true, ground_test: false, recon_enabled: false, dry_run: false,
        enable_b_dropper: true,
        record_buckets: true, area_search_pattern: "W", search_altitude_m: 3.0 }
    },
    {
      id: "drop_real", title: "3. 搜索 + 瞄准投放 + 飞往侦察区",
      text: "与第 2 步相同的完整流程，但实际投放；此处只选择是否飞往侦察区，并设置侦察区距离 / 悬停高度 / 悬停时间。",
      keys: ["recon_enabled", "recon_zone_distance_m", "recon_altitude_m", "recon_hold_s"],
      set: { flight_only_mode: false, survey_only_mode: false, aim_only_mode: false,
        full_mission_mode: true, ground_test: false, dry_run: false, recon_enabled: true,
        enable_b_dropper: true,
        record_buckets: true, recon_zone_distance_m: 60.0, recon_altitude_m: 5.0, recon_hold_s: 5.0 }
    }
  ];

  /* 不在"参数（按飞行流程步骤）"里渲染的字段（避免与步骤卡重复）：
     - flight_only_* 的圈数/高度等进阶参数仍隐藏；flight_only_mode 本身作为互斥模式开关显示；
     - 步骤卡已暴露的 12 项：survey_only_mode/area_search_pattern/search_altitude_m/
       min_bucket_count/record_buckets/bucket_ratio_ref_200/bucket_ratio_tol/dry_run/
       recon_enabled/recon_zone_distance_m/recon_altitude_m/recon_hold_s。
     这些值仍会被保存：步骤卡里有同名输入，collectValues 会采集。 */
  var HIDDEN_FIELDS = { flight_only_pattern_laps: 1,
    flight_only_search_altitude_m: 1, flight_only_continue_to_recon: 1,
    survey_only_mode: 1,
    min_bucket_count: 1, record_buckets: 1, bucket_ratio_ref_200: 1,
    bucket_ratio_tol: 1, dry_run: 1,
    recon_enabled: 1, recon_zone_distance_m: 1, recon_altitude_m: 1, recon_hold_s: 1 };

  /* 这些分组不在"参数"区渲染，而是挂到对应卡片内的折叠区（避免重复/过长）。 */
  var QUICK_MOUNT = { launch: "launch-config", servo: "servo-config" };

  /* 参数卡片：按"飞行流程步骤"分组（不是按"功能"）。
     groups = 沿用后端 schema 的分组名；fields = 从 mission 组里挑出的字段。 */
  var STEP_GROUPS = [
    { title: "① 依赖自启 / 收尾（环境）", groups: ["launch"] },
    { title: "② 起飞 · 航点高度 / 容差", groups: ["waypoint"],
      fields: ["takeoff_altitude_m", "drop_zone_transit_altitude_m"] },
    { title: "③ 投放区 · 区域搜索（W/Z/I）", groups: ["search"],
      fields: ["area_search_pattern", "search_altitude_m", "drop_zone_distance_m",
        "bucket_record_dir", "bucket_assoc_distance_m"] },
    { title: "④ 瞄准 · 投放（视觉）", groups: ["aim", "vision"],
      fields: ["full_mission_mode", "enable_b_dropper", "aim_only_mode",
        "front_dropper", "crosshair_offset_m", "aim_cam_align_threshold_m",
        "ground_test", "close_after_drop", "bucket_ratio_invert"] },
    { title: "⑤ 舵机测试（servo_test.py）", groups: ["servo"] },
    { title: "⑥ 超时 / 恢复", groups: ["timeouts"] },
    { title: "⑦ 进阶 · 锁定 / 滤波 / 限幅 / 依赖",
      groups: ["target_lock", "search_spin", "speeds", "deps"] }
  ];

  function $(id) { return document.getElementById(id); }
  function esc(value) {
    return String(value == null ? "" : value).replace(/[&<>"]/g, function (c) {
      return { "&": "&amp;", "<": "&lt;", ">": "&gt;", "\"": "&quot;" }[c];
    });
  }
  function request(path, options) {
    options = options || {};
    options.headers = Object.assign({}, options.headers || {});
    if (authToken) {
      options.headers["X-Auth-Token"] = authToken;
      window.sessionStorage.setItem("cuadc_webui_token", authToken);
    }
    return fetch(path, options).then(function (r) {
      return r.json().catch(function () { return { ok: false, error: "响应不是 JSON" }; }).then(function (x) {
        if (!r.ok && !x.error) x.error = "HTTP " + r.status;
        return x;
      });
    });
  }
  function valueFor(name, type) {
    var v = state.values[name];
    if (type === "bool") return !!v;
    return v == null ? "" : v;
  }
  function fieldByName(name) { return state.byName[name]; }
  function readInput(f, input) {
    if (f.type === "bool") return input.checked;
    if (f.type === "int") return parseInt(input.value, 10);
    if (f.type === "float") return parseFloat(input.value);
    return input.value;
  }
  /* 同一参数可能同时出现在"步骤卡片"和"参数卡片"两处：改动一处，同步另一处。 */
  function setValue(name, value, except) {
    state.values[name] = value;
    document.querySelectorAll('[data-name="' + name + '"]').forEach(function (el) {
      if (el === except) return;
      if (el.type === "checkbox") {
        el.checked = !!value;
        var line = el.parentNode;   // 同步"已开启/已关闭"文字
        if (line && line.lastChild && line.lastChild.nodeType === 3) {
          line.lastChild.nodeValue = el.checked ? " 已开启" : " 已关闭";
        }
      } else {
        el.value = value;
      }
    });
    // 流程模式互斥：① 仅搜索记录(survey_only_mode) 与 ② 仅瞄准(aim_only_mode) 不能同开。
    if (value === true) {
      if (name === "survey_only_mode") setValue("aim_only_mode", false, null);
      else if (name === "aim_only_mode") setValue("survey_only_mode", false, null);
    }
  }
  function rangeText(f) {
    var bits = [];
    var unit = f.unit;
    if (!unit) {
      if (/ratio|alpha|threshold|tolerance/i.test(f.name)) unit = "无量纲";
      else if (f.type === "int") unit = "次/个";
      else unit = "—";
    }
    bits.push("单位：" + unit);
    if (f.min != null) bits.push("最小：" + f.min);
    if (f.max != null) bits.push("最大：" + f.max);
    if (f.enum) bits.push("可选：" + f.enum.join(" / "));
    if (f.launch_only) bits.push("启动相关");
    return bits.join("；") || "无额外范围";
  }
  function renderField(f) {
    var wrap = document.createElement("label");
    wrap.className = "field";
    var title = document.createElement("span");
    title.className = "field-title";
    title.textContent = f.name + (f.unit ? " [" + f.unit + "]" : "");
    var help = document.createElement("span");
    help.className = "field-help";
    help.textContent = f.comment || "暂无说明";
    var meta = document.createElement("span");
    meta.className = "field-meta";
    meta.textContent = rangeText(f);
    var input;
    if (f.type === "bool") {
      input = document.createElement("input");
      input.type = "checkbox";
      input.checked = valueFor(f.name, f.type);
      input.className = "toggle";
      input.dataset.name = f.name;
      var line = document.createElement("span");
      line.className = "check-line";
      line.appendChild(input);
      line.appendChild(document.createTextNode(input.checked ? " 已开启" : " 已关闭"));
      input.addEventListener("change", function () {
        line.lastChild.nodeValue = input.checked ? " 已开启" : " 已关闭";
        setValue(f.name, input.checked, input);
      });
      wrap.appendChild(title); wrap.appendChild(help); wrap.appendChild(meta); wrap.appendChild(line);
    } else {
      if (f.enum) {
        input = document.createElement("select");
        f.enum.forEach(function (x) { var o = document.createElement("option"); o.value = x; o.textContent = x; input.appendChild(o); });
      } else {
        input = document.createElement("input");
        input.type = f.type === "str" ? "text" : "number";
        if (f.type === "float") input.step = "any";
        if (f.min != null) input.min = f.min;
        if (f.max != null) input.max = f.max;
      }
      input.dataset.name = f.name;
      input.value = valueFor(f.name, f.type);
      input.addEventListener("change", function () { setValue(f.name, readInput(f, input), input); });
      wrap.appendChild(title); wrap.appendChild(help); wrap.appendChild(meta); wrap.appendChild(input);
    }
    return wrap;
  }
  function renderSteps() {
    var root = $("flows"); root.innerHTML = "";
    STEPS.forEach(function (step) {
      var card = document.createElement("details"); card.className = "flow"; card.open = true;
      var sum = document.createElement("summary");
      var h = document.createElement("h3"); h.textContent = step.title; sum.appendChild(h);
      card.appendChild(sum);
      var body = document.createElement("div"); body.className = "flow-body";
      // 「载入本步默认参数」放最上面
      var b = document.createElement("button"); b.className = "flow-load"; b.textContent = "载入本步默认参数";
      b.addEventListener("click", function () {
        Object.keys(step.set).forEach(function (k) { state.values[k] = step.set[k]; });
        renderSteps(); renderConfig();
        $("config-state").textContent = "已载入：" + step.title + "（尚未保存）";
      });
      body.appendChild(b);
      var p = document.createElement("p"); p.textContent = step.text; body.appendChild(p);
      var grid = document.createElement("div"); grid.className = "switch-grid";
      step.keys.forEach(function (n) { var f = fieldByName(n); if (f) grid.appendChild(renderField(f)); });
      body.appendChild(grid);
      var hint = document.createElement("p"); hint.className = "muted";
      hint.textContent = "① 仅搜索记录（不投放）；② 实投：搜索 → 选桶 → 瞄准 → 投放（会发布 /servo/cmd；只演练再手动开 dry_run）；③ 实投，并可选择去侦察区。载入本步会覆盖其它步的模式开关。";
      body.appendChild(hint);
      card.appendChild(body);
      root.appendChild(card);
    });
  }
  function renderConfig() {
    var root = $("config"); root.innerHTML = "";
    var mounts = {};
    Object.keys(QUICK_MOUNT).forEach(function (k) {
      var el = $(QUICK_MOUNT[k]);
      if (el) { el.innerHTML = ""; mounts[k] = el; }
    });
    var used = {};
    STEP_GROUPS.forEach(function (sg, idx) {
      // 单一分组且配了挂载点的（launch→任务控制卡 / servo→舵机卡）渲染到对应卡内，默认折叠。
      var key = (sg.groups && sg.groups.length === 1) ? sg.groups[0] : null;
      var host = (key && mounts[key]) ? mounts[key] : null;
      var isQuick = !!host;
      var fields = [];
      (sg.fields || []).forEach(function (n) {
        var f = fieldByName(n);
        if (f && !used[f.name] && !HIDDEN_FIELDS[f.name]) { used[f.name] = 1; fields.push(f); }
      });
      (sg.groups || []).forEach(function (g) {
        state.fields.forEach(function (f) {
          if (f.group === g && !used[f.name] && !HIDDEN_FIELDS[f.name]) {
            used[f.name] = 1; fields.push(f);
          }
        });
      });
      if (!fields.length) return;
      var d = document.createElement("details");
      d.className = "config-section";
      d.open = false;   // 一律先折叠；由左侧导航控制单页展开
      var sum = document.createElement("summary"); sum.textContent = sg.title; d.appendChild(sum);
      d.configTitle = sg.title;
      var body = document.createElement("div"); body.className = "config-body";
      fields.forEach(function (f) { body.appendChild(renderField(f)); });
      d.appendChild(body);
      (host || root).appendChild(d);
    });
    var rest = state.fields.filter(function (f) {
      return !used[f.name] && !HIDDEN_FIELDS[f.name];
    });
    if (rest.length) {
      var d = document.createElement("details"); d.className = "config-section";
      var sum = document.createElement("summary"); sum.textContent = "其它 / 未归类"; d.appendChild(sum);
      var body = document.createElement("div"); body.className = "config-body";
      rest.forEach(function (f) { body.appendChild(renderField(f)); });
      d.appendChild(body);
      root.appendChild(d);
    }
    buildConfigNav();
  }

  /* 左侧文字按钮导航：单页顺序显示——点按钮打开该页并折叠其它页，滚动到该页中心 */
  function buildConfigNav() {
    var root = $("config"), nav = $("config-nav");
    if (!root || !nav) return;
    nav.innerHTML = "";
    var pages = Array.prototype.slice.call(root.querySelectorAll("details.config-section"));
    pages.forEach(function (d) { d.open = false; });
    var btns = [];
    pages.forEach(function (d) {
      var b = document.createElement("button");
      var s = d.querySelector ? d.querySelector("summary") : null;
      b.textContent = d.configTitle || (s ? s.textContent : "页");
      b.addEventListener("click", function () {
        pages.forEach(function (o) { o.open = (o === d); });
        btns.forEach(function (x) { x.classList.remove("active"); });
        b.classList.add("active");
        if (d.scrollIntoView) d.scrollIntoView({ behavior: "smooth", block: "center" });
      });
      btns.push(b);
      nav.appendChild(b);
    });
    if (pages.length) {
      pages[0].open = true;
      btns[0].classList.add("active");
    }
  }
  function loadConfig() {
    $("config-state").textContent = "读取中…";
    return request("/api/config").then(function (r) {
      if (!r.ok) throw new Error(r.error || (r.errors || []).join("；"));
      state.values = r.values || {};
      state.schema = r.schema || { group_order: [], fields: [] };
      state.fields = state.schema.fields || [];
      state.byName = {};
      state.fields.forEach(function (f) { state.byName[f.name] = f; });
      renderSteps();
      renderConfig();
      // 拉起识别后是否默认打开识别画面（浏览器端）由配置决定，默认 true
      if (state.values.detector_auto_open_view === false) { if (videoOpen) closeVideo(); }
      else { if (!videoOpen) openVideo(); }
      $("config-state").textContent = "已读取：" + (r.path || "auto_drop.yaml");
    }).catch(function (e) { $("config-state").textContent = "读取失败：" + e.message; });
  }
  function saveConfig() {
    // 完整任务与仅飞行模式互斥；避免旧 YAML 的 flight_only=true 抢先进入 RTL 分支。
    if (state.values.full_mission_mode && state.values.flight_only_mode) {
      setValue("flight_only_mode", false, null);
      $("config-state").textContent = "已自动关闭冲突项 flight_only_mode；完整任务才能进入搜索/瞄准/投放";
    }
    $("config-state").textContent = "保存中…";
    return request("/api/config", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ values: state.values })
    }).then(function (r) {
      if (!r.ok) throw new Error((r.errors || [r.error || "保存失败"]).join("；"));
      $("config-state").textContent = "已保存（旧文件已备份）";
      renderSteps(); renderConfig();
    }).catch(function (e) { $("config-state").textContent = "保存失败：" + e.message; });
  }
  /* ---------------- 右上角：一键启动任务依赖（三圆按钮 + 链路下拉） ---------------- */
  function setLaunchMsg(text) { var b = $("launch-message"); if (b) b.textContent = text || ""; }
  function startBundleUi(link) {
    if (link === "real" && !window.confirm("将启动真机 MAVROS（含相机/识别/舵机）。确认飞机处于安全状态？")) return;
    setLaunchMsg("正在启动依赖…");
    return request("/api/start_bundle", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ link: link })
    }).then(function (r) {
      setLaunchMsg(r.ok ? (r.message || "依赖已启动") : ("启动失败：" + (r.error || "")));
    });
  }
  function stopMavrosAll() {
    if (!window.confirm("确认关闭所有 MAVROS（仿真 + 真机）？")) return;
    setLaunchMsg("正在关闭 MAVROS…");
    return request("/api/mavros/stop", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: "{}"
    }).then(function (r) {
      setLaunchMsg(r.ok ? "已关闭所有 MAVROS" : "部分未关闭（见日志/进程）");
    });
  }

  /* ---------------- 舵机测试（A/B 抛投器） ---------------- */
  /* ---------------- 一键启动仿真 / 一键杀后台（原桌面按钮） ---------------- */
  function startSimUp() {
    if (!window.confirm("一键启动仿真：先清场（杀仿真/MAVROS/相机/识别等），再 demo_up 重新拉起（约 1–2 分钟）。继续？")) return;
    if ($("mission-state")) $("mission-state").textContent = "正在启动仿真（清场+拉起）…";
    return request("/api/process/start", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: "sim_up" })
    }).then(function (d) {
      if ($("mission-state")) {
        $("mission-state").textContent = d.ok ? ("仿真启动中 pid=" + d.pid + "（看实时日志 sim_up）")
          : ("启动失败：" + (d.error || ""));
      }
    });
  }
  function killSim() {
    if (!window.confirm("一键杀后台进程：将杀掉仿真/MAVROS/相机/识别/舵机等残留（不会杀 WebUI 服务）。继续？")) return;
    if ($("mission-state")) $("mission-state").textContent = "正在清场…";
    return request("/api/process/start", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name: "kill_sim" })
    }).then(function (d) {
      if ($("mission-state")) {
        $("mission-state").textContent = d.ok ? ("清场中 pid=" + d.pid + "（看实时日志 kill_sim）")
          : ("清场失败：" + (d.error || ""));
      }
    });
  }

  function mavrosRealRunning() {
    return request("/api/processes").then(function (d) {
      var p = (d.processes || []).filter(function (x) { return x.name === "mavros_real"; })[0];
      return !!(p && p.running);
    });
  }
  /* 舵机卡状态：脚本（已开启/未开启）+ 真机 MAVROS（已连接/未连接） */
  function pollServoStatus() {
    return request("/api/processes").then(function (d) {
      var list = d.processes || [];
      var pick = function (n) {
        return list.filter(function (x) { return x.name === n; })[0] || {};
      };
      var st = pick("servo_test"), mr = pick("mavros_real");
      var se = $("servo-script-state");
      if (se) {
        se.textContent = "舵机测试脚本：" + (st.running
          ? ("已开启" + (st.pid ? " (pid=" + st.pid + ")" : "")) : "未开启");
      }
      var me = $("servo-mavros-state");
      if (me) me.textContent = "真机 MAVROS：" + (mr.running ? "已连接" : "未连接");
    });
  }
  function servoAction(action) {
    mavrosRealRunning().then(function (real) {
      if (!real && !window.confirm("未检测到真机 MAVROS。舵机测试需要真机链路，仍要下发指令吗？")) return;
      return request("/api/servo", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: action })
      }).then(function (d) {
        if ($("servo-msg")) $("servo-msg").textContent = d.ok
          ? ("已发送 /servo/cmd: " + (d.action || action)) : ("失败：" + (d.error || ""));
        pollServoStatus();
      });
    });
  }
  function startServoTest() {
    mavrosRealRunning().then(function (real) {
      if (!real && !window.confirm("未检测到真机 MAVROS。舵机测试脚本需要真机链路，仍要启动吗？")) return;
      return request("/api/process/start", {
        method: "POST", headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: "servo_test", confirm: true })
      }).then(function (d) {
        if ($("servo-msg")) $("servo-msg").textContent = d.ok
          ? ("舵机测试脚本已启动 pid=" + d.pid) : ("启动失败：" + (d.error || ""));
        pollServoStatus();
      });
    });
  }
  /* ---------------- 启动全流程 + Enter 授权（免终端） ---------------- */
  function startMission() {
    if (!window.confirm("确认启动全流程任务？将按当前 YAML 参数运行（含切模式/解锁/起飞）。")) return;
    if ($("mission-state")) $("mission-state").textContent = "正在启动…";
    request("/api/mission/start", {
      method: "POST", headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ confirm: true })
    }).then(function (d) {
      if ($("mission-state")) {
        $("mission-state").textContent = d.ok ? ("任务已启动 pid=" + d.pid)
          : ("启动失败：" + (d.error || ""));
      }
      // 任务拉起识别后，按配置自动打开识别画面
      if (d.ok && state.values.detector_auto_open_view !== false) openVideo();
    });
  }
  function confirmMission() {
    request("/api/mission/confirm", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: "{}"
    }).then(function (d) {
      if ($("mission-state")) {
        $("mission-state").textContent = d.ok ? "已发送 Enter（授权起飞）"
          : ("确认失败：" + (d.error || ""));
      }
    });
  }
  function stopMission() {
    if (!window.confirm("确认停止当前全流程任务？")) return;
    request("/api/mission/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}" })
      .then(function (d) {
        if ($("mission-state")) $("mission-state").textContent = d.ok ? "已请求停止" : ("停止失败：" + (d.error || ""));
      });
  }
  function pollMission() {
    return request("/api/mission/status").then(function (d) {
      var el = $("mission-state"); if (!el) return;
      if (d.running) el.textContent = "任务运行中 pid=" + d.pid;
      else if (el.textContent.indexOf("任务运行中") >= 0
            || el.textContent.indexOf("任务已启动") >= 0) el.textContent = "任务已结束";
    });
  }
  /* ---------------- 实时任务日志 ---------------- */
  var LOG_NAMES = ["mission", "sim_up", "kill_sim", "demo_up", "mavros_sim", "mavros_real",
    "camera", "detector", "servo_test", "ros_probe"];
  function fillLogNames() {
    var sel = $("log-name"); if (!sel) return;
    sel.innerHTML = "";
    LOG_NAMES.forEach(function (n) { var o = document.createElement("option"); o.value = n; o.textContent = n; sel.appendChild(o); });
    sel.value = "mission";
  }
  function rawText(path) {
    var headers = {};
    if (authToken) headers["X-Auth-Token"] = authToken;
    return fetch(path, { headers: headers }).then(function (r) { return r.text(); })
      .catch(function (e) { return "（读取日志失败: " + e + "）"; });
  }
  var LOG_ANCHOR = null;   // "清空面板"后：只显示该锚点之后新增的行
  function applyLogAnchor(text) {
    if (!LOG_ANCHOR) return text;
    var idx = text.lastIndexOf(LOG_ANCHOR);
    if (idx < 0) return text;            // 锚点已被 tail 窗口冲掉 → 老老实实全显示
    return text.slice(idx + LOG_ANCHOR.length).replace(/^\n+/, "");
  }
  function pollLog() {
    if (!$("log-auto").checked) return Promise.resolve();
    var name = $("log-name").value || "mission";
    return rawText("/api/log?name=" + encodeURIComponent(name) + "&tail=400").then(function (full) {
      var shown = applyLogAnchor(full);
      var v = $("log-view");
      if (v.textContent === shown) return;
      v.textContent = shown;
      v.scrollTop = v.scrollHeight;
    });
  }
  /* 一键复制：优先 Clipboard API，非安全上下文（局域网 http）回退 execCommand */
  function legacyCopy(text) {
    try {
      var ta = document.createElement("textarea");
      ta.value = text; ta.style.position = "fixed"; ta.style.left = "-9999px";
      document.body.appendChild(ta); ta.select();
      var ok = document.execCommand("copy");
      document.body.removeChild(ta);
      return ok;
    } catch (e) { return false; }
  }
  function copyText(text) {
    if (navigator.clipboard && navigator.clipboard.writeText) {
      return navigator.clipboard.writeText(text).then(function () { return true; })
        .catch(function () { return legacyCopy(text); });
    }
    return Promise.resolve(legacyCopy(text));
  }

  /* ---------------- 识别画面（带标注）：轮询单帧 /api/frame ---------------- */
  var videoOpen = false, videoUrl = null, videoTimer = null;
  function paintFrame() {
    if (!videoOpen) return;
    var headers = {};
    if (authToken) headers["X-Auth-Token"] = authToken;
    fetch("/api/frame?t=" + Date.now(), { headers: headers, cache: "no-store" })
      .then(function (r) { return r.blob(); })
      .then(function (blob) {
        if (!videoOpen) return;
        var img = $("live-view"); if (!img) return;
        var url = URL.createObjectURL(blob);
        img.src = url;
        if (videoUrl) URL.revokeObjectURL(videoUrl);
        videoUrl = url;
      })
      .catch(function () { /* 忽略单次失败，下一轮再试 */ })
      .then(function () { if (videoOpen) videoTimer = setTimeout(paintFrame, 150); });
  }
  function openVideo() {
    if (videoOpen) return;
    videoOpen = true;
    if ($("video-toggle")) $("video-toggle").textContent = "关闭画面";
    if ($("video-state")) $("video-state").textContent = "画面连接中…";
    paintFrame();
  }
  function closeVideo() {
    videoOpen = false;
    if (videoTimer) { clearTimeout(videoTimer); videoTimer = null; }
    var img = $("live-view"); if (img) img.removeAttribute("src");
    if (videoUrl) { URL.revokeObjectURL(videoUrl); videoUrl = null; }
    if ($("video-toggle")) $("video-toggle").textContent = "打开画面";
    if ($("video-state")) $("video-state").textContent = "未打开";
  }
  /* ---------------- 飞机状态栏 + 实时三桶表（同一份 /api/status） ---------------- */
  function vecText(v, d) {
    if (!v || !v.length) return "—";
    return v.map(function (x) { return Number(x).toFixed(d === undefined ? 3 : d); }).join(", ");
  }
  function renderFc(st) {
    var bar = $("fc-bar"); if (!bar) return;
    var fc = st.fc || {}, s = st.state || {};
    bar.innerHTML = "";
    var items = [
      ["模式 / 解锁", (s.mode || "—") + " / " + (s.armed == null ? "—" : s.armed), s.mode ? "ok" : "warn"],
      ["高度 rel_alt(相对起飞点)", fc.rel_alt == null ? "—" : (Number(fc.rel_alt).toFixed(2) + " m"), fc.rel_alt == null ? "" : "ok"],
      ["航向", fc.heading == null ? "—" : (Number(fc.heading).toFixed(1) + "°（" + (fc.heading_src || "?") + "，0=北 顺时针）"), fc.heading == null ? "" : "ok"],
      ["FC NED", fc.ned ? ("N " + fc.ned.n + "  E " + fc.ned.e + "  D " + fc.ned.d) : "—", ""],
      ["FC WGS84", fc.lat == null ? "—" : (Number(fc.lat).toFixed(7) + ", " + Number(fc.lon).toFixed(7) + ", " + Number(fc.alt).toFixed(1) + " m"), ""],
      ["电池", fc.voltage == null ? "—" : (Number(fc.voltage).toFixed(1) + " V / " + Number(fc.current || 0).toFixed(1) + " A"), ""],
      ["卫星 / fix", (fc.satellites == null ? "—" : fc.satellites) + " / " + (fc.fix_type == null ? "—" : fc.fix_type), ""],
      ["链路 / state_age", (s.connected == null ? "—" : s.connected) + " / " + (s.age_s == null ? "—" : (s.age_s + " s")), ""],
    ];
    items.forEach(function (it) {
      var div = document.createElement("div"); div.className = "item";
      div.innerHTML = esc(it[0]) + ": <b class='" + it[2] + "'>" + esc(it[1]) + "</b>";
      bar.appendChild(div);
    });
    if ($("fc-state")) $("fc-state").textContent = "已更新";
  }
  function renderLiveBuckets(st) {
    var tb = document.querySelector("#live-bucket-table tbody"); if (!tb) return;
    var list = st.buckets_live || [];
    tb.innerHTML = "";
    if (!list.length) {
      var tr0 = document.createElement("tr"); tr0.className = "empty-row";
      tr0.innerHTML = "<td colspan='5'>暂无识别目标</td>"; tb.appendChild(tr0); return;
    }
    list.forEach(function (t) {
      var head = document.createElement("tr"); head.className = "picked";
      head.innerHTML = "<td colspan='5'>#" + esc(t.id) + "  conf=" + esc(t.confidence)
        + "  size_proxy=" + (t.size_proxy_m ? Number(t.size_proxy_m).toFixed(3) + " m" : "—")
        + "  " + esc(t.status) + "</td>";
      tb.appendChild(head);
      [["前(+5cm)", t.front_cam, t.front_ned, t.front_wgs84],
       ["后(-5cm)", t.rear_cam, t.rear_ned, t.rear_wgs84]].forEach(function (row) {
        var tr = document.createElement("tr");
        var cam = row[1] || [];
        var dist = (cam.length === 3 && cam[2] > 0)
          ? Math.hypot(cam[0], cam[1], cam[2]).toFixed(2) : "—";
        var w = row[3] || [];
        tr.innerHTML = "<td>" + esc(row[0]) + "</td>"
          + "<td class='mono'>" + vecText(row[1]) + "</td>"
          + "<td class='mono'>" + vecText(row[2]) + "</td>"
          + "<td class='mono'>" + ((w[0] || w[1]) ? (Number(w[0]).toFixed(7) + ", " + Number(w[1]).toFixed(7) + ", " + Number(w[2]).toFixed(1)) : "—") + "</td>"
          + "<td>" + dist + "</td>";
        tb.appendChild(tr);
      });
    });
  }
  function pollStatusUi() {
    return request("/api/status").then(function (d) {
      var st = (d.status || {}).data || {};
      renderFc(st);
      renderLiveBuckets(st);
      var el = $("video-state");
      if (el && videoOpen) {
        if (st.stream_on && st.frame_age_s != null && st.frame_age_s < 2.0) {
          el.textContent = "画面传输中（" + (st.frame_hz || 0) + " fps）";
        } else if (st.stream_on) {
          el.textContent = "已订阅，等待识别画面…";
        } else {
          el.textContent = "连接中…";
        }
      }
    });
  }

  /* ---------------- 桶坐标 / 大小（脚本识别并锁定） ---------------- */
  var SIZE_NAME = { 150: "小号", 200: "中号", 250: "大号" };
  var SIZE_FROM = { small: 150, s: 150, mid: 200, m: 200, medium: 200, large: 250, l: 250 };
  var BUCKET_FILE = "";
  function fmt(v, d) {
    if (v === null || v === undefined || v === "") return "—";
    if (typeof v === "number") { return isNaN(v) ? "—" : (d === undefined ? String(v) : v.toFixed(d)); }
    return String(v);
  }
  function wgs(lat, lon) {
    var has = function (v) { return v !== null && v !== undefined && v !== ""; };
    return (has(lat) && has(lon)) ? (Number(lat).toFixed(7) + ", " + Number(lon).toFixed(7)) : "—";
  }
  function zoneText(z) {
    if (!z) return "—";
    if (Array.isArray(z) && z.length >= 2) return wgs(z[0], z[1]);
    if (typeof z === "object") {
      var la = (z.lat !== undefined) ? z.lat : z.latitude, lo = (z.lon !== undefined) ? z.lon : z.longitude;
      return wgs(la, lo);
    }
    return String(z);
  }
  function selLabel(detail, ids) {
    if (detail && detail.length) {
      return detail.map(function (s, i) {
        var mm = SIZE_FROM[String(s.size_class || "").toLowerCase()];
        return "#" + s.id + (SIZE_NAME[mm] ? " " + SIZE_NAME[mm] : "") + (i === 0 ? "（先投）" : "");
      }).join(" → ");
    }
    return (ids && ids.length) ? ids.join(" → ") : "—";
  }
  function fillBucketFiles(history) {
    var sel = $("bucket-file"); if (!sel) return;
    var names = (history || []).map(function (h) { return h.name; });
    var sig = names.join("|") + "::" + BUCKET_FILE;
    if (sel._sig === sig) return;
    sel._sig = sig;
    sel.innerHTML = "";
    var latest = document.createElement("option"); latest.value = "__latest__";
    latest.textContent = "最新（buckets_latest.json）"; sel.appendChild(latest);
    (history || []).forEach(function (h) {
      var o = document.createElement("option"); o.value = h.name;
      o.textContent = h.name + "  (" + h.time + ")"; sel.appendChild(o);
    });
    sel.value = BUCKET_FILE || "__latest__";
    var meta = (history || []).filter(function (h) { return h.name === BUCKET_FILE; })[0];
    $("bucket-time").textContent = meta ? meta.time : "";
  }
  function item(box, k, v, cls) {
    var div = document.createElement("div"); div.className = "item";
    div.innerHTML = esc(k) + ": <b class='" + (cls || "") + "'>" + esc(v) + "</b>";
    box.appendChild(div);
  }
  function pollBuckets() {
    var path = "/api/buckets" + (BUCKET_FILE ? ("?file=" + encodeURIComponent(BUCKET_FILE)) : "");
    return request(path).then(function (d) {
      var sum = $("bucket-summary"), zones = $("bucket-zones"), notes = $("bucket-notes");
      sum.innerHTML = ""; zones.innerHTML = ""; notes.innerHTML = "";
      if (!d.ok) { item(sum, "读取桶接口失败", d.error || "", "err"); return; }
      fillBucketFiles(d.history);
      var live = (d.live_count === null || d.live_count === undefined) ? "未见" : d.live_count;
      item(sum, "记录桶数", d.count || 0, d.count ? "ok" : "");
      item(sum, "实时识别 count", live, d.live_count ? "ok" : "");
      item(sum, "本次先投", (d.requested_drop != null) ? ("#" + d.requested_drop) : "未指定", d.requested_drop != null ? "ok" : "warn");
      item(sum, "选中桶(先→后)", selLabel(d.selected_detail, d.selected), (d.selected && d.selected.length) ? "ok" : "");
      if (d.mission && d.mission.pattern) {
        item(sum, "搜索航型/高度", d.mission.pattern + " / " + fmt(d.mission.search_altitude_m) + "m"
          + (d.mission.recon_enabled ? " · 侦察区开" : ""), "");
      }
      if (d.ts) item(sum, "记录时间", d.ts, "");
      item(zones, "投放区 WGS84", zoneText(d.drop_zone), "");
      item(zones, "侦察区 WGS84", zoneText(d.recon_zone), "");
      (d.notes || []).forEach(function (n) { item(notes, "备注", n, "warn"); });

      var tb = document.querySelector("#bucket-table tbody"); tb.innerHTML = "";
      var bs = d.buckets || [];
      if (!bs.length) {
        var tr = document.createElement("tr"); tr.className = "empty-row";
        tr.innerHTML = "<td colspan='7'>暂无记录</td>"; tb.appendChild(tr); return;
      }
      bs.forEach(function (b) {
        var tr = document.createElement("tr"), cls = [];
        if (String(b.id) === String(d.requested_drop)) cls.push("picked");
        else if (b.selected) cls.push("selected-row");
        if (cls.length) tr.className = cls.join(" ");
        var mm = b.diameter_class_mm;
        var dia = (mm != null) ? ((SIZE_NAME[mm] || "") + " [" + mm + "mm]") : "";
        tr.innerHTML = "<td>#" + esc(b.id) + (b.selected ? " ✓" : "") + "</td>"
          + "<td class='mono'>" + wgs(b.lat, b.lon) + "</td>"
          + "<td>" + fmt(b.ratio) + "</td>"
          + "<td>" + esc(dia || "—") + "</td>"
          + "<td>" + fmt(b.distance_m) + "</td>"
          + "<td>" + fmt(b.samples) + "</td>"
          + "<td class='muted'>" + esc(b.updated_at || "—") + "</td>";
        tb.appendChild(tr);
      });
    });
  }
  function loop(fn, ms) { return setInterval(function () { fn(); }, ms); }

  $("reload").addEventListener("click", loadConfig);
  $("save").addEventListener("click", saveConfig);
  if ($("btn-start-sim")) $("btn-start-sim").addEventListener("click", function () { startBundleUi("sim"); });
  if ($("btn-start-real")) $("btn-start-real").addEventListener("click", function () { startBundleUi("real"); });
  if ($("btn-stop-mavros")) $("btn-stop-mavros").addEventListener("click", stopMavrosAll);
  if ($("sim-up")) $("sim-up").addEventListener("click", startSimUp);
  if ($("kill-sim")) $("kill-sim").addEventListener("click", killSim);
  if ($("servo-start")) $("servo-start").addEventListener("click", startServoTest);
  [["servo-a-on", "a_on"], ["servo-a-off", "a_off"], ["servo-b-on", "b_on"],
   ["servo-b-off", "b_off"], ["servo-all-off", "all_off"], ["servo-status", "status"]
  ].forEach(function (pair) {
    if ($(pair[0])) $(pair[0]).addEventListener("click", function () { servoAction(pair[1]); });
  });
  if ($("log-refresh")) $("log-refresh").addEventListener("click", pollLog);
  if ($("log-name")) $("log-name").addEventListener("change", function () { LOG_ANCHOR = null; pollLog(); });
  if ($("log-copy")) $("log-copy").addEventListener("click", function () {
    var text = $("log-view").textContent;
    copyText(text).then(function (ok) {
      var box = $("launch-message");
      if (box) box.textContent = ok ? ("日志已复制到剪贴板（" + text.length + " 字符）")
        : "复制失败：请手动选中日志后复制";
    });
  });
  if ($("log-clear")) $("log-clear").addEventListener("click", function () {
    var cur = $("log-view").textContent;
    var lines = cur.split("\n").filter(function (x) { return x.trim() !== ""; });
    LOG_ANCHOR = lines.length ? lines[lines.length - 1] : null;   // 记住当前末行
    $("log-view").textContent = "（面板已清空：从现在起只显示新增日志；刷新页面可看完整 tail）";
    $("log-view").scrollTop = 0;
  });
  if ($("video-toggle")) $("video-toggle").addEventListener("click", function () {
    if (videoOpen) closeVideo(); else openVideo();
  });
  if ($("mission-start")) $("mission-start").addEventListener("click", startMission);
  if ($("mission-confirm")) $("mission-confirm").addEventListener("click", confirmMission);
  if ($("mission-stop")) $("mission-stop").addEventListener("click", stopMission);
  if ($("bucket-file")) $("bucket-file").addEventListener("change", function () {
    BUCKET_FILE = ($("bucket-file").value === "__latest__") ? "" : $("bucket-file").value;
    $("bucket-file")._sig = null;
    pollBuckets();
  });
  fillLogNames();
  loadConfig();
  pollLog();
  pollBuckets();
  pollStatusUi();            // 识别画面是否自动打开由配置 detector_auto_open_view 决定（读配置后处理）
  loop(pollLog, 2000);
  loop(pollBuckets, 3000);
  loop(pollStatusUi, 1000);
  pollMission();
  loop(pollMission, 3000);
  pollServoStatus();
  loop(pollServoStatus, 3000);
})();
