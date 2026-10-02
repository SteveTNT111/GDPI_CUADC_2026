#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""auto_drop_config.py — auto_drop_test.py 的 YAML 配置单一事实源（纯 Python，不 import ROS）。

职责（被 auto_drop_test.py 与 webui/server.py 共用）：
  1. 模式表 SCHEMA：分组的全部可调参数（类型/默认/中文注释/单位/范围/枚举）。
  2. 读 YAML：load_file() 递归拍平成 {ROS 参数名: 值}（YAML 分组名 -> 叶子名一一映射）。
  3. 解析顺序（§3.B）：CLI `-p` > `--config` > CUADC_AUTO_DROP_CONFIG > 默认路径 > 内置默认。
     - CLI `-p` 由 rclpy 在 create_node 时预先声明，故 apply_to_node() 只兜底声明
       "尚未声明"的参数 -> -p 天然最高优先级。
  4. 写 YAML：emit()（保留中文注释与分组，原子写由调用方负责）。
  5. 校验：validate() 镜像 auto_drop_test.AutoDropTest._validate_params 的规则
     （脚本运行时仍会再校验一次，失败 rc=2；本模块供 Web UI 保存前拒绝非法值）。

铁律：缺 YAML / 缺字段一律回落到内置默认，绝不因缺配置崩溃。
"""

import os

try:
    import yaml
except Exception:  # pragma: no cover - pyyaml 在本机已装（5.4.1）
    yaml = None

DEFAULT_PATH = os.path.join(
    os.path.expanduser("~"), "ROS2_CUADC2026", "config", "auto_drop.yaml")

# 分组的展示顺序与中文标题
GROUP_TITLES = {
    "launch": "依赖自启 / 收尾",
    "mission": "任务模式与航线",
    "search": "区域搜索",
    "aim": "瞄准 / 投放",
    "vision": "视觉",
    "servo": "舵机（与 servo_test.py 对齐，#43 真机 A1/A2=SERVO9/10）",
    "waypoint": "航点 / 容差",
    "timeouts": "超时 / 恢复（仿真低 RTF 需放宽；真机用默认）",
    "target_lock": "目标锁定 / 滤波（进阶）",
    "search_spin": "目标丢失搜索 / 滤波（进阶）",
    "speeds": "速度 / 步长限幅（进阶）",
    "deps": "其它依赖 / 遥测超时（进阶）",
}
GROUP_ORDER = list(GROUP_TITLES.keys())


def F(group, name, typ, default, comment, **kw):
    """构造一个字段描述。typ ∈ bool/int/float/str/enum；launch_only=仅启动器用。"""
    d = {
        "group": group, "name": name, "type": typ,
        "default": default, "comment": comment,
        "unit": kw.get("unit", ""),
        "min": kw.get("min"), "max": kw.get("max"),
        "enum": kw.get("enum"),
        "launch_only": bool(kw.get("launch_only", False)),
    }
    if "yaml_default" in kw:
        d["yaml_default"] = kw["yaml_default"]
    return d


# ======================================================================
# 模式表（default = 脚本内置默认；yaml_default = 首次生成 YAML 的推荐值）
# ======================================================================
SCHEMA = [
    # ---------- launch：依赖自启 / 收尾 ----------
    F("launch", "auto_start_mavros", "bool", False, "是否由任务脚本自动拉起 MAVROS"),
    F("launch", "auto_start_detector", "bool", False, "是否自动拉起 相机+识别"),
    F("launch", "auto_start_servo_test", "bool", False, "是否自动拉起 舵机测试节点"),
    F("launch", "auto_shutdown_children", "bool", False,
      "任务结束是否自动收尾它拉起的子进程（空中永远保留 MAVROS）"),
    F("launch", "mavros_fcu_url", "str", "/dev/ttyACM0:115200",
      "MAVROS 飞控 URL（真机网口 UDP / 仿真 tcp 5760）",
      yaml_default="udp://:14555@192.168.13.14:14550"),
    F("launch", "detector_package", "str", "cuadc_vision",
      "识别包：cuadc_vision 或 cuadc_visionY26",
      yaml_default="cuadc_visionY26", enum=["cuadc_vision", "cuadc_visionY26"]),
    F("launch", "detector_launch", "str", "detector_node.launch.py",
      "识别 launch 文件名"),
    F("launch", "camera_source", "str", "gazebo",
      "相机源：仿真 gazebo / 真机 realsense", enum=["gazebo", "realsense"],
      launch_only=True),
    F("launch", "detector_show_window", "bool", False, "识别是否弹标注窗口"),
    F("launch", "detector_auto_open_view", "bool", True,
      "拉起识别节点后默认拉起识别画面（浏览器端自动打开识别画面；识别端仅在有人看时发布标注图）"),

    # ---------- mission：任务模式与航线 ----------
    F("mission", "flight_only_mode", "bool", False, "仅飞行航线验证（禁视觉/舵机/投放）"),
    F("mission", "flight_only_pattern_laps", "int", 1, "仅飞行模式花形圈数", min=1),
    F("mission", "flight_only_search_altitude_m", "float", 3.0,
      "仅飞行模式搜索高度（相对起飞点；完整任务搜索高度见 search_altitude_m）",
      unit="m", min=0.1),
    F("mission", "flight_only_continue_to_recon", "bool", True,
      "仅飞行模式是否继续飞往侦察区"),
    F("mission", "aim_only_mode", "bool", False,
      "仅飞行+仅瞄准：投放区搜索识别到 3 个桶后切入瞄准（只对准，不投放）"),
    F("mission", "survey_only_mode", "bool", False,
      "仅搜索记录：飞往投放区做 W/Z/I 视觉搜索并记录桶坐标后返航（不瞄准、不投放；与 aim_only_mode 互斥）"),
    F("mission", "ground_test", "bool", False, "地面测试（不切模式/不解锁）"),
    F("mission", "full_mission_mode", "bool", True,
      "完整任务模式（总开关）：true=执行完整自主飞行——切 GUIDED→解锁→起飞→飞往投放区→"
      "搜索→瞄准/投放→（可去侦察区）→RTL；可叠加 aim_only_mode/survey_only_mode/recon_enabled 降级；"
      "false=仅做依赖/航点/授权检查，不切模式、不解锁、不起飞（干跑检查）"),
    F("mission", "enable_b_dropper", "bool", True, "启用 B 投放"),
    F("mission", "dry_run", "bool", False,
      "干跑：仅执行瞄准/投放流程，不实际控制舵机（不发 /servo/cmd）；但终端与识别画面仍会显示 drop"),
    F("mission", "close_after_drop", "bool", False, "投放后关闭"),
    F("mission", "min_bucket_count", "int", 1, "最少目标数", min=1),
    F("mission", "area_search_pattern", "enum", "W",
      "区域搜索花形", enum=["W", "Z", "I"]),
    F("mission", "takeoff_altitude_m", "float", 3.0, "起飞高度（相对起飞点）", unit="m", min=0.1),
    F("mission", "drop_zone_transit_altitude_m", "float", 3.0,
      "投放区航程高度", unit="m", min=0.1),
    F("mission", "drop_zone_distance_m", "float", 32.0, "投放区距离", unit="m", min=0.1),
    F("mission", "recon_zone_distance_m", "float", 60.0, "侦察区距离", unit="m", min=0.1),
    F("mission", "recon_altitude_m", "float", 5.0, "侦察高度", unit="m", min=0.1),
    F("mission", "recon_hold_s", "float", 5.0, "侦察悬停时间", unit="s", min=0.1),

    # ---------- mission：T2 桶搜索 / 选桶 / 记录（2026-09-30）----------
    F("mission", "search_altitude_m", "float", 3.0,
      "投放区搜索高度（相对起飞点；起飞后先到该高度，再在该高度飞往投放区做 W/Z/I 搜索）",
      unit="m", min=0.1),
    F("mission", "drop_zone_bearing_deg", "float", -1.0,
      "投放区/侦察区锁定航向（度，0=北顺时针；<0=用起始航向（真机默认：操作手把机头对准中线）；"
      "0..360=固定航向，仅仿真在起飞前 EKF 偏航未对齐时用 90 对齐场地中线）",
      unit="deg", min=-1.0, max=360.0),
    F("mission", "recon_enabled", "bool", False,
      "完成投放/搜索后是否飞往侦察区（默认 false=直接 RTL，对应测试流程 5/6；"
      "true=飞往侦察区悬停后再 RTL，对应流程 7）"),
    F("mission", "record_buckets", "bool", True,
      "是否把识别到的桶坐标原子写落盘 logs/buckets（供 WebUI/赛后分析）"),
    F("mission", "bucket_ratio_ref_200", "float", 0.20,
      "200mm 桶物理尺寸代理值（m，depth*sqrt((bbox_w/fx)^2+(bbox_h/fy)^2)，请用相机实测填入）",
      min=0.001, max=10.0),
    F("mission", "bucket_ratio_tol", "float", 0.15,
      "比值分档容差：比值>ref*(1+tol)=大号、<ref*(1-tol)=小号、之间=中号（物理尺寸代理，大=大号）",
      min=0.0, max=1.0),
    F("mission", "bucket_ratio_invert", "bool", False,
      "尺寸代理方向开关：默认比值大=桶大；仅在标定实测方向相反时置 true"),
    F("mission", "bucket_assoc_distance_m", "float", 0.6,
      "同一桶 ENU 关联门限（米）：须明显小于场地最小桶距（规则三筒随机摆放）、明显大于观测噪声；"
      "调大更抗噪但近距两桶会并档，调小反之", unit="m", min=0.05, max=2.0),
    F("mission", "bucket_record_dir", "str", "~/ROS2_CUADC2026/logs/buckets",
      "桶坐标记录目录（原子写 buckets_<时间戳>.json 并覆盖 buckets_latest.json）"),
    # ---- 双投放点（虚拟相机原点 ±5cm / 准星对准触发）----
    F("mission", "front_dropper", "enum", "A",
      "前投放器舵机（A/B）：前投用'前'准星与前坐标，后投用'后'", enum=["A", "B"]),
    F("mission", "crosshair_offset_m", "float", 0.05,
      "准星偏移（m）：虚拟相机原点沿机体系 X ±该值（须与 detector 的 dual_offset_m 一致）",
      unit="m", min=0.001, max=1.0),
    F("mission", "aim_cam_align_threshold_m", "float", 0.03,
      "对准触发阈值（m）：桶中心相对对应准星的相机系差值小于它即判已对准，触发投放",
      unit="m", min=0.001, max=1.0),

    # ---------- search：区域搜索 ----------
    F("search", "drop_zone_width_m", "float", 8.0,
      "投放区宽度（机身边方向 / 横向；场地规则 8m）", unit="m", min=0.1),
    F("search", "drop_zone_length_m", "float", 5.0,
      "投放区长度（机身前后 / 沿航线方向；场地规则 5m）", unit="m", min=0.1),
    F("search", "detect_range_ref_altitude_m", "float", 3.0,
      "识别范围参考高度（相对起飞点）：下面宽/长是在该高度测得的地面覆盖；实际按 "
      "实际搜索高度/参考高度 线性折算", unit="m", min=0.1),
    F("search", "detect_range_width_m", "float", 2.25,
      "识别范围宽度（机身边方向 / 画面横向；参考高度下）", unit="m", min=0.1),
    F("search", "detect_range_length_m", "float", 1.5,
      "识别范围长度（机身前后方向 / 画面纵向；参考高度下）", unit="m", min=0.1),
    F("search", "area_search_timeout_s", "float", 120.0, "区域搜索超时", unit="s", min=0.1),
    F("search", "area_search_waypoint_tolerance_m", "float", 0.25,
      "区域搜索到点容差", unit="m", min=0.01),
    F("search", "area_search_height_tolerance_m", "float", 0.20,
      "区域搜索高度容差", unit="m", min=0.01),
    F("search", "area_search_dwell_s", "float", 0.5, "区域搜索点停留", unit="s", min=0.1),
    F("search", "second_target_search_timeout_s", "float", 30.0,
      "第二目标搜索超时", unit="s", min=0.1),
    F("search", "reuse_area_search_anchor_for_b", "bool", True,
      "B 复用区域搜索锚点"),

    # ---------- aim：瞄准 / 投放 ----------
    F("aim", "coarse_align_height_m", "float", 2.0, "粗对准高度", unit="m", min=0.1),
    F("aim", "fine_align_height_m", "float", 1.7, "精对准高度", unit="m", min=0.1),
    F("aim", "target_drop_height_m", "float", 1.5, "投放高度", unit="m", min=0.1),
    F("aim", "coarse_align_threshold_m", "float", 0.45, "粗对准阈值", unit="m", min=0.01),
    F("aim", "fine_align_threshold_m", "float", 0.20, "精对准阈值", unit="m", min=0.01),
    F("aim", "align_threshold_m", "float", 0.15, "对准阈值", unit="m", min=0.01),
    F("aim", "height_tolerance_m", "float", 0.15, "高度容差", unit="m", min=0.01),
    F("aim", "stable_time_s", "float", 0.5, "稳定判定时间", unit="s", min=0.01),
    F("aim", "acquire_stable_time_s", "float", 0.8, "目标获取稳定时间", unit="s", min=0.01),
    F("aim", "phase_stable_time_s", "float", 0.5, "阶段稳定时间", unit="s", min=0.01),
    F("aim", "after_a_delay_s", "float", 3.0, "A 后 B 间隔", unit="s", min=0.01),
    F("aim", "require_distinct_b_target", "bool", True, "B 需要不同于 A 的目标"),
    F("aim", "distinct_target_distance_m", "float", 0.60, "A/B 目标最小间距", unit="m", min=0.01),
    F("aim", "setpoint_interval_s", "float", 0.1, "setpoint 发布间隔", unit="s", min=0.01),

    # ---------- vision ----------
    F("vision", "vision_min_confidence", "float", 0.30, "视觉最低置信度", min=0.0, max=1.0),

    # ---------- servo（转发给 servo_test.py）----------
    F("servo", "servo_a_channel", "int", 9, "舵机 A 输出通道（#43 真机 AUX1=SERVO9）",
      min=1, max=16, launch_only=True),
    F("servo", "servo_b_channel", "int", 10, "舵机 B 输出通道（#43 真机 AUX2=SERVO10）",
      min=1, max=16, launch_only=True),
    F("servo", "pwm_open", "int", 1500, "打开 PWM", min=800, max=2200, launch_only=True),
    F("servo", "pwm_close", "int", 1000, "关闭 PWM", min=800, max=2200, launch_only=True),
    F("servo", "enable_ch5", "bool", True, "启用舵机 A（沿用源命名 ch5）", launch_only=True),
    F("servo", "enable_ch6", "bool", True, "启用舵机 B（沿用源命名 ch6）", launch_only=True),

    # ---------- waypoint ----------
    F("waypoint", "waypoint_tolerance_m", "float", 1.0, "航点到点容差", unit="m", min=0.01),
    F("waypoint", "waypoint_vertical_tolerance_m", "float", 0.4,
      "航点垂直容差", unit="m", min=0.01),
    F("waypoint", "waypoint_arrival_hold_s", "float", 2.0, "到点保持时间", unit="s", min=0.01),
    F("waypoint", "waypoint_timeout_s", "float", 120.0, "航点超时", unit="s", min=0.1),

    # ---------- timeouts ----------
    F("timeouts", "state_timeout_s", "float", 2.0, "state 新鲜度超时", unit="s", min=0.1),
    F("timeouts", "guided_state_grace_s", "float", 5.0, "GUIDED state 宽限", unit="s", min=0.1),
    F("timeouts", "pose_timeout_s", "float", 2.0, "pose 新鲜度超时", unit="s", min=0.1),
    F("timeouts", "guided_pose_grace_s", "float", 5.0, "GUIDED pose 宽限", unit="s", min=0.1),
    F("timeouts", "navigation_timeout_s", "float", 2.0, "导航数据超时", unit="s", min=0.1),
    F("timeouts", "navigation_grace_s", "float", 5.0, "导航数据宽限", unit="s", min=0.1),
    F("timeouts", "connection_loss_grace_s", "float", 10.0,
      "链路丢失宽限", unit="s", min=0.1),
    F("timeouts", "connection_timeout_s", "float", 30.0, "连接超时", unit="s", min=0.1),
    F("timeouts", "mode_timeout_s", "float", 8.0, "模式切换超时", unit="s", min=0.1),
    F("timeouts", "takeoff_timeout_s", "float", 40.0, "起飞超时", unit="s", min=0.1),
    F("timeouts", "rtl_timeout_s", "float", 240.0, "RTL 超时", unit="s", min=0.1),
    F("timeouts", "service_retry_attempts", "int", 1, "服务重试次数", min=1),

    # ---------- target_lock（进阶）----------
    F("target_lock", "target_lock_center_threshold_m", "float", 0.20,
      "锁定中心阈值", unit="m", min=0.01),
    F("target_lock", "target_lock_window_s", "float", 0.5, "锁定窗口", unit="s", min=0.01),
    F("target_lock", "target_lock_min_samples", "int", 5, "锁定最少样本", min=2),
    F("target_lock", "target_lock_max_xy_span_m", "float", 0.10,
      "锁定 XY 最大跨度", unit="m", min=0.01),
    F("target_lock", "target_lock_max_d_span_m", "float", 0.15,
      "锁定深度最大跨度", unit="m", min=0.01),

    # ---------- search_spin（进阶）----------
    F("search_spin", "target_filter_alpha", "float", 0.25, "目标滤波系数(0,1]", min=0.001, max=1.0),
    F("search_spin", "max_target_jump_m", "float", 0.80, "目标跳变上限", unit="m", min=0.01),
    F("search_spin", "brief_loss_s", "float", 0.5, "短暂丢失时间", unit="s", min=0.0),
    F("search_spin", "loss_hover_s", "float", 2.0, "丢失悬停时间", unit="s", min=0.0),
    F("search_spin", "reacquire_height_m", "float", 2.0, "重捕获高度", unit="m", min=0.01),
    F("search_spin", "search_start_radius_m", "float", 0.30, "搜索起始半径", unit="m", min=0.01),
    F("search_spin", "search_radius_step_m", "float", 0.30, "搜索半径步长", unit="m", min=0.01),
    F("search_spin", "search_max_radius_m", "float", 1.20, "搜索最大半径", unit="m", min=0.01),
    F("search_spin", "search_dwell_s", "float", 1.5, "搜索停留时间", unit="s", min=0.01),
    F("search_spin", "target_loss_timeout_s", "float", 30.0, "目标丢失超时", unit="s", min=0.01),

    # ---------- speeds（进阶）----------
    F("speeds", "max_horizontal_setpoint_distance_m", "float", 0.60,
      "水平 setpoint 最大步距（命令点单次朝目标挪动的上限；与速率一起决定航线/搜索快慢）",
      unit="m", min=0.01),
    F("speeds", "max_vertical_setpoint_distance_m", "float", 0.20,
      "垂直 setpoint 最大步距（爬升/下降命令点单次挪动上限）", unit="m", min=0.01),
    F("speeds", "max_horizontal_setpoint_rate_mps", "float", 0.25,
      "水平 setpoint 速率（航线/搜索/瞄准的水平速度主旋钮：命令点以该速率向前挪，飞机跟飞）",
      unit="m/s", min=0.01),
    F("speeds", "max_vertical_setpoint_rate_mps", "float", 0.20,
      "垂直 setpoint 速率（爬升/下降速度主旋钮）", unit="m/s", min=0.01),
    F("speeds", "phase_max_horizontal_speed_mps", "float", 0.20,
      "阶段水平最大速度（瞄准阶段判定阈值：实测速度高于它不计入对准/阶段稳定，不限制飞行速度）",
      unit="m/s", min=0.01),
    F("speeds", "phase_max_vertical_speed_mps", "float", 0.15,
      "阶段垂直最大速度（瞄准阶段判定阈值，不限制飞行速度）", unit="m/s", min=0.01),
    F("speeds", "max_horizontal_speed_mps", "float", 0.10,
      "总体水平最大速度（最终停稳判定阈值：实测速度高于它不算停稳，不限制飞行速度）",
      unit="m/s", min=0.01),
    F("speeds", "max_vertical_speed_mps", "float", 0.10,
      "总体垂直最大速度（最终停稳判定阈值，不限制飞行速度）", unit="m/s", min=0.01),

    # ---------- deps（进阶：其它依赖/遥测超时）----------
    F("deps", "dependency_restart_attempts", "int", 0, "依赖重启次数", min=0),
    F("deps", "dependency_restart_backoff_s", "float", 2.0,
      "依赖重启退避", unit="s", min=0.0),
    F("deps", "detector_ready_timeout_s", "float", 30.0,
      "识别就绪超时", unit="s", min=0.01),
    F("deps", "servo_ready_timeout_s", "float", 20.0, "舵机就绪超时", unit="s", min=0.01),
    F("deps", "enroute_detect_stable_s", "float", 0.3,
      "途中检测稳定时间", unit="s", min=0.01),
    F("deps", "aim_timeout_s", "float", 20.0, "瞄准超时", unit="s", min=0.01),
    F("deps", "aim_info_timeout_s", "float", 1.0, "aim_info 新鲜度超时", unit="s", min=0.01),
    F("deps", "service_timeout_s", "float", 10.0, "服务调用超时", unit="s", min=0.01),
    F("deps", "servo_wait_timeout_s", "float", 20.0, "等待舵机订阅者超时", unit="s", min=0.01),
    F("deps", "close_delay_s", "float", 0.8, "关闭延时", unit="s", min=0.0),
    F("deps", "local_position_rate_hz", "float", 10.0, "位姿标称频率", unit="Hz", min=0.01),
    F("deps", "diagnostic_log_interval_s", "float", 1.0,
      "诊断日志间隔", unit="s", min=0.01),
    F("deps", "takeoff_started_threshold_m", "float", 0.30,
      "起飞已开始阈值", unit="m", min=0.01),
    F("deps", "service_retry_backoff_s", "float", 0.5,
      "服务重试退避", unit="s", min=0.0),
]

SCHEMA_BY_NAME = {f["name"]: f for f in SCHEMA}


# ======================================================================
# 读 / 拍平
# ======================================================================
def _flatten(data, out, prefix=""):
    if isinstance(data, dict):
        for key, value in data.items():
            if key is None:
                continue
            name = str(key)
            full = "%s.%s" % (prefix, name) if prefix else name
            _flatten(value, out, full)
    else:
        out[prefix] = data


def load_file(path):
    """读取 YAML -> ({参数名: 值}, [警告])。文件不存在时返回 ({}, 警告)。"""
    if not path:
        return {}, []
    if not os.path.isfile(path):
        return {}, ["配置文件不存在: %s（改用内置默认）" % path]
    if yaml is None:
        return {}, ["未安装 PyYAML，无法读取 %s（改用内置默认）" % path]
    try:
        with open(path, "r", encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
    except Exception as exc:
        return {}, ["配置文件解析失败: %s（改用内置默认）" % exc]
    if data is None:
        return {}, []
    if not isinstance(data, dict):
        return {}, ["配置文件顶层不是映射（改用内置默认）"]
    flat = {}
    _flatten(data, flat)
    warnings = []
    for key in flat:
        leaf = key.split(".")[-1]
        if leaf not in SCHEMA_BY_NAME:
            warnings.append("未知配置项: %s（已忽略其脚本注入）" % key)
    # 归一化：叶子名 -> 值（后出现的同名覆盖前者）
    return {k.split(".")[-1]: v for k, v in flat.items()}, warnings


def builtin_defaults(script_only=False):
    out = {}
    for f in SCHEMA:
        if script_only and f.get("launch_only"):
            continue
        out[f["name"]] = f["default"]
    return out


def effective_defaults():
    """首次生成 YAML 用的推荐值（yaml_default 优先，否则 default）。"""
    out = {}
    for f in SCHEMA:
        out[f["name"]] = f.get("yaml_default", f["default"])
    return out


# ======================================================================
# 解析顺序 / 注入
# ======================================================================
def extract_config_arg(argv):
    """从 argv 中摘除 `--config X` / `--config=X`，返回 (路径或 None, 新 argv)。"""
    path = None
    cleaned = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--config":
            if i + 1 < len(argv):
                path = argv[i + 1]
                i += 2
                continue
            i += 1
            continue
        if a.startswith("--config="):
            path = a.split("=", 1)[1]
            i += 1
            continue
        cleaned.append(a)
        i += 1
    return path, cleaned


def resolve_config_path(cli_path=None, env=None):
    """解析顺序：--config > CUADC_AUTO_DROP_CONFIG > 默认路径。返回路径字符串。"""
    if cli_path:
        return cli_path
    env = env if env is not None else os.environ
    env_path = env.get("CUADC_AUTO_DROP_CONFIG")
    if env_path:
        return env_path
    return DEFAULT_PATH


def apply_to_node(node, config_path):
    """把 YAML 值注入为「尚未声明」的 ROS 参数（-p 已被 rclpy 预先声明 -> 优先级更高）。

    返回 (生效路径, 值字典, 警告列表)。任何异常都不会抛出（缺配置不崩）。
    """
    # 默认路径缺失属正常（回归场景）→ 静默回落内置默认；显式指定却缺失 → 给提示。
    if not config_path or not os.path.isfile(config_path):
        warnings = []
        if config_path and config_path != DEFAULT_PATH:
            warnings.append("配置文件不存在: %s（改用内置默认）" % config_path)
        return config_path, {}, warnings, 0, 0
    values, warnings = load_file(config_path)
    applied = 0
    overridden = 0
    for name, value in values.items():
        if not isinstance(name, str) or not name:
            continue
        if isinstance(value, (dict, list)) or value is None:
            warnings.append("配置项 %s 类型不支持，已跳过" % name)
            continue
        try:
            if node.has_parameter(name):
                continue
            # declare 时若命令行 -p 提供了 override，则返回值即被覆盖后的生效值。
            effective = node.declare_parameter(name, value).value
            applied += 1
            if effective != value:
                overridden += 1
        except Exception as exc:
            warnings.append("注入参数 %s 失败: %s" % (name, exc))
    return config_path, values, warnings, applied, overridden


# ======================================================================
# 写 YAML（保留分组与中文注释）
# ======================================================================
def _fmt_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        # 保留 ".0" 尾巴，明确声明为浮点，避免 int/float 歧义
        return repr(round(value, 6))
    if value is None:
        return "null"
    if isinstance(value, str):
        # 只有明显需要时才加引号（URL 含 ':'，按 YAML 规范需引号）
        if value == "" or any(c in value for c in ":#{}[],&*!|>'\"%@`"):
            return '"%s"' % value.replace('"', '\\"')
        return value
    return str(value)


def _comment_for(field, value):
    parts = [field["comment"]]
    if field.get("unit"):
        parts.append(field["unit"])
    if field.get("enum"):
        parts.append("可选: " + "/".join(str(x) for x in field["enum"]))
    rng = []
    if field.get("min") is not None:
        rng.append("min=%s" % _fmt_scalar(field["min"]))
    if field.get("max") is not None:
        rng.append("max=%s" % _fmt_scalar(field["max"]))
    if rng:
        parts.append(" ".join(rng))
    return "（" + "；".join(parts) + "）"


def emit(values=None):
    """按分组 + 注释生成 YAML 文本。values 缺项回落到 yaml_default/default。"""
    values = values or {}
    lines = [
        "# =====================================================================",
        "# CUADC auto_drop_test.py 配置（单一事实源）",
        "# Web UI 读写本文件；auto_drop_test.py 运行时读本文件。",
        "# 解析优先级：命令行 --ros-args -p 明确覆盖 > 本文件 > 脚本内置默认。",
        "# 缺文件/缺字段一律回落内置默认，不会因此崩溃。",
        "# 生成时间随保存刷新；注释含单位与合法范围。",
        "# =====================================================================",
        "",
    ]
    for group in GROUP_ORDER:
        fields = [f for f in SCHEMA if f["group"] == group]
        if not fields:
            continue
        lines.append("# ===== %s =====" % GROUP_TITLES[group])
        lines.append("%s:" % group)
        for f in fields:
            name = f["name"]
            if name in values:
                value = values[name]
            else:
                value = f.get("yaml_default", f["default"])
            lines.append("  %s: %s  # %s"
                         % (name, _fmt_scalar(value), _comment_for(f, value)))
        lines.append("")
    return "\n".join(lines)


# ======================================================================
# 校验（镜像 auto_drop_test.AutoDropTest._validate_params）
# ======================================================================
def _num(values, name):
    f = SCHEMA_BY_NAME.get(name)
    if f is None:
        return None
    v = values.get(name, f["default"])
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def validate(values):
    """返回 (errors, warnings)。errors 非空即非法，Web UI 拒绝保存。"""
    errors = []
    warnings = []

    def gt0(*names):
        for name in names:
            v = _num(values, name)
            if v is None or v <= 0.0:
                errors.append("%s 必须大于 0" % name)

    def geq(a, b, label):
        va, vb = _num(values, a), _num(values, b)
        if va is not None and vb is not None and va < vb:
            errors.append(label)

    gt0("align_threshold_m", "target_drop_height_m", "height_tolerance_m",
        "stable_time_s", "setpoint_interval_s", "acquire_stable_time_s",
        "phase_stable_time_s", "diagnostic_log_interval_s", "pose_timeout_s",
        "local_position_rate_hz")
    geq("coarse_align_height_m", "fine_align_height_m",
        "coarse_align_height_m 不能低于 fine_align_height_m")
    geq("fine_align_height_m", "target_drop_height_m",
        "fine_align_height_m 不能低于 target_drop_height_m")
    geq("reacquire_height_m", "fine_align_height_m",
        "reacquire_height_m 不能低于 fine_align_height_m")

    alpha = _num(values, "target_filter_alpha")
    if alpha is None or not (0.0 < alpha <= 1.0):
        errors.append("target_filter_alpha 必须在 (0, 1] 范围")

    gt0("coarse_align_threshold_m", "fine_align_threshold_m", "max_target_jump_m",
        "search_start_radius_m", "search_radius_step_m", "search_max_radius_m",
        "search_dwell_s", "target_loss_timeout_s",
        "max_horizontal_setpoint_distance_m", "max_vertical_setpoint_distance_m",
        "max_horizontal_setpoint_rate_mps", "max_vertical_setpoint_rate_mps",
        "phase_max_horizontal_speed_mps", "phase_max_vertical_speed_mps",
        "max_horizontal_speed_mps", "max_vertical_speed_mps",
        "distinct_target_distance_m")

    brief, loss = _num(values, "brief_loss_s"), _num(values, "loss_hover_s")
    if brief is not None and loss is not None and (brief < 0.0 or loss < brief):
        errors.append("必须满足 0 <= brief_loss_s <= loss_hover_s")

    gsg, sts = _num(values, "guided_state_grace_s"), _num(values, "state_timeout_s")
    if gsg is not None and sts is not None and gsg <= sts:
        errors.append("guided_state_grace_s 必须大于 state_timeout_s")
    gpg, pts = _num(values, "guided_pose_grace_s"), _num(values, "pose_timeout_s")
    if gpg is not None and pts is not None and gpg <= pts:
        errors.append("guided_pose_grace_s 必须大于 pose_timeout_s")

    ng, nt = _num(values, "navigation_grace_s"), _num(values, "navigation_timeout_s")
    if ng is not None and nt is not None and ng <= nt:
        errors.append("navigation_grace_s 必须大于 navigation_timeout_s")

    pattern = str(values.get("area_search_pattern",
                             SCHEMA_BY_NAME["area_search_pattern"]["default"])).strip().upper()
    if pattern == "LINE":
        pattern = "I"
    if pattern not in ("W", "Z", "I"):
        errors.append("area_search_pattern 只能是 W、Z、I（LINE 等同 I）")

    laps = _num(values, "flight_only_pattern_laps")
    if laps is not None and int(laps) < 1:
        errors.append("flight_only_pattern_laps 必须大于等于 1")

    gt0("flight_only_search_altitude_m")
    fsa, ra = _num(values, "flight_only_search_altitude_m"), _num(values, "recon_altitude_m")
    if fsa is not None and ra is not None and fsa > ra:
        errors.append("flight_only_search_altitude_m 必须小于等于 recon_altitude_m")

    gt0("drop_zone_width_m", "drop_zone_length_m",
        "detect_range_ref_altitude_m", "detect_range_width_m", "detect_range_length_m",
        "area_search_timeout_s", "area_search_waypoint_tolerance_m",
        "area_search_height_tolerance_m", "area_search_dwell_s")
    aw, al = _num(values, "drop_zone_width_m"), _num(values, "drop_zone_length_m")
    ref = _num(values, "detect_range_ref_altitude_m")
    rng_w, rng_l = _num(values, "detect_range_width_m"), _num(values, "detect_range_length_m")
    alt_candidates = [v for v in (_num(values, "search_altitude_m"),
                                  _num(values, "flight_only_search_altitude_m")) if v]
    if ref and ref > 0 and alt_candidates and rng_w is not None and rng_l is not None:
        scale = max(alt_candidates) / ref
        if aw is not None and rng_w * scale >= aw:
            errors.append("识别范围宽度（按实际搜索高度折算=%.2fm）必须小于投放区宽度"
                          % (rng_w * scale))
        if al is not None and rng_l * scale >= al:
            errors.append("识别范围长度（按实际搜索高度折算=%.2fm）必须小于投放区长度"
                          % (rng_l * scale))

    gt0("drop_zone_distance_m")
    dz, rz = _num(values, "drop_zone_distance_m"), _num(values, "recon_zone_distance_m")
    if dz is not None and rz is not None and rz <= dz:
        errors.append("recon_zone_distance_m 必须大于 drop_zone_distance_m")

    gt0("takeoff_altitude_m", "drop_zone_transit_altitude_m")
    ta = _num(values, "takeoff_altitude_m")
    tr_alt = _num(values, "drop_zone_transit_altitude_m")
    if ra is not None and ta is not None and ra < ta:
        errors.append("recon_altitude_m 必须大于等于 takeoff_altitude_m")
    if ra is not None and tr_alt is not None and ra < tr_alt:
        errors.append("recon_altitude_m 必须大于等于 drop_zone_transit_altitude_m")

    # T2 桶搜索高度 / 比值标定
    if values.get("survey_only_mode") and values.get("aim_only_mode"):
        errors.append("survey_only_mode 与 aim_only_mode 不能同时为 true")
    if str(values.get("front_dropper", "A")).strip().upper() not in ("A", "B"):
        errors.append("front_dropper 只能是 A 或 B")
    gt0("search_altitude_m", "bucket_ratio_ref_200")
    sa = _num(values, "search_altitude_m")
    if sa is not None and ra is not None and sa > ra:
        errors.append("search_altitude_m 必须小于等于 recon_altitude_m")
    tol = _num(values, "bucket_ratio_tol")
    if tol is not None and not (0.0 <= tol <= 1.0):
        errors.append("bucket_ratio_tol 必须在 0..1 范围")
    ref = _num(values, "bucket_ratio_ref_200")
    if ref is not None and not (0.001 <= ref <= 10.0):
        errors.append("bucket_ratio_ref_200 必须在 0.001..10m 范围")
    assoc = _num(values, "bucket_assoc_distance_m")
    if assoc is not None and not (0.05 <= assoc <= 2.0):
        errors.append("bucket_assoc_distance_m 必须在 0.05..2m 范围")
    brd = values.get("bucket_record_dir")
    if brd is not None and not isinstance(brd, str):
        errors.append("bucket_record_dir 必须是字符串路径")

    gt0("recon_hold_s", "waypoint_tolerance_m", "waypoint_vertical_tolerance_m",
        "waypoint_arrival_hold_s", "waypoint_timeout_s", "detector_ready_timeout_s",
        "servo_ready_timeout_s", "enroute_detect_stable_s",
        "second_target_search_timeout_s", "aim_timeout_s",
        "target_lock_center_threshold_m", "target_lock_window_s",
        "target_lock_max_xy_span_m", "target_lock_max_d_span_m",
        "navigation_timeout_s", "navigation_grace_s", "connection_loss_grace_s",
        "takeoff_timeout_s", "takeoff_started_threshold_m", "rtl_timeout_s")

    # 舵机（servo_test.py 的规则）
    for ch in ("servo_a_channel", "servo_b_channel"):
        v = _num(values, ch)
        if v is None or not (1 <= int(v) <= 16):
            errors.append("%s 必须在 1..16（MAIN=1-8, AUX=9-16，#43）" % ch)
    po, pc = _num(values, "pwm_open"), _num(values, "pwm_close")
    if po is None or not (800 <= int(po) <= 2200):
        errors.append("pwm_open 必须在 800..2200")
    if pc is None or not (800 <= int(pc) <= 2200):
        errors.append("pwm_close 必须在 800..2200")
    if po is not None and pc is not None and int(po) == int(pc):
        errors.append("pwm_open 与 pwm_close 不能相同")

    # ---- 通用范围（min/max）----
    for f in SCHEMA:
        name = f["name"]
        if name in ("servo_a_channel", "servo_b_channel", "pwm_open", "pwm_close"):
            continue
        if f["type"] not in ("int", "float"):
            continue
        v = _num(values, name)
        if v is None:
            continue
        if f.get("min") is not None and v < float(f["min"]):
            errors.append("%s 不能小于 %s" % (name, _fmt_scalar(f["min"])))
        if f.get("max") is not None and v > float(f["max"]):
            errors.append("%s 不能大于 %s" % (name, _fmt_scalar(f["max"])))

    # T2 桶记录目录（与 auto_drop_test._validate_params 镜像；ref_200/tol 由上面的通用 min/max 覆盖）
    if not str(values.get("bucket_record_dir", "")).strip():
        errors.append("bucket_record_dir 不能为空")

    # ---- 警告（不阻断保存）：#41 航段容差必须 < 航段长度 ----
    if ta is not None and ta > 0:
        pass
    if dz is not None and _num(values, "waypoint_tolerance_m") is not None:
        if _num(values, "waypoint_tolerance_m") >= dz:
            warnings.append(
                "waypoint_tolerance_m >= drop_zone_distance_m：可能未移动即判到达（#41）")
    if aw is not None and _num(values, "area_search_waypoint_tolerance_m") is not None:
        seg = min(aw, al if al is not None else aw)
        if seg > 0 and _num(values, "area_search_waypoint_tolerance_m") >= seg:
            warnings.append(
                "area_search_waypoint_tolerance_m >= 搜索区短边：可能未移动即判到达（#41）")
    if ta is not None and _num(values, "takeoff_started_threshold_m") is not None:
        if _num(values, "takeoff_started_threshold_m") >= ta:
            warnings.append("takeoff_started_threshold_m >= takeoff_altitude_m，起飞判定可能失效")

    return errors, warnings


def describe():
    """给 Web UI 的字段元数据（含分组标题）。"""
    return {
        "group_order": GROUP_ORDER,
        "group_titles": GROUP_TITLES,
        "fields": [
            {
                "group": f["group"], "name": f["name"], "type": f["type"],
                "default": f["default"], "comment": f["comment"],
                "unit": f["unit"], "min": f["min"], "max": f["max"],
                "enum": f["enum"], "launch_only": f["launch_only"],
            }
            for f in SCHEMA
        ],
        "default_path": DEFAULT_PATH,
    }


def ensure_config(path=DEFAULT_PATH):
    """若配置文件不存在则按推荐默认生成。返回 (是否存在, 路径)。"""
    if os.path.isfile(path):
        return True, path
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(emit(effective_defaults()))
    os.replace(tmp, path)
    return False, path
