# 项目协作约定（AGENTS.md）— CUADC2026 ROS1→ROS2 翻译与仿真验证

> 本文件是**常驻项目提示词**：每个新 session 自动加载，不必再手写。
> 只放"长期有效"的规则/路径/命令；当次任务照常在下一条消息里说。

## 目录（重要）
- **源仓库（只读）**：`/home/lab/GDPI_CUADC_2026` —— 禁止修改，只能读。
- **工作区与全部产物（可写）**：`/home/lab/ROS2_CUADC2026`
  - 三台账：`错题本.md` / `验证记录.md` / `验证手册.md`
  - 运维与验证脚本：`scripts/`（preflight / demo_up / env_guard / verify_* / request_streams / fc_* 等）
  - 交付源码：`src/cuadc_control/scripts/`、`src/cuadc_vision/scripts/`

## 铁律（开工必做）
1. **先读台账**：`~/ROS2_CUADC2026/错题本.md` 的 PRE-FLIGHT 清单 + `#12`(ENU 约定) + `#16c`(链路只认流实据) + `#18`(测量三坑) + `#40`(高度门控参考系) + `#41`(到点容差)，以及 `验证记录.md` 最新小节。**源仓库只读，一切产物落 `~/ROS2_CUADC2026`。**
2. **ROS2 环境**：`source /opt/ros/humble/setup.bash && source ~/ROS2_CUADC2026/install/setup.bash`（顺序不能反）；直跑节点参数一律 `--ros-args -p k:=v`（裸 `-p`/`k:=v` 会被 rcl 静默当 remap 失效）。
3. **动仿真前先体检**：`bash ~/ROS2_CUADC2026/scripts/preflight.sh`；有 FAIL 就 `bash ~/ROS2_CUADC2026/scripts/demo_up.sh`（`--reset` 全清重建）；**同一时刻只跑一个仿真用例**。
4. **杀进程按 pid 精确 kill**，禁用 `pkill` 模糊杀（会杀掉自己所在 shell）。
5. **日志一律 `tee` 落盘**（`/tmp/opencode/`），不要裸管道接 `tail`（错题 #4）。**链路真伪只认话题流实据**，不信 `/mavros/state`（transient_local 缓存假象，#16c）。
6. 新脚本 `chmod +x`（#20）；写完 verify 脚本在 `/tmp/opencode/` 留一份 `.keep.py` 备份（#38）。
7. **高度/坐标先对参考系**（#40/#41）：
   - 一切高度判据与指令用"**相对起飞点**" = `rel_alt`（相对 Home）+ 解锁后地面 `ground_rel_alt` 零点；
   - `pose.z` 是"EKF 局部原点以上高度"（本地 setpoint 坐标），**不当高度判据**；
   - 跨坐标系（ENU/NED）翻译先验证，别照抄符号；
   - **航段容差必须 < 航段长度**（否则"未移动即判到达"）。
8. **交付纪律**：改完先在仿真里自己跑通/复验，再给用户看；并更新三台账（错题本新增条目 / 验证手册"验收标准+清单" / 验证记录新增小节）。
9. **测量必带独立真值**：高度类验证用 Gazebo `iris_demo` 世界 z（`gz.transport13` + `gz.msgs10.pose_v_pb2.Pose_V`），禁止拿飞控自身通道互证。

## 常用命令
```bash
# 体检 / 一键拉起
bash ~/ROS2_CUADC2026/scripts/preflight.sh
bash ~/ROS2_CUADC2026/scripts/demo_up.sh            # 幂等修复
bash ~/ROS2_CUADC2026/scripts/demo_up.sh --reset    # 全清重建

# 编译
cd ~/ROS2_CUADC2026 && colcon build --packages-select cuadc_control cuadc_vision

# 请求飞控数据流（确认 SITL 活着再跑）
python3 ~/ROS2_CUADC2026/scripts/request_streams.py

# 高度门控排查/量化（rel_alt vs pose.z vs gz 真值）
python3 ~/ROS2_CUADC2026/scripts/verify_takeoff_alt.py \
    --ros-args -p takeoff_altitude:=3.0 -p child:=takeoff     # child:=forward 对照
```

## 沟通约定
- **先给排查/实施计划，确认后再动手**；结论用大白话 + 可复现的实据（带时间戳/命令）。
- 涉及既有脚本改动：最小改动、与相邻脚本风格一致，并说明影响面。

---

# 常驻背景：桶识别 / 投放测试流程（每个 session 必读，2026-09-30 起）

【项目背景 — 每个 session 先读这段】

你是 CUADC2026"无人机 ROS1→ROS2 翻译与验证"项目的协作 session。项目有多个 session 并行，
你只负责下面【本次任务】那一段，其它部分不要动。

一、目录与环境（铁律）
- 只读源仓库：/home/lab/GDPI_CUADC_2026（禁止修改，只能读）。
- 工作区（可写，所有产物落这里）：/home/lab/ROS2_CUADC2026。
- 开工先读：~/ROS2_CUADC2026/AGENTS.md；错题本.md 的 PRE-FLIGHT 清单 +
  #4(日志落盘)/#12(ENU)/#15(按 pid 精确 kill,禁用 pkill)/#16c(链路只认话题流实据,不信 /mavros/state)/
  #18(测量三坑)/#20(新脚本 chmod +x)/#26(直跑节点参数必须 --ros-args -p k:=v)/#27(信号收口)/
  #40(高度一律"相对起飞点"= rel_alt+地面基准)/#41(航段容差<航段长度)/#42/#43(舵机 A/B=AUX1/2=SERVO9/10)/
  #44(ros2 topic pub --once 无订阅者会阻塞)；验证记录.md 最新小节；验证手册.md。
- ROS2 环境：source /opt/ros/humble/setup.bash && source ~/ROS2_CUADC2026/install/setup.bash（顺序不能反）。
- 日志一律 tee 落 /tmp/opencode/；同一时刻只跑一个仿真用例；真机写参/动作必须先让用户确认。
- 交付纪律：先给实施计划→用户确认→最小改动→自测→给用户演示→更新三台账
  （验证记录.md 新增小节 / 错题本.md 如踩新坑入表 / 验证手册.md 清单补行）。
- 沟通：结论用大白话 + 可复现实据（命令 + 时间戳 + 日志路径 /tmp/opencode/*）。

二、比赛目标
- 飞机代号：UCES/FEK-104。出发自贡比赛时间节点：2026-10-06。
- 口号：[初心高于胜负，成长胜于输赢][穷究原理，洞悉本质]。
- 拿奖关键：飞机必须能判断桶的大小（直径约 150mm / 200mm / 250mm），
  选出"中号(≈200mm) + 小号(<200mm)"两个桶，并先投离飞机更近的那个。

三、系统架构（现状，务必先读代码确认）
- 控制（src/cuadc_control/scripts/）：
  * auto_drop_test.py —— 全自动投放主状态机（约 3650 行），读 config/auto_drop.yaml；
    已支持 W/Z/I 区域搜索、A/B 瞄准投放、侦察区、flight_only 模式；返回码 0=完成/1=失败/2=参数错。
  * semi_auto_drop_test.py —— 半自动投放；【本期冻结，逻辑先不动】。
  * one_key_takeoff.py / one_key_takeoff_forward_land.py / one_key_takeoff_wgs84_forward_rtl.py
    —— 起飞 / NED 起飞前飞降落 / WGS84 起飞前飞返航（用于分步测试）。
- 视觉（src/cuadc_vision/scripts/ 与同接口的 src/cuadc_visionY26/scripts/，默认用 Y26）：
  * camera_node.py —— 图像源 realsense(真机)/gazebo(仿真)。
  * detector_node.py —— 识别；当前输出 /vision/yolo/detections(YoloDetections)，
    可选发布 /vision/target_global(GeoTarget, 单目标 WGS84)；订阅 /vision/mission_status 显示弹药。
  * servo_test.py —— 舵机终端（节点名 servo_controller）；订阅 /servo/cmd(String)、/servo/open(Bool)，
    发布 /servo/ready(Bool)；A/B 通道参数 servo_a_channel/servo_b_channel 默认 9/10（#43）。
- WebUI（webui/）：server.py（纯标准库后端）+ static/{index.html,app.js,style.css}（原生离线前端）。
  配置单一事实源 config/auto_drop.yaml；auto_drop_test.py 用 --config 读它；
  优先级：命令行 --ros-args -p > --config > CUADC_AUTO_DROP_CONFIG > 默认路径 > 内置默认。
- 关键话题/消息：
  * /vision/color/image_raw、/vision/aligned_depth/image_raw、/vision/color/camera_info
  * /vision/yolo/detections (YoloDetections)：每目标含 confidence、bbox(x_min/y_min/x_max/y_max/center)、
    depth_m、position_valid、camera_*/body_* 坐标、distance_m、bbox_width_m、bbox_height_m。
  * /vision/target_global (GeoTarget)：单目标 WGS84(latitude/longitude/altitude)+enu/body/camera。
  * /servo/cmd(String)、/servo/ready(Bool)、/servo/open(Bool)。
  * /vision/mission_status (MissionStatus)：ammo_a/ammo_b/aiming/last_drop（识别节点订阅它显示弹药）。

四、我们的测试飞行流程（webui 要把这个流程可视化，测试员要明白当前在测哪一步）
1) 测一键起飞 OK。
2) 测 NED 起飞 + 前飞 + 降落 OK。
3) 测 WGS84 起飞 + 前飞 + 返航 OK。
4) 测"手动发现单个目标 → 授权进入自动瞄准 + 投放 → 退出"；同时单独测舵机控制脚本功能 OK。
5) 测自动飞往投放区，仅做识别、记录桶坐标，然后返航。
6) 测自动飞往投放区：识别 → 记录桶坐标 → 全部记录完毕后，按"深度/识别框对角线宽度"的比值，
   选出中号(≈200mm)与小号(<200mm)两个桶，判断哪个离飞机更近，先投近的；投放完返航。
7) 在 1–6 都完成的基础上，最后勾选"完成投放后飞往侦察区"，用 webui 预设侦察区距离/高度/悬停时间
   （目的是让操作手通过图传肉眼评估侦察区识别效果）。

五、共通约束（所有任务都遵守）
- 不改只读源仓库；不破坏既有默认行为（不传 yaml / 不勾选新功能时，行为与现在完全一致，需回归）。
- 半自动脚本 semi_auto_drop_test.py 逻辑先不动；改识别节点要"保留既有话题与字段"以保回归。
- 涉及真机写参/动作先确认；所有新增参数进 config/auto_drop.yaml，并带中文注释与范围。
- 新增脚本 chmod +x；改 CMakeLists 的 install 后 colcon build 验证。
- 任何"看起来成功"的链路判定必须给流实据（ros2 topic hz），不接受 /mavros/state 单点缓存。

六、已确认的接口/配置契约（★用户已拍板，所有 session 遵守，不得各自改）
1) 识别输出接口：**不新增 msg，沿用旧消息 GeoTarget**。识别节点每帧**实时发布多个目标**的 GeoTarget
   （最多 3 个），并用既有字段**标记目标序号 1/2/3**（用 status 或 class_name 承载 "1/2/3"；
   若确需在 GeoTarget 增加 int32 target_id，属"旧消息扩展"，必须先写进计划请用户确认）。
   既有 /vision/target_global 与订阅者兼容不破。
2) 识别节点**只输出**：桶数量 + 各目标 WGS84 坐标（实时）。**不做坐标平滑/估计、不记录、不落盘**。
   "坐标跳变"的**位置估计**由**全自动脚本**在搜索过程中完成。
3) 尺寸比值：由**全自动脚本**计算——用相机 depth_m 与识别框对角线/宽高（结合 camera_info 内参）折算；
   **200mm 桶的参考比值由测试员用相机对准 200mm 桶后手工填入**（bucket_ratio_ref_200）；
   150/250mm 阈值同法标定或以参考值推导；最终以实测标定为准。
4) 剩余弹数：沿用 /vision/mission_status 的 ammo_a/ammo_b（识别节点已订阅显示/门控）。
5) 桶坐标记录：由**全自动脚本**以**原子写**落盘到 ~/ROS2_CUADC2026/logs/buckets/：
   - buckets_<时间戳>.json（每轮一份，断电/重启后可重新读取分析）
   - buckets_latest.json（WebUI 实时读取）
   内容：全部识别到的桶坐标 + 最终选中的两个 + 大小档位/比值 + 距飞机的距离 + 时间戳。
6) 投放区/侦察区锁定：**沿用 ROS1 逻辑**——运行前手工把机头对准投放区/侦察区**中线**；
   脚本运行时按**起始航向 + 配置距离**自动锁定两地 WGS84（现有 plan_waypoints() 即此逻辑）。
   WebUI 显示锁定出的投放区/侦察区 WGS84。
7) 选桶与降级：目标桶 = 比值**最接近参考值(200mm)** 的桶 + 比值**小于参考值**的桶（中号+小号），
   **先投离飞机更近者**；若搜索到的桶不足三个/只有两个，则**直接飞向这两个桶依次投放**。

七、新增配置键（建议，进 config/auto_drop.yaml，由 T2 定义、T3 在页面暴露）
- mission.search_altitude_m     搜索高度（起飞后先到该高度，再在该高度飞往投放区）
- mission.recon_enabled         完成投放/搜索后是否飞往侦察区（默认 false＝保持现有行为）
- mission.aim_only_mode         "仅飞行+仅瞄准"：识别到 3 个桶后切入瞄准（默认 false）
- mission.record_buckets        是否记录桶坐标并上报 WebUI（默认 true）
- mission.bucket_ratio_ref_200  200mm 桶标定比值（测试员手工填入）
- mission.bucket_ratio_tol      比值分档容差（标定后填）
- mission.bucket_record_dir     记录目录（默认 ~/ROS2_CUADC2026/logs/buckets）

【本次任务】见下（只做这一段，别越界）。

## 落地注记（2026-09-30 起，实现现状；与上面契约冲突时以本节为准）
- 契约 1) 已按"旧消息扩展"落地：`GeoTarget.msg` 末尾新增 `int32 target_id` 与 `int32 count`（旧字段全保留）；
  识别节点每帧对最多 3 个目标各发一条 `/vision/target_global`，`target_id` 按 confidence 降序 1/2/3，
  `count`=本帧全部目标数。**不再用 status/class_name 承载序号。**
- 尺寸比值实现：`ratio = depth_m / diag_px`（diag_px = 识别框像素对角线 sqrt(w²+h²)）；
  `bucket_ratio_ref_200`（200mm 手工标定）+ `bucket_ratio_tol`；比值越大 ⇒ 框越小 ⇒ 桶越小；
  方向可用 `bucket_ratio_invert` 反转（实测标定后定）。
- 桶记录实现：`logs/buckets/buckets_<时间戳>.json` + `buckets_latest.json`（原子写；断电/重启可读）；
  字段见 T2 交付说明（schema/generated_at/mission/buckets[]/selected[]/drop_order[]/notes）。
- **接口漂移警告**：工作区里另有被停 agent 留下的 `BucketEstimates.msg` / `BucketTarget.msg` 与
  `/vision/bucket/estimates` 话题（其 `ratio` 是 `bbox_width_m/bbox_height_m` 宽高比，与上面的尺寸比值不是一回事）。
  **只消费 `/vision/target_global` 的 GeoTarget(target_id/count)，不要消费 BucketEstimates**；
  清理/取舍由 T3/T4 的"完成度分析"决定。
- 用户授权：用户已明确"全部功能都授权，不要找我要权限"（含起/重置仿真、构建、启停本项目节点、跑验收），
  不必逐项等待确认；但真机写参/真实动作仍应**先备份、再执行、留台账**。
- 任务分解与完整提示词：`docs/任务提示词_桶识别与投放流程_20260930.md`（P0 + T1–T4）。
