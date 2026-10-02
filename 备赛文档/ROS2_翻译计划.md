---
tags:
  - cuadc
  - ROS2迁移
  - 计划
created: 2026-09-21
status: 待执行
---

# ROS1 → ROS2 翻译计划（逐个翻译 + Gazebo 验证）

> 原则：**每翻译完一个脚本 → 立即在 Gazebo 里验证**（SITL + 比赛场地），确认飞机能按
> 比赛要求（起飞→投放→侦察→返航着陆，全自动）和小队要求（WGS84 setpoint、半自动投放等）完成动作。
>
> 不通过就不进入下一个脚本。

---

## 0. 仿真验证环境（已确认就绪）

| 组件 | 路径/版本 |
|------|----------|
| ROS2 | Humble（`/opt/ros/humble`） |
| Gazebo | Harmonic（`GZ_VERSION=harmonic`，`/usr/bin/gz`） |
| ArduPilot SITL | `~/ardupilot-4.7.1`，`sim_vehicle.py` 支持 `gazebo-*` 帧型 |
| ardupilot_gazebo 插件 | `~/ardupilot_gazebo`（已编译，含 `iris_with_standoffs` 模型） |
| 模型/世界库 | `~/SITL_Models/Gazebo` → `GZ_SIM_RESOURCE_PATH` |
| **比赛场地** | `备赛文档/机架电装选型/cuadc_2026_field.world`（起降H圈、起飞线、投放区3筒、侦察区） |
| MAVROS | `ros-humble-mavros 2.14.0` 已装 |
| 编译目标工作区 | 新建独立文件夹 `~/ROS2_CUADC2026/`（翻译产物全放这，**不修改/不动任何原 ROS1 文件**） |

### 0.0 一次性环境搭建（执行前先做）
1. 建立 `~/ROS2_CUADC2026/`：`src/cuadc_vision`（msg/srv 接口）、`src/cuadc_control`（纯 Python）。原 `代码/` 全部只读，不复制覆盖。
2. 把比赛 world 复制进 GZ 资源路径，iris 模型确认带相机插件。
3. 跑通最小链路：
   `gz sim cuadc_2026_field.world` + `sim_vehicle.py -v ArduCopter -f gazebo-iris` + MAVROS
   → 确认 `/mavros/state` connected、`/mavros/local_position/pose`、`/mavros/global_position/global` 有数据。

---

## 1. 翻译顺序（依赖驱动，逐脚本）

### 阶段 A：基础层（无飞控依赖，先保证可被后续引用）

| # | 脚本/文件 | 原文件位置（比赛main工作版本） | ROS2 翻译要点 | Gazebo 验证 |
|---|-----------|------------------------------|--------------|------------|
| A1 | **5 个自定义消息** | `src/cuadc_vision/msg/*.msg`（BucketInfo / GeoTarget / MissionStatus / YoloDetection / YoloDetections） | 转成 ROS2 `msg` 接口（包内 `msg/`，`ament_cmake` 编译 `rosidl_generate_interfaces`） | `colcon build` 后 `ros2 interface show` 可用 |
| A2 | **competition_mission_common.py** (749行) | `scripts/competition_mission_common.py` | `rospy`→`rclpy`；`rospy.get_param`→`declare_parameter`；`rospy.Time`→`self.get_clock()`；srv 调用改异步/同步 `call`；`mavros_msgs` 服务/话题换成 humble 同名（接口未变）；`MAV_CMD_DO_SET_SERVO(183)` 走 `CommandLong` srv | 加一个自检入口：连 SITL 后依次 `set_mode(GUIDED)→arm→takeoff→command_long(servo 183)` 全部成功 |
| A3 | **one_key_takeoff.py** (389行) | `src/cuadc_vision/scripts/` + `cuadc_control/scripts/` 两处 | 去重一份到 `cuadc_control`；`rospy`→`rclpy` | SITL：一键起飞到设定高度，`/mavros/state.armed=true`、高度达标 |

### 阶段 B：视觉/相机（仿真里没有真 D435i，需做图像源抽象）

| # | 脚本 | 原文件 | ROS2 翻译要点 | Gazebo 验证 |
|---|------|--------|--------------|------------|
| B1 | **camera_node.py** (351行) | `scripts/camera_node.py` | 真机保留 `pyrealsense2` 驱动；**仿真模式**：订阅 GZ 相机话题（`ros_gz_bridge` 桥接 `/camera/color/image_raw`、depth、camera_info），统一对外发布 `/camera/color/image_raw` 等 | GZ 里 iris 相机插件出图，`ros2 topic hz` 稳定 ~30Hz，颜色/深度图能显示 |
| B2 | **detector_node.py** (1725行) | `scripts/detector_node.py`  | `rospy`→`rclpy`；YOLO onnxruntime 推理逻辑保留；像素→相机→机体→ENU→WGS84 变换保留；发布 `/vision/yolo/detections`、`/vision/target_global` 等 | 让飞机飞/悬停到投放区上方，**从话题看到筒检测框**（置信度>阈值）；坐标发布有值且方向合理 |

### 阶段 C：控制包（飞机动作验证）

| # | 脚本 | 原文件 | ROS2 翻译要点 | Gazebo 验证 |
|---|------|--------|--------------|------------|
| C1 | **one_key_takeoff_forward_land.py** (663行) | `cuadc_control/scripts/` | `rclpy`；话题/服务 API 同名迁移 | SITL：起飞→前飞（按 ENU/WGS84 航点）→降落，轨迹合理 |
| C2 | **one_key_takeoff_wgs84_forward_rtl.py** (1195行) | `cuadc_control/scripts/`  | **小队要求核心**：WGS84 setpoint 前飞 + RTL | SITL：验证 WGS84 setpoint → 位移正确 → RTL 返回到起点附近 |

### 阶段 D：主任务（最终整链验证）

| # | 脚本 | 原文件 | ROS2 翻译要点 | Gazebo 验证 |
|---|------|--------|--------------|------------|
| D1 | **competition_main.py** (497行) | `scripts/competition_main.py` | 状态机逻辑原样迁移；rclpy 参数、定时器模式 | **完整比赛流程仿真**：自动起飞→投放区搜索→选桶→瞄准→投放(2瓶)→侦察区→识别→RTL→着陆回 H 圈；全程无人干预，5 分钟内 |
| D2 | **competition_main.launch** | `launch/competition_main.launch` | `.launch` → Python `launch` 文件，参数用 `launch_arguments`/yaml | 一条命令拉起全部节点 |

### 阶段 E：辅助脚本（按需，可能选做）

| # | 脚本 | 说明 |
|---|------|------|
| E1 | `geopose_node.py` (210行) | 目标地理坐标辅助节点 |
| E2 | `single_bucket_aim_test.py` (211行) | 单桶瞄准测试（可先翻译，用于 D 前验证瞄准） |
| E3 | `servo_test.py` (325行) | 舵机测试（验证 `MAV_CMD_DO_SET_SERVO`） |
| E4 | `auto_drop_node.py` (124行) / `main.py` (809行) | 旧逻辑，已并入 competition_main，**翻译优先级最低**，仅保留参考 |
| E5 | `video_recorder_node.py` / `flight_data_video_recorder_node.py` | 记录节点，真机必要，仿真次要 |
| E6 | `LSY/autonomous_drop_mission/`（`autonomous_two_drop_mission.py` + `mission_geometry.py`） | ✅ **纳入翻译**（用户确认）。半自动投放脚本，独立于主版本，单独一节做 ROS2 迁移 + 验证 |

---

## 2. "验证通过"的定义（每个脚本）

- 无 ROS2 报错日志（error/warn 除外但须解释），节点 lifecycle 正常。
- 依赖的话题/服务能 over 并拿到合理数据。
- 涉及飞机动作的：在 GZ 里观察到对应飞行动作（起飞/移动/投放/返航），不触发 FAILSAFE/ABORT。
- 整链达标线（比赛要求）：**全自动完成 起飞→投放区→至少1次有效投放→侦察区→至少1个正确识别→回起降点着陆**。

## 3. 风险点

- GZ 仿真无真 D435i：相机图像用 GZ 相机插件 + ros_gz_bridge 提供（需要确认 iris_with_standoffs 带相机链接，必要时给模型加 camera 插件）。
- 仿真 GPS/RTK 与真机不同：WGS84 变换代码照迁移，验证用仿真坐标系。
- `rospy` 的隐式队列/rate 行为与 rclpy 不同（spin、rate 循环要显式处理）。
- detector_node 1725 行，翻译工作量大，可分拆（配置→推理→变换→发布）分 2~3 次提交验证。

## 4. 交付物

- **`~/ROS2_CUADC2026/`**（独立新建文件夹）：ROS2 版 `cuadc_vision`、`cuadc_control` 包 + launch + 半自动投放包。
- **原 `/home/lab/GDPI_CUADC_2026/代码/` 只读，不修改、不覆盖任何原文件**；原 ROS1 代码保持原样。
- 每脚本的翻译要点 + 验证结果记录（追加到本文件或 `~/ROS2_CUADC2026/验证记录.md`）。

## 5. 已确认决策（2026-09-21）

1. 翻译顺序 A→B→C→D→E（依赖驱动，A2 公共库最先）✅
2. `LSY/autonomous_drop_mission/` 半自动投放脚本 纳入翻译 ✅
3. 翻译产物放独立新建文件夹 `~/ROS2_CUADC2026/`，不修改任何原文件 ✅