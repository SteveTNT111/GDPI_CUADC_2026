# 自主搜寻与双抛投任务（独立新目录）

本目录没有修改项目原有文件。任务坐标采用已经实测确认的：`+X 向右、+Y 向前、+Z 向上`。

第一次使用请先阅读：[快速启动指南](快速启动指南.md)。

## 当前路径与文件结构

当前目录的绝对路径是：

```text
/home/lab/catkin_ws/src/autonomous_drop_mission
```

在 ROS 包 `cuadc_vision` 中，本目录按下面的相对路径被引用：

```text
$(find cuadc_vision)/autonomous_drop_mission
```

当前文件结构如下：

```text
autonomous_drop_mission/
├── README.md
├── 快速启动指南.md
├── MEASUREMENTS.md
├── config/
│   └── mission.yaml
├── launch/
│   └── autonomous_two_drop_mission.launch
├── scripts/
│   ├── autonomous_two_drop_mission.py
│   ├── mission_geometry.py
│   └── __pycache__/                  # Python 自动生成的字节码缓存
└── tests/
    ├── test_mission_geometry.py
    └── __pycache__/                  # Python 自动生成的字节码缓存
```

`__pycache__` 中的 `.pyc` 文件由 Python 运行脚本或测试时自动生成，不是任务源码，不需要手动修改。

## 文件功能说明

| 路径 | 类型 | 功能 |
|---|---|---|
| `README.md` | 总说明文档 | 说明任务流程、目录结构、各文件职责、安全限制、话题和测试方法。 |
| `快速启动指南.md` | 操作手册 | 面向现场操作人员，给出环境加载、安全预览、正式启动、状态查看和人工接管步骤。 |
| `MEASUREMENTS.md` | 标定清单 | 记录实飞前必须测量或确认的数据，包括相机安装偏移、舵机通道/PWM、深度误差和位置波动。 |
| `config/mission.yaml` | ROS 参数文件 | 保存任务高度、搜索距离/速度、对准阈值、目标跟踪、舵机、超时和安全开关等默认参数。默认禁止实飞和真实投放。 |
| `launch/autonomous_two_drop_mission.launch` | ROS 启动文件 | 可选启动 MAVROS、相机节点、检测节点和静态 TF，加载 `mission.yaml`，最后启动任务主节点；命令行参数可覆盖部分 YAML 参数。 |
| `scripts/autonomous_two_drop_mission.py` | ROS 任务主程序 | 完成就绪检查、人工授权、GUIDED/解锁/起飞、前向搜索、视觉跟踪与下降对准、A/B 舵机投放、爬升、RTL/LAND 降级处理以及飞行安全监控。 |
| `scripts/mission_geometry.py` | 纯 Python 几何工具 | 提供像素到飞机右/前偏移换算、相机安装偏移补偿、三维距离、二维限幅、搜索参考轨迹、四元数偏航和角度差等函数；不依赖 ROS。 |
| `tests/test_mission_geometry.py` | 单元测试脚本 | 测试光心零偏移、画面右侧对应 `+X`、画面上方对应 `+Y`、相机偏移补偿、投放器对准、矢量限幅、三维距离、搜索轨迹和偏航角跨界计算。 |

## 三个 Python 脚本的具体职责

### `scripts/autonomous_two_drop_mission.py`

这是整个任务的 ROS 节点入口，节点名为 `autonomous_two_drop_mission`，主要包含以下功能：

- 读取 `mission.yaml` 和 launch 传入的私有参数，并检查高度、速度、投放顺序和安全开关是否合法。
- 等待飞控连接、本地位置、相机内参和 YOLO 检测数据全部就绪。
- 记录起点位置和起飞时偏航，要求操作者输入授权口令后才允许切换 `GUIDED`、解锁和起飞。
- 使用 `VisionTargetTracker` 对多帧检测结果做近邻匹配、历史中值滤波、命中次数筛选和超时清理，从而锁定稳定目标。
- 按 `WAIT_READY → TAKEOFF → SEARCH_FORWARD → AIM_DESCEND/ALIGN_HOLD → DROP → SHIFT_FOR_B → CLIMB_RTL → RTL → COMPLETE` 状态机执行任务。
- 搜索时按设定速度逐步更新 `+Y` 位置参考点；发现稳定目标后停止搜索并进入视觉下降对准。
- 对准时根据目标像素、相机内参和深度计算水平误差，同时将单次水平修正量限制在安全范围内。
- A 投放前再次检查 X/Y/高度释放门限；A 完成后前移指定距离并稳定，再执行 B 投放。
- 通过 `MAV_CMD_DO_SET_SERVO` 分别控制 A、B 舵机，也支持明确启用的空投放演练模式。
- 持续检查飞控连接、本地位置时效、解锁状态、GUIDED 模式、任务总超时和偏航偏差。
- 遥控器切出 `GUIDED` 时进入 `PILOT_OVERRIDE` 并停止发送位置目标；普通任务故障可按配置请求 RTL。
- 发布任务状态、目标三维距离、目标相对偏移和检测画面所需的弹药/投放状态。

脚本订阅：

| 话题 | 用途 |
|---|---|
| `/mavros/state` | 获取飞控连接、模式和解锁状态。 |
| `/mavros/local_position/pose` | 获取飞机本地位置和姿态。 |
| `/vision/color/camera_info` | 获取相机内参 `fx/fy/cx/cy`。 |
| `/vision/yolo/detections` | 获取目标框、置信度和深度。 |

脚本发布：

| 话题 | 用途 |
|---|---|
| `/mavros/setpoint_position/local` | 向 MAVROS 发送本地位置目标。 |
| `/autonomous_drop/status` | 发布 JSON 格式的任务状态和失败原因。 |
| `/autonomous_drop/target_distance_m` | 发布飞机到锁定目标的三维距离。 |
| `/autonomous_drop/target_offset_xyz` | 发布目标相对飞机的 X右/Y前/Z上偏移。 |
| `/vision/mission_status` | 向现有检测显示节点发布瞄准、余量和最近投放信息。 |

脚本调用的 MAVROS 服务包括 `/mavros/set_mode`、`/mavros/cmd/arming`、`/mavros/cmd/takeoff`；启用真实投放时还会调用 `/mavros/cmd/command`。

### `scripts/mission_geometry.py`

这是不依赖 ROS 的数学工具模块，由主程序和测试脚本共同使用：

- `finite()`：检查输入是否都是有限数值。
- `clamp()`：把数值限制在上下界内。
- `limit_vector_2d()`：限制二维修正向量长度，同时保持方向不变。
- `ramped_progress()`：按速度计算搜索参考点前进距离，并在终点截断。
- `pixel_to_right_forward()`：把目标像素和深度换算成飞机坐标系的右向、前向偏移。
- `target_relative_to_aircraft()`：叠加相机安装偏移，得到目标相对飞机参考点的位置。
- `alignment_error()`：计算让实体投放器对准目标所需的飞机平移量。
- `distance_3d()`：计算目标三维距离。
- `quaternion_yaw()`：从四元数提取偏航角。
- `wrapped_angle()`、`angular_difference()`：处理跨越 `-π/π` 边界的角度差。

### `tests/test_mission_geometry.py`

这是基于 Python `unittest` 的几何单元测试。它会把 `scripts/` 加入模块搜索路径，然后直接导入 `mission_geometry.py`，因此不需要启动 ROS、飞控或相机。测试重点是坐标正负号和数值换算，防止“画面方向”和“飞机运动方向”写反。

## 文件之间的调用关系

```text
autonomous_two_drop_mission.launch
├── 加载 config/mission.yaml
├── 可选启动 MAVROS、camera_node.py、detector_node.py
└── 启动 scripts/autonomous_two_drop_mission.py
    └── 导入 scripts/mission_geometry.py

tests/test_mission_geometry.py
└── 单独导入 scripts/mission_geometry.py 进行测试
```

## 完整流程

```text
等待 MAVROS、相机、检测节点
→ 人工口令授权
→ GUIDED、解锁、起飞到 2.5 m
→ 搜寻模式：保持 2.5 m，沿 +Y 以 0.20 m/s 的参考速度前飞，最多 3 m
   ├─ 发现稳定目标：立即进入瞄准模式
   └─ 前飞 3 m 仍无目标：原地升至 3.0 m → RTL → 返回起点并由飞控降落
→ 瞄准模式：下降至 1.2 m，同时持续修正 X/Y
→ 高度约 1.2 m，目标 X/Y 各在 0±0.20 m 内连续保持 2 秒
→ A 抛投器动作
→ 飞机沿 +Y 再前移 0.08 m，位置和高度稳定 2 秒
→ B 抛投器动作
→ 原地升至 3.0 m
→ RTL → 返回起点并由飞控降落
```

搜索速度使用任务文档建议的 `0.10–0.20 m/s` 范围上限 `0.20 m/s`。代码不是一次发送 3 m 远的位置点，而是按时间逐步推进位置参考点，因此参考轨迹沿 `+Y` 以 0.20 m/s 移动，并持续保持 2.5 m 高度。

## 瞄准和投放限制

- 下降过程中一直根据目标位置修正 X/Y。
- 高度未进入 `1.2±0.15 m` 时，A 不允许投放。
- X 和 Y 任一方向超过 `±0.20 m`，2 秒计时立即清零。
- A 投放后不再重新搜索目标，而是使用本地位置向前移动 0.08 m。
- B 投放前要求 0.08 m 位移目标和 1.2 m 高度稳定 2 秒。
- 当前暂定 A=CH5、B=CH6；这只是占位映射，确认接线后再修改。

## 坐标换算

现有检测节点的 `invert_camera_x` 配置与部分文档符号存在冲突。任务节点使用检测框像素、相机内参和深度直接计算：

```text
目标向右 = (u - cx) × depth / fx
目标向前 = -(v - cy) × depth / fy
```

因此画面右侧对应飞机 `+X`，画面上方对应飞机 `+Y`。任务期间保持起飞时偏航；偏航超出默认 12° 会终止视觉控制。

## 安全默认值

以下条件没有全部满足时，节点不会解锁：

- `execute_mission=true`
- `calibration_confirmed=true`
- `enable_servo_drop=true`，或明确启用空投放演练
- MAVROS、本地位置、相机内参和检测话题全部正常
- 操作者输入授权口令 `FLY`

如果遥控器将模式切出 GUIDED，节点立即停止发送位置目标，不会强行抢回控制权。

## 安全预览

不启动飞控和相机，只验证配置与节点：

```bash
roslaunch cuadc_vision autonomous_two_drop_mission.launch \
  start_mavros:=false start_vision:=false
```

运行几何测试：

```bash
python3 $(rospack find cuadc_vision)/autonomous_drop_mission/tests/test_mission_geometry.py
```

## 启动示例

真实舵机通道和 PWM 确认前，不能直接实飞。参数确认后的形式如下：

```bash
roslaunch cuadc_vision autonomous_two_drop_mission.launch \
  execute_mission:=true \
  calibration_confirmed:=true \
  enable_servo_drop:=true \
  takeoff_altitude_m:=2.5 \
  search_forward_distance_m:=3.0 \
  search_forward_speed_mps:=0.20 \
  aim_altitude_m:=1.2 \
  rtl_climb_altitude_m:=3.0 \
  b_forward_shift_m:=0.08
```

## 输出话题

| 话题 | 内容 |
|---|---|
| `/autonomous_drop/status` | 当前模式、抛投器、目标、对准误差和失败原因 |
| `/autonomous_drop/target_distance_m` | 飞机参考点到目标的三维距离 |
| `/autonomous_drop/target_offset_xyz` | 目标相对飞机的 X右/Y前/Z上偏移 |
| `/vision/mission_status` | 给现有检测画面显示 AIMING、A/B DROP 和余量 |

## 地面和低速测试顺序

1. 拆桨验证目标在画面右侧时 X 为正、画面上方时 Y 为正。
2. 用卷尺校验 D435i 深度和三维距离。
3. 单独确认 CH5、CH6 分别连接哪个抛投器，以及开关 PWM。
4. 不挂载物体，验证 2.5 m 起飞、+Y 前飞 3 m、无目标升至 3 m 和 RTL。
5. 开启空投放演练，验证下降修正、2 秒计时、A、前移 0.08 m、B、升高和 RTL。
6. 最后才启用真实舵机投放。
