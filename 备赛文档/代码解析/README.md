# 哈工大 CUADC 代码完整解析

> **解析目的：** 供 GDPI 团队（广东理工职业学院）队员阅读理解哈工大 2025 和 2026 赛季的全部代码架构、控制流程和设计思想。
>
> **解析范围：** 涵盖 2025 赛季全部 C++ 代码（code1~code9，18 个源文件）和 2026 赛季状态机 + 视觉 + 仿真代码。
>
> **原始仓库：** [2026CUADC](https://github.com/Accelerate11/2026CUADC)
>
> **技术栈对比：**
> | 项目 | 哈工大 2025 | 哈工大 2026 | GDPI 2026 |
> |------|------------|------------|-----------|
> | ROS 版本 | ROS1 (Melodic/Noetic) | ROS2 Humble | ROS1 Noetic |
> | Ubuntu | 20.04 | 22.04 | 20.04 |
> | 编程语言 | C++ | C++ | Python |
> | 飞控固件 | ArduPilot (APM) | ArduPilot | ArduPilot |
> | 视觉方案 | 传统 CV（圆形检测） | YOLO ONNX | YOLOv8 + 传统CV |
> | 仿真 | Gazebo Classic | Gazebo Harmonic | Gazebo Classic (AP SITL) |
> | 飞控硬件 | — | CUAV V5 nano / X7+ Pro | CUAV X7 Nano |

---

## 目录

- [2025 赛季代码演变全景图](#2025-赛季代码演变全景图)
- [2025 赛季逐文件解析](#2025-赛季逐文件解析)
  - [第一阶段：基础飞行 (code1~code1.3)](#第一阶段基础飞行-code1code13)
  - [第二阶段：舵机 + 视觉引入 (code2~code3)](#第二阶段舵机--视觉引入-code2code3)
  - [第三阶段：全流程搭建 (code4~code4.2.2)](#第三阶段全流程搭建-code4code422)
  - [第四阶段：北部赛区实战 (code5)](#第四阶段北部赛区实战-code5)
  - [第五阶段：赛后优化 (code6~code7)](#第五阶段赛后优化-code6code7)
  - [第六阶段：GPS 方案 (code8~code9)](#第六阶段gps-方案-code8code9)
- [2026 赛季代码解析](#2026-赛季代码解析)
  - [状态机代码](#状态机代码)
  - [视觉代码](#视觉代码)
  - [仿真环境](#仿真环境)
- [代码架构演化流程图](#代码架构演化流程图)
- [关键设计模式总结](#关键设计模式总结)
- [与 GDPI 代码的对比要点](#与-gdpi-代码的对比要点)

---

## 2025 赛季代码演变全景图

哈工大 2025 赛季代码的演变是一个**教科书级别的工程迭代过程**——从最简单的矩形飞行开始，逐步增加坐标变换、参数化、视觉引导、目标搜索、异常处理，最终形成完整的全自动任务系统。

```
code1 (矩形飞行)
  ├─→ code1.1 (Eigen 坐标变换)
  │     └─→ code1.2 (YAML 参数化)
  │           └─→ code1.3 (距离判断走点)
  │
  ├─→ code2 (舵机控制测试)
  ├─→ code3 (视觉引导引入)
  │
  └─→ code4 (全流程一代，目标位置已知)
        └─→ code4.1 (目标搜索 TAR_FIND)
              └─→ code4.2 (桶排序优先级)
                    ├─→ code4.2.1 (瞄准优化，记忆位置)
                    └─→ code4.2.2 (跳过瞄准，直接投放)
                          └─→ code5 (北部赛区，better_pos)
                                └─→ code6 (密集航点微分)
                                      └─→ code7 (LPF + 姿态修正)
                                            └─→ code8 (GPS 经纬度导航)
                                                  └─→ code9 (混合 GPS+局部)
```

---

## 2025 赛季逐文件解析

### 第一阶段：基础飞行 (code1~code1.3)

#### code1.cpp — 自动矩形飞行

**核心功能：** 在 APM 固件下通过 MAVROS 实现自动矩形飞行

**状态机：**
```
READY_TO_FLY → TAKE_OFF → GO_TO_POINT → LAND → DONE
```

**关键代码逻辑：**
- 等待飞控连接 → 切换 GUIDED 模式 → 解锁
- 起飞到 1m 高度，等待 10 秒
- 依次飞 5 个航点形成矩形：(0,0)→(1,0)→(1,1)→(0,1)→(0,0)
- 每个航点停留 10 秒后切换
- 完成后降落

**技术要点：**
- 坐标未做变换，走点按东北天 (NEU) 方向
- 使用 `ros::Duration(10).sleep()` 做时间判断——实际飞行中不可靠
- 飞控通信通过 MAVROS 的 `/mavros/setpoint_position/local` 发布 setpoint

**流程图：**
```
[MTRON连接] → [切GUIDED] → [解锁] → [起飞1m] → [等10s]
    → [飞(0,0)等10s] → [飞(1,0)等10s] → [飞(1,1)等10s]
    → [飞(0,1)等10s] → [飞(0,0)等10s] → [降落] → [完成]
```

---

#### code1.1.cpp — Eigen 坐标变换

**核心改进：** 引入 Eigen 库进行四元数旋转，使走点按起飞时机头朝向

**关键代码逻辑（`go_to_pub` 第 70 行和 249 行）：**
```cpp
// 记录起飞时的四元数
start_q_eigen = Eigen::Quaterniond(
    local_pos.pose.orientation.w,
    local_pos.pose.orientation.x,
    local_pos.pose.orientation.y,
    local_pos.pose.orientation.z
);

// 发布 setpoint 时应用旋转
Eigen::Quaterniond expect_q_eigen;
// ... 计算期望朝向 ...
// 右乘起飞四元数 = 保持相对起飞朝向
```

**技术要点：**
- 四元数旋转变换：期望坐标 = 起飞朝向 × 预设坐标
- 飞机机头朝北时，和 code1 效果一致
- 飞机机头朝东时，自动将"前方"理解为东方——保证相对方向正确
- 这个变换模式被后续所有版本沿用

---

#### code1.2.cpp — YAML 参数化

**核心改进：** 从 ROS 参数服务器/YAML 文件读取航点坐标和参数

**关键代码逻辑：**
```cpp
nh.getParam("waypoint_num", waypoint_num);
nh.getParam("position1_x", position1_x);
nh.getParam("position1_y", position1_y);
// ...
```

**技术要点：**
- 航点坐标不再硬编码在 C++ 中
- 修改参数后只需重启节点，无需重新编译
- 调试效率大幅提升
- ⚠️ 实际代码有 bug：声明了 `position map[waypoint_num]` 但 GO_TO_POINT 中仍使用硬编码

---

#### code1.3.cpp — 距离判断走点

**核心改进：** 用距离判断替代时间判断来切换航点

**关键代码逻辑：**
```cpp
bool local_pos_check_without_z(Eigen::Vector3d tar, double error) {
    double delta_x = local_pos.pose.position.x - tar.x();
    double delta_y = local_pos.pose.position.y - tar.y();
    return (delta_x * delta_x + delta_y * delta_y) < (error * error);
}
```

**技术要点：**
- 欧几里得距离判断：`√(dx²+dy²) < error`
- 飞机实际到达位置后才切换下一航点
- 必须配合"超时强制切换"机制（后续版本加入）
- 因为场地很大，计时方式无法保证飞机已经到达

---

### 第二阶段：舵机 + 视觉引入 (code2~code3)

#### code2.cpp — 舵机控制测试

**核心功能：** 测试舵机 PWM 控制，为投水机构做准备

**关键代码逻辑：**
```cpp
// MAV_CMD_DO_SET_SERVO (命令号 183)
mavros_msgs::CommandLong cmd;
cmd.request.command = 183;  // DO_SET_SERVO
cmd.request.param1 = servo_id;   // 舵机通道号
cmd.request.param2 = pwm_value;  // PWM 值
```

**技术参数：**
| 参数 | 值 | 说明 |
|------|-----|------|
| SERVO_NUM_1 | 10 | 投水通道 1 |
| SERVO_NUM_2 | 9 | 投水通道 2 |
| SERVO_NEUTRAL | 1300 | 中位 |
| SERVO_MIN | 1100 | 关闭 |
| SERVO_MAX | 1950 | 打开 |

**测试流程：** 舵机1 打开 2 秒 → 舵机2 打开 1 秒 → 降落

---

#### code3.cpp — 视觉首次引入

**核心功能：** 订阅视觉检测节点发布的引导向量，实现视觉引导投水

**关键代码逻辑：**
```cpp
// 订阅视觉话题
guidance_sub = nh.subscribe<geometry_msgs::Point>(
    "/h_detect/guidance_vector", 10, &guidance_cb, this);

// 引导向量含义：
// x = 水平引导 (左右)
// y = 垂直引导 (前后)
// z = 向量模长（置信度指标）
```

**状态机新增：**
```
READY_TO_FLY → TAKE_OFF → PUT_AIM → PUT → LAND → DONE
```

**技术要点：**
- `PUT_AIM` 状态订阅引导向量，当向量有效时切换到 `PUT`
- `PUT` 状态触发舵机投放
- 引导向量 = 相机中心指向目标圆中心的水平偏移
- **视觉那边的坐标系需要和控制组的 xy 一致——需要双方沟通对齐**

---

### 第三阶段：全流程搭建 (code4~code4.2.2)

#### code4.cpp — 全流程一代（目标位置已知）

**核心功能：** 首个完整任务流程，假设三个桶的位置已知

**状态机（11 个状态）：**
```
READY_TO_FLY → TAKE_OFF → GO_TO_PUT → GO_TO_TAR_1 → PUT_AIM_1
  → PUT_1 → GO_TO_TAR_2 → PUT_AIM_2 → PUT_2
  → GO_TO_DET → DET（侦察）→ RETURN → LAND → DONE
```

**代码架构特点：**
- 函数化设计：将前面验证的功能封装为独立函数
- `local_pos_check_without_z()` / `include_z()` / `only_z()` 距离检查
- Eigen 坐标变换
- 视觉引导订阅 + 舵机控制
- 每个状态有 `max_wait_time[]` 超时强制切换

**GO_TO_TAR_1 → PUT_AIM_1 → PUT_1 三状态模式：**
这是哈工大代码中最核心的设计模式，一直延续到最终版本：

```
GO_TO_TAR_1:  飞到粗略确定的桶坐标位置 (pos_tar_1)
    ↓ (距离 < 阈值)
PUT_AIM_1:    用视觉引导向量做精确瞄准
    ↓ (引导向量稳定 + 对准)
PUT_1:        触发舵机投放
```

---

#### code4.1.cpp — 目标搜索 (TAR_FIND)

**核心改进：** 桶的位置未知，需要飞行中搜索

**新增状态 TAR_FIND：**
```
TAR_FIND: 低空飞行搜索，发现桶时记录位置
    ↓ (3 个桶都找到 或 超时)
GO_TO_TAR_1: 飞到记录的位置投桶
```

**目标识别策略：**
- 通过圆形直径匹配来区分 3 个不同大小的桶
- `tar_d1=0.15, tar_d2=0.20, tar_d3=0.25`（桶直径，单位 m）
- 桶位置 = `local_pos + current_guidance`（飞机位置 + 引导向量偏移）

**技术要点：**
- `FIND_height` 参数控制搜索高度
- 存在不完整代码段（`wp_index = ;`，未声明的 `current_d`）

---

#### code4.2.cpp — 桶排序优先级

**核心改进：** 处理桶数量不确定（0~3 个），按直径排序投桶

**关键代码逻辑：**
```cpp
int rank[3] = {3, 2, 1};  // 按直径降序：大桶优先
bool tar_found[4];         // 每个桶是否被找到
int tar_cnt;               // 找到的桶数量
```

**搜索 + 排序算法：**
1. 在 TAR_FIND 中循环搜索航点
2. 当检测到桶时，根据直径判断是第几个桶，记录位置
3. 搜索结束条件：`tar_cnt >= 3` 或超时
4. GO_TO_TAR_1 中按 `rank[]` 顺序遍历：先投最大的桶

**新增视觉订阅：** 订阅 `/h_detect/diameter` 实时获取检测到的圆形直径

---

#### code4.2.1.cpp — 瞄准优化（记忆位置）

**核心改进：** 处理瞄准阶段"看不见圆"的情况

**问题场景：** 飞机飞到桶上方时，摄像头垂直朝下可能拍不到桶的侧面

**解决方案（记忆位置）：**
```cpp
// 看不见圆时，使用上一次识别到的位置
if (current_d == 0) {
    is_guidance_valid = false;
    // 保持上一帧的 guidance 不变
}
```

**新增稳定检测：**
```cpp
bool is_stable_arrival(guidance, threshold=0.05, duration=0.5) {
    // 引导向量模长在 0.5 秒内持续 < 0.05 → 认为对准稳定
}
```

**技术评估（来自开发者）：**
- ⚠️ 实际效果一般，"会反复横跳"
- 后续采用了更好的"看不见圆"处理方案（升高扩大视野）

---

#### code4.2.2.cpp — 跳过瞄准版

**核心功能：** 不瞄准直接投放，进入侦察区

**背景：** 换机载电脑之前投不进去，考虑多拿侦察区时间分

**开发者评价：** "后来意识到只需要修改强制切换状态的判断时间即可，此版没啥用"

---

### 第四阶段：北部赛区实战 (code5)

#### code5.cpp — 北部赛区比赛代码

**核心功能：** 北部赛区正式比赛使用，虽然最终放弃了投靶但留下了重要改进

**关键新增功能：**

**1. better_pos 机制**
```cpp
Eigen::Vector3d better_pos[4];  // 记录每个桶引导向量模长最小的位置

// 在 PUT_AIM 中：
if (guidance_norm < best_guidance_norm) {
    best_guidance_norm = guidance_norm;
    better_pos[idx] = local_pos + guidance_global;
}
// 看不见圆时回到 better_pos
if (!is_guidance_valid) {
    go_to_pub(better_pos[current_tar]);
}
```

**2. LAND_AIM 状态**
- 使用视觉引导降落到降落点
- 同样追踪 better_pos_land

**3. 比例缩放 (ratio1=0.27)**
- 引导向量 × ratio1 = 实际移动量
- 防止过度修正

**开发者反思：**
- 图像延迟导致桶位置不对（核心投不进去的原因）
- 尝试限制飞行速度来消除延迟影响——效果不佳
- better_pos 思路好但效果一般——建议继续研究

---

### 第五阶段：赛后优化 (code6~code7)

#### code6.cpp — 密集航点微分

**核心改进：** 专攻 TAR_FIND → GO_TO_TAR → PUT_AIM 三个重要状态

**核心思路：** 飞机两点距离较大时，姿态变化大 + 速度大 → 影响目标定位

**解决方案：** 多设几个中间航点（手动微分），限制速度和姿态变化

**效果：** ✅ 确实投进了（刘博老师帮助调参）

**技术参数：**
- 到达阈值收紧到 `err_without_z2=0.07`（之前 0.15）
- 稳定检测更保守：阈值 0.08，持续 1 秒
- 18 个搜索航点

---

#### code7.cpp — 最终全流程版本 1

**核心功能：** 北部赛区后发愤图强的完整版本

**关键新增功能：**

**1. 低通滤波 (LPF)**
```cpp
// 指数移动平均，alpha=0.3
filtered_guidance = alpha * raw_guidance + (1-alpha) * filtered_guidance;
```

**2. 姿态倾斜修正 (attitude_correction)**
```cpp
// 通过飞机当前姿态，修正因倾斜导致的桶位置偏差
// 原理：飞机倾斜时，摄像头不朝正下方，计算出的桶位置有偏差
Eigen::Vector3d attitude_correction(double drone_height) {
    // 将相机视线向量通过四元数旋转到全局坐标系
    // 投影到地平面得到正确位置
}
```

**3. 摄像头离线检测**
```cpp
if (camera_offline_cnt > 100) {  // 10 秒无数据
    camera_offline_flag = true;
    // 触发安全返航
}
```

**4. 高度自适应下降**
```cpp
// 引导向量模长 < height_reduce_norm(0.40) → 下降到 ready_put_height
```

**5. 水平框架修正 (createHorizontalFrame)**
- 解决起飞后罗盘干扰导致方向角偏差的问题

**技术参数总览：**
| 参数 | 值 | 说明 |
|------|-----|------|
| alpha | 0.3 | LPF 平滑系数 |
| ratio1 | 0.27 | 引导向量缩放比 |
| height_reduce_norm | 0.40 | 下降触发阈值 |
| err_without_z2 | 0.07 | 瞄准到达精度 |
| 搜索航点数 | 30 | 密集搜索网格 |
| 帧率 | 10Hz | 控制循环频率 |

---

### 第六阶段：GPS 方案 (code8~code9)

#### code8.cpp — GPS 经纬度导航验证

**核心问题发现：** 飞机在地面记录的磁罗盘方向角和起飞后不一致（起飞后电机/电流干扰罗盘）

**解决方案：** 用实测场地经纬度来确定位置，完全跳过罗盘信息，只用 RTK GPS

**关键代码逻辑：**
```cpp
// 向 /mavros/setpoint_raw/global 发布 GlobalPositionTarget
mavros_msgs::GlobalPositionTarget target;
target.latitude = lat;
target.longitude = lon;
target.altitude = alt;
```

**验证结论：** GPS 航点确实解决了坐标系偏移问题

---

#### code9.cpp — 最终全流程版本 2（混合 GPS + 局部）

**核心功能：** 最终比赛版本，混合使用经纬度和局部坐标

**设计哲学：**
```
远距离发点（GO_TO_PUT, TAR_FIND）：使用经纬度 → 防止偏移
瞄准阶段（GO_TO_TAR, PUT_AIM）：使用局部坐标 → 保证飞机方向不变
```

**逻辑流程：**
```
1. 起飞 → 用经纬度飞到投水区上空
2. 搜索航点循环 → 用经纬度，保持航向一致
3. 飞到目标附近 → 用经纬度粗略到达
4. 瞄准对准 → 切到局部坐标系，用视觉引导精确对准
5. 投放 → 舵机指令
6. 侦察 → 经纬度航点循环
7. 返航降落
```

**为什么瞄准阶段要切回局部坐标：**
- 经纬度航点不保证飞机方向（yaw 可能变化）
- 视觉引导需要飞机航向稳定，否则 tf 变换出错
- 局部坐标 + 四元数旋转可以精确保持航向

---

## 2026 赛季代码解析

### 状态机代码

#### offb_node.cpp — ROS2 自动飞行状态机

**技术栈：** ROS2 Humble + MAVROS + ArduPilot

**七状态有限状态机：**
```
WAITING_FCU → SETTING_GUIDED → ARMING → TAKEOFF
  → FLYING_WAYPOINTS → LANDING → DONE
```

**状态详解：**

| 状态 | 功能 | 关键操作 |
|------|------|---------|
| WAITING_FCU | 等待飞控连接 | 检查 `current_state_.connected` |
| SETTING_GUIDED | 设置 GUIDED 模式 | 异步调用 `set_mode` 服务 |
| ARMING | 解锁 | 异步调用 `arming` 服务 |
| TAKEOFF | 起飞到 2m | **禁用 setpoint 发布**，使用 takeoff 服务 |
| FLYING_WAYPOINTS | 飞 5×5m 矩形 | 5 个航点，距离 0.5m 判断到达，每点停 2s |
| LANDING | 降落 | 调用 land 服务，检测高度 < 0.2m |
| DONE | 完成 | `rclcpp::shutdown()` |

**关键设计决策：**

**起飞期间禁用 setpoint 发布 🔑**
```cpp
// 起飞期间 publish_setpoint_enabled_ = false
// 原因：避免 setpoint 与 MAVROS takeoff 指令冲突
// 到达 95% 目标高度后重新启用
```

**航点到达判断：**
```cpp
// 欧几里得距离 < 0.5m 且停留 ≥ 2 秒
double dist = sqrt(pow(x-target_x, 2) + pow(y-target_y, 2));
if (dist < 0.5 && elapsed > 2.0) { next_waypoint(); }
```

**代码架构：**
```
offboard_control/              # ROS2 包
├── src/offb_node.cpp          # 核心状态机
└── launch/
    ├── mavros_offb_real.launch.py    # 真机 + 自动飞行
    ├── mavros_offb_sitl.launch.py    # 仿真 + 自动飞行
    ├── mavros_real.launch.py         # 仅 MAVROS 连接
    └── offb_only.launch.py           # 仅状态机节点
```

**与 2025 代码的关系：**
- 这是 2026 赛季重新用 ROS2 写的**基础框架**
- 状态机是 code1 的 ROS2 版本，功能更基础
- 2025 年的投水/视觉/搜索等高级功能尚未移植到 ROS2 版本
- 但 launch 文件体系更规范（SITL/真机分离）

---

### 视觉代码

#### 视觉 1.0 — YOLO ONNX 部署

**文件清单：**
| 文件 | 用途 |
|------|------|
| `dangerous_target.onnx` (12MB) | ONNX 格式的危险品检测模型 |
| `labeling.py` (11KB) | 数据标注脚本 |
| `single_demo_en.py` (11KB) | 单张图片推理演示 |
| `labels.docx` | 标注类别文档 |

**与 2025 视觉方案对比：**
- 2025：传统 CV 圆形检测（HSV 颜色空间 + 霍夫圆检测），输出引导向量
- 2026：YOLO ONNX 深度学习目标检测，可检测多种危险品标识
- 这是哈工大 2026 赛季向深度学习视觉方案的迁移尝试

---

### 仿真环境

哈工大 2026 赛季搭建了两套仿真环境：

#### cuadc_sim — 主竞赛仿真包

**技术栈：** Gazebo Harmonic + ArduPilot SITL + ROS2

**核心文件：**
| 文件 | 功能 |
|------|------|
| `cuadc_sim.launch.py` | 主启动文件 |
| `simple_mission_demo_node.cpp` | 简单任务演示节点 |
| `virtual_drop_judge_node.cpp` | 虚拟投放判定节点 |
| `generate_scene.py` | 场景随机生成器 |
| `scene.yaml` | 场景配置文件 |
| `mission_params.yaml` | 任务参数 |

**Gazebo 模型：**
- `iris_d435i` — 带 D435i 深度相机的 Iris 四旋翼（完整 3D 模型 + 传感器）
- `hazard_marker` — 10 种危险品标识（flammable, toxic, explosive, radioactive...）
- `drop_bucket_15/20/25` — 三种直径的投水桶
- `recon_bucket` — 侦察桶
- `cuadc_field` — 比赛场地

**虚拟投放判定 (`virtual_drop_judge_node.cpp`)：**
- 监听飞机位置和投放信号
- 判断投放物是否落入桶内
- 输出投放结果（命中/未命中/距离）

#### cuadc_hazard_recognition_sim — 危险品识别仿真

**技术栈：** Gazebo Harmonic + ROS2

**核心区别：**
- 专用于危险品识别任务（比赛第一阶段）
- 模型包含危险品标记 + 识别区域 + 侦察桶
- D435i 相机固定（非机载），用于地面识别

---

## 代码架构演化流程图

```
                       2025 赛季 C++ (ROS1)
                       ════════════════════
                       
  code1.cpp              基础矩形飞行
    ↓
  code1.1.cpp            + Eigen 坐标变换
    ↓
  code1.2.cpp            + YAML 参数化
    ↓
  code1.3.cpp            + 距离判断走点
    ↓
  code2.cpp              舵机 PWM 控制
    ↓
  code3.cpp              视觉引导向量引入
    ↓
  code4.cpp              全流程一代（目标已知）
    ↓
  code4.1.cpp            + TAR_FIND 目标搜索
    ↓
  code4.2.cpp            + 桶排序优先级
    ↓
  code4.2.1.cpp          + 瞄准优化（记忆位置）
    ↓
  code5.cpp              北部赛区（better_pos）
    ↓
  code6.cpp              密集航点微分
    ↓
  code7.cpp              最终版本1 (LPF+姿态修正)
    ↓
  code8.cpp              GPS 经纬度导航验证
    ↓
  code9.cpp              最终版本2 (混合GPS+局部)


                       2026 赛季 (ROS2)
                       ════════════════════
                       
  状态机/                    视觉/
  offb_node.cpp             dangerous_target.onnx
  (七状态 FSM)              (YOLO ONNX 模型)
       ↓                         ↓
  cuadc_sim/                cuadc_hazard_recognition_sim/
  (主竞赛仿真)               (危险品识别仿真)
```

---

## 关键设计模式总结

### 1. 三状态瞄准投放模式 🔑

这是哈工大代码中最重要的设计模式：

```
GO_TO_TAR → 飞到目标粗略位置（远距离接近）
    ↓
PUT_AIM   → 用视觉引导精确对准（近距离修正）
    ↓
PUT       → 触发舵机投放
```

### 2. 多级回退策略

```
正常：视觉引导对准 → 投放
↓ 看不见目标
升高高度扩大视野 → 重新搜索
↓ 仍看不见
回退到 better_pos（历史最佳位置）
↓ 超时
强制切换状态，防止卡死
```

### 3. 坐标系选择策略 (code9)

```
远距离导航 → GPS 经纬度（避免罗盘干扰）
近距离对准 → 局部 ENU 坐标（保持航向稳定）
```

### 4. 信号处理链 (code7)

```
原始引导向量 → 低通滤波(LPF) → 姿态修正 → 比例缩放 → 限幅
    raw          alpha=0.3     attitude    ratio=0.27  clamp
```

### 5. 超时强制切换机制

每个状态都有 `max_wait_time[]` 配置——防止某个环节卡住导致任务失败。

---

## 与 GDPI 代码的对比要点

> 详细的差距对比分析请参见 [[GDPI_vs_HIT_差距分析]]

### 架构层面

| 维度 | 哈工大 2025 | 哈工大 2026 | GDPI 2026 |
|------|------------|------------|-----------|
| 编程语言 | C++ | C++ | Python |
| ROS 版本 | ROS1 | ROS2 | ROS1 |
| 节点数 | ~3 个 | ~2 个 | 7 个 |
| 状态机复杂度 | 12 状态 FSM | 7 状态 FSM | 9 状态 FSM |
| 坐标变换 | Eigen 手写 | TF 树 | TF 树 + geographiclib |
| 参数管理 | YAML (ROS param) | — | YAML (params.yaml) |
| 仿真环境 | ❌ 无 | ✅ Gazebo Harmonic | ✅ Gazebo Classic |

### 视觉方案

| 维度 | 哈工大 2025 | 哈工大 2026 | GDPI 2026 |
|------|------------|------------|-----------|
| 方案 | 传统 CV 圆形检测 | YOLO ONNX | YOLOv8 + 传统CV |
| 输出 | 引导向量 (x,y,z) | 检测框 + 类别 | 3D 坐标 + 置信度 |
| 坐标系 | 二维引导向量 | — | 相机光学系 (X,Y,Z) |
| 深度信息 | 无（纯视觉） | — | D435i 深度图 |
| 坐标变换链 | 无（直接给控制组） | — | 像素→相机→机体→ENU→WGS84 |

### 控制精度

| 维度 | 哈工大 2025 | GDPI 2026 |
|------|------------|-----------|
| 对准精度 | 引导向量模 < 0.05 | XY 偏移 < 10cm |
| 稳定检测 | 0.5~1 秒持续 | 3 秒持续 |
| 投放时机 | 对准稳定 + 高度正确 | 对准稳定 3s |
| 低通滤波 | ✅ alpha=0.3 | ❌ 未实现 |
| 姿态修正 | ✅ 四元数反投影 | 依赖 TF 树 |
| better_pos | ✅ | ❌ 未实现 |

---

> **创建时间：** 2026-07-07
>
> **解析人：** Claude (AI 辅助) — 基于哈工大 2026CUADC 仓库全部代码文件的人工分析
>
> **下一步：** 详见 [[GDPI_vs_HIT_差距分析]] 了解两队代码的具体差距和建议追赶方向
