# ArduPilot Copter Guided 模式控制指令详解（中英对照 + HIT 代码实例）

记录日期：2026-07-11
来源：[ArduPilot 官方开发文档 - Copter Commands in Guided Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
配套代码：`03_竞赛资料/GDPI_CUADC_2026/代码/HIT2025code`

> **本文用途**：将 AP 官方文档英文原文与中文润色逐段对照，并对照哈工大 2025 备赛代码（HIT2025code）说明每条控制指令在实战中如何使用。本文与 HIT 代码将一并同步到 NUC，供机载 codex 修改代码并在仿真中验证，因此每个指令都给出了**可直接落地到 MAVROS 代码**的写法。

## 目录

- [[#1. 指令总览|1. 指令总览]]
- [[#2. 关键前提 MAVLink 是 NED 而 MAVROS 是 ENU|2. 关键前提：MAVLink 是 NED 而 MAVROS 是 ENU]]
- [[#3. 指令 SET_POSITION_TARGET_LOCAL_NED|3. 指令 SET_POSITION_TARGET_LOCAL_NED]]
- [[#4. 指令 SET_POSITION_TARGET_GLOBAL_INT|4. 指令 SET_POSITION_TARGET_GLOBAL_INT]]
- [[#5. 指令 SET_ATTITUDE_TARGET|5. 指令 SET_ATTITUDE_TARGET]]
- [[#6. type_mask 掩码速查表|6. type_mask 掩码速查表]]
- [[#7. 坐标系 MAV_FRAME 理解|7. 坐标系 MAV_FRAME 理解]]
- [[#8. mavros 能否同时取得 ENU 与 WGS84 定位|8. mavros 能否同时取得 ENU 与 WGS84 定位]]
- [[#9. 备赛要点与给 NUC codex 的提示|9. 备赛要点与给 NUC codex 的提示]]

---

## 1. 指令总览

> **英文原文**
> This article lists the MAVLink commands that affect the movement of a Copter. Normally these commands are sent by a ground station or Companion Computers often running DroneKit. The Copter code which processes these commands can be found in `GCS_Mavlink.cpp`.

**中文润色**：本文列出所有能影响多旋翼运动的 MAVLink 指令。这些指令通常由地面站（GCS）或运行 DroneKit 的机载伴飞计算机（我们的 NUC）发送。飞控端处理这些指令的源码位于 `GCS_Mavlink.cpp`。

### 1.1 运动控制消息

| 消息                               | 作用                                   | 适用模式                | MAVROS 对应话题                                                    |
| -------------------------------- | ------------------------------------ | ------------------- | -------------------------------------------------------------- |
| `SET_POSITION_TARGET_LOCAL_NED`  | 以 EKF 原点为参考的本地坐标设定目标位置/速度/加速度/航向/转向率 | Guided              | `/mavros/setpoint_raw/local`、`/mavros/setpoint_position/local` |
| `SET_POSITION_TARGET_GLOBAL_INT` | 以 WGS84 经纬度设定目标位置/速度/航向/转向率          | Guided              | `/mavros/setpoint_raw/global`                                  |
| `SET_ATTITUDE_TARGET`            | 设定目标姿态与爬升率/油门                        | Guided、Guided_NoGPS | `/mavros/setpoint_raw/attitude`                                |

### 1.2 MAV_CMD 命令

需封装在 **`COMMAND_LONG`** 中发送：

- `MAV_CMD_CONDITION_YAW` —— 控制机头朝向（MAVROS 无直接话题，可用 `/mavros/cmd/command` 服务）
- `MAV_CMD_DO_CHANGE_SPEED` —— 改变飞行速度
- `MAV_CMD_DO_FLIGHTTERMINATION` —— 立即解锁电机（**飞机直接坠落！** 慎用）
- `MAV_CMD_DO_PARACHUTE` —— 控制降落伞
- `MAV_CMD_DO_SET_ROI` —— 设定兴趣区域（机头/云台指向某点）
- `MAV_CMD_NAV_TAKEOFF` —— 起飞（HIT 代码用 `/mavros/cmd/takeoff` 服务）
- `MAV_CMD_NAV_LOITER_UNLIM` —— 切换到 Loiter 模式
- `MAV_CMD_NAV_RETURN_TO_LAUNCH` —— 切换到 RTL 返航模式
- `MAV_CMD_NAV_LAND` —— 切换到 Land 降落模式（HIT 代码用 `/mavros/cmd/land` 服务）
- `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` —— 重启/关闭飞控

需封装在 **`COMMAND_INT`** 中发送：

- `MAV_CMD_DO_REPOSITION` —— 重新定位（飞往指定点）

---

## 2. 关键前提 MAVLink 是 NED 而 MAVROS 是 ENU

> ⚠️ **这是理解全部代码的第一前提，必须先看。**

AP 官方文档描述的 `SET_POSITION_TARGET_LOCAL_NED` 使用 **NED（North-East-Down，北-东-地）** 坐标系：X 向北、Y 向东、Z 向下。

但 HIT 代码是通过 **MAVROS** 发指令的，而 MAVROS 遵循 ROS 的 **ENU（East-North-Up，东-北-天）** 约定：X 向东、Y 向北、Z 向上。**MAVROS 内部会自动完成 ENU↔NED 的转换**再发给飞控。

HIT 代码 `code1.1.cpp` 开头的注释正好踩过这个坑：

```cpp
// code1.1.cpp 第 9 行注释：
// go_to_pub 坐标系为东北天，即东为 x，北为 y，右手系，
// 而 apm 疑似未声明的情况下需要保持朝向为 x 正向
```

| | AP 文档 / 原始 MAVLink | HIT 代码 / MAVROS |
| --- | --- | --- |
| 坐标系 | NED（北 X、东 Y、下 Z） | ENU（东 X、北 Y、上 Z） |
| 高度正方向 | 向下为正（z=-10 表示上方 10m） | 向上为正（z=+1.5 表示上方 1.5m） |
| 谁做转换 | —— | MAVROS 自动转换 |

**给 NUC codex 的提醒**：阅读 AP 文档时看到的 NED 数值（如 `takeoff 10` 后 `z=-10` 表示上方），换算到 MAVROS 的 `PoseStamped`/`setpoint_raw/local` 时要变成 ENU（`z=+10`）。两者不要混用。

---

## 3. 指令 SET_POSITION_TARGET_LOCAL_NED

> **英文原文**
> Set the vehicle's target position (as an offset in NED from the EKF origin), velocity, acceleration, heading or turn rate.

**中文润色**：以相对 **EKF 原点** 的 NED 偏移量，设定飞行器的目标位置、速度、加速度、航向或转向率。EKF 原点是飞行器首次获得良好定位估计时所在的位置。

### 3.1 字段说明（中英对照）

| 字段                 | 英文原义                                               | 中文说明                            |         |
| ------------------ | -------------------------------------------------- | ------------------------------- | ------- |
| `time_boot_ms`     | Sender's system time in ms since boot              | 发送方开机以来的时间（毫秒）                  |         |
| `target_system`    | System ID of vehicle                               | 目标飞行器系统 ID                      |         |
| `target_component` | Component ID or just 0                             | 飞控组件 ID，一般填 0                   |         |
| `coordinate_frame` | Valid options listed below                         | 坐标系，见下表                         |         |
| `type_mask`        | Bitmask of fields to ignore                        | 忽略字段的位掩码，见[[#6. type_mask 掩码速查表 | 第 6 节]] |
| `x / y / z`        | Position (m). +X fwd/North, +Y right/East, +Z down | 位置（米）。原始 NED：+X 北、+Y 东、+Z 下     |         |
| `vx / vy / vz`     | Velocity (m/s), same sign convention               | 速度（m/s），符号约定同上                  |         |
| `afx / afy / afz`  | Acceleration (m/s²)                                | 加速度（m/s²）                       |         |
| `yaw`              | yaw/heading in radians (0=North)                   | 航向角（弧度），0=正北                    |         |
| `yaw_rate`         | yaw rate in rad/s                                  | 转向率（rad/s）                      |         |

### 3.2 coordinate_frame 取值

| 坐标系（值） | 英文原义 | 中文说明 |
| --- | --- | --- |
| `MAV_FRAME_LOCAL_NED` (1) | Positions relative to EKF Origin | 位置相对 EKF 原点，NED 帧 |
| `MAV_FRAME_LOCAL_OFFSET_NED` (7) | Positions relative to current position | 位置相对当前位置 |
| `MAV_FRAME_BODY_NED` (8) | Vel/Accel relative to vehicle heading | 位置相对原点，但速度/加速度相对机头 |
| `MAV_FRAME_BODY_OFFSET_NED` (9) | Positions & vel relative to current pos & heading | 位置与速度都相对当前位置和机头朝向 |

### 3.3 AP 官方 MAVProxy/SITL 测试示例

> 运行前先执行：`module load message` → `GUIDED` → `arm throttle` → `takeoff 10`

| 命令                                                                              | 说明                     |
| ------------------------------------------------------------------------------- | ---------------------- |
| `message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 3576 100 0 -10 0 0 0 0 0 0 0 0`  | 飞到 EKF 原点北 100m、上方 10m |
| `message SET_POSITION_TARGET_LOCAL_NED 0 0 0 7 3576 10 0 0 0 0 0 0 0 0 0 0`     | 从当前位置向北飞 10m           |
| `message SET_POSITION_TARGET_LOCAL_NED 0 0 0 9 3576 10 0 0 0 0 0 0 0 0 0 0`     | 从当前位置向机头前方飞 10m        |
| `message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 3527 0 0 0 1 0 0 0 0 0 0 0`      | 以 1m/s 向北飞（仅速度）        |
| `message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 2503 0 0 0 0 0 0 0 0 0 0.7854 0` | 转向东北（航向 45° + 速度为 0）   |
| `message SET_POSITION_TARGET_LOCAL_NED 0 0 0 1 1479 0 0 0 0 0 0 0 0 0 0 0.174`  | 以 10°/s 顺时针旋转          |

> **注意**：发送速度或加速度指令时，需**每秒重发一次**；若 3 秒内未收到新指令，飞行器会自动停止。

### 3.4 HIT 代码如何使用（本地走点）

HIT 主力走点方式就是这条指令。代码没有直接拼 `SET_POSITION_TARGET_LOCAL_NED`，而是发布 `geometry_msgs::PoseStamped` 到 `/mavros/setpoint_position/local`，由 MAVROS 转成该 MAVLink 消息（仅位置，等价 type_mask=3576）。

**发布目标点**（`code1.1.cpp` 第 62–79 行 `go_to_pub`）：

```cpp
// 发布器（第 103 行）
expect_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
    "/mavros/setpoint_position/local", 10);

// go_to_pub：把 (x,y,z) 作为 ENU 目标点发布
void go_to_pub(double x, double y, double z) {
    expect_pos.pose.position.x = x;   // 东
    expect_pos.pose.position.y = y;   // 北
    expect_pos.pose.position.z = z;   // 天（上为正，与 NED 相反）
    // 朝向：始终对齐起飞时机头，避免飞控自旋
    expect_q_eigen = start_q_eigen * expect_q_eigen;
    expect_pos.pose.orientation = ...;
    expect_pos_pub.publish(expect_pos);
}
```

**坐标变换：让走点按“起飞时机头方向”而非固定东北天**（`code1.1.cpp` 第 249–253 行）：

```cpp
manual_pos_eigen = Eigen::Vector3d(x[i], y[i], takeoff_height);
expect_pos_eigen = start_q_eigen * manual_pos_eigen;      // 用起飞姿态四元数旋转
expect_pos.pose.position.x = expect_pos_eigen(0) + start_pos.pose.position.x;
expect_pos.pose.position.y = expect_pos_eigen(1) + start_pos.pose.position.y;
expect_pos.pose.position.z = takeoff_height + start_pos.pose.position.z;
```

- **对应 AP 指令**：`SET_POSITION_TARGET_LOCAL_NED` + `MAV_FRAME_LOCAL_NED` + 仅位置掩码。
- **HIT 的额外处理**：用 Eigen 四元数把机体期望点旋转到起飞航向，等效于用 `BODY_OFFSET_NED (9)` 的“相对机头”语义，但在应用层自己算，好处是位置反馈仍走全局 EKF。
- **走点切换判据**：`code1.cpp` 用固定计时（`ros::Duration(6.0).sleep()`）；`code1.3.cpp` 第 312 行改为**距离判断**（期望位置与当前位置之差小于阈值就切下一个点），更适合大场地——但需配合超时强制切换。

**给 NUC codex 的落地建议**：若要显式控制而不依赖 MAVROS 的 PoseStamped 转换，可改用 `/mavros/setpoint_raw/local`（`mavros_msgs::PositionTarget`），直接设 `coordinate_frame` 与 `type_mask`，语义与 AP 文档一一对应，更可控。

---

## 4. 指令 SET_POSITION_TARGET_GLOBAL_INT

> **英文原文**
> Set the vehicle's target position (in WGS84 coordinates), velocity, heading or turn rate. This is similar to `SET_POSITION_TARGET_LOCAL_NED` except positions are provided as latitude and longitude values and altitudes can be above sea-level, relative to home or relative to terrain.

**中文润色**：以 WGS84 经纬度设定目标位置、速度、航向或转向率。与本地版类似，区别是位置用**经纬度**表示，高度可选海拔、相对起飞点或相对地形。

### 4.1 字段说明（中英对照）

| 字段 | 英文原义 | 中文说明 |
| --- | --- | --- |
| `coordinate_frame` | see below | 高度参考帧，见下表 |
| `type_mask` | Bitmask of fields to ignore | 忽略字段位掩码 |
| `lat_int` | Latitude × 1e7 | 纬度 × 1e7（整数） |
| `lon_int` | Longitude × 1e7 | 经度 × 1e7（整数） |
| `alt` | Alt above sea level/home/terrain | 高度（米），含义取决于 frame |
| `vx / vy / vz` | Velocity m/s (+N, +E, +Down) | 速度：+北、+东、+下 |
| `afx / afy / afz` | Acceleration m/s² | 加速度 |
| `yaw / yaw_rate` | heading rad / rate rad/s | 航向 / 转向率 |

### 4.2 coordinate_frame 取值

| 坐标系（值） | 高度含义 |
| --- | --- |
| `MAV_FRAME_GLOBAL` (0) / `_GLOBAL_INT` (5) | 海拔高度（绝对） |
| `MAV_FRAME_GLOBAL_RELATIVE_ALT` (3) / `_RELATIVE_ALT_INT` (6) | **相对起飞点**高度 ← HIT 用这个 |
| `MAV_FRAME_GLOBAL_TERRAIN_ALT` (10) / `_TERRAIN_ALT_INT` (11) | 相对地形高度 |

### 4.3 AP 官方 MAVProxy/SITL 测试示例

| 命令 | 说明 |
| --- | --- |
| `message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 6 3576 -353621474 1491651746 10 0 0 0 0 0 0 0 0` | 飞到 (-35.36,149.16)，起飞点上方 10m |
| `message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 5 3576 -353621474 1491651746 600 0 0 0 0 0 0 0 0` | 飞到 (-35.36,149.16)，海拔 600m |
| `message SET_POSITION_TARGET_GLOBAL_INT 0 0 0 11 3576 -353621474 1491651746 10 0 0 0 0 0 0 0 0` | 飞到 (-35.36,149.16)，地形上方 10m |

> 注意：`lat_int`/`lon_int` 是经纬度 **×1e7 的整数**（-35.3621474° → `-353621474`）。速度/加速度指令同样需每秒重发。

### 4.4 HIT 代码如何使用（经纬度绝对定位）

HIT 最终版 `code9.cpp`（及 `code8.cpp`、`ceshi.cpp`）用这条指令做**远距离发点**，避免罗盘偏差。注意：MAVROS 的 `GlobalPositionTarget` **直接用 double 经纬度**，无需 ×1e7（×1e7 是底层 MAVLink 的事，MAVROS 帮你处理）。

**发布器与消息构造**（`code8.cpp` 第 84–102、132 行）：

```cpp
expect_global_pos_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>(
    "/mavros/setpoint_raw/global", 10);

void go_to_global_pub(double lat, double lon, double alt) {
    expect_global_pos.header.stamp = ros::Time::now();
    // 相对起飞点高度 = MAV_FRAME_GLOBAL_RELATIVE_ALT
    expect_global_pos.coordinate_frame =
        mavros_msgs::GlobalPositionTarget::FRAME_GLOBAL_REL_ALT;
    // 只用位置，忽略速度/加速度/航向 → 等价 type_mask 仅位置
    expect_global_pos.type_mask =
        mavros_msgs::GlobalPositionTarget::IGNORE_VX | IGNORE_VY | IGNORE_VZ |
        IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW | IGNORE_YAW_RATE;
    expect_global_pos.latitude  = lat;   // 直接填 double 经纬度
    expect_global_pos.longitude = lon;
    expect_global_pos.altitude  = alt;   // 相对起飞点的高度（米）
    expect_global_pos_pub.publish(expect_global_pos);
}
```

**为什么用经纬度**（`README.md` 对 code8/code9 的说明）：

> code8：飞机坐标变换后出现偏差，原因是飞机在地面记录的方向角和起飞后不一致（起飞后罗盘受干扰）。因此改用实测场地经纬度定位，跳过罗盘，只用 RTK 信息。
> code9：最终全流程版本 2（经纬度绝对位置版）。**远距离发点用经纬度防偏移**；瞄准阶段仍用本地发点方式，保证飞机方向不变，确保视觉融合不出问题。

- **对应 AP 指令**：`SET_POSITION_TARGET_GLOBAL_INT` + `FRAME_GLOBAL_RELATIVE_ALT (6)` + 仅位置掩码。
- **判据**：`code8.cpp` 用 Haversine 近似的经纬度差判断是否到点（第 66–71 行），高度用“当前海拔 − 起飞海拔”的相对高度判断（第 74–79 行）。
- **策略要点**：远距离用经纬度（抗罗盘漂移），近距离精瞄用本地 ENU（保证机头稳定利于视觉）——这是 HIT 踩坑后的核心结论，NUC 上继续沿用。

---

## 5. 指令 SET_ATTITUDE_TARGET

> **英文原文**
> Set the vehicle's target attitude and climb rate or thrust. This message is accepted in Guided or Guided_NoGPS (this is the only message accepted by Guided_NoGPS).

**中文润色**：设定目标姿态与爬升率/油门。Guided 与 Guided_NoGPS 模式均支持，且这是 **Guided_NoGPS 唯一可接受的消息**。

### 5.1 字段说明（中英对照）

| 字段 | 类型 | 英文原义 | 中文说明 |
| --- | --- | --- | --- |
| `type_mask` | int8 | Bitmask; should always be 0b00000111 | 位掩码，**应恒为 7** |
| `q` | float[4] | Attitude quaternion (w,x,y,z), zero-rotation faces North | 姿态四元数，零旋转使机头朝**正北** |
| `body_roll_rate` | float | not supported | 不支持 |
| `body_pitch_rate` | float | not supported | 不支持 |
| `body_yaw_rate` | float | not supported | 不支持 |
| `thrust` | float | GUID_OPTIONS=0: climb rate (0.5=hold); =8: thrust 0~1 | GUID_OPTIONS=0 时为爬升率（0.5=不升降）；=8 时为 0~1 油门 |

> **关键点**：三个角速率字段都不支持，只能用四元数 `q` 控姿态；`thrust` 的含义由参数 `GUID_OPTIONS` 决定。

### 5.2 AP 官方测试示例

| 命令 | 说明 |
| --- | --- |
| `attitude 1 0 0 0 0.5` | 保持水平、零爬升率（或 50% 油门） |
| `attitude 1 0 0 0 1.0` | 以 WPNAV_SPEED_UP 爬升（或 100% 油门） |
| `attitude 0.9961947 0.0871557 0 0 0.5` | 滚转 10° + 零爬升率 |

### 5.3 HIT 代码是否用到

HIT2025code **未使用** `SET_ATTITUDE_TARGET`——全流程都是位置控制（本地 ENU + 经纬度），姿态由飞控内环自行稳定。此指令仅在需要姿态直控或无 GPS（Guided_NoGPS）场景才用，对我们侦察救援任务优先级较低，此处仅作原理备查。若 NUC 上要做特技/贴地飞行等姿态级控制，可发布 `mavros_msgs::AttitudeTarget` 到 `/mavros/setpoint_raw/attitude`。

---

## 6. type_mask 掩码速查表

`type_mask` 为位掩码，**置 1 表示忽略该字段**。位定义（`POSITION_TARGET_TYPEMASK`）：

> bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, bit7:AccX, bit8:AccY, bit9:AccZ, bit11:yaw, bit12:yaw_rate

**规则**：提供 Pos/Vel/Accel 时对应 3 个轴必须同时给；至少要给 Pos/Vel/Accel 之一（不能只给 Yaw 或 YawRate）。

| 用途 | 二进制 | 十六进制 | LOCAL_NED 十进制 | GLOBAL_INT 十进制 |
| --- | --- | --- | --- | --- |
| 仅位置 | `0b110111111000` | `0x0DF8` | 3576 | 3576 |
| 仅速度 | `0b110111000111` | `0x0DC7` | 3527 | 3527 |
| 仅加速度 | — | — | 3135 (`0x0C3F`) | 3128 (`0x0C38`) |
| 位置+速度 | `0b110111000000` | `0x0DC0` | 3520 | 3520 |
| 位置+速度+加速度 | `0b110000000000` | `0x0C00` | 3072 | 3072 |
| 仅航向 Yaw | `0b100111111111` | `0x09FF` | 2559 | 2559 |
| 仅转向率 | `0b010111111111` | `0x05FF` | 1535 | 1535 |

> ⚠️ **加速度掩码在两条消息中不一致**：LOCAL_NED=3135（`0x0C3F`），GLOBAL_INT=3128（`0x0C38`）。这是 AP 文档里的坑，NUC codex 拼掩码时注意。
>
> 在 MAVROS 里通常不用手拼数字，而是用 `IGNORE_VX | IGNORE_VY | ...` 这样的枚举按位或（见 HIT code8 的写法），更不易错。

---

## 7. 坐标系 MAV_FRAME 理解

官方记忆口诀（Tip）：

- **`_OFFSET_`** = 相对飞行器**当前位置**
- **`_LOCAL_`** = 相对**起飞点/原点位置**
- **`_BODY_`** = 速度分量**相对机头朝向**，而非 NED 帧

| 坐标系 | 位置参考 | 速度参考 |
| --- | --- | --- |
| `LOCAL_NED` (1) | EKF 原点 | NED |
| `LOCAL_OFFSET_NED` (7) | 当前位置 | NED |
| `BODY_NED` (8) | EKF 原点 | 机头朝向 |
| `BODY_OFFSET_NED` (9) | 当前位置 + 当前航向 | 机头朝向 |

---

## 8. mavros 能否同时取得 ENU 与 WGS84 定位

**结论：可以，而且 HIT 代码就是这么干的。** MAVROS 的 `local_position` 与 `global_position` 是两个独立插件，同时向外发布，互不影响。

### 8.1 两路定位话题

| 话题 | 消息类型 | 内容 | 坐标系 |
| --- | --- | --- | --- |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | EKF 融合后的本地位置+姿态 | **ENU**（东-北-天），相对 EKF 原点/home |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | 本地速度 | ENU |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | **GPS 原始 WGS84 经纬高** | 纬度/经度（度）+ 椭球高 |
| `/mavros/global_position/local` | `nav_msgs/Odometry` | 由 GPS 换算的 UTM/本地位姿 | ENU |
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | 相对起飞点高度 | —— |

> 说明：这里的 WGS84 是 EKF 输出的全局位置（GPS/RTK 已参与融合），字段布局与 GPS 模块原生 WGS84 一致。若要 GPS 模块最原始、未融合的数据，可订阅 `/mavros/gpsstatus/gps1/raw`（`mavros_msgs/GPSRAW`）。

### 8.2 HIT 代码的实证

`ceshi.cpp` 在 `main` 里**同时订阅了两路**：

```cpp
// ceshi.cpp 第 479–480 行：本地 ENU 与 全局 WGS84 同时订阅
ros::Subscriber local_pos_sub  = nh.subscribe<geometry_msgs::PoseStamped>(
    "/mavros/local_position/pose", 10, local_pos_callback);      // ENU 本地位姿
ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>(
    "/mavros/global_position/global", 10, global_pos_callback);  // WGS84 经纬高
```

- `local_pos_callback` 把 ENU 位姿存入 `local_pos`（用于本地精瞄、坐标变换）；
- `global_pos_callback` 把 WGS84 存入 `global_pos`（用于经纬度发点与到点判断）。

两个回调各自独立更新，代码里可随时同时读取当前 ENU 坐标和当前经纬高——完全不冲突。

### 8.3 使用建议

- **精瞄/视觉融合**：用 ENU `local_position/pose`（机头稳定、局部精度高、与相机坐标好对齐）。
- **远距离/抗罗盘漂移发点**：用 WGS84 `global_position/global` + `setpoint_raw/global`。
- 这与 HIT 的最终策略（`code9.cpp`：远距离经纬度、近距离本地）完全一致。

---

## 9. 备赛要点与给 NUC codex 的提示

面向 CUADC 2026 侦察救援，最常用的是位置控制两条指令，重点：

1. **本地点到点（精瞄/短距）** → `SET_POSITION_TARGET_LOCAL_NED`，仅位置掩码 3576。MAVROS 侧发 `PoseStamped` 到 `/mavros/setpoint_position/local`，或发 `PositionTarget` 到 `/mavros/setpoint_raw/local`。**牢记 ENU：z 向上为正**。
2. **经纬度发点（远距/抗漂移）** → `SET_POSITION_TARGET_GLOBAL_INT`，`FRAME_GLOBAL_RELATIVE_ALT (6)`。MAVROS 侧发 `GlobalPositionTarget` 到 `/mavros/setpoint_raw/global`，经纬度直接填 double，无需 ×1e7。
3. **速度控制** → 仅速度掩码 3527，**必须每秒重发**（≥5~10 Hz），否则 3 秒后自动悬停。HIT 的发布循环用 `ros::Rate(10.0)` 正好满足。
4. **走点切换判据** → 优先用“期望−当前”的距离判断（`code1.3`、`code8`），并加超时强制切换，别只靠计时。
5. **坐标变换** → 若希望走点相对起飞航向而非固定东北天，用起飞时四元数 `start_q_eigen` 旋转目标点（`code1.1` 第 249 行）；或远距离直接用经纬度绕开罗盘问题（`code8`/`code9`）。
6. **双定位并用** → ENU（`local_position/pose`）+ WGS84（`global_position/global`）可同时订阅，按阶段取用。
7. **安全** → `MAV_CMD_DO_FLIGHTTERMINATION` 会立即锁桨坠机，测试脚本务必避免误触。

### 关联笔记

- 环境与操作手册见 [[gazebo软件在环仿真知识和操作步骤|ArduPilot Copter 4.5 SITL 与 Gazebo 环境记录]]
- 代码逐份说明见 `03_竞赛资料/GDPI_CUADC_2026/代码/HIT2025code/README.md`

---

> 参考：[Copter Commands in Guided Mode - ArduPilot Dev Docs](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)
