# CUADC Control — 控制验证包

> **包名：** `cuadc_control`（ROS Noetic）
> **用途：** CUADC 2026 的飞控控制验证包，专门存放 SITL 仿真和真机飞行验证脚本，不与视觉包混放。
> **维护：** CUADC Control Team
>
> ⚠️ **本文档所有命令都在 Ubuntu 20.04 + ROS Noetic 终端执行。**
>
> 这个包的目标是把“验证飞控链路是否通、自动起飞是否正常、仿真与真机流程是否一致”单独抽出来，避免所有测试代码继续堆在 `cuadc_src` / `cuadc_vision` 里。

---

## 文件结构

```text
cuadc_control/
├── scripts/
│   ├── one_key_takeoff.py                 # 一键起飞：GUIDED -> ARM -> TAKEOFF -> LOITER
│   ├── one_key_takeoff_forward_land.py    # 已解锁：起飞 -> 前飞 1m -> LAND
│   └── one_key_takeoff_wgs84_forward_rtl.py # 授权后：WGS84 前飞 10m -> RTL/LAND
├── launch/
│   ├── one_key_takeoff.launch             # 一键起飞 launch
│   ├── one_key_takeoff_forward_land.launch # 前飞与降落 launch
│   └── one_key_takeoff_wgs84_forward_rtl.launch # WGS84 航点与 RTL launch
├── CMakeLists.txt
├── package.xml
└── ctrl_README.md
```

---

## 依赖安装

### 系统依赖

```bash
sudo apt install -y \
  ros-noetic-rospy \
  ros-noetic-geometry-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-mavros-msgs \
  ros-noetic-mavros
```

### 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x ~/catkin_ws/src/cuadc_control/scripts/*.py
```

---

## 当前脚本

### 1. `one_key_takeoff.py`

**功能：**

- 自动检查并启动 `roscore`（如未运行）
- 可选自动启动 `MAVROS`
- 等待 `/mavros/state` 和 `/mavros/local_position/pose`
- 自动执行：

```text
GUIDED -> ARM -> TAKEOFF -> LOITER
```

**默认行为：**

- 默认起飞高度：`3.0 m`
- 默认悬停模式：`LOITER`
- 默认保持监控输出，不自动退出

### 2. `one_key_takeoff_forward_land.py`

该脚本要求操作者事先手动解锁，脚本不会连接或调用
`/mavros/cmd/arming`。默认流程为：

```text
确认 connected=True、armed=True
-> GUIDED
-> MAV_CMD_NAV_TAKEOFF 到 2m
-> 使用 /mavros/global_position/rel_alt 确认相对 Home 高度
-> 原地位置保持 2s
-> 根据 compass_hdg 在 NED 中计算当前航向前方 1m 航点
-> 发布 /mavros/setpoint_position/local
-> 到达航点后保持 3s
-> LAND，等待飞控自动上锁
```

运行：

```bash
roslaunch cuadc_control one_key_takeoff_forward_land.launch
```

如果 MAVROS 已经单独启动：

```bash
roslaunch cuadc_control one_key_takeoff_forward_land.launch auto_start_mavros:=false
```

### 3. `one_key_takeoff_wgs84_forward_rtl.py`

该脚本读取程序启动时的：

- `/mavros/local_position/pose`：ROS ENU 本地位置，打印前转换为 NED
- `/mavros/global_position/global`：当前 WGS84 经纬度与高程
- `/mavros/global_position/compass_hdg`：启动航向，`0°` 为北、顺时针为正
- `/mavros/global_position/rel_alt`：相对 Home 高度

脚本用 WGS84 椭球在起点处的曲率半径，将启动航向分解为 North/East
方向，计算前方 `10 m` 的目标经纬度。随后在终端显示并冻结：

```text
当前 NED 坐标
当前 WGS84 坐标
程序启动时航向角
前方 10m 的目标 WGS84 坐标
完整飞行计划
```

飞手必须在终端输入默认授权短语 `YES`。授权前脚本不会切换模式、
不会解锁，也不会发送起飞命令。脚本还会确认启动时相对 Home 高度在地面
容差内，避免在空中误启动该任务。授权后的默认流程为：

```text
GUIDED
-> 自动 ARM
-> MAV_CMD_NAV_TAKEOFF 到相对 Home 3m
-> 持续发布 /mavros/setpoint_raw/global
-> 到达启动航向前方 10m 的 WGS84 经纬度
-> 连续满足到达误差后悬停 5s
-> RTL
-> RTL 自动降落，或确认已回到起飞点后切换 LAND
-> 等待飞控自动上锁
```

运行：

```bash
roslaunch cuadc_control one_key_takeoff_wgs84_forward_rtl.launch
```

如果 MAVROS 已经单独启动：

```bash
roslaunch cuadc_control one_key_takeoff_wgs84_forward_rtl.launch \
  auto_start_mavros:=false
```

SITL 示例：

```bash
roslaunch cuadc_control one_key_takeoff_wgs84_forward_rtl.launch \
  auto_start_mavros:=false \
  forward_distance:=10.0 \
  takeoff_altitude:=3.0
```

> `roslaunch` 启动时脚本优先从 `/dev/tty` 读取授权输入，因此需要在有
> 交互终端的前台运行。无人值守启动或没有控制终端时，授权会失败，任务不会起飞。

#### RTL 会不会自动降落？

对于 ArduCopter，RTL 的标准流程是升至 RTL 高度、返回 Home、等待
`RTL_LOIT_TIME`，然后下降并自动降落、上锁。主要例外是：

- `RTL_ALT_FINAL = 0`：回到 Home 后继续下降并自动降落，这是常用默认值。
- `RTL_ALT_FINAL > 0`：回到 Home 后停在该高度，不会自动降落，需要后续
  模式指令或飞手接管。

本脚本会读取 `RTL_ALT_FINAL`。如果它大于 `0`，脚本在 GPS 和本地位置都
确认飞机回到启动点附近并稳定后切换至 `LAND`；如果参数读取失败或 RTL
本应自动落地却长时间未上锁，也只会在确认回到启动点附近后切换 `LAND`。
如果 RTL 总等待超时但飞机仍不在起飞点附近，脚本保持 `RTL`，不会在异地
强制 `LAND`。

#### WGS84 高度说明

终端打印的目标 WGS84 高程为“启动时 GPS WGS84 高程 + 起飞高度”的参考值。
实际发给飞控的 `GlobalPositionTarget` 使用
`FRAME_GLOBAL_REL_ALT`，其高度字段为相对 Home 的 `3m`，这样不会把 GPS
椭球高误当成 MAVLink 的绝对高度基准。

---

## MAVLink 控制链路

这份脚本不是向 `sim_vehicle.py` 终端发送字符串命令，而是走：

```text
ROS 脚本 -> MAVROS service -> MAVLink -> 飞控 / SITL
```

当前使用的 `MAVROS` 服务：

1. `/mavros/set_mode`
2. `/mavros/cmd/arming`
3. `/mavros/cmd/takeoff`
4. `/mavros/param/get`（WGS84/RTL 脚本读取 `RTL_ALT_FINAL`）

当前使用的 setpoint 话题：

1. `/mavros/setpoint_position/local`（本地 ENU 位置目标）
2. `/mavros/setpoint_raw/global`（WGS84 经纬度与相对 Home 高度目标）

对应的 MAVLink 语义：

- `set_mode("GUIDED")` / `set_mode("LOITER")` -> `SET_MODE`
- `arming(True)` -> `MAV_CMD_COMPONENT_ARM_DISARM`
- `takeoff(altitude=...)` -> `MAV_CMD_NAV_TAKEOFF`
- `setpoint_raw/global` -> `SET_POSITION_TARGET_GLOBAL_INT`

> WGS84/RTL 脚本已经用于全局航点 setpoint 控制验证，但仍应先在 SITL 中
> 验证参数、GPS/EKF 状态、坐标方向和 RTL 行为，再进行真机测试。

---

## 运行方式

### 1. 真机

如果要连接真实飞控：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun cuadc_control one_key_takeoff.py _takeoff_altitude:=3.0
```

如果要通过 launch：

```bash
roslaunch cuadc_control one_key_takeoff.launch takeoff_altitude:=3.0
```

默认 `fcu_url` 为：

```text
/dev/ttyACM0:115200
```

如果设备路径不同：

```bash
roslaunch cuadc_control one_key_takeoff.launch fcu_url:=/dev/ttyUSB0:921600
```

### 2. SITL 仿真

SITL 模式下，不要让脚本按真机串口去拉起 `MAVROS`。推荐流程：

1. 启动 Gazebo + ArduPilot SITL
2. 在 `MAV>` 终端确认有：

```text
output add 127.0.0.1:14550
```

3. 启动 `MAVROS`：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch mavros apm.launch fcu_url:=udp://:14550@
```

4. 启动脚本：

```bash
rosrun cuadc_control one_key_takeoff.py _auto_start_mavros:=false _takeoff_altitude:=3.0
```

如果你希望脚本自己把 `MAVROS` 拉起来：

```bash
rosrun cuadc_control one_key_takeoff.py \
  _auto_start_mavros:=true \
  _mavros_fcu_url:=udp://:14550@ \
  _takeoff_altitude:=3.0
```

---

## 当前测试结论

截至目前，一键起飞脚本在 SITL 中已经验证过以下现象：

- 第一次测试：飞机飞到约 `3m` 后掉落
- 第二次测试：飞机可正常悬停，但逼近 `3m` 的过程较慢

针对这两个现象，已经做了这些修正：

- 默认起飞高度从 `1.0m` 提升到 `3.0m`
- 只有真正达到目标高度后才允许切换到 `LOITER`
- `set_mode()` 未真正切到目标模式时直接失败
- `arm()` 未真正进入 `armed=True` 时直接失败

---

## 后续建议

当前 `cuadc_control` 适合继续放以下脚本：

- 一键降落脚本
- 一键 RTL 脚本
- Guided 定点飞行脚本
- 速度控制 / setpoint 测试脚本
- SITL 与真机链路自检脚本

建议保持原则：

- **控制验证代码** 放 `cuadc_control`
- **视觉与任务代码** 放 `cuadc_src` / `cuadc_vision`
- **比赛正式主流程** 再按稳定版本合并调用
