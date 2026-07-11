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
│   └── one_key_takeoff.py                 # 一键起飞：GUIDED -> ARM -> TAKEOFF -> LOITER
├── launch/
│   └── one_key_takeoff.launch             # 一键起飞 launch
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

对应的 MAVLink 语义：

- `set_mode("GUIDED")` / `set_mode("LOITER")` -> `SET_MODE`
- `arming(True)` -> `MAV_CMD_COMPONENT_ARM_DISARM`
- `takeoff(altitude=...)` -> `MAV_CMD_NAV_TAKEOFF`

> 这和 ArduPilot Guided 模式官方控制思路一致，但当前脚本**还没有**使用 `SET_POSITION_TARGET_GLOBAL_INT` 或 `SET_POSITION_TARGET_LOCAL_NED`。
>
> 这意味着当前包用于“起飞与悬停验证”，不是“航点移动 / setpoint 控制验证”。

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
/dev/ttyACM0:921600
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
