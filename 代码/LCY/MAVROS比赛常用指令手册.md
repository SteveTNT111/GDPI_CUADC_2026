# MAVROS 比赛常用指令手册

本文面向刚开始接触 ROS、MAVROS 和无人机机载电脑开发的同学。目标是帮助你在比赛场景中快速完成：

- 机载电脑连接 PX4 或 ArduPilot 飞控
- 启动 MAVROS
- 查看飞控状态、IMU、GPS、本地位置等关键数据
- 拉取和读取飞控参数
- 切换模式、解锁、发送目标点
- 控制抛投器舵机

比赛背景示例：

```text
机载电脑连接相机/视觉硬件
        ↓
识别地面目标或指定区域
        ↓
通过 MAVROS 获取飞机状态和定位
        ↓
机载电脑控制飞机飞到目标附近
        ↓
控制抛投器投放物体
```

---

## 1. MAVROS 是什么

MAVROS 是一个 ROS 功能包，不是单个脚本。

它的作用是把飞控的 MAVLink 数据转换成 ROS 里的 topic 和 service。

```text
PX4 / ArduPilot 飞控
        ↓ MAVLink
MAVROS
        ↓ ROS topic / service
你的视觉识别、控制、抛投程序
```

简单记忆：

```text
rostopic echo     查看实时数据
rosservice call   调用一次功能，比如读取参数、解锁、切模式
roslaunch         按配置启动一套 ROS 系统
```

---

## 2. 每次开机后的基础准备

打开机载电脑终端后，先加载 ROS 环境：

```bash
source /opt/ros/noetic/setup.bash
source ~/mavros_ws/devel/setup.bash
```

如果你已经把下面两行写进了 `~/.bashrc`，通常只需要：

```bash
source ~/.bashrc
```

检查 ROS 版本：

```bash
rosversion -d
```

正常应看到：

```text
noetic
```

检查 MAVROS 是否安装：

```bash
rospack find mavros
rospack find mavros_extras
```

正常会看到类似：

```text
/opt/ros/noetic/share/mavros
/opt/ros/noetic/share/mavros_extras
```

---

## 3. 查看飞控连接到哪个串口

插上飞控 USB 后执行：

```bash
ls -l /dev/serial/by-id/
ls /dev/ttyACM*
ls /dev/ttyUSB*
groups
```

推荐优先使用 `/dev/serial/by-id/` 下的稳定路径。

例如：

```text
usb-3D_Robotics_PX4_FMU_v5.x_0-if00 -> ../../ttyACM0
```

表示：

```text
稳定路径：/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00
临时设备：/dev/ttyACM0
```

为什么推荐稳定路径？

```text
/dev/ttyACM0 可能因为插拔顺序变化而变成 /dev/ttyACM1
/dev/serial/by-id/... 通常和具体硬件绑定，更稳定
```

如果 `groups` 里没有 `dialout`，说明当前用户可能没有串口权限：

```bash
sudo usermod -aG dialout $USER
```

执行后需要注销并重新登录。

---

## 4. 启动 MAVROS

### 4.1 PX4 固件

PX4 使用：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/你的飞控设备:57600
```

常见波特率：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/你的飞控设备:57600
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/你的飞控设备:115200
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/你的飞控设备:921600
```

如果你已经创建了自己的 launch 文件：

```bash
roslaunch mavros_control_demo px4_onboard.launch
```

### 4.2 ArduPilot / AP 固件

AP 使用：

```bash
roslaunch mavros apm.launch fcu_url:=/dev/serial/by-id/你的飞控设备:115200
```

常见波特率：

```bash
roslaunch mavros apm.launch fcu_url:=/dev/serial/by-id/你的飞控设备:115200
roslaunch mavros apm.launch fcu_url:=/dev/serial/by-id/你的飞控设备:57600
roslaunch mavros apm.launch fcu_url:=/dev/serial/by-id/你的飞控设备:921600
```

如果你已经创建了自己的 launch 文件：

```bash
roslaunch mavros_control_demo ap_onboard.launch
```

### 4.3 PX4 和 AP 的区别

它们都使用 MAVLink，所以 MAVROS 都能连。

但是 launch 文件不同：

```text
PX4       用 px4.launch
ArduPilot 用 apm.launch
```

如果用错 launch，有时也可能部分连上，但参数、模式、插件配置可能不匹配。

---

## 5. 判断 MAVROS 是否打通

启动 MAVROS 后，另开一个终端：

```bash
source ~/.bashrc
rostopic echo -n 1 /mavros/state
```

重点看：

```text
connected: True
armed: False
mode: "xxx"
```

如果 `connected: True`，说明 MAVROS 已经和飞控通信成功。

如果看到：

```text
CON: Got HEARTBEAT, connected.
```

也说明已经收到飞控心跳。

---

## 6. 查看关键实时数据

这些命令用于查看实时数据流。

### 6.1 飞控状态

```bash
rostopic echo /mavros/state
```

常见字段：

```text
connected   是否连接飞控
armed       是否解锁
guided      是否处于可引导控制状态
mode        当前飞行模式
```

### 6.2 IMU 数据

```bash
rostopic echo /mavros/imu/data
```

包含：

```text
orientation           姿态四元数
angular_velocity      角速度
linear_acceleration   加速度
```

### 6.3 电池数据

```bash
rostopic echo /mavros/battery
```

注意：如果只用 USB 供电，可能看到：

```text
voltage: 0.0
percentage: -0.01
```

这通常不代表 MAVROS 错误，而是飞控没有读到真实电池或电源模块。

### 6.4 GPS 经纬度

```bash
rostopic echo /mavros/global_position/global
```

包含：

```text
latitude
longitude
altitude
```

### 6.5 GPS 原始 fix 状态

```bash
rostopic echo /mavros/global_position/raw/fix
```

如果室内没有 GPS，可能出现：

```text
GP: No GPS fix
```

这是常见现象，不代表 MAVROS 没连上。

### 6.6 本地位置

```bash
rostopic echo /mavros/local_position/pose
```

包含：

```text
x
y
z
姿态
```

这是机载控制和视觉闭环里非常常用的话题。

### 6.7 本地速度

```bash
rostopic echo /mavros/local_position/velocity_local
```

### 6.8 查看话题频率

```bash
rostopic hz /mavros/imu/data
rostopic hz /mavros/local_position/pose
rostopic hz /mavros/global_position/global
```

---

## 7. 拉取和读取飞控参数

飞控参数属于配置项，不是实时数据流，所以用 service。

### 7.1 拉取全部参数

```bash
rosservice call /mavros/param/pull "force_pull: true"
```

成功示例：

```text
success: True
param_received: 1143
```

表示 MAVROS 从飞控拿到了 1143 个参数。

### 7.2 读取单个参数

```bash
rosservice call /mavros/param/get "param_id: '参数名'"
```

例如：

```bash
rosservice call /mavros/param/get "param_id: 'SYS_AUTOSTART'"
```

成功示例：

```text
success: True
value:
  integer: 6001
  real: 0.0
```

### 7.3 PX4 常用参数

```bash
rosservice call /mavros/param/get "param_id: 'SYS_AUTOSTART'"
rosservice call /mavros/param/get "param_id: 'MAV_TYPE'"
rosservice call /mavros/param/get "param_id: 'COM_RC_IN_MODE'"
rosservice call /mavros/param/get "param_id: 'COM_ARM_WO_GPS'"
rosservice call /mavros/param/get "param_id: 'EKF2_AID_MASK'"
rosservice call /mavros/param/get "param_id: 'SYSID_THISMAV'"
```

### 7.4 ArduPilot / AP 常用参数

```bash
rosservice call /mavros/param/get "param_id: 'SYSID_THISMAV'"
rosservice call /mavros/param/get "param_id: 'FRAME_CLASS'"
rosservice call /mavros/param/get "param_id: 'GPS_TYPE'"
rosservice call /mavros/param/get "param_id: 'ARMING_CHECK'"
rosservice call /mavros/param/get "param_id: 'SERVO9_FUNCTION'"
rosservice call /mavros/param/get "param_id: 'SERVO10_FUNCTION'"
```

注意：

```text
PX4 和 ArduPilot 的参数名不一样
读取前最好先确认当前固件类型
```

---

## 8. 查看 MAVROS 提供了哪些服务

```bash
rosservice list | grep mavros
```

常见服务：

```text
/mavros/param/get
/mavros/param/set
/mavros/param/pull
/mavros/param/push
/mavros/cmd/arming
/mavros/set_mode
/mavros/cmd/takeoff
/mavros/cmd/land
/mavros/cmd/command
```

---

## 9. 解锁、上锁、切换模式

安全提醒：

```text
测试解锁、模式切换、抛投器、起飞命令前，必须拆桨。
```

### 9.1 解锁

```bash
rosservice call /mavros/cmd/arming "value: true"
```

### 9.2 上锁

```bash
rosservice call /mavros/cmd/arming "value: false"
```

### 9.3 PX4 切 OFFBOARD

```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
```

PX4 进入 OFFBOARD 前，一般必须先持续发送 setpoint。

### 9.4 AP 切 GUIDED

```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'GUIDED'"
```

### 9.5 常见模式

PX4：

```text
MANUAL
POSCTL
OFFBOARD
AUTO.MISSION
AUTO.LOITER
AUTO.LAND
```

ArduPilot：

```text
STABILIZE
LOITER
GUIDED
AUTO
RTL
LAND
```

---

## 10. 起飞和降落

### 10.1 起飞

```bash
rosservice call /mavros/cmd/takeoff "min_pitch: 0
yaw: 0
latitude: 0
longitude: 0
altitude: 3"
```

### 10.2 降落

```bash
rosservice call /mavros/cmd/land "min_pitch: 0
yaw: 0
latitude: 0
longitude: 0
altitude: 0"
```

注意：

```text
不同固件、不同模式下，对 takeoff/land 的支持不同。
比赛前必须在安全环境反复验证。
```

---

## 11. 发送目标点

### 11.1 发布本地位置目标

```bash
rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position:
    x: 1.0
    y: 0.0
    z: 2.0
  orientation:
    w: 1.0" -r 20
```

含义：

```text
x = 1.0
y = 0.0
z = 2.0
以 20Hz 频率持续发送
```

PX4 OFFBOARD 特别注意：

```text
必须先持续发送 setpoint，再切 OFFBOARD。
否则 OFFBOARD 可能切换失败或马上退出。
```

---

## 12. 抛投器 / 舵机控制

常用 MAVLink 命令：

```text
MAV_CMD_DO_SET_SERVO = 183
```

通过 MAVROS 调用：

```bash
rosservice call /mavros/cmd/command "broadcast: false
command: 183
confirmation: 0
param1: 9
param2: 1500
param3: 0
param4: 0
param5: 0
param6: 0
param7: 0"
```

参数含义：

```text
param1: 舵机通道号
param2: PWM 值
```

示例：

```text
1500  中位
1900  释放
1100  复位
```

实际值必须根据你的抛投器实测。

安全提醒：

```text
测试抛投器时必须拆桨。
不要在人或易碎物附近测试。
```

AP 飞控里常需要确认舵机参数，例如：

```bash
rosservice call /mavros/param/get "param_id: 'SERVO9_FUNCTION'"
rosservice call /mavros/param/get "param_id: 'SERVO9_MIN'"
rosservice call /mavros/param/get "param_id: 'SERVO9_MAX'"
rosservice call /mavros/param/get "param_id: 'SERVO9_TRIM'"
```

---

## 13. 记录比赛调试数据

用 rosbag 记录关键数据：

```bash
rosbag record /mavros/state \
  /mavros/imu/data \
  /mavros/local_position/pose \
  /mavros/local_position/velocity_local \
  /mavros/global_position/global \
  /mavros/global_position/raw/fix \
  /mavros/battery
```

停止录制：

```text
Ctrl + C
```

回放：

```bash
rosbag play 文件名.bag
```

---

## 14. 比赛常用启动顺序

### 14.1 PX4

终端 1：

```bash
source ~/.bashrc
roslaunch mavros_control_demo px4_onboard.launch
```

终端 2：

```bash
source ~/.bashrc
rostopic echo -n 1 /mavros/state
rosservice call /mavros/param/pull "force_pull: true"
```

终端 3：

```bash
source ~/.bashrc
rosrun 你的视觉包 你的视觉节点.py
```

### 14.2 ArduPilot / AP

终端 1：

```bash
source ~/.bashrc
roslaunch mavros_control_demo ap_onboard.launch
```

终端 2：

```bash
source ~/.bashrc
rostopic echo -n 1 /mavros/state
rosservice call /mavros/param/pull "force_pull: true"
```

终端 3：

```bash
source ~/.bashrc
rosrun 你的视觉包 你的视觉节点.py
```

---

## 15. 常见报错和含义

### 15.1 serial open: No such file or directory

示例：

```text
FCU: DeviceError: serial:open: No such file or directory
```

原因：

```text
launch 文件里的 fcu_url 写错
飞控没有插上
设备名变了
```

检查：

```bash
ls -l /dev/serial/by-id/
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

### 15.2 No GPS fix

示例：

```text
GP: No GPS fix
```

原因：

```text
室内没有 GPS
GPS 天线没接
GPS 尚未搜星
RTK 未固定
```

这通常不影响 MAVROS 基础连接。

### 15.3 RTT too high for timesync

示例：

```text
TM : RTT too high for timesync
```

含义：

```text
时间同步延迟偏高
刚连接时常见
USB/系统负载较高时也可能出现
```

如果只是偶尔出现，通常不是致命问题。

### 15.4 connected 一直是 False

检查：

```bash
rostopic echo -n 1 /mavros/state
```

可能原因：

```text
波特率错误
launch 文件用错，PX4/AP 不匹配
串口路径错误
飞控没有开启 MAVLink
用户没有 dialout 权限
```

排查顺序：

```bash
ls -l /dev/serial/by-id/
groups
roslaunch mavros px4.launch fcu_url:=设备路径:57600
roslaunch mavros px4.launch fcu_url:=设备路径:115200
roslaunch mavros apm.launch fcu_url:=设备路径:115200
```

---

## 16. 最重要的记忆表

```text
启动 MAVROS：
roslaunch

看实时数据：
rostopic echo

看数据频率：
rostopic hz

拉取参数：
rosservice call /mavros/param/pull

读取参数：
rosservice call /mavros/param/get

解锁/上锁：
rosservice call /mavros/cmd/arming

切模式：
rosservice call /mavros/set_mode

控制舵机/抛投器：
rosservice call /mavros/cmd/command
```

---

## 17. 当前项目建议文件结构

```text
~/mavros_ws/
  src/
    mavros_control_demo/
      launch/
        px4_onboard.launch
        ap_onboard.launch
      scripts/
        px4_basic_reader.py
        mavros_listener.py
```

建议比赛前准备两个 launch：

```text
px4_onboard.launch  用于 PX4 飞机
ap_onboard.launch   用于 ArduPilot/AP 飞机
```

这样换飞机时只需要换启动命令，不用临场改一堆东西。
