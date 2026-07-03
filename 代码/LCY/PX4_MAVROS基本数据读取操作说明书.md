# PX4 MAVROS 基本数据读取操作说明书

目标：

```text
在已经安装好 ROS Noetic、MAVROS、Python 依赖的机载电脑上，
通过 USB 连接 PX4 飞控，
启动 MAVROS，
持续读取飞控姿态、角速度、加速度等基础信息。
```

适用对象：

```text
PX4 固件飞控
Ubuntu 20.04
ROS Noetic
MAVROS
```

不需要：

```text
不需要 Gazebo
不需要 SITL
不需要 ArduPilot 源码
不需要解锁
不需要 OFFBOARD
```

---

## 0. 终端使用方式

整个流程至少需要两个终端：

```text
终端 1：启动 MAVROS，保持它一直运行
终端 2：查看飞控状态、运行姿态读取脚本
```

如果你关闭终端 1，MAVROS 就断开，终端 2 里的姿态读取也会没有数据。

---

## 1. 插好飞控

用 USB 把 PX4 飞控连接到机载电脑。

然后在终端执行：

```bash
source /opt/ros/noetic/setup.bash
```

检查飞控是否被识别：

```bash
ls -l /dev/serial/by-id/
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

如果是 PX4 FMU v5，常见输出类似：

```text
/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00 -> ../../ttyACM0
/dev/ttyACM0
```

后面优先使用 `/dev/serial/by-id/...` 这种稳定路径。

---

## 2. 检查基础环境

执行：

```bash
rosversion -d
rospack find mavros
rospack find mavros_msgs
python3 -c "import rospy; import mavros_msgs.msg; import mavros_msgs.srv; import sensor_msgs.msg; print('mavros python ok')"
groups
```

正常情况下应该能看到：

```text
noetic
/opt/ros/noetic/share/mavros
/opt/ros/noetic/share/mavros_msgs
mavros python ok
```

`groups` 里应该包含：

```text
dialout
```

如果没有 `dialout`，执行：

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

重启后重新检查 `groups`。

---

## 3. 终端 1：启动 MAVROS

打开第一个终端。

先执行：

```bash
source /opt/ros/noetic/setup.bash
```

使用 PX4 launch 启动 MAVROS：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/你的PX4设备路径:57600
```

示例：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:57600
```

如果 `57600` 连不上，再试：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:115200
```

这个终端不要关闭。

---

## 4. 终端 2：确认 MAVROS 连上飞控

打开第二个终端。

执行：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /mavros/state
```

看到：

```text
connected: True
```

说明 MAVROS 已经连上飞控。

如果是：

```text
connected: False
```

或者没有输出，回到终端 1 检查 MAVROS 是否报错，重点看串口路径和波特率。

---

## 5. 手动查看一次姿态数据

只看一条数据：

```bash
rostopic echo -n 1 /mavros/imu/data
```

持续查看数据：

```bash
rostopic echo /mavros/imu/data
```

查看话题频率：

```bash
rostopic hz /mavros/imu/data
```

说明：

```text
-n 1 表示只读取 1 条消息，然后命令会自动结束。
如果想持续更新，不要加 -n 1。
```

`/mavros/imu/data` 里常用字段：

```text
orientation: 姿态四元数
angular_velocity: 角速度
linear_acceleration: 线加速度
```

---

## 6. 手动查看 GPS 星数

先列出 GPS 相关话题：

```bash
rostopic list | grep gps
rostopic list | grep global_position
```

PX4 + MAVROS 常见 GPS 星数话题有两种。你这台电脑当前发布的是第一种：

```text
/mavros/global_position/raw/satellites
/mavros/gpsstatus/gps1/raw
```

优先查看星数：

```bash
rostopic echo -n 1 /mavros/global_position/raw/satellites
```

查看一条原始 GPS 信息：

```bash
rostopic echo -n 1 /mavros/gpsstatus/gps1/raw
```

如果 `/mavros/gpsstatus/gps1/raw` 存在，重点看：

```text
fix_type: GPS 定位类型
satellites_visible: 当前可见卫星数
```

查看 GPS 频率：

```bash
rostopic hz /mavros/global_position/raw/satellites
rostopic hz /mavros/gpsstatus/gps1/raw
```

查看全球位置：

```bash
rostopic echo -n 1 /mavros/global_position/global
```

如果 `/mavros/gpsstatus/gps1/raw` 不存在，先确认 MAVROS 是否已经连接：

```bash
rostopic echo -n 1 /mavros/state
```

如果 `connected: True` 但没有 GPSRAW 话题，优先使用 `/mavros/global_position/raw/satellites` 获取星数，再用 `/mavros/global_position/global` 判断 GPS 是否有定位状态。

---

## 7. 创建持续姿态和 GPS 星数读取 Python 脚本

在第二个终端执行：

```bash
mkdir -p ~/mavros_attitude_test
gedit ~/mavros_attitude_test/px4_attitude_monitor.py
```

把下面内容写入文件：

```python
#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import UInt32
from mavros_msgs.msg import State

try:
    from mavros_msgs.msg import GPSRAW
    HAS_GPSRAW = True
except Exception:
    HAS_GPSRAW = False


last_state = None
last_gps_raw = None
last_global_fix = None
last_satellites = None


def quat_to_euler_deg(q):
    x, y, z, w = q.x, q.y, q.z, q.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def state_cb(msg):
    global last_state
    last_state = msg


def gps_raw_cb(msg):
    global last_gps_raw
    last_gps_raw = msg


def global_fix_cb(msg):
    global last_global_fix
    last_global_fix = msg


def satellites_cb(msg):
    global last_satellites
    last_satellites = msg.data


def imu_cb(msg):
    roll, pitch, yaw = quat_to_euler_deg(msg.orientation)
    av = msg.angular_velocity
    la = msg.linear_acceleration

    mode = last_state.mode if last_state else "UNKNOWN"
    connected = last_state.connected if last_state else False
    if last_gps_raw:
        satellites = last_gps_raw.satellites_visible
    elif last_satellites is not None:
        satellites = last_satellites
    else:
        satellites = -1

    fix_type = last_gps_raw.fix_type if last_gps_raw else -1
    gps_status = last_global_fix.status.status if last_global_fix else -99

    rospy.loginfo_throttle(
        0.2,
        "connected=%s mode=%s | roll=%.2f pitch=%.2f yaw=%.2f deg | "
        "gyro=(%.3f %.3f %.3f) rad/s | acc=(%.2f %.2f %.2f) m/s^2 | "
        "gps_sats=%s fix_type=%s navsat_status=%s",
        connected,
        mode,
        roll,
        pitch,
        yaw,
        av.x,
        av.y,
        av.z,
        la.x,
        la.y,
        la.z,
        satellites,
        fix_type,
        gps_status,
    )


def main():
    rospy.init_node("px4_attitude_monitor")
    rospy.Subscriber("/mavros/state", State, state_cb, queue_size=10)
    rospy.Subscriber("/mavros/imu/data", Imu, imu_cb, queue_size=50)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, global_fix_cb, queue_size=10)
    rospy.Subscriber("/mavros/global_position/raw/satellites", UInt32, satellites_cb, queue_size=10)
    if HAS_GPSRAW:
        rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, gps_raw_cb, queue_size=10)
        rospy.loginfo("px4_attitude_monitor started, GPSRAW and raw/satellites enabled")
    else:
        rospy.logwarn("px4_attitude_monitor started, GPSRAW message type not available; using raw/satellites")
    rospy.spin()


if __name__ == "__main__":
    main()
```

保存后执行：

```bash
chmod +x ~/mavros_attitude_test/px4_attitude_monitor.py
```

---

## 8. 运行姿态和 GPS 星数读取脚本

确认终端 1 的 MAVROS 还在运行。

然后在终端 2 执行：

```bash
source /opt/ros/noetic/setup.bash
python3 ~/mavros_attitude_test/px4_attitude_monitor.py
```

正常情况下会持续输出类似：

```text
connected=True mode=AUTO.LOITER | roll=1.23 pitch=-0.55 yaw=37.80 deg | gyro=(...) | acc=(...) | gps_sats=18 fix_type=3 navsat_status=0
```

实测输出示例：

```text
connected=True mode=AUTO.LOITER | roll=-2.17 pitch=-2.74 yaw=-19.90 deg | gyro=(0.000 0.001 0.001) rad/s | acc=(0.48 -0.37 9.78) m/s^2 | gps_sats=0 fix_type=-1 navsat_status=-99
```

这类输出说明：

```text
姿态数据已经正常接入：roll / pitch / yaw、gyro、acc 都在持续更新。
GPS 星数字段已经接入脚本：gps_sats 已经出现在输出里。
gps_sats=0 表示当前没有可见卫星，或飞控/MAVROS 当前发布的星数就是 0。
fix_type=-1 表示没有收到 /mavros/gpsstatus/gps1/raw 这个 GPSRAW 话题。
navsat_status=-99 表示脚本暂时没有收到 /mavros/global_position/global 的 NavSatFix 数据。
```

如果在室内测试，`gps_sats=0` 很常见。比赛外场测试时，通常希望 `gps_sats` 变成一个正数，例如 `10`、`15`、`20`，并且 `/mavros/global_position/global` 能正常输出经纬度。

按 `Ctrl+C` 可以停止脚本。

---

## 9. 推荐的比赛程序结构

比赛时不要用 `rostopic echo` 当程序输入。

推荐结构：

```text
终端 1：roslaunch mavros px4.launch ...
终端 2：运行你的 Python 控制程序
```

Python 控制程序里订阅：

```text
/mavros/state
/mavros/imu/data
/mavros/local_position/pose
/mavros/global_position/global
/mavros/global_position/raw/satellites
/mavros/gpsstatus/gps1/raw
```

然后根据视觉识别结果，发布或调用 MAVROS 的控制接口。

---

## 10. 常见问题

### 10.1 `Unable to communicate with master`

说明 ROS Master 没起来。

解决：

```text
先启动终端 1 的 roslaunch mavros px4.launch ...
然后再在终端 2 里 rostopic echo。
```

### 10.2 `Permission denied`

说明当前用户没有串口权限。

解决：

```bash
sudo usermod -aG dialout $USER
sudo reboot
```

### 10.3 `/mavros/state` 不是 `connected: True`

常见原因：

```text
1. fcu_url 设备路径写错
2. 波特率不对，PX4 可尝试 57600 或 115200
3. USB 线或飞控未正常识别
4. MAVROS launch 文件用错，PX4 应使用 px4.launch
```

### 10.4 `rostopic echo -n 1` 自动退出

这是正常的。

```text
-n 1 就是只读取 1 条数据。
持续读取要用 rostopic echo /mavros/imu/data。
```

### 10.5 姿态里的 `orientation` 不是角度

`/mavros/imu/data` 里的 `orientation` 是四元数，不是 roll/pitch/yaw。

如果想看角度，用本说明书里的 `px4_attitude_monitor.py`，它已经转换成：

```text
roll / pitch / yaw
```

### 10.6 GPS 星数一直是 `-1`

说明脚本还没有收到 GPS 星数数据。

先检查话题是否存在：

```bash
rostopic list | grep gpsstatus
rostopic list | grep raw/satellites
rostopic echo -n 1 /mavros/global_position/raw/satellites
rostopic echo -n 1 /mavros/gpsstatus/gps1/raw
```

如果话题不存在，但 `/mavros/state` 是 `connected: True`，先用下面命令确认有没有普通 GPS 位置：

```bash
rostopic echo -n 1 /mavros/global_position/global
```

---

## 11. 最小命令版

终端 1：

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00:57600
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /mavros/state
rostopic hz /mavros/imu/data
rostopic echo -n 1 /mavros/global_position/raw/satellites
rostopic echo -n 1 /mavros/gpsstatus/gps1/raw
python3 ~/mavros_attitude_test/px4_attitude_monitor.py
```
