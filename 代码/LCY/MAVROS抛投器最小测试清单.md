# MAVROS 抛投器最小测试清单

目标：

```text
新机载电脑 + 飞控 + MAVROS
只测试通过 MAVROS 控制抛投器舵机
不跑 Gazebo，不跑 SITL，不需要 ArduPilot 源码
```

安全前提：

```text
测试前必须拆桨。
舵机建议外接 BEC 供电，并和飞控共地。
不要在人、玻璃、易碎物附近测试抛投器。
```

---

## 1. 检查新机载电脑是否已有 ROS/MAVROS

```bash
rosversion -d
rospack find mavros
python3 -c "import rospy; import mavros_msgs.srv; print('rospy mavros_msgs ok')"
```

如果输出类似：

```text
noetic
/opt/ros/noetic/share/mavros
rospy mavros_msgs ok
```

说明基础环境够用。

如果 `rosversion` 或 `rospack` 找不到，先执行：

```bash
source /opt/ros/noetic/setup.bash
```

如果仍然找不到，说明这台电脑还没装 ROS/MAVROS。

Ubuntu 20.04 + ROS Noetic 的最小安装命令：

```bash
sudo apt update
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras python3-catkin-tools
sudo apt install -y geographiclib-tools
sudo geographiclib-get-geoids egm96-5
```

---

## 2. 检查飞控串口

插上飞控后执行：

```bash
ls -l /dev/serial/by-id/
ls /dev/ttyACM*
ls /dev/ttyUSB*
groups
```

优先使用稳定路径，例如：

```text
/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00
```

如果 `groups` 里没有 `dialout`，执行：

```bash
sudo usermod -aG dialout $USER
```

然后注销重登，或者重启。

---

## 3. 启动 MAVROS

AP/ArduPilot 固件常用：

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros apm.launch fcu_url:=/dev/serial/by-id/你的飞控设备路径:115200
```

如果 `115200` 不通，可以试：

```bash
roslaunch mavros apm.launch fcu_url:=/dev/serial/by-id/你的飞控设备路径:57600
```

PX4 固件常用：

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/你的飞控设备路径:57600
```

如果 PX4 的 `57600` 不通，再试 `115200`。

---

## 4. 确认 MAVROS 已连接飞控

另开一个终端：

```bash
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /mavros/state
```

看到：

```text
connected: True
```

才继续测试舵机。

---

## 5. 确认舵机通道参数

假设抛投器接在 `SERVO9`：

```bash
rosservice call /mavros/param/get "param_id: 'SERVO9_FUNCTION'"
rosservice call /mavros/param/get "param_id: 'SERVO9_MIN'"
rosservice call /mavros/param/get "param_id: 'SERVO9_MAX'"
rosservice call /mavros/param/get "param_id: 'SERVO9_TRIM'"
```

如果是 AP/ArduPilot，建议抛投器输出通道不要被飞控其他功能占用。可先设置：

```bash
rosservice call /mavros/param/set "param_id: 'SERVO9_FUNCTION'
value:
  integer: 0
  real: 0.0"
```

设置后重新拉参数确认：

```bash
rosservice call /mavros/param/pull "force_pull: true"
rosservice call /mavros/param/get "param_id: 'SERVO9_FUNCTION'"
```

---

## 6. 直接用命令测试舵机

释放位置示例，`SERVO9` 输出 `1900us`：

```bash
rosservice call /mavros/cmd/command "broadcast: false
command: 183
confirmation: 0
param1: 9
param2: 1900
param3: 0
param4: 0
param5: 0
param6: 0
param7: 0"
```

复位位置示例，`SERVO9` 输出 `1100us`：

```bash
rosservice call /mavros/cmd/command "broadcast: false
command: 183
confirmation: 0
param1: 9
param2: 1100
param3: 0
param4: 0
param5: 0
param6: 0
param7: 0"
```

说明：

```text
command: 183 是 MAV_CMD_DO_SET_SERVO
param1: 舵机输出通道号
param2: PWM 值，常见范围 1000-2000
```

---

## 7. Python 最小测试脚本

在新机载电脑上创建：

```bash
mkdir -p ~/payload_test
gedit ~/payload_test/drop_servo_test.py
```

写入：

```python
#!/usr/bin/env python3
import sys
import time
import rospy
from mavros_msgs.srv import CommandLong


def set_servo(channel, pwm):
    rospy.wait_for_service("/mavros/cmd/command")
    cmd = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
    return cmd(
        broadcast=False,
        command=183,
        confirmation=0,
        param1=float(channel),
        param2=float(pwm),
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=0,
    )


if __name__ == "__main__":
    rospy.init_node("drop_servo_test")

    channel = int(sys.argv[1]) if len(sys.argv) > 1 else 9
    release_pwm = int(sys.argv[2]) if len(sys.argv) > 2 else 1900
    reset_pwm = int(sys.argv[3]) if len(sys.argv) > 3 else 1100
    hold_seconds = float(sys.argv[4]) if len(sys.argv) > 4 else 0.8

    rospy.loginfo("release: SERVO%d -> %dus", channel, release_pwm)
    print(set_servo(channel, release_pwm))
    time.sleep(hold_seconds)

    rospy.loginfo("reset: SERVO%d -> %dus", channel, reset_pwm)
    print(set_servo(channel, reset_pwm))
```

赋予执行权限：

```bash
chmod +x ~/payload_test/drop_servo_test.py
```

运行：

```bash
source /opt/ros/noetic/setup.bash
python3 ~/payload_test/drop_servo_test.py 9 1900 1100 0.8
```

含义：

```text
9: 控制 SERVO9
1900: 释放位置 PWM
1100: 复位位置 PWM
0.8: 释放后等待 0.8 秒再复位
```

---

## 8. 如果舵机不动

先看 MAVROS 是否真的连接：

```bash
rostopic echo -n 1 /mavros/state
```

再看服务是否存在：

```bash
rosservice list | grep /mavros/cmd/command
```

再确认通道：

```bash
rosservice call /mavros/param/get "param_id: 'SERVO9_FUNCTION'"
```

常见原因：

```text
1. 舵机接错输出口，SERVO9 不等于板子上随便第 9 个插针
2. 飞控输出口没有给舵机供电，舵机需要外接 BEC
3. 舵机信号线、5V、GND 接线错误
4. SERVO9_FUNCTION 被其他飞控功能占用
5. 飞控安全开关、解锁状态或输出限制阻止了 PWM 输出
6. PWM 值不适合这个抛投器，需要实测 1000/1500/2000
```

