# AP Guided Python 脚本备忘

本文记录后续要放到机载电脑 `mavros_control_demo` 包里的两个 Python 节点：

- `ap_position_reader.py`：读取 GPS / RTK / 本地位置 / 速度 / 飞控状态
- `ap_guided_position_setpoint.py`：在 ArduPilot GUIDED 模式下发送本地位置 setpoint，并判断飞机是否到达并悬停

适用场景：

```text
机载电脑 + MAVROS + ArduPilot/AP 飞控
```

后续如果先用 Gazebo 仿真测试，也可以用同样的 ROS 节点思路，只要仿真中的 MAVROS 话题和服务正常。

---

## 1. 前置条件

机载电脑已经有 ROS Noetic 工作空间：

```text
~/mavros_ws
```

已有 ROS 包：

```text
~/mavros_ws/src/mavros_control_demo
```

每次新开终端先执行：

```bash
source ~/.bashrc
```

或：

```bash
source /opt/ros/noetic/setup.bash
source ~/mavros_ws/devel/setup.bash
```

---

## 2. AP 位置读取节点

文件路径：

```text
~/mavros_ws/src/mavros_control_demo/scripts/ap_position_reader.py
```

创建命令：

```bash
mkdir -p ~/mavros_ws/src/mavros_control_demo/scripts

cat > ~/mavros_ws/src/mavros_control_demo/scripts/ap_position_reader.py <<'EOF'
#!/usr/bin/env python3
import math
import rospy
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped

try:
    from mavros_msgs.msg import GPSRAW
    HAS_GPSRAW = True
except Exception:
    HAS_GPSRAW = False

def state_cb(msg):
    rospy.loginfo_throttle(1, "STATE connected=%s armed=%s mode=%s",
                           msg.connected, msg.armed, msg.mode)

def global_cb(msg):
    rospy.loginfo_throttle(1, "GLOBAL lat=%.8f lon=%.8f alt=%.2f status=%d cov_type=%d",
                           msg.latitude, msg.longitude, msg.altitude,
                           msg.status.status, msg.position_covariance_type)

def raw_fix_cb(msg):
    rospy.loginfo_throttle(1, "RAW_FIX lat=%.8f lon=%.8f alt=%.2f status=%d",
                           msg.latitude, msg.longitude, msg.altitude,
                           msg.status.status)

def local_cb(msg):
    p = msg.pose.position
    rospy.loginfo_throttle(1, "LOCAL ENU x=%.3f y=%.3f z=%.3f", p.x, p.y, p.z)

def vel_cb(msg):
    v = msg.twist.linear
    speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
    rospy.loginfo_throttle(1, "VEL vx=%.3f vy=%.3f vz=%.3f speed=%.3f",
                           v.x, v.y, v.z, speed)

def gpsraw_cb(msg):
    fix_map = {
        0: "NO_GPS", 1: "NO_FIX", 2: "2D_FIX", 3: "3D_FIX",
        4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"
    }
    rospy.loginfo_throttle(1, "GPSRAW fix_type=%s sats=%d eph=%.2f epv=%.2f",
                           fix_map.get(msg.fix_type, str(msg.fix_type)),
                           msg.satellites_visible, msg.eph, msg.epv)

if __name__ == "__main__":
    rospy.init_node("ap_position_reader")
    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, global_cb)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, raw_fix_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_cb)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, vel_cb)

    if HAS_GPSRAW:
        rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, gpsraw_cb)

    rospy.loginfo("ap_position_reader started")
    rospy.spin()
EOF

chmod +x ~/mavros_ws/src/mavros_control_demo/scripts/ap_position_reader.py
```

运行：

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_position_reader.py
```

---

## 3. AP Guided 位置 setpoint 控制节点

文件路径：

```text
~/mavros_ws/src/mavros_control_demo/scripts/ap_guided_position_setpoint.py
```

创建命令：

```bash
mkdir -p ~/mavros_ws/src/mavros_control_demo/scripts

cat > ~/mavros_ws/src/mavros_control_demo/scripts/ap_guided_position_setpoint.py <<'EOF'
#!/usr/bin/env python3
import math
import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL

state = State()
pose = None
vel = None

def state_cb(msg):
    global state
    state = msg

def pose_cb(msg):
    global pose
    pose = msg

def vel_cb(msg):
    global vel
    vel = msg

def yaw_to_quat(yaw):
    q = PoseStamped().pose.orientation
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def distance_to_target(target):
    p = pose.pose.position
    dx = target.pose.position.x - p.x
    dy = target.pose.position.y - p.y
    dz = target.pose.position.z - p.z
    return math.sqrt(dx*dx + dy*dy + dz*dz)

def current_speed():
    if vel is None:
        return 999.0
    v = vel.twist.linear
    return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

if __name__ == "__main__":
    rospy.init_node("ap_guided_position_setpoint")

    target_x = rospy.get_param("~x", 0.0)
    target_y = rospy.get_param("~y", 0.0)
    target_z = rospy.get_param("~z", 2.0)
    target_yaw = rospy.get_param("~yaw", 0.0)

    tolerance = rospy.get_param("~tolerance", 0.3)
    speed_tolerance = rospy.get_param("~speed_tolerance", 0.25)
    hold_time = rospy.get_param("~hold_time", 2.0)
    rate_hz = rospy.get_param("~rate", 10.0)

    auto_guided = rospy.get_param("~auto_guided", True)
    auto_arm = rospy.get_param("~auto_arm", False)
    auto_takeoff = rospy.get_param("~auto_takeoff", False)
    takeoff_alt = rospy.get_param("~takeoff_alt", target_z)

    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, vel_cb)

    pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/set_mode")
    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/cmd/takeoff")

    set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)

    rate = rospy.Rate(rate_hz)

    rospy.loginfo("Waiting for FCU connection...")
    while not rospy.is_shutdown() and not state.connected:
        rate.sleep()

    rospy.loginfo("FCU connected")

    target = PoseStamped()
    target.header.frame_id = "map"
    target.pose.position.x = target_x
    target.pose.position.y = target_y
    target.pose.position.z = target_z
    target.pose.orientation = yaw_to_quat(target_yaw)

    for _ in range(20):
        target.header.stamp = rospy.Time.now()
        pub.publish(target)
        rate.sleep()

    if auto_guided:
        rospy.loginfo("Setting GUIDED mode")
        rospy.loginfo(set_mode(base_mode=0, custom_mode="GUIDED"))

    if auto_arm:
        rospy.loginfo("Arming")
        rospy.loginfo(arm_srv(True))

    if auto_takeoff:
        rospy.loginfo("Takeoff to %.2f m", takeoff_alt)
        rospy.loginfo(takeoff_srv(0, 0, 0, 0, takeoff_alt))

    arrived_since = None
    announced = False

    rospy.loginfo("Publishing target ENU x=%.2f y=%.2f z=%.2f", target_x, target_y, target_z)

    while not rospy.is_shutdown():
        target.header.stamp = rospy.Time.now()
        pub.publish(target)

        if pose is not None:
            dist = distance_to_target(target)
            spd = current_speed()

            rospy.loginfo_throttle(1, "target dist=%.3f speed=%.3f mode=%s armed=%s",
                                   dist, spd, state.mode, state.armed)

            if dist < tolerance and spd < speed_tolerance:
                if arrived_since is None:
                    arrived_since = rospy.Time.now()
                elif (rospy.Time.now() - arrived_since).to_sec() >= hold_time and not announced:
                    rospy.loginfo("ARRIVED_AND_HOLDING: dist=%.3f speed=%.3f", dist, spd)
                    announced = True
            else:
                arrived_since = None
                announced = False

        rate.sleep()
EOF

chmod +x ~/mavros_ws/src/mavros_control_demo/scripts/ap_guided_position_setpoint.py

cd ~/mavros_ws
catkin_make
source ~/.bashrc
```

---

## 4. AP MAVROS 启动

真实 AP 飞控常用：

```bash
roslaunch mavros_control_demo ap_onboard.launch
```

如果需要手动指定串口：

```bash
roslaunch mavros_control_demo ap_onboard.launch fcu_url:=/dev/serial/by-id/你的设备:115200
```

如果 115200 不通，试：

```bash
roslaunch mavros_control_demo ap_onboard.launch fcu_url:=/dev/serial/by-id/你的设备:57600
```

验证是否连通：

```bash
rostopic echo -n 1 /mavros/state
```

重点看：

```text
connected: True
mode: GUIDED 或其他当前模式
```

---

## 5. 运行位置读取节点

终端 1 启动 MAVROS 后，另开终端：

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_position_reader.py
```

它会打印：

```text
STATE
GLOBAL
RAW_FIX
LOCAL ENU
VEL
GPSRAW
```

其中 RTK 重点看：

```text
GPSRAW fix_type=RTK_FLOAT
GPSRAW fix_type=RTK_FIXED
```

---

## 6. 运行位置 setpoint 控制节点

先只发送目标点，不自动解锁、不自动起飞：

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_guided_position_setpoint.py _x:=1.0 _y:=0.0 _z:=2.0
```

带参数示例：

```bash
rosrun mavros_control_demo ap_guided_position_setpoint.py \
  _x:=1.0 \
  _y:=0.0 \
  _z:=2.0 \
  _tolerance:=0.3 \
  _speed_tolerance:=0.25 \
  _hold_time:=2.0
```

如果确认安全，并且已经拆桨或在仿真中，可以开启自动 GUIDED、解锁、起飞：

```bash
rosrun mavros_control_demo ap_guided_position_setpoint.py \
  _x:=1.0 \
  _y:=0.0 \
  _z:=2.0 \
  _auto_guided:=true \
  _auto_arm:=true \
  _auto_takeoff:=true \
  _takeoff_alt:=2.0
```

---

## 7. 如何判断飞机到达并悬停

不能只因为发了 setpoint 就认为飞机已经到达。

脚本里用三个条件判断：

```text
距离目标点足够近
速度足够小
稳定保持一段时间
```

默认条件：

```text
tolerance = 0.3 m
speed_tolerance = 0.25 m/s
hold_time = 2.0 s
```

也就是：

```text
飞机离目标点小于 0.3 米
飞机速度小于 0.25 m/s
连续保持 2 秒
```

满足后打印：

```text
ARRIVED_AND_HOLDING
```

这比“发一次目标点就认为到了”可靠。

---

## 8. setpoint、topic、service 的关系

简单理解：

```text
setpoint 是目标内容
topic 是持续通信通道
service 是一次性请求
```

例如：

```text
目标位置 setpoint
        ↓ 通过 topic 发送
/mavros/setpoint_position/local
        ↓ MAVROS
飞控 GUIDED 模式
```

常见 topic：

```bash
/mavros/setpoint_position/local
/mavros/setpoint_velocity/cmd_vel_unstamped
/mavros/local_position/pose
/mavros/global_position/global
```

常见 service：

```bash
/mavros/set_mode
/mavros/cmd/arming
/mavros/cmd/takeoff
/mavros/cmd/land
/mavros/cmd/command
```

记忆：

```text
持续控制飞机怎么飞：topic 发 setpoint
执行一次动作：service
查看实时状态：topic
读取参数：service
```

---

## 9. 仿真测试时的注意点

后续用 Gazebo / ArduPilot SITL 仿真时，整体流程仍然类似：

```text
启动仿真飞控
启动 MAVROS
运行 ap_position_reader.py
运行 ap_guided_position_setpoint.py
```

不同点通常在 `fcu_url`：

真实串口：

```text
/dev/serial/by-id/xxx:115200
```

仿真 UDP：

```text
udp://:14550@127.0.0.1:14555
```

具体端口以仿真启动方式为准。

---

## 10. 安全提醒

真实飞机测试前：

```text
必须拆桨
先只测试读数据
再测试切模式
再测试解锁
最后才测试位置控制
```

比赛脚本最终应该做成状态机：

```text
WAIT_FCU
SET_GUIDED
ARM
TAKEOFF
SEARCH_TARGET
ALIGN_TARGET
DROP
RETURN_OR_LAND
DONE
```

不要一开始就把所有动作写成一串不可中断的命令。
