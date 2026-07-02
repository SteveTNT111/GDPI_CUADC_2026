# Gazebo 仿真测试脚本与命令备忘

这份文件用于记录：等机载电脑安装好 Gazebo / SITL 后，如何启动 MAVROS、查看飞机状态、读取 GPS/RTK/本地坐标，以及用 Python 节点发送位置 setpoint。

面向场景：

```text
机载电脑 Ubuntu 20.04 + ROS Noetic + MAVROS
先用 Gazebo/SITL 仿真测试脚本，后续再迁移到真实 AP/PX4 飞机
```

重点记住一句话：

```text
仿真和真机的主要区别是 MAVROS 的 fcu_url 不同。
真机一般走串口 /dev/serial/by-id/...
仿真一般走 UDP udp://...
```

---

## 1. 工作空间和包路径

ROS 工作空间：

```bash
~/mavros_ws
```

控制脚本所在功能包：

```bash
~/mavros_ws/src/mavros_control_demo
```

Python 脚本目录：

```bash
~/mavros_ws/src/mavros_control_demo/scripts
```

launch 文件目录：

```bash
~/mavros_ws/src/mavros_control_demo/launch
```

每次新开终端，先执行：

```bash
source ~/.bashrc
```

如果 `~/.bashrc` 还没有写入工作空间环境，则手动执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/mavros_ws/devel/setup.bash
```

---

## 2. 安装仿真前先检查

在机载电脑上先看系统、ROS、空间和内存：

```bash
lsb_release -a
rosversion -d
rospack find mavros
df -h
free -h
```

如果已经装了 Gazebo：

```bash
gazebo --version
```

Ubuntu 20.04 + ROS Noetic 常见组合是 Gazebo Classic 11。

---

## 3. 真机和仿真的 MAVROS 启动区别

### 3.1 AP/ArduPilot 真机，串口连接

先找稳定串口路径：

```bash
ls -l /dev/serial/by-id/
ls /dev/ttyACM*
ls /dev/ttyUSB*
groups
```

如果 `groups` 里没有 `dialout`，需要加入串口权限组：

```bash
sudo usermod -aG dialout $USER
```

然后注销重登，或者重启。

AP 真机启动 MAVROS：

```bash
roslaunch mavros apm.launch fcu_url:=/dev/serial/by-id/你的设备路径:115200
```

有些飞控串口波特率可能是 `57600`，如果 `115200` 不通，再试：

```bash
roslaunch mavros apm.launch fcu_url:=/dev/serial/by-id/你的设备路径:57600
```

### 3.2 PX4 真机，串口连接

PX4 真机常见启动：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/serial/by-id/你的设备路径:57600
```

如果不通，再根据飞控配置尝试 `115200`。

### 3.3 AP/ArduPilot SITL 仿真，UDP 连接

先启动 ArduPilot SITL。具体命令取决于后面安装的仿真环境，常见形式类似：

```bash
sim_vehicle.py -v ArduCopter --console --map
```

如果已经配置了 Gazebo 机型，可能类似：

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

然后另开终端启动 MAVROS：

```bash
source ~/.bashrc
roslaunch mavros apm.launch fcu_url:=udp://:14550@
```

如果 `/mavros/state` 没有 `connected: True`，再试这个 UDP 形式：

```bash
roslaunch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

判断是否连上：

```bash
rostopic echo -n 1 /mavros/state
```

看到类似下面这样就说明 MAVROS 和仿真飞控打通了：

```text
connected: True
armed: False
mode: "GUIDED"
```

### 3.4 PX4 SITL 仿真，UDP 连接

PX4 SITL 启动后，常见 MAVROS 连接方式：

```bash
roslaunch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

判断是否连上：

```bash
rostopic echo -n 1 /mavros/state
```

PX4 如果要用外部位置控制，通常需要进入 `OFFBOARD`，并且必须持续高频发送 setpoint。

---

## 4. 常用检查命令

查看 MAVROS 话题：

```bash
rostopic list | grep mavros
```

查看飞控连接状态：

```bash
rostopic echo -n 1 /mavros/state
```

查看 IMU：

```bash
rostopic echo -n 1 /mavros/imu/data
```

查看本地位置：

```bash
rostopic echo -n 1 /mavros/local_position/pose
```

查看全球 GPS 位置：

```bash
rostopic echo -n 1 /mavros/global_position/global
```

查看原始 GPS fix：

```bash
rostopic echo -n 1 /mavros/global_position/raw/fix
```

拉取全部参数：

```bash
rosservice call /mavros/param/pull "force_pull: true"
```

读取单个参数：

```bash
rosservice call /mavros/param/get "param_id: 'SYS_AUTOSTART'"
```

AP/ArduPilot 常看参数：

```bash
rosservice call /mavros/param/get "param_id: 'FRAME_CLASS'"
rosservice call /mavros/param/get "param_id: 'FRAME_TYPE'"
rosservice call /mavros/param/get "param_id: 'SERVO9_FUNCTION'"
rosservice call /mavros/param/get "param_id: 'SERVO9_MIN'"
rosservice call /mavros/param/get "param_id: 'SERVO9_MAX'"
rosservice call /mavros/param/get "param_id: 'SERVO9_TRIM'"
```

PX4 常看参数：

```bash
rosservice call /mavros/param/get "param_id: 'MAV_TYPE'"
rosservice call /mavros/param/get "param_id: 'SYS_AUTOSTART'"
```

---

## 5. 创建脚本目录

```bash
mkdir -p ~/mavros_ws/src/mavros_control_demo/scripts
mkdir -p ~/mavros_ws/src/mavros_control_demo/launch
```

---

## 6. 坐标/GPS/RTK 监听脚本

文件路径：

```bash
~/mavros_ws/src/mavros_control_demo/scripts/ap_position_reader.py
```

创建文件：

```bash
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
    rospy.loginfo_throttle(
        1.0,
        "STATE connected=%s armed=%s guided=%s mode=%s system_status=%s",
        msg.connected,
        msg.armed,
        msg.guided,
        msg.mode,
        msg.system_status,
    )


def global_cb(msg):
    rospy.loginfo_throttle(
        1.0,
        "GLOBAL lat=%.8f lon=%.8f alt=%.2f status=%d cov_type=%d",
        msg.latitude,
        msg.longitude,
        msg.altitude,
        msg.status.status,
        msg.position_covariance_type,
    )


def raw_fix_cb(msg):
    rospy.loginfo_throttle(
        1.0,
        "RAW_FIX lat=%.8f lon=%.8f alt=%.2f status=%d",
        msg.latitude,
        msg.longitude,
        msg.altitude,
        msg.status.status,
    )


def local_cb(msg):
    p = msg.pose.position
    rospy.loginfo_throttle(
        1.0,
        "LOCAL x=%.2f y=%.2f z=%.2f",
        p.x,
        p.y,
        p.z,
    )


def velocity_cb(msg):
    v = msg.twist.linear
    speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    rospy.loginfo_throttle(
        1.0,
        "VEL vx=%.2f vy=%.2f vz=%.2f speed=%.2f",
        v.x,
        v.y,
        v.z,
        speed,
    )


def gpsraw_cb(msg):
    rospy.loginfo_throttle(
        1.0,
        "GPSRAW fix_type=%s satellites_visible=%s eph=%s epv=%s",
        msg.fix_type,
        msg.satellites_visible,
        msg.eph,
        msg.epv,
    )


def main():
    rospy.init_node("ap_position_reader")

    rospy.Subscriber("/mavros/state", State, state_cb)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, global_cb)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, raw_fix_cb)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, local_cb)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, velocity_cb)

    if HAS_GPSRAW:
        rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, gpsraw_cb)

    rospy.loginfo("ap_position_reader started")
    rospy.spin()


if __name__ == "__main__":
    main()
EOF

chmod +x ~/mavros_ws/src/mavros_control_demo/scripts/ap_position_reader.py
```

运行：

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_position_reader.py
```

作用：

```text
持续打印飞控状态、全球 GPS、原始 GPS、本地位置、本地速度。
适合确认 MAVROS 是否真的有数据，而不是只看 connected: True。
```

---

## 7. AP Guided 位置 setpoint 控制脚本

文件路径：

```bash
~/mavros_ws/src/mavros_control_demo/scripts/ap_guided_position_setpoint.py
```

创建文件：

```bash
cat > ~/mavros_ws/src/mavros_control_demo/scripts/ap_guided_position_setpoint.py <<'EOF'
#!/usr/bin/env python3
import math
import rospy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest
from mavros_msgs.srv import SetMode, SetModeRequest
from geometry_msgs.msg import TwistStamped


class GuidedPositionController:
    def __init__(self):
        self.state = State()
        self.pose = None
        self.velocity = None

        self.target_x = rospy.get_param("~x", 0.0)
        self.target_y = rospy.get_param("~y", 0.0)
        self.target_z = rospy.get_param("~z", 2.0)
        self.target_yaw = rospy.get_param("~yaw", 0.0)

        self.tolerance = rospy.get_param("~tolerance", 0.35)
        self.speed_tolerance = rospy.get_param("~speed_tolerance", 0.25)
        self.hold_time = rospy.get_param("~hold_time", 3.0)

        self.auto_guided = rospy.get_param("~auto_guided", False)
        self.auto_arm = rospy.get_param("~auto_arm", False)
        self.auto_takeoff = rospy.get_param("~auto_takeoff", False)
        self.takeoff_alt = rospy.get_param("~takeoff_alt", self.target_z)

        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=10,
        )

        rospy.Subscriber("/mavros/state", State, self.state_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.velocity_cb)

        rospy.loginfo(
            "target local position x=%.2f y=%.2f z=%.2f yaw=%.2f",
            self.target_x,
            self.target_y,
            self.target_z,
            self.target_yaw,
        )

    def state_cb(self, msg):
        self.state = msg

    def pose_cb(self, msg):
        self.pose = msg

    def velocity_cb(self, msg):
        self.velocity = msg

    def make_target_pose(self):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z

        half_yaw = self.target_yaw * 0.5
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        return msg

    def distance_to_target(self):
        if self.pose is None:
            return None
        p = self.pose.pose.position
        dx = p.x - self.target_x
        dy = p.y - self.target_y
        dz = p.z - self.target_z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def current_speed(self):
        if self.velocity is None:
            return None
        v = self.velocity.twist.linear
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def wait_for_connection(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.state.connected:
            rospy.loginfo_throttle(2.0, "waiting for FCU connection...")
            rate.sleep()
        rospy.loginfo("FCU connected")

    def set_guided_mode(self):
        rospy.wait_for_service("/mavros/set_mode")
        set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        req = SetModeRequest()
        req.custom_mode = "GUIDED"
        res = set_mode(req)
        rospy.loginfo("set GUIDED result mode_sent=%s", res.mode_sent)

    def arm(self):
        rospy.wait_for_service("/mavros/cmd/arming")
        arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        req = CommandBoolRequest()
        req.value = True
        res = arm_srv(req)
        rospy.loginfo("arm result success=%s", res.success)

    def takeoff(self):
        rospy.wait_for_service("/mavros/cmd/takeoff")
        takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        req = CommandTOLRequest()
        req.altitude = self.takeoff_alt
        res = takeoff_srv(req)
        rospy.loginfo("takeoff result success=%s", res.success)

    def run(self):
        self.wait_for_connection()

        rate = rospy.Rate(20)

        for _ in range(40):
            self.setpoint_pub.publish(self.make_target_pose())
            rate.sleep()

        if self.auto_guided:
            self.set_guided_mode()
            rospy.sleep(1.0)

        if self.auto_arm:
            self.arm()
            rospy.sleep(1.0)

        if self.auto_takeoff:
            self.takeoff()
            rospy.sleep(3.0)

        hold_start = None
        arrived_reported = False

        while not rospy.is_shutdown():
            self.setpoint_pub.publish(self.make_target_pose())

            distance = self.distance_to_target()
            speed = self.current_speed()

            if distance is not None and speed is not None:
                inside = distance < self.tolerance and speed < self.speed_tolerance

                rospy.loginfo_throttle(
                    1.0,
                    "distance=%.2f speed=%.2f mode=%s armed=%s",
                    distance,
                    speed,
                    self.state.mode,
                    self.state.armed,
                )

                if inside:
                    if hold_start is None:
                        hold_start = rospy.Time.now()

                    held = (rospy.Time.now() - hold_start).to_sec()

                    if held >= self.hold_time and not arrived_reported:
                        rospy.loginfo(
                            "ARRIVED_AND_HOLDING distance=%.2f speed=%.2f held=%.1fs",
                            distance,
                            speed,
                            held,
                        )
                        arrived_reported = True
                else:
                    hold_start = None
                    arrived_reported = False

            rate.sleep()


def main():
    rospy.init_node("ap_guided_position_setpoint")
    controller = GuidedPositionController()
    controller.run()


if __name__ == "__main__":
    main()
EOF

chmod +x ~/mavros_ws/src/mavros_control_demo/scripts/ap_guided_position_setpoint.py
```

编译工作空间：

```bash
cd ~/mavros_ws
catkin_make
source ~/.bashrc
```

---

## 8. AP SITL 的 MAVROS launch 文件

文件路径：

```bash
~/mavros_ws/src/mavros_control_demo/launch/ap_sitl_mavros.launch
```

创建文件：

```bash
cat > ~/mavros_ws/src/mavros_control_demo/launch/ap_sitl_mavros.launch <<'EOF'
<launch>
  <arg name="fcu_url" default="udp://:14550@" />
  <arg name="gcs_url" default="" />

  <include file="$(find mavros)/launch/apm.launch">
    <arg name="fcu_url" value="$(arg fcu_url)" />
    <arg name="gcs_url" value="$(arg gcs_url)" />
  </include>
</launch>
EOF
```

启动：

```bash
source ~/.bashrc
roslaunch mavros_control_demo ap_sitl_mavros.launch
```

如果连不上，换 UDP 参数：

```bash
roslaunch mavros_control_demo ap_sitl_mavros.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

---

## 9. AP 真机串口 MAVROS launch 文件

文件路径：

```bash
~/mavros_ws/src/mavros_control_demo/launch/ap_serial_mavros.launch
```

创建文件：

```bash
cat > ~/mavros_ws/src/mavros_control_demo/launch/ap_serial_mavros.launch <<'EOF'
<launch>
  <arg name="fcu_url" default="/dev/serial/by-id/CHANGE_ME:115200" />
  <arg name="gcs_url" default="" />

  <include file="$(find mavros)/launch/apm.launch">
    <arg name="fcu_url" value="$(arg fcu_url)" />
    <arg name="gcs_url" value="$(arg gcs_url)" />
  </include>
</launch>
EOF
```

启动时把 `CHANGE_ME` 换成真实路径：

```bash
roslaunch mavros_control_demo ap_serial_mavros.launch fcu_url:=/dev/serial/by-id/你的设备路径:115200
```

---

## 10. PX4 SITL 的 MAVROS launch 文件

文件路径：

```bash
~/mavros_ws/src/mavros_control_demo/launch/px4_sitl_mavros.launch
```

创建文件：

```bash
cat > ~/mavros_ws/src/mavros_control_demo/launch/px4_sitl_mavros.launch <<'EOF'
<launch>
  <arg name="fcu_url" default="udp://:14540@127.0.0.1:14557" />
  <arg name="gcs_url" default="" />

  <include file="$(find mavros)/launch/px4.launch">
    <arg name="fcu_url" value="$(arg fcu_url)" />
    <arg name="gcs_url" value="$(arg gcs_url)" />
  </include>
</launch>
EOF
```

启动：

```bash
roslaunch mavros_control_demo px4_sitl_mavros.launch
```

---

## 11. 推荐的仿真测试流程

### 终端 1：启动仿真飞控

AP SITL 示例：

```bash
sim_vehicle.py -v ArduCopter --console --map
```

如果后续 Gazebo 机型配置好了，可能改成：

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

PX4 SITL 示例：

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

### 终端 2：启动 MAVROS

AP SITL：

```bash
source ~/.bashrc
roslaunch mavros_control_demo ap_sitl_mavros.launch
```

PX4 SITL：

```bash
source ~/.bashrc
roslaunch mavros_control_demo px4_sitl_mavros.launch
```

### 终端 3：确认 MAVROS 连通

```bash
source ~/.bashrc
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/imu/data
rostopic echo -n 1 /mavros/local_position/pose
rostopic echo -n 1 /mavros/global_position/global
```

### 终端 4：运行坐标监听脚本

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_position_reader.py
```

### 终端 5：运行 AP Guided 位置控制脚本

先保守测试，只发送目标点，不自动解锁、不自动起飞：

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_guided_position_setpoint.py _x:=0 _y:=0 _z:=2
```

如果确认仿真安全，再允许脚本自动切 GUIDED、解锁、起飞：

```bash
source ~/.bashrc
rosrun mavros_control_demo ap_guided_position_setpoint.py _x:=2 _y:=0 _z:=2 _auto_guided:=true _auto_arm:=true _auto_takeoff:=true
```

---

## 12. 如何判断飞机准确抵达并悬停

脚本里不是只看“位置接近”，而是同时看三件事：

```text
1. 当前位置到目标点的欧氏距离小于 tolerance
2. 当前速度小于 speed_tolerance
3. 连续保持 hold_time 秒
```

默认参数：

```text
tolerance = 0.35 m
speed_tolerance = 0.25 m/s
hold_time = 3.0 s
```

欧氏距离就是三维空间里两点之间的直线距离：

```text
distance = sqrt((x - target_x)^2 + (y - target_y)^2 + (z - target_z)^2)
```

如果终端打印：

```text
ARRIVED_AND_HOLDING
```

说明脚本认为飞机已经到达目标点附近，并且速度足够小，已经稳定悬停了一段时间。

比赛时不要只靠一次 `distance < tolerance` 判断到达。因为飞机可能只是路过目标点，还没有停稳。

---

## 13. 模式、解锁、起飞、降落常用服务

AP 切 GUIDED：

```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'GUIDED'"
```

解锁：

```bash
rosservice call /mavros/cmd/arming "value: true"
```

起飞到 2 米：

```bash
rosservice call /mavros/cmd/takeoff "min_pitch: 0
yaw: 0
latitude: 0
longitude: 0
altitude: 2"
```

降落：

```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'LAND'"
```

上锁：

```bash
rosservice call /mavros/cmd/arming "value: false"
```

PX4 切 OFFBOARD：

```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'OFFBOARD'"
```

PX4 降落：

```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'AUTO.LAND'"
```

---

## 14. 抛投器/舵机测试命令

通过 MAVLink 命令 `MAV_CMD_DO_SET_SERVO = 183` 控制舵机。

示例：控制 9 号舵机输出 1900us：

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

示例：控制 9 号舵机回到 1100us：

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

注意：

```text
真实飞机上测试抛投器前，必须拆桨。
先确认 SERVOx_FUNCTION、SERVOx_MIN、SERVOx_MAX、SERVOx_TRIM。
```

---

## 15. 记录数据，方便复盘

记录关键 MAVROS 话题：

```bash
mkdir -p ~/bags
rosbag record -O ~/bags/mavros_test.bag \
  /mavros/state \
  /mavros/imu/data \
  /mavros/local_position/pose \
  /mavros/local_position/velocity_local \
  /mavros/global_position/global \
  /mavros/global_position/raw/fix
```

回放：

```bash
rosbag play ~/bags/mavros_test.bag
```

查看 bag 里有什么：

```bash
rosbag info ~/bags/mavros_test.bag
```

---

## 16. 常见问题速查

### `/mavros/state` 没有 connected True

检查：

```bash
rostopic echo -n 1 /mavros/state
```

可能原因：

```text
1. 真机串口路径错了
2. 波特率错了
3. 用户没有 dialout 权限
4. 仿真 UDP 端口不匹配
5. SITL 没有启动或还没输出 MAVLink
```

### 有 connected True，但是没有 GPS

仿真或室内环境经常出现：

```text
GP: No GPS fix
```

这不一定代表 MAVROS 断了。先看：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/imu/data
rostopic echo -n 1 /mavros/local_position/pose
```

如果 IMU 和 local position 有数据，说明 MAVROS 基本是通的。

### AP Guided 脚本发了点，但飞机不动

检查：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/pose
rostopic hz /mavros/setpoint_position/local
```

可能原因：

```text
1. 没有进入 GUIDED
2. 没有解锁
3. 没有起飞，地面上不响应水平位置目标
4. EKF/定位还没 ready
5. setpoint 坐标系理解错了，本地坐标不是经纬度
```

### PX4 OFFBOARD 失败

PX4 需要先持续发送 setpoint，再切 OFFBOARD。

如果先切模式、后发 setpoint，PX4 通常会拒绝进入 OFFBOARD 或马上退出。

---

## 17. 最小记忆版

AP SITL 最小流程：

```bash
# 终端 1
sim_vehicle.py -v ArduCopter --console --map

# 终端 2
source ~/.bashrc
roslaunch mavros_control_demo ap_sitl_mavros.launch

# 终端 3
source ~/.bashrc
rostopic echo -n 1 /mavros/state

# 终端 4
source ~/.bashrc
rosrun mavros_control_demo ap_position_reader.py

# 终端 5
source ~/.bashrc
rosrun mavros_control_demo ap_guided_position_setpoint.py _x:=2 _y:=0 _z:=2 _auto_guided:=true _auto_arm:=true _auto_takeoff:=true
```

真机 AP 最小流程：

```bash
ls -l /dev/serial/by-id/
roslaunch mavros_control_demo ap_serial_mavros.launch fcu_url:=/dev/serial/by-id/你的设备路径:115200
rostopic echo -n 1 /mavros/state
rosrun mavros_control_demo ap_position_reader.py
```

