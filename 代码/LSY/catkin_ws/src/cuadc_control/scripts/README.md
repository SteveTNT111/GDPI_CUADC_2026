# `one_key_takeoff_wgs84_forward_rtl.py` 使用说明

## 脚本作用

该脚本使用 MAVROS 读取飞机启动时的 WGS84 经纬度和罗盘航向，自动计算
“启动航向正前方”的目标经纬度，然后执行：

```text
等待飞控和导航数据
-> 冻结起点、航向和目标 WGS84 坐标
-> 等待飞手在终端输入 YES
-> GUIDED
-> 自动解锁
-> 以解锁后的地面高度为基准上升 3 m
-> 飞到前方 10 m 的 WGS84 航点
-> 悬停 5 s
-> RTL 返回 Home
-> 必要时在 Home 附近切换 LAND
-> 等待降落并自动上锁
```

注意：当前脚本**不接收用户指定的目标经纬度**。目标点由“启动位置 + 启动
航向 + `forward_distance`”自动计算。默认前飞距离为 `10 m`。

坐标和高度约定：

- 经纬度来自 `/mavros/global_position/global`，使用 WGS84 椭球计算短距离目标。
- 航向来自 `/mavros/global_position/compass_hdg`，`0°` 为北，顺时针为正。
- 本地位置来自 `/mavros/local_position/pose`；ROS ENU 在日志中转换为 NED 显示。
- MAVLink 高度字段使用相对 Home 高度；脚本会先用地面 `rel_alt` 将设定的离地
  爬升高度换算成对应的相对 Home 目标，不使用 GPS 椭球绝对高程控制高度。

## 运行前检查

第一次使用必须先在 ArduPilot SITL 中验证。真机运行前至少确认：

1. 飞机在地面、未解锁，桨区无人，遥控器可随时接管。脚本按“启动时飞机
   确定在地面”工作，不会仅凭 `rel_alt` 是否接近 0 判断落地状态。
2. GPS 已定位，EKF、罗盘和 Home 点正常。
3. 机头朝向确实是希望前飞的方向。脚本启动后会冻结此时的航向。
4. 前方和返航路径有足够空域，飞行距离、RTL 高度及围栏参数安全。
5. 当前终端可交互，因为起飞前必须在该终端输入授权短语。

脚本默认会自动解锁；如果飞手已经用遥控器手动解锁，脚本会识别
`armed=True` 并跳过重复解锁。使用自动 GUIDED 起飞时不要靠推油门触发起飞，
授权前应让飞机保持在地面并按现场安全规程保持油门位置。

先编译并加载工作空间：

```bash
cd ~/catkin_ws
catkin_make
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
chmod +x ~/catkin_ws/src/cuadc_control/scripts/one_key_takeoff_wgs84_forward_rtl.py
```

## 真机运行

默认串口为 `/dev/ttyACM0:115200`，脚本会在需要时自动启动 `roscore` 和
MAVROS：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch cuadc_control one_key_takeoff_wgs84_forward_rtl.launch
```

如果飞控在其他串口：

```bash
roslaunch cuadc_control one_key_takeoff_wgs84_forward_rtl.launch \
  fcu_url:=/dev/ttyUSB0:921600
```

启动成功后，终端会打印起点 NED、起点 WGS84、启动航向、目标 WGS84 和完整
飞行计划。逐项检查无误后输入：

```text
YES
```

输入其他内容会取消任务，不会切换模式、解锁或起飞。

## SITL 运行

先启动 ArduPilot SITL，并确保其 MAVLink 输出端口为 `14550`。然后在一个终端
启动 MAVROS：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch mavros apm.launch fcu_url:=udp://:14550@
```

在另一个可交互终端启动任务，关闭脚本的 MAVROS 自动启动功能：

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch cuadc_control one_key_takeoff_wgs84_forward_rtl.launch \
  auto_start_mavros:=false \
  takeoff_altitude:=3.0 \
  forward_distance:=10.0 \
  waypoint_hover_time:=5.0
```

也可以不用 launch，直接运行脚本。`rosrun` 的私有 ROS 参数名前需要下划线：

```bash
rosrun cuadc_control one_key_takeoff_wgs84_forward_rtl.py \
  _auto_start_mavros:=false \
  _takeoff_altitude:=3.0 \
  _forward_distance:=10.0 \
  _waypoint_hover_time:=5.0
```

## 常用参数

以下参数可直接作为 launch 参数传入：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `takeoff_altitude` | `3.0` | 相对脚本执行时地面的爬升高度，单位 m |
| `forward_distance` | `10.0` | 沿启动航向前飞的水平距离，单位 m |
| `waypoint_hover_time` | `5.0` | 到达目标后的悬停时间，单位 s |
| `authorization_phrase` | `YES` | 终端飞行授权短语，英文字母不区分大小写 |
| `auto_start_mavros` | `true` | MAVROS 不存在时是否由脚本自动启动 |
| `fcu_url` | `/dev/ttyACM0:115200` | launch 使用的飞控连接地址 |

示例：起飞 `5 m`、向启动航向前飞 `20 m`、悬停 `8 s`：

```bash
roslaunch cuadc_control one_key_takeoff_wgs84_forward_rtl.launch \
  takeoff_altitude:=5.0 \
  forward_distance:=20.0 \
  waypoint_hover_time:=8.0
```

使用 `rosrun` 时，飞控连接参数名是脚本私有参数 `_mavros_fcu_url`：

```bash
rosrun cuadc_control one_key_takeoff_wgs84_forward_rtl.py \
  _mavros_fcu_url:=/dev/ttyUSB0:921600
```

## 启动前快速自检

MAVROS 已启动时，可以先检查以下数据是否持续更新：

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/global_position/global
rostopic echo -n 1 /mavros/global_position/compass_hdg
rostopic echo -n 1 /mavros/global_position/rel_alt
rostopic echo -n 1 /mavros/local_position/pose
```

`/mavros/state` 应显示 `connected: True`；GPS 经纬度、航向、相对高度和本地
位置必须有效且持续更新。脚本以默认 `2 s` 作为单条数据的新鲜度窗口；授权
等待时间较长时会主动等待下一批数据刷新。飞往航点期间允许默认最多 `5 s` 的
短时导航数据间断，超过后才进入异常返航流程。

## RTL 和异常处理

- 正常到达航点并悬停后，脚本切换到 `RTL`。
- `RTL_ALT_FINAL = 0` 时，ArduCopter 通常会返回 Home 后自动降落。
- `RTL_ALT_FINAL > 0` 时，脚本确认飞机已回到起飞点附近后再切换 `LAND`。
- 尚未发送起飞命令时发生异常，已解锁飞机切换 `LAND`，不会在地面请求 RTL。
- 起飞命令发出后的飞行流程发生异常时，脚本优先请求 `RTL`；请求失败才尝试
  `LAND`。
- 飞往航点或航点悬停时，短暂的 MAVROS `connected=False` 不会立即结束任务；
  脚本默认继续发布 setpoint 并等待最多 `10 s`。连接恢复后继续任务。
- 请求 RTL 前若连接仍中断，脚本会等待连接恢复；已经进入 RTL 后发生断连，
  则保持等待，不会因为一次状态瞬断退出进程。
- 起飞命令等待约 `5 s` 后返回失败且同时出现连接/模式异常时，按“ACK 可能
  丢失”处理：连接恢复后先检查飞机是否已经爬升；已爬升则继续监控，仍在地面
  才重新确认 GUIDED 并重试一次。
- RTL 超时且没有确认飞机回到 Home 附近时，脚本保持 RTL，不在异地强制降落。

不要把按 `Ctrl+C` 当作飞行中的可靠急停方式：它可能只停止 setpoint 发布。
紧急情况下应使用遥控器接管，并按现场情况切换 RTL、LAND 或人工稳定模式。

## 常见问题

### 一直等待导航数据

检查 GPS/EKF 是否正常，以及上述五个 MAVROS 话题是否存在并持续更新。室内无
GPS 时该脚本不能正常使用。

### 地面时 `rel_alt` 不是 0

这是允许的。`/mavros/global_position/rel_alt` 是相对 Home 的高度，不是独立的
落地检测值；Home 高度尚未更新或高度估计存在偏差时，地面可能显示负数或小幅
正数。脚本会在解锁后等待 `rel_alt` 持续有效默认 `2 s`，把当时的高度记录为
地面基准，再
将目标设置为“地面基准 + `takeoff_altitude`”。例如地面为 `-1.30 m`、设置起飞
高度为 `3.00 m` 时，发送的目标相对 Home 高度为 `1.70 m`，实际计划爬升仍为
`3.00 m`。

### 输入 `YES` 后任务仍取消

授权短语必须在运行 `roslaunch`/`rosrun` 的交互终端输入，默认的 `YES`、`yes`
和 `Yes` 都会接受。授权等待期间若飞机位置漂移超过默认 `2 m`，脚本仍会拒绝
起飞，需要重新启动确认。

### 飞机不向预期方向飞

目标方向使用脚本启动时的 `/mavros/global_position/compass_hdg`，不是启动后再
改变的机头方向。检查罗盘方向、磁干扰和机头朝向，修正后重新启动脚本。

### MAVROS 自动启动失败

真机检查设备名、波特率和串口权限；SITL 推荐手动启动 MAVROS，并传入
`auto_start_mavros:=false`，避免脚本尝试连接默认真机串口。
