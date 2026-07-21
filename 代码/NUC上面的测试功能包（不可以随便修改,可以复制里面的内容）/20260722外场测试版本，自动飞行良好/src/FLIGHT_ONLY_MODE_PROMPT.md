# CUADC 自动投放控制器“仅飞行、不识别”模式改造提示词

请在当前 ROS Noetic 工作空间继续开发，以现有源码为准。当前
cuadc_control/scripts/auto_drop_test.py 已经是完整的单节点任务控制器，具备：

- 自动启动 MAVROS、视觉和舵机；
- 起点/航向冻结；
- 投放区和侦察区 WGS84 航点解算；
- Enter 授权；
- GUIDED、自动解锁和起飞；
- 全局航点飞行；
- 投放区 W/Z 本地航线；
- A/B 识别、瞄准和投放；
- 侦察区 5 m 和 RTL；
- state、pose、velocity、导航和连接有界恢复。

本任务只是在现有控制器中增加一个独立的“仅飞行航线验证模式”，不要重写完整任务。

## 一、修改范围

只修改：

- /home/lab/catkin_ws/src/cuadc_control/scripts/auto_drop_test.py
- /home/lab/catkin_ws/src/cuadc_control/launch/auto_drop_test.launch
- /home/lab/catkin_ws/src/cuadc_control/ctrl_README.md

不要修改：

- semi_auto_drop_test.py 及其 launch
- one_key_takeoff_wgs84_forward_rtl.py 及其 launch
- detector_node.py
- servo_test.py
- cuadc_vision 下的 main、competition_main 或其他脚本

不要新建第二份飞行控制脚本。仅飞行模式必须复用 auto_drop_test.py 已有的航点解算、
起飞、全局航点、W/Z 点生成、遥测恢复、侦察区和 RTL 能力。

## 二、模式参数

新增 launch 和脚本参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| flight_only_mode | false | true=只验证航线，不识别、不瞄准、不投放 |
| flight_only_pattern_laps | 1 | 在投放区完整飞行 W/Z 的圈数 |
| flight_only_search_altitude_m | 3.0 | W/Z 航线相对地面高度 |
| flight_only_continue_to_recon | true | W/Z 完成后是否继续侦察区和 RTL |

参数约束：

~~~text
flight_only_pattern_laps >= 1
flight_only_search_altitude_m > 0
flight_only_search_altitude_m <= recon_altitude_m
~~~

flight_only_mode=true 时必须强制覆盖运行行为：

~~~text
视觉识别关闭
自动启动 detector/camera 关闭
舵机节点自动启动关闭
A/B 瞄准关闭
A/B 投放关闭
途中目标截获关闭
目标丢失/冻结逻辑不进入
~~~

即使 launch 中同时传入 auto_start_detector:=true、
auto_start_servo_test:=true 或 enable_b_dropper:=true，只要 flight_only_mode=true，
运行时都必须忽略它们并打印明确警告。

不要修改用户传入的 ROS 参数服务器值；只在节点内部计算有效配置，例如：

~~~text
self.detector_enabled = self.auto_start_detector and not self.flight_only_mode
self.servo_enabled = self.auto_start_servo_test and not self.flight_only_mode
~~~

## 三、仅飞行任务流程

运行：

~~~bash
roslaunch cuadc_control auto_drop_test.launch flight_only_mode:=true
~~~

流程必须是：

~~~text
检查/启动 MAVROS
-> 不启动 camera_node
-> 不启动 detector_node
-> 不启动 servo_test.py
-> 不等待 /vision/bucket/aim_info
-> 不等待 /servo/ready 或 /servo/cmd 订阅者
-> 等待飞控、GPS、本地位姿、速度、航向和 rel_alt
-> 冻结起点 WGS84、起点 NED、起始航向
-> 解算投放区和侦察区两个 WGS84 航点
-> 打印 FLIGHT ONLY 任务摘要
-> 等待 Enter
-> GUIDED -> 自动解锁 -> 起飞到 takeoff_altitude_m
-> 飞往投放区 WGS84 航点
-> 抵达投放区后按 area_search_pattern 完整飞行 W 或 Z
-> 按 flight_only_pattern_laps 完成指定圈数
-> 飞往侦察区并在途中爬升到 recon_altitude_m
-> 侦察区保持 recon_hold_s
-> RTL
~~~

如果 flight_only_continue_to_recon=false：

~~~text
完成投放区 W/Z
-> 直接 RTL
~~~

仅飞行模式中不得因为任何视觉消息切换航线。即使系统中有其他外部 detector 正在运行，
本节点也必须完全忽略 /vision/bucket/aim_info。

## 四、Enter 前的任务摘要

Enter 前打印：

- MODE = FLIGHT_ONLY；
- VISION = DISABLED；
- SERVO = DISABLED；
- AIM/DROP = DISABLED；
- 起点 WGS84；
- 起点 NED；
- 启动航向；
- 投放区 WGS84 和距离；
- 侦察区 WGS84 和距离；
- 起飞高度；
- W/Z 航线高度；
- W/Z 模式；
- W/Z 圈数；
- 侦察高度和保持时间；
- flight_only_continue_to_recon；
- 完整路线。

Enter 前仍然不得切 GUIDED、解锁、起飞或发布会移动飞机的 setpoint。

## 五、投放区 W/Z 纯航线执行

现有 area_search_for_target(dropper) 依赖视觉目标，不能直接用于仅飞行模式，因为它
会等待目标、可能提前返回或按搜索超时结束。

新增独立方法，建议命名：

~~~text
fly_area_pattern_only()
~~~

要求：

1. 复用现有 area_search_anchor、_area_search_local_points()、
   _area_search_enu_points() 和启动航向坐标约定。
2. 搜索中心使用抵达投放区时的本地位置。
3. 搜索长度方向使用起飞前冻结的 start_heading_deg，不使用抵达时可能变化的航向。
4. 内部可以先构造 NED 或机头前/右坐标，但发布到 MAVROS 时必须使用 ENU。
5. W 和 Z 都必须完整经过所有路径点，不受视觉消息影响。
6. 每个点必须满足：
   - 水平误差不超过 area_search_waypoint_tolerance_m；
   - 高度误差不超过 area_search_height_tolerance_m；
   - 连续保持 area_search_dwell_s。
7. 完成最后一点才计为一圈；按 flight_only_pattern_laps 重复。
8. 新一圈从第一个点重新开始，不重新冻结搜索中心或航向。
9. 每秒记录：
   - pattern；
   - 当前圈数/总圈数；
   - 当前点序号/总点数；
   - 目标 ENU；
   - 当前 ENU；
   - 水平和垂直误差；
   - state/pose/velocity 年龄；
   - 当前模式和 RC。
10. 该函数绝不读取目标 valid、目标 NED、A/B delta 或 ammo。

W/Z 点必须沿用当前比赛搜索区参数：

~~~text
area_search_width_m
area_search_length_m
area_search_footprint_width_m
area_search_footprint_length_m
area_search_waypoint_tolerance_m
area_search_height_tolerance_m
area_search_dwell_s
~~~

## 六、高度处理

航点高度都以起飞后记录的 ground_relative_altitude 为基准：

~~~text
起飞/飞往投放区目标 rel_alt
= ground_relative_altitude + takeoff_altitude_m

投放区 W/Z 目标 rel_alt
= ground_relative_altitude + flight_only_search_altitude_m

侦察区目标 rel_alt
= ground_relative_altitude + recon_altitude_m
~~~

不要把 D435 深度、目标 NED D 或 local ENU Z 当作相对地面高度。

从投放区全局航点切到 W/Z 本地 setpoint 时：

1. 停止发布全局 setpoint；
2. 先发布当前本地位置保持；
3. 以有界垂直速率移动到 flight_only_search_altitude_m；
4. 到达高度容差后开始 W/Z 第一段；
5. 禁止全局和本地 setpoint 并发。

从 W/Z 切到侦察区时：

1. 停止发布本地 W/Z setpoint；
2. 构造侦察区 GlobalPositionTarget；
3. 侦察区 target altitude 使用 ground_relative_altitude + recon_altitude_m；
4. 在前往侦察区过程中完成由搜索高度到 5 m 的爬升。

## 七、依赖启动必须真正跳过视觉和舵机

修改 start_dependencies() 和 wait_dependencies_ready()：

flight_only_mode=true 时：

- 只检查或启动 MAVROS；
- 不调用 rosnode 检查 detector_node/camera_node/servo_controller 是否运行；
- 不 spawn detector_node.launch；
- 不 spawn camera_node.launch；
- 不 spawn servo_test.py；
- 不等待视觉消息；
- 不等待 servo_ready；
- 不检查 /servo/cmd subscriber；
- 不启动依赖重启监控中的视觉/舵机分支。

如果外部视觉或舵机节点已经运行，不要杀死它们；本节点只是不使用它们。

## 八、状态机分支

在现有 run() 中尽早分流，建议状态：

~~~text
INIT_DEPENDENCIES
WAIT_MAVROS
WAIT_NAVIGATION
PLAN_WAYPOINTS
WAIT_ENTER
GUIDED_ARM_TAKEOFF
GOTO_DROP_ZONE
FLIGHT_ONLY_PATTERN
GOTO_RECON_CLIMB
RECON_HOLD
RTL
COMPLETE
~~~

进入 FLIGHT_ONLY_PATTERN 后禁止进入：

~~~text
A_SEARCH
A_AIM
A_DROP
RETURN_DROP_ZONE_FOR_B
B_SEARCH
B_AIM
B_DROP
~~~

推荐在 GOTO_DROP_ZONE 调用：

~~~text
fly_global_waypoint(drop_target, "GOTO_DROP_ZONE", detect_dropper=None)
~~~

或者增加明确的 monitor_vision=false 参数。不能继续使用 detect_dropper="A"，否则
外部视觉话题可能使航线测试意外切入瞄准。

## 九、不得发布的内容

flight_only_mode=true 时：

- /servo/cmd 发布次数必须为 0；
- A on、B on、A off、B off 均不得发布；
- 不得把 ammo_a/ammo_b 减少；
- 不得发布 last_drop=A/B；
- 不得打印“A/B 已投放”；
- 不得执行 aim_dropper()；
- 不得执行 area_search_for_target()；
- 不得根据 BucketAimInfo 产生本地修正。

/vision/mission_status 可以不发布；如果为了兼容显示仍发布，必须始终：

~~~text
aiming=false
last_drop=""
~~~

## 十、保留的飞行恢复和安全条件

仅飞行模式必须继续使用完整任务已有的：

- state_timeout_s 和 guided_state_grace_s；
- pose_timeout_s 和 guided_pose_grace_s；
- navigation_timeout_s 和 navigation_grace_s；
- connection_loss_grace_s；
- 服务重试和退避；
- LOCAL_POSITION_NED 10 Hz 请求；
- 全局航点连接/导航有界恢复；
- 最后安全 setpoint；
- CONTROL TRACE；
- fresh 非 GUIDED、armed=false、长时间断连等安全退出；
- 任务失败时优先 RTL；
- Ctrl+C 时的空中 RTL 保护。

不能因为“只是航线测试”就删除或放宽这些安全逻辑。

短暂 GPS、heading、rel_alt、pose 或 velocity 停更时，保持最后安全 setpoint 并等待
恢复；超过原有宽限后才 FAILSAFE。

## 十一、launch 使用命令

W 字航线测试：

~~~bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  area_search_pattern:=W
~~~

Z 字航线测试：

~~~bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  area_search_pattern:=Z
~~~

自定义夜间测试场地：

~~~bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  drop_zone_distance_m:=20.0 \
  recon_zone_distance_m:=38.0 \
  takeoff_altitude_m:=3.0 \
  flight_only_search_altitude_m:=3.0 \
  recon_altitude_m:=5.0 \
  area_search_pattern:=W \
  flight_only_pattern_laps:=1
~~~

只测试投放区 W/Z，完成后直接 RTL：

~~~bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  flight_only_continue_to_recon:=false \
  area_search_pattern:=Z
~~~

## 十二、日志要求

启动时必须有醒目日志：

~~~text
FLIGHT ONLY MODE
vision=DISABLED servo=DISABLED aiming=DISABLED drop=DISABLED
~~~

每次状态转换继续使用现有 _transition()，并记录：

- 旧状态和新状态；
- 原因；
- connected/armed/mode；
- 当前 NED/WGS84/rel_alt；
- 当前全局或本地安全 setpoint；
- W/Z 圈数和点序号。

不得出现容易误解的“目标搜索”“A 瞄准”“投放成功”等日志。

## 十三、文档更新

在 ctrl_README.md 中新增“仅飞行航线验证模式”章节，写明：

- 不启动相机、识别或舵机；
- 不瞄准、不投放；
- 完整路线；
- W/Z 切换命令；
- 自定义场地命令；
- 圈数和高度参数；
- 是否继续侦察区；
- Enter 前不动作；
- 保留的遥测恢复和安全退出；
- 夜间测试仍必须确保空域、地图边界、Home、GPS/EKF 和遥控接管安全。

添加 2026-07-22 修改记录。

## 十四、验证

执行：

~~~bash
cd /home/lab/catkin_ws/src/cuadc_control
python3 -m py_compile scripts/auto_drop_test.py
python3 -c "import xml.etree.ElementTree as ET; ET.parse('launch/auto_drop_test.launch')"
source /home/lab/catkin_ws/devel/setup.bash
ROS_HOME=/tmp/cuadc_ros_home \
  roslaunch --dump-params cuadc_control auto_drop_test.launch \
  flight_only_mode:=true area_search_pattern:=W
cd /home/lab/catkin_ws
catkin_make --pkg cuadc_control
~~~

删除 py_compile 生成的 __pycache__。

dump-params 必须确认：

~~~text
flight_only_mode=true
flight_only_pattern_laps=1
flight_only_search_altitude_m=3.0
flight_only_continue_to_recon=true
area_search_pattern=W
~~~

还要通过静态搜索确认 flight_only 分支不会调用：

~~~text
aim_dropper
drop
area_search_for_target
wait_for_target_and_confirm
_wait_for_servo_subscriber
~~~

## 十五、验收标准

1. 默认 flight_only_mode=false 时，现有完整自动投放任务行为不变。
2. flight_only_mode=true 时不启动 camera、detector 或 servo_test。
3. 不等待视觉或舵机话题。
4. Enter 前飞机不动作。
5. Enter 后正常 GUIDED、解锁、起飞并飞往投放区。
6. 投放区完整飞行指定 W 或 Z，不因任何视觉消息提前结束。
7. W/Z 完成圈数和每个点到达情况有明确日志。
8. 全程 /servo/cmd 发布次数为 0。
9. 不进入 A/B 搜索、瞄准或投放状态。
10. 默认继续飞往侦察区、爬升到 5 m、保持后 RTL。
11. flight_only_continue_to_recon=false 时 W/Z 完成后直接 RTL。
12. 临时遥测中断进入有界恢复，不因小问题立刻退出。
13. fresh 非 GUIDED、上锁或超出恢复宽限时仍安全停止控制。
14. W/Z、投放区距离、侦察区距离、航线高度和圈数均可通过 launch 参数调整。
