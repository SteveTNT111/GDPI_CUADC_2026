# CUADC 全自动投放测试脚本改造提示词

请直接在当前 ROS Noetic 工作空间继续开发。先完整读取当前源码、launch 和
ctrl_README.md，以仓库现状为准，不要依据旧聊天记录重建项目。

## 一、准确修改范围

只修改：

- /home/lab/catkin_ws/src/cuadc_control/scripts/auto_drop_test.py
- /home/lab/catkin_ws/src/cuadc_control/launch/auto_drop_test.launch
- /home/lab/catkin_ws/src/cuadc_control/ctrl_README.md

只读参考，不修改：

- cuadc_control/scripts/one_key_takeoff_wgs84_forward_rtl.py
- cuadc_control/launch/one_key_takeoff_wgs84_forward_rtl.launch
- cuadc_vision/scripts/detector_node.py
- cuadc_vision/launch/detector_node.launch
- cuadc_vision/scripts/servo_test.py

严禁修改：

- cuadc_control/scripts/semi_auto_drop_test.py
- cuadc_control/launch/semi_auto_drop_test.launch
- cuadc_vision/scripts/main.py
- cuadc_vision/scripts/competition_main.py
- 其他一键起飞和验证脚本

本任务不再使用 main 调度，不新建 RaceCommand 等比赛自定义消息。全任务由
auto_drop_test.py 单节点调度。

## 二、最终任务流程

运行 roslaunch cuadc_control auto_drop_test.launch 后执行：

~~~text
确保 ROS/MAVROS 可用
-> 自动拉起 camera_node + detector_node，OpenCV 画面必须打开
-> 自动拉起 servo_test.py
-> 等待飞控连接、GPS、本地位姿、航向、相对高度、视觉和舵机就绪
-> 以启动点 WGS84、启动航向和地面相对高度为基准
-> 自动解算投放区、侦察区两个 WGS84 航点
-> 在终端打印两个航点、距离、高度和完整任务摘要
-> 等待操作者按 Enter
-> Enter 后切 GUIDED、自动解锁、起飞到相对地面 3.0 m
-> 飞向投放区
   -> 航行途中提前稳定发现桶：立即停止全局航点推进，切入 A 瞄准投放
   -> 抵达投放区仍未发现桶：以投放区为中心执行 W/Z 字 NED 搜索
-> 完成 A 投放
-> 获取不同于 A 的第二个桶；已发现则直接瞄准，否则继续 W/Z 搜索
-> 完成 B 投放
-> 如果提前发现 A 后尚未真正到达投放区，而 B 需要搜索：
   先恢复飞到预设投放区中心，再从该中心搜索 B
-> A/B 均处理完成后，飞往预先解算的侦察区
-> 航行过程中爬升到相对地面 5.0 m
-> 抵达侦察区并保持可配置时间
-> 请求 RTL
~~~

两个航点都必须相对最初起飞点和最初航向解算。侦察区默认总前向距离是 60 m，
不是从当前位置再飞 60 m；默认投放区 35 m，因此两区中心默认相距约 25 m。

## 三、脚本权限和职责

新的 auto_drop_test.py 是完整任务控制器，获得以下职责：

- 自动检查并按需启动一份 MAVROS；
- 自动切换 GUIDED；
- 自动解锁；
- 调用起飞服务；
- 发布 WGS84 全局位置 setpoint；
- 发布本地位置 setpoint；
- 执行 W/Z 区域搜索；
- 执行 A/B 分层瞄准和目标丢失恢复；
- 发布现有 /servo/cmd；
- 最后请求 RTL。

同一时刻只能由脚本中的一个状态发布 setpoint。全局航点飞行与本地瞄准/搜索必须由
同一个顶层状态机串行切换，禁止两个后台循环同时发布全局和本地 setpoint。

“提升权限”仅表示提高该 ROS 节点在任务状态机中的控制职责，不得修改 Linux 用户
权限、设备权限或使用 sudo。

## 四、复用 WGS84 脚本的成熟逻辑

从现有 one_key_takeoff_wgs84_forward_rtl.py 中迁移或等价复用以下经过验证的逻辑，
但不要修改原文件：

- ROS master/MAVROS 检查；
- 飞控连接等待；
- GPS、本地位姿、航向、相对高度新鲜度检查；
- 起点和起始航向冻结；
- WGS84 目标点解算；
- 地面 rel_alt 基准记录；
- GUIDED 回读确认；
- 自动解锁与回读；
- 起飞服务重试和“起飞是否已经开始”判断；
- GlobalPositionTarget.FRAME_GLOBAL_REL_ALT setpoint；
- 航点水平/垂直误差和连续到达判定；
- 短暂连接/导航数据中断恢复；
- RTL 请求、回读、连接恢复和返航等待。

不要简单复制两个独立的 run() 流程。将这些能力拆成方法并接入 auto_drop_test.py
的一个顶层任务状态机。

## 五、起飞授权和航点参数

启动后必须先完成航点解算并打印：

- 起飞点 WGS84；
- 起始 NED；
- 起始航向；
- 投放区 WGS84；
- 侦察区 WGS84；
- 起飞高度；
- 侦察高度；
- W/Z 搜索模式；
- A/B 余弹；
- dry-run 状态。

只等待一次 Enter。Enter 前不得切换 GUIDED、解锁、调用起飞服务、发布会让飞机
移动的 setpoint 或打开 A/B 舵机。

Enter 代表操作者授权执行完整任务，不再要求输入 YES、RTL 或 2。

新增并同步脚本/launch 默认参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| full_mission_mode | true | true=执行完整比赛任务 |
| takeoff_altitude_m | 3.0 | 相对地面起飞和前往投放区高度 |
| drop_zone_distance_m | 35.0 | 投放区相对起点的前向距离 |
| recon_zone_distance_m | 60.0 | 侦察区相对起点的总前向距离 |
| recon_altitude_m | 5.0 | 侦察区相对地面高度 |
| recon_hold_s | 5.0 | 侦察区拍摄保持时间 |
| waypoint_tolerance_m | 1.0 | WGS84 航点水平到达容差 |
| waypoint_vertical_tolerance_m | 0.4 | 航点垂直误差容差 |
| waypoint_arrival_hold_s | 2.0 | 连续到达判定时间 |
| auto_start_mavros | true | 自动检查并按需启动 MAVROS |
| mavros_fcu_url | 保持现场现值 | 飞控连接地址 |
| detector_show_window | true | 自动启动 detector 时打开 CV 窗口 |

必须验证：

~~~text
drop_zone_distance_m > 0
recon_zone_distance_m > drop_zone_distance_m
takeoff_altitude_m > 0
recon_altitude_m >= takeoff_altitude_m
~~~

自定义测试场地示例：

~~~bash
roslaunch cuadc_control auto_drop_test.launch \
  drop_zone_distance_m:=20.0 \
  recon_zone_distance_m:=38.0 \
  takeoff_altitude_m:=3.0 \
  recon_altitude_m:=5.0 \
  area_search_pattern:=Z
~~~

## 六、自动启动依赖

保留现有“已经运行则不重复启动”的逻辑，并增强为：

1. MAVROS 不存在时按 mavros_fcu_url 启动一份；存在时复用。
2. camera/detector 都未运行时启动：

~~~bash
roslaunch cuadc_vision detector_node.launch \
  start_camera:=true \
  show_window:=true
~~~

3. detector 已运行而 camera 未运行时，只启动 camera。
4. camera 已运行而 detector 未运行时，启动 detector，并传入
   start_camera:=false show_window:=true。
5. 自动启动 servo_test.py；servo_controller 已存在时不重复启动。
6. 自动启动的子进程意外退出时，不要立刻结束飞行任务。空中先保持最后安全 setpoint，
   按可配置次数尝试重启依赖。

新增参数：

| 参数 | 默认值 |
|---|---:|
| dependency_restart_attempts | 3 |
| dependency_restart_backoff_s | 2.0 |
| detector_ready_timeout_s | 30.0 |
| servo_ready_timeout_s | 20.0 |

## 七、飞往投放区时提前发现目标

把 WGS84 航点飞行循环改造成可持续检查视觉目标的循环。

飞往投放区时：

- 继续以 10 Hz 发布投放区全局 setpoint；
- 同时检查 /vision/bucket/aim_info；
- 只有消息新鲜、valid=true、A NED 有效且目标连续稳定
  enroute_detect_stable_s 后才触发提前瞄准；
- 默认 enroute_detect_stable_s=0.3；
- 触发后记录飞机当时 NED/WGS84、桶 NED、目标消息年龄和距投放区剩余距离；
- 在同一个控制循环中停止发布全局 setpoint；
- 立即发布当前本地位置保持 setpoint；
- 确认本地保持已经开始后再进入 A 的瞄准状态；
- 不得让全局航点 setpoint 与本地瞄准 setpoint 并发。

单帧误检、短暂深度无效或 NED 无效时继续飞往投放区，不退出任务。

## 八、投放区 W/Z 搜索

抵达投放区后仍没有满足条件的目标时：

- 以抵达投放区时的本地位置为搜索中心；
- 以起飞时冻结的机头航向作为投放区长度方向；
- 在内部构造 NED 搜索点；
- 发布到 MAVROS 本地 ENU 话题前明确转换：

~~~text
ENU.x = NED.e
ENU.y = NED.n
ENU.z = -NED.d
~~~

- W/Z 搜索保持当前搜索高度，不自动降到投放高度；
- 搜索途中目标一旦稳定有效，立即停止路径推进并进入瞄准；
- area_search_pattern:=W 或 Z 必须通过 launch 参数切换；
- 保留现有搜索区宽度、长度、相机视野、路径点容差和 dwell 参数；
- 投放区距离、侦察区距离和 W/Z 模式必须互相独立，均可调整。

不得把 D435 CENTER Z、目标框深度、MAVROS local ENU Z、目标 NED D 混为同一量。

## 九、A/B 目标和余弹流程

脚本自己维护：

~~~text
ammo_a = 1
ammo_b = 1
completed_drop_targets = []
~~~

流程：

1. 第一个合法目标使用 A。
2. A 指令发布后记录冻结目标 NED。
3. B 优先选择距离 A 目标至少 distinct_target_distance_m=0.60 的不同目标。
4. B 尚未发现不同目标时继续 W/Z 搜索。
5. 如果 A 是途中提前投放，而 B 需要区域搜索，先飞到原预设投放区航点，再搜索 B。
6. B 搜索超过 second_target_search_timeout_s=30.0 仍无不同桶时，保持最后安全位置，
   强制发布 B on。
7. A/B 都处理后不回 LOITER，直接进入侦察区航点阶段。

enable_b_dropper:=true 必须保持支持。显式设为 false 时，A 完成后直接去侦察区。

当前 servo_test.py 没有结构化执行回执，所以日志必须区分“舵机命令已发布”和
“真实载荷已经释放”。没有 MAV_CMD_DO_SET_SERVO 回执时不得声称已经确认物理释放。
按现有接口，比赛任务可以在确认 /servo/cmd 有订阅者并完成命令发布后继续。

## 十、目标中心稳定后冻结 NED

目标 NED 在远距离和检测框抖动阶段继续滤波更新。当飞机接近识别框中心、目标 NED
在短窗口内变化很小时，冻结目标位置。

新增参数：

| 参数 | 默认值 |
|---|---:|
| target_lock_center_threshold_m | 0.20 |
| target_lock_window_s | 0.5 |
| target_lock_min_samples | 5 |
| target_lock_max_xy_span_m | 0.10 |
| target_lock_max_d_span_m | 0.15 |

锁定条件：

- 当前目标消息新鲜有效；
- 当前 dropper 对应的 NED 有效；
- X/Y 米制偏差均小于 target_lock_center_threshold_m；
- 窗口样本数足够；
- 窗口内 N/E 最大跨度不超过阈值；
- D 最大跨度不超过阈值；
- 没有单帧大跳变。

满足后：

- 对窗口 N/E/D 分别取中位数形成 frozen_target_ned；
- 日志输出样本数、N/E/D span、原始 NED 和冻结 NED；
- 后续 FINAL、强制投放和正常投放都只使用 frozen NED；
- 不再因为新识别框中心轻微移动而更新目标；
- 只有当前投放器流程取消、选择不同目标或进入下一枚弹时才清除冻结值。

冻结前目标丢失：执行现有保持、升高、局部搜索和重捕获。

冻结后目标短暂丢失：继续保持并飞向 frozen NED，不立即退出，也不重新用噪声目标
覆盖冻结位置。只有达到真正安全退出条件才终止该瞄准流程。

## 十一、0.5 秒正常投放和 20 秒强制投放

比赛模式默认：

- stable_time_s=0.5；
- phase_stable_time_s=0.5；
- X/Y、高度、水平速度、垂直速度门槛保持当前值；
- 正常投放仍要求所有门槛连续满足 0.5 s；
- 不得继续放宽门槛。

新增 aim_timeout_s=20.0。

计时规则：

- A/B 获得有效目标并进入 COARSE 时开始；
- 目标短暂丢失、state/pose/velocity 恢复期间暂停；
- 恢复后继续累计，不要因轻微问题永远重置为 0；
- 明确换成不同目标时才重新开始该目标的瞄准计时。

20 s 到期仍未正常满足投放条件时：

1. 冻结或选择最后可信目标 NED；
2. 发布最后安全保持 setpoint；
3. 标记 forced_drop=true；
4. 直接发布当前 dropper 的舵机打开命令；
5. 日志打印当前 X/Y、高度误差、水平/垂直速度、目标 NED 和 setpoint。

强制投放只绕过“未完全对准”条件，绝不能绕过：

- 飞控断开且超过恢复宽限；
- armed=false；
- 收到新鲜非 GUIDED 模式，说明飞手接管；
- 没有安全保持 setpoint；
- servo 节点不可用且重启失败；
- ROS shutdown。

## 十二、小问题不得直接退出：有界恢复策略

以下属于可恢复问题，不得单次发生就 return False 退出整个任务：

- 单帧或短时间目标丢失；
- 深度暂时无效；
- NED 暂时无效；
- 检测框跳变被拒绝；
- /mavros/state 短暂不新鲜；
- pose/velocity 同时短暂停更；
- GPS、航向或相对高度短暂停更；
- MAVROS 短暂断连；
- set_mode、arming、takeoff 服务一次超时；
- detector、camera、servo 子进程意外退出；
- W/Z 路径点短时间未进入容差；
- 目标稳定计时被轻微抖动打断。

恢复原则：

- 空中持续发布最后安全 setpoint；
- 冻结当前状态机阶段和稳定计时；
- 不使用陈旧 pose 计算新的目标修正；
- 恢复后继续原阶段，不从任务开头重来；
- 服务调用使用有上限的重试和退避；
- 依赖进程使用有上限的重启；
- 每秒输出恢复原因、消息年龄、已恢复时间和剩余宽限；
- 恢复超时后才进入任务级安全处理。

建议参数：

| 参数 | 默认值 |
|---|---:|
| state_timeout_s | 2.0 |
| guided_state_grace_s | 5.0 |
| pose_timeout_s | 2.0 |
| guided_pose_grace_s | 5.0 |
| navigation_timeout_s | 2.0 |
| navigation_grace_s | 5.0 |
| connection_loss_grace_s | 10.0 |
| service_retry_attempts | 3 |
| service_retry_backoff_s | 0.5 |

以下属于真正安全退出/接管条件，不能为了“不退出”而忽略：

- 收到新鲜非 GUIDED 模式，特别是 RC 切到 ALT_HOLD/LOITER；
- armed=false；
- 飞控断连超过宽限；
- state、pose、velocity 或导航数据超过恢复宽限；
- 没有最后安全 setpoint；
- 飞行前检查明确失败；
- 起飞多次重试后仍未开始；
- RTL 请求失败并且无法确认飞控状态；
- ROS shutdown/Ctrl+C。

空中任务级失败时优先请求 RTL；RTL 失败才按现有安全逻辑考虑 LAND。未离地前失败
不得无条件请求 RTL。

## 十三、顶层状态机

必须使用显式状态，禁止通过零散布尔变量隐式控制整场任务。建议：

~~~text
INIT_DEPENDENCIES
WAIT_MAVROS
WAIT_NAVIGATION
PLAN_WAYPOINTS
WAIT_ENTER
GUIDED_ARM_TAKEOFF
GOTO_DROP_ZONE
A_SEARCH
A_AIM
A_DROP
RETURN_DROP_ZONE_FOR_B
B_SEARCH
B_AIM
B_DROP
GOTO_RECON_CLIMB
RECON_HOLD
RTL
COMPLETE
RECOVERY
FAILSAFE
~~~

所有状态转换记录旧状态、新状态、原因、当前模式、armed/connected、当前
NED/WGS84/相对高度、A/B 余弹和最近安全 setpoint。

## 十四、保留现有低空瞄准安全设计

除本次明确修改的稳定时间和强制投放外，保留：

- COARSE 相对目标高度 2.0 m；
- FINE 1.7 m；
- FINAL/DROP 1.5 m；
- 目标 NED 低通滤波；
- 单帧大跳变拒绝；
- setpoint 水平/垂直限幅和时间斜坡；
- 目标丢失保持、升高和局部搜索；
- LOCAL_POSITION_NED 10 Hz 请求；
- state 与 pose/velocity 有界恢复；
- 新鲜非 GUIDED、上锁、断连安全处理；
- A/B 支持和 B 不同目标黑名单；
- CONTROL TRACE 诊断。

dry_run:=true 必须保留：执行完整起飞、航点、搜索、瞄准、侦察和 RTL 流程，但不
发布真实 A/B 舵机打开命令。ground_test:=true 继续用于不实际起飞的地面测试。

## 十五、launch 必须暴露的比赛参数

至少包含：

~~~text
full_mission_mode
takeoff_altitude_m
drop_zone_distance_m
recon_zone_distance_m
recon_altitude_m
recon_hold_s
area_search_pattern
area_search_width_m
area_search_length_m
area_search_timeout_s
second_target_search_timeout_s
enroute_detect_stable_s
stable_time_s
phase_stable_time_s
aim_timeout_s
target_lock_center_threshold_m
target_lock_window_s
target_lock_min_samples
target_lock_max_xy_span_m
target_lock_max_d_span_m
auto_start_mavros
mavros_fcu_url
auto_start_detector
detector_show_window
auto_start_servo_test
dependency_restart_attempts
connection_loss_grace_s
dry_run
~~~

脚本回退默认值必须与 launch 默认值完全同步。

比赛默认启动：

~~~bash
roslaunch cuadc_control auto_drop_test.launch
~~~

切换 Z 字：

~~~bash
roslaunch cuadc_control auto_drop_test.launch area_search_pattern:=Z
~~~

自定义场地：

~~~bash
roslaunch cuadc_control auto_drop_test.launch \
  drop_zone_distance_m:=28.0 \
  recon_zone_distance_m:=52.0 \
  area_search_pattern:=W
~~~

全流程 dry-run：

~~~bash
roslaunch cuadc_control auto_drop_test.launch dry_run:=true
~~~

## 十六、文档要求

更新 ctrl_README.md：

- 新的完整任务流程和 Enter 授权语义；
- 两个航点的解算基准；
- 所有新增参数和默认值；
- W/Z 切换及自定义场地命令；
- 0.5 s 正常投放；
- 20 s 强制投放；
- B 搜索 30 s 强制释放；
- 小问题有界恢复和真正安全退出条件；
- D435 CENTER Z、框深度、local ENU Z、目标 NED D 的区别；
- 当前舵机接口只能确认命令发布，不能确认物理载荷已释放；
- 添加当前日期修改记录。

## 十七、验证要求

至少执行：

~~~bash
cd /home/lab/catkin_ws/src/cuadc_control
python3 -m py_compile scripts/auto_drop_test.py
python3 -c "import xml.etree.ElementTree as ET; ET.parse('launch/auto_drop_test.launch')"
source /home/lab/catkin_ws/devel/setup.bash
ROS_HOME=/tmp/cuadc_ros_home \
  roslaunch --dump-params cuadc_control auto_drop_test.launch
cd /home/lab/catkin_ws
catkin_make --pkg cuadc_control
~~~

删除语法检查生成的 __pycache__。在 dump-params 中确认：

~~~text
full_mission_mode=true
takeoff_altitude_m=3.0
drop_zone_distance_m=35.0
recon_zone_distance_m=60.0
recon_altitude_m=5.0
area_search_pattern=W
stable_time_s=0.5
phase_stable_time_s=0.5
aim_timeout_s=20.0
second_target_search_timeout_s=30.0
detector_show_window=true
~~~

## 十八、验收标准

1. 不启动 main，只运行 auto_drop_test.launch 即可完成整场任务。
2. 启动后自动拉起视觉和舵机，视觉 OpenCV 画面打开。
3. Enter 前只检查和解算航点，飞机、模式和舵机不动作。
4. Enter 后自动起飞到 3 m 并飞向可配置投放区。
5. 途中目标稳定出现时能无 setpoint 冲突地切入 A 瞄准。
6. 到达投放区没有目标时能按参数执行 W 或 Z 字 NED 搜索。
7. 正常对准门槛连续满足 0.5 s 后立即发布投放命令。
8. 瞄准累计超过 20 s 时强制投放，但不绕过连接、armed、GUIDED 和安全保持条件。
9. 目标接近中心且 NED 稳定后冻结坐标，识别框抖动不再改变 frozen NED。
10. 目标短暂丢失、深度无效或 state/pose/velocity 短暂停更不会直接结束任务。
11. A 后能选择不同 B 目标；B 搜索 30 s 无目标时强制发布 B 命令。
12. A/B 完成后飞向预设侦察区并爬升到 5 m。
13. 侦察保持结束后请求 RTL。
14. fresh 非 GUIDED、上锁或真正超出恢复宽限时仍能安全停止控制。
15. 原 semi_auto_drop_test.py 和原 WGS84 脚本没有任何修改。
