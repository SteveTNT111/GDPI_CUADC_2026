# CUADC Control — ROS Noetic 控制验证包

`cuadc_control` 存放 CUADC 2026 的飞行控制验证脚本；视觉检测、`BucketAimInfo`、
`MissionStatus` 和舵机桥位于 `cuadc_vision`。真机前必须先完成拆桨地面检查和
`dry_run`，并确保遥控器可随时切走 `GUIDED`。

## 编译

```bash
cd ~/catkin_ws
catkin_make --pkg cuadc_control
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

## 脚本边界

- `one_key_takeoff_wgs84_forward_rtl.py`：独立的双 WGS84 航点/RTL 验证脚本。
- `semi_auto_drop_test.py`：原半自动低空瞄准测试，保持不变。
- `auto_drop_test.py`：本页说明的完整比赛任务控制器；不依赖 `main.py`，不使用
  `RaceCommand` 等比赛自定义调度消息。

同一时间不要运行另一个会发布 MAVROS 全局/本地位置 setpoint、切换模式或控制
A/B 舵机的任务节点。

## 全自动任务

默认命令：

```bash
roslaunch cuadc_control auto_drop_test.launch
```

单个 `auto_drop_test.py` 节点串行执行：

```text
检查并按需启动 MAVROS
-> 按当前节点状态启动 camera_node / detector_node（OpenCV 窗口默认打开）
-> 按需启动 servo_test.py
-> 等待飞控、GPS、本地位姿、速度、航向、rel_alt、视觉和舵机就绪
-> 冻结启动点 WGS84、启动 NED、启动航向和启动 rel_alt
-> 从同一个启动点和启动航向解算投放区/侦察区两个 WGS84 航点
-> 打印完整任务摘要，只等待一次 Enter
-> GUIDED -> 自动解锁 -> 记录地面 rel_alt -> 起飞到离地 3.0m
-> 飞往投放区；途中稳定发现桶可立即停止全局航点并切到本地 A 瞄准
-> 抵达投放区仍无桶时，以投放区中心和启动航向执行 W/Z/I 搜索
-> A 分层瞄准和命令发布
-> 选择距离 A 至少 0.60m 的不同 B 目标；必要时继续 W/Z/I 搜索
-> 若 A 在途中完成而 B 需要搜索，先返回预设投放区中心
-> B 分层瞄准和命令发布；30s 无不同目标则在安全保持下强制发布 B on
-> 飞往预先解算的侦察区，并在航行中爬升到离地 5.0m
-> 侦察区保持 5.0s -> RTL
```

投放区和侦察区都相对最初起飞点及最初航向解算。默认投放区总前向距离是
`32m`，侦察区总前向距离是 `60m`，两中心约相距 `28m`；侦察区不是从投放完成
位置“再向前飞 60m”。

### Enter 授权

启动后会先完成依赖检查、导航数据等待、航点解算并打印：

- 起飞点 WGS84、起始 NED、起始航向；
- 投放区和侦察区 WGS84；
- 起飞/投放区航程/侦察高度、W/Z/I 模式、A/B 余弹、`dry_run`；
- 完整任务路线。

只按一次 Enter。Enter 前不切 `GUIDED`、不解锁、不调用起飞服务、不发布会让飞机
移动的 setpoint，也不打开 A/B 舵机。Enter 表示授权执行整个任务，不再输入
`YES`、`RTL` 或 `2`。

### 启动示例

Z 字搜索：

```bash
roslaunch cuadc_control auto_drop_test.launch area_search_pattern:=Z
```

自定义场地：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  drop_zone_distance_m:=28.0 \
  recon_zone_distance_m:=52.0 \
  area_search_pattern:=W
```

另一组测试场地：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  drop_zone_distance_m:=20.0 \
  recon_zone_distance_m:=38.0 \
  takeoff_altitude_m:=3.0 \
  recon_altitude_m:=5.0 \
  area_search_pattern:=Z
```

完整飞行 dry-run（仍起飞、航点、搜索、瞄准、侦察和 RTL，但不发舵机打开命令）：

```bash
roslaunch cuadc_control auto_drop_test.launch dry_run:=true
```

只使用 A：

```bash
roslaunch cuadc_control auto_drop_test.launch enable_b_dropper:=false
```

## 仅飞行航线验证模式

`flight_only_mode:=true` 复用同一个 `auto_drop_test.py` 的起点/航向冻结、WGS84
航点解算、自动起飞、全局航点、W/Z/I 点生成、遥测恢复、侦察区和 RTL，不新建第二份
飞行脚本。该模式只验证航线：不启动或等待 `camera_node`、`detector_node`、
`servo_test.py`，不订阅视觉目标或舵机 ready，不瞄准、不投放，且不会发布
`/servo/cmd`。即使同时传入 `auto_start_detector:=true`、
`auto_start_servo_test:=true` 或 `enable_b_dropper:=true`，节点也只在内部将其有效配置
强制为禁用，不修改 ROS 参数服务器中的原值；已经由外部启动的视觉或舵机节点不会
被停止，但本节点完全忽略它们。

默认路线：

```text
检查/按需启动 MAVROS
-> 等待飞控、GPS、本地位姿、速度、航向和 rel_alt
-> 冻结起点 WGS84、起点 NED 和启动航向
-> 解算投放区/侦察区 WGS84 航点并打印 FLIGHT ONLY 摘要
-> 等待一次 Enter
-> GUIDED -> 自动解锁 -> 起飞
-> 飞往投放区（不监控视觉目标）
-> 当前本地位置保持 -> 有界垂直移动到航线高度
-> 按冻结启动航向完整飞行指定圈数的 W、Z 或 I
-> 飞往侦察区并在途中爬升到侦察高度 -> 保持 -> RTL
```

若 `flight_only_continue_to_recon:=false`，区域航线完成后直接 RTL。Enter 前只完成依赖
检查、导航等待、起点冻结、航点解算和摘要打印，不切 `GUIDED`、不解锁、不起飞，
也不发布会移动飞机的 setpoint。

W 字航线：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  area_search_pattern:=W
```

Z 字航线：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  area_search_pattern:=Z
```

I 一字航线（也可写 `area_search_pattern:=LINE`）：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  area_search_pattern:=I
```

自定义夜间测试场地：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  drop_zone_distance_m:=20.0 \
  recon_zone_distance_m:=38.0 \
  takeoff_altitude_m:=3.0 \
  flight_only_search_altitude_m:=3.0 \
  recon_altitude_m:=5.0 \
  area_search_pattern:=W \
  flight_only_pattern_laps:=1
```

只测试投放区 Z 字航线，完成后直接 RTL：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  flight_only_continue_to_recon:=false \
  area_search_pattern:=Z
```

W/Z/I 航线中心取抵达投放区时的本地位置，长度方向使用 Enter 前冻结的启动航向；
每一圈必须完整通过全部路径点，新一圈回到第一个点，但不重新冻结中心或航向。各点
继续使用 `area_search_*` 尺寸、覆盖范围、水平/高度容差和连续停留参数。高度均以
解锁后、起飞前记录的 `ground_relative_altitude` 为基准：投放区全局航点使用
`drop_zone_transit_altitude_m`，区域航线使用 `flight_only_search_altitude_m`，侦察区使用
`recon_altitude_m`；D435 深度、目标 NED D 和 local ENU Z 不能替代相对地面高度。

仅飞行模式保留 state/pose/velocity/GPS/航向/rel_alt、连接和服务的有界恢复、
LOCAL_POSITION_NED 10Hz 请求、最后安全 setpoint、`CONTROL TRACE`、新鲜非
`GUIDED`/上锁/长时间断连安全退出、任务失败优先 RTL 和 Ctrl+C 空中保护。短暂
停更时保持最后安全 setpoint 并冻结路径点停留计时，超过原恢复宽限才进入
`FAILSAFE`。

夜间测试仍必须确认合法且清空的空域、区域航线全部点和 RTL 路径均在地图/场地边界内，
正确设置 Home，确认 GPS/EKF 健康，并保持遥控器可随时切走 `GUIDED` 接管。仅飞行
不等于降低真机飞行风险。

### 2026-07-22 W 航线实飞记录与下一次 Z 测试计划

一次 `flight_only_mode=true` 的 W 字航线实飞已完成，整体飞行效果良好，未报告
异常模式退出或航线失控。全程接近 3 分钟，主要耗时集中在投放区 W 字航线。当前
搜索区参数为 `8.0m x 5.0m`，扣除相机覆盖尺寸后，投放区中心到 W 第一点约
`3.37m`，从中心切入并完整通过 W 四点的本地航程约 `20.97m`。脚本默认
`max_horizontal_setpoint_rate_mps=0.25`，仅按 setpoint 斜坡计算的理想时间下限约
`83.9s`；实际还包含飞机跟随、转弯、容差保持和每点 `0.5s` 连续停留，因此搜索
耗时接近两分钟符合当前参数表现。抵达投放区中心后，第一个 W 点位于中心后方
`1.75m`、横向 `2.875m`，会出现约 `3.37m` 的低速折返/斜向切入。

本记录基于现场口述结果，未附带该次完整 `CONTROL TRACE` 或 `.tlog`，因此不记录
未经日志确认的实际速度、RC、消息年龄和当次距离覆盖值。当次测试时源码默认投放区
中心距离为 `35m`；若现场启动命令使用了 `drop_zone_distance_m:=20.0`，则当次实际值为 `20m`，
应以启动前 `FLIGHT ONLY 任务摘要` 或保存的 launch 命令为准。

下一次计划完整测试 1 圈 Z 字航线，把本地 setpoint 水平斜坡提高到 `0.5m/s`，保持
搜索区尺寸、路径点水平/高度容差和停留时间不变：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  area_search_pattern:=Z \
  flight_only_pattern_laps:=1 \
  max_horizontal_setpoint_rate_mps:=0.5
```

上述命令在当时未覆盖 `drop_zone_distance_m` 和 `recon_zone_distance_m`，因此使用
当时的默认 `35m/60m`。若现场已确认投放区中心应为 `20m`、侦察区为 `38m`，使用：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  area_search_pattern:=Z \
  flight_only_pattern_laps:=1 \
  max_horizontal_setpoint_rate_mps:=0.5 \
  drop_zone_distance_m:=20.0 \
  recon_zone_distance_m:=38.0
```

GUIDED 下存在两层水平速度限制：

- `max_horizontal_setpoint_rate_mps` 是本脚本发布本地 W/Z/I/瞄准位置 setpoint 时的
  水平斜坡速度；本次从 `0.25` 临时覆盖为 `0.5m/s`。这不是飞控参数。
- 本脚本的全局 WGS84 目标只发送位置，高度、速度和加速度字段均忽略；飞往投放区和
  侦察区时，ArduCopter GUIDED 位置控制的水平速度上限主要由 `WPNAV_SPEED` 决定，
  单位为 `cm/s`，例如 `200=2.0m/s`、`500=5.0m/s`。加减速和垂直速度还分别受
  `WPNAV_ACCEL`、`WPNAV_SPEED_UP`、`WPNAV_SPEED_DN` 等飞控参数约束。

可在 Mission Planner 的 `CONFIG/TUNING -> Full Parameter Tree` 中读取或修改
`WPNAV_SPEED`。也可在 MAVROS 连接后读取：

```bash
rosrun mavros mavparam get WPNAV_SPEED
```

如确需持久改为 `2.0m/s`，先确认单位和当前值后再执行：

```bash
rosrun mavros mavparam set WPNAV_SPEED 200
```

本次 W 实飞的全局航点段效果良好，因此下一次 Z 测试建议先不修改
`WPNAV_SPEED/WPNAV_ACCEL`，只用 launch 覆盖本地航线斜坡到 `0.5m/s`，这样可以把
变量限制在区域搜索速度，便于比较两次测试。

### 2026-07-22 Z 航线实飞结果与下一次 I/LINE 测试计划

使用 Z 字航线并把 `max_horizontal_setpoint_rate_mps` 覆盖为 `0.5m/s` 的测试结果
正常。现场确认原默认投放区中心距离 `35m` 偏远，因此从本次修改开始，脚本和 launch
的 `drop_zone_distance_m` 默认值统一改为 `32m`；侦察区仍为相对起点 `60m`，默认
两中心前向间隔变为约 `28m`。

新增 `I` 一字航线，`LINE` 是同义写法。飞机抵达投放区中心并完成高度切换后，按
起飞前冻结航向定义左右：先飞到左端，再横穿到右端。默认搜索区宽 `8.0m`、配置的
相机覆盖宽 `2.25m`，所以左右点分别位于中心横向 `-2.875m` 和 `+2.875m`；第一圈
从中心到左端约 `2.875m`，再到右端约 `5.75m`，合计约 `8.625m`。在 `0.5m/s`
setpoint 斜坡下，不计跟随、转弯和停留的理想时间约 `17.25s`。

I 航线只横穿 8m 宽度，不主动扫描 5m 长度方向。按当前配置的
`area_search_footprint_length_m=1.5`，仅从参数几何不能保证覆盖整个 5m 长度；4m
高度下真实相机视场、畸变、桶尺寸、光照和识别边缘效果需要天亮后实测，重点记录
左右边缘与长度前后端是否误检或漏检。时间紧张可用 I 快速横穿；时间充裕时优先使用
W（或按现场验证选择 Z）扫描航线，不应把 I 当作已经证明无漏检的完整覆盖方案。

新增 `drop_zone_transit_altitude_m`，独立设置起飞完成后前往投放区全局航点的相对
地面高度。默认值跟随 `takeoff_altitude_m`，因此不传新参数时仍保持原高度行为。
`flight_only_search_altitude_m` 继续独立控制抵达投放区后区域航线的高度。下一次
4m I 航线测试建议把航程和搜索高度都明确设为 4m，起飞高度仍保留默认 3m：

```bash
roslaunch cuadc_control auto_drop_test.launch \
  flight_only_mode:=true \
  area_search_pattern:=I \
  flight_only_pattern_laps:=1 \
  max_horizontal_setpoint_rate_mps:=0.5 \
  drop_zone_transit_altitude_m:=4.0 \
  flight_only_search_altitude_m:=4.0 \
  recon_altitude_m:=5.0
```

该命令使用新的默认投放区/侦察区距离 `32m/60m`。流程是离地 3m 起飞，在前往
32m 投放区过程中爬升到离地 4m，抵达后维持 4m 完成“中心 -> 左端 -> 右端”，随后
按默认设置前往 60m 侦察区并爬升到 5m，保持后 RTL。

## 顶层状态机和 setpoint 所有权

任务使用显式状态并在每次转换记录旧状态、新状态、原因、模式、连接/解锁状态、
NED/WGS84、rel_alt、A/B 余弹和最近安全 setpoint：

```text
INIT_DEPENDENCIES -> WAIT_MAVROS -> WAIT_NAVIGATION -> PLAN_WAYPOINTS
-> WAIT_ENTER -> GUIDED_ARM_TAKEOFF -> GOTO_DROP_ZONE
-> A_SEARCH/A_AIM/A_DROP -> RETURN_DROP_ZONE_FOR_B（按需）
-> B_SEARCH/B_AIM/B_DROP -> GOTO_RECON_CLIMB -> RECON_HOLD
-> RTL -> COMPLETE
```

错误进入 `FAILSAFE`。全局航点、本地保持、W/Z/I 搜索和瞄准都在顶层状态机中串行
调用，不存在两个后台循环并发发布全局/本地 setpoint。途中目标稳定触发时，本轮
不再发布全局目标，立即发布当前本地位置保持；确认本地保持开始后才进入 A 瞄准。

## W/Z/I 投放区搜索

搜索中心是抵达预设投放区时的本地位置；若 A 途中提前完成且 B 需要搜索，先返回
预设投放区中心。长度方向始终使用起飞前冻结的机头航向。路径点先在 NED 中构造，
发布 MAVROS 本地位置前显式转换：

```text
ENU.x = NED.e
ENU.y = NED.n
ENU.z = -NED.d
```

搜索保持进入搜索时的高度，不自动下降到 `1.5m` 投放高度。W/Z/I 航线一轮结束但
没有目标时会返回第一个路径点继续，直到对应搜索超时。搜索途中一旦目标连续有效，
立即停止路径推进并进入分层瞄准。

## A/B、目标冻结和投放规则

控制器内部维护：

```text
ammo_a = 1
ammo_b = 1（enable_b_dropper=false 时为 0）
completed_drop_targets = []
```

第一个合法目标给 A。A 命令发布后保存冻结目标 NED；B 默认拒绝距离任一已完成
目标小于 `0.60m` 的观测。若 30s 内仍无不同 B 目标，在飞控连接、armed、GUIDED、
安全保持 setpoint 和舵机链路均有效时强制发布 `B on`。显式设置
`enable_b_dropper:=false` 后，A 完成即进入侦察区。

远距离和框抖动阶段继续对目标 NED 做 `alpha=0.25` 低通滤波，并拒绝单帧水平跳变
超过 `0.80m` 的观测。当对应 A/B 的 X/Y 米制误差均小于 `0.20m`，且最近 `0.5s`
至少有 5 个样本、N/E 跨度各不超过 `0.10m`、D 跨度不超过 `0.15m` 时，对 N/E/D
分别取中位数形成 `frozen_target_ned`。日志记录样本数、跨度、原始值和冻结值。

冻结后，新的识别框轻微移动不再改写目标。短暂目标丢失时仍保持/飞向冻结 NED；
只有当前投放器流程结束、取消或切换到下一枚弹时才清除冻结值。

### 0.5s 正常投放

保留低空分层和原门槛：

```text
COARSE: 相对目标 2.0m
FINE  : 相对目标 1.7m
FINAL : 相对目标 1.5m
```

COARSE/FINE 阶段门槛连续满足 `0.5s` 才下一个高度。FINAL 同时满足 X、Y、高度、
水平速度 `<0.10m/s` 和垂直速度 `<0.10m/s`，并连续保持 `0.5s` 后立即发布当前
投放器命令。没有放宽位置、高度或速度门槛。

### 20s 强制投放

A/B 获得合法目标并进入 COARSE 后开始累计有效瞄准时间。目标短暂丢失或 state、
pose、velocity 恢复期间暂停，恢复后继续累计；轻微抖动不会把累计时间永久清零。
累计达到 `20s` 仍未正常满足全部门槛时，选择冻结或最后可信 NED，发布最后安全
本地保持，标记 `forced_drop=true` 并发布对应舵机打开命令。

强制投放不能绕过：飞控连接、`armed=true`、新鲜 `GUIDED`、安全保持 setpoint、
servo 节点/订阅者和 ROS 正常运行。新鲜 `ALT_HOLD/LOITER` 等非 GUIDED 状态表示
飞手接管，立即停止自动投放。

## 有界恢复和安全退出

以下短暂问题进入有界恢复，不因单帧事件退出：目标/深度/NED 丢失、检测跳变、
state/pose/velocity/GPS/航向/rel_alt 暂停、MAVROS 短暂断连、服务一次超时、自动
依赖进程退出、路径点短时间未进容差、稳定计时被轻微抖动打断。

空中恢复时持续发布最后安全 setpoint，冻结当前阶段和相关计时，不使用陈旧 pose
计算新修正。服务默认重试 3 次、退避 `0.5s`；自动依赖默认重启 3 次、退避
`2.0s`。恢复日志每秒给出原因、消息年龄、已等待时间和剩余宽限。

以下仍是真正安全退出/接管条件：

- 收到新鲜非 GUIDED 模式，特别是 RC 切到 ALT_HOLD/LOITER；
- `armed=false`；
- 断连超过 `10s`；
- state、pose、velocity 或导航数据超过各自恢复宽限；
- 没有最后安全 setpoint；
- 飞行前检查失败、起飞重试后仍未开始；
- servo 不可用且重启失败；
- RTL 请求失败且无法确认飞控状态；
- ROS shutdown/Ctrl+C。

空中任务级失败优先请求 RTL，RTL 失败再尝试 LAND；尚未确认离地时不无条件请求
RTL。收到新鲜非 GUIDED 表示飞手已接管，控制循环立即停止继续发任务 setpoint。

## 自动依赖启动

- MAVROS 不存在：按 `mavros_fcu_url` 启动一份；已存在则复用。
- camera/detector 都不存在：启动 `detector_node.launch start_camera:=true`。
- detector 存在、camera 不存在：只启动 `camera_node.launch`。
- camera 存在、detector 不存在：启动 detector，传 `start_camera:=false`。
- 自动启动 detector 时默认传 `show_window:=true`，OpenCV 画面必须打开。
- `servo_controller` 不存在：启动现有 `servo_test.py`；已存在则不重复。
- 自动启动的子进程在空中退出：保持最后安全位置并有界重启，不立即终止飞行。

## 坐标量不得混用

- `D435 CENTER Z`：深度图画面中心小区域的中位深度，是相机直接测得的场景距离。
- 检测框 `depth_m` / 画面 `z=`：目标框中心深度，用于反投影目标和瞄准点。
- MAVROS local ENU Z：`/mavros/local_position/pose.position.z`，本地坐标向上为正。
- 目标 NED D：视觉输出的绝对本地 NED 向下坐标，转换到 ENU 时取负号。

这些量语义不同，不能互相替代。当前飞行高度控制仍使用目标 NED D 和相对目标
阶段高度，不直接把 D435 CENTER Z 当作 local ENU Z。

## 舵机回执限制

`auto_drop_test.py` 只确认 `/servo/cmd` 有订阅者并完成 `A on` / `B on` 发布。
当前 `servo_test.py` 内部调用 `MAV_CMD_DO_SET_SERVO`，但没有把执行结果通过结构化
ROS 消息回传给任务控制器。因此日志只写“舵机命令已发布”，不能声称真实载荷已经
物理释放。`dry_run=true` 不发布真实打开命令。

## 主要参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `full_mission_mode` | `true` | 执行完整比赛任务 |
| `flight_only_mode` | `false` | 只验证完整航线，强制禁用视觉/舵机/瞄准/投放 |
| `flight_only_pattern_laps` | `1` | 投放区完整 W/Z/I 圈数，必须至少为 1 |
| `flight_only_search_altitude_m` | `3.0` | W/Z/I 相对地面高度，不得高于侦察高度 |
| `flight_only_continue_to_recon` | `true` | 区域航线后继续侦察区和 RTL；false 时直接 RTL |
| `takeoff_altitude_m` | `3.0` | 自动起飞目标离地高度 |
| `drop_zone_transit_altitude_m` | `takeoff_altitude_m` | 前往投放区全局航点的离地高度 |
| `drop_zone_distance_m` | `32.0` | 投放区相对起点总前向距离 |
| `recon_zone_distance_m` | `60.0` | 侦察区相对起点总前向距离 |
| `recon_altitude_m` | `5.0` | 侦察区离地高度 |
| `recon_hold_s` | `5.0` | 侦察区保持时间 |
| `waypoint_tolerance_m` | `1.0` | WGS84 水平到达容差 |
| `waypoint_vertical_tolerance_m` | `0.4` | 航点垂直容差 |
| `waypoint_arrival_hold_s` | `2.0` | 连续到达判定时间 |
| `area_search_pattern` | `W` | `W`、`Z`、`I`；`LINE` 等同 `I` |
| `area_search_width_m` | `8.0` | 搜索区横向宽度 |
| `area_search_length_m` | `5.0` | 搜索区启动航向长度 |
| `area_search_timeout_s` | `120.0` | A/普通区域搜索总时限 |
| `second_target_search_timeout_s` | `30.0` | B 不同目标搜索时限 |
| `area_search_waypoint_tolerance_m` | `0.25` | 搜索路径点水平容差 |
| `area_search_height_tolerance_m` | `0.20` | 搜索路径点高度容差 |
| `area_search_dwell_s` | `0.5` | 路径点连续停留 |
| `enroute_detect_stable_s` | `0.3` | 途中目标触发稳定时间 |
| `stable_time_s` | `0.5` | FINAL 全门槛连续时间 |
| `phase_stable_time_s` | `0.5` | COARSE/FINE 连续时间 |
| `aim_timeout_s` | `20.0` | 有效瞄准累计强制投放时间 |
| `target_lock_center_threshold_m` | `0.20` | 冻结前 X/Y 中心门槛 |
| `target_lock_window_s` | `0.5` | 冻结样本窗口 |
| `target_lock_min_samples` | `5` | 冻结最少样本数 |
| `target_lock_max_xy_span_m` | `0.10` | N/E 各自最大跨度 |
| `target_lock_max_d_span_m` | `0.15` | D 最大跨度 |
| `distinct_target_distance_m` | `0.60` | B 与已完成目标最小距离 |
| `auto_start_mavros` | `true` | 按需自动启动 MAVROS |
| `mavros_fcu_url` | `/dev/ttyACM0:115200` | 现场飞控地址 |
| `auto_start_detector` | `true` | 按需启动 camera/detector |
| `detector_show_window` | `true` | detector OpenCV 窗口 |
| `auto_start_servo_test` | `true` | 按需启动 servo_test.py |
| `dependency_restart_attempts` | `3` | 自动依赖重启次数 |
| `dependency_restart_backoff_s` | `2.0` | 依赖重启退避 |
| `detector_ready_timeout_s` | `30.0` | 视觉就绪等待 |
| `servo_ready_timeout_s` | `20.0` | 舵机就绪等待 |
| `state_timeout_s` | `2.0` | state 新鲜阈值 |
| `guided_state_grace_s` | `5.0` | state 总恢复宽限 |
| `pose_timeout_s` | `2.0` | pose/velocity 新鲜阈值 |
| `guided_pose_grace_s` | `5.0` | pose/velocity 总恢复宽限 |
| `navigation_timeout_s` | `2.0` | GPS/航向/rel_alt 新鲜阈值 |
| `navigation_grace_s` | `5.0` | 导航总恢复宽限 |
| `connection_loss_grace_s` | `10.0` | 飞控断连宽限 |
| `service_retry_attempts` | `3` | 模式/解锁/起飞服务重试 |
| `service_retry_backoff_s` | `0.5` | 服务重试退避 |
| `dry_run` | `false` | 完整飞行但不发舵机打开命令 |
| `ground_test` | `false` | 不实际起飞的拆桨地面测试 |

低空瞄准、滤波、局部重捕获和限幅参数仍在 launch 中完整暴露，包括
`coarse_align_height_m=2.0`、`fine_align_height_m=1.7`、
`target_drop_height_m=1.5`、`max_horizontal_speed_mps=0.10`、
`max_vertical_speed_mps=0.10`、`target_filter_alpha=0.25`、
`max_target_jump_m=0.80` 及 setpoint 距离/速率限制。

参数必须满足：

```text
drop_zone_distance_m > 0
recon_zone_distance_m > drop_zone_distance_m
takeoff_altitude_m > 0
drop_zone_transit_altitude_m > 0
recon_altitude_m >= takeoff_altitude_m
recon_altitude_m >= drop_zone_transit_altitude_m
flight_only_pattern_laps >= 1
0 < flight_only_search_altitude_m <= recon_altitude_m
```

## 基本排查

```bash
rostopic echo -n 1 /mavros/state
rostopic echo /vision/bucket/aim_info
rostopic info /servo/cmd
rosnode list
```

如 GUIDED 被拒绝，检查 GPS/EKF、Home、飞控模式配置和飞控日志；不要通过放宽脚本
门槛绕过飞控安全检查。

## 2026-07-21 全自动比赛任务改造记录

- 将 `auto_drop_test.py` 从手动起飞后的 W/Z 测试重构为单节点完整任务控制器；原
  `semi_auto_drop_test.py` 和原 WGS84 验证脚本未修改。
- 迁移双 WGS84 航点、地面 rel_alt 基准、GUIDED/解锁/起飞重试、全局相对高度
  setpoint、连续到达、RTL 回读及有界恢复逻辑。
- 新增一次 Enter 的完整任务授权语义，Enter 前不产生飞行动作或舵机打开动作。
- 新增飞往投放区途中稳定目标截获，并保证同一循环停止全局 setpoint 后才发布本地
  保持和进入 A 瞄准。
- W/Z 路径改为围绕预设投放区中心、按启动航向在 NED 中构造并显式转 ENU。
- 新增 A/B 余弹、不同目标黑名单、途中 A 后为 B 返回预设投放区中心、B 30s 搜索
  超时强制命令、侦察区 5m 爬升/保持和 RTL。
- 正常阶段/最终稳定时间改为 `0.5s`，原位置、高度、速度门槛不放宽。
- 新增 NED 中位数冻结窗口和有效瞄准累计 `20s` 强制投放；强制投放仍必须满足连接、
  armed、GUIDED、安全保持和舵机链路条件。
- 新增 MAVROS/视觉/舵机按需启动、OpenCV 窗口默认开启、子进程有界重启，以及
  state/pose/velocity/导航/连接的有界恢复日志。
- 明确区分 D435 CENTER Z、检测框深度、local ENU Z 和目标 NED D；明确当前舵机
  接口只能确认命令发布，不能确认真实载荷物理释放。

## 2026-07-22 仅飞行航线验证模式修改记录

- 在现有 `auto_drop_test.py` 顶层状态机中新增独立 `FLIGHT_ONLY_PATTERN` 分支，默认
  完整飞行 1 圈 W/Z/I 后前往侦察区、爬升、保持并 RTL；可选择完成区域航线后直接 RTL。
- 新增 `flight_only_mode`、圈数、航线高度和是否继续侦察区四个参数，并同步脚本
  fallback 默认值与 launch 默认值及参数约束。
- flight-only 运行时强制禁用视觉、舵机、A/B 瞄准、途中截获和投放；不检查、启动、
  等待或重启 camera/detector/servo 分支，外部视觉消息也不会改变航线。
- 新增纯航线 W/Z/I 执行器：抵达投放区后冻结中心，使用起飞前启动航向，先本地保持并
  有界调整到相对地面航线高度，再按水平/高度容差和连续停留要求完整通过全部路径点。
- flight-only 的每秒日志新增 pattern、圈/点、目标/当前 ENU、水平/垂直误差、
  state/pose/velocity 年龄、模式和 RC；状态转换同时记录当前安全 setpoint 与圈/点。
- 保留既有全局/本地 setpoint 串行所有权、LOCAL_POSITION_NED 请求、遥测/连接/服务
  有界恢复、任务失败优先 RTL 和飞手接管安全退出。
- 根据 Z 字 `0.5m/s` 实飞正常结果，将投放区默认前向距离从 `35m` 调整为 `32m`。
- 新增 `I`/`LINE` 一字航线：从投放区中心先到有效宽度左端，再横穿到右端；端点按
  `area_search_width_m` 和 `area_search_footprint_width_m` 自动计算。
- 新增 `drop_zone_transit_altitude_m`，把自动起飞高度、前往投放区航程高度和
  flight-only 区域航线高度拆分为可独立设置的三层高度。
- 记录下一次白天 4m I 航线验证命令，并明确 I 不扫描 5m 长度方向；时间充裕优先
  使用 W/Z 扫描航线评估漏检。
