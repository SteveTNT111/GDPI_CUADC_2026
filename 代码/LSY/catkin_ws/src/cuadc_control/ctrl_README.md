# CUADC Control — 控制验证包

> ROS 包名：`cuadc_control`（ROS Noetic）  
> 用途：存放 CUADC 2026 真机/SITL 控制验证脚本；视觉识别与消息定义位于
> `cuadc_vision`。

## 文件结构

```text
cuadc_control/
├── scripts/
│   ├── one_key_takeoff.py
│   ├── one_key_takeoff_forward_land.py
│   ├── one_key_takeoff_wgs84_forward_rtl.py
│   └── semi_auto_drop_test.py
├── launch/
│   ├── one_key_takeoff.launch
│   ├── one_key_takeoff_forward_land.launch
│   ├── one_key_takeoff_wgs84_forward_rtl.launch
│   └── semi_auto_drop_test.launch
├── CMakeLists.txt
├── package.xml
└── ctrl_README.md
```

## 编译

```bash
cd ~/catkin_ws
catkin_make
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
chmod +x ~/catkin_ws/src/cuadc_control/scripts/*.py
```

## 现有控制脚本

### `one_key_takeoff.py`

自动检查飞控链路并执行：

```text
GUIDED -> ARM -> TAKEOFF -> LOITER
```

### `one_key_takeoff_forward_land.py`

要求操作者事先手动解锁，执行起飞、沿当前航向前飞和降落验证。脚本不会调用
`/mavros/cmd/arming`。

```bash
roslaunch cuadc_control one_key_takeoff_forward_land.launch
```

### `one_key_takeoff_wgs84_forward_rtl.py`

读取本地位置、WGS84、航向和相对 Home 高度，经人工授权后验证 WGS84 航点、
RTL/LAND 流程。该脚本包含自动解锁和起飞逻辑，只能按对应测试规程运行。

```bash
roslaunch cuadc_control one_key_takeoff_wgs84_forward_rtl.launch
```

## 半自动瞄准与投放测试

脚本和 launch 已统一命名：

```text
cuadc_control/scripts/semi_auto_drop_test.py
cuadc_control/launch/semi_auto_drop_test.launch
```

### 职责边界

半自动脚本不自动解锁、不自动起飞、不调用底层
`MAV_CMD_DO_SET_SERVO`。飞手先手动把飞机飞到桶附近并保持 `LOITER`，随后脚本：

```text
检查/启动 camera_node 与 detector_node
-> 检查/启动 servo_test.py
-> 等待 /vision/bucket/aim_info
-> 识别到桶后等待 Enter
-> 检查 connected、armed、LOITER 和 NED 有效性
-> GUIDED（回读 /mavros/state 确认）
-> 锁定接管时的高度和航向，不自动升降
-> NED 转 ENU，以最大 0.05m/s 仅水平向桶心移动
-> 短暂丢帧立即冻结悬停点；10 帧后判丢，原地等待重获 4 秒
-> 水平米制偏差和投放高度连续满足阈值
-> 通过 /servo/cmd 发布 "A on"
-> 发布 MissionStatus.last_drop="A"
-> LOITER，把控制权交还飞手
```

当 `enable_b_dropper:=true` 时，脚本只请求一次 `GUIDED`。A 完成后保持
`GUIDED` 等待 3 秒，直接使用 B 瞄准点继续对准并投放；只在 A/B 全部
完成后才切回 `LOITER`。A 到 B 之间不重置目标丢失状态，仍然连续使用
`10 帧判丢 + 4 秒重获`容错。

`ground_test:=true` 专用于拆桨室内测试：忽略 armed 和当前模式，不要求定位或
NED，不切 GUIDED、不发布飞行 setpoint；测试人员用手移动飞机完成视觉对准后，
脚本仍会通过 `/servo/cmd` 控制真实 A/B 舵机。启动地面模式本身即视为操作者
授权，因此识别到有效桶目标后不会等待 Enter：默认从 A 开始，X/Y 米制偏差均
小于阈值并连续稳定指定时间后自动打开舵机。飞控和 MAVROS 仍需保持连接。

### 坐标与判定

视觉节点发布 `cuadc_vision/BucketAimInfo`：

```text
/vision/bucket/aim_info
```

脚本使用对应抛投器的绝对 NED 目标点计算水平桶心。进入
`GUIDED` 时立即锁定当前 ENU Z 高度和航向，不使用视觉结果下发垂直移动。
飞手应在按 Enter 前手动稳定到计划投放高度。水平坐标发布到
`/mavros/setpoint_position/local` 前转换为 MAVROS ENU：

```text
ENU.x = NED.e
ENU.y = NED.n
ENU.z = 接管时当前 ENU.z（全程保持）
```

默认每 `0.1s` 发布一次 setpoint，虚拟位置目标每秒最多水平移动
`0.05m`。以下条件连续满足 `2.0s` 为瞄准完成：

```text
abs(delta_x_m) < 0.15
abs(delta_y_m) < 0.15
abs(实际高度 - target_drop_height_m) < 0.15
```

### 自动启动规则

- `detector_node` 和 `camera_node` 都已运行：不重复启动。
- detector 未运行：通过 `cuadc_vision/detector_node.launch` 拉起；相机已运行时
  自动传入 `start_camera:=false`。
- detector 已运行但 camera 未运行：通过 `camera_node.launch` 只启动相机。
- `servo_controller` 未运行：使用现有 `cuadc_vision/servo_test.py` 启动舵机节点。
- 半自动脚本等待 `/servo/ready=true`，确认舵机节点完成启动复位后才开始瞄准计时。
- 可用 `auto_start_detector:=false` 或 `auto_start_servo_test:=false` 禁用自动启动。

`servo_test.py` 是唯一实现 `MAV_CMD_DO_SET_SERVO` 的节点。半自动脚本只向
`/servo/cmd` 发布 `A on/A off/B on/B off`。

### 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `ground_test` | `false` | true=未解锁地面手持瞄准和真实舵机测试 |
| `enable_b_dropper` | `false` | false=只投 A；true=继续投 B |
| `align_threshold_m` | `0.15` | X/Y 米制对准阈值 |
| `target_drop_height_m` | `1.5` | 相对识别到的桶顶/地面目标点的投放高度 |
| `height_tolerance_m` | `0.15` | 投放高度允许误差 |
| `stable_time_s` | `2.0` | 连续对准时间 |
| `setpoint_interval_s` | `0.1` | setpoint 发布间隔 |
| `max_horizontal_speed_mps` | `0.05` | 自主水平对中的最大目标移动速度 |
| `max_target_offset_m` | `2.0` | 识别目标水平跳变安全上限，超限即退出 |
| `max_surface_z_drift_m` | `0.50` | 桶面高度估计漂移上限，超限即退出 |
| `lateral_divergence_growth_m` | `0.10` | 水平二维误差在监测窗口内的基础发散阈值 |
| `lateral_divergence_window_s` | `3.0` | 横向误差发散监测窗口；退出后切回 LOITER |
| `command_error_buffer_m` | `0.03` | 5cm/s 指令误差缓冲；同时用于发散阈值和对准释放滞回，不允许小于3cm |
| `after_a_delay_s` | `3.0` | A 完成后到 B 开始前的等待时间 |
| `auto_start_detector` | `true` | 自动检查并拉起视觉节点 |
| `auto_start_servo_test` | `true` | 自动检查并拉起舵机节点 |
| `close_after_drop` | `false` | 投放后是否自动发布对应 `off` |
| `dry_run` | `false` | true=不发 `/servo/cmd`，只打印计划动作 |
| `min_bucket_count` | `1` | 至少检测到多少个桶才提示 Enter |
| `lost_frame_threshold` | `10` | 连续多少帧目标无效后正式判定丢失 |
| `target_reacquire_timeout_s` | `4.0` | 正式丢失后等待重新找到目标的时间 |
| `vision_frame_rate_hz` | `30.0` | 将丢失帧数换算成时间阈值时使用的视频帧率 |

单桶现场保持 `min_bucket_count:=1`。如果现场明确要求两个以上桶才提示，可设：

```bash
roslaunch cuadc_control semi_auto_drop_test.launch min_bucket_count:=2
```

### 启动命令

只测试 A（默认）：

```bash
roslaunch cuadc_control semi_auto_drop_test.launch
```

启用 A+B：

```bash
roslaunch cuadc_control semi_auto_drop_test.launch enable_b_dropper:=true
```

建议空中 A+B 测试使用专用 launch。它默认 `dry_run:=true`：

```bash
roslaunch cuadc_control semi_auto_ab_drop_test.launch
```

确认空中 dry-run 的 `LOITER -> GUIDED -> A -> B -> LOITER` 全流程正常后，
才允许真实投放：

```bash
roslaunch cuadc_control semi_auto_ab_drop_test.launch \
  dry_run:=false \
  auto_start_servo_test:=true
```

专用 launch 默认不启动舵机节点，因此默认 dry-run 不会因舵机节点启动
复位而产生真实舵机动作。真实投放必须显式传入上面两个参数。

Dry-run（不发布任何舵机命令）：

```bash
roslaunch cuadc_control semi_auto_drop_test.launch dry_run:=true
```

拆桨、未解锁的室内手持对准容错测试（默认 dry-run，30 FPS）：

```bash
roslaunch cuadc_control semi_auto_drop_ground_test.launch
```

该专用 launch 默认 `stable_time_s:=30.0`，方便先测试遮挡、恢复和超时，
避免持续对准 2 秒后过早完成 dry-run。如需测试完整的对准投放流程，可显式传入
`stable_time_s:=2.0`。

如果视觉和相机已经另行启动，也可以直接启动通用 launch：

```bash
roslaunch cuadc_control semi_auto_drop_test.launch \
  ground_test:=true \
  enable_b_dropper:=true \
  lost_frame_threshold:=10 \
  target_reacquire_timeout_s:=4.0 \
  align_threshold_m:=0.02
```

确认已经拆桨且需要测试真实舵机时，才使用：

```bash
roslaunch cuadc_control semi_auto_drop_servo_ground_test.launch
```

该专用 launch 会自动启动舵机节点，默认只测试 A，且不要求按 Enter。
X/Y 米制误差均小于 `0.02m` 并连续稳定 `2s` 后，真实发布 `A on`；
`1s` 后发布 `A off`，视觉窗口显示红色 `A DROP!!!` 约 `3s`。
手持移动造成偏差超限、检测丢帧或 `valid` 暂时变为 false 时，脚本会立即清零
本次投放稳定计时。连续无效未达 `10` 帧时只视为短暂丢帧；第 `10` 帧起正式
判定目标丢失并开始 `4.0s` 重获倒计。从首个无效结果起，脚本就会
冻结当前水平位置、高度和航向，以 `10Hz` 持续发布悬停点。倒计内恢复
则从当前悬停点继续以 `0.05m/s` 对中，并从 `0s` 重新累计稳定时间；
超过 `4.0s` 仍未恢复则停止 `AIMING!!` 显示并结束地面流程，用于模拟交还飞手。
在 30 FPS 下，10 帧约等于 `0.33s`。由于 CPU YOLO 的实际输出帧率可能低于
相机帧率，程序使用“连续 10 个无效结果”或“从首个无效结果起持续 `0.33s`”
中先到的条件判定目标丢失。

### 飞手现场操作

1. 检查载荷、舵机、遥控器模式开关、GPS/EKF 和 MAVROS 连接。
2. 手动解锁并起飞；脚本不会替飞手解锁或起飞。
3. 手动飞到桶附近，切换并稳定保持 `LOITER`。
4. 观察识别画面和终端。确认飞机已经由飞手稳定在计划投放高度，再按 Enter。
5. 按 Enter 授权短时 `GUIDED`；随时准备用遥控器接管。
6. 启用 A+B 时，A 投放后飞机仍保持 `GUIDED`，脚本自动切换到 B
   瞄准点。只有 A/B 都完成后才请求 `LOITER`。
7. 确认终端回读模式为 `LOITER` 后，再由飞手继续控制。

### 安全行为

- `armed == false`、未收到新鲜 aim info、目标 NED 无效或 GUIDED 回读确认失败时，
  空中模式不发送投放命令；`ground_test:=true` 是唯一允许未解锁真实舵机测试的
  显式例外。
- 地面模式不切换飞行模式、不要求 NED、不发送任何飞行 setpoint。
- 空中模式按 Enter 后锁定当前高度与航向，只做水平对中，不自动升降。
- `dry_run:=true` 仅禁止真实投放伺服输出，仍会切换 `GUIDED` 并进行水平飞行，不是地面预览。
- 瞄准期间目标数据丢失、模式丢失或出现异常时，优先请求 `LOITER`，不请求 RTL。
- Ctrl+C/ROS shutdown 时，如果脚本曾请求 GUIDED，会尝试切回 `LOITER`。
- `close_after_drop` 默认关闭，因此默认不会自动复位已打开的舵机。
- 空中测试时不要同时运行其他会发布本地位置 setpoint 或切换飞行模式的任务节点。

## MAVROS 控制链路

本包通过 ROS/MAVROS 与飞控通信，不向 MAVProxy 终端发送字符串命令：

```text
ROS 脚本 -> MAVROS service/topic -> MAVLink -> 飞控或 SITL
```

半自动脚本使用：

- `/mavros/set_mode`
- `/mavros/state`
- `/mavros/setpoint_position/local`
- `/vision/bucket/aim_info`
- `/vision/mission_status`
- `/servo/cmd`

## 基本排查

```bash
rostopic echo -n 1 /mavros/state
rostopic echo /vision/bucket/aim_info
rostopic info /servo/cmd
rosnode list
```

如 `GUIDED` 被飞控拒绝，先检查 GPS/EKF、Home、模式配置和飞控日志；不要通过修改
脚本绕过飞控安全检查。真机飞行前应先完成拆桨地面检查和 SITL 验证。
