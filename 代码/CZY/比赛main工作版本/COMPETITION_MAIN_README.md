# GDPI_CUADC_2026 比赛 main 工作版本

本目录是从 `代码/NUC上面的测试功能包（不可以随便修改,可以复制里面的内容）` 复制出来的比赛工作版本。原 NUC 测试功能包未修改。

## 新增/修改文件

- `src/cuadc_vision/scripts/competition_mission_common.py`
  - 复用 MAVROS 的 GUIDED/ARM/TAKEOFF/RTL、local/global setpoint、`MAV_CMD_DO_SET_SERVO` 调用方式。
  - 提供目标多帧跟踪、滑动中值锁定、A/B 弹药与抛投器补偿。
- `src/cuadc_vision/scripts/competition_main.py`
  - 比赛主状态机：起飞、投放区航线搜索、多桶稳定选择、瞄准、投放、侦察、RTL。
- `src/cuadc_vision/scripts/single_bucket_aim_test.py`
  - 真机单桶瞄准测试脚本：不自动起飞，不自动 RTL，完成后可切 LOITER。
- `src/cuadc_vision/launch/competition_main.launch`
  - 主任务启动文件，默认危险动作关闭。
- `src/cuadc_vision/launch/single_bucket_aim_test.launch`
  - 单桶瞄准测试启动文件，默认 dry-run。
- `src/cuadc_vision/CMakeLists.txt`
  - 将新增 Python 脚本加入 catkin install。

## 主状态机流程

`INIT -> WAIT_READY -> PREARM -> TAKEOFF -> GOTO_DROP_ZONE -> DROP_ZONE_SEARCH -> TARGET_SELECT -> AIMING -> DROP -> POST_DROP`

若仍有弹药，`POST_DROP` 回到 `DROP_ZONE_SEARCH`。A/B 弹药为 0 后进入：

`GOTO_RECON_ZONE -> RECON_OBSERVE -> RTL -> COMPLETE`

异常进入 `FAILSAFE` 或 `ABORT`。空中优先 RTL，RTL 失败再 LAND；未起飞/地面测试不发送起飞和实投动作。

状态发布：

- `/mission/main_status`：JSON 字符串，包含 main 状态、飞控模式、armed、A/B 弹药、当前抛投器、锁定目标、搜索/瞄准/投放/侦察阶段标志。
- `/vision/mission_status`：复用 detector_node 已有叠加显示，发布弹药、aiming、A/B DROP 显示。

## 单桶测试命令

先启动 MAVROS、相机、detector_node，或让 launch 同时启动视觉节点。

只做 dry-run，不自动切 GUIDED：

```bash
roslaunch cuadc_vision single_bucket_aim_test.launch dry_run:=true auto_guided:=false test_dropper:=A
```

允许脚本自动切 GUIDED，但不打开舵机：

```bash
roslaunch cuadc_vision single_bucket_aim_test.launch dry_run:=true auto_guided:=true test_dropper:=A
```

实投一次 A 抛投器：

```bash
roslaunch cuadc_vision single_bucket_aim_test.launch dry_run:=false enable_servo_drop:=true auto_guided:=true test_dropper:=A
```

测试完成后，若当前为 GUIDED，脚本默认切 LOITER，飞手手动接管降落。

## 主任务启动命令

安全地面检查，不起飞、不实投：

```bash
roslaunch cuadc_vision competition_main.launch ground_test:=true enable_takeoff:=false enable_servo_drop:=false
```

真实比赛流程，飞手输入授权短语后自动起飞，舵机仍 dry-run：

```bash
roslaunch cuadc_vision competition_main.launch enable_takeoff:=true enable_servo_drop:=false
```

真实比赛流程并允许舵机实投：

```bash
roslaunch cuadc_vision competition_main.launch enable_takeoff:=true enable_servo_drop:=true
```

## NUC 验证步骤

1. `catkin_make` 或 `catkin build` 编译 `代码/比赛main工作版本` 的 catkin 工作区。
2. `source devel/setup.bash`。
3. 启动 MAVROS，确认 `/mavros/state` connected。
4. 启动相机和 `detector_node.py`，确认：
   - `/vision/yolo/detections`
   - `/vision/yolo/detection`
   - `/vision/target_global`
   - `/vision/bucket/info`
5. 先运行单桶 dry-run，确认日志出现 `DROP CONDITION SATISFIED`，飞机对准过程稳定。
6. 再把 `dry_run:=false enable_servo_drop:=true` 做一次低风险实投验证。
7. 最后运行 `competition_main.launch`，先 `enable_servo_drop:=false` 跑完整流程，再打开实投。

## 关键参数

- `enable_takeoff`：主任务是否允许自动 GUIDED/ARM/TAKEOFF，默认 `false`。
- `require_authorization` / `authorization_phrase`：飞手终端授权确认，默认需要输入 `YES`。
- `ground_test`：地面测试模式，不发送起飞、setpoint、实投。
- `enable_servo_drop`：是否真的发送 `MAV_CMD_DO_SET_SERVO`，默认 `false`。
- `ammo_a` / `ammo_b`：A/B 初始弹药。
- `dropper_a_forward_offset_m` / `dropper_b_forward_offset_m`：A/B 相对相机的机体 X 前向补偿，默认 `+0.05` / `-0.05`。
- `target_min_hits`：目标连续稳定帧数，默认 `3`。
- `target_reject_jump_m`：锁定目标跳变拒绝阈值。
- `aiming_threshold_m`：投放水平误差阈值。
- `aiming_stable_time_s`：满足阈值后的稳定持续时间。
- `drop_zone_entry_enu` / `search_waypoints_enu` / `recon_zone_enu`：相对启动点的 ENU 航点。
- `use_wgs84_drop_zone` / `drop_zone_lat` / `drop_zone_lon` / `drop_zone_rel_alt`：可选 WGS84 投放区入口。
- `use_wgs84_recon_zone` / `recon_zone_lat` / `recon_zone_lon` / `recon_zone_rel_alt`：可选 WGS84 侦察区。

## 风险点和待实测项

- A/B 前后 5cm 补偿按 detector_node 现有映射处理：`X_body = -Y_cam`，机体 X 为前。仍需实测确认 D435i 实际安装方向与代码注释一致。
- 主状态机固定锁定目标 ENU 后不逐帧追检测框；若 GPS/local pose 漂移大，锁定点会受影响。
- 默认 3 个搜索航点覆盖 8m x 5m，只是初值；应按实测 YOLO 高度漏检率调整 `takeoff_altitude` 和 `search_waypoints_enu`。
- WGS84 区域入口只负责飞到入口点，搜索航线仍建议使用 local ENU 扫描点，避免多套坐标混用。
- `enable_servo_drop` 默认关闭，实投前必须单独验证舵机通道、PWM 开关方向和保持时间。
