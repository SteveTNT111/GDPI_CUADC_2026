# 自主搜寻与双抛投任务（独立新目录）

本目录没有修改项目原有文件。任务坐标采用已经实测确认的：`+X 向右、+Y 向前、+Z 向上`。

第一次使用请先阅读：[快速启动指南](快速启动指南.md)。

## 完整流程

```text
等待 MAVROS、相机、检测节点
→ 人工口令授权
→ GUIDED、解锁、起飞到 2.5 m
→ 搜寻模式：保持 2.5 m，沿 +Y 以 0.20 m/s 的参考速度前飞，最多 3 m
   ├─ 发现稳定目标：立即进入瞄准模式
   └─ 前飞 3 m 仍无目标：原地升至 3.0 m → RTL → 返回起点并由飞控降落
→ 瞄准模式：下降至 1.2 m，同时持续修正 X/Y
→ 高度约 1.2 m，目标 X/Y 各在 0±0.20 m 内连续保持 2 秒
→ A 抛投器动作
→ 飞机沿 +Y 再前移 0.08 m，位置和高度稳定 2 秒
→ B 抛投器动作
→ 原地升至 3.0 m
→ RTL → 返回起点并由飞控降落
```

搜索速度使用任务文档建议的 `0.10–0.20 m/s` 范围上限 `0.20 m/s`。代码不是一次发送 3 m 远的位置点，而是按时间逐步推进位置参考点，因此参考轨迹沿 `+Y` 以 0.20 m/s 移动，并持续保持 2.5 m 高度。

## 瞄准和投放限制

- 下降过程中一直根据目标位置修正 X/Y。
- 高度未进入 `1.2±0.15 m` 时，A 不允许投放。
- X 和 Y 任一方向超过 `±0.20 m`，2 秒计时立即清零。
- A 投放后不再重新搜索目标，而是使用本地位置向前移动 0.08 m。
- B 投放前要求 0.08 m 位移目标和 1.2 m 高度稳定 2 秒。
- 当前暂定 A=CH5、B=CH6；这只是占位映射，确认接线后再修改。

## 坐标换算

现有检测节点的 `invert_camera_x` 配置与部分文档符号存在冲突。任务节点使用检测框像素、相机内参和深度直接计算：

```text
目标向右 = (u - cx) × depth / fx
目标向前 = -(v - cy) × depth / fy
```

因此画面右侧对应飞机 `+X`，画面上方对应飞机 `+Y`。任务期间保持起飞时偏航；偏航超出默认 12° 会终止视觉控制。

## 文件

- [任务主程序](scripts/autonomous_two_drop_mission.py)
- [参数配置](config/mission.yaml)
- [启动文件](launch/autonomous_two_drop_mission.launch)
- [待测数据](MEASUREMENTS.md)
- [几何测试](tests/test_mission_geometry.py)

## 安全默认值

以下条件没有全部满足时，节点不会解锁：

- `execute_mission=true`
- `calibration_confirmed=true`
- `enable_servo_drop=true`，或明确启用空投放演练
- MAVROS、本地位置、相机内参和检测话题全部正常
- 操作者输入授权口令 `FLY`

如果遥控器将模式切出 GUIDED，节点立即停止发送位置目标，不会强行抢回控制权。

## 安全预览

不启动飞控和相机，只验证配置与节点：

```bash
roslaunch cuadc_vision autonomous_two_drop_mission.launch \
  start_mavros:=false start_vision:=false
```

运行几何测试：

```bash
python3 $(rospack find cuadc_vision)/autonomous_drop_mission/tests/test_mission_geometry.py
```

## 启动示例

真实舵机通道和 PWM 确认前，不能直接实飞。参数确认后的形式如下：

```bash
roslaunch cuadc_vision autonomous_two_drop_mission.launch \
  execute_mission:=true \
  calibration_confirmed:=true \
  enable_servo_drop:=true \
  takeoff_altitude_m:=2.5 \
  search_forward_distance_m:=3.0 \
  search_forward_speed_mps:=0.20 \
  aim_altitude_m:=1.2 \
  rtl_climb_altitude_m:=3.0 \
  b_forward_shift_m:=0.08
```

## 输出话题

| 话题 | 内容 |
|---|---|
| `/autonomous_drop/status` | 当前模式、抛投器、目标、对准误差和失败原因 |
| `/autonomous_drop/target_distance_m` | 飞机参考点到目标的三维距离 |
| `/autonomous_drop/target_offset_xyz` | 目标相对飞机的 X右/Y前/Z上偏移 |
| `/vision/mission_status` | 给现有检测画面显示 AIMING、A/B DROP 和余量 |

## 地面和低速测试顺序

1. 拆桨验证目标在画面右侧时 X 为正、画面上方时 Y 为正。
2. 用卷尺校验 D435i 深度和三维距离。
3. 单独确认 CH5、CH6 分别连接哪个抛投器，以及开关 PWM。
4. 不挂载物体，验证 2.5 m 起飞、+Y 前飞 3 m、无目标升至 3 m 和 RTL。
5. 开启空投放演练，验证下降修正、2 秒计时、A、前移 0.08 m、B、升高和 RTL。
6. 最后才启用真实舵机投放。
