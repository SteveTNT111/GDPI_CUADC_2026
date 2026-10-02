# CUADC 2026 — Gazebo 仿真手把手操作教程

> 环境组成（三个进程，缺一不可）：
> | 进程 | 作用 | 端口/话题 |
> |---|---|---|
> | **GZ server** (`gz sim -s -r`) | 3D 物理仿真（飞机本体+场地） | GZ transport |
> | **SITL** (`arducopter --model JSON`) | 飞控逻辑（接管 IMU/GPS，算控制） | TCP 5760 |
> | **MAVROS** | ROS 与飞控之间的"翻译官" | `/mavros/*` 话题/服务 |

---

## 0. 快速检查环境是否在跑

```bash
ps -eo pid,cmd | grep -E "[g]z sim|[a]rducopter|[m]avros_node" | awk '{print $1, $2}'
```
应看到 3 个进程。缺哪个就重启哪个（见第 6 节）。

---

## 1. 打开 3D 画面

```bash
gz sim -g
```
- **左键拖动**：旋转视角
- **右键拖动**：平移
- **滚轮**：缩放
- 看飞机：左侧模型列表 `iris_demo`，双击可跟随
- 找不到飞机：先确认它是否已起飞（rel_alt > 0），或按 `Ctrl+Shift+P` 重置视角

---

## 2. 请求数据流（每次重启 SITL 后必做！）

SITL 默认不主动推 GPS/姿态数据，要让 MAVROS 话题有数据：

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_CUADC2026/install/setup.bash
python3 ~/ROS2_CUADC2026/scripts/request_streams.py
```
看到 8 个 ✅ 就对了。

---

## 3. 看飞机状态（核心观察命令）

```bash
# 连接/模式/解锁状态
ros2 topic echo /mavros/state --once

# GPS 位置（起飞后会变化）
ros2 topic echo /mavros/global_position/global --once

# 离地高度（起飞后应为正数）
ros2 topic echo /mavros/global_position/rel_alt --once

# 航向角
ros2 topic echo /mavros/global_position/compass_hdg --once

# 数据刷新频率（应 ~10Hz）
ros2 topic hz /mavros/local_position/pose
```

---

## 4. 手动控制飞机（起飞 → 悬停 → 降落）

一条一条复制到终端（每行之间留 2~3 秒）：

```bash
# ① 切到 GUIDED 模式
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: GUIDED}"

# ② 解锁（ARM）—— 这时螺旋桨开始转
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# ③ 起飞到 3 米（稍等，飞5~10秒）
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 3.0}"

# ④ 看它是不是飞起来了
ros2 topic echo /mavros/global_position/rel_alt --once

# ⑤ 降落（自己下来）
ros2 service call /mavros/cmd/land mavros_msgs/srv/CommandTOL "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 0.0}"
```
**提示**：如果对不上服务类型，先跑 `ros2 interface show mavros_msgs/srv/CommandTOL` 看字段。

---

## 5. 一键跑完整验证（连接→导航→起飞→舵机→降落）

```bash
source /opt/ros/humble/setup.bash
source ~/ROS2_CUADC2026/install/setup.bash
python3 ~/ROS2_CUADC2026/scripts/verify_a2.py --takeoff-alt 2.5
```
全部 ✅ 后飞机自动降回地面。注意它会动舵机（CH5），只是 SITL 里没有实体舵机，看服务返回 `success=True` 即可。

---

## 6. 重启整个环境（一键）

```bash
bash ~/ROS2_CUADC2026/scripts/restart_sim.sh
```
脚本自动：清旧进程 → 起 GZ → 起 SITL → 起 MAVROS → 等连接。
重启后**必须**再做第 2 步请求数据流。

> 若只用 `restart_sim.sh` 起 GZ 时已经开了 GUI 窗口，画面会断，重新 `gz sim -g` 即可。

---

## 7. 常见问题排查

| 症状 | 原因 | 解决 |
|---|---|---|
| `ros2 topic echo /mavros/state` 超时 | MAVROS 没连上 SITL | 看 `/tmp/opencode/mavros.log`，重跑 restart_sim.sh |
| GPS/位置话题一直没数据 | 没请求数据流 | 跑 `request_streams.py`（第 2 步） |
| 飞机不动/高度为 0 | 没解锁或没切 GUIDED | 按第 4 节 ①② 顺序来 |
| ARM 提示 rejected | SITL 预检不通过 | 等 EKF 收敛（启动后 15~30 秒），再看 `ros2 topic echo /mavros/state` 的 system_status |
| 画面黑屏 | 无头 server 没连上 GUI | `gz sim -g` 重开；确认 `ps` 里有 GZ server |
| 想重置飞机位置 | — | 重启整个环境（第 6 节） |

---

## 8. 日志在哪

- GZ: `/tmp/opencode/gz_server.log`
- SITL: `/tmp/opencode/sitl.log`
- MAVROS: `/tmp/opencode/mavros.log`

看某类错误：`grep -aE "GPS|EKF|ERROR" /tmp/opencode/mavros.log | tail -20`