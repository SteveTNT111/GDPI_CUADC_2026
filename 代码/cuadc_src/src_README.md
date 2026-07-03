# CUADC Vision — 主功能包

> **包名：** `cuadc_vision`（ROS Noetic）
> **用途：** CUADC 2026 全部飞行代码——飞行控制、视觉检测、坐标变换、舵机投放、飞行录像
> **维护：** 伍尚京

---

## 文件结构

```
cuadc_src/
├── scripts/                                   # Python 节点（7个）
│   ├── main.py                                #   主控：状态机 + 模式切换 + 起飞
│   ├── servo_test.py                          #   舵机测试：终端 on/off 控制 CH5/CH6
│   ├── camera_node.py                         #   D435i 相机驱动
│   ├── detector_node.py                       #   YOLO 目标检测
│   ├── geopose_node.py                        #   坐标变换（相机→机体→ENU→WGS84）
│   ├── flight_data_video_recorder_node.py     #   飞行数据录像（解锁自动录）
│   └── auto_drop_node.py                      #   自动抛投触发
├── launch/                                    # 启动文件（5个）
│   ├── run_main.launch                        #   总启动：主控 + 相机 + 检测 + geopose
│   ├── cuadc_run.launch                       #   视觉管线：相机 + 检测 + geopose
│   ├── run_servo_test.launch                  #   舵机测试终端
│   ├── run_flight_recorder.launch             #   飞行数据录像
│   └── auto_drop.launch                       #   自动抛投
├── config/
│   └── params.yaml                            #   全局参数
├── msg/
│   ├── GeoTarget.msg                          #   大地坐标目标
│   ├── YoloDetection.msg                      #   单目标检测
│   └── YoloDetections.msg                     #   全部检测
├── models/
├── CMakeLists.txt
└── package.xml
```

---

## 依赖安装

### 系统依赖

```bash
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-mavros-msgs \
  ros-noetic-mavros
```

### Python 依赖

```bash
pip3 install pyrealsense2 opencv-python geographiclib ultralytics
```

### 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x ~/catkin_ws/src/cuadc_src/scripts/*.py
```

---

## 节点与启动命令

### 1. main.py — 飞行主控

状态机驱动，通过 MAVROS 切换飞行模式、解锁、起飞。

流程：`INIT → PREARM → ARMED → TAKEOFF → HOLD → MISSION → LAND/RTL`

```bash
# 手动模式
roslaunch cuadc_vision run_main.launch

# 自动起飞
roslaunch cuadc_vision run_main.launch auto_arm:=true auto_takeoff:=true

# 自定义高度
roslaunch cuadc_vision run_main.launch takeoff_altitude:=15.0
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `takeoff_altitude` | 10.0 | 起飞高度 (m) |
| `auto_arm` | false | 自动解锁 |
| `auto_takeoff` | false | 自动起飞 |

---

### 2. servo_test.py — 舵机测试

MAVROS RC Override 控制 CH5/CH6，PWM 1100=关闭 / 1400=打开。

```bash
# 终端交互（输入 on/off 控制）
roslaunch cuadc_vision run_servo_test.launch

# 双通道
roslaunch cuadc_vision run_servo_test.launch enable_ch6:=true
```

ROS 话题控制（另开终端）：

```bash
rostopic pub /servo/cmd std_msgs/String "data: 'on'"
rostopic pub /servo/cmd std_msgs/String "data: 'off'"
```

---

### 3. camera_node.py — D435i 相机驱动

```bash
roslaunch cuadc_vision cuadc_run.launch          # 30FPS
roslaunch cuadc_vision cuadc_run.launch fps:=15  # 降帧率
```

发布：`/d435i/color/image_raw`、`/d435i/aligned_depth/image_raw`、`/d435i/color/camera_info`

---

### 4. detector_node.py — YOLO 检测

```bash
# CPU 推理（模型已放在 models/ 目录则无需指定路径）
roslaunch cuadc_vision cuadc_run.launch enable_yolo:=true

# GPU + 调阈值
roslaunch cuadc_vision cuadc_run.launch enable_yolo:=true yolo_device:=cuda:0 yolo_conf_threshold:=0.7

# 自定义模型路径
roslaunch cuadc_vision cuadc_run.launch enable_yolo:=true yolo_model_path:=/home/lab/my_model.pt
```

发布：`/yolo/detection`（最佳）、`/yolo/detections`（全部）、`/yolo/annotated_image`

---

### 5. geopose_node.py — 大地坐标变换

相机系 → 机体 → ENU → WGS84 经纬高。需要 MAVROS 运行。

```bash
roslaunch cuadc_vision cuadc_run.launch enable_yolo:=true enable_geopose:=true
```

输出：`/competition/target_global` (GeoTarget)

---

### 6. flight_data_video_recorder_node.py — 飞行数据录像

解锁自动录，画面叠加高度/GPS/电压，上锁自动停。输出 .avi + .csv。

```bash
roslaunch cuadc_vision run_flight_recorder.launch                    # 正常录制
roslaunch cuadc_vision run_flight_recorder.launch show_window:=true  # 开窗口
roslaunch cuadc_vision run_flight_recorder.launch record_immediately:=true  # 不等解锁
```

输出目录：`~/cuadc_flight_videos/YYYYMMDD_HHMMSS/`

---

### 7. auto_drop_node.py — 自动抛投

检测目标对准相机光心后，自动触发舵机释放。

```bash
roslaunch cuadc_vision auto_drop.launch
roslaunch cuadc_vision auto_drop.launch pixel_threshold:=30.0 min_conf:=0.7
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pixel_threshold` | 20.0 | 对准像素阈值 |
| `min_conf` | 0.5 | 最低置信度 |
| `channel` | 9 | 舵机通道号 |
| `release_pwm` | 1900 | 释放 PWM |
| `reset_pwm` | 1100 | 复位 PWM |

---

## 启动文件总览

| 文件 | 包含节点 | 用途 |
|------|---------|------|
| `run_main.launch` | main + camera + detector + geopose | 总启动 |
| `cuadc_run.launch` | camera + detector + geopose | 仅视觉 |
| `run_servo_test.launch` | servo_test | 舵机测试 |
| `run_flight_recorder.launch` | camera + flight_data_video_recorder | 飞行录像 |
| `auto_drop.launch` | detector + auto_drop | 自动抛投 |

---

## 比赛完整启动

```bash
# 1. 启动 MAVROS
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600

# 2. 启动总控
roslaunch cuadc_vision run_main.launch auto_arm:=false auto_takeoff:=false

# 3（可选）. 自动抛投
roslaunch cuadc_vision auto_drop.launch

# 4（可选）. 飞行录像
roslaunch cuadc_vision run_flight_recorder.launch
```

---

## ROS 话题速查

| 节点 | 话题 | 用途 |
|------|------|------|
| camera_node | `/d435i/color/image_raw` | 彩色图 |
| camera_node | `/d435i/aligned_depth/image_raw` | 深度图 |
| camera_node | `/d435i/color/camera_info` | 相机内参 |
| detector_node | `/yolo/detection` | 最佳目标 |
| detector_node | `/yolo/detections` | 全部目标 |
| geopose_node | `/competition/target_global` | 大地坐标 |
| servo_test | `/servo/cmd` | 舵机指令 |

```bash
# 验证话题
rostopic list | grep -E "d435i|yolo|competition|servo"
rostopic echo /competition/target_global
```

---

## 常见问题

**D435i 不识别：** 插 USB 3.0 蓝色口，`realsense-viewer` 确认画面。

**YOLO 报错：** `pip3 install ultralytics torch`

**TF 报错：** MAVROS 是否在运行？`rostopic list | grep mavros`

**坐标不准：** 检查 `invert_camera_x`，镜头朝下设 `true`。

**舵机不动：** 检查飞控通道映射，确认 CH5/CH6 配置为 servo。

**录像卡顿：** 降帧率 `video_fps:=15.0`，或换 codec `codec:=XVID`。
