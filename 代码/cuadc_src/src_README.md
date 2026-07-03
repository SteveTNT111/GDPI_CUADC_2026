# CUADC Vision — 主功能包

> **包名：** `cuadc_vision`（ROS Noetic）
> **用途：** CUADC 2026 全部飞行代码——飞行控制、视觉检测、坐标变换、舵机投放、飞行录像
> **维护：** 伍尚京
>
> ⚠️ **本文档所有命令都是在 NUC（Ubuntu 20.04）的终端里执行的，不是在 Windows 上执行。**
> 在自己 Windows 电脑上只需用 VS Code 编辑代码 + Git 提交，不需要装 ROS、不需要跑任何命令。

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
├── launch/                                    # 启动文件（6个，每个 .py 一个同名 launch）
│   ├── camera_node.launch                     #   D435i 相机（配合 rviz 查看画面）
│   ├── detector_node.launch                   #   YOLO 检测（相机 + 检测 + OpenCV 窗口）
│   ├── geopose_node.launch                    #   大地坐标变换（待创建）
│   ├── run_main.launch                        #   总启动：主控 + 相机 + 检测 + geopose
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

| 库 | 用途（本项目中） | 学习资源 |
|---|---|---|
| **pyrealsense2** | D435i 相机驱动，获取彩色/深度图像流（`camera_node.py`） | [官方文档](https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html) · [GitHub](https://github.com/IntelRealSense/librealsense) · [SDK 下载](https://www.intelrealsense.com/sdk-2/) |
| **opencv-python** | 图像处理——录像叠加 OS、画面标注、图像编解码（`flight_data_video_recorder_node.py`、`detector_node.py`） | [官方文档](https://docs.opencv.org/) · [PyPI](https://pypi.org/project/opencv-python/) · [GitHub](https://github.com/opencv/opencv-python) |
| **geographiclib** | 大地测量——WGS84 椭球面上的正反算、相机→机体→ENU→经纬高的坐标变换（`geopose_node.py`） | [官方文档](https://geographiclib.sourceforge.io/Python/doc/) · [PyPI](https://pypi.org/project/geographiclib/) · [GitHub](https://github.com/geographiclib/geographiclib-python) |
| **ultralytics** | YOLO 目标检测——模型加载、推理、结果解析（`detector_node.py`） | [官方文档](https://docs.ultralytics.com/) · [PyPI](https://pypi.org/project/ultralytics/) · [GitHub](https://github.com/ultralytics/ultralytics) |

### 编译

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x ~/catkin_ws/src/cuadc_src/scripts/*.py
```

---

## 节点详解

> 每个节点都配好了 launch 文件，直接用 `roslaunch` 一行命令启动，不需要手动 `rosrun`。

---

### 1. main.py — 飞行主控

**启动文件：** `run_main.launch`

**功能：** 通过 MAVROS 控制无人机完成全自动飞行流程。内部是一个状态机，依次执行：等待连接 → 检查 EKF/GPS → 切换 GUIDED 模式 → 解锁 → 起飞 → 悬停待命 → 执行任务 → 着陆/返航。

**测试什么：** 验证飞控与机载电脑的 MAVROS 通信是否正常，验证 GUIDED 模式下起飞→悬停→着陆的完整链路。

**启动后你可以：**
- 手动模式（默认）：启动后终端会打印当前状态，用遥控器或 QGC 手动解锁起飞，main.py 会跟随状态变化
- 自动模式：加 `auto_arm:=true auto_takeoff:=true`，启动后无人机会自动完成解锁→起飞→悬停，无需遥控器干预

**启动命令：**

```bash
# ① 手动模式——安全第一，调试用
roslaunch cuadc_vision run_main.launch
# 等价于 auto_arm:=false auto_takeoff:=false
# 启动后什么都不会自动发生，需要你用遥控器操作
# 终端会打印：等待飞控连接... → 飞控已连接 → 等待 EKF 收敛... → 等待手动解锁...
```

```bash
# ② 自动起飞——比赛用
roslaunch cuadc_vision run_main.launch auto_arm:=true auto_takeoff:=true
# 启动后自动切 GUIDED → 解锁 → 起飞到默认 10m → 悬停
# 终端会打印每一步的进度
```

```bash
# ③ 自动起飞 + 指定高度
roslaunch cuadc_vision run_main.launch auto_arm:=true auto_takeoff:=true takeoff_altitude:=15.0
```

**依赖：** 必须先启动 MAVROS（`roslaunch mavros apm.launch`），飞控已连接且 GPS 有 fix。

**验证是否正常：** 终端应依次打印 `飞控已连接 → EKF 就绪 → 解锁成功 → 起飞指令已发送 → 到达目标高度`。如果反复打印 `等待飞控连接...`，说明 MAVROS 没启动或飞控没插好。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `takeoff_altitude` | 10.0 | 起飞目标高度 (m) |
| `auto_arm` | false | true = 启动后自动解锁 |
| `auto_takeoff` | false | true = 解锁后自动起飞 |

---

### 2. servo_test.py — 舵机测试

**启动文件：** `run_servo_test.launch`

**功能：** 通过 MAVLink `MAV_CMD_DO_SET_SERVO` 指令直接控制飞控舵机输出引脚（SERVO5），不经过飞控逻辑（无需设置 SERVOx_FUNCTION）。单舵机联动双抛投器：PWM 1000 = 全关，1500 = 仅第一个开，2000 = 全开。

**飞控参数要求：** `SERVO5_FUNCTION` / `SERVO6_FUNCTION` 保持 **0 (Disabled)**，飞控不碰这个引脚。`RC_OVERRIDE_TIME` 必须 > 0（默认 3 即可），设成 0 会拒绝所有外部指令。

**测试什么：** 验证舵机接线是否正确、PWM 值是否能驱动抛投器、机械结构是否顺畅。**不需要飞机起飞，地上就能测。**

**启动后你可以：**
- 在终端输入 `on` → 舵机转到 PWM 2000，两个抛投器都打开
- 在终端输入 `off` → 舵机转到 PWM 1000，两个抛投器都关闭
- 输入 `q` 退出
- 也可以通过 ROS 话题远程控制（见下方）

**启动命令：**

```bash
# ① 默认：只控制 CH5（一个舵机驱动两侧抛投器）
roslaunch cuadc_vision run_servo_test.launch
# 启动后终端显示 "servo>" 提示符，输入 on/off 控制
```

```bash
# ② 双通道独立控制（CH5 左 + CH6 右）
roslaunch cuadc_vision run_servo_test.launch enable_ch6:=true
```

```bash
# ③ 通过 ROS 话题控制（另开终端，无需交互）
rostopic pub /servo/cmd std_msgs/String "data: 'on'"
rostopic pub /servo/cmd std_msgs/String "data: 'off'"
```

**验证是否正常：** 输入 `on` 后舵机应转动并保持，终端打印 `舵机 CH5 → 打开 (PWM=1400)`。舵机不动则检查飞控是否上电、通道映射是否正确。

---

### 3. camera_node.py — D435i 相机驱动

**启动文件：** `camera_node.launch`

**功能：** 打开 Intel RealSense D435i 深度相机，发布彩色图像、对齐后的深度图像、相机内参。

**测试什么：** 验证 D435i 在 NUC 上能否正常出图。配合 `rqt_image_view` 或 `rviz` 查看画面。

**启动后你可以：**
- 在 rviz 里 Add → By topic → `/vision/color/image_raw` 看彩色画面
- 终端日志确认 USB 类型（usb=3.2 正常，usb=2.1 有问题）

**启动命令：**

```bash
# 打开相机，用 rviz 看画面
roslaunch cuadc_vision camera_node.launch
# 另开终端：rviz
rviz

# 或用 rqt 快速看
rosrun rqt_image_view rqt_image_view /vision/color/image_raw
```

```bash
# USB 不稳定时降帧率
roslaunch cuadc_vision camera_node.launch fps:=15
```

**发布话题：**

| 话题 | 类型 | 用途 |
|------|------|------|
| `/vision/color/image_raw` | sensor_msgs/Image | 彩色图（640×480 bgr8） |
| `/vision/aligned_depth/image_raw` | sensor_msgs/Image | 深度图（已对齐到彩色） |
| `/vision/color/camera_info` | sensor_msgs/CameraInfo | 相机内参矩阵 |

**验证是否正常：**

```bash
rostopic hz /vision/color/image_raw   # 应该有稳定 30Hz 输出
# 终端日志应打印：D435i started. 640x480 bgr8@30
# 如果打印 usb=2.1 则说明 USB 工作在 2.0 模式，需换 USB3 口或线
```

---

---

### 4. detector_node.py — YOLO 目标检测

**启动文件：** `detector_node.launch`

**功能：** 一键启动 D435i 相机 + YOLO 检测，弹出 OpenCV 窗口显示实时标注画面。加载 YOLOv8 模型逐帧推理，发布检测结果。

**测试什么：** 验证模型能否正确识别圆筒、置信度是否够高、检测帧率是否满足实时要求。

**启动后你可以：**
- 自动弹出 `YOLO Detection` 窗口，实时看到检测框和距离标注
- 用 `rostopic echo /yolo/detection` 查看检测数据
- 拿着圆筒在相机前移动，观察检测框是否跟随

**启动命令：**

```bash
# ① 首次测试——CPU 推理
roslaunch cuadc_vision detector_node.launch

# ② GPU 推理 + 提高阈值
roslaunch cuadc_vision detector_node.launch yolo_device:=cuda:0 yolo_conf_threshold:=0.7

# ③ 自定义模型
roslaunch cuadc_vision detector_node.launch yolo_model_path:=/home/lab/my_model.pt
```

**发布话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vision/yolo/detection` | YoloDetection | 当前帧最佳目标 |
| `/vision/yolo/detections` | YoloDetections | 当前帧全部目标 |
| `/vision/annotated_image` | sensor_msgs/Image | 标注画面 |

**验证是否正常：** 窗口弹出且画面流畅，终端打印 `Detector started. model=...`，拿着圆筒在镜头前目标框跟随移动。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `yolo_device` | cpu | cpu / cuda:0 |
| `yolo_conf_threshold` | 0.5 | 低于此值的目标不发布 |
| `yolo_imgsz` | 640 | 推理分辨率，越小越快 |

---

### 5. geopose_node.py — 大地坐标变换

**启动文件：** `cuadc_run.launch`（加 `enable_geopose:=true`）

**功能：** 把 YOLO 检测到的目标从相机坐标系一路变换到 WGS84 大地坐标（经纬度 + 海拔）。变换链：相机系 → tf2 → 机体坐标系 → 四元数旋转 → ENU（东北天）→ geographiclib → 经纬高。

**测试什么：** 验证坐标变换链路是否完整——把目标放到已知 GPS 位置，看输出的经纬度是否接近。

**启动后你可以：**
- 用 `rostopic echo /competition/target_global` 查看大地坐标
- 结合 QGC 地图验证坐标点是否落在正确位置
- 飞控收到坐标后可以直接导航到目标上空

**启动命令：**

```bash
# 一条命令启动 camera + YOLO + geopose
roslaunch cuadc_vision cuadc_run.launch enable_yolo:=true enable_geopose:=true
```

**依赖：** 必须 MAVROS 在运行（提供 `/mavros/global_position/global` 和 `/mavros/local_position/pose`）。

**输出话题：** `/competition/target_global`（类型 GeoTarget，包含 latitude / longitude / altitude）

**验证是否正常：**

```bash
rostopic echo /competition/target_global
# 看到 latitude / longitude / altitude 有具体数值且 status="ok" 即正常
# 如果 status="no_valid_global_position"，说明 MAVROS 没提供 GPS 数据
```

---

### 6. flight_data_video_recorder_node.py — 飞行数据录像

**启动文件：** `run_flight_recorder.launch`

**功能：** 飞控解锁后自动开始录像，画面左下角叠加实时飞行数据（相对高度、GPS 坐标、电压、电流、飞行模式）。上锁后自动停止。同时输出 .avi 视频和 .csv 数据表。

**测试什么：** 不用起飞也能测——加 `record_immediately:=true` 立即录像，验证画面叠加文字是否清晰、CSV 数据是否完整。**真正的用途是飞行后回放视频，分析不同高度下相机能拍到多大范围、什么高度识别圆筒效果最好。**

**启动后你可以：**
- 地面测试：`record_immediately:=true show_window:=true` 立即看到叠加画面
- 实际飞行：正常启动，解锁自动录，降落自动停
- 拿 CSV 数据在 Excel 里画高度-时间曲线

**启动命令：**

**① 实际飞行用**——解锁自动开始，上锁自动停止：

```bash
roslaunch cuadc_vision run_flight_recorder.launch
```

**② 地面调试**——开窗口看叠加效果：

```bash
roslaunch cuadc_vision run_flight_recorder.launch show_window:=true
```

**③ 不等解锁，立即开始录**（纯地面测试）：

```bash
roslaunch cuadc_vision run_flight_recorder.launch record_immediately:=true show_window:=true
```

**输出：** `~/cuadc_flight_videos/YYYYMMDD_HHMMSS/cuadc_flight_YYYYMMDD_HHMMSS.avi` + `.csv`

**CSV 列：** frame, ros_time, armed, mode, latitude, longitude, global_alt_m, relative_alt_m, local_x/y/z_m, voltage_v, current_a, battery_percent

**验证是否正常：** 终端打印 `Flight video recording started` 且输出目录出现 .avi 文件。打开视频应看到相机画面 + 左上角叠加的飞行数据文字。

---

### 7. auto_drop_node.py — 自动抛投

**启动文件：** `auto_drop.launch`

**功能：** 监听 YOLO 检测结果，当检测到的目标中心与相机光心的像素偏差小于阈值时，自动通过 MAVROS 发送舵机指令触发抛投。释放后保持一段时间再复位，并设冷却时间防止重复触发。

**测试什么：** 验证"对准即投"逻辑——把圆筒放在相机视野中央，观察是否自动触发舵机。**地面就能测，不需要起飞。**

**启动后你可以：**
- 启动节点 + YOLO 检测
- 把圆筒移到画面中央
- 观察终端是否打印 `triggering drop`
- 听舵机动作声音确认触发

**启动命令：**

**① 默认参数**：

```bash
roslaunch cuadc_vision auto_drop.launch
```

**② 放宽对准要求 + 提高置信度**：

```bash
roslaunch cuadc_vision auto_drop.launch pixel_threshold:=30.0 min_conf:=0.7
```

**工作流程：** 检测到目标 → 像素偏差 ≤ pixel_threshold → 置信度 ≥ min_conf → 距上次抛投 ≥ cooldown → 舵机转到 release_pwm → 保持 hold_seconds → 舵机回到 reset_pwm

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pixel_threshold` | 20.0 | 对准容差 (px)，越小对准越严格 |
| `min_conf` | 0.5 | 最低检测置信度 |
| `channel` | 9 | 舵机通道号 |
| `release_pwm` | 1900 | 释放时的 PWM 值 |
| `reset_pwm` | 1100 | 复位时的 PWM 值 |
| `hold_seconds` | 0.8 | 释放保持时间 (s) |
| `cooldown` | 2.0 | 两次抛投最小间隔 (s) |

**验证是否正常：** 确保已启动 YOLO 检测（`cuadc_run.launch enable_yolo:=true`），圆筒移到画面中央时终端应打印 `triggering drop`，舵机应动作。

---

## 启动文件总览

| 文件 | 包含节点 | 用途 |
|------|---------|------|
| `camera_node.launch` | camera | 相机驱动（配合 rviz） |
| `detector_node.launch` | camera + detector | 相机 + YOLO + OpenCV 窗口 |
| `run_main.launch` | main + camera + detector + geopose | 总启动 |
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

**catkin_make 报 ddynamic_reconfigure 错误：**
```
CMake Error: Project 'ddynamic_reconfigure' specifies ...
'.../ddynamic_reconfigure-kinetic-devel/include' as an include dir, which is not found.
```
原因：工作空间 `~/catkin_ws/src/` 里混入了 `realsense-ros` 和 `ddynamic_reconfigure-kinetic-devel` 这两个无关包。`cuadc_vision` 用的是 Python 库 `pyrealsense2`，不需要 ROS 的 `realsense2_camera` 包。

```bash
# 删掉无关包
rm -rf ~/catkin_ws/src/realsense-ros
rm -rf ~/catkin_ws/src/ddynamic_reconfigure-kinetic-devel
cd ~/catkin_ws && catkin_make
```

**D435i 不识别：** 插 USB 3.0 蓝色口，`realsense-viewer` 确认画面。

**YOLO 报错：** `pip3 install ultralytics torch`

**TF 报错：** MAVROS 是否在运行？`rostopic list | grep mavros`

**坐标不准：** 检查 `invert_camera_x`，镜头朝下设 `true`。

**舵机不动：** 检查飞控通道映射，确认 CH5/CH6 配置为 servo。

**录像卡顿：** 降帧率 `video_fps:=15.0`，或换 codec `codec:=XVID`。

**main.py 反复报 `set_mode 服务调用失败` / `connection abort`：**
这是因为 MAVROS 还没启动，main.py 连不上飞控。先开一个终端启动 MAVROS：
```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
```
等连上飞控后再启动 `run_main.launch`。或者先把 main.py 的终端 `Ctrl+C` 关掉。

**roslaunch 报 `is neither a launch file in package`：**
说明 ROS 找不到 launch 文件。常见原因：
1. 新开终端没 source 环境变量
2. 添加新文件后没重新 `catkin_make`

```bash
# 检查 launch 文件是否存在
ls ~/catkin_ws/src/cuadc_src/launch/

# 重新编译 + source（每次新终端都要 source）
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 验证包是否被 ROS 识别
rospack find cuadc_vision
# 应输出：/home/lab/catkin_ws/src/cuadc_src
```
