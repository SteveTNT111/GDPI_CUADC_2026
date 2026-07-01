# CUADC Vision

比赛视觉系统：D435i 相机 + YOLO 检测 + 目标大地坐标解算。

## 文件结构

```
cuadc_vision/
├── launch/cuadc_vision.launch   ← 一键启动
├── config/params.yaml           ← 参数
├── msg/                         ← 自定义消息（3个）
├── scripts/
│   ├── camera_node.py           ← D435i 相机驱动
│   ├── detector_node.py         ← YOLO 检测
│   └── geopose_node.py          ← 大地坐标解算
├── CMakeLists.txt
└── package.xml
```

## 依赖

```bash
sudo apt install -y ros-noetic-cv-bridge ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs ros-noetic-mavros
pip3 install pyrealsense2 opencv-python geographiclib ultralytics
```

## 部署

```bash
cp -r cuadc_vision ~/catkin_ws/src/
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

## 启动

```bash
# 仅相机+检测
roslaunch cuadc_vision cuadc_vision.launch

# 相机+检测+大地坐标（比赛模式，需 MAVROS 运行）
roslaunch cuadc_vision cuadc_vision.launch enable_geopose:=true
```

## 话题接口

| 节点 | 输出话题 | 类型 |
|------|---------|------|
| camera_node | `/vision/color/image_raw` | sensor_msgs/Image |
| camera_node | `/vision/aligned_depth/image_raw` | sensor_msgs/Image |
| camera_node | `/vision/color/camera_info` | sensor_msgs/CameraInfo |
| detector_node | `/vision/yolo/detection` | YoloDetection（最佳检测） |
| detector_node | `/vision/yolo/detections` | YoloDetections（全部检测） |
| detector_node | `/vision/annotated_image` | sensor_msgs/Image（标注图） |
| geopose_node | `/vision/target_global` | GeoTarget（经纬度） |

### geopose_node 输入（来自 MAVROS）

| 话题 | 用途 |
|------|------|
| `/mavros/global_position/global` | 飞机 GPS |
| `/mavros/local_position/pose` | 飞机 ENU 位姿 |

## 常见问题

**D435i 不识别**：插 USB 3.0 口（蓝色），运行 `realsense-viewer` 确认。

**YOLO 导入失败**：`pip3 install ultralytics torch`

**TF 报错**：确认 MAVROS 运行中，`rosrun tf view_frames` 检查 TF 树。

**坐标不准**：检查 `invert_camera_x` 参数，镜头朝下时通常设为 `true`。
