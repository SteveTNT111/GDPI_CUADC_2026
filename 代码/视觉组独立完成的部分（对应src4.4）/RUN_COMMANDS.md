# D435i 视觉检测 —— 常用运行命令

> 适用于 `视觉组独立完成的部分/d435i_yellow_circle_detector`
> 系统：Ubuntu 20.04 + ROS Noetic
> 工作空间路径：`~/uav_ws`

---

## 环境准备

每次新开终端先 source：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

---

## 一、比赛实战命令

### 1.1 完整模式：YOLO + geopose（推荐）

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true \
  enable_global_target:=true \
  yolo_model_path:=/home/lab/model/best.pt
```

启动节点：相机 + YOLO检测 + 大地坐标变换
输出话题：`/yolo/detection`、`/yolo/detections`、`/competition/target_global`

### 1.2 完整模式 + 比赛任务仲裁

```bash
# 投放区
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true \
  enable_global_target:=true \
  enable_competition_task:=true \
  mission_stage:=drop \
  yolo_model_path:=/home/lab/model/best.pt

# 灾情侦察区
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true \
  enable_global_target:=true \
  enable_competition_task:=true \
  mission_stage:=disaster \
  yolo_model_path:=/home/lab/model/best.pt
```

输出话题：`/competition/target`（结构化的比赛目标）

---

## 二、调试命令

### 2.1 地面调试：YOLO + 窗口显示 + geopose

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  show_window:=true \
  enable_yolo:=true \
  enable_global_target:=true \
  yolo_model_path:=/home/lab/model/best.pt
```

YOLO 窗口会叠加显示目标的 GPS 经纬高和机体坐标。

### 2.2 仅 YOLO（无 geopose，无窗口）

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true \
  yolo_model_path:=/home/lab/model/best.pt
```

### 2.3 仅 YOLO + 窗口（无 geopose）

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  show_window:=true \
  enable_yolo:=true \
  yolo_model_path:=/home/lab/model/best.pt
```

---

## 三、数据采集命令

### 3.1 空中采集（关窗口，低负载）

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true \
  enable_record:=true \
  yolo_model_path:=/home/lab/model/best.pt
```

### 3.2 地面调试 + 采集

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  show_window:=true \
  enable_yolo:=true \
  enable_record:=true \
  yolo_model_path:=/home/lab/model/best.pt
```

### 3.3 自定义采集参数

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_record:=true \
  record_dir:=/home/lab/dataset \
  record_fps:=15 \
  image_format:=jpg \
  jpeg_quality:=95
```

默认保存目录：`~/yellow_circle_dataset/YYYYMMDD_HHMMSS/images/`

---

## 四、自定义参数

### 改 YOLO 参数

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true \
  yolo_conf_threshold:=0.7 \
  yolo_imgsz:=320 \
  yolo_device:=cpu
```

### 改相机帧率

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true \
  fps:=15
```

### 改 geopose 坐标帧

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true \
  enable_global_target:=true \
  body_frame:=base_link \
  camera_frame:=d435i_color_optical_frame
```

---

## 五、话题验证

启动后检查话题是否正常：

```bash
# 相机话题
rostopic list | grep d435i

# YOLO 话题
rostopic list | grep yolo
rostopic echo /yolo/detection        # 最高置信度目标
rostopic echo /yolo/detections       # 全部目标

# geopose 输出
rostopic echo /competition/target_global

# 比赛任务输出
rostopic echo /competition/target
```

### 手动测试 geopose 话题

```bash
# 发布 fake GPS（测试用）
rostopic pub /mavros/global_position/global sensor_msgs/NavSatFix \
  "header: {frame_id: 'map'}
  status: {status: 0, service: 1}
  latitude: 31.0
  longitude: 121.0
  altitude: 100.0"

# 发布 fake 位姿
rostopic pub /mavros/local_position/pose geometry_msgs/PoseStamped \
  "header: {frame_id: 'map'}
  pose: {position: {x: 0, y: 0, z: -100}, orientation: {x: 0, y: 0, z: 0, w: 1}}"
```

---

## 六、完整参数速查

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  show_window:=false \              # 是否显示 OpenCV 窗口
  fps:=30 \                         # 相机帧率
  enable_record:=false \            # 是否保存图片
  enable_yolo:=false \              # 是否启动 YOLO
  yolo_model_path:=/home/lab/model/best.pt \
  yolo_conf_threshold:=0.5 \        # YOLO 置信度阈值
  yolo_imgsz:=640 \                 # YOLO 输入尺寸
  yolo_device:=cpu \                # 推理设备 (cpu / cuda:0)
  enable_global_target:=false \     # 是否启动 geopose 坐标变换
  enable_competition_task:=false \  # 是否启动比赛任务仲裁
  mission_stage:=drop               # 比赛阶段: drop / disaster
```
