# Ubuntu 20.04 + ROS Noetic D435/D435i 黄色圆检测与图片数据集采集说明

本项目是 **ROS1 / ROS Noetic** 代码，不是 ROS2。Ubuntu 工作空间固定使用：

```text
~/uav_ws
```

源码包目录：

```text
~/uav_ws/src/d435i_yellow_circle_detector
```

当前默认参数：

```text
相机分辨率: 640x480
默认帧率: 30 FPS
检测方法: HSV 黄色过滤 -> 二值化 -> 轮廓/圆度筛选
显示窗口: 默认关闭
数据采集: 默认关闭, 开启后只保存图片, 不保存视频
```

## 1. 功能节点

- `d435i_camera_node.py`: 使用 `pyrealsense2` 打开 D435/D435i, 发布彩色图和对齐后的深度图。
- `yellow_binarizer_node.py`: 对彩色图做黄色 HSV 过滤, 发布黑白二值图。
- `yellow_circle_detector_node.py`: 在二值图中找轮廓并筛选圆, 发布检测结果和标注图。
- `annotated_image_viewer_node.py`: 显示带检测标注的彩色 OpenCV 窗口。
- `binary_image_viewer_node.py`: 显示黑白二值化 OpenCV 窗口。
- `dataset_image_recorder_node.py`: 采集图片数据集, 只保存照片, 不保存视频。

## 2. 创建工作空间并编译

第一次在 Ubuntu 20.04 上使用：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/uav_ws/src
cp -r /path/to/CUADC/src/d435i_yellow_circle_detector ~/uav_ws/src/
cd ~/uav_ws
catkin_make
source ~/uav_ws/devel/setup.bash
chmod +x ~/uav_ws/src/d435i_yellow_circle_detector/scripts/*.py
rospack profile
```

把 `/path/to/CUADC` 换成你实际拷贝到 Ubuntu 里的路径。

每次打开新终端后都要先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
```

## 3. 普通检测启动

只检测和发布话题, 不显示窗口, 不采集图片：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch
```

显示两个 OpenCV 窗口：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

窗口名称：

```text
yellow_circle_detector  彩色标注画面
yellow_binary           黄色二值化黑白画面
```

默认显示比例是 `1.0`。如果屏幕太小, 可以临时缩小：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true display_scale:=0.5
```

## 4. 空中采集图片数据集

开启采集后, 程序会以默认 `30 FPS` 保存相机彩色图片。不会保存 `.avi` 或其他视频文件。

推荐空中采集命令, 不开 OpenCV 窗口, 降低机载电脑负载：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true
```

如果地面调试时需要同时看检测画面：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

默认保存目录：

```text
~/yellow_circle_dataset/YYYYMMDD_HHMMSS/images/
```

文件名格式：

```text
d435i_raw_000001_时间戳.jpg
d435i_raw_000002_时间戳.jpg
...
```

指定保存目录、帧率和图片格式：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true record_dir:=/home/lab/dataset record_fps:=30 image_format:=jpg jpeg_quality:=95
```

停止采集：在 roslaunch 终端按 `Ctrl+C`。节点退出时会打印本次保存图片数量和丢帧数量。

采集建议：

- 空中采集建议关闭 `show_window`, 只保留 `enable_record:=true`。
- 使用 SSD 或速度较快的存储盘, 避免写盘太慢导致队列满。
- 如果出现 `Dataset image queue is full`, 可以降低 `jpeg_quality` 或临时降低 `record_fps`。
- 640x480 JPG 30FPS 会持续占用磁盘空间, 起飞前确认剩余容量。

## 5. ROS 话题关系

相机节点发布：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
```

黄色二值化节点订阅：

```text
/d435i/color/image_raw
```

黄色二值化节点发布：

```text
/yellow_circle/binary_image
```

黄色圆检测节点订阅：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
/yellow_circle/binary_image
```

黄色圆检测节点发布：

```text
/yellow_circle                 d435i_yellow_circle_detector/YellowCircle
/yellow_circle/annotated_image sensor_msgs/Image, bgr8
```

图片数据集采集节点订阅：

```text
/d435i/color/image_raw
```

## 6. launch 关键参数

```xml
<arg name="show_window" default="false" />
<arg name="fps" default="30" />
<arg name="display_scale" default="1.0" />
<arg name="enable_record" default="false" />
<arg name="record_dir" default="$(env HOME)/yellow_circle_dataset" />
<arg name="record_fps" default="$(arg fps)" />
<arg name="image_format" default="jpg" />
<arg name="jpeg_quality" default="95" />
```

黄色 HSV 阈值在 `yellow_binarizer_node` 中：

```xml
<param name="h_min" value="18" />
<param name="h_max" value="42" />
<param name="s_min" value="80" />
<param name="s_max" value="255" />
<param name="v_min" value="80" />
<param name="v_max" value="255" />
```

二值图形态学处理参数：

```xml
<param name="morph_kernel" value="5" />
<param name="morph_open_iterations" value="1" />
<param name="morph_close_iterations" value="2" />
```

圆筛选参数：

```xml
<param name="min_area" value="120.0" />
<param name="max_area" value="200000.0" />
<param name="min_radius" value="6.0" />
<param name="min_circularity" value="0.55" />
<param name="center_roi_ratio" value="0.85" />
```

## 7. ROS 找不到 launch 的修复

如果出现：

```text
RLException: [d435i_yellow_circle.launch] is neither a launch file in package [d435i_yellow_circle_detector]
```

说明当前终端没有找到这个 ROS 包。执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

如果仍然找不到, 重新编译：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

## 8. 常见清理命令

如果出现同名节点冲突：

```bash
rosnode kill /d435i_camera_node
rosnode kill /yellow_binarizer_node
rosnode kill /yellow_circle_detector_node
rosnode kill /annotated_image_viewer_node
rosnode kill /binary_image_viewer_node
rosnode kill /dataset_image_recorder_node
```

如果 30FPS 在当前电脑上卡顿, 可以临时降到 15FPS：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true fps:=15
```
