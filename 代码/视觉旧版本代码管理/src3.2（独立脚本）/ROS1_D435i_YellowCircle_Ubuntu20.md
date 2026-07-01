# Ubuntu 20.04 + ROS Noetic D435/D435i 黄色圆检测说明

本项目是 **ROS1 / ROS Noetic** 代码，不是 ROS2。Ubuntu 工作空间固定使用：

```text
~/uav_ws
```

源码包目录：

```text
~/uav_ws/src/d435i_yellow_circle_detector
```

当前版本已经拆成多个独立节点：
- `d435i_camera_node.py`：开启 D435/D435i 深度相机 SDK，发布原始彩色图和深度图。
- `yellow_binarizer_node.py`：黄色识别和二值化，发布黑白二值图。
- `yellow_circle_detector_node.py`：在二值图上识别黄色圆，发布检测结果和彩色标注图。
- `annotated_image_viewer_node.py`：开启 OpenCV 黄色识别彩色画面窗口。
- `binary_image_viewer_node.py`：开启 OpenCV 黄色二值化窗口。
- `raw_video_recorder_node.py`：录制原始相机画面。
- `annotated_video_recorder_node.py`：录制黄色识别彩色标注画面。
- `binary_video_recorder_node.py`：录制黄色二值化画面。

## 1. 创建工作空间并编译

第一次在 Ubuntu 上使用：

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

`/path/to/CUADC` 要换成你实际拷贝到 Ubuntu 上的路径。

## 2. 每次打开新终端必须先 source

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
```

推荐每次启动都用完整形式：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

## 3. 一键启动所有节点

只检测和发布话题，不显示窗口、不录制：

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

显示窗口并同时录制三个视频：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

只录制，不显示窗口：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true
```

## 4. OpenCV 窗口

`show_window:=true` 会启动两个独立显示节点：

```text
annotated_image_viewer_node -> yellow_circle_detector 窗口
binary_image_viewer_node    -> yellow_binary 窗口
```

默认显示比例是 `1.0`。如果画面太大：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true display_scale:=0.5
```

## 5. 录制功能

`enable_record:=true` 会启动三个独立录制节点。

默认保存目录：

```text
~/yellow_circle_records
```

生成三个视频文件：

```text
yellow_circle_raw_时间戳.avi
yellow_circle_annotated_时间戳.avi
yellow_circle_binary_时间戳.avi
```

指定保存目录：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true record_dir:=/home/lab/yellow_circle_records
```

## 6. 话题关系

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

黄色圆识别节点订阅：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
/yellow_circle/binary_image
```

黄色圆识别节点发布：

```text
/yellow_circle                 d435i_yellow_circle_detector/YellowCircle
/yellow_circle/annotated_image sensor_msgs/Image, bgr8
```

显示节点和录制节点只订阅对应图像话题，不再参与检测计算。

## 7. launch 关键参数

```xml
<arg name="show_window" default="false" />
<arg name="fps" default="30" />
<arg name="display_scale" default="1.0" />
<arg name="enable_record" default="false" />
<arg name="record_dir" default="$(env HOME)/yellow_circle_records" />
<arg name="record_fps" default="$(arg fps)" />
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

二值图形态学处理在 `yellow_binarizer_node` 中：

```xml
<param name="morph_kernel" value="5" />
<param name="morph_open_iterations" value="1" />
<param name="morph_close_iterations" value="2" />
```

圆筛选参数在 `yellow_circle_detector_node` 中：

```xml
<param name="min_area" value="120.0" />
<param name="max_area" value="200000.0" />
<param name="min_radius" value="6.0" />
<param name="min_circularity" value="0.55" />
<param name="center_roi_ratio" value="0.85" />
```

## 8. ROS 找不到 launch 的修复

如果出现：

```text
RLException: [d435i_yellow_circle.launch] is neither a launch file in package [d435i_yellow_circle_detector] nor is [d435i_yellow_circle_detector] a launch file name
```

说明当前终端没有找到这个 ROS 包。执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

如果仍然找不到，重新编译：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

## 9. 常见清理命令

如果出现同名节点冲突：

```bash
rosnode kill /d435i_camera_node
rosnode kill /yellow_binarizer_node
rosnode kill /yellow_circle_detector_node
rosnode kill /annotated_image_viewer_node
rosnode kill /binary_image_viewer_node
rosnode kill /raw_video_recorder_node
rosnode kill /annotated_video_recorder_node
rosnode kill /binary_video_recorder_node
```

如果 30 FPS 在当前机器上仍然卡顿，可以临时降到 15 FPS：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true fps:=15
```
