# Ubuntu 20.04 + ROS Noetic D435/D435i 黄色圆检测说明

本项目是 **ROS1 / ROS Noetic** 代码，不是 ROS2。Ubuntu 工作空间固定使用：

```text
~/uav_ws
```

源码包目录：

```text
~/uav_ws/src/d435i_yellow_circle_detector
```

当前功能：
- 使用 `pyrealsense2` 读取 D435/D435i 彩色图和深度图。
- 默认 `30 FPS`。
- 使用 HSV 只提取黄色区域。
- 将黄色区域二值化，并发布黑白二值图。
- 在二值图上找轮廓，按面积、圆度、半径筛选黄色圆。
- 只输出一个最优黄色圆。
- 可选开启 OpenCV 显示窗口。
- 可选开启视频录制，保存原始画面、标注画面和二值化画面。

## 1. 创建工作空间并编译

如果是第一次在 Ubuntu 上使用，执行：

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

注意：`/path/to/CUADC` 要换成你实际拷贝到 Ubuntu 上的路径。

## 2. 每次打开新终端必须先 source

每次新开一个终端，都先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
```

建议直接用下面这种完整形式运行，避免忘记 source：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

## 3. 开启显示窗口

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

会打开两个 OpenCV 窗口：
- `yellow_circle_detector`：彩色标注图。
- `yellow_binary`：黄色区域的黑白二值图。

默认显示比例是 `1.0`，不会主动放大。如果画面仍然太大，可以临时缩小：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true display_scale:=0.5
```

## 4. 开启录制功能

你的这条命令本身是正确的：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

但运行前必须保证当前终端已经 source 过 `~/uav_ws/devel/setup.bash`。推荐使用完整命令：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

默认保存目录：

```text
~/yellow_circle_records
```

会生成三个视频文件：

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

如果只想录制，不想显示 OpenCV 窗口：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true
```

## 5. 话题

相机节点发布：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
```

黄色圆检测节点发布：

```text
/yellow_circle                 d435i_yellow_circle_detector/YellowCircle
/yellow_circle/annotated_image sensor_msgs/Image, bgr8
/yellow_circle/binary_image    sensor_msgs/Image, mono8
```

调参时优先看：

```text
/yellow_circle/binary_image
```

黄色圆在这张图里应该是白色区域，背景应该尽量是黑色。

## 6. 关键参数

launch 文件中的默认参数：

```xml
<arg name="show_window" default="false" />
<arg name="fps" default="30" />
<arg name="display_scale" default="1.0" />
<arg name="enable_record" default="false" />
<arg name="record_dir" default="$(env HOME)/yellow_circle_records" />
<arg name="record_fps" default="$(arg fps)" />
```

黄色 HSV 阈值：

```xml
<param name="h_min" value="18" />
<param name="h_max" value="42" />
<param name="s_min" value="80" />
<param name="s_max" value="255" />
<param name="v_min" value="80" />
<param name="v_max" value="255" />
```

二值图形态学处理：

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

## 7. 这次 RLException 的原因和修复

如果出现：

```text
RLException: [d435i_yellow_circle.launch] is neither a launch file in package [d435i_yellow_circle_detector] nor is [d435i_yellow_circle_detector] a launch file name
```

含义是：当前终端的 ROS 环境没有找到 `d435i_yellow_circle_detector` 这个包，通常不是录制参数的问题。

先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

如果 `rospack find` 能输出：

```text
/home/lab/uav_ws/src/d435i_yellow_circle_detector
```

再运行：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

如果 `rospack find` 仍然失败，检查包是否真的在工作空间里：

```bash
ls ~/uav_ws/src/d435i_yellow_circle_detector/package.xml
ls ~/uav_ws/src/d435i_yellow_circle_detector/launch/d435i_yellow_circle.launch
```

如果上面两个文件存在，重新编译并刷新包缓存：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

然后再启动：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

## 8. 常见清理命令

如果出现同名节点冲突：

```bash
rosnode kill /d435i_camera_node
rosnode kill /yellow_circle_detector
pkill -f d435i_camera_node.py
pkill -f yellow_circle_detector.py
```

如果 30 FPS 在当前机器上仍然卡顿，可以临时降到 15 FPS：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true fps:=15
```
