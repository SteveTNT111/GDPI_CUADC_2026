# Ubuntu 20.04 + ROS Noetic D435/D435i 黄色圆检测说明

本项目是 **ROS1 / ROS Noetic** 代码，不是 ROS2。源码目录：

```text
D:\CUADC\src\d435i_yellow_circle_detector
```

Ubuntu 工作空间使用：

```text
~/uav_ws
```

当前功能：

- 使用 `pyrealsense2` 读取 D435/D435i 彩色图和深度图。
- 默认 `30 FPS`。
- 使用 HSV 提取黄色区域。
- 将黄色区域二值化，发布二值图。
- 在黄色二值图上找轮廓，通过面积、圆度、半径筛选黄色圆。
- 只输出一个最优黄色圆。

## 1. 编译与运行

```bash
mkdir -p ~/uav_ws/src
cp -r /path/to/CUADC/src/d435i_yellow_circle_detector ~/uav_ws/src/
cd ~/uav_ws
catkin_make
source devel/setup.bash
chmod +x ~/uav_ws/src/d435i_yellow_circle_detector/scripts/*.py
```

每个新终端都先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
```

运行：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

会打开两个 OpenCV 窗口：

- `yellow_circle_detector`：彩色标注图。
- `yellow_binary`：黑白二值图。

如果窗口卡顿：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true display_scale:=0.5
```

如果 30 FPS 仍然不稳：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true fps:=15
```

## 2. 话题

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

调参时优先查看：

```text
/yellow_circle/binary_image
```

它显示的是 HSV 黄色过滤后的二值图。黄色圆在这张图里应该是白色区域。

## 3. 关键参数

相机参数：

```xml
<arg name="fps" default="30" />
<arg name="display_scale" default="1.0" />
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

## 4. 调参建议

如果二值图里黄色圆不明显：

- 调整 `h_min/h_max`。
- 黄色偏暗时，降低 `v_min`。
- 黄色不够纯时，降低 `s_min`。

如果二值图噪声多：

- 增大 `morph_open_iterations`。
- 增大 `min_area`。
- 增大 `min_radius`。

如果黄色区域识别到了，但圆检测不稳定：

- 降低 `min_circularity`，例如 `0.45`。
- 增大 `center_roi_ratio` 到 `1.0`。

## 5. 常见清理命令

如果出现同名节点冲突：

```bash
rosnode kill /d435i_camera_node
rosnode kill /yellow_circle_detector
pkill -f d435i_camera_node.py
pkill -f yellow_circle_detector.py
```

如果 ROS 找不到包：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```
