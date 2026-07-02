# Ubuntu 20.04 + ROS Noetic D435i 黄色圆检测说明

本项目是 **ROS1 / ROS Noetic** 代码，不是 ROS2。源码目录在：

```text
D:\CUADC\src\d435i_yellow_circle_detector
```

Ubuntu 中建议工作空间名称为：

```text
~/uav_ws
```

功能包括：

- 使用 D435/D435i 的 Python SDK `pyrealsense2` 读取彩色图和深度图。
- 发布 ROS 图像话题。
- 使用 OpenCV 的 HSV 颜色过滤、形态学处理、轮廓检测和圆度筛选识别黄色圆。
- 输出黄色圆中心像素坐标、半径、面积、中心点深度和相对图像中心偏移。

## 1. 安装依赖

在 Ubuntu 20.04 中执行：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-sensor-msgs \
  python3-pip \
  python3-catkin-tools \
  python3-opencv
```

配置 ROS 环境：

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

安装 RealSense Python SDK：

```bash
python3 -m pip install --user pyrealsense2
```

如果相机权限异常，需要安装 Intel librealsense 的 udev rules，然后重新插拔相机。

## 2. 创建工作空间

```bash
mkdir -p ~/uav_ws/src
cd ~/uav_ws
catkin_make
source devel/setup.bash
```

把功能包复制到 Ubuntu 工作空间：

```bash
cp -r /path/to/CUADC/src/d435i_yellow_circle_detector ~/uav_ws/src/
```

编译：

```bash
cd ~/uav_ws
catkin_make
source devel/setup.bash
```

如果脚本没有执行权限：

```bash
chmod +x ~/uav_ws/src/d435i_yellow_circle_detector/scripts/*.py
```

## 3. 运行

每打开一个新终端，都需要先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
```

启动：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch
```

如果需要 OpenCV 弹窗预览：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

如果不手动启动 `roscore`，`roslaunch` 会自动启动 ROS Master。

## 4. 主要话题

D435/D435i 相机节点发布：

```text
/d435i/color/image_raw          sensor_msgs/Image, bgr8 彩色图
/d435i/aligned_depth/image_raw  sensor_msgs/Image, 32FC1 深度图，单位米
```

黄色圆检测节点发布：

```text
/yellow_circle                  d435i_yellow_circle_detector/YellowCircle
/yellow_circle/annotated_image  sensor_msgs/Image, bgr8 标注图
```

查看检测结果：

```bash
rostopic echo /yellow_circle
```

查看图像：

```bash
rqt_image_view
```

## 5. 功能包结构

```text
d435i_yellow_circle_detector/
  CMakeLists.txt
  package.xml
  launch/
    d435i_yellow_circle.launch
  msg/
    YellowCircle.msg
  scripts/
    d435i_camera_node.py
    yellow_circle_detector.py
```

文件作用：

- `d435i_camera_node.py`：使用 `pyrealsense2` 读取 D435/D435i 彩色和深度数据，并发布 ROS 图像话题。
- `yellow_circle_detector.py`：订阅彩色图和深度图，用 OpenCV 检测黄色圆，并发布检测结果。
- `YellowCircle.msg`：定义黄色圆检测结果消息。
- `d435i_yellow_circle.launch`：同时启动相机节点和检测节点，并设置参数。

## 6. 相机参数

当前 launch 默认参数：

```xml
<param name="color_width" value="640" />
<param name="color_height" value="480" />
<param name="depth_width" value="640" />
<param name="depth_height" value="480" />
<param name="fps" value="6" />
<param name="color_format" value="bgr8" />
<param name="wait_timeout_ms" value="5000" />
<param name="timeout_restart_count" value="3" />
<param name="auto_profile_fallback" value="true" />
<param name="align_depth_to_color" value="true" />
```

说明：

- `fps=6`：降低带宽压力，提高稳定性。
- `color_format=bgr8`：适合你现在 `usb=3.2` 的连接。
- `wait_timeout_ms=5000`：等待相机帧最长 5000 ms。
- `timeout_restart_count=3`：连续 3 次超时后自动切换更保守的 RealSense profile。
- `auto_profile_fallback=true`：节点会枚举相机真实支持的 color/depth profile，并自动尝试。
- `align_depth_to_color=true`：深度图对齐到彩色图，方便根据黄色圆中心像素取深度。

启动日志中应看到类似：

```text
RealSense device: name=Intel RealSense D435 ... usb=3.2
Actual streams: color=640x480 bgr8@6 depth=640x480 format.z16@6
D435i started...
```

如果 `usb=2.1`，说明相机仍然工作在 USB2 模式。D435/D435i 的彩色+深度同步流在 USB2 下很不稳定，应换 USB3.0/3.2 接口或 USB3 数据线。

## 7. 黄色检测参数

HSV 黄色阈值：

```xml
<param name="h_min" value="18" />
<param name="h_max" value="42" />
<param name="s_min" value="80" />
<param name="s_max" value="255" />
<param name="v_min" value="80" />
<param name="v_max" value="255" />
```

轮廓筛选参数：

```xml
<param name="min_area" value="120.0" />
<param name="max_area" value="200000.0" />
<param name="min_radius" value="6.0" />
<param name="min_circularity" value="0.55" />
<param name="center_roi_ratio" value="0.85" />
```

调参建议：

- 检测不到黄色圆：调整 `h_min/h_max/s_min/v_min`。
- 误检小黄点：增大 `min_area` 或 `min_radius`。
- 误检非圆形黄色物体：增大 `min_circularity`。
- 目标可能在画面边缘：增大 `center_roi_ratio` 到 `1.0`。

## 8. 常见问题

### 8.1 roslaunch 找不到 launch 文件

错误：

```text
RLException: [d435i_yellow_circle.launch] is neither a launch file in package [d435i_yellow_circle_detector]
```

原因通常是当前终端没有 source 工作空间，或者包目录不在 `~/uav_ws/src`。

检查：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

正常输出应类似：

```text
/home/lab/uav_ws/src/d435i_yellow_circle_detector
```

如果找不到，检查文件：

```bash
ls ~/uav_ws/src/d435i_yellow_circle_detector/package.xml
ls ~/uav_ws/src/d435i_yellow_circle_detector/launch/d435i_yellow_circle.launch
```

然后重新编译：

```bash
cd ~/uav_ws
catkin_make
source ~/uav_ws/devel/setup.bash
rospack profile
```

### 8.2 同名节点冲突

错误：

```text
shutdown request: [/d435i_camera_node] Reason: new node registered with same name
```

原因是 ROS Master 中已经有同名节点，通常是之前的 `roslaunch` 还在运行，或旧进程没有退出。

清理：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
rosnode list
rosnode kill /d435i_camera_node
rosnode kill /yellow_circle_detector
```

如果仍然冲突：

```bash
ps aux | grep d435i_yellow_circle_detector
pkill -f d435i_camera_node.py
pkill -f yellow_circle_detector.py
```

然后重新运行：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

### 8.3 RealSense 帧超时

错误：

```text
RealSense frame read failed: Frame didn't arrive within 5000
```

检查：

```bash
lsusb
```

运行节点时看日志中的 USB 类型：

```text
usb=3.2
```

如果是 `usb=2.1`，优先换 USB3 接口或 USB3 数据线。还需要确认没有其他程序占用相机，例如：

```bash
pkill -f realsense-viewer
```

### 8.4 OpenCV findContours 返回值错误

如果出现：

```text
ValueError: too many values to unpack (expected 2)
```

这是 OpenCV 3/4 的 `cv2.findContours()` 返回值差异。当前 `yellow_circle_detector.py` 已经兼容两种返回格式：

```python
def find_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 2:
        contours, hierarchy = result
    else:
        _, contours, hierarchy = result
    return contours, hierarchy
```

更新代码后重新编译运行即可。

## 9. 推荐启动流程

如果不确定当前 ROS 状态是否干净，按下面流程重新启动：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
rosnode kill /d435i_camera_node
rosnode kill /yellow_circle_detector
pkill -f d435i_camera_node.py
pkill -f yellow_circle_detector.py
cd ~/uav_ws
catkin_make
source ~/uav_ws/devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```
