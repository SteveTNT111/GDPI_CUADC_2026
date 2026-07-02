# D435/D435i 黄色圆检测 ROS1 代码说明

当前版本面向 **Ubuntu 20.04 + ROS Noetic**。默认相机帧率为 `30 FPS`。检测逻辑是先识别黄色区域并二值化, 再在二值图中通过轮廓、面积、半径和圆度筛选一个最可靠的圆。

## 1. 文件结构

```text
d435i_yellow_circle_detector/
  CMakeLists.txt
  package.xml
  launch/d435i_yellow_circle.launch
  msg/YellowCircle.msg
  scripts/d435i_camera_node.py
  scripts/yellow_binarizer_node.py
  scripts/yellow_circle_detector_node.py
  scripts/annotated_image_viewer_node.py
  scripts/binary_image_viewer_node.py
  scripts/dataset_image_recorder_node.py
  scripts/start_d435i_record.sh
  scripts/install_desktop_launcher.sh
  D435i_YellowCircle_Record.desktop
```

源码目录中可能还保留旧版视频录制脚本, 但当前 `launch` 文件和 `CMakeLists.txt` 不再启动或安装这些视频节点。当前数据采集只保存图片, 不保存视频。

## 2. 总体数据流

```text
D435/D435i
  -> d435i_camera_node.py
  -> /d435i/color/image_raw
  -> yellow_binarizer_node.py
  -> /yellow_circle/binary_image
  -> yellow_circle_detector_node.py
  -> /yellow_circle
  -> /yellow_circle/annotated_image
```

深度数据流：

```text
D435/D435i
  -> d435i_camera_node.py
  -> /d435i/aligned_depth/image_raw
  -> yellow_circle_detector_node.py
```

显示和采集节点：

```text
/yellow_circle/annotated_image -> annotated_image_viewer_node.py
/yellow_circle/binary_image    -> binary_image_viewer_node.py
/d435i/color/image_raw         -> dataset_image_recorder_node.py
```

## 3. d435i_camera_node.py

作用：打开 RealSense D435/D435i 并发布 ROS 图像话题。

主要功能：

- 使用 `pyrealsense2` 打开相机。
- 读取彩色图和深度图。
- 将深度图对齐到彩色图。
- 发布 `/d435i/color/image_raw` 和 `/d435i/aligned_depth/image_raw`。
- 自动枚举并切换 RealSense 可用 profile, 降低 USB 带宽或格式不匹配导致的启动失败概率。
- 默认使用 `640x480, 30 FPS, bgr8, z16`。

发布：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
```

## 4. yellow_binarizer_node.py

作用：把彩色图中的黄色区域提取出来, 生成黑白二值图。

主要功能：

- 订阅 `/d435i/color/image_raw`。
- 将 BGR 图像转换为 HSV。
- 使用 HSV 阈值提取黄色区域。
- 对 mask 做开运算和闭运算, 减少噪声并填补小空洞。
- 发布 `/yellow_circle/binary_image`。

关键逻辑：

```python
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
```

发布：

```text
/yellow_circle/binary_image
```

## 5. yellow_circle_detector_node.py

作用：在二值图中识别一个黄色圆, 并输出检测结果。

主要功能：

- 订阅原始彩色图、对齐深度图和黄色二值图。
- 在二值图中查找外轮廓。
- 通过面积、半径、圆度和中心 ROI 筛选圆。
- 只选择评分最高的一个圆。
- 查询圆心附近深度, 输出单位为米。
- 发布结构化检测结果 `/yellow_circle`。
- 发布彩色标注图 `/yellow_circle/annotated_image`。

圆度计算：

```python
circularity = 4.0 * math.pi * area / (perimeter * perimeter)
```

最优圆评分：

```python
score = area * circularity - distance_to_center * 2.0
```

订阅：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
/yellow_circle/binary_image
```

发布：

```text
/yellow_circle
/yellow_circle/annotated_image
```

## 6. annotated_image_viewer_node.py

作用：显示彩色检测结果窗口。

主要功能：

- 订阅 `/yellow_circle/annotated_image`。
- 打开 OpenCV 窗口 `yellow_circle_detector`。
- 显示带圆心、半径、检测框和深度文字的彩色画面。
- 支持 `display_scale` 显示缩放, 默认 `1.0`。

此节点只在 `show_window:=true` 时启动。

## 7. binary_image_viewer_node.py

作用：显示黄色二值化黑白窗口。

主要功能：

- 订阅 `/yellow_circle/binary_image`。
- 打开 OpenCV 窗口 `yellow_binary`。
- 显示黄色区域的黑白二值化结果。
- 支持 `display_scale` 显示缩放, 默认 `1.0`。

此节点只在 `show_window:=true` 时启动。

## 8. dataset_image_recorder_node.py

作用：采集图片数据集。它只保存照片, 不保存视频。

主要功能：

- 订阅 `/d435i/color/image_raw`。
- 按 `record_fps` 控制保存频率, 默认跟随相机 `30 FPS`。
- 使用后台写盘线程保存图片, 减少对相机回调的阻塞。
- 使用有界队列缓存待写图片, 写盘太慢时会主动提示丢帧数量。
- 每次启动创建一个新的时间戳目录。
- 支持 `jpg/jpeg/png`。

默认输出目录：

```text
~/yellow_circle_dataset/YYYYMMDD_HHMMSS/images/
```

默认文件名：

```text
d435i_raw_000001_时间戳.jpg
```

常用参数：

```text
image_topic     默认 /d435i/color/image_raw
record_dir      默认 ~/yellow_circle_dataset
record_fps      默认 30
image_format    默认 jpg
jpeg_quality    默认 95
queue_size      默认 120
```

空中采集推荐命令：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true
```

## 9. msg/YellowCircle.msg

结构化黄色圆检测结果：

```text
header          图像时间戳和坐标系
detected        是否检测到黄色圆
x               圆心像素 x
y               圆心像素 y
radius          圆半径, 单位像素
area            轮廓面积
depth_m         圆心附近深度, 单位米
center_offset_x 圆心相对图像中心的 x 偏移
center_offset_y 圆心相对图像中心的 y 偏移
```

## 10. start_d435i_record.sh

作用：在 Ubuntu 桌面双击启动时真正执行 ROS 命令。

它执行的流程等价于：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

这个脚本会先检查 `~/uav_ws`、`/opt/ros/noetic/setup.bash` 和 `devel/setup.bash` 是否存在。`roslaunch` 退出后终端不会立刻关闭, 方便查看错误日志和保存图片数量。

## 11. install_desktop_launcher.sh

作用：在 Ubuntu 桌面生成可双击的启动图标。

运行方式：

```bash
cd ~/uav_ws/src/d435i_yellow_circle_detector
bash scripts/install_desktop_launcher.sh
```

生成文件：

```text
~/Desktop/D435i_YellowCircle_Record.desktop
```

## 12. D435i_YellowCircle_Record.desktop

作用：Ubuntu 桌面启动器文件。

双击后会打开终端, 调用：

```text
~/uav_ws/src/d435i_yellow_circle_detector/scripts/start_d435i_record.sh
```

如果 Ubuntu 提示 `Untrusted application launcher`, 右键桌面图标并选择 `Allow Launching`。

## 13. launch/d435i_yellow_circle.launch

一键启动脚本。默认启动三个核心节点：

```text
d435i_camera_node
yellow_binarizer_node
yellow_circle_detector_node
```

当 `show_window:=true` 时额外启动：

```text
annotated_image_viewer_node
binary_image_viewer_node
```

当 `enable_record:=true` 时额外启动：

```text
dataset_image_recorder_node
```

显示并采集图片的启动命令：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

空中采集时建议关闭窗口：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true
```
