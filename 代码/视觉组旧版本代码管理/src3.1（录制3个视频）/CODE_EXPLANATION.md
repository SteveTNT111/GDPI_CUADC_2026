# D435/D435i 黄色圆检测 ROS1 代码说明

当前版本用于 **Ubuntu 20.04 + ROS Noetic**，功能是检测黄色圆：
- 默认 `30 FPS`。
- 使用 HSV 提取黄色区域。
- 将黄色区域二值化。
- 在二值图上找轮廓并筛选圆。
- 只输出一个最优黄色圆。
- 可显示彩色标注窗口和黑白二值化窗口。
- 可录制原始彩色视频、彩色标注视频和二值化视频。

## 1. 文件结构

```text
d435i_yellow_circle_detector/
  CMakeLists.txt
  package.xml
  launch/d435i_yellow_circle.launch
  msg/YellowCircle.msg
  scripts/d435i_camera_node.py
  scripts/yellow_circle_detector.py
```

## 2. 数据流程

```text
D435/D435i
  -> d435i_camera_node.py
  -> /d435i/color/image_raw
  -> /d435i/aligned_depth/image_raw
  -> yellow_circle_detector.py
  -> /yellow_circle
  -> /yellow_circle/annotated_image
  -> /yellow_circle/binary_image
```

## 3. launch/d435i_yellow_circle.launch

作用：
- 同时启动相机节点和黄色圆检测节点。
- 设置相机分辨率、FPS、图像格式、深度对齐。
- 设置黄色 HSV 阈值。
- 设置圆检测筛选参数。
- 控制 OpenCV 窗口显示和视频录制。

主要参数：

```xml
<arg name="show_window" default="false" />
<arg name="fps" default="30" />
<arg name="display_scale" default="1.0" />
<arg name="enable_record" default="false" />
<arg name="record_dir" default="$(env HOME)/yellow_circle_records" />
<arg name="record_fps" default="$(arg fps)" />
```

说明：
- `show_window:=true`：打开 OpenCV 显示窗口。
- `display_scale:=1.0`：显示原始 1.0 倍画面。
- `enable_record:=true`：开启视频录制。
- `record_dir:=...`：指定录制保存目录。
- `record_fps:=...`：指定录制视频帧率，默认跟相机 FPS 一致。

## 4. scripts/d435i_camera_node.py

作用：
- 使用 `pyrealsense2` 打开 D435/D435i。
- 读取彩色图和深度图。
- 将深度图对齐到彩色图。
- 发布 ROS 图像话题。
- 自动枚举和切换 RealSense 可用 profile，降低 USB 带宽不匹配导致的启动失败概率。

发布话题：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
```

默认 FPS：

```python
fps = int(rospy.get_param("~fps", 30))
```

默认彩色图格式：

```xml
<param name="color_format" value="bgr8" />
```

## 5. scripts/yellow_circle_detector.py

作用：
- 订阅彩色图和深度图。
- 将彩色图从 BGR 转成 HSV。
- 用 HSV 阈值提取黄色。
- 生成黑白二值图。
- 对二值图做开运算和闭运算，减少噪声和孔洞。
- 在二值图上找轮廓。
- 用面积、圆度、半径、中心 ROI 筛选圆。
- 选择评分最高的一个黄色圆。
- 发布检测结果、标注图和二值图。
- 可选显示窗口。
- 可选录制视频。

黄色阈值：

```python
self.lower_yellow = np.array([h_min, s_min, v_min], dtype=np.uint8)
self.upper_yellow = np.array([h_max, s_max, v_max], dtype=np.uint8)
mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
```

二值图处理：

```python
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=self.morph_open_iterations)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.morph_close_iterations)
```

OpenCV 3/4 兼容的轮廓查找：

```python
def find_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 2:
        contours, hierarchy = result
    else:
        _, contours, hierarchy = result
    return contours, hierarchy
```

圆度计算：

```python
circularity = 4.0 * math.pi * area / (perimeter * perimeter)
```

最优圆评分：

```python
score = area * circularity - distance_to_center * 2.0
```

## 6. msg/YellowCircle.msg

作用：定义黄色圆检测结果消息。

字段含义：

```text
header          图像时间戳和坐标系
detected        是否检测到黄色圆
x               圆心像素 x
y               圆心像素 y
radius          圆半径，单位像素
area            轮廓面积
depth_m         圆心附近深度，单位米
center_offset_x 圆心相对图像中心的 x 偏移
center_offset_y 圆心相对图像中心的 y 偏移
```

## 7. 发布话题

```text
/yellow_circle
/yellow_circle/annotated_image
/yellow_circle/binary_image
```

说明：
- `/yellow_circle`：结构化检测结果。
- `/yellow_circle/annotated_image`：原图上画出检测到的黄色圆。
- `/yellow_circle/binary_image`：黄色区域二值图，用于调试 HSV 阈值。

当 `show_window:=true` 时，显示两个窗口：
- `yellow_circle_detector`：彩色标注图。
- `yellow_binary`：黑白二值图。

## 8. 录制功能

录制由 `yellow_circle_detector.py` 里的 OpenCV `VideoWriter` 完成。

开启命令：

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

保存文件：

```text
yellow_circle_raw_时间戳.avi
yellow_circle_annotated_时间戳.avi
yellow_circle_binary_时间戳.avi
```

如果出现 `RLException: [d435i_yellow_circle.launch] is neither a launch file...`，说明当前终端没有找到 ROS 包。先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

只有 `rospack find` 能找到包之后，`roslaunch` 才能正常运行。
