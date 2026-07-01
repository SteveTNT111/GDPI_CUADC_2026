# D435/D435i 黄色圆检测 ROS1 多节点代码说明

当前版本已经把原来集中在一个检测脚本里的功能拆成多个独立 ROS1 节点。默认平台是 **Ubuntu 20.04 + ROS Noetic**，默认相机帧率是 `30 FPS`。

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
  scripts/raw_video_recorder_node.py
  scripts/annotated_video_recorder_node.py
  scripts/binary_video_recorder_node.py
```

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

深度图数据流：

```text
D435/D435i
  -> d435i_camera_node.py
  -> /d435i/aligned_depth/image_raw
  -> yellow_circle_detector_node.py
```

显示节点和录制节点只订阅图像话题：

```text
/d435i/color/image_raw              -> raw_video_recorder_node.py
/yellow_circle/annotated_image      -> annotated_image_viewer_node.py
/yellow_circle/annotated_image      -> annotated_video_recorder_node.py
/yellow_circle/binary_image         -> binary_image_viewer_node.py
/yellow_circle/binary_image         -> binary_video_recorder_node.py
```

## 3. d435i_camera_node.py

节点作用：开启深度相机 SDK 节点。

功能：
- 使用 `pyrealsense2` 打开 D435/D435i。
- 读取彩色图和深度图。
- 将深度图对齐到彩色图。
- 发布 ROS 图像话题。
- 自动枚举和切换 RealSense 可用 profile，降低 USB 带宽不匹配导致的启动失败概率。

发布：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
```

## 4. yellow_binarizer_node.py

节点作用：黄色识别节点和黄色二值化节点。

功能：
- 订阅 `/d435i/color/image_raw`。
- 将 BGR 图像转换为 HSV。
- 使用 HSV 阈值提取黄色区域。
- 对黄色区域做开运算和闭运算。
- 发布黑白二值图。

发布：

```text
/yellow_circle/binary_image
```

关键代码：

```python
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
```

## 5. yellow_circle_detector_node.py

节点作用：黄色圆识别节点。

功能：
- 订阅原始彩色图、深度图和黄色二值图。
- 在二值图上找轮廓。
- 用面积、圆度、半径和中心 ROI 筛选黄色圆。
- 只选择一个评分最高的黄色圆。
- 查询圆心附近深度。
- 发布结构化识别结果。
- 发布彩色标注图。

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

圆度计算：

```python
circularity = 4.0 * math.pi * area / (perimeter * perimeter)
```

最优圆评分：

```python
score = area * circularity - distance_to_center * 2.0
```

## 6. annotated_image_viewer_node.py

节点作用：开启 OpenCV 的黄色识别彩色画面节点。

功能：
- 订阅 `/yellow_circle/annotated_image`。
- 打开 `yellow_circle_detector` 窗口。
- 显示带黄色圆标注的彩色画面。
- 支持 `display_scale` 缩放显示。

此节点只在 launch 参数 `show_window:=true` 时启动。

## 7. binary_image_viewer_node.py

节点作用：开启 OpenCV 的二值化界面节点。

功能：
- 订阅 `/yellow_circle/binary_image`。
- 打开 `yellow_binary` 窗口。
- 显示黑白二值化画面。
- 支持 `display_scale` 缩放显示。

此节点只在 launch 参数 `show_window:=true` 时启动。

## 8. raw_video_recorder_node.py

节点作用：原始画面的屏幕录制节点。

功能：
- 订阅 `/d435i/color/image_raw`。
- 录制相机看到的原始彩色画面。
- 不叠加检测框和文字。

输出文件：

```text
yellow_circle_raw_时间戳.avi
```

此节点只在 launch 参数 `enable_record:=true` 时启动。

## 9. annotated_video_recorder_node.py

节点作用：黄色识别彩色画面的屏幕录制节点。

功能：
- 订阅 `/yellow_circle/annotated_image`。
- 录制带检测框、圆心和深度文字的彩色画面。

输出文件：

```text
yellow_circle_annotated_时间戳.avi
```

此节点只在 launch 参数 `enable_record:=true` 时启动。

## 10. binary_video_recorder_node.py

节点作用：二值化画面的屏幕录制节点。

功能：
- 订阅 `/yellow_circle/binary_image`。
- 录制黑白二值化画面。

输出文件：

```text
yellow_circle_binary_时间戳.avi
```

此节点只在 launch 参数 `enable_record:=true` 时启动。

## 11. msg/YellowCircle.msg

结构化黄色圆检测结果：

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

## 12. launch/d435i_yellow_circle.launch

一键启动脚本。默认会启动三个核心节点：

```text
d435i_camera_node
yellow_binarizer_node
yellow_circle_detector_node
```

当 `show_window:=true` 时，额外启动：

```text
annotated_image_viewer_node
binary_image_viewer_node
```

当 `enable_record:=true` 时，额外启动：

```text
raw_video_recorder_node
annotated_video_recorder_node
binary_video_recorder_node
```

显示并录制的启动命令：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```
