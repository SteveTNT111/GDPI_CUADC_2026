# D435i 黄色圆检测 ROS1 代码说明

本文说明 `D:\CUADC\src` 中生成代码的作用。当前源码目录下的 ROS1 功能包是：

```text
D:\CUADC\src\d435i_yellow_circle_detector
```

该包面向 **Ubuntu 20.04 + ROS Noetic**，使用 catkin 构建，不是 ROS2 包。

## 1. 总体数据流

运行后有两个主要节点：

```text
d435i_camera_node.py
yellow_circle_detector.py
```

数据流：

```text
D435/D435i 相机
  -> d435i_camera_node.py
  -> /d435i/color/image_raw
  -> /d435i/aligned_depth/image_raw
  -> yellow_circle_detector.py
  -> /yellow_circle
  -> /yellow_circle/annotated_image
```

含义：

- `d435i_camera_node.py`：用 `pyrealsense2` 读取彩色图和深度图，并发布 ROS 图像话题。
- `yellow_circle_detector.py`：订阅彩色图和深度图，用 OpenCV 检测黄色圆，并发布检测结果。
- `/yellow_circle`：黄色圆检测结果。
- `/yellow_circle/annotated_image`：画出轮廓、圆心和深度文字的标注图。

## 2. 功能包结构

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

## 3. `package.xml`

路径：

```text
D:\CUADC\src\d435i_yellow_circle_detector\package.xml
```

作用：

- 声明 ROS1 功能包名称：`d435i_yellow_circle_detector`。
- 声明构建工具：`catkin`。
- 声明 Python 节点、图像消息、自定义消息需要的依赖。

主要依赖：

```xml
<buildtool_depend>catkin</buildtool_depend>
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
<depend>rospy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
```

依赖含义：

- `rospy`：ROS1 Python 节点接口。
- `sensor_msgs`：提供 `sensor_msgs/Image` 图像消息。
- `std_msgs`：提供 `Header` 等标准消息依赖。
- `cv_bridge`：OpenCV/Numpy 图像和 ROS Image 之间转换。
- `image_transport`：ROS 图像传输相关依赖。
- `message_generation`：编译 `YellowCircle.msg` 时使用。
- `message_runtime`：运行时使用自定义消息时需要。

## 4. `CMakeLists.txt`

路径：

```text
D:\CUADC\src\d435i_yellow_circle_detector\CMakeLists.txt
```

作用：

- 配置 catkin 编译流程。
- 生成自定义消息 `YellowCircle`。
- 安装 Python 节点脚本和 launch 文件。

关键代码：

```cmake
add_message_files(
  FILES
  YellowCircle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
```

这部分告诉 catkin 根据 `msg/YellowCircle.msg` 生成 ROS 消息代码。

```cmake
catkin_install_python(PROGRAMS
  scripts/d435i_camera_node.py
  scripts/yellow_circle_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

这部分把两个 Python 文件作为 ROS 可执行节点安装。

## 5. `launch/d435i_yellow_circle.launch`

路径：

```text
D:\CUADC\src\d435i_yellow_circle_detector\launch\d435i_yellow_circle.launch
```

作用：

- 同时启动相机节点和黄色圆检测节点。
- 统一配置相机参数、话题名、HSV 阈值和轮廓筛选参数。

相机节点：

```xml
<node pkg="d435i_yellow_circle_detector"
      type="d435i_camera_node.py"
      name="d435i_camera_node"
      output="screen">
```

检测节点：

```xml
<node pkg="d435i_yellow_circle_detector"
      type="yellow_circle_detector.py"
      name="yellow_circle_detector"
      output="screen">
```

当前相机默认参数：

```xml
<param name="color_width" value="640" />
<param name="color_height" value="480" />
<param name="depth_width" value="640" />
<param name="depth_height" value="480" />
<arg name="fps" default="45" />
<arg name="display_scale" default="1.0" />
<param name="fps" value="$(arg fps)" />
<param name="color_format" value="bgr8" />
<param name="wait_timeout_ms" value="5000" />
<param name="timeout_restart_count" value="3" />
<param name="auto_profile_fallback" value="true" />
<param name="align_depth_to_color" value="true" />
```

说明：

- `fps=45`：当前默认值，在流畅度和传输稳定性之间折中；如果卡顿或帧超时，可运行时改成 `fps:=30`、`fps:=15` 或 `fps:=6`。
- `display_scale=1.0`：OpenCV 预览显示比例，卡顿时可以运行 `display_scale:=0.5`。
- `color_format=bgr8`：USB3.2 下优先使用 OpenCV 直接可用的 BGR 图。
- `queue_size=1`：发布和订阅都尽量只保留最新帧，减少旧帧堆积导致的延迟。
- `wait_timeout_ms=5000`：等待 RealSense 帧的超时时间。
- `timeout_restart_count=3`：连续 3 次读帧超时后重启 pipeline。
- `auto_profile_fallback=true`：启动时枚举相机真实支持的 profile，并在失败时切换。
- `align_depth_to_color=true`：把深度图对齐到彩色图坐标系。

黄色检测参数：

```xml
<param name="h_min" value="18" />
<param name="h_max" value="42" />
<param name="s_min" value="80" />
<param name="s_max" value="255" />
<param name="v_min" value="80" />
<param name="v_max" value="255" />
<param name="min_area" value="120.0" />
<param name="max_area" value="200000.0" />
<param name="min_radius" value="6.0" />
<param name="min_circularity" value="0.55" />
<param name="center_roi_ratio" value="0.85" />
```

## 6. `msg/YellowCircle.msg`

路径：

```text
D:\CUADC\src\d435i_yellow_circle_detector\msg\YellowCircle.msg
```

作用：

- 定义黄色圆检测结果的自定义 ROS 消息。

字段：

```text
Header header
bool detected
int32 x
int32 y
float32 radius
float32 area
float32 depth_m
float32 center_offset_x
float32 center_offset_y
```

含义：

- `header`：时间戳和坐标系。
- `detected`：是否检测到黄色圆。
- `x/y`：圆心像素坐标。
- `radius`：像素半径。
- `area`：轮廓面积，单位是像素面积。
- `depth_m`：圆心深度，单位米。
- `center_offset_x/y`：圆心相对图像中心的像素偏移。

## 7. `scripts/d435i_camera_node.py`

路径：

```text
D:\CUADC\src\d435i_yellow_circle_detector\scripts\d435i_camera_node.py
```

作用：

- 打开 D435/D435i。
- 读取彩色帧和深度帧。
- 将深度对齐到彩色图。
- 将图像转换为 ROS `sensor_msgs/Image`。
- 发布 `/d435i/color/image_raw` 和 `/d435i/aligned_depth/image_raw`。

### 7.1 格式定义

代码支持以下 RealSense 彩色格式：

```python
COLOR_FORMATS = {
    "bgr8": rs.format.bgr8,
    "rgb8": rs.format.rgb8,
    "yuyv": rs.format.yuyv,
    "y8": rs.format.y8,
}
```

发布给 ROS 时最终都转换成 `bgr8`。

### 7.2 `get_bool_param()`

作用：

- 安全读取 ROS 布尔参数。
- 兼容 launch 中的 `true/false` 字符串。

这样可以避免 Python 中 `bool("false") == True` 的问题。

### 7.3 `build_profile_candidates()`

作用：

- 构造 RealSense stream profile 候选列表。
- 第一项是 launch 请求的配置。
- 后续候选来自相机真实支持的 profile 枚举。

这样不会硬试相机不支持的分辨率或格式。

### 7.4 `enumerate_supported_profiles()`

作用：

- 通过 `rs.context().query_devices()` 查询 RealSense 设备。
- 枚举 color sensor 和 depth sensor 支持的 stream profile。
- 只保留可以同时使用的 color/depth 帧率组合。
- USB3 下优先 `bgr8/rgb8`，USB2 下优先更省带宽的格式。

启动时会打印：

```text
Found ... supported low-bandwidth RealSense profile candidates.
```

### 7.5 `make_config()`

作用：

- 根据当前 profile 创建 `rs.config()`。
- 配置 color stream 和 depth stream。

### 7.6 `log_device_info()`

作用：

- 打印相机型号、序列号、固件版本和 USB 类型。

示例：

```text
RealSense device: name=Intel RealSense D435 serial=... firmware=... usb=3.2
```

如果看到 `usb=2.1`，说明相机工作在 USB2 模式，彩色+深度同步流会不稳定，应换 USB3 接口或 USB3 数据线。

### 7.7 `log_actual_stream_profiles()`

作用：

- 打印 pipeline 实际启动的 color/depth stream。

示例：

```text
Actual streams: color=640x480 bgr8@45 depth=640x480 format.z16@45
```

这可以确认 RealSense 实际使用了什么格式、分辨率和帧率。

### 7.8 `convert_color_frame_to_bgr()`

作用：

- 根据 RealSense 实际返回的 frame format 转换成 OpenCV 使用的 BGR 图。

支持：

- `bgr8`：直接使用。
- `rgb8`：用 `cv2.COLOR_RGB2BGR` 转换。
- `yuyv`：用 `cv2.COLOR_YUV2BGR_YUY2` 转换。
- `y8`：灰度图转 BGR。

这部分解决了之前 `yuyv` reshape 尺寸不匹配的问题。

### 7.9 `start()`

作用：

- 启动 RealSense pipeline。
- 如果当前 profile 启动失败，尝试下一个 profile。
- 读取深度比例 `depth_scale`。

深度图原始值需要乘以 `depth_scale` 才是米：

```python
depth_m = depth_raw.astype(np.float32) * self.depth_scale
```

### 7.10 `try_next_profile()`

作用：

- 在当前 profile 超时或启动失败时，切换到下一个候选 profile。

日志示例：

```text
Switching RealSense profile after repeated frame timeouts...
```

### 7.11 `restart_pipeline()`

作用：

- 停止当前 RealSense pipeline。
- 创建新的 `rs.pipeline()`。
- 用当前 profile 重新启动。

### 7.12 `spin()`

作用：

- 主循环。
- 调用 `wait_for_frames()` 等待图像帧。
- 对齐深度到彩色图。
- 转换彩色图为 BGR。
- 将深度图转换成米单位 `32FC1`。
- 发布 ROS 图像消息。

发布编码：

```python
color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
depth_msg = self.bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
```

### 7.13 `stop()`

作用：

- 节点关闭时停止 RealSense pipeline。
- 设置 `self.shutting_down=True`，避免 ROS 因同名节点冲突关闭节点时仍继续读帧。

这部分解决了关闭期间出现：

```text
wait_for_frames cannot be called before start()
```

的附带警告。

## 8. `scripts/yellow_circle_detector.py`

路径：

```text
D:\CUADC\src\d435i_yellow_circle_detector\scripts\yellow_circle_detector.py
```

作用：

- 订阅彩色图和深度图。
- 用 HSV 阈值筛选黄色区域。
- 用轮廓、面积、圆度、半径和中心区域过滤目标。
- 查询圆心深度。
- 发布检测结果和标注图。

### 8.1 `find_contours()`

作用：

- 兼容 OpenCV 3 和 OpenCV 4 的 `cv2.findContours()` 返回值差异。

```python
def find_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 2:
        contours, hierarchy = result
    else:
        _, contours, hierarchy = result
    return contours, hierarchy
```

这部分解决：

```text
ValueError: too many values to unpack (expected 2)
```

### 8.2 `YellowCircleDetector.__init__()`

作用：

- 读取 ROS 参数。
- 创建发布者和订阅者。
- 初始化 HSV 阈值和轮廓过滤参数。

订阅：

```python
self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback, queue_size=1, buff_size=2**24)
self.color_sub = rospy.Subscriber(color_topic, Image, self.color_callback, queue_size=1, buff_size=2**24)
```

发布：

```python
self.result_pub = rospy.Publisher(result_topic, YellowCircle, queue_size=1)
self.annotated_pub = rospy.Publisher(annotated_topic, Image, queue_size=1)
```

### 8.3 `depth_callback()`

作用：

- 接收深度图。
- 转换成 `32FC1` Numpy 数组。
- 保存最新深度图。

使用 `threading.Lock()` 是为了避免深度回调和彩色回调同时读写深度图。

### 8.4 `color_callback()`

作用：

- 接收彩色图。
- 调用 `detect()`。
- 调用 `build_result()`。
- 发布 `/yellow_circle`。
- 发布 `/yellow_circle/annotated_image`。
- 如果 `show_window=true`，弹出 OpenCV 窗口。

### 8.5 `detect()`

作用：

- 黄色圆检测核心函数。

流程：

```text
BGR 图像
  -> HSV 图像
  -> inRange 黄色阈值
  -> 开运算去噪
  -> 闭运算补洞
  -> find_contours 找轮廓
  -> 面积过滤
  -> 圆度过滤
  -> 半径过滤
  -> 中心区域过滤
  -> 选取得分最高目标
  -> 查询深度
  -> 画轮廓、圆心、半径和深度文字
```

黄色过滤：

```python
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
```

形态学处理：

```python
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
```

圆度计算：

```python
circularity = 4.0 * math.pi * area / (perimeter * perimeter)
```

得分：

```python
score = area * circularity - distance_to_center * 2.0
```

含义：

- 面积越大、越圆，得分越高。
- 离图像中心越远，得分越低。

### 8.6 `lookup_depth()`

作用：

- 根据圆心像素 `(x, y)` 查询深度。
- 不是只取一个像素，而是取中心附近 `5x5` 区域有效深度的中位数。

这样可以降低深度噪声影响。

### 8.7 `build_result()`

作用：

- 将检测结果转换为 `YellowCircle` 自定义消息。

如果未检测到目标：

```python
result.detected = False
result.x = -1
result.y = -1
result.depth_m = 0.0
```

如果检测到目标：

```python
result.detected = True
result.x = detection["x"]
result.y = detection["y"]
result.radius = detection["radius"]
result.area = detection["area"]
result.depth_m = detection["depth_m"]
```

中心偏移：

```python
result.center_offset_x = detection["x"] - width * 0.5
result.center_offset_y = detection["y"] - height * 0.5
```

后续做控制时，可以根据偏移量判断目标在画面左侧、右侧、上方还是下方。

## 9. 常改参数

检测不到黄色圆：

```text
h_min, h_max, s_min, s_max, v_min, v_max
```

误检小噪点：

```text
min_area, min_radius
```

误检非圆形黄色物体：

```text
min_circularity
```

目标可能在画面边缘：

```text
center_roi_ratio
```

## 10. 当前已处理的问题

本文档对应的代码已经处理了以下问题：

- USB2 下 RealSense 读帧超时：加入 USB 类型日志、低帧率、profile fallback。
- USB3.2 下 YUYV reshape 错误：改为按实际 frame format 转换。
- OpenCV `findContours()` 返回值不一致：加入 `find_contours()` 兼容函数。
- 同名节点关闭时 RealSense pipeline 已停止：加入 `shutting_down` 标志。
- ROS launch 参数布尔值字符串问题：加入 `get_bool_param()`。
