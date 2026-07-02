# D435/D435i 黄色圆检测 ROS1 代码说明

当前版本按你的要求恢复为 **黄色圆检测**：

- 默认 `30 FPS`。
- 使用 HSV 只提取黄色区域。
- 将黄色区域二值化。
- 在二值图上找轮廓并筛选圆。
- 只输出一个最优黄色圆。

## 1. 数据流

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

## 2. 主要文件

```text
d435i_yellow_circle_detector/
  launch/d435i_yellow_circle.launch
  scripts/d435i_camera_node.py
  scripts/yellow_circle_detector.py
  msg/YellowCircle.msg
```

## 3. `d435i_camera_node.py`

作用：

- 打开 D435/D435i。
- 读取彩色图和深度图。
- 将深度图对齐到彩色图。
- 发布 ROS 图像话题。

当前默认：

```python
fps = int(rospy.get_param("~fps", 30))
```

发布：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
```

## 4. `yellow_circle_detector.py`

作用：

- 订阅彩色图和深度图。
- 将彩色图转 HSV。
- 用 HSV 阈值提取黄色区域。
- 对黄色区域做二值化和形态学处理。
- 找轮廓并筛选圆。
- 发布检测结果、标注图、二值图。

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

圆筛选：

```python
circularity = 4.0 * math.pi * area / (perimeter * perimeter)
```

最终选择评分最高的一个圆：

```python
score = area * circularity - distance_to_center * 2.0
```

## 5. 发布话题

```text
/yellow_circle
/yellow_circle/annotated_image
/yellow_circle/binary_image
```

其中：

- `/yellow_circle`：圆心、半径、面积、深度等结构化结果。
- `/yellow_circle/annotated_image`：原图上画出检测到的圆。
- `/yellow_circle/binary_image`：黄色区域二值图，用于调试 HSV 阈值。

## 6. 最常调的参数

HSV：

```text
h_min, h_max, s_min, s_max, v_min, v_max
```

形态学：

```text
morph_kernel, morph_open_iterations, morph_close_iterations
```

圆筛选：

```text
min_area, max_area, min_radius, min_circularity, center_roi_ratio
```

如果二值图里黄色圆已经很清楚，但没有检测到圆，优先降低 `min_circularity` 或增大 `center_roi_ratio`。
