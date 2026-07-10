# detector_node.py — 代码流程框架

> 对应文件：`scripts/detector_node.py`

---

## 一、整体流程（伪代码）

```
┌─────────────────────────────────────────────────────┐
│  启动 (main)                                         │
│    → rospy.init_node("detector_node")                │
│    → DetectorNode()                                  │
│      → __init__()                                     │
│        1. 读 ROS 参数（模型路径、阈值、目标类别等）     │
│        2. 加载 YOLO 模型 (ultralytics.YOLO)           │
│        3. 订阅话题                                    │
│             /vision/color/image_raw       ← image_callback
│             /vision/aligned_depth/image_raw ← depth_callback
│             /vision/color/camera_info      ← camera_info_callback
│        4. 创建发布者                                  │
│             /vision/yolo/detection         (YoloDetection)
│             /vision/yolo/detections        (YoloDetections)
│             /vision/annotated_image         (Image)
│      → rospy.spin()   # 等待回调                      │
└─────────────────────────────────────────────────────┘
```

---

## 二、主回调：image_callback（每一帧触发）

```
image_callback(color_image_msg):
│
├─[1] ROS Image → OpenCV BGR  (cv_bridge)
│
├─[2] YOLO 推理  (self.model(frame, ...))
│     结果 = results[0]
│       ├── boxes  (N 个检测框)
│       │    └── 每个 box: xyxy坐标, cls类别, conf置信度
│       └── names  (类别名映射字典)
│
├─[3] build_results(header, results[0], image_shape)
│       │
│       ├── 遍历每个 box:
│       │   ├── build_single(box)
│       │   │   ├── 提取 xyxy → x_min, y_min, x_max, y_max
│       │   │   ├── 算中心像素 (cx, cy)
│       │   │   ├── 获取 class_name, confidence
│       │   │   ├── lookup_depth(cx, cy)  ← 从深度图中位数取
│       │   │   ├── project_pixel(cx, cy, depth)
│       │   │   │   └── 针孔模型反投影 → camera_x/y/z_m
│       │   │   └── 填充 YoloDetection 消息
│       │   │
│       │   └── _is_target_class(class_name)?
│       │         └── 关键词模糊匹配 (target_classes)
│       │              NO  → 丢弃这个 box
│       │              YES → 加入 detections 列表
│       │
│       └── 返回: (best_msg, detections_msg, best)
│
├─[4] 画标注画面
│   ├── annotated = frame.copy()
│   ├── 遍历每个通过的 detection: cv2.rectangle (黄色框)
│   └── if best: draw_overlay(annotated, best)
│       ├── 圆心 + 十字线 (红色)          ← cv2.circle + line
│       ├── 框 (黄色)                     ← cv2.rectangle
│       ├── 框上方标签文字               ← draw_text_bg()
│       └── 左下角固定面板               ← draw_panel()
│
├─[5] 发布 3 个话题
│
└─[6] if show_window: cv2.imshow("YOLO Detection", annotated)
```

---

## 三、两个文字绘制函数

### 框旁边浮动的文字：`draw_text_bg()`

```
draw_text_bg(image, text, x, y, ...):
│
│  位置：由调用者指定 (x, y)，跟着框走
│  用途：在每个预测框旁边显示 "cylinder 0.85 x=0.12 y=-0.05 z=2.30 d=2.31m"
│
├── 计算文字尺寸 (cv2.getTextSize)
├── 防止越界 (min/max 限定在画面内)
├── 画黑色背景矩形
└── cv2.putText 写文字
```

### 画面周边固定的文字：`draw_panel()`

```
draw_panel(image, lines, img_w, img_h):
│
│  位置：固定在画面左下角 (x0=4, y0=img_h-ph-4)
│  用途：始终显示最佳目标的摘要
│        "Best: cylinder conf=0.85"
│        "x=0.12m y=-0.05m z=2.30m d=2.31m"
│        "center=(320, 240)"
│
├── 计算面板尺寸 (遍历每行文字)
├── 定位：x0=4, y0=max(4, img_h - 面板高度 - 4)
├── 画黑色背景矩形 + 黄色边框
└── 逐行 cv2.putText 写文字
```

---

## 四、数据流图

```
相机
 │
 ├──→ /vision/color/image_raw ──→ image_callback
 │                                    │
 ├──→ /vision/aligned_depth/image_raw │
 │    → depth_callback                │
 │      存储 self.latest_depth ←─────┘ lookup_depth()
 │
 └──→ /vision/color/camera_info
      → camera_info_callback
        存储 self.latest_camera_info ←── project_pixel()

                              推理结果
                                 │
                    ┌────────────┼────────────┐
                    ▼            ▼            ▼
          /vision/yolo    /vision/yolo   /vision
          /detection      /detections    /annotated_image
          (最佳目标)       (全部目标)     (标注画面)
```

---

## 五、关键参数速查

| 参数 | 默认值 | 在代码中哪里 |
|------|--------|------------|
| `model_path` | `$(find cuadc_vision)/models/best.pt` | `__init__` rospkg 查找 |
| `conf_threshold` | 0.5 | `image_callback` → `self.model(frame, conf=...)` |
| `target_classes` | `cylinder,tong,barrel,...` | `build_results` → `_is_target_class()` |
| `show_window` | false | `image_callback` 最后一步 |
| `depth_patch_radius` | 2 | `lookup_depth` median 区域半径 |

---

## 六、你想改文字显示时，改哪里？

| 想改的效果 | 改哪个函数 | 注意 |
|-----------|-----------|------|
| 框上方标签的文字内容 | `draw_overlay()` 里的 `label = "..."` | 第 274 行 |
| 框上方标签的颜色/大小 | `draw_text_bg()` 的 `text_color` / `font_scale` 参数 | 第 285 行调用处 |
| 左下角面板的文字内容 | `draw_overlay()` 里的 `panel = [...]` | 第 277 行 |
| 左下角面板的颜色/位置 | `draw_panel()` 的 `x0, y0` / `(0,255,255)` | 第 306-309 行 |
| 要不要画那个红色圆心十字 | `draw_overlay()` 里 `cv2.circle` + `cv2.line` | 可以注释掉 |
