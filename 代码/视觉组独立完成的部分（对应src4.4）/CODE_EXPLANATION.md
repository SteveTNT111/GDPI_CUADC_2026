# D435i 视觉检测 + 大地坐标变换 ROS1 代码说明

> **版本：** with_geopose（基于旧版 d435i_yellow_circle_detector 升级）
> **适用系统：** Ubuntu 20.04 + ROS Noetic
> **相机：** Intel RealSense D435/D435i，默认 30 FPS

旧版只能输出**相机坐标系**下的目标位置 (x, y, z)。本版在旧版基础上增加了 `target_geopose_node.py`，通过 tf2 + 四元数旋转 + geographiclib，将检测结果完整变换到 **WGS84 大地坐标 (经纬高)**，飞控可以直接用这个坐标导航。

> **版本来源：** 本包对应 `视觉组旧版本代码管理/src4.4(显示假的大地坐标)`。
> src4.4 首次引入了 `target_geopose_node.py` + `GeoTarget.msg`，本版只清理了一个未使用的 import（`tf2_geometry_msgs`），其余代码与 src4.4 一致。
> 4.0→4.1→4.2→4.3→4.3.1→**4.4** 这一支是大地坐标功能的开发迭代链路。

---

## 1. 文件结构

```text
d435i_yellow_circle_detector/                  # ROS 功能包根目录
  CMakeLists.txt                                # CMake 编译配置
  package.xml                                   # ROS 包信息与依赖声明
  launch/d435i_yellow_circle.launch             # 总启动文件（参数开关控制各节点）
  msg/YellowCircle.msg                          # 黄色圆检测结果消息定义（遗留）
  msg/YoloDetection.msg                         # YOLO 单目标检测结果消息定义
  msg/YoloDetections.msg                        # YOLO 多目标检测结果消息定义
  msg/MissionTarget.msg                         # 比赛任务目标消息定义
  msg/GeoTarget.msg                             # ★ 新增：大地坐标目标消息（经纬高）
  scripts/d435i_camera_node.py                  # D435i 相机驱动节点（发布 RGB + 深度 + 内参）
  scripts/yellow_binarizer_node.py              # 黄色区域 HSV 二值化节点（遗留）
  scripts/yellow_circle_detector_node.py        # 黄色圆轮廓检测 + 3D 定位节点（遗留）
  scripts/yolo_detector_node.py                 # ✏️ YOLO 目标检测节点（本版叠加 GPS 显示）
  scripts/target_geopose_node.py                # ★ 坐标变换节点（相机 → 机体 → ENU → WGS84）
  scripts/competition_task_node.py              # 比赛任务仲裁节点（投放区 / 灾情侦察区）
  scripts/annotated_image_viewer_node.py        # 黄色圆检测结果 OpenCV 显示窗口（遗留）
  scripts/binary_image_viewer_node.py           # 黄色二值化 OpenCV 显示窗口（遗留）
  scripts/dataset_image_recorder_node.py        # 数据集采集节点（按帧保存原始图片）
  scripts/start_d435i_record.sh                 # 桌面双击启动时的 ROS 命令脚本
  scripts/install_desktop_launcher.sh           # 桌面快捷方式安装脚本
  D435i_YellowCircle_Record.desktop             # Ubuntu 桌面启动器文件
```

> ⚠️ **注意：** 源码中仍保留黄色圆检测相关脚本（`yellow_binarizer_node.py`、`yellow_circle_detector_node.py` 等），这是旧版遗留。**CUADC 比赛场景中不会出现黄色圆**，实战部署时应只启用 YOLO + geopose 管线。相关清理工作待后续版本完成。

---

## 2. 总体数据流

### 2.1 YOLO 检测管线（比赛主用）

```text
D435/D435i
  → d435i_camera_node.py
  → /d435i/color/image_raw + /d435i/aligned_depth/image_raw + /d435i/color/camera_info
  → yolo_detector_node.py
  → /yolo/detection  (YoloDetection: 最高置信度目标)
  → /yolo/detections (YoloDetections: 全部目标)
```

### 2.2 大地坐标变换管线 ★ 新增

```text
/yolo/detection                    → target_geopose_node.py
/mavros/global_position/global     → target_geopose_node.py  (无人机 GPS)
/mavros/local_position/pose        → target_geopose_node.py  (无人机 ENU 位姿)
                                      ↓
                               tf2: 相机系 → 机体坐标系
                               四元数旋转: 机体 → ENU
                               geographiclib: ENU → WGS84 经纬高
                                      ↓
                              /competition/target_global  (GeoTarget)
```

### 2.3 比赛任务管线

```text
/yolo/detections → competition_task_node.py → /competition/target
/yolo/detection  → target_geopose_node.py   → /competition/target_global
```

### 2.4 黄色圆检测管线（旧版遗留，比赛不用）

```text
D435/D435i
  → d435i_camera_node.py
  → /d435i/color/image_raw
  → yellow_binarizer_node.py
  → /yellow_circle/binary_image
  → yellow_circle_detector_node.py
  → /yellow_circle
  → /yellow_circle/annotated_image
```

### 2.5 显示和采集节点

```text
/yellow_circle/annotated_image → annotated_image_viewer_node.py  (黄色圆窗口, 遗留)
/yellow_circle/binary_image    → binary_image_viewer_node.py     (二值化窗口, 遗留)
/d435i/color/image_raw         → dataset_image_recorder_node.py  (数据集采集)
/yolo/annotated_image          → yolo_detector_node.py           (YOLO窗口, 含GPS叠加)
```

---

## 3. 节点详解

### 节点总览

| 节点 | 类型 | 比赛需要? | 说明 |
|------|------|----------|------|
| `d435i_camera_node.py` | 旧版 | ✅ 必须 | D435i 相机驱动，所有节点的数据源 |
| `yolo_detector_node.py` | ✏️ 修改 | ✅ 必须 | YOLO 目标检测，本版叠加了 GPS 显示 |
| `target_geopose_node.py` | ★ 新增 | ✅ 必须 | 坐标变换核心：相机系→机体→ENU→WGS84 |
| `competition_task_node.py` | 旧版 | ✅ 可选 | 比赛任务仲裁（投放区/灾情侦察区） |
| `dataset_image_recorder_node.py` | 旧版 | ✅ 可选 | 数据集采集，保存原始图片到磁盘 |
| `yellow_binarizer_node.py` | 旧版 | ❌ 冗余 | 黄色区域 HSV 二值化 |
| `yellow_circle_detector_node.py` | 旧版 | ❌ 冗余 | 黄色圆轮廓检测+3D定位 |
| `annotated_image_viewer_node.py` | 旧版 | ❌ 冗余 | 黄色圆检测结果 OpenCV 窗口 |
| `binary_image_viewer_node.py` | 旧版 | ❌ 冗余 | 黄色二值化结果 OpenCV 窗口 |

> **比赛最小运行只需 3 个节点：** 相机 + YOLO + geopose。
> 黄色圆相关的 4 个节点（`yellow_binarizer`、`yellow_circle_detector`、`annotated_image_viewer`、`binary_image_viewer`）是旧版遗留，比赛中不会出现黄色圆，后续版本应删除。

---

### 3.1 d435i_camera_node.py 【旧版】

**类型：** 旧版节点，与旧版 `d435i_yellow_circle_detector` 中的完全一致。

作用：打开 RealSense D435/D435i 并发布 ROS 图像话题。

主要功能：

- 使用 `pyrealsense2` 打开相机。
- 读取彩色图和深度图。
- 将深度图对齐到彩色图。
- 发布 `/d435i/color/image_raw`、`/d435i/aligned_depth/image_raw` 和 `/d435i/color/camera_info`。
- 自动枚举并切换 RealSense 可用 profile, 降低 USB 带宽或格式不匹配导致的启动失败概率。
- 默认使用 `640×480, 30 FPS, bgr8, z16`。

发布：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
/d435i/color/camera_info
```

---

### 3.2 yellow_binarizer_node.py 【旧版 · ⚠️ 比赛无用】

**类型：** 旧版节点，与旧版 `d435i_yellow_circle_detector` 中的完全一致。

作用：把彩色图中的黄色区域提取出来, 生成黑白二值图。

> 比赛场景中不需要此节点。保留仅供地面调试参考。

订阅：`/d435i/color/image_raw`
发布：`/yellow_circle/binary_image`

---

### 3.3 yellow_circle_detector_node.py 【旧版 · ⚠️ 比赛无用】

**类型：** 旧版节点，与旧版 `d435i_yellow_circle_detector` 中的完全一致。

作用：在二值图中识别一个黄色圆, 输出相机坐标系下的三维位置。

> 比赛场景中不需要此节点。保留仅供地面调试参考。

关键逻辑：

- 在二值图中查找外轮廓, 通过面积、半径、圆度和中心 ROI 筛选圆。
- 查询圆心附近深度, 反投影为相机坐标系 `(x, y, z)`。
- 输出相机到目标中心的直线距离 `d`。

订阅：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
/yellow_circle/binary_image
/d435i/color/camera_info
```

发布：

```text
/yellow_circle
/yellow_circle/annotated_image
```

---

### 3.4 yolo_detector_node.py 【✏️ 修改 · 比赛主用】

**类型：** 旧版节点基础上修改。核心检测逻辑未变，新增了 geopose 数据订阅和 GPS 叠加显示。

作用：加载 Ultralytics YOLO 模型, 对 D435i 彩色图进行目标检测。**本版新增了大地坐标叠加显示。**

主要功能：

- 订阅 `/d435i/color/image_raw`、`/d435i/aligned_depth/image_raw` 和 `/d435i/color/camera_info`。
- 加载默认模型 `/home/lab/model/best.pt`。
- 使用 `conf_threshold` 控制置信度阈值, 默认 `0.5`。
- 默认使用 CPU 推理, `imgsz=640`。
- 发布 `/yolo/detection`（最高置信度目标）、`/yolo/detections`（全部目标）。
- 发布 `/yolo/annotated_image`（标注画面）。

**★ 新增 geopose 显示：**

- 订阅 `/competition/target_global`（GeoTarget），获取大地坐标。
- 在 YOLO 窗口上叠加显示：
  - 目标经纬度和海拔（lat/lon/alt）
  - 目标在机体坐标系下的位置（body_x/y/z）
  - 坐标数据随时间衰减，过期（默认 >1s）后自动隐藏

启动示例：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true \
  enable_global_target:=true \
  yolo_model_path:=/home/lab/model/best.pt
```

---

### 3.5 target_geopose_node.py 【★ 新增 · 核心】

**类型：** 本版全新节点，旧版中不存在。这是 `with_geopose` 版本的核心增量。

作用：将 YOLO 检测结果从相机坐标系完整变换到 WGS84 大地坐标系。

**坐标变换链：**

```
相机光学坐标系 (camera_x/y/z_m)
    ↓  tf2 查找 camera_frame → body_frame
机体坐标系 (body_x/y/z_m)
    ↓  四元数旋转 (无人机姿态)
ENU 东北天坐标系 (enu_east/north/up_m)
    ↓  geographiclib Geodesic.Direct
WGS84 大地坐标 (latitude/longitude/altitude)
```

**输入：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/yolo/detection` | YoloDetection | YOLO 最高置信度目标 |
| `/mavros/global_position/global` | NavSatFix | 无人机 GPS 位置 |
| `/mavros/local_position/pose` | PoseStamped | 无人机 ENU 位姿 |

**输出：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/competition/target_global` | GeoTarget | 目标的完整大地坐标 |

**状态码（`target.status`）：**

| 状态 | 含义 |
|------|------|
| `ok` | 变换成功，坐标有效 |
| `no_detection` | YOLO 未检测到目标 |
| `low_confidence` | 置信度低于阈值 |
| `invalid_camera_position` | 相机系位置无效 |
| `no_valid_global_position` | GPS 无固定解 |
| `no_local_pose` | 无 ENU 位姿数据 |
| `tf_camera_to_body_failed` | tf2 变换失败 |

**关键参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `min_confidence` | 0.30 | 最低置信度阈值 |
| `body_frame` | `base_link` | 机体坐标系 frame |
| `camera_frame` | `d435i_color_optical_frame` | 相机光学 frame |
| `transform_timeout_sec` | 0.10 | tf2 查找超时 |
| `publish_invalid` | true | 是否发布无效结果（含状态码） |

**坐标变换实现细节：**

- **tf2 变换：** 先尝试带时间戳的变换，失败后回退到最新可用 tf（`rospy.Time(0)`）。
- **四元数旋转：** 使用自实现函数 `quaternion_rotate_vector()`，不依赖 tf2 旋转，避免额外依赖。
- **ENU→WGS84：** 优先使用 `geographiclib` 库的 `Geodesic.Direct` 方法（高精度），失败时回退到局部平面近似。
- **GPS 有效性检查：** 要求 `NavSatStatus != STATUS_NO_FIX` 且经纬高均为有限值。

**依赖安装（NUC 上）：**

```bash
pip3 install geographiclib
```

---

### 3.6 competition_task_node.py 【旧版 · 可选】

**类型：** 旧版节点，与旧版 `d435i_yellow_circle_detector` 中的完全一致。

作用：把比赛场地规则编码成任务目标输出。

主要功能：

- 订阅 `/yolo/detections`。
- `mission_stage=drop` 时识别投放区白色圆筒, 按 15cm、20cm、25cm 直径分类。
- `mission_stage=disaster` 时优先识别危险化学品标识, 其次识别 20cm 白色圆筒。
- 输出 `/competition/target`。
- 对投放区目标输出 A 区半径和 B 区半径, B 区外径固定 1m。

启动示例：

```bash
# 投放区
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true enable_competition_task:=true mission_stage:=drop

# 灾情侦察区
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch \
  enable_yolo:=true enable_competition_task:=true mission_stage:=disaster
```

---

### 3.7 辅助节点

#### annotated_image_viewer_node.py 【旧版 · ⚠️ 比赛无用】

**类型：** 旧版节点，与旧版完全一致。

订阅 `/yellow_circle/annotated_image`，用 OpenCV `imshow` 弹出黄色圆检测结果窗口。只在 `show_window:=true` 时启动。只显示黄色圆结果，对比赛没用。

#### binary_image_viewer_node.py 【旧版 · ⚠️ 比赛无用】

**类型：** 旧版节点，与旧版完全一致。

订阅 `/yellow_circle/binary_image`，用 OpenCV `imshow` 弹出黄色二值化黑白窗口。只在 `show_window:=true` 时启动。仅用于调试黄色圆 HSV 参数。

#### dataset_image_recorder_node.py 【旧版 · 可选】

**类型：** 旧版节点，与旧版完全一致。

订阅 `/d435i/color/image_raw`，按帧保存原始彩色图到 `~/yellow_circle_dataset/YYYYMMDD_HHMMSS/images/`。后台线程写盘不阻塞相机回调。只在 `enable_record:=true` 时启动。

#### 辅助节点启动条件汇总

| 节点 | 类型 | 启动条件 |
|------|------|---------|
| `annotated_image_viewer_node.py` | 旧版 ⚠️ | `show_window:=true` |
| `binary_image_viewer_node.py` | 旧版 ⚠️ | `show_window:=true` |
| `dataset_image_recorder_node.py` | 旧版 | `enable_record:=true` |

---

## 4. 消息定义

### 4.1 GeoTarget.msg ★ 新增

```yaml
Header header             # 时间戳和坐标系
bool valid                # 大地坐标是否有效
string status             # 状态码: ok / no_detection / low_confidence / ...
string source_topic       # 检测来源话题
string class_name         # 目标类别名
float64 confidence        # 检测置信度
float64 center_x          # 目标中心像素 x
float64 center_y          # 目标中心像素 y
float64 camera_x_m        # 相机光学系 x (m)
float64 camera_y_m        # 相机光学系 y (m)
float64 camera_z_m        # 相机光学系 z (m)
float64 body_x_m          # 机体坐标系 x (m)
float64 body_y_m          # 机体坐标系 y (m)
float64 body_z_m          # 机体坐标系 z (m)
float64 enu_east_m        # ENU 东向偏移 (m)
float64 enu_north_m       # ENU 北向偏移 (m)
float64 enu_up_m          # ENU 天向偏移 (m)
float64 latitude          # 目标纬度 (WGS84)
float64 longitude         # 目标经度 (WGS84)
float64 altitude          # 目标海拔 (WGS84)
```

### 4.2 YoloDetection.msg

```yaml
Header header
bool detected
string class_name
float64 confidence
float64 center_x / center_y
float64 width / height
float64 depth_m
float64 camera_x/y/z_m
float64 distance_m
```

### 4.3 YoloDetections.msg

```yaml
Header header
YoloDetection[] detections
```

### 4.4 MissionTarget.msg

```yaml
Header header
string mission_stage
string target_description
float64 target_diameter_m
float64 a_zone_radius_m
float64 b_zone_diameter_m
```

---

## 5. launch 文件参数

### 5.1 基础参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `show_window` | false | 是否显示 OpenCV 窗口 |
| `fps` | 30 | 相机帧率 |
| `display_scale` | 1.0 | 显示缩放 |

### 5.2 黄色圆参数（遗留）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_diameter_m` | 0.20 | 黄色目标实际直径 (m) |
| `depth_max_valid_m` | 0.35 | 深度可信上限 (m) |
| `close_radius_px` | 42.0 | 过近半径阈值 (px) |
| `close_distance_m` | 0.16 | 过近距离阈值 (m) |

### 5.3 YOLO 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_yolo` | false | 是否启动 YOLO |
| `yolo_model_path` | `/home/lab/model/best.pt` | 模型路径 |
| `yolo_conf_threshold` | 0.5 | 置信度阈值 |
| `yolo_imgsz` | 640 | 推理输入尺寸 |
| `yolo_device` | cpu | 推理设备 |
| `invert_camera_x` | true | YOLO 输出 x 是否反向 |

### 5.4 大地坐标参数 ★ 新增

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_global_target` | false | 是否启动 geopose 节点 |
| `global_target_topic` | `/competition/target_global` | GeoTarget 输出话题 |
| `global_target_max_age_sec` | 1.0 | GPS 数据最大有效期 (s) |
| `body_frame` | `base_link` | 机体坐标系 frame |
| `camera_frame` | `d435i_color_optical_frame` | 相机光学 frame |
| `global_position_topic` | `/mavros/global_position/global` | 无人机 GPS 话题 |
| `local_pose_topic` | `/mavros/local_position/pose` | 无人机 ENU 位姿话题 |

### 5.5 采集参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_record` | false | 是否保存图片 |
| `record_dir` | `~/yellow_circle_dataset` | 图片保存根目录 |
| `record_fps` | 30 | 图片保存帧率 |
| `image_format` | jpg | 图片格式 |
| `jpeg_quality` | 95 | JPG 质量 |

### 5.6 比赛任务参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_competition_task` | false | 是否启动比赛任务节点 |
| `mission_stage` | drop | 比赛阶段: drop / disaster |

---

## 6. 运行命令与部署

常用启动命令和 NUC 部署步骤详见：**[RUN_COMMANDS.md](RUN_COMMANDS.md)**

---

## 7. 与 cuadc_src 的对接

`cuadc_src` 视觉管线中的 `geopose_node.py` 与本包的 `target_geopose_node.py` 功能重叠（都是相机→机体→ENU→大地）。两者关系如下：

| | target_geopose_node.py (本包) | geopose_node.py (cuadc_src) |
|---|---|---|
| 输入 | `/yolo/detection` | 待定（可能是 `/yolo/detection` 或自定义话题） |
| 输出 | `/competition/target_global` | 待定 |
| 所在包 | d435i_yellow_circle_detector | cuadc_vision |
| 用途 | 视觉包自带，独立测试用 | 主系统中统一调度 |

**建议：** 最终部署时选择其中一个即可，避免重复计算。如果主系统由 cuadc_src 统一调度，可由 cuadc_src 的 `geopose_node.py` 订阅本包的 `/yolo/detection` 做坐标变换，本包只负责检测输出。

---

## 8. 旧版升级说明

相对于 `d435i_yellow_circle_detector` 旧版（无 geopose），本版变更如下：

| 文件 | 变更 |
|------|------|
| `CMakeLists.txt` | +geometry_msgs, +tf2_ros, +tf2_geometry_msgs, +GeoTarget.msg, +target_geopose_node.py |
| `package.xml` | +geometry_msgs, +tf2_ros, +tf2_geometry_msgs |
| `launch/d435i_yellow_circle.launch` | +大地坐标参数, +target_geopose_node 条件启动 |
| `msg/GeoTarget.msg` | 新增 |
| `scripts/target_geopose_node.py` | 新增 |
| `scripts/yolo_detector_node.py` | +订阅 GeoTarget, +叠加 GPS 显示 |

其余所有脚本和消息文件与原版字节级一致，黄色圆检测代码未做任何修改。

---

> 以上文档适用于 `视觉组独立完成的部分/d435i_yellow_circle_detector` (2026-07-02)。
