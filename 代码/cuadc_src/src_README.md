# CUADC SRC — 稳定版功能包

> **包名：** `cuadc_src`（ROS Noetic）
> **用途：** CUADC 2026 全部飞行代码——飞行控制、视觉检测、坐标变换、舵机投放、飞行录像
> **维护：** 伍尚京
>
> 🔒 本文件夹是**稳定版本**。代码来源：队员在自己 NUC 上开发 → 推到仓库自己的文件夹 → 伍尚京 review → 飞机 NUC 验证 → 验证通过后才合并进这里。

---

## 文件结构

```
cuadc_src/
├── scripts/                                   # Python 节点（7个）
│   ├── main.py                                #   主控：状态机 + 模式切换 + 起飞
│   ├── servo_test.py                          #   舵机测试：终端 on/off 控制 CH5/CH6
│   ├── camera_node.py                         #   D435i 相机驱动
│   ├── detector_node.py                       #   YOLO 目标检测
│   ├── geopose_node.py                        #   坐标变换（相机→机体→ENU→WGS84）
│   ├── flight_data_video_recorder_node.py     #   飞行数据录像（解锁自动录）
│   └── auto_drop_node.py                      #   自动抛投触发
├── launch/                                    # 启动文件（6个，每个 .py 一个同名 launch）
│   ├── camera_node.launch                     #   D435i 相机（配合 rviz 查看画面）
│   ├── detector_node.launch                   #   YOLO 检测（相机 + 检测 + OpenCV 窗口）
│   ├── geopose_node.launch                    #   大地坐标变换（待创建）
│   ├── run_main.launch                        #   总启动：主控 + 相机 + 检测 + geopose
│   ├── run_servo_test.launch                  #   舵机测试终端
│   ├── run_flight_recorder.launch             #   飞行数据录像
│   └── auto_drop.launch                       #   自动抛投
├── config/
│   └── params.yaml                            #   全局参数
├── msg/
│   ├── GeoTarget.msg                          #   大地坐标目标
│   ├── MissionStatus.msg                      #   任务状态（弹药/瞄准/抛投事件）
│   ├── YoloDetection.msg                      #   单目标检测
│   ├── YoloDetections.msg                     #   全部检测
│   └── BucketInfo.msg                         #   目标数量 + 像素偏差
├── models/
├── CMakeLists.txt
└── package.xml
```

---

## 依赖安装

### 系统依赖

```bash
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-mavros-msgs \
  ros-noetic-mavros
```

### Python 依赖

```bash
pip3 install pyrealsense2 opencv-python geographiclib ultralytics
```

| 库 | 用途（本项目中） | 学习资源 |
|---|---|---|
| **pyrealsense2** | D435i 相机驱动，获取彩色/深度图像流（`camera_node.py`） | [官方文档](https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html) · [GitHub](https://github.com/IntelRealSense/librealsense) · [SDK 下载](https://www.intelrealsense.com/sdk-2/) |
| **opencv-python** | 图像处理——录像叠加 OS、画面标注、图像编解码（`flight_data_video_recorder_node.py`、`detector_node.py`） | [官方文档](https://docs.opencv.org/) · [PyPI](https://pypi.org/project/opencv-python/) · [GitHub](https://github.com/opencv/opencv-python) |
| **geographiclib** | 大地测量——WGS84 椭球面上的正反算、相机→机体→ENU→经纬高的坐标变换（`geopose_node.py`） | [官方文档](https://geographiclib.sourceforge.io/Python/doc/) · [PyPI](https://pypi.org/project/geographiclib/) · [GitHub](https://github.com/geographiclib/geographiclib-python) |
| **ultralytics** | YOLO 目标检测——模型加载、推理、结果解析（`detector_node.py`） | [官方文档](https://docs.ultralytics.com/) · [PyPI](https://pypi.org/project/ultralytics/) · [GitHub](https://github.com/ultralytics/ultralytics) |

### 编译

```bash
cd ~/catkin_ws
```

```bash
catkin_make
```

```bash
source devel/setup.bash
```

```bash
chmod +x ~/catkin_ws/src/cuadc_src/scripts/*.py
```

---

## 坐标系统详解

> 本项目涉及 **5 个坐标系**，从相机像素一路变换到全球经纬度。
> 理解这些坐标系的定义和它们之间的变换关系，是读懂 `detector_node.py` 和 `geopose_node.py` 的前提。
> 每个小节末尾附有 **gptimage 绘图提示**，复制到 ChatGPT/gptimage 即可生成辅助理解的示意图。

### 坐标系全览

```
┌──────────────────────────────────────────────────────────────────┐
│                     坐标变换全链路                                 │
│                                                                   │
│  像素 (u,v)  ──→  相机光学系  ──→  机体 FRD  ──→  ENU  ──→  WGS84 │
│   (2D 图像)      (3D 右手系)     (前右下)      (东北天)   (经纬高) │
│                                                                   │
│  detector_node   detector_node       TF树      四元数旋转  geographiclib│
│   YOLO 输出      project_pixel    camera→body  body→ENU   ENU→WGS84   │
└──────────────────────────────────────────────────────────────────┘
```

| 序号 | 坐标系 | 英文 | 维度 | 原点 | 轴定义 | 谁来提供/计算 |
|------|--------|------|------|------|--------|-------------|
| ① | 像素系 | Pixel | 2D | 图像左上角 | u→右, v→下 | OpenCV / YOLO |
| ② | 相机光学系 | Camera Optical | 3D | 相机光心 | X→右, Y→下, Z→前 | detector_node 反投影 |
| ③ | 机体坐标系 | Body (FRD) | 3D | 无人机质心 | X→前, Y→右, Z→下 | TF 树变换 |
| ④ | 东北天坐标系 | ENU | 3D | 起飞点 | E→东, N→北, U→上 | MAVROS + 四元数旋转 |
| ⑤ | 大地坐标系 | WGS84 | 2D+1 | 地球椭球 | 纬度, 经度, 海拔 | RTK GPS + geographiclib |

---

### ① 像素坐标系 (Pixel Frame)

> **范围：** 640×480 的 2D 图像 | **所在节点：** `detector_node.py`

```
(0,0) ────────────────────→ u (col, 向右)
  │
  │      · (cx,cy) 光心
  │         ≈ (320, 240)
  │
  ↓
  v (row, 向下)
```

- **原点 (0,0)**：图像左上角
- **u 轴**：水平向右（列号增大），范围 0~639
- **v 轴**：垂直向下（行号增大），范围 0~479
- **光心 (cx, cy)**：相机光轴在图像上的投影点，约 (320, 240)

> YOLO 检测输出的 `x_min, y_min, x_max, y_max` 和 `center_x, center_y` 都在此坐标系下。

---

### ② 相机光学坐标系 (Camera Optical Frame)

> **范围：** 相机前方的 3D 空间（约 0.3~10m） | **所在节点：** `detector_node.py` → `project_pixel()`

```
         Z+ (光轴，从镜头指向场景 / 指向地面)
        ↗
       /
      相 机 —————→ X+ (右)
       |
       ↓
      Y+ (下)
```

**这是标准针孔相机模型 / OpenCV 光学坐标系：**

| 轴 | 正方向 | 画面像素对应 | 定义来源 |
|----|--------|------------|---------|
| **X** | 右 | u 增大方向 | `X = (u - cx) × Z / fx` |
| **Y** | 下 | v 增大方向 | `Y = (v - cy) × Z / fy` |
| **Z** | 前（光轴指向场景） | 深度图数值 | `Z = depth` |

**右手定则验证：** X(右) × Y(下) = Z(前) ✓

**画面位置与坐标正负：**

| 目标在画面中 | X 符号 | Y 符号 | 原因 |
|-------------|--------|--------|------|
| 右边 | **正 (+)** | — | u > cx |
| 左边 | **负 (−)** | — | u < cx |
| 下方 | — | **正 (+)** | v > cy |
| 上方 | — | **负 (−)** | v < cy |
| 正中心 | 0 | 0 | u=cx, v=cy |

> **窗口画面上的坐标轴标记：** 红色箭 → 右 (X+)，绿色箭 ↓ 下 (Y+)。目标在箭头方向数值为正，反方向数值为负。

<details>
<summary><b>📐 gptimage 绘图提示 1：相机光学坐标系示意图</b></summary>

```
A 3D coordinate system diagram showing a camera with three colored axes:
- Red axis pointing RIGHT (labeled "X+ Right / 右")
- Green axis pointing DOWN (labeled "Y+ Down / 下")
- Blue axis pointing FORWARD out of the lens (labeled "Z+ Forward / 前 / 光轴")
- Camera body drawn as a simple box with lens visible on the front face
- A small image plane shown behind the lens, with u-axis right and v-axis down matching X/Y directions
- A target object (cylinder) shown in front of the camera at some offset from the optical axis
- Dashed projection lines from target to image plane, showing pixel coordinates (u,v)
- Label at top: "标准相机光学坐标系 (OpenCV Pinhole Camera Model)"
- Right-hand rule annotation in corner: X × Y = Z
- Style: clean technical 3D illustration, white background, Chinese + English labels
```
</details>

---

### ③ 机体坐标系 (Body Frame / FRD)

> **框架名：** `base_link` | **TF 变换：** `d435i_color_optical_frame` → `base_link`（由外部 TF 树发布）
> **所在节点：** `geopose_node.py` → `transform_camera_to_body()`

#### 3.1 机体坐标系定义 (FRD)

ArduPilot / PX4 使用 **FRD (Forward-Right-Down)** 机体坐标系：

```
         X_body (前)
        ↗
       /
      无人机 —————→ Y_body (右)
       |
       ↓
      Z_body (下)
```

| 轴 | 正方向 | 含义 |
|----|--------|------|
| **X_body** | 前方 | 无人机机头指向 |
| **Y_body** | 右方 | 无人机右侧 |
| **Z_body** | 下方 | 指向地面 |

**右手定则验证：** X(前) × Y(右) = Z(下) ✓

> **FRD vs NED：** FRD（前右下）是机体固连坐标系，随无人机旋转；NED（北东地）是固定在世界上的坐标系。当无人机机头朝北时，FRD 的 (前,右,下) 与 NED 的 (北,东,地) 完全对齐。

#### 3.2 相机→机体的物理安装关系

相机安装在机体**下方**，镜头朝下，相机外壳的物理"上方"朝向机体**前方**：

```
              无人机机体
          ┌──────────────┐
    前 ←  │    (机头)     │  → 后
          │              │
          │   ┌─────┐    │
          │   │D435i│    │  ← 相机在机体下方
          │   │ 相机 │    │     镜头 ↓ 朝地面
          │   │     │    │
          │   └─────┘    │
          │     ↑        │
          │  相机上方     │
          │  朝向机头     │
          └──────────────┘
                ↓ 地面
```

**坐标轴对应关系（相机光学系 → 机体 FRD）：**

| 相机光学轴 | 方向 | 映射到机体 | 推导 |
|-----------|------|-----------|------|
| X_cam (右) | → | **Y_body (右)** | 相机右侧 = 机体右侧 |
| Y_cam (下) | ↓ | **−X_body (后)** | 相机光学"下" = 机体后方 |
| Z_cam (光轴) | ↗ | **Z_body (下)** | 相机光轴指向地面 |

**变换公式：**
```
X_body (前) = −Y_cam     ← 因为相机 −Y (物理上方) 朝向机体前方
Y_body (右) =  X_cam     ← 相机右侧 = 机体右侧
Z_body (下) =  Z_cam     ← 相机光轴 = 指向地面
```

**推导逻辑：**
1. 相机光学 Y 的正方向是"下"（图像 v 增大方向）
2. 相机的物理"上方" = 光学系的 **−Y** 方向
3. 该"上方"朝向机体前方 → −Y_cam = +X_body → **X_body = −Y_cam**

> ⚠️ 这个变换由 TF 树中的静态变换 `d435i_color_optical_frame → base_link` 实现，**不硬编码在代码中**。
> 如果实际安装方向不同，需要更新 TF 静态变换参数，而不是改 detector_node 或 geopose_node。

<details>
<summary><b>📐 gptimage 绘图提示 2：相机→机体安装关系图</b></summary>

```
A side-view technical diagram showing a quadcopter drone with a D435i camera mounted underneath:

- Drone drawn as a simple cross-frame body with 4 arms and motors/propellers
- D435i camera drawn as a rectangular box attached to the bottom center of the drone
- Camera lens facing DOWN toward ground
- Camera "top" side (flat face of the camera body, labeled "相机物理上方") pointing toward drone FRONT

On the camera, draw 3 colored optical axes:
- Red arrow: X_cam → Right (labeled "X_cam 右")
- Green arrow: Y_cam → Down (labeled "Y_cam 下 = 光学Y+")
- Blue arrow: Z_cam → Forward toward ground (labeled "Z_cam 光轴")

On the drone body, draw 3 colored body axes at the drone center:
- Red arrow: X_body → Forward (labeled "X_body 前")
- Green arrow: Y_body → Right, out of page (labeled "Y_body 右")
- Blue arrow: Z_body → Down (labeled "Z_body 下")

Draw mapping arrows/labels showing the correspondence:
- "X_cam → Y_body (右)" connecting camera X to body Y
- "Y_cam → −X_body (后)" connecting camera Y to opposite of body X
- "Z_cam → Z_body (下)" connecting camera Z to body Z

Ground plane shown at bottom with a target object (cylinder/bucket) for context
Label at top: "D435i 安装朝向 — 相机光学系 → 机体 FRD"
Style: clean mechanical side-view illustration, white background, colored axes, Chinese + English labels
```
</details>

---

### ④ ENU 坐标系（东北天 / Local Tangent Plane）

> **数据来源：** MAVROS `/mavros/local_position/pose`
> **所在节点：** `geopose_node.py` → `quaternion_rotate_vector()`

#### 4.1 ENU 坐标系定义

ENU (East-North-Up) 是以**起飞点**为原点的局部切平面坐标系：

```
         N (北)
        ↗
       /
      起飞点 —————→ E (东)
       |
       ↓
      U (上) ← 垂直地面向上
```

| 轴 | 正方向 | 含义 |
|----|--------|------|
| **E (East)** | 东 | 正东方向 |
| **N (North)** | 北 | 正北方向 |
| **U (Up)** | 上 | 垂直地面向上（与重力反向） |

**右手定则验证：** E(东) × N(北) = U(上) ✓

#### 4.2 MAVROS 如何提供 ENU 数据和无人机姿态

MAVROS 从飞控获取 EKF（扩展卡尔曼滤波）的融合估计结果，发布两个关键话题：

| 话题 | 类型 | 字段 | 用途 |
|------|------|------|------|
| `/mavros/local_position/pose` | PoseStamped | `pose.position` → 无人机在 ENU 下的 (east, north, up) 坐标 | 提供 ENU 位置基准 |
| | | `pose.orientation` → 四元数 `(x,y,z,w)` | **机体 FRD → ENU 的旋转** |
| `/mavros/global_position/global` | NavSatFix | `latitude, longitude, altitude` | 无人机当前的 WGS84 坐标 (RTK GPS) |

> **关于四元数 `pose.orientation`：** 它表示从**机体 FRD** 到 **ENU** 的旋转。
> 例如无人机水平放置、机头朝北时，FRD 的 X(前) 与 N(北) 对齐，四元数接近 `(0, 0, 0, 1)`（无旋转）。

#### 4.3 机体→ENU 旋转变换（四元数旋转）

```python
# geopose_node.py 核心代码
# 第 1 步：从 TF 获取目标在机体 FRD 下的坐标
body_point = self.transform_camera_to_body(msg)          # camera → body
bx, by, bz = body_point.point.x, body_point.point.y, body_point.point.z

# 第 2 步：用无人机姿态四元数旋转 body → ENU
east_m, north_m, up_m = quaternion_rotate_vector(
    latest_local_pose.pose.orientation,   # 四元数：机体 FRD → ENU
    (bx, by, bz)                          # 待旋转的机体坐标向量
)
```

**数学含义：** `v_enu = q ⊗ v_body ⊗ q⁻¹`（四元数旋转公式）

**直观对照（无人机水平、不同机头朝向下，机体轴在 ENU 中的方向）：**

| 无人机机头朝向 | 机体 X(前) → ENU | 机体 Y(右) → ENU |
|--------------|-----------------|-----------------|
| 朝北 | N (北) | E (东) |
| 朝东 | E (东) | S (南 = −N) |
| 朝南 | S (南 = −N) | W (西 = −E) |
| 朝西 | W (西 = −E) | N (北) |

<details>
<summary><b>📐 gptimage 绘图提示 3：机体 FRD → ENU 四元数旋转</b></summary>

```
A top-down 2D diagram illustrating how the drone body frame rotates within the ENU frame:

Left half — ENU World Frame:
- Origin at bottom-left marked "起飞点 (ENU Origin)"
- East axis (E) drawn as horizontal arrow to the right, labeled "E (东)"
- North axis (N) drawn as vertical arrow upward, labeled "N (北)"
- Up axis (U) noted as "out of page ↑"
- Label: "ENU 世界坐标系（固定在地球上）"

Right half — Drone Body Frame:
- A drone icon (top-down view, X-shaped quadcopter) drawn at some angle
- Two colored body axes overlaid on the drone:
  - Red arrow: X_body pointing forward from drone nose, labeled "X_body (前)"
  - Green arrow: Y_body pointing right from drone, labeled "Y_body (右)"
- The yaw angle between North and drone heading clearly marked with an arc, labeled "偏航角 (Yaw)"
- A target cylinder drawn at some position relative to the drone
- A vector arrow from drone center to target, labeled "目标机体偏移 (bx, by, bz)"

Bottom section — The Rotation:
- Formula box: "q ⊗ v_body ⊗ q⁻¹ = v_enu"
- Two result vectors shown in ENU frame:
  - "(east, north) = 四元数旋转后的 ENU 偏移"
  - Arrow emphasizing this is what gets added to drone GPS position

Label at top: "机体 FRD → ENU 旋转变换 (四元数)"
Style: clean top-down 2D technical diagram, colored axes, Chinese + English labels
```
</details>

---

### ⑤ WGS84 大地坐标系

> **数据来源：** RTK GPS 模块 → 飞控 → MAVROS `/mavros/global_position/global`
> **所在节点：** `geopose_node.py` → `enu_offset_to_geodetic()`

#### 5.1 WGS84 坐标系定义

WGS84 (World Geodetic System 1984) 是 GPS 使用的全球大地坐标系：

```
                      北极 (90°N)
                         |
            (-) 西经 ← — + — → (+) 东经
                         |
                      赤道 (0°)
                         |
                      南极 (90°S)
```

| 分量 | 符号 | 单位 | 范围 | 说明 |
|------|------|------|------|------|
| **纬度 (Latitude)** | φ / lat | 度 (°) | −90° ~ +90° | 赤道面与法线的夹角 |
| **经度 (Longitude)** | λ / lon | 度 (°) | −180° ~ +180° | 本初子午面与当地子午面的夹角 |
| **海拔 (Altitude)** | h / alt | 米 (m) | 任意 | 沿椭球法线到 WGS84 参考椭球面的距离 |

> ⚠️ **GPS 海拔 ≠ 气压计高度。** GPS 海拔是相对于 WGS84 椭球面（数学曲面），不是海平面也不是地面。

#### 5.2 RTK GPS 精度

| GPS 模式 | 水平精度 | 垂直精度 | 本项目使用 |
|----------|---------|---------|-----------|
| 普通 GNSS | 2~5m | 5~10m | — |
| RTK Float | 20~50cm | 50cm~1m | 过渡状态 |
| **RTK Fixed** | **1~3cm** | **2~5cm** | ✅ 比赛用 |

> 比赛要求使用 RTK Fixed 模式。RTK 通过地面基站发送差分改正数给无人机上的 RTK 接收机（流动站）来实现厘米级定位。
>
> **RTK 通信方式：纯单向广播**
>
> ```
> 基准站（地面）                    流动站①（无人机）
>    📡 ——— RTCM 广播 ———→         📡 接收
>                          →       流动站②（无人机）
>                          →       流动站③（无人机）
>                          →       不限数量，只管听
>
>          ← 没有任何回传，不需要双向连接 ←
> ```
>
> 基准站只管往天空喊 RTCM 差分数据，飞机上的流动端只管接收——**不需要建立双向通信链路**。这和 FM 收音机一个道理：电台广播，收音机收听，收音机不需要跟电台说话。
>
> ⚠️ 注意区分：P9 数传模块本身**具备**双向通信能力（做 MAVLink 数传时 GCS ↔ 飞机互通），但在 RTK 基站模式下它被配置为 **纯发送模式**（只发不收）。另外，如果你用 4G NTRIP 方式从千寻/CORS 站获取差分数据，那个走的是 TCP 双向连接——但你们用的无线电 RTK 不需要。
>
> **RTK 信号广播距离（CUAV C-RTK Base）：**
>
> | 距离指标 | 数值 | 含义 |
> |---------|------|------|
> | **数传广播距离** | 最远 **30km** | 内置 P9 数传模块的无线电传输能力（理想视距） |
> | **RTK 有效精度距离** | **10km 以内** | 超过此距离，定位精度从厘米级退化到亚米级 |
> | **比赛实际距离** | 通常 500m~2km | 你们的比赛场地尺度，信号完全够用 |
>
> RTK 基站工作模式是**点对多点广播**——一个基站可以同时服务不限数量的无人机。但注意：30km 是理想视距条件（两端天线无遮挡、架设高度足够），实际中受地形、电磁干扰和天线高度影响会打折扣。比赛场地范围内（通常 < 2km）只要基站天线架好，信号完全没问题。

#### 5.3 ENU → WGS84 变换 (Vincenty 正算)

```python
# geopose_node.py 核心代码
# 以无人机当前 GPS 坐标为参考原点
origin_lat = latest_global.latitude
origin_lon = latest_global.longitude
origin_alt = latest_global.altitude

# 第 1 步：ENU 偏移 → 水平距离 + 方位角
horizontal_m = sqrt(east_m² + north_m²)      # 水平距离 (m)
azimuth_deg = atan2(east_m, north_m)          # 方位角 (度，从北顺时针)

# 第 2 步：Vincenty 正算 → WGS84 经纬度
result = Geodesic.WGS84.Direct(
    origin_lat, origin_lon, azimuth_deg, horizontal_m
)
target_lat = result["lat2"]        # 目标纬度
target_lon = result["lon2"]        # 目标经度
target_alt = origin_alt + up_m     # 目标海拔 = 无人机海拔 + 高度差
```

**计算流程可视化：**
```
  ENU (east, north)                      大地坐标
       │                                     │
       ├─→ horizontal_m = √(e²+n²)           │
       ├─→ azimuth = atan2(east, north)      │
       │                                     ↓
       └─→ Geodesic.WGS84.Direct ──→ (lat, lon)
                                         alt = origin_alt + up
```

> `geographiclib` 使用 Vincenty 算法在 WGS84 椭球面上进行高精度大地线正算（给定起点、方位角、距离 → 终点经纬度），精度可达 0.1mm。

<details>
<summary><b>📐 gptimage 绘图提示 4：ENU 局部坐标 → WGS84 大地坐标</b></summary>

```
A diagram showing the transition from local ENU plane to global WGS84 coordinates:

Left side — Local ENU Plane:
- A flat square plane representing the local tangent plane
- Origin point at center marked "起飞点 (ENU Origin = drone's GPS)"
- Drone icon shown at some position on this plane
- Target cylinder shown at a different position
- Vector from drone to target labeled "ENU 偏移 (east_m, north_m, up_m)"
- East and North axes drawn on the plane

Right side — WGS84 Globe:
- A curved cross-section of the Earth showing the WGS84 ellipsoid
- Drone position marked on the surface with GPS coordinates visible
- A curved geodesic line from drone position to target position
- The geodesic labeled "Vincenty Direct (大地线正算)"
- Target position marked with its own GPS coordinates
- Annotation showing: azimuth angle from north, horizontal distance along geodesic

Middle — Transformation Box:
- "atan2(east, north) → azimuth"
- "√(east²+north²) → distance"
- "Geodesic.WGS84.Direct(lat, lon, azimuth, distance) → (lat_target, lon_target)"
- "alt_target = alt_drone + up"

Label at top: "ENU 局部坐标 → WGS84 大地坐标 (geographiclib Vincenty 正算)"
Style: clean split diagram with flat plane + curved Earth, labeled formulas
```
</details>

---

### 完整变换链路：从像素到经纬度

```
┌──────────────────────────────────────────────────────────────────────┐
│                      完整坐标变换链路                                   │
│                                                                       │
│  Step 1           Step 2           Step 3          Step 4    Step 5   │
│  detector_node    TF 查询          geopose_node    geopose_node       │
│                                                                       │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐ ┌─────┐│
│  │ 像素系    │    │ 相机光学系 │    │ 机体 FRD  │    │ ENU 坐标系│ │WGS84││
│  │ (u,v)    │───→│ X,Y,Z    │───→│ X,Y,Z    │───→│ E,N,U    │─→│经纬││
│  │ 2D 图像   │    │ 3D 右手系  │    │ 前右下     │    │ 东北天     │ │度高││
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘ └─────┘│
│       ↑               ↑               ↑               ↑         ↑     │
│   针孔反投影       TF 静态变换      四元数旋转      Vincenty正算  RTK  │
│   X=(u-cx)Z/fx   camera→body     body→ENU       ENU→WGS84     GPS   │
│   Y=(v-cy)Z/fy                                                        │
│   Z=depth                                                             │
└──────────────────────────────────────────────────────────────────────┘
```

#### 每一步的输入/输出/实现

| 步骤 | 变换 | 输入 | 输出 | 实现位置 | 关键依赖 |
|------|------|------|------|---------|---------|
| 1 | 像素 → 相机 | (u,v,depth) | (X_cam, Y_cam, Z_cam) | `detector_node.py:project_pixel()` | 相机内参 fx,fy,cx,cy |
| 2 | 相机 → 机体 | (X_cam, Y_cam, Z_cam) | (X_body, Y_body, Z_body) | `geopose_node.py:transform_camera_to_body()` | TF 树 camera→body |
| 3 | 机体 → ENU | (X_body, Y_body, Z_body) | (E, N, U) | `geopose_node.py:quaternion_rotate_vector()` | MAVROS 四元数 |
| 4 | ENU → WGS84 | (E, N, U) + 无人机 GPS | (lat, lon, alt) | `geopose_node.py:enu_offset_to_geodetic()` | MAVROS GPS + geographiclib |

#### GeoTarget 消息承载所有中间结果

`geopose_node` 输出的 `GeoTarget` 消息包含了变换链上**每一步**的坐标值，方便调试：

```
camera_x_m, camera_y_m, camera_z_m    # Step 1 输出：相机光学系
body_x_m, body_y_m, body_z_m          # Step 2 输出：机体 FRD
enu_east_m, enu_north_m, enu_up_m     # Step 3 输出：ENU 偏移
latitude, longitude, altitude          # Step 4 输出：WGS84 大地坐标
```

#### 数据源汇总

| 数据            | 来源              | 话题                                    | 提供节点            |
| ------------- | --------------- | ------------------------------------- | --------------- |
| 彩色图像          | D435i RGB 相机    | `/vision/color/image_raw`             | `camera_node`   |
| 深度图           | D435i Stereo IR | `/vision/aligned_depth/image_raw`     | `camera_node`   |
| 相机内参          | D435i 标定        | `/vision/color/camera_info`           | `camera_node`   |
| 无人机 GPS (RTK) | MAVROS          | `/mavros/global_position/global`      | 外部 (飞控)         |
| 无人机 ENU 位姿    | MAVROS (EKF2)   | `/mavros/local_position/pose`         | 外部 (飞控)         |
| 相机→机体 TF      | TF 静态变换         | `d435i_color_optical_frame→base_link` | 外部 (TF 树)       |
| YOLO 检测结果     | YOLOv8 推理       | `/vision/yolo/detection`              | `detector_node` |
| 大地坐标目标        | 全链路输出           | `/vision/target_global`               | `geopose_node`  |

<details>
<summary><b>📐 gptimage 绘图提示 5：完整变换链路全景图</b></summary>

```
A comprehensive horizontal pipeline diagram showing the full coordinate transformation chain:

Layout: 5 connected stages in a horizontal row, with data flowing left to right

Stage 1 [PIXEL — 像素系]:
- Icon: camera image frame with pixel grid overlay
- Shows: (u,v) coordinates, bounding box drawn around a detected cylinder
- Arrow pointing to next stage labeled "project_pixel()"

Stage 2 [CAMERA — 相机光学系]:
- Icon: camera body with colored 3D axes (X red→right, Y green→down, Z blue→forward)
- Formula box: "X=(u-cx)·Z/fx, Y=(v-cy)·Z/fy, Z=depth"
- Output box: "(X_cam, Y_cam, Z_cam)"
- Arrow pointing to next stage labeled "TF: camera→body"

Stage 3 [BODY — 机体 FRD]:
- Icon: top-down drone silhouette with body axes (X red→forward, Y green→right, Z blue→down)
- Transform box: "X_body=−Y_cam, Y_body=X_cam, Z_body=Z_cam"
- Output box: "(X_body, Y_body, Z_body)"
- Arrow pointing to next stage labeled "Quaternion Rotate"

Stage 4 [ENU — 东北天]:
- Icon: compass rose showing East and North
- Transform box: "v_enu = q ⊗ v_body ⊗ q⁻¹"
- Output box: "(east, north, up)"
- Arrow pointing to next stage labeled "Vincenty Direct"

Stage 5 [WGS84 — 大地坐标]:
- Icon: small globe or Earth sphere with lat/lon grid
- Transform box: "Geodesic.WGS84.Direct(...)"
- Output box: "(latitude, longitude, altitude)"
- Target marker on globe surface

Below the pipeline, a data source row showing:
- "D435i" → camera intrinsics + depth
- "飞控 EKF2" → attitude quaternion + GPS (via MAVROS)
- "TF 树" → camera→body static transform
- "RTK GPS" → centimeter-accurate global position

Color scheme: each stage uses a distinct accent color:
- Stage 1: gray (image)
- Stage 2: orange (camera)
- Stage 3: blue (body)
- Stage 4: green (ENU)
- Stage 5: purple (WGS84)

Label at top: "坐标变换全链路：像素 → 相机光学 → 机体 FRD → ENU → WGS84"
Subtitle: "detector_node ──→ geopose_node"
Style: clean horizontal pipeline, professional technical diagram, Chinese + English labels
```
</details>

---

### 坐标系快速对照表

| 问题 | 答案 |
|------|------|
| 相机光学系的 Y+ 是哪个方向？ | **下**（OpenCV 图像 v 增大方向） |
| 相机光学系与机体 FRD 的关系？ | X_cam→Y_body(右), Y_cam→−X_body(后), Z_cam→Z_body(下) |
| ENU 的 U 是什么方向？ | **Up (上)**，垂直地面向上 |
| ENU 的原点在哪里？ | **起飞点**（MAVROS 的 local origin / EKF2 原点） |
| 四元数旋转做了什么？ | 把机体 FRD 下的目标偏移向量转到 ENU 坐标系 |
| RTK 提供什么精度的位置？ | RTK Fixed 模式水平 1~3cm，垂直 2~5cm |
| NED 和 FRD 的区别？ | NED 固定在地球上（北东地），FRD 固定在机身上（前右下），两者经偏航旋转后对齐 |
| 如果相机安装方向变了，改哪里？ | 修改 TF 树中 `d435i_color_optical_frame → base_link` 的静态变换 |

---

## 节点详解

> 每个节点都配好了 launch 文件，直接用 `roslaunch` 一行命令启动，不需要手动 `rosrun`。

---

### 1. main.py — 飞行主控

**启动文件：** `run_main.launch`

**功能：** 通过 MAVROS 控制无人机完成全自动飞行流程。内部是一个状态机，依次执行：等待连接 → 检查 EKF/GPS → 切换 GUIDED 模式 → 解锁 → 起飞 → 悬停待命 → 执行任务（自动瞄准 + 自动抛投）→ 着陆/返航。集成了舵机控制功能和弹药管理。支持 `ground_test:=true` 地面测试模式，跳过 EKF 和起飞，解锁后直接进入 MISSION 状态。

**MISSION 状态自动任务：**
1. **自动切 GUIDED**：检测到目标数量 > 0 且当前非 GUIDED 模式 → 自动切换到 GUIDED
2. **自动抛投**：相机系 XY 偏移均 < 10cm → 开始 3 秒稳定计时 → 稳定后触发抛投 → 5 秒冷却

**发布话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vision/mission_status` | MissionStatus | 弹药数量 + 瞄准状态 + 抛投事件 |

#### `/vision/mission_status` 消息详解 (MissionStatus)

此消息由 main.py 以 10Hz 发布，detector_node 订阅后用于画面显示。

| 字段          | 类型     | 含义                      |
| ----------- | ------ | ----------------------- |
| `ammo_a`    | uint8  | 前抛投器 (A) 剩余弹药数，0=无/未挂载  |
| `ammo_b`    | uint8  | 后抛投器 (B) 剩余弹药数，0=无/未挂载  |
| `aiming`    | bool   | 飞控处于 GUIDED 模式且正在执行对准任务 |
| `last_drop` | string | 最近一次抛投的抛投器编号，"A"/"B"/"" |

> **实现原理：** main.py 在状态机主循环中每 10Hz 调用 `_publish_status()`，将当前弹药、瞄准状态和抛投事件打包发送。detector_node 收到 `last_drop` 非空时触发 3 秒的 "A DROP!!!" 显示。main.py 在 3 秒后自动将 `last_drop` 清空。

**测试什么：** 验证飞控与机载电脑的 MAVROS 通信是否正常，验证 GUIDED 模式下起飞→悬停→着陆的完整链路。

**启动后你可以：**
- 手动模式（默认）：启动后终端会打印当前状态，用遥控器或 QGC 手动解锁起飞，main.py 会跟随状态变化
- 自动模式：加 `auto_arm:=true auto_takeoff:=true`，启动后无人机会自动完成解锁→起飞→悬停，无需遥控器干预
- **地面测试模式**：加 `ground_test:=true`，跳过 EKF 检查 + 跳过 GUIDED 切换和起飞，解锁后直接进入 MISSION 状态。适用于室内手持飞机对准圆筒测试自动抛投逻辑（见下方"室内地面测试流程"）

**启动命令：**

**① 手动模式**——安全第一，调试用：

```bash
roslaunch cuadc_vision run_main.launch
# 等价于 auto_arm:=false auto_takeoff:=false
# 启动后什么都不会自动发生，需要你用遥控器操作
```

**② 自动起飞**——比赛用：

```bash
roslaunch cuadc_vision run_main.launch auto_arm:=true auto_takeoff:=true
```

**③ 自动起飞 + 指定高度 + 自定义弹药：**

```bash
roslaunch cuadc_vision run_main.launch \
  auto_arm:=true auto_takeoff:=true takeoff_altitude:=15.0 \
  ammo_a:=2 ammo_b:=0
```

**④ 地面测试模式**（室内手持测试，不飞行）：

```bash
roslaunch cuadc_vision run_main.launch ground_test:=true
```

或者只用 rosrun 启动 main.py 单独测试：

```bash
rosrun cuadc_vision main.py _ground_test:=true
```

**依赖：** 必须先启动 MAVROS（`roslaunch mavros apm.launch`），飞控已连接且 GPS 有 fix。

**验证是否正常：** 终端应依次打印 `飞控已连接 → EKF 就绪 → 解锁成功 → 起飞指令已发送 → 到达目标高度`。MISSION 状态下检测到目标后终端打印 `检测到 N 个目标 → 自动切换 GUIDED 模式`。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `takeoff_altitude` | 10.0 | 起飞目标高度 (m) |
| `auto_arm` | false | true = 启动后自动解锁 |
| `auto_takeoff` | false | true = 解锁后自动起飞 |
| `ground_test` | false | true = 地面测试模式（跳过 EKF + 跳过起飞，解锁后直接进入 MISSION） |
| `enable_auto_guided` | true | 检测到目标时自动切 GUIDED |
| `enable_auto_drop` | true | 对准后自动抛投 |
| `drop_align_threshold_m` | 0.10 | 对准 XY 阈值 (m)，即 10cm |
| `drop_stable_time_s` | 3.0 | 稳定对准计时 (s) |
| `drop_cooldown_s` | 5.0 | 两次抛投冷却间隔 (s) |
| `ammo_a` | 1 | 前抛投器 (A) 初始弹药数 |
| `ammo_b` | 0 | 后抛投器 (B) 初始弹药数 (0=未挂载) |
| `enable_ch5` | true | 启用 SERVO5 舵机通道 |
| `enable_ch6` | false | 启用 SERVO6 舵机通道 |
| `pwm_open` | 2000 | 抛投器打开 PWM (μs) |
| `pwm_close` | 1000 | 抛投器关闭 PWM (μs) |
| `servo_hold_s` | 0.8 | 打开后保持时间 (s) |

#### 室内地面测试流程（ground_test 模式）

> 适用于室内无 GPS 环境下，手持飞机对准圆筒测试自动抛投逻辑。不需要飞行。

**状态机路径（地面测试模式）：**

```
INIT → PREARM → 跳过 EKF → 等待手动解锁 → ARMED → 直接进入 MISSION
                                                              │
                                              ┌───────────────┘
                                              ▼
                                    _check_auto_guided()  ← 检测到目标 → 切 GUIDED（室内可能失败，不影响抛投）
                                    _check_auto_drop()    ← 对准稳定 3s → 触发舵机抛投
```

**操作步骤：**

```bash
# 1. 启动 MAVROS（终端 1）
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600

# 2. 启动 main.py 地面测试模式（终端 2）
rosrun cuadc_vision main.py _ground_test:=true

# 3. 启动相机 + 检测（终端 3）
roslaunch cuadc_vision detector_node.launch show_window:=true
```

**4. 操作飞控：**
- 按飞控上的安全开关（safety switch），灯变为常亮
- 遥控器解锁（STABILIZE 或 ALTHOLD 模式即可，无需 GPS）
- 终端打印 `地面测试模式：解锁后直接进入 MISSION 状态`

**5. 测试自动抛投：**
- 手拿起飞机，将 D435i 相机对准圆筒
- 观察 detector_node 窗口：目标出现黄色框，左下角 `detected: 1`
- 观察 main.py 终端：
  ```
  检测到 1 个目标 → 自动切换 GUIDED 模式
  目标已对准 | x=0.023 y=-0.015 (< 10cm) | 开始 3.0s 稳定计时...
  稳定对准 3.0s | x=0.023 y=-0.015 | 触发抛投！
  舵机 CH5 → 打开 (PWM=2000)
  舵机 CH5 → 关闭 (PWM=1000)
  抛投完成！A 抛投器 | 剩余 A=0 B=0 | 冷却 5.0s
  ```

> **注意：**
> - 室内无 GPS，GUIDED 模式切换大概率失败（终端会打印 `飞行模式切换失败`），这是正常现象，**自动抛投不依赖 GUIDED 模式**
> - 地面测试时 `enable_auto_guided` 可以设为 false 跳过 GUIDED 切换尝试：`_ground_test:=true _enable_auto_guided:=false`
> - 舵机动作需要飞控已解锁 + 安全开关已解除，否则 MAVLink 舵机指令可能被飞控忽略
> - 测试完毕后 `Ctrl+C` 退出，遥控器上锁

---

#### 已知 Bug 与修复记录

##### Bug #1: `_wait_for_ekf()` 崩溃 — AttributeError: 'Pose' object has no attribute 'covariance'

**发现时间：** 2026-07-06

**现象：** main.py 启动后，在 PREARM 状态调用 `_wait_for_ekf()` 时崩溃：
```
AttributeError: 'Pose' object has no attribute 'covariance'
```

**原因：** `/mavros/local_position/pose` 话题类型是 `geometry_msgs/PoseStamped`，其 `pose` 字段是 `geometry_msgs/Pose` 类型（只含 `position` 和 `orientation`），**没有 `covariance` 字段**。原代码 `pose.pose.covariance[0]` 试图访问不存在的属性。

协方差字段只存在于 `geometry_msgs/PoseWithCovariance` 类型中，而 MAVROS 的 `/mavros/local_position/pose` 发布的是不含协方差的 `PoseStamped`。

**修复：** 将 EKF 检查逻辑从"读协方差矩阵"改为"检查位置数值是否已更新（非全零）"。修改后的代码在 `main.py:_wait_for_ekf()`：

```python
# 修复前（错误）：
cov = pose.pose.covariance      # Pose 没有 covariance 属性！
if cov[0] > 0.0: ...

# 修复后（正确）：
pos = self.current_pose.pose.position
has_pose = abs(pos.x) > 0.001 or abs(pos.y) > 0.001 or abs(pos.z) > 0.001
if has_pose and self.current_state.connected:
    return True  # EKF 就绪
```

同时添加了 `ground_test` 参数支持室内无 GPS 场景（见上方"室内地面测试流程"）。

---

### 2. servo_test.py — 舵机测试

**启动文件：** `run_servo_test.launch`

**功能：** 通过 MAVLink `MAV_CMD_DO_SET_SERVO` 指令直接控制飞控舵机输出引脚（SERVO5），不经过飞控逻辑（无需设置 SERVOx_FUNCTION）。单舵机联动双抛投器：PWM 1000 = 全关，1500 = 仅第一个开，2000 = 全开。

**飞控参数要求：** `SERVO5_FUNCTION` / `SERVO6_FUNCTION` 保持 **0 (Disabled)**，飞控不碰这个引脚。`RC_OVERRIDE_TIME` 必须 > 0（默认 3 即可），设成 0 会拒绝所有外部指令。

**测试什么：** 验证舵机接线是否正确、PWM 值是否能驱动抛投器、机械结构是否顺畅。**不需要飞机起飞，地上就能测。**

**启动后你可以：**
- 在终端输入 `on` → 舵机转到 PWM 2000，两个抛投器都打开
- 在终端输入 `off` → 舵机转到 PWM 1000，两个抛投器都关闭
- 输入 `q` 退出
- 也可以通过 ROS 话题远程控制（见下方）

**启动命令：**

```bash
# ① 默认：只控制 CH5（一个舵机驱动两侧抛投器）
roslaunch cuadc_vision run_servo_test.launch
# 启动后终端显示 "servo>" 提示符，输入 on/off 控制
```

```bash
# ② 双通道独立控制（CH5 左 + CH6 右）
roslaunch cuadc_vision run_servo_test.launch enable_ch6:=true
```

```bash
# ③ 通过 ROS 话题控制（另开终端，无需交互）
rostopic pub /servo/cmd std_msgs/String "data: 'on'"
```

```bash
rostopic pub /servo/cmd std_msgs/String "data: 'off'"
```

**验证是否正常：** 输入 `on` 后舵机应转动并保持，终端打印 `舵机 CH5 → 打开 (PWM=1400)`。舵机不动则检查飞控是否上电、通道映射是否正确。

---

### 3. camera_node.py — D435i 相机驱动

**启动文件：** `camera_node.launch`

**功能：** 打开 Intel RealSense D435i 深度相机，发布彩色图像、对齐后的深度图像、相机内参。

### D435i 硬件组成

D435i 正面有三个"眼睛"，很容易搞混：

```
┌─────────────────────────────────┐
│   D435i 正面                     │
│                                 │
│  ┌──┐      ┌──────┐      ┌──┐  │
│  │左│      │ 中间  │      │右│  │
│  │IR│      │ IR投影│      │IR│  │
│  │相│      │  +    │      │相│  │
│  │机│      │ 点阵  │      │机│  │
│  └──┘      └──────┘      └──┘  │
│  (大圆)    (最大的那个)    (大圆) │
│                                 │
│            ┌──────┐             │
│            │ RGB  │             │
│            │ 相机 │ ← 这个最小 │
│            └──────┘             │
└─────────────────────────────────┘
```

| 组件 | 外观 | 干什么的 |
|------|------|---------|
| **左 IR 相机** | 大圆镜片 | 拍红外灰度图（左眼） |
| **右 IR 相机** | 大圆镜片 | 拍红外灰度图（右眼） |
| **IR 点阵投影器** | 中间最大、带红色滤光片 | 往场景投射不可见的红外散斑点阵 |
| **RGB 相机** | 旁边最小的那个 | 拍普通彩色照片 |

> **D435i 不是结构光相机。** 它的原理是 **主动红外立体视觉（Active Stereo）**：红外投影器往物体表面打散斑点阵（增加纹理），左右两个 IR 相机分别拍下来，然后像人眼一样做立体匹配算出每个像素的距离。深度不是从那个最大的中间模组直接"拍"出来的——它是左右 IR 相机算出来的。

### 深度到底是怎么测出来的？

你的理解是对的——**测距靠的就是左右两个 IR 相机的视差（disparity）**，和人的双眼测距原理完全一样：

```
        物体表面的一个点
              ●
             / \
            /   \
           /     \
          /       \
     左IR相机    右IR相机
         ↑         ↑
      左眼看到   右眼看到
      偏右一点   偏左一点
              ↓
        视差 → 距离
```

**那 IR 投影器是干什么的？** 它只是"辅助打光"——往场景上投射随机散斑点阵，给白墙、地板、纯色物体这些没有纹理的表面强行加上纹理。没有投影器，左右 IR 相机面对一面白墙时找不到匹配点，深度图就会有空洞（对应区域深度=0）。

> 类比：你在纯白的桌面上放一个纯白的球，闭上一只眼看——球和桌面融为一体，看不出球的边界。但如果在球和桌面上撒一把芝麻，立刻就能看出球的立体形状了。IR 投影器就是在"撒芝麻"。

### 为什么做视觉 SLAM 要用胶带遮住 IR 投影器？

有几种情况：

**1. 多相机互相干扰（最常见）**
两台或更多 D435i 同时工作，每台都在往场景投射红外斑点。相机 A 的 IR 相机看到的不只是自己投影器的图案，还有相机 B 的图案——两个图案混在一起，立体匹配完全乱掉，深度图变成雪花噪点。解决办法就是把多余的投影器遮住，只留一台开投影。

**2. 室外使用**
阳光里含有大量红外线，直接把 IR 投影器的微弱斑点淹没了——就像在正午太阳下用手电筒，等于没有。遮不遮没区别，但户外场景纹理丰富（草地、建筑、树木），被动立体就够了。

**3. 用 IR 图像做视觉里程计（VO）**
少数 SLAM 方案直接拿 IR 相机的灰度图做特征跟踪。投影器的斑点在连续帧之间会随相机运动而漂移（它们固定在投影器上，不是固定在场景上），对特征匹配是纯噪声。遮掉反而更准。

> **对你们的比赛来说：** 只用一个 D435i、在室外飞——**不需要遮**。户外红外被阳光淹没，投影器本来就没什么用，遮不遮效果一样。比赛场景（草地、跑道、圆筒）纹理丰富，被动立体完全够用。

### 那结构光相机是什么？

结构光相机（Structured Light）是另一种技术路线，和 D435i 完全不同：

| | **结构光** | **D435i（主动立体）** |
|------|-----------|----------------------|
| 投影内容 | 已知的编码图案（条纹/网格） | 随机散斑点阵 |
| 接收端 | **单台**相机 | **两台** IR 相机 |
| 原理 | 图案被物体表面扭曲 → 从扭曲量反算形状 | 左右图像视差 → 三角测距 |
| 代表产品 | Kinect v1、Intel SR305、iPhone Face ID | D435i、D455 |
| 精度 | 近距离极高（亚毫米） | 中远距离好 |
| 室外 | 基本不能用（阳光淹没投影） | 可切换为被动立体 |

D435i 产品线里没有结构光相机。Intel 的结构光产品是 **SR300/SR305** 系列（已停产），那是给室内短距离用的（比如手势识别、人脸扫描）。

### 为什么 `/vision/aligned_depth/image_raw` 在 rviz 里几乎是黑的

深度话题是 `32FC1` 格式，每个像素存的不是颜色，而是**以米为单位的距离值**。比如 0.5 表示 50cm，3.0 表示 3m，10.0 表示 10m。

rviz 直接把 0~10 的浮点数映射到 0~255 灰度：
- 0.1m → 灰度值 ≈ 2.5 → **纯黑**
- 0.5m → 灰度值 ≈ 12 → **几乎是黑的**
- 3.0m → 灰度值 ≈ 76 → 深灰
- 10.0m → 灰度值 ≈ 255 → 白色

**所以室内近距离（0.5~3m）的深度图看起来就是一片漆黑**——这不是坏了，是数值太小。把相机对着窗外远处的东西，画面就会变亮。

> 想看彩色深度热力图，用 Intel 官方工具：
> ```bash
> realsense-viewer
> ```
> 里面对深度图做了伪彩色渲染（近=红、远=蓝），比 rviz 直观得多。

**启动命令：**

**① 默认 30FPS：**

```bash
roslaunch cuadc_vision camera_node.launch
```

**② USB 不稳定时降帧率：**

```bash
roslaunch cuadc_vision camera_node.launch fps:=15
```

---

### 在 rviz 中查看相机画面

rviz 可以同时查看**彩色画面**和**深度画面**，并排对比。

**第一步——启动 rviz（另开终端）：**

```bash
rviz
```

**第二步——添加彩色画面：**
1. 点击左下角 **Add** 按钮
2. 弹出对话框，选 **By topic** 标签页
3. 找到 `/vision/color/image_raw`，选中后点 **OK**
4. 画面区域会出现 D435i 彩色图像

**第三步——添加深度画面（并排对比）：**
1. 再次点击 **Add** → **By topic**
2. 找到 `/vision/aligned_depth/image_raw`，选中后点 **OK**
3. 现在 rviz 里有两个 Image 窗口，直接拖动窗口边缘即可并排显示

> **关于深度图显示：** 深度图是 `32FC1` 格式（每个像素存储的是米为单位的深度值）。rviz 默认以灰度显示——**越近越暗、越远越亮**。这不是画面坏了，是正常的深度数据可视效果。如果需要彩色深度渲染图（像 `realsense-viewer` 里那种红蓝热力图），可以用 rqt 的深度图可视化插件（见下方）。

**效果：** 左侧彩色画面看目标，右侧深度画面看距离——两个窗口并排，方便对比。

---

### 用 rqt 快速看

如果只是临时看一眼画面，不想配 rviz，用 rqt 更快：

```bash
rosrun rqt_image_view rqt_image_view /vision/color/image_raw
# 下拉框可切换到 /vision/aligned_depth/image_raw 看深度图
```

**发布话题：**

| 话题 | 类型 | 用途 |
|------|------|------|
| `/vision/color/image_raw` | sensor_msgs/Image | 彩色图（640×480 bgr8） |
| `/vision/aligned_depth/image_raw` | sensor_msgs/Image | 深度图（已对齐到彩色） |
| `/vision/color/camera_info` | sensor_msgs/CameraInfo | 相机内参矩阵 |

**验证是否正常：**

```bash
rostopic hz /vision/color/image_raw   # 应该有稳定 30Hz 输出
# 终端日志应打印：D435i started. 640x480 bgr8@30
# 如果打印 usb=2.1 则说明 USB 工作在 2.0 模式，需换 USB3 口或线
```

---

---

### 4. detector_node.py — YOLO 目标检测

**启动文件：** `detector_node.launch`

**功能：** 一键启动 D435i 相机 + YOLO 检测。支持**自动启动 roscore / MAVROS**（如未运行），读取飞控数据并在画面左下角面板显示。加载 YOLOv8 模型逐帧推理，发布检测结果和飞控数据。

**测试什么：** 验证模型能否正确识别圆筒、置信度是否够高、检测帧率是否满足实时要求。验证 MAVROS 连接和数据读取是否正常。

**启动后你可以：**
- 加 `show_window:=true` 时弹出 `YOLO Detection` 窗口，实时看到检测框、距离标注和**飞控状态面板**
- 用 `rostopic echo /vision/yolo/detection` 查看检测数据
- 用 `rostopic echo /vision/bucket/info` 查看目标数量和像素偏差
- 拿着圆筒在相机前移动，观察检测框是否跟随
- 直接 `python3 detector_node.py` 启动（不依赖 roslaunch），脚本会自动拉起 roscore 和 MAVROS

**自动启动（auto-start）机制：**

脚本启动时自动检测运行环境：
1. **检测 roscore**：通过 XML-RPC 探测 ROS master。如果未运行，自动 `roscore &` 并等待就绪（30s 超时）
2. **检测 MAVROS**：通过检查 `/mavros/state` 话题是否存在。如果未运行，自动 `roslaunch mavros apm.launch &` 并等待就绪（30s 超时）
3. 两项自动启动均可通过参数关闭：`auto_start_mavros:=false`

> 自动启动仅在地面调试时方便使用。比赛飞行时应使用 `run_main.launch` 统一管理所有节点的启动。

**启动命令：**

**① 首次测试——CPU 推理（无窗口、自动启动 MAVROS）：**

```bash
roslaunch cuadc_vision detector_node.launch
# 如果 roscore 未运行，脚本会自动启动它
# 如果 MAVROS 未运行，脚本会自动启动它（等待 30s 超时后跳过）
```

**② 地面调试——带画面窗口 + MAVROS 数据：**

```bash
roslaunch cuadc_vision detector_node.launch show_window:=true
# 窗口左下角显示飞控连接状态、电压、卫星数、RTK GPS 坐标
```

**③ 关闭自动启动 MAVROS（MAVROS 已在其他终端运行）：**

```bash
roslaunch cuadc_vision detector_node.launch auto_start_mavros:=false
```

**④ 直接 Python 启动（无 roslaunch，全自动）：**

```bash
python3 ~/catkin_ws/src/cuadc_src/scripts/detector_node.py
# 自动拉起 roscore → 自动拉起 MAVROS → 启动检测
# 这是最快的地面测试方式
```

**⑤ GPU 推理 + 提高阈值：**

```bash
roslaunch cuadc_vision detector_node.launch yolo_device:=cuda:0 yolo_conf_threshold:=0.7
```

**⑥ 自定义模型 + 指定飞控串口：**

```bash
roslaunch cuadc_vision detector_node.launch \
  yolo_model_path:=/home/lab/my_model.pt \
  mavros_fcu_url:=/dev/ttyUSB0:921600
```

**发布话题：**

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vision/yolo/detection` | YoloDetection | 当前帧最高置信度目标 |
| `/vision/yolo/detections` | YoloDetections | 当前帧全部通过过滤的目标 |
| `/vision/bucket/info` | BucketInfo | 目标数量 + 最佳目标的像素偏差 |
| `/vision/annotated_image` | sensor_msgs/Image | 标注画面（含检测框 + 飞控面板） |

#### `/vision/bucket/info` 消息详解 (BucketInfo)

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | Header | 时间戳和 frame_id |
| `count` | int32 | **当前帧识别到的目标物体数量**（通过类别过滤后的） |
| `delta_x` | float32 | 最佳目标中心与画面光心的**水平像素偏差** (px)，正值=目标在光心右侧 |
| `delta_y` | float32 | 最佳目标中心与画面光心的**垂直像素偏差** (px)，正值=目标在光心下方 |

> **count vs detected：** `count` 替代了 `YoloDetection.detected` 的"是/否"二元判断，直接给出检测到的目标数量：`count=0` 表示无目标，`count>0` 表示有目标且数量明确。
>
> **delta_x / delta_y 的用途：** 供 `auto_drop_node` 判断目标是否在画面中心附近（对准即投）。正值和负值的含义与相机光学坐标系的 X(右+)、Y(下+) 一致。

**验证是否正常：** 终端打印 `Detector started. model=...`，拿着圆筒在镜头前目标框跟随移动；如果加了 `show_window:=true`，窗口应正常弹出且画面流畅。飞控连接后终端打印 `FC data available: ...`，面板显示飞控数据。

**FPS 基准测试（NUC i7-1265U，CPU 推理，imgsz=640）：**

| 模型 | FPS | 备注 |
|------|-----|------|
| YOLOv8n | ~18.6 | 最快，精度略低 |
| YOLOv8s | ~8.6 | 精度更高但偏慢 |

> 比赛推荐用 YOLOv8n，18.6 FPS 满足实时性要求。如果后续换 NUC 带 GPU（如 NUC 14 Pro+ 配 Arc GPU），可切 `yolo_device:=cuda:0` 获得更高帧率。

**测试检测脚本（同步开启画面窗口）：**

```bash
roslaunch cuadc_vision detector_node.launch show_window:=true
```

**预测框旁边显示的文字说明：**

预测框上方会显示一行格式如下的文字：
```
cylinder 0.85 x→0.12 y↓0.30 z=2.30 d=2.31m
```

| 字段 | 含义 | 坐标系 |
|------|------|--------|
| `cylinder` | YOLO 模型输出的类别名 | — |
| `0.85` | 置信度 | — |
| `x→` | 目标相对相机光心水平偏移（正值=右方，X+） | 相机光学系 |
| `y↓` | 目标相对相机光心垂直偏移（正值=下方，Y+） | 相机光学系 |
| `z` | 沿光轴方向的深度距离（相机正前方，Z+） | 相机光学系 |
| `d` | 相机到目标的直线距离 `sqrt(x²+y²+z²)` | — |

> **z 和 d 的区别：** 目标在画面正中心时 `z = d`。目标偏离中心时 `d > z`，因为斜着量过去比直着量的深度更长。简单说：`z` 是踩在脚下的深度，`d` 是斜线拉过去的那根线长度。
>
> 画面正中心有一个标准相机光学坐标系标记：红色 x 轴 → 右 (X+)，绿色 y 轴 ↓ 下 (Y+)。

### 标准相机光学坐标系

> 📖 **完整的 5 坐标系定义和变换链路见上方「坐标系统详解」章节。** 下面是与 detector_node 直接相关的相机光学系摘要。

本节点输出和显示的坐标遵循**标准针孔相机模型 / OpenCV 光学坐标系**：

```
         Z+ (光轴，从镜头向外)
        ↗
       /
      相 机 —————→ X+ (右)
       |
       ↓
      Y+ (下)
```

**坐标系定义：**
- **Z+**：沿光轴从镜头指向被拍摄物体（相机"前方"）
- **X+**：指向相机右侧（画面像素 u 增大方向）
- **Y+**：指向相机下方（画面像素 v 增大方向）
- 这是右手定则：X × Y = Z（右 × 下 = 前）

**反投影公式（标准针孔模型，无任何符号翻转）：**
```
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
Z = depth
```

其中 `(u, v)` 是像素坐标，`(cx, cy)` 是光心像素，`fx, fy` 是焦距。

**画面位置与坐标正负的对应：**

| 目标在画面中的位置 | camera_x (X) | camera_y (Y) | 原因 |
|-------------------|-------------|-------------|------|
| 右边 | **正 (+)** | — | u > cx → (u - cx) > 0 |
| 左边 | **负 (-)** | — | u < cx → (u - cx) < 0 |
| 下方 | — | **正 (+)** | v > cy → (v - cy) > 0 |
| 上方 | — | **负 (-)** | v < cy → (v - cy) < 0 |
| 正中心 | **0** | **0** | u = cx, v = cy |

> **画面上的坐标轴箭头（红 x→ 右、绿 y↓ 下）与数值正负完全一致**：目标在箭头指的方向上，数值就是正的；在反方向上，数值就是负的。

### 相机→机体坐标变换

相机安装在机体下方，镜头朝下，相机外壳的物理"上方"朝向机体前方。相机光学坐标系到机体坐标系的映射：

```
机体前方 (X_body) = -相机Y  (因为相机 -Y 方向 = 相机物理上方 → 机体前方)
机体右方 (Y_body) =  相机X  (相机右侧 = 机体右侧)
机体下方 (Z_body) =  相机Z  (相机光轴 = 指向地面)
```

这个旋转变换由 TF 树（`d435i_color_optical_frame` → `base_link`）和 `geopose_node` 负责。`detector_node` 只输出**纯净的标准相机光学坐标**，不做任何自定义符号翻转。

### 画面左下角固定面板（飞控状态）

窗口左下角有一个固定面板，显示检测摘要 + 飞控状态：

**飞控未连接时：**
```
detected: 3  FPS 18.6
FC: disconnected
model: best.pt  device: cpu
```

**飞控已连接时（有 RTK GPS）：**
```
detected: 3  FPS 18.6
FC: connected  GUIDED  ARM
Bat: 16.8V  Sat: 20  RTK Fixed
GPS: 22.1234567  114.1234567  15.23m
model: best.pt  device: cpu
```

**面板各行含义：**

| 行 | 字段 | 含义 | 数据来源 |
|----|------|------|---------|
| 1 | `detected: N` | 当前帧目标数量（0=无目标） | YOLO 推理结果 |
| 1 | `FPS X.X` | 推理帧率 | 本地计时 |
| 2 | `FC: connected` | 飞控连接状态 | `/mavros/state.connected` |
| 2 | `MODE` | 当前飞行模式（GUIDED/AUTO/LOITER/...） | `/mavros/state.mode` |
| 2 | `ARM/DISARM` | 解锁状态 | `/mavros/state.armed` |
| 3 | `Bat: X.XV` | 电池电压 | `/mavros/battery.voltage` |
| 3 | `Sat: N` | 可见卫星数量 | `/mavros/gpsstatus/gps1/raw.satellites_visible` |
| 3 | `RTK Fixed` | GPS 定位类型 | `/mavros/gpsstatus/gps1/raw.fix_type` |
| 4 | `GPS: lat lon alt` | 厘米级大地坐标（WGS84） | `/mavros/global_position/global` |

> **面板数据容错：** 如果 MAVROS 未运行或 `mavros_msgs` 未安装，飞控数据行自动隐藏（只显示 `FC: disconnected`），不影响检测功能。
>
> **GPS 坐标精度：** RTK Fixed 模式下，面板显示的经纬度精度可达 7 位小数（厘米级）。`altitude` 为 WGS84 椭球高度 (m)。

### 画面中下区域任务叠加文字 (AIMING!! / DROP!!!)

画面中心偏下位置会根据任务状态叠加闪烁文字：

| 状态 | 显示 | 颜色 | 效果 | 触发条件 |
|------|------|------|------|---------|
| 瞄准中 | **AIMING!!** | 黄色 | 0.5Hz 闪烁 | 飞控进入 GUIDED 模式 + MISSION 状态 |
| 抛投中 | **A DROP!!!** | 红色 | 常亮 3 秒 | main.py 发送 MAVLink 舵机打开指令 |

> **通信原理：** main.py 通过 `/vision/mission_status` 话题 (MissionStatus 消息) 向 detector_node 发送 `aiming` 和 `last_drop` 字段。detector_node 的 `_mission_cb` 回调接收后更新本地状态，`_draw_mission_overlay()` 在每帧画面中下区域叠加对应文字。
>
> **为什么选择自定义消息而不是直接监听 MAVLink：** 自定义 ROS 消息 (`MissionStatus`) 更简单可靠——main.py 是任务状态的唯一权威来源，detector_node 只需被动接收显示，不需要解析 MAVLink 协议。这避免了在 detector_node 中引入 pymavlink 依赖和复杂的 MAVLink 消息解析逻辑。

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `yolo_device` | cpu | cpu / cuda:0 |
| `yolo_conf_threshold` | 0.5 | 低于此值的目标不发布 |
| `yolo_imgsz` | 640 | 推理分辨率，越小越快 |
| `target_classes` | `cylinder,tong,barrel,...` | 只显示类别名含这些关键词的目标 |
| `auto_start_mavros` | true | 自动检测并启动 MAVROS |
| `mavros_fcu_url` | `/dev/ttyACM0:921600` | 飞控串口地址 |
| `show_window` | false | 弹出 OpenCV 窗口 |
| `fc_state_topic` | `/mavros/state` | 飞控状态话题 |
| `fc_battery_topic` | `/mavros/battery` | 电池状态话题 |
| `fc_gps_topic` | `/mavros/global_position/global` | GPS 大地坐标话题 |
| `fc_gpsraw_topic` | `/mavros/gpsstatus/gps1/raw` | GPS 原始数据话题（卫星数+RTK状态） |

---

### 5. geopose_node.py — 大地坐标变换

**启动文件：** `run_main.launch`（加 `enable_geopose:=true`）

**功能：** 实现完整的坐标变换链：相机光学系 → TF → 机体 FRD → 四元数旋转 → ENU (东北天) → geographiclib → WGS84 (经纬高)。

> 📖 **坐标系定义和变换公式详见上方「坐标系统详解」章节**，涵盖 5 个坐标系的定义、每步变换的公式、物理安装关系和数据流。本节聚焦于节点的使用方式。

**变换流程（代码实现）：**
```
1. tf_buffer.transform()       camera_optical_frame → base_link (机体 FRD)
2. quaternion_rotate_vector()  机体 FRD → ENU (用 MAVROS 姿态四元数旋转)
3. enu_offset_to_geodetic()    ENU 偏移 → WGS84 经纬高 (Vincenty 正算)
```

**测试什么：** 验证坐标变换链路是否完整——把目标放到已知 GPS 位置，看输出的经纬度是否接近。

**启动后你可以：**
- 用 `rostopic echo /vision/target_global` 查看大地坐标
- 结合 QGC 地图验证坐标点是否落在正确位置
- 飞控收到坐标后可以直接导航到目标上空
- 查看 GeoTarget 消息中的 `body_x/y/z_m` 和 `enu_east/north/up_m` 中间结果来调试变换链

**启动命令：**

```bash
# 一条命令启动 main + camera + YOLO + geopose
roslaunch cuadc_vision run_main.launch enable_geopose:=true
```

```bash
# 只测变换链路（需要先启动 MAVROS 和 detector_node）
roslaunch cuadc_vision run_main.launch enable_geopose:=true auto_arm:=false auto_takeoff:=false
```

**依赖：** MAVROS 运行中，提供：
- `/mavros/global_position/global` — 无人机 RTK GPS 坐标
- `/mavros/local_position/pose` — 无人机 ENU 位姿 + 姿态四元数
- TF 树中 `d435i_color_optical_frame → base_link` 的静态变换

**输出话题：** `/vision/target_global`（类型 GeoTarget）

**GeoTarget 消息字段：**

| 字段 | 坐标系 | 含义 |
|------|--------|------|
| `camera_x/y/z_m` | 相机光学系 | 目标在相机系下的 3D 坐标（Step 1 输出） |
| `body_x/y/z_m` | 机体 FRD | 目标在机体坐标系下的坐标（Step 2 输出） |
| `enu_east/north/up_m` | ENU | 目标相对无人机的 ENU 偏移（Step 3 输出） |
| `latitude, longitude, altitude` | WGS84 | 目标大地坐标（Step 4 输出，最终结果） |
| `valid` / `status` | — | `"ok"` 表示变换成功；其他值见下方状态码 |

**status 状态码：**

| status | 含义 | 处理建议 |
|--------|------|---------|
| `ok` | 变换成功，坐标有效 | — |
| `no_detection` | 当前帧未检测到目标 | 正常，等待检测 |
| `low_confidence` | 检测置信度低于 `min_confidence` | 调低阈值或等待更好的帧 |
| `invalid_camera_position` | 深度图无效（depth=0 或 NaN） | 检查深度相机、目标是否在有效距离内 |
| `no_valid_global_position` | MAVROS 未提供有效 GPS 数据 | 等待 RTK GPS fix |
| `tf_camera_to_body_failed` | TF 查询相机→机体变换失败 | 检查 TF 树配置 |

**验证是否正常：**

```bash
rostopic echo /vision/target_global
# status="ok" → 变换链路正常
# latitude/longitude/altitude 有具体数值
# 可与 QGC 地图对照验证坐标是否落在正确位置

# 查看中间变换结果（调试用）
rostopic echo /vision/target_global | grep -E "status|body_|enu_|lat|lon|alt"
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `detection_topic` | `/vision/yolo/detection` | 订阅的检测结果话题 |
| `body_frame` | `base_link` | 机体坐标系 TF frame ID |
| `camera_frame` | `d435i_color_optical_frame` | 相机光学系 TF frame ID |
| `min_confidence` | 0.30 | 最低检测置信度，低于此值不变换 |
| `transform_timeout_sec` | 0.10 | TF 查询超时 (s) |
| `publish_invalid` | true | 是否发布无效/失败的变换结果（调试用） |

<details>
<summary><b>📐 gptimage 绘图提示 6：geopose_node 变换链代码流程图</b></summary>

```
A flowchart-style diagram showing the geopose_node internal processing pipeline:

Top: Input boxes
- "YOLO Detection (camera_x/y/z_m)" from detector_node
- "MAVROS /local_position/pose (orientation quaternion)" from flight controller
- "MAVROS /global_position/global (RTK GPS)" from flight controller

Middle: Processing steps in vertical sequence
Step 1: "transform_camera_to_body()" 
  - Sub-step: "tf_buffer.transform(camera_frame → body_frame)"
  - Output: (body_x, body_y, body_z)
  
Step 2: "quaternion_rotate_vector()"
  - Sub-step: "q ⊗ (body_x,body_y,body_z) ⊗ q⁻¹"
  - Output: (east_m, north_m, up_m)
  
Step 3: "enu_offset_to_geodetic()"
  - Sub-step 1: "atan2(east, north) → azimuth"
  - Sub-step 2: "√(east²+north²) → horizontal distance"
  - Sub-step 3: "Geodesic.WGS84.Direct(lat, lon, azimuth, distance)"
  - Output: (lat, lon, alt + up)

Bottom: Output box
- "GeoTarget: camera_xyz + body_xyz + enu_enu + lat/lon/alt + status"

Side annotations:
- Error branches for each step (e.g., TF timeout → "tf_camera_to_body_failed")
- Key dependencies: TF tree, MAVROS, geographiclib

Label at top: "geopose_node.py 变换流程"
Style: clean vertical flowchart, code-block style processing steps, Chinese + English labels
```
</details>

---

### 6. flight_data_video_recorder_node.py — 飞行数据录像

**启动文件：** `run_flight_recorder.launch`

**功能：** 飞控解锁后自动开始录像，画面左下角叠加实时飞行数据（相对高度、GPS 坐标、电压、电流、飞行模式）。上锁后自动停止。同时输出 .avi 视频和 .csv 数据表。

**测试什么：** 不用起飞也能测——加 `record_immediately:=true` 立即录像，验证画面叠加文字是否清晰、CSV 数据是否完整。**真正的用途是飞行后回放视频，分析不同高度下相机能拍到多大范围、什么高度识别圆筒效果最好。**

**启动后你可以：**
- 地面测试：`record_immediately:=true show_window:=true` 立即看到叠加画面
- 实际飞行：正常启动，解锁自动录，降落自动停
- 拿 CSV 数据在 Excel 里画高度-时间曲线

**启动命令：**

**① 实际飞行用**——解锁自动开始，上锁自动停止：

```bash
roslaunch cuadc_vision run_flight_recorder.launch
```

**② 地面调试**——开窗口看叠加效果：

```bash
roslaunch cuadc_vision run_flight_recorder.launch show_window:=true
```

**③ 不等解锁，立即开始录**（纯地面测试）：

```bash
roslaunch cuadc_vision run_flight_recorder.launch record_immediately:=true show_window:=true
```

**输出：** `~/cuadc_flight_videos/YYYYMMDD_HHMMSS/cuadc_flight_YYYYMMDD_HHMMSS.avi` + `.csv`

**CSV 列：** frame, ros_time, armed, mode, latitude, longitude, global_alt_m, relative_alt_m, local_x/y/z_m, voltage_v, current_a, battery_percent

**验证是否正常：** 终端打印 `Flight video recording started` 且输出目录出现 .avi 文件。打开视频应看到相机画面 + 左上角叠加的飞行数据文字。

---

### 7. auto_drop_node.py — 自动抛投

**启动文件：** `auto_drop.launch`

**功能：** 监听 YOLO 检测结果，当检测到的目标中心与相机光心的像素偏差小于阈值时，自动通过 MAVROS 发送舵机指令触发抛投。释放后保持一段时间再复位，并设冷却时间防止重复触发。

**测试什么：** 验证"对准即投"逻辑——把圆筒放在相机视野中央，观察是否自动触发舵机。**地面就能测，不需要起飞。**

**启动后你可以：**
- 启动节点 + YOLO 检测
- 把圆筒移到画面中央
- 观察终端是否打印 `triggering drop`
- 听舵机动作声音确认触发

**启动命令：**

**① 默认参数**：

```bash
roslaunch cuadc_vision auto_drop.launch
```

**② 放宽对准要求 + 提高置信度**：

```bash
roslaunch cuadc_vision auto_drop.launch pixel_threshold:=30.0 min_conf:=0.7
```

**工作流程：** 检测到目标 → 像素偏差 ≤ pixel_threshold → 置信度 ≥ min_conf → 距上次抛投 ≥ cooldown → 舵机转到 release_pwm → 保持 hold_seconds → 舵机回到 reset_pwm

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `pixel_threshold` | 20.0 | 对准容差 (px)，越小对准越严格 |
| `min_conf` | 0.5 | 最低检测置信度 |
| `channel` | 9 | 舵机通道号 |
| `release_pwm` | 1900 | 释放时的 PWM 值 |
| `reset_pwm` | 1100 | 复位时的 PWM 值 |
| `hold_seconds` | 0.8 | 释放保持时间 (s) |
| `cooldown` | 2.0 | 两次抛投最小间隔 (s) |

**验证是否正常：** 确保已启动 YOLO 检测（`cuadc_run.launch enable_yolo:=true`），圆筒移到画面中央时终端应打印 `triggering drop`，舵机应动作。

---

## 启动文件总览

| 文件 | 包含节点 | 用途 |
|------|---------|------|
| `camera_node.launch` | camera | 相机驱动（配合 rviz） |
| `detector_node.launch` | camera + detector | 相机 + YOLO + OpenCV 窗口 |
| `run_main.launch` | main + camera + detector + geopose | 总启动 |
| `run_servo_test.launch` | servo_test | 舵机测试 |
| `run_flight_recorder.launch` | camera + flight_data_video_recorder | 飞行录像 |
| `auto_drop.launch` | detector + auto_drop | 自动抛投 |

---

## 比赛完整启动

**1. 启动 MAVROS（终端 1）：**

```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
```

**2. 启动总控（终端 2）：**

```bash
roslaunch cuadc_vision run_main.launch auto_arm:=false auto_takeoff:=false
```

**3（可选）. 自动抛投（终端 3）：**

```bash
roslaunch cuadc_vision auto_drop.launch
```

**4（可选）. 飞行录像（终端 4）：**

```bash
roslaunch cuadc_vision run_flight_recorder.launch
```

---

## ROS 话题速查

| 节点 | 话题 | 用途 |
|------|------|------|
| camera_node | `/d435i/color/image_raw` | 彩色图 |
| camera_node | `/d435i/aligned_depth/image_raw` | 深度图 |
| camera_node | `/d435i/color/camera_info` | 相机内参 |
| detector_node | `/vision/yolo/detection` | 最佳目标 |
| detector_node | `/vision/yolo/detections` | 全部目标 |
| detector_node | `/vision/bucket/info` | 目标数量 + 像素偏差 |
| main.py | `/vision/mission_status` | 弹药/瞄准/抛投事件 |
| geopose_node | `/vision/target_global` | 大地坐标 |
| servo_test | `/servo/cmd` | 舵机指令 |

```bash
# 验证话题
rostopic list | grep -E "d435i|yolo|competition|servo"
rostopic echo /competition/target_global
```

---

## 常见问题

**catkin_make 报 ddynamic_reconfigure 错误：**
```
CMake Error: Project 'ddynamic_reconfigure' specifies ...
'.../ddynamic_reconfigure-kinetic-devel/include' as an include dir, which is not found.
```
原因：工作空间 `~/catkin_ws/src/` 里混入了 `realsense-ros` 和 `ddynamic_reconfigure-kinetic-devel` 这两个无关包。`cuadc_vision` 用的是 Python 库 `pyrealsense2`，不需要 ROS 的 `realsense2_camera` 包。

```bash
# 删掉无关包
rm -rf ~/catkin_ws/src/realsense-ros
rm -rf ~/catkin_ws/src/ddynamic_reconfigure-kinetic-devel
cd ~/catkin_ws && catkin_make
```

**D435i 不识别：** 插 USB 3.0 蓝色口，`realsense-viewer` 确认画面。

**YOLO 报错：** `pip3 install ultralytics torch`

**detector_node 报 `KeyError: 16` / `bad callback`：**
典型报错：
```text
[ERROR] ... bad callback: <bound method DetectorNode.image_callback ...>
File ".../detector_node.py", line ..., in image_callback
  annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
KeyError: 16
```
原因：ROS Noetic 自带的 `cv_bridge` 在部分 OpenCV 4 环境下，把 `bgr8` 图像转成 `sensor_msgs/Image` 时会触发内部映射错误。

当前项目已在 `detector_node.py` 中绕开这个问题，改为手工构造 `sensor_msgs/Image`。如果你仍然看到这个报错，通常说明当前终端运行的还是旧代码或旧工作空间环境。

```bash
# 重新编译并重新 source
cd ~/catkin_ws
catkin_make
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 带画面调试
roslaunch cuadc_vision detector_node.launch show_window:=true
```

如果是在没有桌面环境的终端（例如 SSH）里调试，不要开窗口：
```bash
roslaunch cuadc_vision detector_node.launch
```

**TF 报错：** MAVROS 是否在运行？`rostopic list | grep mavros`

**坐标与画面不一致：** 确认你理解标准相机光学坐标系的定义：X+ = 右，Y+ = 下，Z+ = 光轴前方。窗口画面中心的坐标轴（红 x→ 右、绿 y↓ 下）标注了正方向。相机→机体的旋转由 TF 树和 geopose_node 负责，detector_node 不做额外翻转。

**detector_node 面板不显示飞控数据：** 检查：
1. MAVROS 是否运行：`rostopic list | grep mavros`
2. `mavros_msgs` 是否安装：`dpkg -l | grep ros-noetic-mavros-msgs`
3. 飞控是否上电连接，GPS 是否有 fix
4. 终端是否打印 `FC data NOT available`（说明 MAVROS 未运行且 `auto_start_mavros=false`）
5. 如果 MAVROS 未运行，加 `auto_start_mavros:=true` 让脚本自动启动

**detector_node 面板不显示卫星数：** MAVROS 的 `gps_status` 插件默认可能未启用。检查 `/mavros/gpsstatus/gps1/raw` 话题是否存在：
```bash
rostopic list | grep gpsstatus
# 如果不存在，说明 MAVROS 未加载 gps_status 插件
# 需要修改 MAVROS 配置或 launch 文件启用它
```

**舵机不动：** 检查飞控通道映射，确认 CH5/CH6 配置为 servo。

**录像卡顿：** 降帧率 `video_fps:=15.0`，或换 codec `codec:=XVID`。

**main.py 反复报 `set_mode 服务调用失败` / `connection abort`：**
这是因为 MAVROS 还没启动，main.py 连不上飞控。先开一个终端启动 MAVROS：
```bash
roslaunch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
```
等连上飞控后再启动 `run_main.launch`。或者先把 main.py 的终端 `Ctrl+C` 关掉。

**roslaunch 报 `is neither a launch file in package`：**
说明 ROS 找不到 launch 文件。常见原因：
1. 新开终端没 source 环境变量
2. 添加新文件后没重新 `catkin_make`

```bash
# 检查 launch 文件是否存在
ls ~/catkin_ws/src/cuadc_src/launch/

# 重新编译 + source（每次新终端都要 source）
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 验证包是否被 ROS 识别
rospack find cuadc_vision
# 应输出：/home/lab/catkin_ws/src/cuadc_src
```
