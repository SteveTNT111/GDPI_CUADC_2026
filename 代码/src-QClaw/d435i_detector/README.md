# D435i 黄色圆检测 —— Ubuntu 20.04 + ROS Noetic 部署指南

## 1. 环境要求

| 项目 | 版本 |
|------|------|
| OS | Ubuntu 20.04 LTS |
| ROS | Noetic Ninjemys |
| Python | 3.8（系统自带） |
| RealSense SDK | librealsense2 ≥ 2.50 |
| pyrealsense2 | ≥ 2.50 |
| OpenCV | ≥ 4.2 |

---

## 2. 安装 ROS Noetic（如尚未安装）

```bash
# 设置 sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 设置密钥
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 安装
sudo apt update
sudo apt install ros-noetic-desktop-full

# 环境变量（写入 ~/.bashrc）
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 初始化 rosdep
sudo rosdep init
rosdep update
```

---

## 3. 安装 RealSense SDK & pyrealsense2

```bash
# 添加 Intel RealSense apt 源
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
  sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# 安装 librealsense2
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# 安装 pyrealsense2
pip3 install pyrealsense2

# USB 权限（将当前用户加入 video 组）
sudo usermod -aG video $USER
# 重新登录后生效
```

> 验证相机连接：
> ```bash
> rs-enumerate-devices   # 应列出 D435i 设备信息
> realsense-viewer       # GUI 预览彩色+深度画面
> ```

---

## 4. 创建 catkin 工作空间

```bash
# 创建工作空间
mkdir -p ~/d435i_ws/src

# 将本项目的 d435i_detector 拷贝到 src 下
# 假设你把项目拷贝到了 Ubuntu 上，路径为 ~/d435i_project/
cp -r ~/d435i_project/d435i_detector ~/d435i_ws/src/

# ⚠️ 删除 ROS2 残留文件（如果从 Windows 拷贝过来的）
cd ~/d435i_ws/src/d435i_detector
rm -f setup.py setup.cfg
rm -rf d435i_detector/      # ROS2 的 Python 包目录，ROS1 不需要
rm -rf resource/             # ROS2 的包标记目录
rm -f launch/d435i_detector.launch.py  # ROS2 的 Python launch 文件

# 确认目录结构
tree ~/d435i_ws/
# 应该看到：
# ~/d435i_ws/
# └── src/
#     └── d435i_detector/
#         ├── CMakeLists.txt
#         ├── package.xml
#         ├── msg/
#         │   └── YellowCircle.msg
#         ├── scripts/
#         │   ├── d435i_publisher.py
#         │   └── yellow_detector.py
#         └── launch/
#             └── d435i_detector.launch
```

---

## 5. 编译

```bash
cd ~/d435i_ws

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# catkin 编译
source /opt/ros/noetic/setup.bash
catkin_make

# source 工作空间
source devel/setup.bash

# 建议写入 ~/.bashrc
echo "source ~/d435i_ws/devel/setup.bash" >> ~/.bashrc

# 给脚本添加可执行权限
chmod +x ~/d435i_ws/src/d435i_detector/scripts/*.py
```

> **⚠️ 首次编译后自定义消息才能被 Python 节点 import。如果 `from d435i_detector.msg import YellowCircle` 报错，请确认已 `source devel/setup.bash` 并重新开终端。**

---

## 6. 运行

### 6.1 一键启动（launch 文件）

```bash
source ~/d435i_ws/devel/setup.bash
roslaunch d435i_detector d435i_detector.launch
```

### 6.2 分别启动（调试用）

```bash
# 终端1：启动 roscore
roscore

# 终端2：启动相机驱动
rosrun d435i_detector d435i_publisher.py

# 终端3：启动黄色检测
rosrun d435i_detector yellow_detector.py
```

### 6.3 带参数启动（动态调参）

```bash
# 方式1：通过 launch 文件传参
roslaunch d435i_detector d435i_detector.launch h_min:=15 h_max:=50 min_area:=300

# 方式2：运行时通过 rosparam 修改
rosparam set /yellow_detector/h_min 15
rosparam set /yellow_detector/h_max 50

# 方式3：rqt_reconfigure 图形化调参
rosrun rqt_reconfigure rqt_reconfigure
```

---

## 7. 话题列表

| 话题 | 消息类型 | 发布节点 | 说明 |
|------|----------|----------|------|
| `/d435i/color_image` | `sensor_msgs/Image` | d435i_publisher | 彩色图像 (BGR8) |
| `/d435i/depth_image` | `sensor_msgs/Image` | d435i_publisher | 深度图像 (16UC1) |
| `/d435i/camera_info` | `sensor_msgs/CameraInfo` | d435i_publisher | 彩色相机内参 |
| `/d435i/yellow_circle` | `d435i_detector/YellowCircle` | yellow_detector | 黄色圆位置+深度 |
| `/d435i/detected_image` | `sensor_msgs/Image` | yellow_detector | 标注后的检测效果图 |

### 常用调试命令

```bash
# 查看所有话题
rostopic list

# 查看检测结果
rostopic echo /d435i/yellow_circle

# 查看话题频率
rostopic hz /d435i/color_image

# 查看话题信息
rostopic info /d435i/yellow_circle

# 查看自定义消息定义
rosmsg show d435i_detector/YellowCircle

# 用 rqt_image_view 可视化图像
rosrun rqt_image_view rqt_image_view

# 用 rviz 可视化
rviz
```

---

## 8. 自定义消息 YellowCircle

```
# msg/YellowCircle.msg
Header header     # ROS 标准时间戳 + frame_id
int32 x           # 目标中心 X 像素坐标
int32 y           # 目标中心 Y 像素坐标
float32 depth     # 目标中心深度值（米）
```

查看：`rosmsg show d435i_detector/YellowCircle`

---

## 9. 检测算法说明

```
彩色帧 → BGR2HSV → 黄色 HSV 掩码 → 形态学开闭去噪
       → findContours → 面积过滤 → 圆度筛选(>0.6)
       → minEnclosingCircle 得中心+半径
       → 深度图取中心点深度 → 发布 YellowCircle
```

**关键调参项：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `h_min / h_max` | 20 / 40 | HSV 色相范围，黄色约 20~40 |
| `s_min / s_max` | 80 / 255 | 饱和度范围 |
| `v_min / v_max` | 80 / 255 | 明度范围 |
| `min_area` | 200 | 最小轮廓面积（像素²），过滤小噪点 |
| `circularity_thresh` | 0.6 | 圆度阈值 0~1，1 为完美圆 |
| `depth_scale` | 0.001 | 深度缩放系数（米/z16单位） |
| `show_window` | true | 是否弹出 OpenCV 显示窗口 |

**调参建议：**
- 室内偏暗：降低 `v_min` 到 50
- 黄色偏橙：提高 `h_max` 到 50
- 黄色偏绿：降低 `h_min` 到 15
- 噪点多：提高 `min_area` 到 500
- 圆不规整：降低 `circularity_thresh` 到 0.4

---

## 10. 常见问题

### Q: `ImportError: No module named pyrealsense2`
```bash
pip3 install pyrealsense2
# 如果 pip3 不存在
sudo apt install python3-pip
```

### Q: 相机打不开 / `No device detected`
```bash
# 检查 USB 连接
lsusb | grep Intel
# 应看到 Intel RealSense D435i

# 权限问题
sudo usermod -aG video $USER
# 重新登录

# 重置 USB
sudo apt install usbutils
usbreset "Intel RealSense D435i"
```

### Q: `ImportError: No module named d435i_detector.msg`
```bash
# 确保已编译并 source
cd ~/d435i_ws
catkin_make
source devel/setup.bash
# 重新开终端
```

### Q: 检测不到黄色圆
1. 用 `rqt_image_view` 查看 `/d435i/color_image` 确认画面正常
2. 调低 HSV 范围：`rosparam set /yellow_detector/h_min 15`
3. 减小面积阈值：`rosparam set /yellow_detector/min_area 100`
4. 降低圆度：`rosparam set /yellow_detector/circularity_thresh 0.4`
5. 在代码中临时添加 `cv2.imshow('mask', mask)` 查看 HSV 掩码效果

### Q: 深度值为 0
- D435i 深度有效范围约 0.1m ~ 10m
- 反光/透明表面深度可能为 0
- 可在检测代码中取周围 5×5 区域中值代替单点深度

### Q: `catkin_make` 找不到 cv_bridge
```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
```

---

## 11. 项目文件结构

```
d435i_ws/
└── src/
    └── d435i_detector/
        ├── CMakeLists.txt               # catkin 构建配置（含自定义消息生成）
        ├── package.xml                  # 包依赖声明
        ├── msg/
        │   └── YellowCircle.msg         # 自定义消息 (Header, x, y, depth)
        ├── scripts/
        │   ├── d435i_publisher.py        # D435i 驱动：彩色+深度+内参发布
        │   └── yellow_detector.py        # 黄色圆检测：HSV+轮廓+圆度+深度
        └── launch/
            └── d435i_detector.launch     # XML launch 一键启动
```

---

## 12. 从 ROS2 版本迁移需删除的残留文件

如果你是从之前的 ROS2 版本拷贝过来的，以下文件是 ROS2 专属，需要删除：

```bash
cd ~/d435i_ws/src/d435i_detector
rm -f setup.py setup.cfg
rm -rf d435i_detector/          # ROS2 Python 包目录
rm -rf resource/                # ROS2 包标记
rm -f launch/d435i_detector.launch.py   # ROS2 Python launch
```
