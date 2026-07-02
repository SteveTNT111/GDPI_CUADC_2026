# Ubuntu 20.04 + ROS Noetic D435i 视觉检测部署说明

本文档从安装 Ubuntu 20.04 和 ROS Noetic 开始，说明如何在机载电脑 Intel NUC 上部署当前 ROS1 工程。

工程目标：

- 使用 Intel RealSense D435/D435i 读取彩色图和对齐后的深度图。
- 使用 OpenCV 对黄色圆进行 HSV 过滤、二值化和圆形检测。
- 支持显示彩色检测窗口和黄色二值化窗口。
- 支持采集原始彩色图片数据集。
- 支持可选 YOLO 检测节点。
- 支持比赛任务辅助节点。

当前工程是 **ROS1 / ROS Noetic**，不是 ROS2。Ubuntu 工作空间固定使用：

```text
~/uav_ws
```

源码包路径固定使用：

```text
~/uav_ws/src/d435i_yellow_circle_detector
```

注意：Ubuntu 20.04 和 ROS Noetic 都已经不是最新维护版本。如果比赛或设备环境指定 Ubuntu 20.04 + ROS1，可以继续按本文部署；如果是全新长期维护项目，建议另行评估更新系统。

## 1. 安装 Ubuntu 20.04

### 1.1 准备安装盘

在另一台电脑上准备：

- Ubuntu 20.04.6 LTS Desktop 镜像。
- 8GB 或更大的 U 盘。
- Rufus、balenaEtcher 或 Ubuntu 自带启动盘工具。

写入镜像时建议选择：

```text
分区类型: GPT
目标系统: UEFI
文件系统: FAT32
```

### 1.2 NUC BIOS 建议

进入 NUC BIOS 后建议检查：

- 启动模式使用 UEFI。
- USB 启动已开启。
- Secure Boot 如遇到驱动或第三方内核模块问题，可以先关闭。
- D435i 插在标注 10Gbps 或 USB 3.x 的接口。

### 1.3 安装 Ubuntu

安装时建议：

- 语言可选中文或英文。
- 安装类型选择 Normal installation。
- 勾选安装第三方驱动和媒体解码器。
- 如果 NUC 只用于机载视觉，可以选择 Erase disk and install Ubuntu。

用户建议保持为：

```text
用户名: lab
主目录: /home/lab
```

因为当前脚本和模型路径默认使用 `/home/lab`。

### 1.4 安装后基础设置

打开终端，先更新系统：

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl wget git vim net-tools usbutils build-essential cmake python3-pip
```

检查系统版本：

```bash
lsb_release -a
```

应看到：

```text
Ubuntu 20.04.x LTS
Codename: focal
```

## 2. 安装 ROS Noetic

### 2.1 添加 ROS 软件源

```bash
sudo apt update
sudo apt install -y curl gnupg2 lsb-release
sudo mkdir -p /usr/share/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros1.list
sudo apt update
```

如果 `gpg --dearmor` 提示文件已存在，可以执行：

```bash
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
```

然后重新执行上一段添加 key 的命令。

### 2.2 安装 ROS Noetic Desktop Full

```bash
sudo apt install -y ros-noetic-desktop-full
```

### 2.3 初始化 rosdep

```bash
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
sudo rosdep init
rosdep update
```

如果 `sudo rosdep init` 提示已经存在，可以忽略。

### 2.4 配置 ROS 环境变量

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

检查 ROS：

```bash
rosversion -d
```

应输出：

```text
noetic
```

## 3. 安装本工程依赖

### 3.1 安装 ROS 依赖包

```bash
sudo apt install -y \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-sensor-msgs \
  ros-noetic-std-msgs \
  ros-noetic-message-generation \
  ros-noetic-message-runtime \
  python3-opencv \
  python3-numpy
```

### 3.2 安装 RealSense Python SDK

当前代码使用 Python SDK：

```bash
pip3 install pyrealsense2
```

检查是否安装成功：

```bash
python3 -c "import pyrealsense2 as rs; print(rs.__version__)"
```

### 3.3 安装 RealSense udev 规则

如果普通用户无法访问 D435i，需要安装 udev 规则。推荐先用系统包：

```bash
sudo apt install -y librealsense2-udev-rules librealsense2-utils
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后重新插拔相机。

检查相机：

```bash
lsusb | grep -i real
realsense-viewer
```

`realsense-viewer` 能看到彩色和深度画面，说明相机硬件和 USB 通道基本正常。

### 3.4 可选：安装 YOLO 依赖

只有需要启动 YOLO 节点时才安装：

```bash
pip3 install ultralytics
```

默认模型路径：

```text
/home/lab/model/best.pt
```

检查模型是否存在：

```bash
ls /home/lab/model/best.pt
```

## 4. 创建 ROS 工作空间

### 4.1 创建 `uav_ws`

```bash
mkdir -p ~/uav_ws/src
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
catkin_make
```

添加工作空间环境变量：

```bash
echo "source ~/uav_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4.2 拷贝源码包

把 Windows 里的源码包：

```text
D:\CUADC\src\d435i_yellow_circle_detector
```

复制到 Ubuntu：

```text
~/uav_ws/src/d435i_yellow_circle_detector
```

最终目录应为：

```text
~/uav_ws/src/d435i_yellow_circle_detector/package.xml
~/uav_ws/src/d435i_yellow_circle_detector/CMakeLists.txt
~/uav_ws/src/d435i_yellow_circle_detector/scripts/
~/uav_ws/src/d435i_yellow_circle_detector/launch/
~/uav_ws/src/d435i_yellow_circle_detector/msg/
```

### 4.3 编译工程

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
chmod +x ~/uav_ws/src/d435i_yellow_circle_detector/scripts/*.py
rospack profile
```

如果更新过 `msg` 文件，必须重新执行：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 5. 当前代码文件作用

主要文件：

```text
launch/d435i_yellow_circle.launch
scripts/d435i_camera_node.py
scripts/yellow_binarizer_node.py
scripts/yellow_circle_detector_node.py
scripts/annotated_image_viewer_node.py
scripts/binary_image_viewer_node.py
scripts/dataset_image_recorder_node.py
scripts/yolo_detector_node.py
scripts/competition_task_node.py
msg/YellowCircle.msg
msg/YoloDetection.msg
msg/YoloDetections.msg
msg/MissionTarget.msg
```

节点作用：

- `d435i_camera_node.py`: 使用 `pyrealsense2` 打开 D435/D435i，发布彩色图、对齐深度图和相机内参。
- `yellow_binarizer_node.py`: 对彩色图做 HSV 黄色过滤，发布黑白二值化图。
- `yellow_circle_detector_node.py`: 在二值化图中找黄色圆，输出圆心、半径、距离和三维坐标。
- `annotated_image_viewer_node.py`: 显示黄色圆彩色检测结果窗口。
- `binary_image_viewer_node.py`: 显示黄色二值化黑白窗口。
- `dataset_image_recorder_node.py`: 按帧保存原始彩色图片数据集，不保存视频。
- `yolo_detector_node.py`: 可选，加载 YOLO 模型检测目标，并显示最高置信度目标的中心点和距离。
- `competition_task_node.py`: 可选，根据比赛任务阶段输出投放区或灾情侦察区目标提示。

## 6. ROS 话题关系

相机节点发布：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
/d435i/color/camera_info
```

黄色二值化节点：

```text
订阅: /d435i/color/image_raw
发布: /yellow_circle/binary_image
```

黄色圆检测节点：

```text
订阅: /d435i/color/image_raw
订阅: /d435i/aligned_depth/image_raw
订阅: /d435i/color/camera_info
订阅: /yellow_circle/binary_image
发布: /yellow_circle
发布: /yellow_circle/annotated_image
```

YOLO 节点：

```text
订阅: /d435i/color/image_raw
订阅: /d435i/aligned_depth/image_raw
订阅: /d435i/color/camera_info
发布: /yolo/detection
发布: /yolo/detections
发布: /yolo/annotated_image
```

比赛任务辅助节点：

```text
订阅: /yolo/detections
发布: /competition/target
```

## 7. 启动黄色圆检测

### 7.1 不显示窗口，只发布话题

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch
```

### 7.2 显示 OpenCV 窗口

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

会显示：

```text
yellow_circle_detector  彩色检测结果
yellow_binary           黄色二值化黑白图
```

当前默认显示比例是 `1.0`。如果窗口太大，可以临时缩小：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true display_scale:=0.5
```

## 8. 距离和坐标说明

OpenCV 画面中的常见字段：

```text
r       检测到的圆半径，单位 px
raw     RealSense 深度图直接读取到的深度，单位 m
dia     根据目标实际直径和像素半径反推的距离，单位 m
cal     根据手动/自动校准得到的距离，单位 m
x       目标中心相对相机光轴的水平位置，单位 m
y       目标中心相对相机光轴的竖直位置，单位 m
z       目标中心前向距离，单位 m
d       相机到目标中心的直线距离，单位 m
```

黄色圆检测的近距离规则：

- 当估算目标距离小于 `16cm` 时，不显示 `x/y/z/d`，显示 `too_close`。
- 当估算目标距离大于或等于 `16cm` 时，只要有正数距离估计，就显示数值。
- `close_distance_m` 默认是 `0.16`。
- `close_radius_px` 默认是 `42.0`，用于辅助判断目标是否过近。

如果要调整近距离阈值：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true close_distance_m:=0.15
```

如果黄色圆实际直径不是 20cm，必须修改：

```bash
target_diameter_m:=实际直径米数
```

例如黄色圆直径是 6.5cm：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true target_diameter_m:=0.065
```

## 9. 采集图片数据集

开启采集后，只保存原始彩色图片，不保存视频。

推荐空中采集命令：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true
```

地面调试时如果需要看窗口：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

默认保存目录：

```text
~/yellow_circle_dataset/YYYYMMDD_HHMMSS/images/
```

默认保存频率：

```text
30 FPS
```

默认图片格式：

```text
jpg, quality=95
```

自定义保存目录：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true record_dir:=/home/lab/dataset record_fps:=30 image_format:=jpg jpeg_quality:=95
```

停止采集：

```text
在 roslaunch 终端按 Ctrl+C
```

## 10. 启动 YOLO 检测

YOLO 默认不启动。确认模型存在：

```bash
ls /home/lab/model/best.pt
```

启动 YOLO：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_yolo:=true yolo_model_path:=/home/lab/model/best.pt
```

YOLO 窗口行为：

- 输出置信度最高的目标数据。
- 在检测框上方显示 `x/y/z/d`。
- 在画面左下角固定显示当前最高置信度目标信息。
- 默认 `invert_camera_x:=true`，即相机向左运动时 `x` 为负，相机向右运动时 `x` 为正。

如果需要恢复普通相机光学坐标 x 方向：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_yolo:=true invert_camera_x:=false
```

## 11. 启动比赛任务辅助节点

投放区阶段：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_yolo:=true enable_competition_task:=true mission_stage:=drop yolo_model_path:=/home/lab/model/best.pt
```

灾情侦察区阶段：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_yolo:=true enable_competition_task:=true mission_stage:=disaster yolo_model_path:=/home/lab/model/best.pt
```

输出话题：

```text
/competition/target
```

说明：

- `mission_stage:=drop` 用于投放区，识别 15cm、20cm、25cm 白色圆筒。
- `mission_stage:=disaster` 用于灾情侦察区，优先处理危险化学品标识和 20cm 白色圆筒。
- D435i 只能提供相机看到的相对位置，无法单独判断无人机是否已经飞到全场 30m 或 55m 位置；这个判断需要飞控或导航系统提供。

## 12. 桌面双击启动脚本

安装桌面启动图标：

```bash
cd ~/uav_ws/src/d435i_yellow_circle_detector
bash scripts/install_desktop_launcher.sh
```

生成的启动器通常在：

```text
~/Desktop/D435i_YellowCircle_Record.desktop
```

如果 Ubuntu 桌面提示：

```text
Untrusted application launcher
```

右键图标，选择：

```text
Allow Launching
```

如果图标不在桌面上，检查真实桌面目录：

```bash
xdg-user-dir DESKTOP
ls "$(xdg-user-dir DESKTOP)"
```

启动器执行的命令等价于：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

## 13. launch 关键参数

当前主要参数：

```text
show_window              是否显示 OpenCV 窗口，默认 false
fps                      相机帧率，默认 30
display_scale            显示缩放，默认 1.0
enable_record            是否保存图片，默认 false
record_dir               图片保存根目录，默认 ~/yellow_circle_dataset
record_fps               图片保存帧率，默认跟随 fps
image_format             图片格式，默认 jpg
jpeg_quality             JPG 质量，默认 95
target_diameter_m        黄色目标实际直径，默认 0.20
close_distance_m         小于该距离不显示 x/y/z/d，默认 0.16
close_radius_px          过近半径辅助阈值，默认 42.0
enable_yolo              是否启动 YOLO，默认 false
yolo_model_path          YOLO 模型路径，默认 /home/lab/model/best.pt
yolo_conf_threshold      YOLO 置信度阈值，默认 0.5
yolo_imgsz               YOLO 输入尺寸，默认 640
yolo_device              YOLO 推理设备，默认 cpu
invert_camera_x          YOLO 输出 x 是否反向，默认 true
enable_competition_task  是否启动比赛任务辅助节点，默认 false
mission_stage            比赛阶段，默认 drop
```

## 14. 常见问题

### 14.1 找不到 launch 文件

现象：

```text
RLException: [d435i_yellow_circle.launch] is neither a launch file in package ...
```

处理：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rospack profile
roscd d435i_yellow_circle_detector
```

如果 `roscd` 失败，说明源码包没有放到：

```text
~/uav_ws/src/d435i_yellow_circle_detector
```

或者没有重新 `catkin_make`。

### 14.2 No RealSense device found

处理顺序：

```bash
lsusb | grep -i real
realsense-viewer
```

如果 `lsusb` 看不到相机：

- 换 USB 3.x / 10Gbps 接口。
- 换短一点、质量好一点的数据线。
- 确认使用的是数据线，不是只能充电的线。
- 重新插拔相机。

如果 `lsusb` 能看到但 ROS 不能打开：

```bash
sudo apt install -y librealsense2-udev-rules librealsense2-utils
sudo udevadm control --reload-rules
sudo udevadm trigger
```

然后重新插拔相机。

### 14.3 Frame didn't arrive within 5000

这通常是 USB 带宽、线材、接口或 RealSense profile 不稳定导致。

建议：

- 使用 NUC 上标注 10Gbps 的 USB 接口。
- 不要通过 USB Hub。
- 降低帧率，例如 `fps:=15`。
- 确认 `realsense-viewer` 中彩色和深度都能稳定输出。

启动示例：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true fps:=15
```

### 14.4 OpenCV 画面卡顿

建议：

- 空中采集时关闭窗口，只使用 `enable_record:=true`。
- 不启动 YOLO。
- 保持 `fps:=30`，不要上 45 或 60。
- 如果写盘慢，降低 `jpeg_quality`。

### 14.5 YOLO 无法启动

检查：

```bash
python3 -c "from ultralytics import YOLO; print('ok')"
ls /home/lab/model/best.pt
```

如果缺依赖：

```bash
pip3 install ultralytics
```

如果 NUC 没有 NVIDIA GPU，保持：

```text
yolo_device:=cpu
```

## 15. 推荐运行命令汇总

普通黄色圆检测：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

空中采集数据集：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true
```

地面调试并采集数据集：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

YOLO 检测：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_yolo:=true yolo_model_path:=/home/lab/model/best.pt
```

投放区任务辅助：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_yolo:=true enable_competition_task:=true mission_stage:=drop yolo_model_path:=/home/lab/model/best.pt
```

灾情侦察区任务辅助：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_yolo:=true enable_competition_task:=true mission_stage:=disaster yolo_model_path:=/home/lab/model/best.pt
```
