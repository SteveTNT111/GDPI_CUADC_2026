# Ubuntu 20.04 + ROS Noetic D435/D435i 黄色圆检测与图片数据集采集说明

本项目是 **ROS1 / ROS Noetic** 代码，不是 ROS2。Ubuntu 工作空间固定使用：

```text
~/uav_ws
```

源码包目录：

```text
~/uav_ws/src/d435i_yellow_circle_detector
```

当前默认参数：

```text
相机分辨率: 640x480
默认帧率: 30 FPS
检测方法: HSV 黄色过滤 -> 二值化 -> 轮廓/圆度筛选
目标直径: 0.20 m
位置输出: 相机坐标系下 (x, y, z) 和距离 d
显示窗口: 默认关闭
数据采集: 默认关闭, 开启后只保存图片, 不保存视频
```

## 1. 功能节点

- `d435i_camera_node.py`: 使用 `pyrealsense2` 打开 D435/D435i, 发布彩色图和对齐后的深度图。
- `yellow_binarizer_node.py`: 对彩色图做黄色 HSV 过滤, 发布黑白二值图。
- `yellow_circle_detector_node.py`: 在二值图中找轮廓并筛选圆, 发布检测结果、三维相对位置和标注图。
- `annotated_image_viewer_node.py`: 显示带检测标注的彩色 OpenCV 窗口。
- `binary_image_viewer_node.py`: 显示黑白二值化 OpenCV 窗口。
- `dataset_image_recorder_node.py`: 采集图片数据集, 只保存照片, 不保存视频。

## 2. 创建工作空间并编译

第一次在 Ubuntu 20.04 上使用：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/uav_ws/src
cp -r /path/to/CUADC/src/d435i_yellow_circle_detector ~/uav_ws/src/
cd ~/uav_ws
catkin_make
source ~/uav_ws/devel/setup.bash
chmod +x ~/uav_ws/src/d435i_yellow_circle_detector/scripts/*.py
rospack profile
```

把 `/path/to/CUADC` 换成你实际拷贝到 Ubuntu 里的路径。

如果你是覆盖旧版本代码, 因为 `msg/YellowCircle.msg` 新增了三维位置字段, 覆盖后也必须重新编译：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

每次打开新终端后都要先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
```

## 3. 普通检测启动

只检测和发布话题, 不显示窗口, 不采集图片：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch
```

显示两个 OpenCV 窗口：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true
```

窗口名称：

```text
yellow_circle_detector  彩色标注画面
yellow_binary           黄色二值化黑白画面
```

默认显示比例是 `1.0`。如果屏幕太小, 可以临时缩小：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true display_scale:=0.5
```

## 4. 空中采集图片数据集

开启采集后, 程序会以默认 `30 FPS` 保存相机彩色图片。不会保存 `.avi` 或其他视频文件。

推荐空中采集命令, 不开 OpenCV 窗口, 降低机载电脑负载：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true
```

如果地面调试时需要同时看检测画面：

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

文件名格式：

```text
d435i_raw_000001_时间戳.jpg
d435i_raw_000002_时间戳.jpg
...
```

指定保存目录、帧率和图片格式：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch enable_record:=true record_dir:=/home/lab/dataset record_fps:=30 image_format:=jpg jpeg_quality:=95
```

近距离测试时, 如果实物距离相机不超过 20cm, 建议显式限制最大有效深度：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true depth_max_valid_m:=0.35
```

如果黄色圆的实际直径不是 20cm, 必须把 `target_diameter_m` 改成你实际测量的黄色圆直径。例如黄色圆直径是 6.5cm：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true target_diameter_m:=0.065 depth_max_valid_m:=0.35
```

如果你现在测试的不是 20cm 圆筒, 而是瓶身标签上的小黄色圆/椭圆, 不建议继续使用默认 `target_diameter_m=0.20`。可以用已知距离做一次自动校准。例如先把目标放在相机前方实测 `0.18m` 处, 启动：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true auto_calibrate_distance:=true calibration_distance_m:=0.18 depth_max_valid_m:=0.35
```

启动后保持目标不动约 1 秒, 程序会采集 30 帧半径并自动校准。后续距离会使用 `depth_source=calibrated`。

停止采集：在 roslaunch 终端按 `Ctrl+C`。节点退出时会打印本次保存图片数量和丢帧数量。

采集建议：

- 空中采集建议关闭 `show_window`, 只保留 `enable_record:=true`。
- 使用 SSD 或速度较快的存储盘, 避免写盘太慢导致队列满。
- 如果出现 `Dataset image queue is full`, 可以降低 `jpeg_quality` 或临时降低 `record_fps`。
- 640x480 JPG 30FPS 会持续占用磁盘空间, 起飞前确认剩余容量。

## 5. 创建桌面双击启动图标

如果想在 Ubuntu 20.04 桌面上双击启动下面这条命令：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true
```

在 Ubuntu 中执行一次：

```bash
cd ~/uav_ws/src/d435i_yellow_circle_detector
bash scripts/install_desktop_launcher.sh
```

执行后桌面会生成：

```text
D435i_YellowCircle_Record.desktop
```

脚本会自动识别 Ubuntu 的真实桌面目录。不同系统可能是 `~/Desktop`、`~/桌面`，或者 `xdg-user-dir DESKTOP` 返回的路径。

以后双击这个桌面图标即可启动检测、显示两个 OpenCV 窗口并同时保存图片数据集。

如果 Ubuntu 提示 `Untrusted application launcher`, 在桌面图标上右键, 选择 `Allow Launching`。如果仍不能双击启动, 执行：

```bash
chmod +x "$(xdg-user-dir DESKTOP)/D435i_YellowCircle_Record.desktop"
chmod +x ~/uav_ws/src/d435i_yellow_circle_detector/scripts/start_d435i_record.sh
```

如果终端能看到 `~/Desktop/D435i_YellowCircle_Record.desktop`, 但图形桌面看不到, 先查看真实桌面目录：

```bash
xdg-user-dir DESKTOP
ls "$(xdg-user-dir DESKTOP)"
```

## 6. ROS 话题关系

相机节点发布：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
/d435i/color/camera_info
```

黄色二值化节点订阅：

```text
/d435i/color/image_raw
```

黄色二值化节点发布：

```text
/yellow_circle/binary_image
```

黄色圆检测节点订阅：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
/yellow_circle/binary_image
/d435i/color/camera_info
```

黄色圆检测节点发布：

```text
/yellow_circle                 d435i_yellow_circle_detector/YellowCircle
/yellow_circle/annotated_image sensor_msgs/Image, bgr8
```

`/yellow_circle` 中新增的三维位置字段：

```text
raw_depth_m     RealSense 深度图直接读到的原始深度
diameter_depth_m 根据目标实际直径和图像半径反推的距离
calibrated_depth_m 根据已知距离校准反推的距离
depth_source    最终采用的测距来源: depth / diameter / calibrated / none
position_valid  是否成功计算三维位置
camera_x_m      目标在相机光学坐标系下的 x, 单位 m
camera_y_m      目标在相机光学坐标系下的 y, 单位 m
camera_z_m      目标在相机光学坐标系下的 z, 单位 m
distance_m      相机到目标中心的直线距离 d, 单位 m
body_x_m        目标在机体系下的 x, 未启用手眼标定时等于 camera_x_m
body_y_m        目标在机体系下的 y, 未启用手眼标定时等于 camera_y_m
body_z_m        目标在机体系下的 z, 未启用手眼标定时等于 camera_z_m
body_distance_m 机体系下距离, 未启用手眼标定时等于 distance_m
```

相机光学坐标系采用 ROS 常用约定：`x` 向右, `y` 向下, `z` 向前。

图片数据集采集节点订阅：

```text
/d435i/color/image_raw
```

## 7. launch 关键参数

```xml
<arg name="show_window" default="false" />
<arg name="fps" default="30" />
<arg name="display_scale" default="1.0" />
<arg name="enable_record" default="false" />
<arg name="record_dir" default="$(env HOME)/yellow_circle_dataset" />
<arg name="record_fps" default="$(arg fps)" />
<arg name="image_format" default="jpg" />
<arg name="jpeg_quality" default="95" />
<arg name="target_diameter_m" default="0.20" />
<arg name="depth_max_valid_m" default="0.35" />
<arg name="auto_calibrate_distance" default="false" />
<arg name="calibration_distance_m" default="0.20" />
<arg name="calibration_radius_px" default="0.0" />
```

黄色 HSV 阈值在 `yellow_binarizer_node` 中：

```xml
<param name="h_min" value="18" />
<param name="h_max" value="42" />
<param name="s_min" value="80" />
<param name="s_max" value="255" />
<param name="v_min" value="80" />
<param name="v_max" value="255" />
```

二值图形态学处理参数：

```xml
<param name="morph_kernel" value="5" />
<param name="morph_open_iterations" value="1" />
<param name="morph_close_iterations" value="2" />
```

圆筛选参数：

```xml
<param name="min_area" value="120.0" />
<param name="max_area" value="200000.0" />
<param name="min_radius" value="6.0" />
<param name="min_circularity" value="0.55" />
<param name="center_roi_ratio" value="0.85" />
<param name="target_diameter_m" value="0.20" />
<param name="use_diameter_depth_fallback" value="true" />
<param name="prefer_diameter_depth" value="true" />
<param name="depth_min_valid_m" value="0.05" />
<param name="depth_max_valid_m" value="0.35" />
<param name="depth_disagreement_ratio" value="0.35" />
<param name="auto_calibrate_distance" value="false" />
<param name="calibration_distance_m" value="0.20" />
<param name="calibration_radius_px" value="0.0" />
<param name="calibration_samples" value="30" />
```

`target_diameter_m=0.20` 对应图中的圆筒直径 20 cm。这个值必须等于被识别黄色圆的实际直径, 不是目标到相机的距离。

近距离场景下 D435/D435i 深度图可能不稳定。当前默认认为 `0.05m ~ 0.35m` 是有效测试范围, 如果 RealSense 原始深度超过这个范围, 或者与直径反推结果差异过大, 程序会优先采用 `diameter_depth_m`。

查看最终采用了哪种测距方式：

```bash
rostopic echo /yellow_circle/depth_source
```

查看三种测距来源的数值：

```bash
rostopic echo /yellow_circle/raw_depth_m
rostopic echo /yellow_circle/diameter_depth_m
rostopic echo /yellow_circle/calibrated_depth_m
```

手动校准也可以不自动采样：先看画面左下角或 `/yellow_circle/radius` 中的当前像素半径, 例如目标在 `0.18m` 时半径约 `42px`, 下次启动可用：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true enable_record:=true calibration_distance_m:=0.18 calibration_radius_px:=42 depth_max_valid_m:=0.35
```

手眼标定/机体系转换参数默认关闭：

```xml
<param name="enable_body_transform" value="false" />
<param name="camera_to_body_x_m" value="0.0" />
<param name="camera_to_body_y_m" value="0.0" />
<param name="camera_to_body_z_m" value="0.0" />
<param name="camera_to_body_roll_deg" value="0.0" />
<param name="camera_to_body_pitch_deg" value="0.0" />
<param name="camera_to_body_yaw_deg" value="0.0" />
```

完成相机到机体的手眼标定后, 把 `enable_body_transform` 改为 `true`, 再填入相机坐标系到机体系的平移和欧拉角即可输出机体系坐标。

## 8. ROS 找不到 launch 的修复

如果出现：

```text
RLException: [d435i_yellow_circle.launch] is neither a launch file in package [d435i_yellow_circle_detector]
```

说明当前终端没有找到这个 ROS 包。执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

如果仍然找不到, 重新编译：

```bash
cd ~/uav_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

## 9. 常见清理命令

如果出现同名节点冲突：

```bash
rosnode kill /d435i_camera_node
rosnode kill /yellow_binarizer_node
rosnode kill /yellow_circle_detector_node
rosnode kill /annotated_image_viewer_node
rosnode kill /binary_image_viewer_node
rosnode kill /dataset_image_recorder_node
```

如果 30FPS 在当前电脑上卡顿, 可以临时降到 15FPS：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch show_window:=true fps:=15
```
