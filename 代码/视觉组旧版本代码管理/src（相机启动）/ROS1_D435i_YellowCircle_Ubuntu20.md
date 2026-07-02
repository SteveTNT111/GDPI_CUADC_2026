# Ubuntu 20.04 + ROS Noetic D435/D435i 相机启动说明

本项目是 **ROS1 / ROS Noetic** 代码，不是 ROS2。

Ubuntu 20.04 工作空间固定使用：

```text
~/d435i_ws
```

源码在 Ubuntu 20.04 中的存储位置为：

```text
~/d435i_ws/src/
```

包目录应为：

```text
~/d435i_ws/src/d435i_yellow_circle_detector
```

## 1. 当前启动内容

当前 `launch/d435i_yellow_circle.launch` 只启动相机节点：

```text
d435i_camera_node.py
```

该节点使用 `pyrealsense2` 打开 D435/D435i，并发布：

```text
/d435i/color/image_raw
/d435i/aligned_depth/image_raw
/d435i/color/camera_info
```

## 2. 创建工作空间并编译

第一次在 Ubuntu 20.04 上使用：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/d435i_ws/src
cp -r /path/to/CUADC/src/d435i_yellow_circle_detector ~/d435i_ws/src/
cd ~/d435i_ws
catkin_make
source ~/d435i_ws/devel/setup.bash
chmod +x ~/d435i_ws/src/d435i_yellow_circle_detector/scripts/*.py
rospack profile
```

把 `/path/to/CUADC` 换成你实际拷贝到 Ubuntu 里的路径。

如果是覆盖旧版本代码，覆盖后重新编译：

```bash
cd ~/d435i_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

每次打开新终端后都要先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/d435i_ws/devel/setup.bash
```

## 3. 启动相机

启动 D435/D435i 相机节点：

```bash
cd ~/d435i_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch
```

如果 30 FPS 在当前电脑上卡顿，可以临时降到 15 FPS：

```bash
roslaunch d435i_yellow_circle_detector d435i_yellow_circle.launch fps:=15
```

## 4. 创建桌面双击启动图标

在 Ubuntu 中执行一次：

```bash
cd ~/d435i_ws/src/d435i_yellow_circle_detector
bash scripts/install_desktop_launcher.sh
```

执行后桌面会生成：

```text
D435i_YellowCircle_Record.desktop
```

脚本会自动识别 Ubuntu 的真实桌面目录。不同系统可能是 `~/Desktop`、`~/桌面`，或者 `xdg-user-dir DESKTOP` 返回的路径。

以后双击这个桌面图标即可启动 D435/D435i 相机节点。

如果 Ubuntu 提示 `Untrusted application launcher`，在桌面图标上右键，选择 `Allow Launching`。如果仍不能双击启动，执行：

```bash
chmod +x "$(xdg-user-dir DESKTOP)/D435i_YellowCircle_Record.desktop"
chmod +x ~/d435i_ws/src/d435i_yellow_circle_detector/scripts/start_d435i_record.sh
```

## 5. launch 关键参数

当前 launch 只保留相机帧率参数：

```xml
<arg name="fps" default="30" />
```

相机节点参数：

```xml
<param name="color_width" value="640" />
<param name="color_height" value="480" />
<param name="depth_width" value="640" />
<param name="depth_height" value="480" />
<param name="fps" value="$(arg fps)" />
<param name="color_format" value="bgr8" />
<param name="wait_timeout_ms" value="5000" />
<param name="timeout_restart_count" value="3" />
<param name="auto_profile_fallback" value="true" />
<param name="align_depth_to_color" value="true" />
<param name="frame_id" value="d435i_color_optical_frame" />
<param name="color_topic" value="/d435i/color/image_raw" />
<param name="depth_topic" value="/d435i/aligned_depth/image_raw" />
<param name="camera_info_topic" value="/d435i/color/camera_info" />
```

## 6. ROS 找不到 launch 的修复

如果出现：

```text
RLException: [d435i_yellow_circle.launch] is neither a launch file in package [d435i_yellow_circle_detector]
```

说明当前终端没有找到这个 ROS 包。执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/d435i_ws/devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

如果仍然找不到，重新编译：

```bash
cd ~/d435i_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
rospack profile
rospack find d435i_yellow_circle_detector
```

## 7. 常见清理命令

如果出现同名节点冲突：

```bash
rosnode kill /d435i_camera_node
```
