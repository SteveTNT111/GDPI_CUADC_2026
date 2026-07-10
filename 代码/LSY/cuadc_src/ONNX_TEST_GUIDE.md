# LSY 目录使用 best.onnx 测试 detector_node

## 当前状态

- 模型已移动到：`/home/lab/GDPI_CUADC_2026/代码/LSY/cuadc_src/models/best.onnx`
- `detector_node.py`、`detector_node.launch`、`run_main.launch` 默认模型已切到 `best.onnx`
- 当前机器已经有 `ultralytics`
- 当前机器已经有 `onnxruntime`（已验证可导入）

## 为什么不能直接按平常方式 roslaunch

当前机器里有两份同名 ROS 包：

- `~/catkin_ws/src/cuadc_src`
- `/home/lab/GDPI_CUADC_2026/代码/LSY/cuadc_src`

如果直接运行：

```bash
roslaunch cuadc_vision detector_node.launch
```

ROS 很可能会优先使用 `~/catkin_ws/src/cuadc_src`，而不是 `LSY` 这份代码。

## 推荐测试方式

先安装 ONNX 推理依赖：

```bash
pip3 install onnxruntime
```

然后运行这个脚本：

```bash
bash /home/lab/GDPI_CUADC_2026/代码/LSY/cuadc_src/run_detector_onnx.sh
```

这个脚本现在直接走 `LSY` 源码，不再通过 `roslaunch + devel/lib` 的旧包装脚本。它会做这些事：

1. `source /opt/ros/noetic/setup.bash`
2. `source ~/catkin_ws/devel/setup.bash`
3. 把 `/home/lab/GDPI_CUADC_2026/代码/LSY` 放到 `ROS_PACKAGE_PATH` 最前面
4. 如果 ROS master 没运行，自动启动 `roscore`
5. 直接用源码启动 `scripts/camera_node.py`
6. 直接用源码启动 `scripts/detector_node.py`

所以现在只需要一个终端，不需要再手动开 3 个终端。

## 常用命令

带窗口测试：

```bash
bash /home/lab/GDPI_CUADC_2026/代码/LSY/cuadc_src/run_detector_onnx.sh
```

无窗口测试：

```bash
bash /home/lab/GDPI_CUADC_2026/代码/LSY/cuadc_src/run_detector_onnx.sh false
```

GPU 测试：

```bash
bash /home/lab/GDPI_CUADC_2026/代码/LSY/cuadc_src/run_detector_onnx.sh true cuda:0
```

指定相机帧率：

```bash
bash /home/lab/GDPI_CUADC_2026/代码/LSY/cuadc_src/run_detector_onnx.sh true cpu 15
```

## 节点启动后怎么判断正常

终端里应看到类似信息：

```text
Detector started. model=/home/lab/GDPI_CUADC_2026/代码/LSY/cuadc_src/models/best.onnx
```

然后检查话题：

```bash
rostopic list | grep /vision
rostopic echo /vision/yolo/detection
rostopic echo /vision/bucket/info
```

如果打开了窗口，把目标放到 D435i 前面，正常现象是：

- 能看到黄色检测框
- `/vision/yolo/detection` 持续输出
- `/vision/bucket/info` 里的偏差会随目标移动变化

## 依赖检查

如果后续换环境，启动前可以先自查：

```bash
python3 -c 'import onnxruntime, ultralytics; print("deps ok")'
```
