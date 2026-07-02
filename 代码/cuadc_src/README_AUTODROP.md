# 自动抛投节点说明

文件: `scripts/auto_drop_node.py`

快速说明：
- 订阅 `/vision/yolo/detection` (消息类型 `YoloDetection`)。
- 订阅 `/vision/color/camera_info` 获取相机光心用于像素差计算。
- 当检测中心与相机中心的像素距离小于 `pixel_threshold` 且置信度满足 `min_conf` 时，调用 `/mavros/cmd/command` 触发 `MAV_CMD_DO_SET_SERVO`（command=183）。

运行示例：

roslaunch cuadc_vision auto_drop.launch

可调参数示例（launch 或 rosparam）：
- `~pixel_threshold` (float) 像素阈值，默认 20.0
- `~min_conf` (float) 最小置信度，默认 0.5
- `~channel` (int) 舵机通道号，默认 9

注意：脚本为新增文件，不修改 `cuadc_src` 现有其它文件。
