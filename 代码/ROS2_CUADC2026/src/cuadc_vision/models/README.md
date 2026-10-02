# YOLO 权重目录

把训练好的权重放在这里，文件名建议 `best.pt`。

- detector_node.py 默认模型路径 = `<脚本目录>/../models/best.pt`：
  - 源码直跑：`src/cuadc_vision/models/best.pt`
  - `ros2 run`：`install/cuadc_vision/lib/models/best.pt`（本目录会被安装到 `lib/models`）
- 支持格式：`.pt`（YOLOv8，需 `pip3 install ultralytics`）、`.onnx`（需 `onnxruntime`）。
- 也可不管默认路径，运行时显式指定：
  `... --ros-args -p model_path:=/绝对路径/best.pt`

注意：权重文件通常较大，不建议提交到版本库。
