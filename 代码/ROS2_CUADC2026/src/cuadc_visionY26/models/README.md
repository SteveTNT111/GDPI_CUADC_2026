# YOLO26 ONNX 权重目录（cuadc_visionY26）

把台式机训练并导出的 **YOLO26 ONNX** 权重放到这里，文件名用：

```
best_yolo26.onnx
```

## 放哪里

- 源目录（放这里，然后 `colcon build`）：
  ```
  ~/ROS2_CUADC2026/src/cuadc_visionY26/models/best_yolo26.onnx
  ```
- 构建后被安装到这里（`ros2 run` 默认找这个）：
  ```
  ~/ROS2_CUADC2026/install/cuadc_visionY26/lib/models/best_yolo26.onnx
  ```

## 两种用法

1. **默认路径**（推荐）：把文件命名为 `best_yolo26.onnx` 放进本目录 → 重新
   `colcon build --packages-select cuadc_visionY26`，然后正常启动即可，不用传参数。
2. **临时指定**（不用重新 build）：启动时显式传绝对路径
   ```
   ros2 run cuadc_visionY26 detector_node.py --ros-args \
     -p model_path:=$HOME/ROS2_CUADC2026/src/cuadc_visionY26/models/best_yolo26.onnx
   ```

## 导出命令（在台式机训练机上）

```bash
yolo export model=best.pt format=onnx imgsz=640 opset=18 simplify=True
# 想更快可用 imgsz=416（端到端约 2x），但注意小目标召回
```

- 类别顺序/名称必须和训练时一致（`detector_node` 的 `class_names` 参数可覆盖显示名）。
- 本包默认 `target_classes` 关键词仍是圆筒相关；若类别名不同，运行时改
  `-p target_classes:=...`。
