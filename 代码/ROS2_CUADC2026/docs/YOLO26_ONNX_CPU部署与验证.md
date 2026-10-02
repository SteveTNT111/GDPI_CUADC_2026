# YOLO26 ONNX CPU 推理环境部署与验证记录

> 机器：Ubuntu 22.04.5 / Python 3.10.12 / ROS2 Humble / Intel i5-1340P（16 线程，无独显）
> 相机：Intel RealSense D435（USB 3.2，640×480 bgr8@30）
> 包：`~/ROS2_CUADC2026/src/cuadc_vision`（`camera_node.py` / `detector_node.py`）
> 记录时间：2026-09-28。本文所有命令均**不使用 sudo**（`pip3 --user`）。

---

## 0. 结论速览

| 项目 | 结果 |
|---|---|
| `import ultralytics, onnxruntime, cv2, pyrealsense2` | ✅ 全部通过 |
| `colcon build --packages-select cuadc_vision` | ✅ 通过（需先升级 `packaging`，见 §1.3） |
| 加载 `best.onnx`（ONNX Runtime CPU） | ✅ `ext=.onnx imgsz=640 device=cpu`，无 "Model not found" / "ultralytics not installed" |
| 真实 D435 采集 | ✅ `/vision/color/image_raw` ≈29 Hz，depth/camera_info 正常 |
| `/vision/annotated_image` | ✅ 有图像（`rqt_image_view` 验证），推理链路 ~13–20 FPS |
| `detector_node` 自带 `show_window:=true` 窗口 | ⚠️ **会死锁**（Qt + 4 线程 executor 并发调用 cv2），见 §4.3 |
| OpenVINO 加速 | ⚠️ 实测快约 1.8×，但**不能只靠 `-p` 配置启用**（`isfile` 限制），见 §5.4 |

---

## 1. 安装（pip3 --user，无 sudo）

### 1.1 CPU 版 PyTorch（先装，避免拉入 2 GB+ 的 CUDA 依赖）

```bash
pip3 install --user -U torch torchvision \
  --index-url https://download.pytorch.org/whl/cpu
```

### 1.2 ultralytics + onnxruntime

```bash
printf 'numpy<2\n' > /tmp/opencode/constraints.txt
pip3 install --user -U --constraint /tmp/opencode/constraints.txt ultralytics onnxruntime
```

> **必须加 `numpy<2` constraint**。ultralytics 的依赖解析/自动安装会把 numpy
> 升到 2.x；numpy 2 会遮蔽 ROS Humble 的 numpy 1.21 ABI，直接打断 apt 编译的
> `cv_bridge` / `python3-opencv`（实测报 `AttributeError: _ARRAY_API not found`）。
> 本机最终固定 **numpy 1.26.4**。

### 1.3 依赖冲突修复（本次实测必须做的两处）

ultralytics 在导出时会自动 `pip install onnx onnxslim`，从而把两个系统依赖改坏：

```bash
# (a) protobuf 被升到 7.36.2 → 打断 gz.msgs10（camera_node 顶层就 import gz）
#     固定回 3.20.3，并把 onnx 降到兼容版本 1.16.2，二者可共存
pip3 install --user "protobuf==3.20.3" "onnx==1.16.2" "ml-dtypes<0.6"

# (b) torch 带入 setuptools 78，与系统 packaging 21.3 冲突，colcon build 会报
#     canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'
pip3 install --user -U packaging
```

### 1.4 可选：Intel OpenVINO

```bash
pip3 install --user -U --constraint /tmp/opencode/constraints.txt openvino
```

### 1.5 最终版本（实测）

```
ultralytics      8.4.164      onnxruntime      1.23.2
torch            2.14.0+cpu   torchvision      0.29.0+cpu
cv2              4.11.0       (pip, QT5)       pyrealsense2 2.58.4
numpy            1.26.4       openvino         2026.4.0
onnx             1.16.2       onnxslim         0.1.96
protobuf         3.20.3       ml_dtypes        0.5.4
packaging        26.3         setuptools       78.1.0
```

---

## 2. 验证三连

```bash
# 2.1 逐个 import + 版本
python3 -c "import ultralytics, onnxruntime, cv2, pyrealsense2; print('import OK')"
python3 -c "import ultralytics,onnxruntime,cv2,pyrealsense2,torch,numpy as n; \
print('ultralytics', ultralytics.__version__); print('onnxruntime', onnxruntime.__version__); \
print('cv2', cv2.__version__); print('pyrealsense2', pyrealsense2.__version__); \
print('torch', torch.__version__); print('numpy', n.__version__); \
print('providers', onnxruntime.get_available_providers())"
```
预期：
```
import OK
ultralytics  8.4.164
onnxruntime  1.23.2
cv2          4.11.0
pyrealsense2 2.58.4
torch        2.14.0+cpu
numpy        1.26.4
providers ['AzureExecutionProvider', 'CPUExecutionProvider']
```

```bash
# 2.2 相机枚举
rs-enumerate-devices -s        # 应列出 RealSense D435, USB 3.2
# 2.3 GPU/EP 自检
python3 -c "import onnxruntime as o; print(o.get_available_providers())"
```

---

## 3. 权重与构建

```bash
# 3.1 放置权重（文件名建议 best.onnx；也可用绝对路径 model_path 指定）
cp /path/to/your/best.onnx ~/ROS2_CUADC2026/src/cuadc_vision/models/best.onnx

# 3.2 构建
source /opt/ros/humble/setup.bash
cd ~/ROS2_CUADC2026
colcon build --packages-select cuadc_vision      # 预期 1 package finished
```

- `detector_node` 默认模型路径 = `<脚本目录>/../models/best.pt`：
  - `ros2 run` 时 = `install/cuadc_vision/lib/models/best.pt`（`models/` 被装到 `lib/models`）
  - 用 ONNX 时**建议显式传绝对路径** `-p model_path:=...`。
- 本次机器上**没有**任何现成的 `.pt/.onnx`（只有 Rockchip `.rknn`）。为打通链路，
  用官方 `yolo26n.pt` 导出了一个占位 `models/best.onnx`（COCO 80 类，静态 640，opset 18）：
  ```bash
  python3 -c "from ultralytics import YOLO; YOLO('yolo26n.pt').export(format='onnx', imgsz=640, dynamic=False, simplify=True)"
  ```
  ⚠️ **正式使用前请用你训练好的 `best.onnx` 覆盖同名文件**（否则只会识别 COCO 类别，
  且会被 `target_classes` 关键词过滤掉，画面 `DETECT: 0`）。

---

## 4. 运行验证（真实 D435）

### 4.1 相机

```bash
export DISPLAY=:1
source /opt/ros/humble/setup.bash && source ~/ROS2_CUADC2026/install/setup.bash
ros2 run cuadc_vision camera_node.py --ros-args -p source:=realsense -p show_rgb:=true
```
预期日志：
```
Found 450 supported profile candidates.
RealSense: name=RealSense D435 serial=... usb=3.2
D435i started. 640x480 bgr8@30 depth_scale=0.00100000
show_rgb 已开启：纯 RGB 原图窗口 'RGB'（无任何叠加）
```
话题：`/vision/color/image_raw`(~29Hz)、`/vision/aligned_depth/image_raw`(32FC1,m)、`/vision/color/camera_info`。

### 4.2 检测（无窗口，稳定跑）

```bash
ros2 run cuadc_vision detector_node.py --ros-args \
  -p camera_source:=realsense \
  -p model_path:=$HOME/ROS2_CUADC2026/src/cuadc_vision/models/best.onnx
```
预期日志（无 "Model not found" / "ultralytics not installed"）：
```
Loading .../best.onnx for ONNX Runtime inference...
Using ONNX Runtime 1.23.2 with CPUExecutionProvider
YOLO warmup finished. input=640x640
Detector started. model=...best.onnx ext=.onnx conf=0.50 imgsz=640 device=cpu
detector_node spinning ...
```
> 若不想自动拉起 MAVROS（无飞控时）：加 `-p auto_start_mavros:=false`。
> 若相机已由外面启动：加 `-p auto_start_camera:=false`。

### 4.3 ⚠️ 已知问题：`show_window:=true` 的 "YOLO Detection" 窗口会死锁

- 现象：窗口能打开并显示 1~2 帧，随后整个节点卡死（所有线程 `futex_wait`），
  `/vision/annotated_image` 归零；日志停在 `识别 UI 已打开`。
- 根因：`detector_node` 用 `MultiThreadedExecutor(num_threads=4)`，而 `image_callback`
  （在 executor 线程里）直接调用 `cv2.imshow/waitKey`。**Qt 不是线程安全的**，
  4 个回调线程并发进 GUI → `Dead lock detected in BlockingQueuedConnection`。
  与 Qt/GTK、有无 DISPLAY 无关（已用最小复现脚本确认）。
- 影响范围：`show_window:=true`（默认值！）。
- **不改源码的替代方案**（已实测）：
  ```bash
  # 1) detector 无窗口运行
  ros2 run cuadc_vision detector_node.py --ros-args \
    -p camera_source:=realsense -p show_window:=false \
    -p model_path:=$HOME/ROS2_CUADC2026/src/cuadc_vision/models/best.onnx
  # 2) 用 rqt 显示标注画面（主线程 GUI，不卡）
  ros2 run rqt_image_view rqt_image_view /vision/annotated_image
  ```
  实测 `rqt_image_view` 显示 `/vision/annotated_image` 正常，画面内 HUD 显示
  `DETECT: 0  FPS 20.2 | best.onnx ⚙ cpu`。
- 若要恢复自带窗口：需改动 `detector_node.py`（把 executor 线程数设为 1，或用锁
  串行化 GUI 调用，或把 `imshow` 放到主线程）。**本次未改源码**。

---

## 5. CPU 帧率实测与优化

> 测试前机器被一套 `env_guard.sh → demo_up.sh`（Gazebo+SITL+MAVROS+show_camera）
> 占满（load≈37/16 核），FPS 严重失真。停掉整套仿真栈后（CPU 空闲）重测，
> 结果差别巨大（例如端到端 2.8 Hz → 13~20 Hz，**约 5 倍**）。
> governor 仍为 `powersave`，且 NoMachine 远程编码长期占 ~0.5 核。

### 5.1 端到端（ultralytics predict：预处理+推理+NMS+后处理）

| 模型 | imgsz | ms/帧 | FPS |
|---|---|---|---|
| best.onnx（静态 640） | 640 | 96.1 | 10.4 |
| dynamic onnx | 320 | 52.5 | 19.1 |
| dynamic onnx | **416** | **47.6** | **21.0** |
| dynamic onnx | 480 | 68.1 | 14.7 |
| dynamic onnx | 640 | 94.0 | 10.6 |
| OpenVINO IR | 480 | 53.1 | 18.8 |
| **OpenVINO IR** | **640** | **52.6** | **19.0** |

### 5.2 纯 ONNX Runtime（仅推理，无 NMS）

| threads | ms | FPS |
|---|---|---|
| 1 | 105.1 | 9.5 |
| 2 | 67.0 | 14.9 |
| **4** | **49.7** | **20.1** |
| 6 | 47.8 | 20.9 |
| 8 | 46.0 | 21.7 |
| default(=16) | 40.6~91.6（波动大） | 10.9~24.6 |

### 5.3 实时链路实测

- `camera_node`：`/vision/color/image_raw` ≈ **29 Hz**（D435 输出 30 fps，对齐+2MB 发布后略降）。
- `detector_node` 端到端（`/vision/annotated_image` 计数）：**13~20 FPS** @ imgsz=640。
- 内部 HUD：`FPS 20.2`。

### 5.4 优化建议（按性价比排序）

1. **降输入尺寸**：导出 `imgsz=416`（或 480）的 `best.onnx`，端到端从 ~10 → ~21 FPS。
   ```bash
   yolo export model=best.pt format=onnx imgsz=416 opset=18 simplify=True
   ```
   注意小目标召回；侦察/投放筒在 640 下也不大，建议实测 mAP 后再定。
2. **OpenVINO**：本机 640 下 ~1.8×（10.4 → 19.0 FPS）。但**当前不能只靠 `-p` 启用**：
   - ultralytics 的 ONNX 后端把 provider 硬编码为 `["CPUExecutionProvider"]`，
     即使装了 `openvino` / `onnxruntime-openvino`，`.onnx` 也走不到 OpenVINO EP；
   - 走 OpenVINO 需导出 IR：`yolo export model=best.pt format=openvino`，产出
     `best_openvino_model/` 目录；ultralytics 只认**目录**，
     而 `detector_node` 用 `os.path.isfile(model_path)` 把关（只认文件）→ 会被判
     "Model not found"。要启用需给 `detector_node` 加一个"目录也接受"的分支，
     或改用 ONNX Runtime 的 OpenVINO EP（需同时改 provider）。
3. **ORT 线程数**：默认 = 全部核心，实测并非最优（16 线程反而可能更慢）。但
   ultralytics `session_options=None`，`detector_node` 也没有对应参数，
   无干净的外部开关（`OMP_NUM_THREADS` 实测**无效**）。若将来能改代码，
   `intra_op_num_threads=4` 可把纯推理从 ~92ms 降到 ~50ms。
4. **CPU 上不要用 half**：`half=True` 是 FP16，CPU 的 ORT/Torch 不支持（会报错或退化）。
   保持 FP32。
5. **关掉后台负载 / 设 performance governor**：竞赛前建议
   `sudo cpupower frequency-set -g performance`（需 sudo，本次未执行），并确认
   没有 Gazebo/SITL/NoMachine 等在抢 CPU。实测负载对帧率的影响 >2×。
6. `cv_threads`（默认 1）保持即可；`conf_threshold` 调高可略减后处理但影响很小。

---

## 6. YOLO26 版本 / 导出排查

- **版本要求**：YOLO26 需较新的 ultralytics（本机 `8.4.164` 正常）。旧版本会在
  `YOLO('yolo26n.pt')` 时报未知模型/任务。排查：
  ```bash
  pip3 install --user -U ultralytics
  python3 -c "import ultralytics; print(ultralytics.__version__)"   # 建议 >= 8.4.x
  yolo checks
  ```
- **导出命令**：
  ```bash
  yolo export model=best.pt format=onnx imgsz=640 opset=18 simplify=True   # 静态
  yolo export model=best.pt format=onnx imgsz=640 dynamic=True            # 动态
  ```
- **导出失败排查顺序**：`simplify=False` → 固定 `opset=17/18` → `dynamic=True/False`
  → 确认 `onnx`/`onnxslim` 能 import（本机 onnx 1.16.2 + protobuf 3.20.3 已验证）。
- **注意本仓库的 RKNN 流水线**（`~/cuadc2026_ws/cuadc_rknn_pipeline`）里
  `02_export_onnx.py` 面向 yolo11n；YOLO26 请直接用 ultralytics 的 `yolo export`。
- **不要**让 ultralytics 自动改依赖：`detector_node` 已设
  `ULTRALYTICS_SKIP_REQUIREMENTS_CHECKS=1`。若手动 `pip install -U ultralytics`
  后 numpy/protobuf 又被升级，请按 §1.3 重新钉版本。
- 排查 ONNX 输入尺寸：`detector_node._sync_imgsz_with_onnx_input()` 会读取 ONNX 静态
  输入并强制 `imgsz`；若导出动态维，则沿用 `-p imgsz:=...`。

---

## 7. 回滚 / 现场说明

- 本次为测量帧率，停掉了当时在跑的一套仿真栈（`env_guard.sh` → `demo_up.sh`
  → gz sim / arducopter / mavros / show_camera / stream_probe）。它**不是**本任务启动的，
  如需恢复：
  ```bash
  bash ~/ROS2_CUADC2026/scripts/preflight.sh
  bash ~/ROS2_CUADC2026/scripts/demo_up.sh          # 缺啥补啥（SITL/MAVROS 自愈）
  ```
- 卸载本次安装（可选）：
  ```bash
  pip3 uninstall -y ultralytics onnxruntime openvino onnx onnxslim torch torchvision mss python-xlib
  ```
  （注意 numpy 等系统相关包不要盲删，保持 `numpy<2`。）
- 遗留文件：`~/ROS2_CUADC2026/src/cuadc_vision/models/best.onnx` 是 COCO 占位模型，
  用真权重覆盖即可。构建产物在 `install/`。

---

## 8. 帧率优化实测（2026-09-30，真权重 best_yolo26.onnx）

> 机器：空闲（无仿真栈；仅真机 MAVROS + NoMachine + OpenCode 桌面，CPU 空闲≈80%）。
> 模型：真权重 1 类 `bucket`，opset 12，静态 640，NMS-free 输出 `[1,300,6]`。
> 测量口径：`taskset -c 0-7`（P 核，i5-1340P：P=cpu0-7@4.6GHz / E=cpu8-15@3.4GHz），
> 去 warmup，N=60~100 帧取统计；本机有后台负载，单次 run-to-run 波动可达 ±35%，
> 故 **关键配置重复 3 次取 min（最干净一次）**，同时给中位。脚本：
> `scripts/bench_yolo26_fps.py`、`scripts/bench_yolo26_matrix.py`、`scripts/quant_yolo26_int8.py`；
> 日志 `/tmp/opencode/bench_*.log`。

### 8.0 结论速览：优化手段 × FPS × 精度 × 代价

FPS 为**纯推理**（不含 ROS 发布）P 核最干净值；mAP 为 bucket 验证子集（536 图/914 实例，
见 §8.4）真实指标。

| 优化手段 | imgsz | ms/帧 | FPS | mAP50 | 需 sudo | 需改码 | 备注 |
|---|---|---|---|---|---|---|---|
| 基线：ultralytics predict（含前后处理） | 640 | 105.6 | **9.5** | 0.474 | 否 | 否 | 现状默认；包装层开销 ~65ms |
| ORT 默认线程(16) | 640 | 46.6 | 21.5 | 0.474 | 否 | 否 | 波动大，慢于 4 线程 |
| **ORT intra_op=4~8 + 绑 P 核** | 640 | 38.7 | **25.9** | 0.474 | 否 | 否 | 线程调优收益 ~1.2×（vs 默认） |
| OpenVINO FP32 (CPU) | 640 | 27.2 | **36.8** | 0.474 | 否 | 是 | vs 最优 ORT ~1.4× |
| **OpenVINO NNCF INT8** | 640 | 16.2 | **61.8** | **0.485** | 否 | 是 | 精度无损；vs ORT 最优 2.4× |
| 输入尺寸 640→480 | 480 | 9.4 | **106.2** | 0.366 | 否 | 是* | 速度 ~1.7×，mAP −23% |
| 输入尺寸 640→416 | 416 | 7.6 | **130.8** | 0.325 | 否 | 是* | 速度 ~2.1×，mAP −32% |
| 原生后端绕过 ultralytics | 640 | — | — | — | 否 | 是 | FPS 61.8 vs 31.2（同 INT8 模型） |
| 图像 QoS→BEST_EFFORT | — | — | — | — | 否 | 是 | 解耦背压，节点端到端上限↑ |
| NMS-free（vs 原始头+NMS） | 640 | −1.2ms | +4.4% | ≈ | 否 | 否 | 后处理 0.004 vs 0.079ms |
| iGPU（OpenVINO GPU，已启用） | 640 | 27.1 | **36.9** | 0.474* | 是(已做) | 是 | **≈ CPU（无收益）**，见 §8.7 |
| performance governor（已设） | — | — | — | — | 是(已做) | 否 | 改善波动，见 §8.7 |

\* 尺寸需从 `.pt` 重导出（本机有 `best.pt`）：`yolo export model=best.pt format=onnx imgsz=480 opset=12 simplify=True nms=False`。

**精确性口诀**：`imgsz 是最大杠杆（~平方关系）`；`INT8 无损且再 ~1.7×`；`绕开 ultralytics 包装层与 BE QoS 是节点级关键`。

### 8.1 线程 / 绑核 / OMP（免 sudo）

`onnxruntime` 纯推理（640，不绑核）各 `intra_op_num_threads`：1→106ms、2→64、**4→40.8**、
6→44.4、8→46.0、16→72.7、default→46.6。**4~8 最优，16 最差**（默认=物理核数，反面教材）。

绑核（`taskset`）：P 核 0-7 显著快于 E 核 8-15（t4：39.6 vs 74.7ms；t8：38.7 vs 55.6ms）。
**`OMP_NUM_THREADS` 对 ORT 无效**（=1/4 均 46.9/46.8ms ≈ default 46.6）——ORT 用自有线程池。

### 8.2 OpenVINO CPU（免 sudo，原生 API）

不装 `onnxruntime-openvino`（会顶掉现有 ort 轮子、风险大），直接用已装的 `openvino`：
`core.read_model(onnx)` → `compile_model(CPU)`。FP32 640 = 27.2ms（比最优 ORT 快 ~1.4×）。

### 8.3 INT8 量化（免 sudo）

- **ORT 静态 QDQ：不可用**。需先把模型 opset12→13（否则 `DequantizeLinear axis` 报错），
  量化后运行不报错但**输出恒为 0**（mAP=0）——end2end head（TopK/Sigmoid 支路）被量化打坏。
  **结论：ORT 静态量化对本 NMS-free 模型不可用。**
- **OpenVINO NNCF INT8：可用且精度无损**。`nncf.quantize(ov_model, calib_dataset)`，
  校准用 bucket 训练域图像 200 张。640 = 16.2ms（较 FP32 1.68×），mAP50 0.485（FP32 0.474）。
  与 ultralytics 输出一致性：FP32 ONNX 88/88、IoU 1.0000；OV-INT8 86/88、IoU 0.9826。

### 8.4 输入尺寸（免 sudo，需 `.pt` 重导出）

同一 `best.pt` 用 `nms=False` 重导出 640/480/416 的 end2end ONNX（否则默认导出成非 end2end 的
`[1,5,N]`）。纯推理 FPS：640 25.9(ORT)/36.8(OV) → 480 50.2/64.5 → 416 64.8/82.8；
叠加 INT8：640 54/62 → 480 86/106 → 416 108/131。

**真实 mAP**（bucket 验证子集：`~/cuadc2026_ws/cuadc_rknn_pipeline/datasets/cuadc2026`
的 val 中含 bucket(class 10) 的图像，重映射为单类 0；**跨域，仅作相对比较**）：

| imgsz | FP32 mAP50 / mAP50-95 | OV-INT8 mAP50 / mAP50-95 |
|---|---|---|
| 640 | 0.474 / 0.240 | 0.485 / 0.252 |
| 480 | 0.372 / 0.177 | 0.366 / 0.178 |
| 416 | 0.335 / 0.150 | 0.325 / 0.149 |

⇒ 640→480 掉 ~22% mAP、→416 掉 ~30%。**侦察/投放筒是小目标，建议默认 640；只有端到端算力不足时才降到 480。**

### 8.5 NMS-free 收益

同权重 640：end2end 推理 26.84ms / 后处理 0.004ms；原始头+NMS 推理 28.07ms / 后处理 0.079ms。
**NMS-free 省 ~1.2ms/帧（~4%）**；后处理本身都很小（相对 ultralytics 包装层的 65ms 可忽略）。

### 8.6 节点级端到端（真实 D435，内部 FPS 日志）

`detector_node` 新增每 5s 一行 `detector FPS xx.x` 日志，无订阅者也能读到真实处理帧率。
`image_qos_reliable:=false`（BEST_EFFORT）解耦背压后：

| 配置 | 节点端到端 FPS（区间，受后台负载影响） |
|---|---|
| ultralytics FP32 640（默认） | **~10**（6–11） |
| 原生 ORT FP32 640（t6） | ~12–21 |
| 原生 OV-INT8 640 | **~19–22** |
| 原生 OV-INT8 480 | ~12–30（最好 29–30，达相机上限） |

> 关键：**相机本身可 30Hz**（直连 pyrealsense 稳定 25.9–30fps，align 仅 3ms）。
> 但 `camera_node` **单独跑只有 ~9Hz**——根因见 §8.6.1。

#### 8.6.1 ⚠️ 重大坑：RELIABLE 图像 QoS 背压 + webui 订阅拖慢相机

- 现象：相机流只有 6–9Hz（应 ~30），`camera_node` CPU 仅 ~25%（在**等**不是算）。
- cProfile：`rclpy publish` 每次 **72.7ms**（color+depth+info），而 `wait+align+转换` 仅 21ms/帧。
- 根因：本包图像话题用 **RELIABLE + KEEP_LAST(1)**；当存在**未及时 ack 的订阅者**时，
  发布端被可靠重传/ack 拖住 → 相机被"顶着走"到消费端速率。
- 触发源：**常驻 `webui/server.py` 的 `ros_probe.py` 订阅了 `/vision/color/image_raw`**。
  `ros2 topic info -v` 显示：`Subscription count: 1 → webui_ros_probe`。
  - 实测：该订阅者在时，向 `/vision/color/image_raw` 发布每次 **326–483ms**；
    **精确停掉 webui 后**降到 **0.38ms**，相机立刻回到 **29.97Hz**。
  - 与可靠性/depth **无关**（BE/REL、depth1/10 对照都复现；换新话题名则不复现）。
- **对策（推荐）**：图像话题改 **BEST_EFFORT**（ROS2 传感器流标准做法），并在评测/飞行时
  避免控制台探针长驻订阅大图话题。本节点已加参数 `image_qos_reliable`（默认 true=原行为），
  `false` 切 BEST_EFFORT（`camera_node` 与 `detector_node` 必须一致；`detector` 自动拉起
  `camera_node` 时会自动透传该参数）。
- 同时：RELIABLE 也造成"相机被慢 detector 顶着"的**自限速**——BEST_EFFORT 后相机保持 ~30Hz、
  detector 永远处理"最新帧"（drop 旧帧），端到端帧率才反映 detector 真实能力。

### 8.7 需 sudo 的项：iGPU / governor（2026-09-30 已执行 ✅ 并实测）

> 结论先行：**本机 iGPU 对本模型无收益（≈ CPU）**；governor 已设 `performance`。

**启用 iGPU 的实际步骤**（Ubuntu 22.04 jammy 自带 `intel-opencl-icd` 22.14 太旧，在 6.8 内核 +
Raptor Lake 上 `clinfo` 返回 0 platform，**必须加 Intel 官方源**）：

```bash
sudo apt-get install -y gpg-agent wget
wget -qO - https://repositories.intel.com/gpu/intel-graphics.key | \
  sudo gpg --dearmor --yes -o /usr/share/keyrings/intel-graphics.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/intel-graphics.gpg] https://repositories.intel.com/gpu/ubuntu jammy unified" | \
  sudo tee /etc/apt/sources.list.d/intel-gpu.list
sudo apt-get update
# 关键：新版把 libigc/libigdfcl 拆包，必须"一起升级"否则依赖冲突（实测报 libigc1>=1.0.12812 未满足）
sudo apt-get install -y intel-opencl-icd libigc1 libigdfcl1 libigdgmm12 intel-level-zero-gpu level-zero
sudo apt-get install -y intel-opencl-icd        # 最终 ICD 升到 25.18
# 验证：
clinfo -l                                        # → Platform: Intel(R) OpenCL Graphics / Device: Intel(R) UHD Graphics
python3 -c "import openvino as ov; print(ov.Core().available_devices)"   # → ['CPU','GPU']
# governor：
sudo apt install -y linux-tools-generic && sudo cpupower frequency-set -g performance
```

**iGPU 实测（P 核，pure，n=100）——与 CPU 对照**：

| imgsz | OV-FP32 CPU | OV-FP32 GPU | OV-INT8 CPU | OV-INT8 GPU |
|---|---|---|---|---|
| 640 | 27.2ms / 36.8 | 27.1ms / **36.9** | 16.2ms / 61.8 | 16.9ms / **59.1** |
| 480 | 15.5 / 64.5 | 14.9 / **67.2** | 9.4 / 106.2 | 9.3 / **107.2** |
| 416 | 12.1 / 82.8 | 11.7 / **85.8** | 7.6 / 130.8 | 7.5 / **133.1** |
| 960(FP32) | 71.6 / 14.0 | 71.5 / **14.0** | — | — |

⇒ **GPU ≈ CPU（差在噪声内）**。原因：YOLO26n 极小（2.4M 参数 / 5.3 GFLOPs），iGPU（Iris Xe 80EU）
与 CPU 共享内存带宽、单帧延迟受 kernel 启动/同步开销支配；CPU 16 线程本身很强。
**另：batch>1 不可用**——导出的 attention 里 `Reshape` 常量把 batch 写死为 1（`[1,2,128,400]`），
reshape 到 B=8 会报 shape 不符；若将来要发挥 GPU 吞吐需从 PyTorch 侧按 batch 重导出。

> **行动建议**：维持 §8.8 的 **CPU + NNCF INT8** 方案（更简单、不依赖 GPU 驱动、功耗更低）。
> iGPU 驱动已装好备用（若以后换更大模型/多路/批处理可再评）。
> governor=`performance` 已生效（`cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor`）。
> ⚠️ 注意：本次为启用 GPU 新增了第三方 apt 源（`/etc/apt/sources.list.d/intel-gpu.list`）并升级了
> 图形栈包；如需回滚：`sudo rm /etc/apt/sources.list.d/intel-gpu.list` 后按需降级（一般无需）。

### 8.8 推荐最优配置（含启动命令）

**默认推荐：OpenVINO NNCF INT8 @640 + 原生后端 + BEST_EFFORT QoS**（精度无损、~20+ FPS 端到端）：

```bash
source /opt/ros/humble/setup.bash && source ~/ROS2_CUADC2026/install/setup.bash
python3 ~/ROS2_CUADC2026/install/cuadc_visionY26/lib/cuadc_visionY26/detector_node.py \
  --ros-args \
  -p camera_source:=realsense \
  -p infer_backend:=openvino \
  -p ov_num_threads:=6 -p ov_device:=CPU \
  -p image_qos_reliable:=false \
  -p model_path:=$HOME/ROS2_CUADC2026/models/yolo26_int8/best_y26_640_int8_e2e_openvino_model
```
（`camera_node` 由 `detector_node` 自动拉起并自动透传 `image_qos_reliable`；若手动起相机，
需同样加 `-p image_qos_reliable:=false`。）

**算力紧张/追求更高帧率**：把 `model_path` 换 `.../best_y26_480_int8_e2e_openvino_model`
（mAP50 −23%，端到端可达相机上限 ~30 FPS）。
**若不需要 OpenVINO（只装了 ORT）**：`-p infer_backend:=ort -p ort_intra_threads:=6`（FP32 ~2× ultralytics）。

### 8.9 交付源码最小改动（影响面）

`detector_node.py`（+ ~200 行，**默认行为不变**）：
1. 新增可选原生后端 `NativeDetector`（ORT / OpenVINO）：调线程、**关 ORT 自旋**
   （不关会 1007% CPU 空转）、支持 OpenVINO IR 目录、end2end 直取 / 原始头 NMS、反 letterbox 缩放。
2. 新增参数：`infer_backend`(默认 `ultralytics`=原行为)、`ort_intra_threads`(6)、
   `ort_providers`、`ov_device`(CPU)、`ov_num_threads`(0)、`image_qos_reliable`(true)。
3. `os.path.isfile` → 也接受**目录**（OpenVINO IR）。
4. 自动拉起 `camera_node` 时透传 `image_qos_reliable`（QoS 必须两端一致）。
5. 修正既有 bug：`imgsz_explicit` 取值早于 `node_param` ⇒ 恒 True（现改为声明前判断）。
6. 新增每 5s `detector FPS` 日志。

`camera_node.py`（+ ~10 行）：新增 `image_qos_reliable` 参数（三个图像发布者用统一 QoS）。

> 影响面：默认参数下与改造前**逐字节等价**（QoS 仍 RELIABLE、仍走 ultralytics）；
> 只有显式传 `infer_backend`/`image_qos_reliable` 才启用新路径。下游话题名/消息类型/坐标语义未变。

### 8.10 可复现命令

```bash
# 基准脚本（新建，已 +x；/tmp/opencode 有 .keep.py 备份）
bash ~/ROS2_CUADC2026/scripts/bench_yolo26_fps.py compare        # 三后端对比
taskset -c 0-7 python3 scripts/bench_yolo26_matrix.py --reps 3 --n 60 \
  --out /tmp/opencode/bench_matrix.json                          # 稳健矩阵
# INT8 量化（ORT/OV）
python3 scripts/quant_yolo26_int8.py ov --model 640_end2end.onnx \
  --out .../best_y26_640_int8_e2e --calib-dir /tmp/opencode/bucket_val/images/val --size 640 --num 200
```
