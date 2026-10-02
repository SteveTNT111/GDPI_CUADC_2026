# NX（Orin NX / ARM64）恢复部署说明

> 本文件由 x86 开发机（NUC13 / Ubuntu 22.04 / ROS2 Humble）生成，交给 **Orin NX 上的 agent 或操作手**执行。
> 目标：把本目录（ROS2 工作区**源码快照**）恢复到 NX 的 `$HOME/ROS2_CUADC2026`，重建编译，并完成冒烟验收。
> 生成时间：2026-10-03。**本快照不含模型权重、不含 `build/`、`install/`、`log/`（架构不同，必须重建）。**

---

## 0. 先读这三条铁律

1. **架构不同，禁止照搬编译产物**：x86 上的 `build/ install/` 里全是 `*.so (x86_64)`，NX 是 **aarch64**，必须在 NX 上 `colcon build` 重建。
2. **权重不在仓库里**：识别/推理用的 `.pt/.onnx/OpenVINO IR` 已按仓库策略排除，需**单独拷贝**（见 §4）。
3. **一切以仓库根目录《错题本.md》为准**：尤其 PRE-FLIGHT 清单、#26（参数写法）、#16c（链路只认话题流实据）、#40/#41（高度参考系/容差）、#43（舵机 AUX）、#45/#46（图像 QoS / 推理后端）。

---

## 1. 目录与路径约定

| 项 | 值 |
|---|---|
| 本快照在仓库中的位置 | `<repo>/代码/ROS2_CUADC2026/` |
| **运行时必须落到的位置** | `$HOME/ROS2_CUADC2026`（脚本里写死了这个路径） |
| 参考目标机 | Orin NX，JetPack 6.x = Ubuntu 22.04，aarch64 |
| ROS2 发行版 | Humble（arm64 官方源） |

⚠️ 不要直接 `cd <repo>/代码/ROS2_CUADC2026` 就跑——路径不对，必须按 §2 拷到 `$HOME/ROS2_CUADC2026`。

> 若 NX 的登录用户不是 `lab`：本快照脚本绝大多数用 `$HOME/ROS2_CUADC2026`，**不受影响**；
> 但仿真相关脚本里写死了 `/home/lab/SITL_Models`、`/home/lab/ardupilot_gazebo`、`/home/lab/ardupilot-7.1` 等，见 §5.2。
> 真机 MAVROS 启动器 `scripts/launchers/run_real_fc_mavros.sh` 里网口名 `enp86s0` 也要按 NX 实际改（见 §5.1）。

---

## 2. 恢复工作区到主目录（第一步）

```bash
# 1) clone / pull 仓库（若 NX 上还没有）
git clone https://github.com/SteveTNT111/GDPI_CUADC_2026.git ~/GDPI_CUADC_2026
# 已有则： cd ~/GDPI_CUADC_2026 && git pull

# 2) 一键恢复到 $HOME/ROS2_CUADC2026（幂等；不会覆盖你单独放进去的权重）
bash ~/GDPI_CUADC_2026/代码/ROS2_CUADC2026/scripts/nx_restore.sh
```

`nx_restore.sh` 做四件事：rsync 拷贝源码到 `$HOME/ROS2_CUADC2026` → 给脚本补执行位 → 检查/提示缺失依赖与待改路径 → 打印后续步骤。
它**不联网、不装依赖、不 build**（避免半路失败），依赖与构建按 §3、§6 手动做。

> 之后的更新流程：`git pull` → 再跑一次 `nx_restore.sh`（rsync 是增量，权重不会被删）。

---

## 3. 依赖安装（APT + PIP）

### 3.1 ROS2 Humble + 飞控 + 相机（apt）

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-base ros-humble-rclpy \
  ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs \
  ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-geographic-msgs ros-humble-tf2-ros ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rosidl-default-generators ros-humble-rosidl-default-runtime \
  python3-colcon-common-extensions python3-numpy python3-opencv

# MAVROS 需要 GeographicLib 数据集
ros2 run mavros install_geographiclib_datasets.sh || \
  sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

- **MAVROS arm64**：Humble 官方源一般有 arm64 二进制；若 `apt` 找不到 `ros-humble-mavros*`，需按 mavros 官方文档源码编译（`raspberrypi`/`source build`）。
- **RealSense**：
  - 首选 `sudo apt install -y ros-humble-realsense2-camera ros-humble-librealsense2`；
  - 若 `camera_node.py` 走 `pyrealsense2`，还需装 Python 绑定：Intel 官方 arm64 包或源码编译 librealsense（见 §3.2）。
- 舵机/图像若要用 `rqt_image_view`：`sudo apt install -y ros-humble-rqt-image-view`。

### 3.2 Python（pip；注意 `numpy<2` 必须钉住）

x86 的 CPU 轮子（`torch +cpu`）在 arm64 **装不上**，必须换。

```bash
# 钉 numpy<2（否则遮蔽 ROS Humble 的 numpy ABI，cv_bridge/python3-opencv 会报 _ARRAY_API not found）
printf 'numpy<2\n' > /tmp/constraints.txt

# --- 路线 A（推荐，跑原生后端，最省事）：ONNX Runtime + OpenVINO，可完全不装 torch/ultralytics
pip3 install --user -U --constraint /tmp/constraints.txt onnxruntime openvino
# Orin 有 GPU 时建议：用 JetPack 对应版本 onnxruntime-gpu（含 CUDA/TensorRT EP），见 §7

# --- 路线 B：继续用 ultralytics 包装层（需要 torch）
#     Orin 用 NVIDIA JetPack 对应轮子，不要用 PyPI 的 x86 CPU 轮：
#     https://developer.download.nvidia.com/compute/redist/jp/  （按 JetPack 版本选 v6.x）
pip3 install --user -U --constraint /tmp/constraints.txt ultralytics onnxruntime

# 相机 Python 绑定（若需要源码编译 librealsense）：
#   参考 https://github.com/IntelRealSense/librealsense  build -> pip3 install . 生成 pyrealsense2

# 仿真/日志辅助（可选）
pip3 install --user pymavlink MAVProxy

# 兼容性钉版（若 ultralytics 自动升级了依赖，按 x86 文档同款修复）
pip3 install --user "protobuf==3.20.3" "onnx==1.16.2" "ml-dtypes<0.6" -U packaging
```

> 若只走路线 A（`infer_backend:=ort` 或 `openvino`），`detector_node.py` 不会 import ultralytics/torch，**可以不装 torch**，NX 上更轻。
> 需要 `geographiclib` 精确大地解算时：`pip3 install --user geographiclib`（不装则 detector 走本地近似回退，见错题本 #48）。

---

## 4. 模型权重放置（**仓库里没有，必须单独拷**）

把权重按下面路径放到 `$HOME/ROS2_CUADC2026`（拷贝方式：U 盘 / 私有网盘 / scp，**不要进公开仓库**）：

| 用途 | 目标路径 |
|---|---|
| Y26 主权重 ONNX | `src/cuadc_visionY26/models/best_yolo26.onnx` |
| Y26 训练权重（重导出用） | `src/cuadc_visionY26/models/best.pt` |
| 旧 YOLOv8 权重 | `src/cuadc_vision/models/best.onnx` |
| OpenVINO INT8 IR（原生后端用，整个目录） | `models/yolo26_int8/best_y26_640_int8_e2e_openvino_model/{best.xml,best.bin,metadata.yaml}`（416/480 同结构） |

⚠️ `cuadc_vision/CMakeLists.txt` 与 `cuadc_visionY26/CMakeLists.txt` 会把 `models/` 装到 `install/.../lib/models/`。
所以**权重要在 `colcon build` 之前放好**；若 build 之后才放，需重跑一次 `colcon build`（或直接同步到 `install/*/lib/models/`）。

---

## 5. 配置修改（真机 vs 仿真）

单一事实源：`~/ROS2_CUADC2026/config/auto_drop.yaml`（WebUI 也读写它）。

### 5.1 真机（默认）

| 位置 | 键/变量 | NX 上要改成 |
|---|---|---|
| `config/auto_drop.yaml` | `launch.mavros_fcu_url` | 保持 `udp://:14555@192.168.13.14:14550`，**前提是飞控 IP/端口不变** |
| `config/auto_drop.yaml` | `launch.camera_source` | `realsense`（真机；仿真改 `gazebo`） |
| `config/auto_drop.yaml` | `launch.detector_package` | `cuadc_visionY26`（默认） |
| `scripts/launchers/run_real_fc_mavros.sh` | `enp86s0`、nmcli `cuav-v6x`、`192.168.13.100/24` | 改为 NX 实际网口名（`ip -br link` 查），并用 nmcli 重建同网段连接 |
| `config/webui.token` | — | **不需要拷贝**；`server.py` 首次运行会自动生成（权限 600） |

### 5.2 仿真（仅当要在 NX 上跑 Gazebo；纯真机可跳过）

4 个脚本里写死了 x86 开发机的仿真路径，需按 NX 实际改：
`scripts/demo_up.sh`、`scripts/env_guard.sh`、`scripts/start_gz.sh`、`scripts/start_sitl.sh`

```bash
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$HOME/ROS2_CUADC2026/models:<NX上的 SITL_Models/Gazebo>:<NX上的 ardupilot_gazebo/models>"
export GZ_SIM_SYSTEM_PLUGIN_PATH="<NX上的 ardupilot_gazebo/build>"
```
以及 `start_sitl.sh` 里的 `ARDSRC=<NX上的 ardupilot 源码目录>`。
这些**外部仿真资源不在本仓库内**，要么在 NX 上重建（见 §8），要么把 x86 上的目录整体拷到 NX 再改路径。

---

## 6. 构建

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/ROS2_CUADC2026"
colcon build --packages-select cuadc_vision cuadc_visionY26 cuadc_control
# 顺序会自动按依赖解析：cuadc_vision(消息) → cuadc_visionY26 / cuadc_control
source "$HOME/ROS2_CUADC2026/install/setup.bash"
```

> 首次可能因 `packaging` 版本报 `canonicalize_version() ... strip_trailing_zero`：`pip3 install --user -U packaging`（错题本/detector 文档 §1.3）。

---

## 7. 冒烟验收（逐条给实据）

```bash
source /opt/ros/humble/setup.bash && source "$HOME/ROS2_CUADC2026/install/setup.bash"

# 1) 三个包 + 消息类型
ros2 pkg list | grep cuadc
python3 -c "import cuadc_vision.msg; print('msgs OK')"

# 2) 控制脚本在
ls "$HOME/ROS2_CUADC2026/install/cuadc_control/lib/cuadc_control/" | grep auto_drop_test.py

# 3) 推理依赖（按你选的路线）
python3 -c "import onnxruntime as o; print('ORT providers:', o.get_available_providers())"
python3 -c "import openvino as ov; print('OV devices:', ov.Core().available_devices)"   # 走 OpenVINO 时
# Orin GPU 路线预期能看到 TensorrtExecutionProvider / CUDAExecutionProvider

# 4) 相机（先只插 D435，别起 ROS）
rs-enumerate-devices -s        # 或 python3 -c "import pyrealsense2 as rs; print(rs.__version__)"

# 5) 相机节点（真机）
ros2 run cuadc_visionY26 camera_node.py --ros-args -p source:=realsense

# 6) 识别节点（无窗口，原生 ORT 路线示例）
ros2 run cuadc_visionY26 detector_node.py --ros-args \
  -p camera_source:=realsense -p auto_start_mavros:=false -p show_window:=false \
  -p infer_backend:=ort -p ort_intra_threads:=4 \
  -p model_path:=$HOME/ROS2_CUADC2026/src/cuadc_visionY26/models/best_yolo26.onnx
# 链路真伪只看话题流： ros2 topic hz /vision/color/image_raw   /vision/yolo/detections

# 7) WebUI
bash "$HOME/ROS2_CUADC2026/scripts/webui_up.sh"    # 浏览器打开提示的 URL
```

> 真机 MAVROS 连通性验收按 `scripts/launchers/run_real_fc_mavros.sh`，**不要以 `/mavros/state connected:true` 单独作数**（transient_local 假象，错题本 #16c），要看 `/mavros/local_position/pose` 等话题率。

---

## 8. 在 NX 上启用仿真（可选；纯真机飞行可跳过）

需要 arm64 重建三样（都不能从 x86 拷二进制）：

1. **Gazebo Harmonic（arm64）**：`sudo apt install -y gz-harmonic`（提供 `gz sim` 8.x + `gz.transport13`）。
2. **ardupilot_gazebo 插件（arm64）**：源码 clone → `mkdir build && cd build && cmake .. && make -j` → 记下 `build/` 路径填 `GZ_SIM_SYSTEM_PLUGIN_PATH`。
3. **ArduPilot SITL（arm64）**：clone ArduPilot 4.7.x → `./waf configure --board sitl && ./waf copter`，得到 `build/sitl/bin/arducopter`，填 `start_sitl.sh` 的 `ARDSRC`。
   - `SITL_Models/Gazebo` 资源可整体拷贝（纯资产，跨架构安全）。

仿真环境变量（每次开终端或写进 `~/.bashrc`）：
```bash
export GZ_VERSION=harmonic
export GZ_SIM_RESOURCE_PATH="$HOME/ROS2_CUADC2026/models:$HOME/SITL_Models/Gazebo:$HOME/ardupilot_gazebo/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/ardupilot_gazebo/build"
```
然后 `bash scripts/demo_up.sh`（幂等：缺啥补啥）。注意本快照脚本默认路径是 `/home/lab/...`，务必按 §5.2 改。

---

## 9. NX 移植常见坑（务必对照）

| 坑 | 说明 |
|---|---|
| x86 `install/` 被误用 | 本快照已排除；若你从别处拷了整个工作区，务必删掉 `build/ install/ log/` 再 build |
| `numpy>=2` | 会打断 ROS Humble 的 cv_bridge/opencv，必须 `numpy<2` |
| 图像 QoS RELIABLE 背压 | 相机被慢订阅者/探针顶到 ~8Hz；评测/飞行时用 `-p image_qos_reliable:=false`（错题本 #45） |
| ORT 默认线程=物理核 | 在 arm64 上也不一定最优；用 `-p ort_intra_threads:=4`（错题本 #46） |
| ONNX 静态 INT8 | 别用 ORT 静态量化打坏 end2end 头；用 OpenVINO NNCF（错题本 #46） |
| 舵机不动 | 飞控 AUX 输出默认 Disabled；A/B=AUX1/2=SERVO9/10（错题本 #43）——这是 FCU 参数，NX 只下发 |
| `geographiclib` 未装 | detector 静默走本地近似回退（错题本 #48）；需要精确解算再装 |
| 参数写法 | 直跑节点必须 `--ros-args -p k:=v`，裸 `-p`/`k:=v` 会被静默当 remap（错题本 #26） |
| ROS 环境顺序 | 先 `/opt/ros/humble/setup.bash` 再工作区 `install/setup.bash`，顺序不能反 |

---

## 10. 交付检查清单（NX agent 收尾）

- [ ] `$HOME/ROS2_CUADC2026` 已生成，`src/{cuadc_control,cuadc_vision,cuadc_visionY26}` 齐全
- [ ] 权重已按 §4 放好且路径正确
- [ ] `colcon build` 三包成功
- [ ] `import cuadc_vision.msg` 通过
- [ ] RealSense 能出图（`/vision/color/image_raw` 有率）
- [ ] 识别节点能出 `/vision/yolo/detections`
- [ ] 真机 MAVROS 能连上并看到 `/mavros/local_position/pose` 有流
- [ ] `config/auto_drop.yaml` 的 `mavros_fcu_url` / `camera_source` 已按现场确认
- [ ] （若跑仿真）§5.2 路径与 §8 三件套已就绪
- [ ] 任何"看起来成功"的链路判定都有 `ros2 topic hz` 实据
