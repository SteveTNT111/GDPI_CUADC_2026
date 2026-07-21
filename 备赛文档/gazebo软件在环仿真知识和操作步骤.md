# ArduPilot Copter 4.5 SITL 与 Gazebo 环境记录

记录日期：2026-06-25

## 目录

- [[#1. 当前已完成的工作|1. 当前已完成的工作]]
- [[#2. Gazebo 当前版本判断|2. Gazebo 当前版本判断]]
- [[#3. 真实飞控固件与 SITL 虚拟飞控的区别|3. 真实飞控固件与 SITL 虚拟飞控的区别]]
- [[#4. SITL 中传感器数据由什么产生|4. SITL 中传感器数据由什么产生]]
- [[#5. SITL 虚拟飞控是否需要编译|5. SITL 虚拟飞控是否需要编译]]
- [[#6. MAP 和 console 有什么用|6. MAP 和 console 有什么用]]
- [[#7. 接下来 Gazebo 插件怎么装|7. 接下来 Gazebo 插件怎么装]]
- [[#8. 后续建议|8. 后续建议]]
- [[#9. 参考资料|9. 参考资料]]
- [[#10. 日常操作手册|10. 日常操作手册]]
- [[#11. MAVProxy console 和 map 是否能当地面站|11. MAVProxy console 和 map 是否能当地面站]]
- [[#12. GPS 数据链路总览|12. GPS 数据链路总览]]
- [[#13. MAVProxy console 刷屏问题排查|13. MAVProxy console 刷屏问题排查]]
- [[#14. MAVROS 验证命令|14. MAVROS 验证命令]]
- [[#15. 多窗口分工说明|15. 多窗口分工说明]]
- [[#16. 比赛场地世界 + D435i 相机 + GPS 流脚本|16. 比赛场地世界 + D435i 相机 + GPS 流脚本]]
- [[#17. 2026-06-26 踩坑全记录|17. 2026-06-26 踩坑全记录]]

---

## 1. 当前已完成的工作

本次已经在 Ubuntu 20.04 环境中完成并验证了 ArduPilot Copter 4.5 的基础 SITL 编译与运行。

已确认的信息：

- ArduPilot 源码目录：`~/ardupilot`
- 当前分支：`Copter-4.5`
- 当前提交：`b720755`
- 子模块状态：已经初始化，前 80 行未见 `-` 或 `+` 开头的异常状态
- Python 版本：`Python 3.8.10`
- MAVProxy 已安装：`/home/lab/.local/bin/mavproxy.py`
- Gazebo 已安装版本：`Gazebo Classic 11.15.1`

已经解决的问题：

- ArduPilot 源码中混入 Windows CRLF 换行，导致 bash 报错：
  - `$'\r': 未找到命令`
  - `/usr/bin/env: "python3\r": 没有那个文件或目录`
- `waf` 和 `waf-light` 缺少执行权限，导致：
  - `bash: ./waf: 权限不够`
  - `Permission denied: '/home/lab/ardupilot/modules/waf/waf-light'`

已经执行过的关键修复命令：

```bash
sudo apt update
sudo apt install -y dos2unix

cd ~/ardupilot
dos2unix waf
dos2unix Tools/completion/completion.bash
find ~/ardupilot/Tools/completion/bash -type f -print0 | xargs -0 dos2unix
dos2unix ~/ardupilot/Tools/autotest/sim_vehicle.py
dos2unix ~/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh

chmod +x waf
chmod +x Tools/autotest/sim_vehicle.py
chmod +x Tools/autotest/autotest.py
chmod +x Tools/environment_install/install-prereqs-ubuntu.sh
chmod +x modules/waf/waf-light
```

已经编译成功：

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

成功结果：

```text
'copter' finished successfully
```

说明 `build/sitl/bin/arducopter` 已经成功生成，ArduPilot Copter 4.5 的 SITL 核心环境可用。

已经运行成功：

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

运行后 `MAP` 和 `console` 都已经出现，说明纯 ArduPilot SITL 已经能启动。

## 2. Gazebo 当前版本判断

执行过：

```bash
gazebo --version
```

输出：

```text
Gazebo multi-robot simulator, version 11.15.1
```

结论：

当前安装的是 **Gazebo Classic 11.15.1**。

注意：

- `Copyright (C) 2012` 不是当前软件版本年份，而是版权声明起始信息。
- Ubuntu 20.04 配套 Gazebo Classic 11 是正常组合。
- 现在不应直接照搬新版 `gz sim` / Garden / Harmonic 教程。
- 当前命令体系是 `gazebo`，不是新版 Gazebo Sim 的 `gz sim`。

## 3. 真实飞控固件与 SITL 虚拟飞控的区别

真实飞机上的飞控固件通常运行在 STM32 等 MCU 上，例如 Pixhawk、Cube、Mateksys 等硬件。它需要面对真实硬件资源：

- PWM / DShot 输出
- UART 串口
- I2C
- SPI
- CAN
- ADC
- GPIO
- IMU、气压计、磁罗盘、GPS、空速计等真实传感器

因此真实飞控需要针对不同板型定义硬件引脚和外设映射。ArduPilot 中这类定义通常体现在具体板型的 `hwdef.dat`、`hwdef-bl.dat` 以及 HAL 层相关代码中。

SITL 不运行在 MCU 上，而是运行在 Linux 普通进程中。它不直接操作真实 GPIO、SPI、I2C、PWM 引脚，也不需要针对飞控板重新定义物理引脚。

SITL 编译出来的是一个 Linux 可执行程序，例如：

```text
~/ardupilot/build/sitl/bin/arducopter
```

可以把它理解成“运行在电脑上的 ArduCopter 飞控大脑”。

## 4. SITL 中传感器数据由什么产生

真实飞控的传感器数据来自真实 IMU、GPS、气压计、磁罗盘等硬件。

SITL 中没有真实传感器，传感器数据由仿真系统产生。

纯 SITL 场景中，ArduPilot 内部的 SITL 仿真模型会生成飞机的位置、速度、姿态、高度等状态，并进一步合成虚拟传感器数据，例如：

- 虚拟 IMU
- 虚拟 GPS
- 虚拟气压计
- 虚拟磁罗盘
- 虚拟电池
- 虚拟遥控输入

这些数据再喂给 ArduPilot 的导航、姿态控制、位置控制、EKF 等模块。高层控制逻辑和真实飞控固件非常接近，但底层硬件访问被 SITL/HAL 替换成了仿真接口。

接入 Gazebo 后，传感器和物理世界主要由 Gazebo 负责：

- Gazebo 负责物理引擎、重力、碰撞、模型姿态、位置、速度。
- Gazebo 插件负责把模型状态和传感器信息传给 ArduPilot SITL。
- ArduPilot SITL 计算电机输出。
- Gazebo 插件再把电机输出作用到 Gazebo 中的无人机模型上。

因此，Gazebo 模式中可以粗略理解为：

```text
Gazebo 虚拟世界 -> Gazebo 插件 -> ArduPilot SITL -> 电机输出 -> Gazebo 虚拟世界
```

## 5. SITL 虚拟飞控是否需要编译

需要。

SITL 虚拟飞控本质上仍然是 ArduPilot，只是编译目标不是具体飞控板，而是 Linux 上的 `sitl` 板型。

编译命令是：

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf copter
```

真实硬件固件常见编译方式类似：

```bash
./waf configure --board CubeOrange
./waf copter
```

两者区别主要在 `--board`。

对比：

| 项目 | 真实飞控固件 | SITL 虚拟飞控 |
|---|---|---|
| 运行位置 | STM32 等 MCU | Linux 电脑进程 |
| 编译目标 | 具体硬件板型 | `sitl` |
| 底层系统 | ChibiOS / NuttX 等 | Linux / POSIX |
| 传感器来源 | 真实硬件 | ArduPilot 内置仿真或 Gazebo |
| 引脚定义 | 需要 | 一般不需要 |
| 电机输出 | PWM / DShot 等真实输出 | 发送给仿真模型 |
| MAVLink | 支持 | 支持 |
| 控制算法 | 基本一致 | 基本一致 |

## 6. MAP 和 console 有什么用

`sim_vehicle.py -v ArduCopter --console --map` 启动后出现的两个窗口来自 MAVProxy。

### console

`console` 是 MAVProxy 控制台和状态窗口，用途包括：

- 查看飞控模式
- 查看解锁状态
- 查看高度、姿态、GPS、EKF 等状态
- 输入飞控命令
- 改参数
- 加载任务
- 模拟起飞、降落、切模式

常用命令：

```text
mode guided
arm throttle
takeoff 5
mode loiter
mode land
param show ARMING_CHECK
param set ARMING_CHECK 0
```

### map

`map` 是 MAVProxy 的地图窗口，用途包括：

- 显示飞机当前位置
- 显示航迹
- 显示航点任务
- 观察飞机是否按预期移动
- 检查 GPS 坐标和任务路线

纯 SITL 中，即使没有 Gazebo，也可以通过 `map` 观察虚拟飞机在地图上的运动。

## 7. 接下来 Gazebo 插件怎么装

当前机器是 Ubuntu 20.04 + Gazebo Classic 11.15.1，所以优先路线是 **Gazebo Classic 插件路线**。

需要注意：ArduPilot 官方当前主推的是新版 Gazebo Sim 插件 `ArduPilot/ardupilot_gazebo`，适合 Garden / Harmonic / Ionic 等新版 Gazebo；而 Gazebo Classic 属于 legacy 路线。ArduPilot 文档中也明确把 Gazebo11 及更早版本归为 legacy。

### 7.1 路线 A：继续使用当前 Gazebo Classic 11

安装 Gazebo Classic 开发包和编译工具：

```bash
sudo apt update
sudo apt install -y git cmake build-essential pkg-config libgazebo11-dev
```

克隆 legacy 插件：

```bash
cd ~
git clone https://github.com/khancyr/ardupilot_gazebo.git
cd ~/ardupilot_gazebo
mkdir -p build
cd build
cmake ..
make -j4
sudo make install
```

配置 Gazebo 环境变量：

```bash
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=$HOME/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

运行方式通常是两个终端。

终端 1：启动 Gazebo 世界：

```bash
cd ~/ardupilot_gazebo
gazebo --verbose worlds/iris_arducopter_runway.world
```

终端 2：启动 ArduPilot SITL：

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

然后在 MAVProxy 中测试：

```text
mode guided
arm throttle
takeoff 5
```

风险提示：

- `khancyr/ardupilot_gazebo` 仓库已经归档，不再维护。
- ArduPilot legacy 文档较旧，部分说明以 Gazebo 7-9 为主，但该插件 README 提到开发分支可用于 Gazebo 8 及以上。
- 如果在 Gazebo 11 上编译或运行失败，优先记录报错，再决定是修 legacy 插件，还是迁移到新版 Gazebo Sim。

### 7.2 路线 B：未来推荐路线，迁移到新版 Gazebo Sim

如果后续项目周期允许，更推荐：

- Ubuntu 22.04
- Gazebo Harmonic
- ArduPilot 官方 `ArduPilot/ardupilot_gazebo`
- `sim_vehicle.py --model JSON`

新版官方插件支持 JSON 数据交换、更多传感器、更好的 lockstep 调试和新版渲染后端。典型命令是：

```bash
gz sim -v4 -r iris_runway.sdf
```

另一个终端：

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

但是这不是当前机器已经安装好的 Gazebo Classic 11 路线。不要把两套路由混用。

## 8. 后续建议

短期目标：

1. 保持当前 Ubuntu 20.04 + Gazebo Classic 11 环境不大改。
2. 先把纯 SITL 的 `guided -> arm -> takeoff -> land` 流程跑熟。
3. 再安装 Gazebo Classic 插件并跑通 `gazebo-iris`。
4. 每次成功运行后记录命令、报错和截图，便于比赛环境复现。

中期目标：

1. 建立一份固定的 `setup.sh`，用于自动修复 CRLF、补执行权限、设置 PATH。
2. 记录 ArduPilot 参数文件，例如 `copter.parm`。
3. 学会用 MAVProxy 或 Mission Planner 加载航点任务。
4. 明确竞赛用的机型、传感器需求、任务流程，再决定是否需要 ROS/MAVROS。

长期建议：

1. 如果只是验证飞控逻辑，纯 SITL 足够快。
2. 如果要验证视觉、避障、室内场景、多机协同，需要 Gazebo。
3. 如果要做真实上机，必须回到具体飞控板型的硬件定义、传感器驱动、端口映射、电源与安全检查。
4. 仿真和真机不要混为一谈：SITL 验证控制逻辑，HITL/真机验证硬件链路。

## 9. 参考资料

- ArduPilot Gazebo legacy 文档：<https://ardupilot.org/dev/docs/sitl-with-gazebo-legacy.html>
- ArduPilot 新版 Gazebo 文档：<https://ardupilot.org/dev/docs/sitl-with-gazebo.html>
- 新版官方插件：<https://github.com/ArduPilot/ardwoxianupilot_gazebo>
- Gazebo Classic legacy 插件：<https://github.com/khancyr/ardupilot_gazebo>

## 10. 日常操作手册

> 环境：Ubuntu 20.04 + ArduPilot Copter-4.5 + Gazebo Classic 11.15.1
> 世界文件：[[cuadc_2026_field.world]]
> 圆筒生成：[[generate_hollow_cylinders.py]]

**首次使用（仅一次）**：

```bash
# 将 vault 中的 generate_hollow_cylinders.py 拷贝到 ~/
python3 ~/generate_hollow_cylinders.py
```

---

### 10.0 纯飞控测试（最快，不需要 Gazebo）

只测起降、切模式、航点，一条命令：

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
```

进去直接 `mode guided` → `arm throttle` → `takeoff 5`。map 上能看到飞机动，不依赖 Gazebo。

> 注意不要加 `-f gazebo-iris`，那是连 Gazebo 的。

### 10.1 终端 1：清残留 + roscore + Gazebo 后端

```bash
killall -9 gzserver gzclient gzmaster gazebo 2>/dev/null
pkill -9 -f mavproxy; pkill -9 -f arducopter
ss -ulnp | grep -E "9002|9003" | awk '{print $NF}' | grep -oP 'pid=\K[0-9]+' | xargs -r kill -9
sleep 2

source /opt/ros/noetic/setup.bash
roscore &
sleep 1

cd ~/ardupilot_gazebo
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
gzserver --verbose -s libgazebo_ros_api_plugin.so worlds/cuadc_2026_field.world
```

### 10.2 终端 2：Gazebo 3D 画面

```bash
cd ~/ardupilot_gazebo
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
gzclient --verbose
```

### 10.3 终端 3：SITL 驾驶舱

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

### 10.4 起飞与降落

出现 `STABILIZE>` 后，按顺序输入：

**1. 切模式**

```
mode guided
```

**2. 解锁**

```
arm throttle
```

**3. 起飞**

```
takeoff 5
```

**4. 悬停**

```
mode loiter
```

**5. 降落**

```
mode land
```

**6. 加锁**

```
disarm
```

### 10.5 查看 D435i 相机画面

**确认相机状态**：

```bash
source /opt/ros/noetic/setup.bash

# 相机节点（只有一个 /gazebo，所有传感器都在里面）
rosnode list | grep -iE "camera|d435|gazebo"

# 全部相机话题
rostopic list | grep -E "color|depth"
```

话题速查：

| 话题 | 内容 | 算法用哪个 |
|---|---|---|
| `/color/color/image_raw` | 640×480 彩色图 | ✅ YOLO 检测 |
| `/depth/depth/image_raw` | 640×480 深度图（L16 灰度） | ✅ 深度估计 |
| `/depth/depth/camera_info` | 深度相机内参 | ✅ 像素→3D 投影 |
| `/depth/points` | **自动生成的点云** | ✅ PCL 处理 |
| `.../compressed` / `.../theora` | 压缩传输版 | ❌ 本地调试不看 |

**查看节点图**：

```bash
rqt_graph
```

**rqt_image_view（最简，推荐）**：

```bash
rqt_image_view
```

弹窗后顶部下拉框选话题：
- `/color/color/image_raw` → 看到彩色画面
- `/depth/depth/image_raw` → 看到深度图（远处亮/近处暗）

可以开两个窗口同时看彩色和深度。

**RViz（功能最全，跟真机调试一样）**：

```bash
rosrun rviz rviz
```

进去后按照真实调试流程配置：

1. **固定坐标系**：左侧 `Global Options` → `Fixed Frame` 填 `base_link`
2. **加彩色图**：左下 `Add` → 选 `Image` → OK → 展开 `Image` → `Image Topic` 选 `/color/color/image_raw`
3. **加深度图**：再 `Add` → 选 `Image` → OK → 同样在 `Image Topic` 选 `/depth/depth/image_raw`
4. **加 TF**（看相机位姿）：`Add` → 选 `TF` → OK，就能看到 base_link 下面的相机坐标系
5. **加点云**（可选，不一定有）：`Add` → 选 `PointCloud2` → Topic 选 `/depth/depth/points`

效果：RViz 里左边彩色图，右边深度图，中间 3D 视图能看到相机相对机身的位置，跟真机联调时一模一样。

**终端看 fps**：

```bash
rostopic hz /color/color/image_raw   # 彩色帧率
rostopic hz /depth/depth/image_raw   # 深度帧率
```

### 10.6 GUIDED 航点飞行

**命令行手动指定坐标**：

```
mode guided
arm throttle
takeoff 5
guided -35.363262 149.165237 10   # 飞向指定经纬度
mode loiter
mode land
```

**Map 窗口点哪飞哪**：`mode guided` → `arm throttle` → `takeoff 5` → 在地图窗口**右键 → Fly to**

### 10.7 Gazebo 3D 快捷操作

| 问题 | 操作 |
|---|---|
| 飞机不见了 | 左侧面板右键 iris → **Move To**；或 `Ctrl+R` |
| 旋转/平移/缩放 | 左键拖拽 / 中键拖拽 / 滚轮 |
| 跟踪飞机 | 左侧面板右键 iris → **Follow** |

### 10.8 [存档] Iris 原版跑道世界

<details>
<summary>点击展开（不常用）</summary>

终端 1：

```bash
cd ~/ardupilot_gazebo
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
gzserver --verbose worlds/iris_arducopter_runway.world
```

终端 2：

```bash
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
gzclient --verbose
```

终端 3：

```bash
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

</details>

## 11. MAVProxy console 和 map 是否能当地面站

可以，至少在 SITL 阶段完全够用。

`console` 和 `map` 来自 MAVProxy，可以充当轻量地面站：

- `console`：查看飞控状态、切模式、解锁、起飞、降落、改参数。
- `map`：查看飞机位置、轨迹、航点和 GPS 移动情况。

它们不能完全替代 Mission Planner 或 QGroundControl 的所有图形化功能，但对 SITL 调试已经够用。

常用 MAVProxy 命令：

```text
mode guided
arm throttle
takeoff 5
mode loiter
mode land
disarm
```

查看模式：

```text
mode
```

查看参数：

```text
param show ARMING_CHECK
param show FRAME_CLASS
```

设置参数示例：

```text
param set ARMING_CHECK 0
```

如果解锁失败，优先看 `console` 里的报错，不要一上来就关安全检查。

## 12. GPS 数据链路总览

在当前 SITL + Gazebo 环境里，GPS 坐标**不是**来自真实 GPS 接收机，也不是来自 Gazebo 地图服务，而是 **ArduPilot SITL 内部仿真生成的虚拟 GPS 坐标**。整条链路如下：

```text
┌─────────────────────────────────────────────────────────────┐
│  Gazebo 物理世界                                              │
│  iris_with_standoffs 模型（姿态、位置、速度、碰撞、重力）       │
│                                                              │
│  ardupilot_gazebo 插件                                       │
│  （读取 Gazebo 模型状态，写回电机推力/力矩）                     │
└─────────────┬───────────────────────────────────────────────┘
              │ 仿真状态数据（Socket / TCP）
              ▼
┌─────────────────────────────────────────────────────────────┐
│  ArduPilot SITL（build/sitl/bin/arducopter）                  │
│                                                              │
│  SITL 模型根据仿真状态，内部合成虚拟传感器数据：                  │
│  ├─ 虚拟 IMU（加速度计 + 陀螺仪）                               │
│  ├─ 虚拟 GPS（经纬高、速度、卫星数、HDOP）                      │
│  ├─ 虚拟气压计（高度）                                         │
│  ├─ 虚拟磁罗盘（航向）                                         │
│  └─ 虚拟电池                                                  │
│                                                              │
│  这些数据进入 ArduCopter 的 EKF、导航、控制回路                  │
│  控制输出 → 发回 Gazebo 插件 → 驱动 Gazebo 模型运动               │
└─────────────┬───────────────────────────────────────────────┘
              │ MAVLink（UDP 端口 14550 或更多）
              ▼
┌─────────────────────────────────────────────────────────────┐
│  MAVProxy（console + map）                                    │
│                                                              │
│  轻量地面站，可以：                                             │
│  ├─ 查看 GPS 坐标 → map 窗口实时显示飞机在地图上的位置            │
│  ├─ 切模式、解锁、起飞、降落                                    │
│  ├─ 转发 MAVLink → output add 127.0.0.1:14550                │
│  └─ 查看原始 MAVLink 消息                                     │
└─────────────┬───────────────────────────────────────────────┘
              │ MAVLink over UDP :14550
              ▼
┌─────────────────────────────────────────────────────────────┐
│  MAVROS（ROS 桥，mavros apm.launch）                         │
│                                                              │
│  将 MAVLink 消息转换为 ROS topics：                             │
│  ├─ /mavros/global_position/global   → sensor_msgs/NavSatFix │
│  │   lat, lon, alt（WGS84 经纬度 + 椭球高）                     │
│  ├─ /mavros/global_position/raw/fix  → 更接近原始 GPS 数据     │
│  ├─ /mavros/imu/data                 → 虚拟 IMU               │
│  ├─ /mavros/state                    → 飞控模式 + 解锁状态      │
│  ├─ /mavros/local_position/pose      → 本地坐标（ENU）          │
│  └─ /mavros/...更多 topics                                   │
└─────────────┬───────────────────────────────────────────────┘
              │ ROS topics
              ▼
┌─────────────────────────────────────────────────────────────┐
│  你的 ROS 节点（Python / C++）                                 │
│                                                              │
│  订阅 /mavros/global_position/global 获取 NavSatFix           │
│  → 拿到当前 GPS 经纬高                                        │
│  订阅 /mavros/state 获取飞控模式 + 解锁状态                     │
│  → 知道飞机当前在什么模式、是否解锁                              │
└─────────────────────────────────────────────────────────────┘
```

### 12.1 关键结论

1. **GPS 不是真 GPS**，是 SITL 内部根据飞机在 Gazebo 世界中的位置反算出来的 WGS84 坐标。
2. **仿真原点**决定 GPS 坐标会落在哪个地理区域。默认是 ArduPilot 测试点（澳大利亚 CMAC 附近）。可以通过 `--home` 参数修改。
3. **MAVROS 要拿到 GPS**，需要上游链路全部通：Gazebo → 插件 → SITL → MAVLink → MAVProxy → UDP → MAVROS。
4. **MAVProxy 的 map 窗口** 是查看 GPS 最直观的方式——飞机动了，地图上的图标就跟着动。

### 12.2 从 MAVROS 拿 GPS 坐标的 Python 脚本

```python
#!/usr/bin/env python3
"""mavros_gps_logger.py - 从 MAVROS 订阅 GPS 坐标并打印/记录"""
import csv
import os
import rospy

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State

latest_state = State()
writer = None
csv_file = None


def state_cb(msg):
    global latest_state
    latest_state = msg


def gps_cb(msg):
    stamp = msg.header.stamp.to_sec()
    fix_ok = msg.status.status >= 0

    line = (
        f"t={stamp:.3f} "
        f"mode={latest_state.mode or 'UNKNOWN'} "
        f"armed={latest_state.armed} "
        f"fix={fix_ok} "
        f"lat={msg.latitude:.8f} "
        f"lon={msg.longitude:.8f} "
        f"alt={msg.altitude:.3f}"
    )
    rospy.loginfo(line)

    if writer:
        writer.writerow([
            stamp,
            latest_state.mode,
            latest_state.armed,
            fix_ok,
            msg.latitude,
            msg.longitude,
            msg.altitude,
        ])
        csv_file.flush()


if __name__ == "__main__":
    rospy.init_node("mavros_gps_logger")

    topic = rospy.get_param("~topic", "/mavros/global_position/global")
    csv_path = rospy.get_param("~csv", "")

    if csv_path:
        csv_file = open(os.path.expanduser(csv_path), "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["time", "mode", "armed", "fix_ok",
                         "latitude", "longitude", "altitude"])

    rospy.Subscriber("/mavros/state", State, state_cb, queue_size=10)
    rospy.Subscriber(topic, NavSatFix, gps_cb, queue_size=10)

    rospy.loginfo(f"Listening GPS topic: {topic}")
    rospy.spin()
```

运行：

```bash
chmod +x ~/mavros_gps_logger.py
source /opt/ros/noetic/setup.bash

# 仅打印到终端
python3 ~/mavros_gps_logger.py

# 同时保存 CSV
python3 ~/mavros_gps_logger.py _csv:=~/sitl_gps_log.csv

# 订阅更原始的 GPS fix
python3 ~/mavros_gps_logger.py _topic:=/mavros/global_position/raw/fix
```

## 13. MAVProxy console 刷屏问题排查

### 13.1 症状

MAVProxy console 窗口里遥测消息（如 `Flight battery 100 percent`）持续高速滚动，导致 `MAV>` 命令提示符被遮挡，甚至无法正常输入命令。

### 13.2 原因

MAVProxy 默认在 console 窗口同时展示多个模块的输出（status、battery、GPS、RC 等），当某些模块更新频率很高时就会刷屏。

### 13.3 解决方案

**方案 A：静音刷屏模块**

在 console 窗口确保鼠标焦点在窗口内，按回车获取 `MAV>` 提示符后输入：

```
set shownoise battery -1
```

这会彻底关掉电池状态输出。其他刷屏模块可以用同样方法静音。

**方案 B：用快捷键切换页面**

MAVProxy console 内置多个显示页面：
- `F1`：status 总览页
- `F2`：console 命令输入页（最干净）
- `F4`：遥测页
- 其他 F 键对应不同模块

按 `F2` 可以切到只显示命令提示符的页面。

**方案 C：另开一个纯命令终端（推荐）**

不动现有窗口，新建终端，直接 MAVProxy 连 SITL TCP 端口：

```bash
mavproxy.py --master=tcp:127.0.0.1:5760
```

这会得到一个干净的 `MAV>` 终端，专用于命令输入，不受刷屏影响。它和 console 窗口共享同一个 SITL 连接。

**方案 D：启动时不带 --console**

```bash
# 终端 1：Gazebo（照旧）
# 终端 2：SITL 不加 console/map
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --no-mavproxy

# 终端 3：干净的 MAVProxy
mavproxy.py --master=tcp:127.0.0.1:5760 --console --map
```

### 解锁前检查清单

当 console 里出现 **`pre-arm good`** 后，说明所有安全检查已通过，可以解锁。

常见 PreArm 报错及等待时间：

| 报错 | 原因 | 处理 |
|---|---|---|
| `PreArm: Accels inconsistent` | 加速度计还没稳定 | 等 5-10 秒，EKF 初始化完自动消失 |
| `PreArm: EKF attitude is bad` | EKF 姿态没收敛 | 同上 |
| `PreArm: AHRS: not using configured AHRS type` | AHRS 切换中 | 等它切到 EKF3 就好 |
| `PreArm: GPS is not healthy` | GPS 没定位 | 等 GPS fix，SITL 里通常很快 |
| `pre-arm good` | ✅ 全部通过 | 可以 `arm throttle` 了 |

同时出现的正常初始化消息（不是报错）：
- `EKF3 IMU0 tilt alignment complete` — 姿态对准完成
- `EKF3 IMU0 MAG0 initial yaw alignment complete` — 航向对准完成
- `GPS 1: detected as u-blox at 230400 baud` — GPS 检测到
- `EKF3 IMU0 is using GPS` — EKF 正在融合 GPS

## 14. MAVROS 验证命令

以下命令按 ROS 1 Noetic 记录，适用于 Ubuntu 20.04。

### 14.1 检查 ROS 环境

```bash
source /opt/ros/noetic/setup.bash
rosversion -d
```

期望输出：

```text
noetic
```

检查 MAVROS 是否安装：

```bash
dpkg -l | grep ros-noetic-mavros
```

如果没有安装：

```bash
sudo apt update
sudo apt install -y ros-noetic-mavros ros-noetic-mavros-extras
```

安装地理数据集：

```bash
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

如果上面脚本不存在，可以用：

```bash
cd /tmp
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
```

### 14.2 启动 roscore

终端 4：

```bash
source /opt/ros/noetic/setup.bash
roscore
```

### 14.3 启动 MAVROS

终端 5：

```bash
source /opt/ros/noetic/setup.bash

roslaunch mavros apm.launch fcu_url:=udp://:14550@
```

这个命令假设 ArduPilot SITL / MAVProxy 正在向本机 `14550` 端口输出 MAVLink：

```bash
--out udp:127.0.0.1:14550
```

或者 MAVProxy 控制台里已经执行：

```text
output add 127.0.0.1:14550
```

### 14.4 检查 MAVROS 是否读到飞控

终端 6：

```bash
source /opt/ros/noetic/setup.bash

rostopic list | grep mavros
```

查看连接状态：

```bash
rostopic echo -n 1 /mavros/state
```

如果连接成功，通常会看到：

```text
connected: True
```

查看 IMU：

```bash
rostopic echo -n 1 /mavros/imu/data
```

查看 GPS：

```bash
rostopic echo -n 1 /mavros/global_position/global
```

查看本地位置：

```bash
rostopic echo -n 1 /mavros/local_position/pose
```

查看心跳频率：

```bash
rostopic hz /mavros/state
```

### 14.5 如果 MAVROS 没有连接

先确认 SITL 启动命令里有：

```bash
--out udp:127.0.0.1:14550
```

或者在 MAVProxy 中执行：

```text
output list
output add 127.0.0.1:14550
```

再重启 MAVROS：

```bash
source /opt/ros/noetic/setup.bash
roslaunch mavros apm.launch fcu_url:=udp://:14550@
```

如果仍然不通，可以临时尝试让 MAVROS 直接连 SITL TCP 端口：

```bash
roslaunch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760
```

但直接 TCP 方式可能和 MAVProxy 抢同一个连接。当前更推荐通过 MAVProxy 的 UDP output 转发给 MAVROS。

## 15. 多窗口分工说明

当你执行 `sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map` 后，桌面上总共会出现 **4 个窗口**。它们各有分工，很容易搞混，这里集中说明。

### 15.1 窗口总览

| 窗口 | 长什么样 | 角色 | 能打字控制飞机吗 | 要不要动它 |
|---|---|---|---|---|
| **黑色终端**（飞控进程） | `Starting SITL Gazebo` / `SERIAL0 on TCP port 5760` / `Home: -35.xxx ...` | ArduCopter 虚拟飞控进程的日志输出 | ❌ 不能 | 放着看日志，不用管 |
| **运行 sim_vehicle.py 的终端** | 有 `MAV>` / `GUIDED>` 提示符 | **MAVProxy 命令行，你的驾驶舱** | ✅ 这里打字控制飞机 | 在这输入 `mode`/`arm`/`takeoff` 等命令 |
| **Console GUI 窗口** | 彩色文字，多模块遥测刷屏（模式、电池、GPS、EKF） | MAVProxy 遥测仪表盘 | 理论上能打，但刷屏导致体验极差，不建议 | 用 F1-F12 切换页面查看状态 |
| **Map GUI 窗口** | 卫星地图 + 飞机图标 + 轨迹线 | MAVProxy 导航地图 | ❌ 不能 | 观察飞机位置和轨迹 |

### 15.2 关键认知：谁才是命令输入终端

最容易搞混的一点：

- **Console GUI 窗口**（标题栏写着 "MAVProxy Console"）看起来像地面站，但它只是遥测显示板。
- **真正输入命令的地方**是你运行 `sim_vehicle.py` 的那个终端窗口——提示符长这样：`MAV>` 或 `GUIDED>` 或 `STABILIZE>`。

`sim_vehicle.py --console --map` 这条命令启动后：
1. 在它自己的终端里启动 MAVProxy 命令行界面（`MAV>` 提示符）
2. 额外弹出一个 Console GUI 窗口（遥测仪表盘）
3. 额外弹出一个 Map GUI 窗口（导航地图）

### 15.3 已验证可用的完整操作流程

从零开始跑通起降的完整步骤：

```bash
# === 每次启动前先清残留 ===
pkill -9 gzserver
pkill -9 gzclient
pkill -9 -f mavproxy
pkill -9 -f arducopter

# === 终端 1：启动 Gazebo 后端（物理引擎）===
cd ~/ardupilot_gazebo
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
gzserver --verbose worlds/iris_arducopter_runway.world

# === 终端 2（可选）：启动 Gazebo 前端（3D 画面）===
cd ~/ardupilot_gazebo
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
gzclient --verbose

# === 终端 3：启动 SITL + MAVProxy（驾驶舱）===
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

然后在这个**终端 3**（有 `MAV>` 提示符的那个）里输入：

```
mode guided       ← 切到 GUIDED 模式
arm throttle      ← 解锁
takeoff 5         ← 起飞到 5 米
mode loiter       ← 悬停
mode land         ← 降落
disarm            ← 加锁
```

### 15.4 常见问题速查

| 现象 | 原因 | 解决 |
| --- | --- | --- |
| 终端显示 `link 1 down` | MAVProxy 连不到飞控 | 飞控进程没启动或端口被占，用 `pkill` 清残留后重开 |
| `bind failed on port 5760` | 端口被上次的进程占着 | `pkill -9 -f mavproxy` 然后重试 |
| Console GUI 里打字没反应 | 刷屏太快把输入盖住了 | 用快捷键 `F2` 切到干净的输入页，或直接用 `MAV>` 终端 |
| `Flight battery 100 percent` 刷屏 | MAVProxy 默认显示电池状态 | 输入 `set shownoise battery -1` 关掉电池输出 |
| `pre-arm good` 出现后还没反应 | 一切正常，等你输命令 | 直接输入 `arm throttle` |
| `PreArm: ...` 报错 | EKF/GPS 还没就绪 | 等 5-10 秒看 `pre-arm good` 出现 |

## 16. 比赛场地世界 + D435i 相机 + GPS 流脚本

### 16.1 比赛场地 Gazebo 世界

已创建比赛场地世界文件，存放在 vault 中：[[cuadc_2026_field.world]]

#### 场地布局（基于 CUADC 2026 规则 Section 3.1）

```
                         飞行方向 X+ →
    ╔══════════════════════════════════════════════════════════════╗
    ║                      场地俯视图 (单位: 米)                    ║
    ╠══════════════════════════════════════════════════════════════╣
    ║                                                              ║
    ║  ┌─────────────────────┐  ┌──────────┐  ┌──────────┐        ║
    ║  │     起降区           │  │  投放区   │  │ 侦察区    │        ║
    ║  │  33m × 8m          │  │ 8m × 5m  │  │ 8m × 5m  │        ║
    ║  │                     │  │ [1][2][3]│  │[1][2][3] │        ║
    ║  │   ○ ← 起降点        │  │          │  │ [4] [5]  │        ║
    ║  │  (Ø0.8m, H标识)     │  │  ─起飞线  │  │          │        ║
    ║  └─────────────────────┘  └──────────┘  └──────────┘        ║
    ║  X=0              X=30           X=32.5           X=57.5    ║
    ║  原点                                                 总长~60m║
    ╚══════════════════════════════════════════════════════════════╝
```

| 区域 | 位置 (X) | 尺寸 | 内容 |
|---|---|---|---|
| 起降区 | 0~33m | 33m × 8m | 灰色地面，起降点 Ø80cm 圆形 + "H" 标识，起飞线 X=2m 红色标记 |
| 投放区 | 30~38m (中心 32.5m) | 8m × 5m | 蓝色地面，3 个白色圆筒（Ø15/20/25cm, h=30cm），每个带 1m 直径 B 区环 |
| 侦察区 | 55~63m (中心 57.5m) | 8m × 5m | 棕色地面，5 个白色圆筒（Ø20cm, h=15cm），其中 3 个内含 12×12cm 危险品标识 |

#### 在 NUC 上部署世界文件

```bash
# 1. 将 vault 中的 world 文件复制到 Gazebo 世界目录
cp cuadc_2026_field.world ~/ardupilot_gazebo/worlds/

# 2. 启动时使用新世界文件
cd ~/ardupilot_gazebo
export GAZEBO_MODEL_DATABASE_URI=
export GAZEBO_MODEL_PATH=$HOME/ardupilot_gazebo/models:$HOME/.gazebo/models:$GAZEBO_MODEL_PATH
QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
gzserver --verbose worlds/cuadc_2026_field.world

# 3. 另一个终端启动 SITL
cd ~/ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

#### 修改 SITL 的 Home 位置

默认 Home 是澳大利亚 CMAC，比赛场地在另一个位置时可以改：

```bash
# 方法1: 启动时指定经纬度
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map \
  --home=22.5,113.5,0,0   # 例如: 深圳附近

# 方法2: 用预设位置（查看可选列表）
./Tools/autotest/sim_vehicle.py --list-locations
# 例如选 Shenzhen: --location=Shenzhen
```

> 注意：世界文件里的物体位置和 GPS 坐标通过原点对应。在 Gazebo 里飞机飞了 30m 到投放区上方，GPS 坐标也会跟着偏移 ~30m。

---

### 16.2 在 IRIS 模型上添加 D435i 深度相机

两种方案，从简单到完整：

#### 方案 A：快速方案——直接用 Iris 自带相机 + Gazebo 深度插件（当天搞定）

IRIS 模型自带了一个前向摄像头（在 `iris_with_standoffs` 的 SDF 中）。如果只需要验证视觉算法，最快的方式是给它加深度渲染能力。

找到并编辑 IRIS 模型文件：

```bash
# 找到 iris_with_standoffs 模型位置
find ~/ardupilot_gazebo/models -name "*.sdf" | grep iris

# 通常是: ~/ardupilot_gazebo/models/iris_with_standoffs/iris_with_standoffs.sdf
```

在模型的 `<sensor type="camera">` 旁边，添加一个深度相机传感器：

```xml
<!-- 在 iris_with_standoffs.sdf 中，找到已有的 camera sensor，在后面添加： -->

<!-- ====== 深度相机（模拟 D435i 深度流）====== -->
<sensor name="depth_camera" type="depth">
  <pose>0.1 0 0.05 0 0.2 0</pose>   <!-- 前向安装，略微下倾11° -->
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.51844</horizontal_fov>    <!-- 87° HFOV，D435i 典型值 -->
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>                <!-- 深度图：32位浮点，单位米 -->
    </image>
    <clip>
      <near>0.15</near>    <!-- D435i 最小深度 -->
      <far>10.0</far>      <!-- 10m 最大探测距离 -->
    </clip>
  </camera>
  <plugin name="depth_camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/camera</namespace>
    </ros>
    <cameraName>depth</cameraName>
    <imageTopicName>image_raw</imageTopicName>
    <depthImageTopicName>depth/image_raw</depthImageTopicName>
    <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
    <frameName>depth_camera_link</frameName>
  </plugin>
</sensor>

<!-- ====== 彩色相机（Iris 自带的升级为 ROS 发布）====== -->
<sensor name="color_camera" type="camera">
  <pose>0.1 0 0.05 0 0.2 0</pose>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.51844</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="color_camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/camera</namespace>
    </ros>
    <cameraName>color</cameraName>
    <imageTopicName>color/image_raw</imageTopicName>
    <cameraInfoTopicName>color/camera_info</cameraInfoTopicName>
    <frameName>camera_link</frameName>
  </plugin>
</sensor>
```

#### 方案 B：自建 D435i 模型（推荐，不需要额外 apt 包）

> `ros-noetic-realsense-gazebo-plugin` 这个包不存在，不用找了。
> 用 Gazebo 自带的 `libgazebo_ros_camera.so` 就行。

**第 1 步：确认插件库存在。** 复制下面一整段到终端执行：

```bash
sudo apt install -y ros-noetic-gazebo-ros-pkgs
ls /opt/ros/noetic/lib/libgazebo_ros_camera.so
# 上面这行有输出就对了，不是 "No such file"
```

**第 2 步：创建 D435i 模型文件。** 复制下面一整段到终端执行（用 `cat` 直接把 XML 写入文件，不用手动粘贴）：

```bash
mkdir -p ~/.gazebo/models/d435i

# 写入 model.config
cat > ~/.gazebo/models/d435i/model.config << 'EOF'
<?xml version="1.0" ?>
<model>
  <name>d435i</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>CUADC Team</name></author>
  <description>Intel RealSense D435i depth camera (Gazebo model)</description>
</model>
EOF

# 写入 model.sdf（深度相机 + 彩色相机，640×480，30Hz）
cat > ~/.gazebo/models/d435i/model.sdf << 'EOF'
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="d435i">
    <static>false</static>
    <link name="camera_link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>1e-6</iyy><iyz>0</iyz>
          <izz>1e-6</izz>
        </inertia>
      </inertial>

      <!-- 外观：小黑方块代表相机机身 -->
      <visual name="body">
        <geometry><box><size>0.09 0.025 0.025</size></box></geometry>
        <material>
          <ambient>0.05 0.05 0.05 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>

      <!-- 深度相机 -->
      <sensor name="depth" type="depth">
        <update_rate>15</update_rate>
        <pose>0.045 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.51844</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>L16</format>
          </image>
          <clip>
            <near>0.15</near>
            <far>10.0</far>
          </clip>
        </camera>
        <plugin name="depth_plugin" filename="libgazebo_ros_depth_camera.so">
          <ros><namespace>/camera</namespace></ros>
          <cameraName>depth</cameraName>
          <imageTopicName>depth/image_raw</imageTopicName>
          <depthImageTopicName>depth/image_raw</depthImageTopicName>
          <depthImageCameraInfoTopicName>depth/camera_info</depthImageCameraInfoTopicName>
          <frameName>d435i_depth_link</frameName>
          <pointCloudCutoff>0.4</pointCloudCutoff>
        </plugin>
      </sensor>

      <!-- 彩色相机 -->
      <sensor name="color" type="camera">
        <update_rate>15</update_rate>
        <pose>0.045 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.51844</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <plugin name="color_plugin" filename="libgazebo_ros_camera.so">
          <ros><namespace>/camera</namespace></ros>
          <cameraName>color</cameraName>
          <imageTopicName>color/image_raw</imageTopicName>
          <cameraInfoTopicName>color/camera_info</cameraInfoTopicName>
          <frameName>d435i_color_link</frameName>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>
EOF

echo "=== DONE ==="
echo "Verify:"
ls -la ~/.gazebo/models/d435i/
# 应该看到 model.config 和 model.sdf
```

**第 3 步：把 D435i 挂到 Iris 上。** 执行：

```bash
nano ~/ardupilot_gazebo/models/iris_with_standoffs/iris_with_standoffs.sdf
```

在文件里搜索 `gimbal`（按 `Ctrl+W` 输入 `gimbal` 回车），找到类似这样的段落：

```xml
    <include>
      <uri>model://gimbal_small_2d</uri>
      ...
    </include>
    <joint name="gimbal_joint" ...>
      ...
    </joint>
```

**把这一段全部删掉，替换为：**

```xml
    <!-- D435i 深度相机（替代占位云台方块） -->
    <include>
      <name>d435i_camera</name>
      <uri>model://d435i</uri>
      <pose>0.15 0 0.05 0 0.2 0</pose>
    </include>
    <joint name="d435i_joint" type="fixed">
      <parent>iris_with_standoffs::base_link</parent>
      <child>d435i_camera::camera_link</child>
    </joint>
```

`Ctrl+X` → `Y` → 回车保存。**重启 Gazebo 生效。**

#### 查看虚拟相机画面

有 3 种方式：

```bash
# 方式 1：rqt_image_view（最简单）
rosrun rqt_image_view rqt_image_view

# 方式 2：RViz
roscore &
rosrun rviz rviz
# 在 RViz 中: Add → By topic → /color/color/image_raw
#             Add → By topic → /depth/depth/image_raw

# 方式 3：image_view 终端
rosrun image_view image_view image:=/color/color/image_raw
rosrun image_view image_view image:=/depth/depth/image_raw
```

确认话题存在：

```bash
rostopic list | grep camera
# 应该看到:
# /color/color/image_raw
# /depth/depth/image_raw
# /camera/depth/camera_info
```

---

#### 附 A：IRIS 是什么

**IRIS** 是 3D Robotics 公司 2014 年推出的一款四旋翼无人机，ArduPilot 项目把它作为默认仿真机型。你在 Gazebo 里看到的那个白色机身 + 四个旋翼 + 起落架的飞机就是它。

关键参数：

| 参数 | 值 |
|---|---|
| 轴距 | 550mm |
| 起飞重量 | ~1.5kg |
| 电池 | 3S~4S |
| 桨尺寸 | 10 英寸 |

在仿真里，它由以下文件组成：

```text
~/ardupilot_gazebo/models/iris_with_standoffs/
  ├── model.config      ← 告诉 Gazebo "这是个叫 iris 的模型"
  ├── model.sdf         ← 机身 + 旋翼 + IMU 的定义（你刚编辑的文件）
  └── meshes/
        ├── iris.dae          ← 机身的 3D 外观
        ├── iris_prop_ccw.dae ← 逆时针螺旋桨
        └── iris_prop_cw.dae  ← 顺时针螺旋桨
```

启动 SITL 时 `-f gazebo-iris` 就是告诉 ArduPilot 用 Iris 的参数文件（`gazebo-iris.parm`），里面预设了 PID、电池、重量等跟 Iris 匹配的参数。

> 比赛用的飞机可能是自研机架，参数需要自己调。但在仿真阶段用 Iris 验证控制逻辑和视觉算法足够了。

#### 附 B：URDF vs SDF 格式对比

Gazebo 仿真里经常遇到两个格式，容易搞混。

| | URDF | SDF |
|---|---|---|
| 全称 | Unified Robot Description Format | Simulation Description Format |
| 谁家的 | ROS 社区 | Gazebo（Open Robotics） |
| 描述什么 | **只描述机器人本身**（连杆 + 关节） | **描述整个仿真世界**（机器人 + 场景 + 传感器 + 灯光 + 物理） |
| 文件后缀 | `.urdf` 或 `.xacro` | `.sdf` 或 `.world` |
| 传感器 | 不支持原生传感器标签 | 内建 `<sensor>` 支持深度相机/IMU/GPS 等 |
| 碰撞 vs 视觉 | 共用同一几何体 | **分开定义**：碰撞用简单几何，视觉用精细 mesh |
| 引用其他模型 | 用 Xacro 的 `<xacro:include>` | 直接用 `<include><uri>model://...</uri></include>` |
| 位姿写法 | `<origin xyz="..." rpy="..."/>` | `<pose>X Y Z Roll Pitch Yaw</pose>` |
| 物理参数 | 只有质量 + 惯性矩阵 | 还有摩擦系数、阻尼、接触参数等 |
| 关节类型 | revolute, continuous, prismatic, fixed, floating, planar | 同上 + 更多（gearbox, screw 等） |

**一句话**：URDF 是"这个机器人长什么样"，SDF 是"这个世界里有什么，怎么动"。

你当前项目里的用法：
- `iris_with_standoffs/model.sdf` = SDF 格式，定义 Iris 无人机
- `cuadc_2026_field.world` = SDF 格式，定义整个比赛场地世界
- 世界文件用 `<include>` 把 IRIS 模型拉进场

```text
cuadc_2026_field.world (SDF 世界)
  ├── <include> iris_with_standoffs (SDF 模型)
  │     ├── base_link (机身)
  │     ├── rotor_0~3 (旋翼)
  │     ├── iris/imu_link (IMU 传感器)
  │     └── <include> d435i (你刚加的深度相机)
  ├── runway_base (跑道)
  ├── drop_cylinder_1~3 (投放区圆筒)
  └── recon_cylinder_1~5 (侦察区圆筒)
```

`<joint type="fixed">` 就是"焊死"——相机相对于机身不动。如果是 `type="revolute"` 就是旋转关节（比如旋翼）。

---

### 16.3 GPS 坐标实时流脚本

已创建脚本，存放在 vault 中：[[mavros_gps_stream.py]]

#### 功能

在终端里持续刷新显示 GPS 坐标，格式如下：

```
╔══════════════════════════════════════════════════════════════╗
║  MAVROS GPS Stream  │  2026-06-25 21:30:45                  ║
╠══════════════════════════════════════════════════════════════╣
║  GPS   | lat:   -35.36326123  lon:   149.16523210  alt:  584.12 m ║
║  FIX   | type: 3 (3D_FIX)  sats: 12  hdop_approx: 0.720          ║
║  MODE  | GUIDED           armed: True   connected: True           ║
║  SPEED | ground:  2.34 m/s  vertical:  0.12 m/s                  ║
╠══════════════════════════════════════════════════════════════╣
║  Refresh: 2 Hz | Ctrl+C to quit                              ║
╚══════════════════════════════════════════════════════════════╝
```

#### 前提条件

MAVROS 必须已经启动并连上飞控：

```bash
# 终端 A：确认 MAVROS 在跑
source /opt/ros/noetic/setup.bash
roslaunch mavros apm.launch fcu_url:=udp://:14550@

# 终端 B：验证连接
source /opt/ros/noetic/setup.bash
rostopic echo -n 1 /mavros/state
# 看到 connected: True 就行
```

#### 在 NUC 上部署并运行

```bash
# 1. 将 vault 中的脚本复制到 NUC
#    (可以通过 scp / U盘 / VSCode Remote / 直接粘贴)

# 2. 赋执行权限
chmod +x ~/mavros_gps_stream.py

# 3. 运行（前提：roscore + MAVROS 已经在跑）
source /opt/ros/noetic/setup.bash
python3 ~/mavros_gps_stream.py

# 可选参数:
python3 ~/mavros_gps_stream.py _rate:=5           # 5Hz 刷新
python3 ~/mavros_gps_stream.py _csv:=~/gps.csv    # 同时保存 CSV
python3 ~/mavros_gps_stream.py _gps_topic:=/mavros/global_position/raw/fix  # 用原始 GPS 话题
```

#### 链路总结

```
MAVROS 启动 (roslaunch mavros apm.launch)
    ↓
GPS 话题自动发布 (/mavros/global_position/global)
    ↓
mavros_gps_stream.py 订阅并格式化输出
    ↓
终端实时刷新显示 + 可选 CSV 记录
```

---

### 16.4 下一步开发顺序建议

仿真环境搭建完成后的开发优先级：

| 优先级 | 任务 | 依赖 | 说明 |
|---|---|---|---|
| P0 | 在仿真里验证起降 + GPS 坐标正常 | ⬜ 当前 | 跑通 `takeoff 5` → 看 GPS 流变化 |
| P0 | 在仿真里发 setpoint 控制飞机移动 | GPS 流正常 | 验证 Guided 模式的位置/速度控制 |
| P1 | 装 D435i 模型，验证彩色图+深度图输出 | 仿真正常 | 跑通 `rqt_image_view` 看到画面 |
| P1 | 视觉组在仿真画面中测圆筒检测算法 | 相机流正常 | 用仿真图验证 CV/YOLO 检测 |
| P2 | 仿真联调：检测→定位→setpoint→飞到目标上方 | 控制+视觉就绪 | 全闭环仿真验证 |
| P3 | 上实机 | 仿真全通 | 迁移到真飞机 |

## 17. 2026-06-26 踩坑全记录

本次从零搭建 CUADC 比赛场地 Gazebo 世界 + D435i 深度相机集成，历时整晚，以下是每个坑和解决方案。

---

### 17.1 MAVProxy 窗口分工

**现象**：打开 `sim_vehicle.py --console --map` 后看到一堆窗口，不知道哪个能打字。

**根因**：`sim_vehicle.py` 启动后产生 **4 个窗口**，只有带 `MAV>` 提示符的终端能输入命令。

| 窗口 | 能打字吗 | 用途 |
|---|---|---|
| 黑色 SITL 飞控终端 | ❌ | 虚拟飞控日志 |
| 运行 `sim_vehicle.py` 的终端（`MAV>`） | ✅ | 驾驶舱，命令在这里打 |
| Console 弹窗（GUI 遥测） | ❌刷屏 | 看状态 |
| Map 弹窗（地图） | ❌ | 看位置 |

**Console 刷屏解决**：`set shownoise battery -1` 关电池输出，或 F2 切干净页面。

---

### 17.2 模型名坑：iris_with_ardupilot ≠ iris_with_standoffs

**现象**：手工编辑了 `iris_with_standoffs/model.sdf` 加 ArduPilot 插件，结果怎么都飞不起来。

**根因**：NUC 上有**两个** Iris 模型目录：

| 目录 | 有无 ArduPilot 插件 | 用途 |
|---|---|---|
| `iris_with_ardupilot` | ✅ 自带 | 真·能飞的仿真模型 |
| `iris_with_standoffs` | ❌ 没有 | 纯 3D 展示模型 |

原版可飞的世界 `iris_arducopter_runway.world` 用的是 `iris_with_ardupilot`。我们一直在错误地编辑 `iris_with_standoffs`。**耽误了约 3 小时。**

**解决**：世界文件改用 `model://iris_with_ardupilot`，一步到位。

---

### 17.3 进程残留坑：UDP 端口杀不干净

**现象**：`pkill -9 gzserver` 后 ArduPilot 插件仍然报 `failed to bind 127.0.0.1:9002`。

**根因**：
- `fuser -k 9002/tcp` 只杀 **TCP** 端口
- ArduPilot 插件用 **UDP** 端口 9002/9003
- `gzmaster` 进程残留会保活端口，`pkill` 打不到

**正确杀法**：

```bash
killall -9 gzserver gzclient gzmaster gazebo 2>/dev/null
pkill -9 -f mavproxy; pkill -9 -f arducopter
ss -ulnp | grep -E "9002|9003" | awk '{print $NF}' | grep -oP 'pid=\K[0-9]+' | xargs -r kill -9
sleep 2
```

**教训**：残留进程是 SITL 环境 80% 的问题根源。每次启动前必须彻底清。

---

### 17.4 物理引擎坑：接触校正速度缺失

**现象**：飞机站在地面上自动弹飞、翻滚，`NAV_TAKEOFF: FAILED`。

**根因**：CUADC 世界文件缺少 ODE 物理引擎参数。原版 `iris_arducopter_runway.world` 有：

```xml
<contact_max_correcting_vel>0.1</contact_max_correcting_vel>
```

默认值极高，飞机碰地瞬间被暴力弹开。

**解决**：完整复制原版世界的 `<physics>` 和 `<gravity>` 块。

---

### 17.5 地面参数坑：摩擦系数太小

**现象**：飞机滑来滑去站不稳。

**根因**：原版地面 `mu=100, mu2=50`，我们写的 `mu=0.5, mu2=0.5`——差 200 倍。

**解决**：地面摩擦系数对齐原版 `100/50`，Plane 尺寸 `5000×5000`。

---

### 17.6 D435i 相机坑：父链接前缀

**现象**：Gazebo 报 `Couldn't Find Parent Link[base_link]`。

**根因**：`iris_with_ardupilot` 模型结构是嵌套的：

```text
iris_demo (wrapper)
  └── iris (sub-model)
        └── base_link
```

相机 joint 的 `<parent>base_link</parent>` 找不到，因为 `base_link` 在 `iris` 子模型里。

**解决**：改为 `<parent>iris::base_link</parent>`。

---

### 17.7 相机方向坑：pitch 正负号

**现象**：相机朝上拍到飞机肚子。

**根因**：SDF 的 pitch 正负号定义——`+1.5708` 朝下、`-1.5708` 朝上。写反了。

**解决**：`sed -i 's|-1.5708|1.5708|'`。

---

### 17.8 深度相机格式坑：R_FLOAT32 → L16

**现象**：Gazebo 报 `Unsupported Gazebo ImageFormat`，rqt 深度图全黑。

**根因**：
- 深度插件必须是 `libgazebo_ros_depth_camera.so`（不是 `libgazebo_ros_camera.so`）
- 格式 `R_FLOAT32` 兼容性差 → 换成 `L16`
- 帧率降到 `15Hz`

**解决**：D435i 模型文件最终版已修正。

---

### 17.9 地图制作坑：颜色、区域、H 标识

| 问题 | 最终方案 |
|---|---|
| 投放区/侦察区全是蓝色/棕色 | 换成跑道灰底色 + 白色边框 + 彩色角标 |
| 中间空隙是绿草 | 大型灰色跑道平面覆盖全场 |
| 圆筒是实心圆柱 | 生成开口空心 DAE mesh（Collada 平滑着色） |
| B 环不跟圆筒移动 | B 环嵌入圆筒模型 SDF，跟着一起拖 |
| H 标识不显示 | 加厚到 1cm，抬高到跑道面以上 |
| H 像"工"不是 H | 调比例（竖线 0.36m，横线 0.2m），转 90° |

---

### 17.10 最终可用的完整启动流程

参见 [[#10. 日常操作手册|第 10 章]]，核心三步：

1. **终端 1**（清残留 + roscore + gzserver）
2. **终端 2**（gzclient）
3. **终端 3**（SITL + MAVProxy）

纯飞控测试用 10.0 一条命令即可。

---

### 17.11 关键认知总结

| 认知 | 细节 |
|---|---|
| **先纯 SITL 再 Gazebo** | 10.0 一条命令秒测飞控，Gazebo 出问题立即排除法 |
| **清残留是第一步** | UDP 端口 + gzmaster + arducopter，一个不能漏 |
| **不要随便编辑模型 SDF** | 搞清模型名、链接层级再动手，用 `git checkout` 快速回滚 |
| **对齐能飞的世界文件** | 物理引擎、地面摩擦、Plane 尺寸、插件结构，一模一样抄 |
| **排查顺序**：端口 → 插件 → 物理 → 模型 | 不要同时改多个东西 |
| **D435i 话题名**：`/color/color/image_raw`、`/depth/depth/image_raw` | 不是 `/camera/...` |
| **`-f gazebo-iris` vs 不加** | 加 = 连 Gazebo，不加 = 纯飞控 |

