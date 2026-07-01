# 代码文件夹

> CUADC 2026 全部飞行代码。**本文件夹推送到 GitHub，也是 NUC 机载电脑上 ROS 工作空间的来源。**

---

## 目录结构

```
代码/
├── README.md                              # 你正在看的
├── cuadc_src/                             # 🔒 主功能包（伍尚京维护，其他人不要动）
│   ├── scripts/                           #   核心 Python 节点
│   │   ├── main.py                        #     主控：状态机 + 飞行模式切换 + 起飞指令
│   │   ├── servo_test.py                  #     舵机测试：MAVROS 控制 5/6 通道抛投器
│   │   ├── camera_node.py                 #     D435i 相机驱动
│   │   ├── detector_node.py               #     目标检测（YOLO + 传统CV）
│   │   └── geopose_node.py               #     坐标变换（相机→机体→ENU→大地）
│   ├── launch/                            #   启动文件
│   │   ├── cuadc_run.launch               #     主启动：一键启动全部节点
│   │   └── run_servo_test.launch          #     舵机测试终端（手动 on/off）
│   ├── config/                            #   参数文件
│   │   └── params.yaml
│   ├── msg/                               #   自定义 ROS 消息
│   │   ├── GeoTarget.msg
│   │   ├── YoloDetection.msg
│   │   └── YoloDetections.msg
│   ├── models/                            #   仿真模型
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── README.md
│
├── 视觉原始版本存档/                        # 视觉组交付物（多版本迭代记录，参考用）
│   ├── src-1.0/ 到 src-4.4/              #   YOLO + 黄色圆检测各版本
│   └── src最新版本/                        #   当前最可用版本
│
├── src-QClaw/                             # D435i 检测器 ROS 包（参考）
│
├── 识别圆筒的yolov8权重文件/               # YOLO 模型
│   └── best.pt
│
└── （队员代码文件夹）                       # 见下方规则
    ├── 你的名字/                           #   用你的名字或功能命名
    │   ├── 代码文件.py                     #     能跑的代码
    │   └── 代码文件.md                     #     同名说明文档
    └── ...
```

---

## cuadc_src —— 主功能包

**这是上飞机的核心代码。所有权归伍尚京，其他人只读。**

### 功能概述

| 脚本 | 功能 | 运行方式 |
|------|------|---------|
| `main.py` | 状态机主控：切换飞行模式 → 起飞 → 巡航 → 识别 → 对准 → 投放 → 返航 | `roslaunch cuadc_src cuadc_run.launch` |
| `servo_test.py` | 舵机测试：终端输入 `on`/`off` 控制 5/6 通道 PWM（1100/1400） | `roslaunch cuadc_src run_servo_test.launch` |
| `camera_node.py` | D435i 驱动，发布 RGB + 深度图 | 由 cuadc_run.launch 自动启动 |
| `detector_node.py` | YOLO + 传统CV 检测，发布目标位置 | 由 cuadc_run.launch 自动启动 |
| `geopose_node.py` | 相机系 → 机体 → ENU → 大地坐标变换 | 由 cuadc_run.launch 自动启动 |

### 舵机说明

- 5 通道：抛投器 1（PWM 1100=关闭, 1400=打开）
- 6 通道：抛投器 2（PWM 1100=关闭, 1400=打开）
- 一个舵机可以驱动两侧抛投器（机械联动），也可以用两个舵机独立控制

### 部署到 NUC

```bash
# 在 NUC 上
cd ~/catkin_ws/src
ln -s /path/to/GDPI_CUADC_2026/代码/cuadc_src .
cd ~/catkin_ws
catkin_make
```

测试完成后，将 NUC 硬盘镜像克隆到另外两台 NUC。

---

## 队员代码管理规则

### 每个人必须遵守

1. **cuadc_src 不要碰** —— 那是伍尚京管的，有问题在群里说
2. **在自己的文件夹里干活** —— 用你的名字或功能名建文件夹
3. **每个代码文件配一个同名 .md 说明** —— 至少写清楚：
   - 这个脚本干什么
   - 依赖什么（pip list）
   - 怎么跑（命令行）
4. **在自己电脑上测通了再上传** —— 不要传半成品

### 提交步骤（VS Code 图形界面）

```
① 把代码和说明文件放进 代码/你的名字/
② Ctrl+Shift+G 打开源代码管理
③ 只点你自己文件的 + 号（不要点别人的）
④ 写一行提交信息："陈智勇：桶检测脚本 v2"
⑤ 点 ✓ 提交
⑥ 点 ... → 推送
```

### 如果你想在 NUC 上独立测试一个功能

不要动 cuadc_src。在你的文件夹下建一个 `src/` 子文件夹：

```
代码/你的名字/
├── 功能名.py              # 你的代码
├── 功能名.md              # 说明文档
└── src/                   # ROS 包（可选）
    └── 你的测试包/
        ├── scripts/
        ├── launch/
        ├── CMakeLists.txt
        └── package.xml
```

把 `src/你的测试包` 复制到 NUC 的 `~/catkin_ws/src/` 下面即可独立运行，不会影响 cuadc_src。

---

## NUC 同步计划

```
当前：
  一号机 NUC（黑色）← 伍尚京开发 cuadc_src v1.0

v1.0 测试通过后：
  一号机 NUC 硬盘 → 克隆 → 二号机 NUC（视觉组白色#1）
                         → 三号机 NUC（仿真组白色#2）

三台 NUC 运行同一个 cuadc_src，各自负责不同任务。
```

---

> 以上规则从 2026-07-02 起执行。有问题在群里 @伍尚京。
