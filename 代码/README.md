# 代码文件夹

> CUADC 2026 全部飞行代码。**本文件夹推送到 GitHub。**
>
> 🔥 **开发流程**：
> ```
> 队员在自己的 NUC 上开发 → 推到自己文件夹 → GitHub 仓库
>     → 伍尚京 review → 拷到飞机 NUC 验证
>     → 验证通过 → 合并进 cuadc_src/（稳定版）
> ```
> - `cuadc_src/` = 🔒 伍尚京维护的稳定版。**只有验证过的代码才能进这里。**
> - `代码/队员名字/` = 队员的独立开发空间，写完 push 上来等 review。

---

## 目录结构

```
代码/
├── README.md                                    # 你正在看的
│
├── cuadc_src/                                   # 🔒 主功能包（伍尚京维护，其他人不要动）
│   ├── scripts/                                 #   核心 Python 节点
│   │   ├── main.py                              #     主控：状态机 + 飞行模式切换 + 起飞指令
│   │   ├── servo_test.py                        #     舵机测试：MAVROS 控制 5/6 通道抛投器
│   │   ├── camera_node.py                       #     D435i 相机驱动
│   │   ├── detector_node.py                     #     目标检测（YOLO + 传统CV）
│   │   ├── geopose_node.py                     #     坐标变换（相机→机体→ENU→大地）
│   │   ├── flight_data_video_recorder_node.py   #     飞行数据录像：解锁自动录制 + OSD叠加
│   │   └── auto_drop_node.py                    #     自动抛投：检测对准后自动触发舵机释放
│   ├── launch/                                  #   启动文件
│   │   ├── run_main.launch                      #     总启动：主控 + 相机 + YOLO + 坐标变换
│   │   ├── camera_node.launch                   #     D435i 相机（配合 rviz）
│   │   ├── detector_node.launch                 #     YOLO 检测（相机 + OpenCV 窗口）
│   │   ├── run_servo_test.launch               #     舵机测试终端（手动 on/off）
│   │   ├── run_flight_recorder.launch           #     飞行数据录像（解锁自动录，叠加高度/GPS/电压）
│   │   └── auto_drop.launch                     #     自动抛投触发
│   ├── config/                                  #   参数文件
│   │   └── params.yaml
│   ├── msg/                                     #   自定义 ROS 消息
│   │   ├── GeoTarget.msg
│   │   ├── YoloDetection.msg
│   │   └── YoloDetections.msg
│   ├── models/                                  #   YOLO 模型存放（best.pt 不入库，需手动放入）
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src_README.md                             #   节点与启动命令参考
│
├── 视觉组独立完成的部分（对应src4.4）/           # ★ 视觉组最新交付：YOLO + 大地坐标变换
│   ├── CODE_EXPLANATION.md                      #   完整架构说明与节点详解
│   ├── RUN_COMMANDS.md                          #   常用运行命令快速参考
│   └── d435i_yellow_circle_detector/            #   ROS 功能包（可直接放 NUC 编译）
│
├── d435i_yellow_circle_detector（对应src4.3.1）/ # 旧版视觉包（有 YOLO，无 geopose，待删除）
│
├── 视觉组旧版本代码管理/                         # 视觉组全部迭代版本存档（参考用）
│   ├── src-1.0/ 到 src-1.2/                    #   基础黄色圆检测
│   ├── src2.0/ 到 src2.1/                      #   增加二值化
│   ├── src3.0/ 到 src3.4/                      #   增加录制/采集/桌面脚本
│   ├── src4.0/ 到 src4.3.1/                    #   增加 YOLO + 比赛任务
│   ├── src4.4(显示假的大地坐标)/                #   增加大地坐标（← 新版来源）
│   └── src（相机启动）/                         #   最简相机启动版本
│
├── src-QClaw/                                   # D435i 检测器 ROS2 包（参考）
│   ├── d435i_detector/                          #   ROS2 版本（setup.py）
│   └── src-1.1(最可用，30FPS）/                  #   ROS1 版本副本（与旧版存档重复）
│
└── （队员代码文件夹）                            # 见下方规则
    ├── 你的名字/                                 #   用你的名字或功能命名
    │   ├── 代码文件.py                           #     能跑的代码
    │   └── 代码文件.md                           #     同名说明文档
    └── ...
```

---

## cuadc_src —— 主功能包

**这是上飞机的核心代码。所有权归伍尚京，其他人只读。**

### 功能概述

| 脚本 | 功能 | 运行方式 |
|------|------|---------|
| `main.py` | 状态机主控：切换飞行模式 → 起飞 → 巡航 → 识别 → 对准 → 投放 → 返航 | `roslaunch cuadc_vision run_main.launch` |
| `servo_test.py` | 舵机测试：终端输入 `A/B on/off`、`QDFS`、`all off` 控制 5/6 通道 PWM | `roslaunch cuadc_vision run_servo_test.launch` |
| `camera_node.py` | D435i 驱动，发布 RGB + 深度图 | `roslaunch cuadc_vision camera_node.launch` |
| `detector_node.py` | YOLO 目标检测，弹 OpenCV 窗口 | `roslaunch cuadc_vision detector_node.launch` |
| `geopose_node.py` | 相机系 → 机体 → ENU → 大地坐标变换 | 由 run_main.launch 自动启动（可选） |
| `flight_data_video_recorder_node.py` | 飞行数据录像：解锁自动录制，画面叠加高度/GPS/电压 | `roslaunch cuadc_vision run_flight_recorder.launch` |
| `auto_drop_node.py` | 自动抛投：检测目标对准光心后自动触发舵机释放 | `roslaunch cuadc_vision auto_drop.launch` |

### 启动文件说明

| 启动文件 | 包含节点 | 用途 |
|---------|---------|------|
| `run_main.launch` | main + camera + detector + geopose | **总启动**：完整比赛流程一键启动 |
| `camera_node.launch` | camera | 相机驱动（配合 rviz 查看画面） |
| `detector_node.launch` | camera + detector | YOLO 检测（带 OpenCV 窗口） |
| `run_servo_test.launch` | servo_test | 舵机独立测试终端 |
| `run_flight_recorder.launch` | camera + flight_data_video_recorder | 飞行数据录像（解锁自动录，用于航线规划） |
| `auto_drop.launch` | detector + auto_drop | 自动抛投触发（检测对准→释放舵机） |

### 舵机说明

- 5 通道：A 舵机，对应前方抛投器
- 6 通道：B 舵机，对应后方抛投器
- 一个舵机可以驱动两侧抛投器（机械联动），也可以用两个舵机独立控制

## 工作流程

### 队员：在自己的 NUC 上开发 → 推到仓库

```bash
# 1. 在自己的 NUC 上写代码（用 Claude Code / VS Code 都行）
#    代码放在 ~/catkin_ws/src/你的测试包/

# 2. 测试通过后，把代码复制到仓库里自己的文件夹
cp -r ~/catkin_ws/src/你的测试包 ~/GDPI_CUADC_2026/代码/你的名字/

# 3. 推送
cd ~/GDPI_CUADC_2026
git add 代码/你的名字/
git commit -m "陈智勇：xxx 功能验证通过"
git push
```

### 伍尚京：review → 验证 → 合并进稳定版

```bash
# 1. 拉取队员推送的代码
cd ~/GDPI_CUADC_2026 && git pull

# 2. review 代码/队员名字/ 下的内容

# 3. 拷贝到飞机 NUC 上验证
#    （通过 U 盘或 scp）

# 4. 验证通过后，合并进 cuadc_src/
#    把跑通的代码整理好放进 cuadc_src/scripts/ 或其他对应位置

# 5. 推送稳定版
git add 代码/cuadc_src
git commit -m "合并验证通过：陈智勇 xxx 功能"
git push
```

---

## 如何在 NUC 上推送代码

> 在 NUC 上改完代码想直接推回 GitHub？可以的。让 NUC 的 VS Code 登录你的 GitHub 账号即可。

### 一次性配置：让 NUC 登录 GitHub

#### 方式 A：HTTPS + Token（推荐，最简单）

**第 1 步：在 NUC 上配置 Git 身份**

```bash
git config --global user.name "你的GitHub用户名"
git config --global user.email "你的GitHub注册邮箱"
```

**第 2 步：在任意一台能上网的电脑上生成 GitHub Token**

1. 浏览器打开 https://github.com/settings/tokens
2. 点 `Generate new token` → `Generate new token (classic)`
3. Note 填 `NUC1`，过期时间选长一点
4. 勾选 `repo`（全部仓库读写权限）
5. 点生成，**复制 token（只显示一次！）**

**第 3 步：在 NUC 上用 Token 克隆**

```bash
cd ~
git clone https://github.com/SteveTNT111/GDPI_CUADC_2026.git
# 用户名输入你的 GitHub 用户名
# 密码粘贴刚才复制的 token（终端里不会显示，正常）
```

**第 4 步：让 Git 记住 token（不用每次输入）**

```bash
git config --global credential.helper store
```

之后在 NUC 的 VS Code 里 commit → push 就能直接用，不会再弹登录。

#### 方式 B：SSH Key（更专业，适合多台 NUC）

**第 1 步：NUC 上生成 SSH 密钥**

```bash
ssh-keygen -t ed25519 -C "nuc1@cuadc" -f ~/.ssh/id_ed25519_github
# 一路回车不设密码
```

**第 2 步：复制公钥**

```bash
cat ~/.ssh/id_ed25519_github.pub
```

**第 3 步：上传到 GitHub**

1. 浏览器打开 https://github.com/settings/keys
2. 点 `New SSH Key`
3. Title 填 `NUC1`，Key 粘贴刚才复制的公钥
4. 保存

**第 4 步：用 SSH 地址克隆**

```bash
cd ~
git clone git@github.com:SteveTNT111/GDPI_CUADC_2026.git
```

### VS Code 里 commit 和 push

和普通电脑上一样：

1. `Ctrl+Shift+G` 打开源代码管理
2. 点你修改的文件的 `+` 号（stage）
3. 上方输入框写提交信息，点 `✓`（commit）
4. 点 `...` → `推送`（push）

### NUC 修代码的提交规范

NUC 上的修改（包括 Claude Code 的修改）放在 `代码/NUC修复/` 下，**不要直接改 cuadc_src/**。由伍尚京确认后手动合并进主分支。

```
代码/NUC修复/
├── 2026-07-02_NUC1_舵机控制修正/
│   ├── servo_fix.py
│   └── servo_fix.md
└── 2026-07-03_NUC2_视觉参数调优/
    ├── detector_params.yaml
    └── detector_params.md
```

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
