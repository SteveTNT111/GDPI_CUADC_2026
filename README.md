# GDPI CUADC 2026

> 广东理工职业学院 CUADC 2026 备赛仓库。
> 🚨 南部赛区 7/21 | 7/5 报名截止 | 7 人团队 | ROS Noetic + ArduPilot + YOLOv8
>
> B站：[待补充]

---

## 队伍

广东理工职业学院，首次参赛。参考了哈工大 2026CUADC 的开源仓库。

---

## 技术栈

| 模块 | 方案 | 状态 |
|------|------|:--:|
| 飞控 | X7 Nano + ArduPilot | ✅ |
| 定位 | RTK 厘米级 | ✅ |
| 机载电脑 | Intel NUC ×3 + Ubuntu 20.04 + ROS Noetic | ✅ |
| 深度相机 | Intel RealSense D435i | ✅ |
| 目标检测 | YOLOv8 + 传统CV（黄色圆检测） | ✅ |
| 控制 | MAVROS + AP Guided 模式 setpoint | 🔄 |
| 仿真 | AP SITL + Gazebo Classic | ✅ |
| 舵机投放 | MAVROS 舵机指令 | ✅ |
| 坐标变换 | 相机系 → 机体 → ENU → 大地 | 🔄 |

---

## 目录结构

```
GDPI_CUADC_2026/
├── README.md                         # 你正在看的（会持续更新）
├── 备赛文档/                          # 项目规划与管理
│   ├── 00_备赛计划.md                 #   总体时间线与目标
│   ├── 01_队伍分工.md                 #   人员与职责
│   ├── 02_进度追踪.md                 #   每日进展记录
│   ├── 03_参考仓库.md                 #   开源参考仓库与视频链接
│   ├── 04_学习任务与能力考核.md        #   队员学习路径
│   ├── 06_系统程序架构.md              #   ROS 节点设计与状态机
│   ├── 07_工程规范.md                 #   代码提交与文档规范
│   ├── Git使用指南-VSCode图形界面.md   #   不会命令行的看这个
│   ├── 动力套装选型/                   #   电机/桨/电池力效分析
│   │   ├── 飞机重量参数.md
│   │   └── 力效表.png
│   └── ArduPilot_Copter45_SITL_Gazebo环境记录.md
├── 代码/                              # 飞行代码（ROS 工作空间）
│   ├── cuadc_src/                     #   主 ROS 包（视觉+控制+坐标变换）
│   │   ├── scripts/                   #      核心节点：camera/detector/geopose
│   │   ├── launch/                    #      启动文件
│   │   ├── config/                    #      参数文件
│   │   ├── msg/                       #      自定义消息
│   │   └── models/                    #      仿真模型
│   ├── src-QClaw/                     #   D435i 检测器 ROS 包（备份参考）
│   └── 视觉/                          #   视觉组交付物（多版本迭代记录）
└── .gitignore
```


---

## 参考仓库

感谢以下队伍的开源：

| 队伍 | 仓库 | 说明 |
|------|------|------|
| 哈工大 2026 | [2026CUADC](https://github.com/Accelerate11/2026CUADC) | 主要参考 |
| CUADC-code | [CUADC-code](https://github.com/mikelamch/CUADC-code) | YOLO 深度测距 |
| CUADC-zh | [CUADC-zh](https://github.com/ZH-zzh-ZH/CUADC-.git) | offboard 控制 |
| 2024 CUADC | [2024_CUADC](https://github.com/une-glace/2024_CUADC) | 全链路参考 |

详见 [备赛文档/03_参考仓库.md](备赛文档/03_参考仓库.md)


---

## 许可证

MIT License —— 随便用、随便改、随便分发。保留署名就行。

详见 [LICENSE](LICENSE)

---

> 最后更新：2026-07-02
