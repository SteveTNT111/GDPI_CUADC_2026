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
| 飞行控制 | MAVROS + ArduPilot Guided 模式 | ✅ |
| 仿真 | AP SITL + Gazebo Classic | ✅ |
| 舵机投放 | MAVROS 舵机 PWM 控制 | ✅ |
| 坐标变换 | 相机系 → 机体 → ENU → WGS84 大地坐标 | ✅ |

---

## 目录结构

```
GDPI_CUADC_2026/
├── README.md                               # 你正在看的
├── 备赛文档/                                # 项目规划与管理
│   ├── 00_备赛计划.md                       #   总体时间线与目标
│   ├── 01_队伍分工.md                       #   人员与职责
│   ├── 02_进度追踪.md                       #   每日进展记录
│   ├── 03_参考仓库.md                       #   开源参考仓库与视频链接
│   ├── 04_学习任务与能力考核.md              #   队员学习路径
│   ├── 06_系统程序架构.md                    #   ROS 节点设计与状态机
│   ├── 07_工程规范.md                       #   代码提交与文档规范
│   ├── Git使用指南-VSCode图形界面.md         #   不会命令行的看这个
│   ├── 动力套装选型/                         #   电机/桨/电池力效分析
│   │   ├── 飞机重量参数.md
│   │   └── 力效表.png
│   └── ArduPilot_Copter45_SITL_Gazebo环境记录.md
│
├── 机械结构/                                # ZD550 机架 + D435i 相机三维模型
│   ├── README.md                            #   机架说明与文件格式
│   ├── ZD550四旋翼无人机碳纤维机架模型图纸 stp格式/
│   └── d435i-intel-realsense-1.snapshot.1/  #   D435i 相机 SolidWorks 模型
│
├── 代码/                                    # 全部飞行代码
│   ├── README.md                            #   代码目录详细说明
│   ├── cuadc_src/                           #   🔒 主功能包（伍尚京维护）
│   │   ├── scripts/                         #     main / servo_test / camera / detector / geopose / flight_recorder
│   │   ├── launch/                          #     camera_node / detector_node / run_main / run_servo_test / run_flight_recorder / auto_drop
│   │   ├── config/                          #     params.yaml
│   │   ├── msg/                             #     自定义 ROS 消息
│   │   └── models/                          #     YOLO 模型存放（best.pt 不入库，需手动放入）
│   ├── 视觉组独立完成的部分（对应src4.4）/     #   ★ 视觉组最新交付：YOLO + 大地坐标变换
│   │   ├── CODE_EXPLANATION.md              #     架构说明与节点详解
│   │   ├── RUN_COMMANDS.md                  #     常用命令快速参考
│   │   └── d435i_yellow_circle_detector/    #     ROS 功能包
│   ├── d435i_yellow_circle_detector（对应src4.3.1）/  # 旧版视觉包（待删除）
│   ├── 视觉组旧版本代码管理/                  #   视觉组完整迭代存档（src-1.0 ~ src4.4）
│   ├── src-QClaw/                           #   D435i 检测器 ROS2 包（参考）
│
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
