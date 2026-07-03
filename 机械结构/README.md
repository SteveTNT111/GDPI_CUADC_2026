# 机械结构

> CUADC 2026 无人机机体与传感器安装结构。

---

## 选用机架

**ZD550 四旋翼碳纤维折叠机架**（第一代，550mm 轴距）。

市售标准件，在此基础上根据比赛需求做了改装（电机座、设备安装板、相机挂载等）。

---

## 目录结构

```
机械结构/
├── README.md                                              # 你正在看的
├── ZD550四旋翼无人机碳纤维机架模型图纸 stp格式/              # 机架原厂图纸
│   ├── Frame ZD550.stp                                    #   STEP 格式装配体（通用，各 CAD 可打开）
│   ├── Frame ZD550.SLDASM                                 #   SolidWorks 装配体
│   ├── Frame ZD550.png                                    #   展开状态预览
│   ├── Frame ZD550 stowed.png                             #   折叠状态预览
│   └── Frame ZD550.SLDPRT                                 #   SolidWorks 零件（如有）
│
└── d435i-intel-realsense-1.snapshot.1/                    # D435i 深度相机三维模型
    └── D435i_Solid.SLDPRT                                 #   SolidWorks 零件
```

> **注意：** `.zip` 压缩包不能上传到 Git 仓库，已将内容解压。请把 `.zip` 加入 `.gitignore`，避免下次误提交。

---

## 文件格式说明

| 格式 | 说明 | 用什么打开 |
|------|------|-----------|
| `.stp` / `.step` | 通用 CAD 交换格式 | SolidWorks / Fusion 360 / FreeCAD / CATIA |
| `.SLDASM` | SolidWorks 装配体 | SolidWorks |
| `.SLDPRT` | SolidWorks 零件 | SolidWorks |

如果你没有 SolidWorks，用 Fusion 360（免费）或 FreeCAD（开源）导入 `.stp` 即可查看和修改。

---

## 改装说明

> TODO：在此记录对 ZD550 机架的具体改装——
> - 电机座是否改动
> - 设备安装板（NUC / 飞控 / RTK）的位置
> - D435i 相机挂载方式（下挂 / 前挂 / 角度 / 减震）
> - 抛投器的安装位置与联动机构
> - 电池安装位置

---

## 关于压缩包

Git 对二进制大文件不友好，`.zip` 超过一定大小会被拒绝（或需要 Git LFS）。
如果你把 `.zip` 解压后里面的 `.stp` / `.SLDPRT` 仍然上传失败，可能是单个 CAD 文件超过 100MB。
解决方案：
- 用 Git LFS 管理大文件
- 或者只上传 `.stp`（STEP 通用格式，体积最小），不传 SolidWorks 原生格式
