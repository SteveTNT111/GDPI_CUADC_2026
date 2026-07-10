# 代码文件夹

> CUADC 2026 全部飞行代码。**推送到 GitHub。**
>
> 🔥 **开发流程**：队员 NUC 开发 → 推到自己文件夹 → 伍尚京 review → 飞机 NUC 验证 → 合并进 `cuadc_src/`

---

## 目录结构

```
代码/
├── CODE_README.md                        # 你正在看的
│
├── cuadc_src/                            # 🔒 稳定版（伍尚京维护，验证通过才合并）
│   ├── scripts/                          # 8 个 Python 节点
│   ├── launch/                           # 7 个启动文件
│   ├── config/params.yaml
│   ├── msg/                              # 5 个自定义消息
│   ├── src_README.md                     # 节点详解 + 坐标系 + 依赖
│   └── detector_node_flow.md
│
├── NUC上面的测试功能包/                    # NUC 实际飞行版本（根据飞行数据修改）
│
├── LSY/                                  # 赖斯远 — 视觉 ONNX 测试
│   └── cuadc_src/
│
├── LCY/                                  # 刘辰宇 — MAVROS/Guided 学习文档
│
├── 视觉组独立完成的部分（对应src4.4）/      # 视觉组最新交付
│   └── d435i_yellow_circle_detector/
│
├── 视觉组旧版本代码管理/                   # 迭代存档（src-1.0 ~ src4.4）
│
└── src-QClaw/                            # D435i 检测器参考
```

---

## 各文件夹说明

| 文件夹 | 谁管 | 说明 |
|--------|:--:|------|
| `cuadc_src/` | 🔒 伍尚京 | 稳定版，所有验证过的代码最终合并到这里 |
| `NUC上面的测试功能包/` | 🔒 伍尚京 | NUC 飞行版本，勿修改 |
| `LSY/` | 赖斯远 | 视觉 ONNX 测试 |
| `LCY/` | 刘辰宇 | MAVROS / Gazebo / Guided 文档 |
| `视觉组独立完成的部分/` | 存档 | 视觉组最新交付 |
| `视觉组旧版本代码管理/` | 存档 | 迭代历史参考 |
| `src-QClaw/` | 参考 | D435i 检测器 ROS2 包 |

---

## 工作流程

### 队员：NUC 开发 → 推到自己文件夹

```bash
cp -r ~/catkin_ws/src/你的测试包 ~/GDPI_CUADC_2026/代码/你的名字/
cd ~/GDPI_CUADC_2026
git add 代码/你的名字/
git commit -m "XXX：xxx 功能"
git push
```

### 伍尚京：review → 验证 → 合并进稳定版

```bash
cd ~/GDPI_CUADC_2026 && git pull
# → review 代码/队员名字/
# → 飞机 NUC 验证
# → 通过后合并进 cuadc_src/
git add 代码/cuadc_src
git commit -m "合并验证通过：xxx"
git push
```

---

> 节点详解、依赖安装、坐标系统说明见 [cuadc_src/src_README.md](cuadc_src/src_README.md)
