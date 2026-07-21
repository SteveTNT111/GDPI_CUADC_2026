# MissionStatus.msg — main.py → detector_node 任务状态同步
#
# 用于 detector_node 在画面上显示弹药、瞄准、抛投等任务状态信息。

uint8 ammo_a                # 前抛投器 (A) 剩余弹药数，0 表示无/未挂载
uint8 ammo_b                # 后抛投器 (B) 剩余弹药数，0 表示无/未挂载
bool aiming                 # 飞控处于 GUIDED 模式且正在执行对准任务
string last_drop            # 最近一次抛投的抛投器编号 ("A" / "B" / "")
