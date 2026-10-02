#!/usr/bin/env bash
# 一键重启 CUADC 2026 仿真环境 —— 现为 demo_up.sh --reset 的兼容壳
#
# 历史版本三宗罪（错题 #17，2026-09-23 重建）:
#   1) nohup 无 setsid → 启动命令一结束整套环境被进程组回收（#16a 必死）
#   2) 用 /mavros/state connected 判连接 → 缓存假象误导（#16c）
#   3) 只拉 server+SITL+MAVROS+rc 四件，不拉流（#2）、不拉相机/GUI/网页 →
#      "重启完还要手工补 5、6 步" = 每次演示都慢的根源之一
# 现在: demo_up --reset = 精确清场 → 全栈重建 → 流闸门 → 窗口摆位 →
#       130s 危险窗口 → 启动 env_guard → preflight 收官体检，一条命令到底。
exec bash "$(dirname "$0")/demo_up.sh" --reset "$@"
