#!/bin/bash
# 清理 ROS2 残留文件（在 Ubuntu 上运行）
# 用法：cd ~/d435i_ws/src/d435i_detector && bash cleanup_ros2.sh

set -e

echo "=== 清理 ROS2 残留文件 ==="

rm -f setup.py
echo "[删除] setup.py"

rm -f setup.cfg
echo "[删除] setup.cfg"

rm -rf d435i_detector/
echo "[删除] d435i_detector/  (ROS2 Python 包目录)"

rm -rf resource/
echo "[删除] resource/  (ROS2 包标记)"

rm -f launch/d435i_detector.launch.py
echo "[删除] launch/d435i_detector.launch.py  (ROS2 Python launch)"

rm -f cleanup_ros2.sh
echo "[删除] cleanup_ros2.sh  (本脚本)"

echo ""
echo "=== 清理完成！ROS1 文件已保留 ==="
echo "保留的文件："
echo "  CMakeLists.txt  package.xml  README.md"
echo "  msg/YellowCircle.msg"
echo "  scripts/d435i_publisher.py  scripts/yellow_detector.py"
echo "  launch/d435i_detector.launch"
