cmake_minimum_required(VERSION 3.8)
project(cuadc_visionY26)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 复用 cuadc_vision 里已经生成的 ROS 接口（msg/*），本包不再重复生成，
# 以保证消息类型与 cuadc_vision 完全一致（控制脚本无需改动）。
find_package(cuadc_vision REQUIRED)

# Python 脚本安装（与 cuadc_vision 同名脚本布局）
install(PROGRAMS
  scripts/camera_node.py
  scripts/detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# YOLO26 ONNX 权重目录（models/ → lib/models，匹配 detector_node 默认路径 ../models/best_yolo26.onnx）
install(DIRECTORY models
  DESTINATION lib
)

ament_package()
