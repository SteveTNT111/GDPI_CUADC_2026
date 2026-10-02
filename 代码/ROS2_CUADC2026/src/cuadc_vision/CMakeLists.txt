cmake_minimum_required(VERSION 3.8)
project(cuadc_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

set(msg_files
  "msg/BucketAimInfo.msg"
  "msg/BucketEstimates.msg"
  "msg/BucketInfo.msg"
  "msg/BucketTarget.msg"
  "msg/GeoTarget.msg"
  "msg/MissionStatus.msg"
  "msg/YoloDetection.msg"
  "msg/YoloDetections.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

# Python 脚本 / 公共库安装（保持与原 ROS1 同名脚本布局，
# 以便脚本之间以 `from xxx import ...` 方式互相引用）
install(PROGRAMS
  scripts/camera_node.py
  scripts/detector_node.py
  scripts/servo_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# launch 文件安装（detector_node.launch.py → share/cuadc_vision/launch）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# YOLO 权重目录安装（models/ → lib/models，匹配 detector_node 默认路径 ../models/best.pt）
install(DIRECTORY models
  DESTINATION lib
)

ament_package()