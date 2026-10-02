cmake_minimum_required(VERSION 3.8)
project(cuadc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cuadc_vision REQUIRED)

# Python 控制脚本（保持与原 ROS1 cuadc_control 同名脚本布局）
install(PROGRAMS
  scripts/auto_drop_config.py
  scripts/one_key_takeoff.py
  scripts/one_key_takeoff_forward_land.py
  scripts/one_key_takeoff_wgs84_forward_rtl.py
  scripts/semi_auto_drop_test.py
  scripts/auto_drop_test.py
  DESTINATION lib/${PROJECT_NAME}
)

# YAML 配置模板：运行时以工作区 config/auto_drop.yaml 为准（Web UI 编辑该文件）；
# 此处随包安装一份默认，供 ensure_config() 首次播种 / 打包参考。
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../config/auto_drop.yaml")
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../config/auto_drop.yaml"
    DESTINATION share/${PROJECT_NAME}/config)
endif()

ament_package()