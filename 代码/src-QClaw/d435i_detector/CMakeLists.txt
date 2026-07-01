cmake_minimum_required(VERSION 3.0.2)
project(d435i_detector)

## 使用 C++14
add_compile_options(-std=c++14)

## 查找 catkin 及所需组件
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

## 生成自定义消息
add_message_files(
  FILES
  YellowCircle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin 配置
###################################
catkin_package(
  CATKIN_DEPENDS
    roscpp rospy std_msgs sensor_msgs cv_bridge image_transport message_runtime
)

###########
## Install
###########

# 安装 Python 脚本
catkin_install_python(PROGRAMS
  scripts/d435i_publisher.py
  scripts/yellow_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 安装 msg 文件
install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)
