cmake_minimum_required(VERSION 3.0.2)
project(cuadc_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  mavros_msgs
  sensor_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  rospy
  geometry_msgs
  mavros_msgs
  sensor_msgs
  std_msgs
)

catkin_install_python(PROGRAMS
  scripts/one_key_takeoff.py
  scripts/one_key_takeoff_forward_land.py
  scripts/one_key_takeoff_wgs84_forward_rtl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
