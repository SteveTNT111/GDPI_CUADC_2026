cmake_minimum_required(VERSION 3.0.2)
project(cuadc_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy std_msgs sensor_msgs geometry_msgs
  cv_bridge image_transport tf2_ros tf2_geometry_msgs
  message_generation
)

add_message_files(FILES YoloDetection.msg YoloDetections.msg GeoTarget.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS
  rospy std_msgs sensor_msgs geometry_msgs
  cv_bridge image_transport tf2_ros tf2_geometry_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/camera_node.py scripts/detector_node.py scripts/geopose_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
