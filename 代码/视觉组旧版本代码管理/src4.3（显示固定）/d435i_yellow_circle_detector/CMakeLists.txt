cmake_minimum_required(VERSION 3.0.2)
project(d435i_yellow_circle_detector)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  message_generation
)

add_message_files(
  FILES
  YellowCircle.msg
  YoloDetection.msg
  YoloDetections.msg
  MissionTarget.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
    message_runtime
)

catkin_install_python(PROGRAMS
  scripts/d435i_camera_node.py
  scripts/yellow_binarizer_node.py
  scripts/yellow_circle_detector_node.py
  scripts/annotated_image_viewer_node.py
  scripts/binary_image_viewer_node.py
  scripts/dataset_image_recorder_node.py
  scripts/yolo_detector_node.py
  scripts/competition_task_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
