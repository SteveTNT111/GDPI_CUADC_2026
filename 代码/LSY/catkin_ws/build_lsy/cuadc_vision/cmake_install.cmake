# Install script for directory: /home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cuadc_vision/msg" TYPE FILE FILES
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision/msg/YoloDetection.msg"
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision/msg/YoloDetections.msg"
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision/msg/GeoTarget.msg"
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision/msg/BucketInfo.msg"
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision/msg/BucketAimInfo.msg"
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision/msg/MissionStatus.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cuadc_vision/cmake" TYPE FILE FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/cuadc_vision-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/devel_lsy/include/cuadc_vision")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/devel_lsy/share/roseus/ros/cuadc_vision")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/devel_lsy/share/common-lisp/ros/cuadc_vision")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/devel_lsy/share/gennodejs/ros/cuadc_vision")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/devel_lsy/lib/python3/dist-packages/cuadc_vision")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/devel_lsy/lib/python3/dist-packages/cuadc_vision")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/cuadc_vision.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cuadc_vision/cmake" TYPE FILE FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/cuadc_vision-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cuadc_vision/cmake" TYPE FILE FILES
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/cuadc_visionConfig.cmake"
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/cuadc_visionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cuadc_vision" TYPE FILE FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/camera_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/detector_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/main.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/servo_test.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/flight_data_video_recorder_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/auto_drop_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/video_recorder_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/competition_mission_common.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/competition_main.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_vision" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_vision/catkin_generated/installspace/single_bucket_aim_test.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cuadc_vision" TYPE DIRECTORY FILES
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision/launch/"
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_vision/config/"
    )
endif()

