# Install script for directory: /home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_control

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_control/catkin_generated/installspace/cuadc_control.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cuadc_control/cmake" TYPE FILE FILES
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_control/catkin_generated/installspace/cuadc_controlConfig.cmake"
    "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_control/catkin_generated/installspace/cuadc_controlConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cuadc_control" TYPE FILE FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_control/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_control" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_control/catkin_generated/installspace/one_key_takeoff.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_control" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_control/catkin_generated/installspace/one_key_takeoff_forward_land.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_control" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_control/catkin_generated/installspace/one_key_takeoff_wgs84_forward_rtl.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cuadc_control" TYPE PROGRAM FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/build_lsy/cuadc_control/catkin_generated/installspace/semi_auto_drop_test.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cuadc_control" TYPE DIRECTORY FILES "/home/lab/GDPI_CUADC_2026/代码/LSY/catkin_ws/src/cuadc_control/launch/")
endif()

