#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""camera_node.launch.py — cuadc_visionY26 相机节点启动（D435 / realsense）。

cuadc_control 在 detector_package 下找不到 camera_node 时会用
`ros2 launch <detector_package> camera_node.launch.py`，故本包提供该 launch。

用法：
  ros2 launch cuadc_visionY26 camera_node.launch.py
  ros2 launch cuadc_visionY26 camera_node.launch.py source:=realsense show_rgb:=true
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument("source",   default_value="realsense",
                              description="图像源：realsense（真机）/ gazebo（仿真）"),
        DeclareLaunchArgument("show_rgb", default_value="false",
                              description="true=弹出纯 RGB 原图窗口"),
        DeclareLaunchArgument("fps",      default_value="30",
                              description="相机帧率"),
    ]

    cam = Node(
        package="cuadc_visionY26",
        executable="camera_node.py",
        output="screen",
        name="camera_node",
        parameters=[{
            "source":   LaunchConfiguration("source"),
            "show_rgb": LaunchConfiguration("show_rgb"),
            "fps":      LaunchConfiguration("fps"),
        }],
    )

    return LaunchDescription(args + [cam])
