#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
D435i 检测系统 launch 文件
同时启动相机驱动节点和黄色圆检测节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        # ---- 可调参数 ----
        DeclareLaunchArgument('fps', default_value='30',
                              description='Camera frame rate'),
        DeclareLaunchArgument('enable_align', default_value='true',
                              description='Align depth to color frame'),
        DeclareLaunchArgument('h_min', default_value='20',
                              description='HSV H lower bound for yellow'),
        DeclareLaunchArgument('h_max', default_value='40',
                              description='HSV H upper bound for yellow'),
        DeclareLaunchArgument('s_min', default_value='80',
                              description='HSV S lower bound'),
        DeclareLaunchArgument('s_max', default_value='255',
                              description='HSV S upper bound'),
        DeclareLaunchArgument('v_min', default_value='80',
                              description='HSV V lower bound'),
        DeclareLaunchArgument('v_max', default_value='255',
                              description='HSV V upper bound'),
        DeclareLaunchArgument('min_area', default_value='200',
                              description='Minimum contour area in pixels'),
        DeclareLaunchArgument('circularity_thresh', default_value='0.6',
                              description='Min circularity (0-1, 1=perfect circle)'),

        # ---- D435i 相机驱动节点 ----
        Node(
            package='d435i_detector',
            executable='d435i_publisher',
            name='d435i_publisher',
            output='screen',
            parameters=[{
                'fps': LaunchConfiguration('fps'),
                'enable_align': LaunchConfiguration('enable_align'),
            }],
        ),

        # ---- 黄色圆检测节点 ----
        Node(
            package='d435i_detector',
            executable='yellow_detector',
            name='yellow_detector',
            output='screen',
            parameters=[{
                'h_min': LaunchConfiguration('h_min'),
                'h_max': LaunchConfiguration('h_max'),
                's_min': LaunchConfiguration('s_min'),
                's_max': LaunchConfiguration('s_max'),
                'v_min': LaunchConfiguration('v_min'),
                'v_max': LaunchConfiguration('v_max'),
                'min_area': LaunchConfiguration('min_area'),
                'circularity_thresh': LaunchConfiguration('circularity_thresh'),
            }],
        ),
    ])
