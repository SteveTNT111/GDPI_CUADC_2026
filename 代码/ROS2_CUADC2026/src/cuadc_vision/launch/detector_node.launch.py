#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""detector_node.launch.py — B2 检测节点启动（仿真桩 + 静态 TF）。

用法：
  ros2 launch cuadc_vision detector_node.launch.py [args]

仿真（GZ source）：
  ros2 launch cuadc_vision detector_node.launch.py stub:=true \
      auto_start_mavros:=false publish_geo:=false

真机（Realsense + 权重）：
  ros2 launch cuadc_vision detector_node.launch.py stub:=false \
      auto_start_mavros:=true
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument("stub",           default_value="false",
                              description="stub_detector=true: 仿真桩检测（无 YOLO 权重）"),
        DeclareLaunchArgument("auto_start_mavros", default_value="false",
                              description="true=节点内启动 MAVROS（默认 false：请用桌面按钮先起 MAVROS）"),
        DeclareLaunchArgument("publish_geo",     default_value="true",
                              description="true=发布 GeoTarget 到 /vision/geo_target"),
        DeclareLaunchArgument("conf",            default_value="0.50",
                              description="YOLO/confidence 阈值 (conf_threshold)"),
        DeclareLaunchArgument("device",          default_value="cpu",
                              description="推理设备（cpu/cuda）"),
        DeclareLaunchArgument("imgsz",           default_value="640",
                              description="推理图像尺寸"),
        DeclareLaunchArgument("model_path",      default_value="",
                              description="YOLO 权重路径（空=使用默认 models/best.pt）"),
    ]

    stub        = LaunchConfiguration("stub")
    auto_mavros = LaunchConfiguration("auto_start_mavros")
    publish_geo = LaunchConfiguration("publish_geo")
    conf        = LaunchConfiguration("conf")
    device      = LaunchConfiguration("device")
    imgsz       = LaunchConfiguration("imgsz")
    model_path  = LaunchConfiguration("model_path")

    # --- 1) 静态 TF: base_link → d435i_color_optical_frame ---
    # z=0.40, yaw=+90° 为仿真名义外参（对应 camera_link 实际 t=(0.12,0,0.02)+pitch45°）。
    # 真机 D435i 正下安装时改为对应值（或由 tf2 从 URDF 发布）。
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.40",
            "--yaw", "1.57079632679",  # +90°
            "--frame-id", "base_link",
            "--child-frame-id", "d435i_color_optical_frame",
        ],
        output="screen",
        name="static_tf_d435i",
    )

    # --- 2) detector_node.py ---
    # ros2 run 通过 symlink-install 找脚本，launch 同理：script_name 起始。
    detector = Node(
        package="cuadc_vision",
        executable="detector_node.py",
        output="screen",
        name="detector_node",
        parameters=[{
            "stub_detector":      stub,
            "auto_start_mavros":  auto_mavros,
            "publish_geo_target": publish_geo,
            "conf_threshold":     conf,
            "device":             device,
            "imgsz":              imgsz,
            "model_path":         model_path,
        }],
        # 条件发布：仅当 stub=true 时延迟 2s 等 TF ready
    )

    # TF 先于 detector 启动 1.5s，确保 TF tree 已有 base_link→optical
    detector_delayed = TimerAction(period=1.5, actions=[detector])

    return LaunchDescription(args + [static_tf, detector_delayed])
