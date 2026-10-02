#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""detector_node.launch.py — cuadc_visionY26 检测节点启动（YOLO26 ONNX + 静态 TF）。

与 cuadc_vision 版本接口完全一致（节点名 detector_node、话题名、消息类型不变），
区别只有：默认模型改为本包 models/best_yolo26.onnx。

用法：
  ros2 launch cuadc_visionY26 detector_node.launch.py
  ros2 launch cuadc_visionY26 detector_node.launch.py model_path:=/abs/path/best_yolo26.onnx
  ros2 launch cuadc_visionY26 detector_node.launch.py auto_start_mavros:=false
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

try:
    from ament_index_python.packages import get_package_prefix
    _DEFAULT_MODEL = os.path.join(
        get_package_prefix("cuadc_visionY26"), "lib", "models", "best_yolo26.onnx"
    )
except Exception:
    # 包尚未安装时留空，节点会退回自己的默认路径 ../models/best_yolo26.onnx
    _DEFAULT_MODEL = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "models", "best_yolo26.onnx"
    )


def generate_launch_description():
    args = [
        DeclareLaunchArgument("stub",           default_value="false",
                              description="stub_detector=true: 仿真桩检测（无权重）"),
        DeclareLaunchArgument("auto_start_mavros", default_value="false",
                              description="true=节点内启动 MAVROS（默认 false：请用桌面按钮先起 MAVROS）"),
        DeclareLaunchArgument("auto_start_camera", default_value="true",
                              description="true=节点内拉起 headless camera_node"),
        DeclareLaunchArgument("camera_source",  default_value="realsense",
                              description="相机源：realsense（真机）/ gazebo（仿真）"),
        DeclareLaunchArgument("publish_geo",     default_value="true",
                              description="true=发布 GeoTarget"),
        DeclareLaunchArgument("conf",            default_value="0.50",
                              description="置信度阈值 (conf_threshold)"),
        DeclareLaunchArgument("device",          default_value="cpu",
                              description="推理设备（CPU 用 cpu）"),
        DeclareLaunchArgument("imgsz",           default_value="640",
                              description="推理图像尺寸（ONNX 静态输入会被自动覆盖）"),
        DeclareLaunchArgument("model_path",      default_value=_DEFAULT_MODEL,
                              description="YOLO26 ONNX 权重路径（默认为本包 models/best_yolo26.onnx）"),
        DeclareLaunchArgument("show_window",     default_value="true",
                              description="true=弹出带 UI 的识别画面；false=后台无画面"),
        DeclareLaunchArgument("window_name",     default_value="YOLO26 Detection",
                              description="识别 UI 窗口标题"),
    ]

    stub        = LaunchConfiguration("stub")
    auto_mavros = LaunchConfiguration("auto_start_mavros")
    auto_camera = LaunchConfiguration("auto_start_camera")
    camera_source = LaunchConfiguration("camera_source")
    publish_geo = LaunchConfiguration("publish_geo")
    conf        = LaunchConfiguration("conf")
    device      = LaunchConfiguration("device")
    imgsz       = LaunchConfiguration("imgsz")
    model_path  = LaunchConfiguration("model_path")
    show_window = LaunchConfiguration("show_window")
    window_name = LaunchConfiguration("window_name")

    # --- 1) 静态 TF: base_link → d435i_color_optical_frame（与 cuadc_vision 相同）---
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.0", "--y", "0.0", "--z", "0.40",
            "--yaw", "1.57079632679",  # +90°
            "--frame-id", "base_link",
            "--child-frame-id", "d435i_color_optical_frame",
        ],
        output="screen",
        name="static_tf_d435i",
    )

    # --- 2) detector_node.py（节点名保持 detector_node，控制脚本据此判断是否在跑）---
    detector = Node(
        package="cuadc_visionY26",
        executable="detector_node.py",
        output="screen",
        name="detector_node",
        parameters=[{
            "stub_detector":      stub,
            "auto_start_mavros":  auto_mavros,
            "auto_start_camera":  auto_camera,
            "camera_source":      camera_source,
            "publish_geo_target": publish_geo,
            "conf_threshold":     conf,
            "device":             device,
            "imgsz":              imgsz,
            "model_path":         model_path,
            "show_window":        show_window,
            "window_name":        window_name,
        }],
    )

    detector_delayed = TimerAction(period=1.5, actions=[detector])

    return LaunchDescription(args + [static_tf, detector_delayed])
