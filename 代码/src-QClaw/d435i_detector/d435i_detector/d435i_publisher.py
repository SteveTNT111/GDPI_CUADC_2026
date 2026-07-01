#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
D435i 深度相机驱动节点
- 使用 pyrealsense2 读取彩色图像 + 深度数据
- 彩色图像发布到 /d435i/color_image  (sensor_msgs/Image)
- 深度图像发布到 /d435i/depth_image  (sensor_msgs/Image)
- 相机内参发布到 /d435i/camera_info  (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class D435iPublisher(Node):
    """D435i 相机驱动发布节点"""

    def __init__(self):
        super().__init__('d435i_publisher')

        # ---- 声明参数 ----
        self.declare_parameter('fps', 30)
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('enable_align', True)  # 是否将深度对齐到彩色

        fps = self.get_parameter('fps').value
        self.enable_align = self.get_parameter('enable_align').value

        # ---- RealSense 管线初始化 ----
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(
            rs.stream.color,
            self.get_parameter('color_width').value,
            self.get_parameter('color_height').value,
            rs.format.bgr8, fps)
        self.config.enable_stream(
            rs.stream.depth,
            self.get_parameter('depth_width').value,
            self.get_parameter('depth_height').value,
            rs.format.z16, fps)

        # 启动管线
        self.profile = self.pipeline.start(self.config)

        # 对齐对象：深度→彩色
        self.align = rs.align(rs.stream.color) if self.enable_align else None

        # 深度尺度（将 z16 像素值转为米）
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.get_logger().info(f'Depth scale: {self.depth_scale:.6f} m/unit')

        # ---- QoS 配置 ----
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # ---- ROS2 发布者 ----
        self.color_pub = self.create_publisher(Image, '/d435i/color_image', qos)
        self.depth_pub = self.create_publisher(Image, '/d435i/depth_image', qos)
        self.info_pub  = self.create_publisher(CameraInfo, '/d435i/camera_info', qos)

        self.cv_bridge = CvBridge()

        # ---- 定时器：按 fps 周期抓帧发布 ----
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('D435i publisher started')

    # ------------------------------------------------------------------
    def _build_camera_info(self, color_intrinsics):
        """从 RealSense 内参对象构造 CameraInfo 消息"""
        info = CameraInfo()
        info.width = color_intrinsics.width
        info.height = color_intrinsics.height
        info.k = list(color_intrinsics.intrinsics_matrix())  # 3x3 行优先 → 9 元素列表
        # D 畸变系数
        coeffs = color_intrinsics.coeffs
        info.d = [float(c) for c in coeffs]
        info.distortion_model = 'plumb_bob'
        # R (identity) / P (简单拼 K + [0,0,0])
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            info.k[0], info.k[1], info.k[2], 0.0,
            info.k[3], info.k[4], info.k[5], 0.0,
            info.k[6], info.k[7], info.k[8], 0.0,
        ]
        return info

    # ------------------------------------------------------------------
    def timer_callback(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except Exception as e:
            self.get_logger().warn(f'wait_for_frames failed: {e}')
            return

        # 对齐深度到彩色坐标系
        if self.align is not None:
            frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # ---- 彩色图像 ----
        color_img = np.asanyarray(color_frame.get_data())  # HxWx3 BGR uint8
        color_msg = self.cv_bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
        color_msg.header.stamp = self.get_clock().now().to_msg()
        color_msg.header.frame_id = 'd435i_color_optical_frame'
        self.color_pub.publish(color_msg)

        # ---- 深度图像 ----
        depth_img = np.asanyarray(depth_frame.get_data())  # HxW uint16
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
        depth_msg.header.stamp = color_msg.header.stamp
        depth_msg.header.frame_id = 'd435i_depth_optical_frame'
        self.depth_pub.publish(depth_msg)

        # ---- 相机内参 ----
        color_intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
        info_msg = self._build_camera_info(color_intrinsics)
        info_msg.header.stamp = color_msg.header.stamp
        info_msg.header.frame_id = 'd435i_color_optical_frame'
        self.info_pub.publish(info_msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = D435iPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
