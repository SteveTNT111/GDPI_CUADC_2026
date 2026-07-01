#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
D435i 深度相机驱动节点 (ROS Noetic)
- 使用 pyrealsense2 读取彩色图像 + 深度数据
- 彩色图像发布到 /d435i/color_image  (sensor_msgs/Image)
- 深度图像发布到 /d435i/depth_image  (sensor_msgs/Image)
- 相机内参发布到 /d435i/camera_info  (sensor_msgs/CameraInfo)
"""

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class D435iPublisher:
    """D435i 相机驱动发布节点"""

    def __init__(self):
        # ---- 从参数服务器读取参数 ----
        self.fps           = rospy.get_param('~fps', 30)
        self.color_width   = rospy.get_param('~color_width', 640)
        self.color_height  = rospy.get_param('~color_height', 480)
        self.depth_width   = rospy.get_param('~depth_width', 640)
        self.depth_height  = rospy.get_param('~depth_height', 480)
        self.enable_align  = rospy.get_param('~enable_align', True)

        # ---- RealSense 管线初始化 ----
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(
            rs.stream.color,
            self.color_width, self.color_height,
            rs.format.bgr8, self.fps)
        self.config.enable_stream(
            rs.stream.depth,
            self.depth_width, self.depth_height,
            rs.format.z16, self.fps)

        self.profile = self.pipeline.start(self.config)

        # 对齐对象：深度→彩色
        self.align = rs.align(rs.stream.color) if self.enable_align else None

        # 深度尺度（将 z16 像素值转为米）
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        rospy.loginfo('Depth scale: %.6f m/unit' % self.depth_scale)

        # ---- ROS 发布者 ----
        self.color_pub = rospy.Publisher('/d435i/color_image', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/d435i/depth_image', Image, queue_size=10)
        self.info_pub  = rospy.Publisher('/d435i/camera_info', CameraInfo, queue_size=10)

        self.cv_bridge = CvBridge()

        # ---- 定时器 ----
        timer_period = 1.0 / self.fps
        self.timer = rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        rospy.loginfo('D435i publisher started')

    # ------------------------------------------------------------------
    def _build_camera_info(self, color_intrinsics):
        """从 RealSense 内参对象构造 CameraInfo 消息"""
        info = CameraInfo()
        info.width  = color_intrinsics.width
        info.height = color_intrinsics.height
        info.K = list(color_intrinsics.intrinsics_matrix())  # 3x3 行优先 → 9 元素
        coeffs = color_intrinsics.coeffs
        info.D = [float(c) for c in coeffs]                  # 畸变系数列表
        info.distortion_model = 'plumb_bob'
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [
            info.K[0], info.K[1], info.K[2], 0.0,
            info.K[3], info.K[4], info.K[5], 0.0,
            info.K[6], info.K[7], info.K[8], 0.0,
        ]
        return info

    # ------------------------------------------------------------------
    def timer_callback(self, event):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except Exception as e:
            rospy.logwarn('wait_for_frames failed: %s' % str(e))
            return

        # 对齐深度到彩色坐标系
        if self.align is not None:
            frames = self.align.process(frames)

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        now = rospy.Time.now()

        # ---- 彩色图像 ----
        color_img = np.asanyarray(color_frame.get_data())  # HxWx3 BGR uint8
        color_msg = self.cv_bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
        color_msg.header.stamp    = now
        color_msg.header.frame_id = 'd435i_color_optical_frame'
        self.color_pub.publish(color_msg)

        # ---- 深度图像 ----
        depth_img = np.asanyarray(depth_frame.get_data())  # HxW uint16
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
        depth_msg.header.stamp    = now
        depth_msg.header.frame_id = 'd435i_depth_optical_frame'
        self.depth_pub.publish(depth_msg)

        # ---- 相机内参 ----
        color_intrinsics = color_frame.profile.as_video_stream_profile().get_intrinsics()
        info_msg = self._build_camera_info(color_intrinsics)
        info_msg.header.stamp    = now
        info_msg.header.frame_id = 'd435i_color_optical_frame'
        self.info_pub.publish(info_msg)

    # ------------------------------------------------------------------
    def shutdown(self):
        self.pipeline.stop()
        self.timer.shutdown()


def main():
    rospy.init_node('d435i_publisher', anonymous=False)
    node = D435iPublisher()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
