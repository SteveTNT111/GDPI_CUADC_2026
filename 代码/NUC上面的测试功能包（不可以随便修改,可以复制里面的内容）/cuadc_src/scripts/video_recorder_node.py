#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
video_recorder_node.py — D435i RGB 视频录制节点

功能：启动即录，Ctrl+C 停止。录制的视频用于 Roboflow RAPID 模式训练 YOLO。

用法：
  roslaunch cuadc_vision video_recorder.launch
"""

import os
from datetime import datetime

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class VideoRecorderNode:
    def __init__(self):
        color_topic = rospy.get_param("~color_topic", "/vision/color/image_raw")
        self.fps = float(rospy.get_param("~fps", 30.0))
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/cuadc_videos"))
        self.codec = rospy.get_param("~codec", "MJPG")
        self.file_prefix = rospy.get_param("~file_prefix", "cuadc_train")
        self.show_window = rospy.get_param("~show_window", True)
        self.window_name = rospy.get_param("~window_name", "CUADC Video Recorder (Ctrl+C to stop)")

        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.writer = None
        self.video_path = None
        self.frame_count = 0
        self.started = False

        self.sub = rospy.Subscriber(
            color_topic, Image, self.image_callback,
            queue_size=1, buff_size=2 ** 24
        )

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Video recorder ready, waiting for images on %s ...", color_topic)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Image conversion failed: %s", exc)
            return

        if not self.started:
            self.start_recording(frame.shape)

        if self.writer is not None:
            self.writer.write(frame)
            self.frame_count += 1

        if self.show_window:
            label = "REC {}  {}x{}  FPS {:.0f}".format(
                self.frame_count, frame.shape[1], frame.shape[0], self.fps)
            disp = frame.copy()
            cv2.putText(disp, label, (8, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imshow(self.window_name, disp)
            cv2.waitKey(1)

    def start_recording(self, shape):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = "{}_{}.avi".format(self.file_prefix, stamp)
        self.video_path = os.path.join(self.output_dir, filename)
        h, w = shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec[:4])
        self.writer = cv2.VideoWriter(self.video_path, fourcc, self.fps, (w, h))
        if not self.writer.isOpened():
            rospy.logerr("Failed to open video writer: %s", self.video_path)
            self.writer = None
            return
        self.started = True
        rospy.loginfo("Recording started: %s (%dx%d @ %.0f FPS)",
                      self.video_path, w, h, self.fps)

    def shutdown(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.show_window:
            cv2.destroyWindow(self.window_name)
        if self.video_path:
            rospy.loginfo("Recording stopped. frames=%d → %s", self.frame_count, self.video_path)


def main():
    rospy.init_node("video_recorder_node")
    VideoRecorderNode()
    rospy.spin()


if __name__ == "__main__":
    main()
