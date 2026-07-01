#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from datetime import datetime

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class BinaryVideoRecorderNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/yellow_circle/binary_image")
        self.record_dir = os.path.expanduser(rospy.get_param("~record_dir", "~/yellow_circle_records"))
        self.record_fps = float(rospy.get_param("~record_fps", 30.0))
        self.filename_prefix = rospy.get_param("~filename_prefix", "yellow_circle_binary")
        self.bridge = CvBridge()
        self.writer = None
        self.output_path = None
        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.on_shutdown(self.shutdown)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Binary recorder conversion failed: %s", exc)
            return

        if self.writer is None:
            self.open_writer(image)
            if self.writer is None:
                return

        self.writer.write(image)

    def open_writer(self, image):
        os.makedirs(self.record_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path = os.path.join(self.record_dir, "{}_{}.avi".format(self.filename_prefix, timestamp))
        height, width = image.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        writer = cv2.VideoWriter(self.output_path, fourcc, self.record_fps, (width, height), False)
        if not writer.isOpened():
            rospy.logerr("Failed to open binary video writer: %s", self.output_path)
            return
        self.writer = writer
        rospy.loginfo("Recording binary video to: %s", self.output_path)

    def shutdown(self):
        if self.writer is not None:
            self.writer.release()
            self.writer = None


def main():
    rospy.init_node("binary_video_recorder_node")
    BinaryVideoRecorderNode()
    rospy.spin()


if __name__ == "__main__":
    main()
