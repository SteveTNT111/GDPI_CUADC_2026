#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class YellowBinarizerNode:
    def __init__(self):
        color_topic = rospy.get_param("~color_topic", "/d435i/color/image_raw")
        binary_topic = rospy.get_param("~binary_topic", "/yellow_circle/binary_image")

        h_min = int(rospy.get_param("~h_min", 18))
        h_max = int(rospy.get_param("~h_max", 42))
        s_min = int(rospy.get_param("~s_min", 80))
        s_max = int(rospy.get_param("~s_max", 255))
        v_min = int(rospy.get_param("~v_min", 80))
        v_max = int(rospy.get_param("~v_max", 255))

        self.lower_yellow = np.array([h_min, s_min, v_min], dtype=np.uint8)
        self.upper_yellow = np.array([h_max, s_max, v_max], dtype=np.uint8)
        self.morph_kernel = self.make_odd(int(rospy.get_param("~morph_kernel", 5)), minimum=1)
        self.morph_open_iterations = int(rospy.get_param("~morph_open_iterations", 1))
        self.morph_close_iterations = int(rospy.get_param("~morph_close_iterations", 2))

        self.bridge = CvBridge()
        self.binary_pub = rospy.Publisher(binary_topic, Image, queue_size=1)
        self.color_sub = rospy.Subscriber(
            color_topic,
            Image,
            self.color_callback,
            queue_size=1,
            buff_size=2**24,
        )

        rospy.loginfo(
            "Yellow binarizer started. HSV lower=%s upper=%s",
            self.lower_yellow.tolist(),
            self.upper_yellow.tolist(),
        )

    def color_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Color conversion failed in binarizer: %s", exc)
            return

        binary_mask = self.make_binary_mask(image)
        binary_msg = self._make_img_msg(binary_mask, "mono8")
        binary_msg.header = msg.header
        self.binary_pub.publish(binary_msg)

    @staticmethod
    def _make_img_msg(array, encoding):
        if not array.flags["C_CONTIGUOUS"]:
            array = np.ascontiguousarray(array)
        msg = Image()
        msg.height = array.shape[0]
        msg.width = array.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = int(array.shape[1] * array.dtype.itemsize)
        msg.data = array.tobytes()
        return msg

    def make_binary_mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

        kernel = np.ones((self.morph_kernel, self.morph_kernel), np.uint8)
        if self.morph_open_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=self.morph_open_iterations)
        if self.morph_close_iterations > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=self.morph_close_iterations)

        return mask

    def make_odd(self, value, minimum):
        value = max(int(value), minimum)
        if value % 2 == 0:
            value += 1
        return value


def main():
    rospy.init_node("yellow_binarizer_node")
    YellowBinarizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
