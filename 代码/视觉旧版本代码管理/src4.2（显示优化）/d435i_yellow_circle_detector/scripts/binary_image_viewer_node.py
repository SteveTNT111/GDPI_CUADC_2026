#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class BinaryImageViewerNode:
    def __init__(self):
        image_topic = rospy.get_param("~image_topic", "/yellow_circle/binary_image")
        self.window_name = rospy.get_param("~window_name", "yellow_binary")
        self.display_scale = float(rospy.get_param("~display_scale", 1.0))
        self.bridge = CvBridge()
        self.window_ready = False
        self.sub = rospy.Subscriber(
            image_topic,
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
            rospy.logwarn_throttle(2.0, "Binary image conversion failed: %s", exc)
            return

        if not self.window_ready:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            self.window_ready = True

        if self.display_scale > 0.0 and self.display_scale != 1.0:
            image = cv2.resize(image, None, fx=self.display_scale, fy=self.display_scale, interpolation=cv2.INTER_AREA)

        cv2.imshow(self.window_name, image)
        cv2.waitKey(1)

    def shutdown(self):
        if self.window_ready:
            try:
                cv2.destroyWindow(self.window_name)
            except cv2.error:
                pass


def main():
    rospy.init_node("binary_image_viewer_node")
    BinaryImageViewerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
