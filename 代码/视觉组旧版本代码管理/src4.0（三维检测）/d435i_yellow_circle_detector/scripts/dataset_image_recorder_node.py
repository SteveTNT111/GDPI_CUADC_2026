#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import queue
import threading
from datetime import datetime

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class DatasetImageRecorderNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/d435i/color/image_raw")
        self.record_dir = os.path.expanduser(rospy.get_param("~record_dir", "~/yellow_circle_dataset"))
        self.record_fps = float(rospy.get_param("~record_fps", 30.0))
        self.image_format = rospy.get_param("~image_format", "jpg").strip().lower()
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 95))
        self.png_compression = int(rospy.get_param("~png_compression", 3))
        self.filename_prefix = rospy.get_param("~filename_prefix", "d435i")
        self.queue_size = int(rospy.get_param("~queue_size", 120))

        if self.image_format not in ("jpg", "jpeg", "png"):
            rospy.logwarn("Unsupported image_format=%s. Falling back to jpg.", self.image_format)
            self.image_format = "jpg"

        self.bridge = CvBridge()
        self.min_interval = 1.0 / self.record_fps if self.record_fps > 0.0 else 0.0
        self.last_save_time = 0.0
        self.interval_tolerance = 0.002
        self.sequence = 0
        self.saved_count = 0
        self.dropped_count = 0
        self.stopped = False

        session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.record_dir, session_name)
        self.image_dir = os.path.join(self.session_dir, "images")
        os.makedirs(self.image_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.write_queue = queue.Queue(maxsize=self.queue_size)
        self.writer_thread = threading.Thread(target=self.writer_loop)
        self.writer_thread.daemon = True
        self.writer_thread.start()

        self.sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "Dataset image recorder started. topic=%s output=%s fps=%.2f format=%s",
            self.image_topic,
            self.image_dir,
            self.record_fps,
            self.image_format,
        )

    def image_callback(self, msg):
        now = rospy.Time.now().to_sec()
        stamp = msg.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = now

        if (
            self.min_interval > 0.0
            and stamp - self.last_save_time < self.min_interval - self.interval_tolerance
        ):
            return
        self.last_save_time = stamp

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Dataset image conversion failed: %s", exc)
            return

        self.sequence += 1
        filename = "{}_{:06d}_{:.6f}.{}".format(
            self.filename_prefix,
            self.sequence,
            stamp,
            "jpg" if self.image_format == "jpeg" else self.image_format,
        )
        path = os.path.join(self.image_dir, filename)

        try:
            self.write_queue.put_nowait((path, image.copy()))
        except queue.Full:
            self.dropped_count += 1
            rospy.logwarn_throttle(
                2.0,
                "Dataset image queue is full. Dropped frames=%d. Use lower record_fps or faster storage.",
                self.dropped_count,
            )

    def writer_loop(self):
        while not self.stopped or not self.write_queue.empty():
            try:
                path, image = self.write_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            params = []
            if self.image_format in ("jpg", "jpeg"):
                params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            elif self.image_format == "png":
                params = [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression]

            ok = cv2.imwrite(path, image, params)
            if ok:
                self.saved_count += 1
                if self.saved_count % 100 == 0:
                    rospy.loginfo("Saved dataset images: %d", self.saved_count)
            else:
                rospy.logwarn("Failed to save dataset image: %s", path)

            self.write_queue.task_done()

    def shutdown(self):
        self.stopped = True
        self.writer_thread.join(timeout=5.0)
        rospy.loginfo(
            "Dataset image recorder stopped. saved=%d dropped=%d output=%s",
            self.saved_count,
            self.dropped_count,
            self.image_dir,
        )


def main():
    rospy.init_node("dataset_image_recorder_node")
    DatasetImageRecorderNode()
    rospy.spin()


if __name__ == "__main__":
    main()
