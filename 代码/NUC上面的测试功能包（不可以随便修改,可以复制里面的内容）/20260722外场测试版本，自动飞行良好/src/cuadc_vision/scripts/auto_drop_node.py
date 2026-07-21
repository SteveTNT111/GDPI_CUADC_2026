#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
auto_drop_node.py — 自动抛投触发节点

功能：
  - 订阅 `/vision/yolo/detection` (YoloDetection)，以及相机的 `CameraInfo`。
  - 计算检测中心相对于相机光心的像素差值，当小于阈值时通过 `/mavros/cmd/command` 触发舵机抛投（MAV_CMD_DO_SET_SERVO）。
  - 可配置参数: 舵机通道/释放PWM/复位PWM/阈值/最小置信度/冷却时间等。

注意：不修改已有文件，仅新增节点脚本。
"""

import time
import threading

import rospy
from sensor_msgs.msg import CameraInfo
from cuadc_vision.msg import YoloDetection
from mavros_msgs.srv import CommandLong


class AutoDropNode:
    def __init__(self):
        rospy.init_node("auto_drop_node", anonymous=False)

        # 参数
        self.channel = int(rospy.get_param("~channel", 9))
        self.release_pwm = int(rospy.get_param("~release_pwm", 1900))
        self.reset_pwm = int(rospy.get_param("~reset_pwm", 1100))
        self.hold_seconds = float(rospy.get_param("~hold_seconds", 0.8))
        self.pixel_threshold = float(rospy.get_param("~pixel_threshold", 20.0))
        self.min_conf = float(rospy.get_param("~min_conf", 0.6))
        self.cooldown = float(rospy.get_param("~cooldown", 2.0))

        self.camera_info = None
        self.camera_info_lock = threading.Lock()

        self.last_drop_time = 0.0
        self.triggered = False

        # 订阅
        rospy.Subscriber("/vision/color/camera_info", CameraInfo, self.camera_info_cb, queue_size=1)
        rospy.Subscriber("/vision/yolo/detection", YoloDetection, self.detection_cb, queue_size=1)

        rospy.loginfo("auto_drop_node ready: channel=%d pwm_rel=%d pwm_rst=%d threshold=%.1fpx min_conf=%.2f",
                      self.channel, self.release_pwm, self.reset_pwm, self.pixel_threshold, self.min_conf)

    def camera_info_cb(self, msg: CameraInfo):
        with self.camera_info_lock:
            self.camera_info = msg

    def get_camera_center(self):
        with self.camera_info_lock:
            ci = self.camera_info
        if ci is None:
            return None
        cx = float(ci.K[2])
        cy = float(ci.K[5])
        return (cx, cy)

    def detection_cb(self, det: YoloDetection):
        if not det.detected:
            return
        if det.confidence < self.min_conf:
            return

        center = self.get_camera_center()
        if center is None:
            rospy.logdebug("camera_info not ready, skipping detection")
            return

        cx, cy = center
        dx = float(det.center_x) - cx
        dy = float(det.center_y) - cy
        dist = (dx * dx + dy * dy) ** 0.5

        rospy.loginfo_throttle(2.0, "detected center offset: dx=%.1f dy=%.1f norm=%.1fpx conf=%.2f", dx, dy, dist, det.confidence)

        now = time.time()
        if dist <= self.pixel_threshold and (now - self.last_drop_time) >= self.cooldown:
            rospy.loginfo("offset %.1f <= threshold %.1f — triggering drop", dist, self.pixel_threshold)
            success = self.call_set_servo(self.channel, self.release_pwm)
            if success:
                time.sleep(self.hold_seconds)
                self.call_set_servo(self.channel, self.reset_pwm)
                self.last_drop_time = time.time()
            else:
                rospy.logwarn("failed to call set_servo service")

    def call_set_servo(self, channel, pwm):
        try:
            rospy.wait_for_service("/mavros/cmd/command", timeout=5.0)
        except Exception as exc:
            rospy.logwarn("service /mavros/cmd/command unavailable: %s", exc)
            return False

        try:
            cmd = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
            res = cmd(broadcast=False,
                      command=183,
                      confirmation=0,
                      param1=float(channel),
                      param2=float(pwm),
                      param3=0,
                      param4=0,
                      param5=0,
                      param6=0,
                      param7=0)
            rospy.loginfo("set_servo result: %s", res)
            return True
        except Exception as exc:
            rospy.logerr("call to /mavros/cmd/command failed: %s", exc)
            return False


def main():
    node = AutoDropNode()
    rospy.spin()


if __name__ == '__main__':
    main()
