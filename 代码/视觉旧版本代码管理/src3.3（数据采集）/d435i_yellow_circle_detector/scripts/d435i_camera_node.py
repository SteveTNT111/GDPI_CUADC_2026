#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


COLOR_FORMATS = {
    "bgr8": rs.format.bgr8,
    "rgb8": rs.format.rgb8,
    "yuyv": rs.format.yuyv,
    "y8": rs.format.y8,
}

COLOR_FORMAT_NAMES = {
    rs.format.bgr8: "bgr8",
    rs.format.rgb8: "rgb8",
    rs.format.yuyv: "yuyv",
    rs.format.y8: "y8",
}

COLOR_BYTES_PER_PIXEL = {
    "bgr8": 3,
    "rgb8": 3,
    "yuyv": 2,
    "y8": 1,
}


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def make_profile(color_width, color_height, depth_width, depth_height, fps, color_format):
    return {
        "color_width": int(color_width),
        "color_height": int(color_height),
        "depth_width": int(depth_width),
        "depth_height": int(depth_height),
        "fps": int(fps),
        "color_format": color_format,
    }


class D435iCameraNode:
    def __init__(self):
        color_width = int(rospy.get_param("~color_width", 640))
        color_height = int(rospy.get_param("~color_height", 480))
        depth_width = int(rospy.get_param("~depth_width", 640))
        depth_height = int(rospy.get_param("~depth_height", 480))
        fps = int(rospy.get_param("~fps", 30))
        color_format = rospy.get_param("~color_format", "bgr8").strip().lower()
        if color_format not in COLOR_FORMATS:
            rospy.logwarn("Unsupported color_format=%s. Falling back to bgr8.", color_format)
            color_format = "bgr8"

        self.auto_profile_fallback = get_bool_param("~auto_profile_fallback", True)
        self.wait_timeout_ms = int(rospy.get_param("~wait_timeout_ms", 5000))
        self.timeout_restart_count = int(rospy.get_param("~timeout_restart_count", 3))
        self.align_depth_to_color = get_bool_param("~align_depth_to_color", True)
        self.frame_id = rospy.get_param("~frame_id", "d435i_color_optical_frame")
        color_topic = rospy.get_param("~color_topic", "/d435i/color/image_raw")
        depth_topic = rospy.get_param("~depth_topic", "/d435i/aligned_depth/image_raw")

        self.bridge = CvBridge()
        self.color_pub = rospy.Publisher(color_topic, Image, queue_size=1)
        self.depth_pub = rospy.Publisher(depth_topic, Image, queue_size=1)

        self.pipeline = rs.pipeline()
        self.profile_candidates = self.build_profile_candidates(
            color_width,
            color_height,
            depth_width,
            depth_height,
            fps,
            color_format,
        )
        self.profile_index = 0
        self.active_profile = self.profile_candidates[self.profile_index]
        self.align = rs.align(rs.stream.color) if self.align_depth_to_color else None
        self.depth_scale = None
        self.consecutive_timeouts = 0
        self.pipeline_started = False
        self.device_logged = False
        self.shutting_down = False

    def build_profile_candidates(self, color_width, color_height, depth_width, depth_height, fps, color_format):
        requested = make_profile(color_width, color_height, depth_width, depth_height, fps, color_format)
        candidates = [requested]
        candidates.extend(self.enumerate_supported_profiles(max_requested_fps=fps))

        unique = []
        seen = set()
        for profile in candidates:
            key = tuple(profile.items())
            if key not in seen:
                unique.append(profile)
                seen.add(key)
        return unique

    def enumerate_supported_profiles(self, max_requested_fps):
        try:
            devices = rs.context().query_devices()
        except Exception as exc:
            rospy.logwarn("Could not query RealSense devices before start: %s", exc)
            return []

        if len(devices) == 0:
            rospy.logwarn("No RealSense device found while enumerating stream profiles.")
            return []

        color_profiles = []
        depth_profiles = []
        device = devices[0]
        usb_type = self.get_device_info(device, rs.camera_info.usb_type_descriptor)
        prefer_native_bgr = str(usb_type).startswith("3")
        for sensor in device.query_sensors():
            for stream_profile in sensor.get_stream_profiles():
                try:
                    video_profile = stream_profile.as_video_stream_profile()
                except Exception:
                    continue

                stream_type = stream_profile.stream_type()
                stream_format = stream_profile.format()
                profile = {
                    "width": int(video_profile.width()),
                    "height": int(video_profile.height()),
                    "fps": int(stream_profile.fps()),
                    "format": stream_format,
                }

                if stream_type == rs.stream.color and stream_format in COLOR_FORMAT_NAMES:
                    color_profiles.append(profile)
                elif stream_type == rs.stream.depth and stream_format == rs.format.z16:
                    depth_profiles.append(profile)

        candidates = []
        max_fps = max(6, int(max_requested_fps))
        for color in color_profiles:
            for depth in depth_profiles:
                if color["fps"] != depth["fps"]:
                    continue
                if color["fps"] > max_fps:
                    continue

                color_format = COLOR_FORMAT_NAMES[color["format"]]
                candidates.append(
                    make_profile(
                        color["width"],
                        color["height"],
                        depth["width"],
                        depth["height"],
                        color["fps"],
                        color_format,
                    )
                )

        def bandwidth_score(profile):
            color_bytes = (
                profile["color_width"]
                * profile["color_height"]
                * COLOR_BYTES_PER_PIXEL[profile["color_format"]]
            )
            depth_bytes = profile["depth_width"] * profile["depth_height"] * 2
            if prefer_native_bgr:
                format_order = {"bgr8": 0, "rgb8": 1, "yuyv": 2, "y8": 3}
            else:
                format_order = {"yuyv": 0, "bgr8": 1, "rgb8": 2, "y8": 3}
            format_penalty = format_order.get(profile["color_format"], 9) * 1000000
            return (profile["fps"], color_bytes + depth_bytes + format_penalty)

        candidates.sort(key=bandwidth_score)
        rospy.loginfo("Found %d supported low-bandwidth RealSense profile candidates.", len(candidates))
        return candidates

    def make_config(self, profile):
        config = rs.config()
        config.enable_stream(
            rs.stream.color,
            profile["color_width"],
            profile["color_height"],
            COLOR_FORMATS[profile["color_format"]],
            profile["fps"],
        )
        config.enable_stream(
            rs.stream.depth,
            profile["depth_width"],
            profile["depth_height"],
            rs.format.z16,
            profile["fps"],
        )
        return config

    def get_device_info(self, device, info_type):
        try:
            return device.get_info(info_type)
        except Exception:
            return "unknown"

    def log_device_info(self, device):
        if self.device_logged:
            return

        get_info = lambda info_type: self.get_device_info(device, info_type)
        usb_type = get_info(rs.camera_info.usb_type_descriptor)
        rospy.loginfo(
            "RealSense device: name=%s serial=%s firmware=%s usb=%s",
            get_info(rs.camera_info.name),
            get_info(rs.camera_info.serial_number),
            get_info(rs.camera_info.firmware_version),
            usb_type,
        )
        if not str(usb_type).startswith("3"):
            rospy.logwarn(
                "RealSense is connected as USB %s. D435/D435i color+depth streaming is unreliable on USB 2.x; "
                "this node will try low-bandwidth profiles, but USB 3.x is strongly recommended.",
                usb_type,
            )
        self.device_logged = True

    def log_actual_stream_profiles(self, pipeline_profile):
        try:
            color_profile = pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile()
            depth_profile = pipeline_profile.get_stream(rs.stream.depth).as_video_stream_profile()
            rospy.loginfo(
                "Actual streams: color=%dx%d %s@%d depth=%dx%d %s@%d",
                color_profile.width(),
                color_profile.height(),
                COLOR_FORMAT_NAMES.get(color_profile.format(), str(color_profile.format())),
                color_profile.fps(),
                depth_profile.width(),
                depth_profile.height(),
                COLOR_FORMAT_NAMES.get(depth_profile.format(), str(depth_profile.format())),
                depth_profile.fps(),
            )
        except Exception as exc:
            rospy.logwarn("Could not read actual RealSense stream profiles: %s", exc)

    def convert_color_frame_to_bgr(self, color_frame):
        frame_profile = color_frame.get_profile().as_video_stream_profile()
        width = int(frame_profile.width())
        height = int(frame_profile.height())
        frame_format = frame_profile.format()
        image = np.asanyarray(color_frame.get_data())

        if frame_format == rs.format.bgr8:
            if image.ndim == 3:
                return image
            return image.reshape((height, width, 3))

        if frame_format == rs.format.rgb8:
            if image.ndim == 1:
                image = image.reshape((height, width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if frame_format == rs.format.yuyv:
            flat = image.reshape(-1)
            expected_size = height * width * 2
            if flat.size == expected_size:
                yuyv = flat.reshape((height, width, 2))
                return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)

            rospy.logwarn_throttle(
                2.0,
                "YUYV frame has unexpected size=%d, expected=%d. Publishing grayscale fallback.",
                flat.size,
                expected_size,
            )
            gray = image.reshape((height, width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if frame_format == rs.format.y8:
            gray = image.reshape((height, width))
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        raise RuntimeError("Unsupported RealSense color frame format: %s" % frame_format)

    def start(self):
        config = self.make_config(self.active_profile)
        try:
            profile = self.pipeline.start(config)
        except Exception as exc:
            if self.auto_profile_fallback and self.try_next_profile("pipeline start failed: %s" % exc):
                self.pipeline = rs.pipeline()
                self.start()
                return
            rospy.logerr("Failed to start RealSense pipeline: %s", exc)
            sys.exit(1)

        device = profile.get_device()
        self.log_device_info(device)
        self.log_actual_stream_profiles(profile)
        depth_sensor = device.first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        self.pipeline_started = True
        self.consecutive_timeouts = 0
        rospy.loginfo(
            "D435i started. profile=%d/%d color=%dx%d %s depth=%dx%d z16 fps=%d depth_scale=%.8f m/unit wait_timeout=%d ms",
            self.profile_index + 1,
            len(self.profile_candidates),
            self.active_profile["color_width"],
            self.active_profile["color_height"],
            self.active_profile["color_format"],
            self.active_profile["depth_width"],
            self.active_profile["depth_height"],
            self.active_profile["fps"],
            self.depth_scale,
            self.wait_timeout_ms,
        )

    def try_next_profile(self, reason):
        if not self.auto_profile_fallback:
            return False
        if self.profile_index + 1 >= len(self.profile_candidates):
            return False

        self.profile_index += 1
        self.active_profile = self.profile_candidates[self.profile_index]
        rospy.logwarn(
            "Switching RealSense profile after %s. Next profile=%d/%d color=%dx%d %s depth=%dx%d fps=%d",
            reason,
            self.profile_index + 1,
            len(self.profile_candidates),
            self.active_profile["color_width"],
            self.active_profile["color_height"],
            self.active_profile["color_format"],
            self.active_profile["depth_width"],
            self.active_profile["depth_height"],
            self.active_profile["fps"],
        )
        return True

    def restart_pipeline(self):
        if self.shutting_down or rospy.is_shutdown():
            return

        rospy.logwarn("Restarting RealSense pipeline after repeated frame timeouts.")
        try:
            if self.pipeline_started:
                self.pipeline.stop()
        except Exception as exc:
            rospy.logwarn("Ignoring RealSense stop error during restart: %s", exc)

        rospy.sleep(1.0)
        self.pipeline_started = False
        self.pipeline = rs.pipeline()
        self.start()

    def spin(self):
        while not rospy.is_shutdown() and not self.shutting_down:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=self.wait_timeout_ms)
                if self.shutting_down or rospy.is_shutdown():
                    break

                if self.align is not None:
                    frames = self.align.process(frames)

                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    rospy.logwarn_throttle(2.0, "Waiting for both color and depth frames.")
                    rospy.sleep(0.01)
                    continue

                self.consecutive_timeouts = 0

                color_image = self.convert_color_frame_to_bgr(color_frame)
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth_m = depth_raw.astype(np.float32) * self.depth_scale

                stamp = rospy.Time.now()
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                depth_msg = self.bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
                color_msg.header.stamp = stamp
                depth_msg.header.stamp = stamp
                color_msg.header.frame_id = self.frame_id
                depth_msg.header.frame_id = self.frame_id

                self.color_pub.publish(color_msg)
                self.depth_pub.publish(depth_msg)
            except RuntimeError as exc:
                if self.shutting_down or rospy.is_shutdown():
                    break

                self.consecutive_timeouts += 1
                rospy.logwarn(
                    "RealSense frame read failed (%d/%d): %s",
                    self.consecutive_timeouts,
                    self.timeout_restart_count,
                    exc,
                )
                if self.consecutive_timeouts >= self.timeout_restart_count and not rospy.is_shutdown():
                    if self.try_next_profile("repeated frame timeouts"):
                        self.restart_pipeline()
                    else:
                        self.restart_pipeline()
            except Exception as exc:
                if self.shutting_down or rospy.is_shutdown():
                    break
                rospy.logerr_throttle(2.0, "Unexpected D435i error: %s", exc)

    def stop(self):
        self.shutting_down = True
        if self.pipeline_started:
            try:
                self.pipeline.stop()
            except Exception as exc:
                rospy.logwarn("Ignoring RealSense stop error during shutdown: %s", exc)
            finally:
                self.pipeline_started = False
        cv2.destroyAllWindows()


def main():
    rospy.init_node("d435i_camera_node")
    node = D435iCameraNode()
    node.start()
    rospy.on_shutdown(node.stop)
    node.spin()


if __name__ == "__main__":
    main()
