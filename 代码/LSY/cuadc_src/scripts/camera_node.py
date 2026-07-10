#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import cv2
import numpy as np
import pyrealsense2 as rs
import rospy
from sensor_msgs.msg import CameraInfo, Image


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

COLOR_BYTES_PER_PIXEL = {"bgr8": 3, "rgb8": 3, "yuyv": 2, "y8": 1}


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class CameraNode:
    def __init__(self):
        fps = int(rospy.get_param("~fps", 30))
        color_format = rospy.get_param("~color_format", "bgr8").strip().lower()
        if color_format not in COLOR_FORMATS:
            rospy.logwarn("Unsupported color_format=%s. Falling back to bgr8.", color_format)
            color_format = "bgr8"

        self.wait_timeout_ms = int(rospy.get_param("~wait_timeout_ms", 5000))
        self.timeout_restart_count = int(rospy.get_param("~timeout_restart_count", 3))
        self.auto_profile_fallback = get_bool_param("~auto_profile_fallback", True)
        self.align_depth_to_color = get_bool_param("~align_depth_to_color", True)
        self.frame_id = rospy.get_param("~frame_id", "d435i_color_optical_frame")

        self.color_pub = rospy.Publisher("/vision/color/image_raw", Image, queue_size=1)
        self.depth_pub = rospy.Publisher("/vision/aligned_depth/image_raw", Image, queue_size=1)
        self.camera_info_pub = rospy.Publisher("/vision/color/camera_info", CameraInfo, queue_size=1)

        color_width = int(rospy.get_param("~color_width", 640))
        color_height = int(rospy.get_param("~color_height", 480))
        depth_width = int(rospy.get_param("~depth_width", 640))
        depth_height = int(rospy.get_param("~depth_height", 480))

        self.pipeline = rs.pipeline()
        self.profile_candidates = self.build_profile_candidates(
            color_width, color_height, depth_width, depth_height, fps, color_format)
        self.profile_index = 0
        self.active_profile = self.profile_candidates[self.profile_index]
        self.align = rs.align(rs.stream.color) if self.align_depth_to_color else None
        self.depth_scale = None
        self.camera_info_template = None
        self.consecutive_timeouts = 0
        self.pipeline_started = False
        self.device_logged = False
        self.shutting_down = False

    def build_profile_candidates(self, cw, ch, dw, dh, fps, cf):
        def make_profile(cw, ch, dw, dh, fps, cf):
            return {"color_width": int(cw), "color_height": int(ch),
                    "depth_width": int(dw), "depth_height": int(dh),
                    "fps": int(fps), "color_format": cf}
        requested = make_profile(cw, ch, dw, dh, fps, cf)
        candidates = [requested]
        candidates.extend(self.enumerate_supported_profiles(max_requested_fps=fps))
        unique, seen = [], set()
        for p in candidates:
            key = tuple(p.items())
            if key not in seen:
                unique.append(p)
                seen.add(key)
        return unique

    def enumerate_supported_profiles(self, max_requested_fps):
        try:
            devices = rs.context().query_devices()
        except Exception as exc:
            rospy.logwarn("Could not query RealSense devices: %s", exc)
            return []
        if len(devices) == 0:
            rospy.logwarn("No RealSense device found.")
            return []
        color_profiles, depth_profiles = [], []
        device = devices[0]
        usb_type = self.get_device_info(device, rs.camera_info.usb_type_descriptor)
        prefer_native_bgr = str(usb_type).startswith("3")
        for sensor in device.query_sensors():
            for sp in sensor.get_stream_profiles():
                try:
                    vp = sp.as_video_stream_profile()
                except Exception:
                    continue
                st, fmt = sp.stream_type(), sp.format()
                p = {"width": int(vp.width()), "height": int(vp.height()),
                     "fps": int(sp.fps()), "format": fmt}
                if st == rs.stream.color and fmt in COLOR_FORMAT_NAMES:
                    color_profiles.append(p)
                elif st == rs.stream.depth and fmt == rs.format.z16:
                    depth_profiles.append(p)

        candidates = []
        max_fps = max(6, int(max_requested_fps))
        for c in color_profiles:
            for d in depth_profiles:
                if c["fps"] != d["fps"] or c["fps"] > max_fps:
                    continue
                candidates.append({
                    "color_width": c["width"], "color_height": c["height"],
                    "depth_width": d["width"], "depth_height": d["height"],
                    "fps": c["fps"], "color_format": COLOR_FORMAT_NAMES[c["format"]],
                })

        def bandwidth_score(p):
            cb = p["color_width"] * p["color_height"] * COLOR_BYTES_PER_PIXEL[p["color_format"]]
            db = p["depth_width"] * p["depth_height"] * 2
            if prefer_native_bgr:
                order = {"bgr8": 0, "rgb8": 1, "yuyv": 2, "y8": 3}
            else:
                order = {"yuyv": 0, "bgr8": 1, "rgb8": 2, "y8": 3}
            return (p["fps"], cb + db + order.get(p["color_format"], 9) * 1000000)

        candidates.sort(key=bandwidth_score)
        rospy.loginfo("Found %d supported profile candidates.", len(candidates))
        return candidates

    def make_config(self, profile):
        config = rs.config()
        config.enable_stream(rs.stream.color, profile["color_width"], profile["color_height"],
                             COLOR_FORMATS[profile["color_format"]], profile["fps"])
        config.enable_stream(rs.stream.depth, profile["depth_width"], profile["depth_height"],
                             rs.format.z16, profile["fps"])
        return config

    def get_device_info(self, device, info_type):
        try:
            return device.get_info(info_type)
        except Exception:
            return "unknown"

    def log_device_info(self, device):
        if self.device_logged:
            return
        get_info = lambda t: self.get_device_info(device, t)
        usb_type = get_info(rs.camera_info.usb_type_descriptor)
        rospy.loginfo("RealSense: name=%s serial=%s firmware=%s usb=%s",
                       get_info(rs.camera_info.name), get_info(rs.camera_info.serial_number),
                       get_info(rs.camera_info.firmware_version), usb_type)
        if not str(usb_type).startswith("3"):
            rospy.logwarn("RealSense on USB %s. USB 3.x required for color+depth.", usb_type)
        self.device_logged = True

    def build_camera_info(self, pipeline_profile):
        cp = pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = cp.get_intrinsics()
        ci = CameraInfo()
        ci.width, ci.height = int(intr.width), int(intr.height)
        ci.distortion_model = "plumb_bob"
        ci.D = list(intr.coeffs)
        ci.K = [intr.fx, 0.0, intr.ppx, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 1.0]
        ci.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.P = [intr.fx, 0.0, intr.ppx, 0.0, 0.0, intr.fy, intr.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci

    @staticmethod
    def _make_img_msg(array, encoding):
        """Construct a sensor_msgs/Image from a numpy array without cv_bridge.

        cv_bridge on ROS Noetic + OpenCV 4.x has a bug where cvtype_to_name
        is missing keys (e.g. CV_8UC3=16) that encoding_to_cvtype2 produces,
        causing KeyError when encoding "bgr8" is used.
        """
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

    def convert_to_bgr(self, color_frame):
        fp = color_frame.get_profile().as_video_stream_profile()
        w, h, fmt = int(fp.width()), int(fp.height()), fp.format()
        image = np.asanyarray(color_frame.get_data())
        if fmt == rs.format.bgr8:
            return image if image.ndim == 3 else image.reshape((h, w, 3))
        if fmt == rs.format.rgb8:
            if image.ndim == 1:
                image = image.reshape((h, w, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if fmt == rs.format.yuyv:
            flat = image.reshape(-1)
            if flat.size == h * w * 2:
                return cv2.cvtColor(flat.reshape((h, w, 2)), cv2.COLOR_YUV2BGR_YUY2)
            return cv2.cvtColor(image.reshape((h, w)), cv2.COLOR_GRAY2BGR)
        if fmt == rs.format.y8:
            return cv2.cvtColor(image.reshape((h, w)), cv2.COLOR_GRAY2BGR)
        raise RuntimeError("Unsupported format: %s" % fmt)

    def start(self):
        config = self.make_config(self.active_profile)
        try:
            profile = self.pipeline.start(config)
        except Exception as exc:
            if self.auto_profile_fallback and self.try_next_profile("start failed: %s" % exc):
                self.pipeline = rs.pipeline()
                self.start()
                return
            rospy.logerr("Failed to start RealSense: %s", exc)
            sys.exit(1)
        device = profile.get_device()
        self.log_device_info(device)
        self.camera_info_template = self.build_camera_info(profile)
        self.depth_scale = device.first_depth_sensor().get_depth_scale()
        self.pipeline_started = True
        self.consecutive_timeouts = 0
        rospy.loginfo("D435i started. %dx%d %s@%d depth_scale=%.8f",
                       self.active_profile["color_width"], self.active_profile["color_height"],
                       self.active_profile["color_format"], self.active_profile["fps"],
                       self.depth_scale)

    def try_next_profile(self, reason):
        if not self.auto_profile_fallback:
            return False
        if self.profile_index + 1 >= len(self.profile_candidates):
            return False
        self.profile_index += 1
        self.active_profile = self.profile_candidates[self.profile_index]
        rospy.logwarn("Switching profile after %s. New: %dx%d %s@%d", reason,
                       self.active_profile["color_width"], self.active_profile["color_height"],
                       self.active_profile["color_format"], self.active_profile["fps"])
        return True

    def restart_pipeline(self):
        if self.shutting_down or rospy.is_shutdown():
            return
        rospy.logwarn("Restarting RealSense pipeline.")
        try:
            if self.pipeline_started:
                self.pipeline.stop()
        except Exception as exc:
            rospy.logwarn("Stop error during restart: %s", exc)
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
                    continue

                self.consecutive_timeouts = 0
                color_img = self.convert_to_bgr(color_frame)
                depth_raw = np.asanyarray(depth_frame.get_data())
                depth_m = depth_raw.astype(np.float32) * self.depth_scale

                stamp = rospy.Time.now()
                color_msg = self._make_img_msg(color_img, "bgr8")
                depth_msg = self._make_img_msg(depth_m, "32FC1")
                color_msg.header.stamp = stamp
                depth_msg.header.stamp = stamp
                color_msg.header.frame_id = self.frame_id
                depth_msg.header.frame_id = self.frame_id

                if self.camera_info_template is not None:
                    ci = CameraInfo()
                    ci.header.stamp = stamp
                    ci.header.frame_id = self.frame_id
                    ci.width = self.camera_info_template.width
                    ci.height = self.camera_info_template.height
                    ci.distortion_model = self.camera_info_template.distortion_model
                    ci.D = list(self.camera_info_template.D)
                    ci.K = list(self.camera_info_template.K)
                    ci.R = list(self.camera_info_template.R)
                    ci.P = list(self.camera_info_template.P)
                    self.camera_info_pub.publish(ci)

                self.color_pub.publish(color_msg)
                self.depth_pub.publish(depth_msg)
            except RuntimeError as exc:
                if self.shutting_down or rospy.is_shutdown():
                    break
                self.consecutive_timeouts += 1
                rospy.logwarn("Frame read failed (%d/%d): %s",
                               self.consecutive_timeouts, self.timeout_restart_count, exc)
                if self.consecutive_timeouts >= self.timeout_restart_count:
                    self.try_next_profile("repeated timeouts")
                    self.restart_pipeline()
            except Exception as exc:
                if self.shutting_down or rospy.is_shutdown():
                    break
                self.consecutive_timeouts += 1
                rospy.logwarn("Frame processing error (%d/%d): %s",
                               self.consecutive_timeouts, self.timeout_restart_count, exc)
                if self.consecutive_timeouts >= self.timeout_restart_count:
                    self.try_next_profile("repeated errors")
                    self.restart_pipeline()

    def stop(self):
        self.shutting_down = True
        if self.pipeline_started:
            try:
                self.pipeline.stop()
            except Exception:
                pass
            finally:
                self.pipeline_started = False
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


def main():
    rospy.init_node("camera_node")
    node = CameraNode()
    node.start()
    rospy.on_shutdown(node.stop)
    node.spin()


if __name__ == "__main__":
    main()
