#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
import threading
from datetime import datetime

import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState, Image, NavSatFix
from std_msgs.msg import Float64

try:
    from mavros_msgs.msg import State
except Exception:
    State = None


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def finite_or_zero(value):
    try:
        value = float(value)
    except Exception:
        return 0.0
    if not math.isfinite(value):
        return 0.0
    return value


class FlightDataVideoRecorderNode:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/d435i/color/image_raw")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.global_topic = rospy.get_param("~global_position_topic", "/mavros/global_position/global")
        self.local_pose_topic = rospy.get_param("~local_pose_topic", "/mavros/local_position/pose")
        self.rel_alt_topic = rospy.get_param("~rel_alt_topic", "/mavros/global_position/rel_alt")
        self.battery_topic = rospy.get_param("~battery_topic", "/mavros/battery")

        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", os.path.join("~", "cuadc_flight_videos"))
        )
        self.video_fps = float(rospy.get_param("~video_fps", 30.0))
        self.codec = rospy.get_param("~codec", "MJPG")
        self.file_prefix = rospy.get_param("~file_prefix", "cuadc_flight")
        self.overlay_scale = float(rospy.get_param("~overlay_scale", 0.55))
        self.overlay_thickness = int(rospy.get_param("~overlay_thickness", 2))
        self.record_immediately = get_bool_param("~record_immediately", False)
        self.stop_on_disarm = get_bool_param("~stop_on_disarm", True)
        self.write_csv = get_bool_param("~write_csv", True)
        self.show_window = get_bool_param("~show_window", False)
        self.window_name = rospy.get_param("~window_name", "CUADC Flight Recorder")

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_state = None
        self.latest_global = None
        self.latest_local_pose = None
        self.latest_rel_alt = None
        self.latest_battery = None

        self.recording = False
        self.writer = None
        self.csv_file = None
        self.csv_writer = None
        self.session_dir = None
        self.video_path = None
        self.csv_path = None
        self.frame_count = 0
        self.start_time = None
        self.last_armed = False

        os.makedirs(self.output_dir, exist_ok=True)

        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        if State is None:
            rospy.logwarn(
                "mavros_msgs/State is not available. Armed-trigger recording will not work until ros-*-mavros-msgs is installed."
            )
        else:
            self.state_sub = rospy.Subscriber(self.state_topic, State, self.state_callback, queue_size=10)

        self.global_sub = rospy.Subscriber(self.global_topic, NavSatFix, self.global_callback, queue_size=10)
        self.local_sub = rospy.Subscriber(self.local_pose_topic, PoseStamped, self.local_pose_callback, queue_size=10)
        self.rel_alt_sub = rospy.Subscriber(self.rel_alt_topic, Float64, self.rel_alt_callback, queue_size=10)
        self.battery_sub = rospy.Subscriber(self.battery_topic, BatteryState, self.battery_callback, queue_size=10)

        rospy.on_shutdown(self.shutdown)

        if self.record_immediately:
            rospy.loginfo("record_immediately is true; recording will start when the first image arrives.")

        rospy.loginfo(
            "Flight data video recorder started. image=%s state=%s global=%s local=%s rel_alt=%s battery=%s output=%s",
            self.image_topic,
            self.state_topic,
            self.global_topic,
            self.local_pose_topic,
            self.rel_alt_topic,
            self.battery_topic,
            self.output_dir,
        )

    def state_callback(self, msg):
        armed = bool(msg.armed)
        with self.lock:
            self.latest_state = msg
            was_armed = self.last_armed
            self.last_armed = armed

        if armed and not was_armed:
            rospy.loginfo("MAVROS armed signal received; video recording will start on the next image frame.")
            self.start_recording()
        elif (not armed) and was_armed and self.stop_on_disarm:
            rospy.loginfo("MAVROS disarmed signal received; stopping video recording.")
            self.stop_recording()

    def global_callback(self, msg):
        with self.lock:
            self.latest_global = msg

    def local_pose_callback(self, msg):
        with self.lock:
            self.latest_local_pose = msg

    def rel_alt_callback(self, msg):
        with self.lock:
            self.latest_rel_alt = msg

    def battery_callback(self, msg):
        with self.lock:
            self.latest_battery = msg

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Flight recorder image conversion failed: %s", exc)
            return

        if self.record_immediately and not self.recording:
            self.start_recording()

        snapshot = self.get_snapshot()
        annotated = self.draw_overlay(frame.copy(), msg.header.stamp, snapshot)

        if self.recording:
            self.ensure_writer(annotated.shape)
            if self.writer is not None:
                self.writer.write(annotated)
                self.frame_count += 1
                if self.csv_writer is not None:
                    self.write_csv_row(msg.header.stamp, snapshot)

        if self.show_window:
            cv2.imshow(self.window_name, annotated)
            cv2.waitKey(1)

    def start_recording(self):
        if self.recording:
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(self.output_dir, stamp)
        os.makedirs(self.session_dir, exist_ok=True)
        self.video_path = os.path.join(self.session_dir, "{}_{}.avi".format(self.file_prefix, stamp))
        self.csv_path = os.path.join(self.session_dir, "{}_{}.csv".format(self.file_prefix, stamp))
        self.writer = None
        self.frame_count = 0
        self.start_time = rospy.Time.now()
        self.recording = True

        if self.write_csv:
            self.csv_file = open(self.csv_path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "frame",
                    "ros_time",
                    "record_time_sec",
                    "armed",
                    "mode",
                    "latitude",
                    "longitude",
                    "global_alt_m",
                    "relative_alt_m",
                    "local_x_m",
                    "local_y_m",
                    "local_z_m",
                    "voltage_v",
                    "current_a",
                    "battery_percent",
                ]
            )
        rospy.loginfo("Flight video recording started. video=%s csv=%s", self.video_path, self.csv_path)

    def stop_recording(self):
        if not self.recording:
            return
        self.recording = False
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        rospy.loginfo(
            "Flight video recording stopped. frames=%d video=%s csv=%s",
            self.frame_count,
            self.video_path,
            self.csv_path,
        )

    def ensure_writer(self, shape):
        if self.writer is not None:
            return
        height, width = shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*self.codec[:4])
        self.writer = cv2.VideoWriter(self.video_path, fourcc, self.video_fps, (width, height))
        if not self.writer.isOpened():
            rospy.logerr("Failed to open VideoWriter: %s", self.video_path)
            self.writer = None
            self.stop_recording()
            return
        rospy.loginfo("VideoWriter opened: %s size=%dx%d fps=%.1f codec=%s", self.video_path, width, height, self.video_fps, self.codec)

    def get_snapshot(self):
        with self.lock:
            state = self.latest_state
            global_fix = self.latest_global
            local_pose = self.latest_local_pose
            rel_alt = self.latest_rel_alt
            battery = self.latest_battery
        return {
            "armed": bool(state.armed) if state is not None else False,
            "mode": state.mode if state is not None else "unknown",
            "lat": finite_or_zero(global_fix.latitude) if global_fix is not None else 0.0,
            "lon": finite_or_zero(global_fix.longitude) if global_fix is not None else 0.0,
            "global_alt": finite_or_zero(global_fix.altitude) if global_fix is not None else 0.0,
            "rel_alt": finite_or_zero(rel_alt.data) if rel_alt is not None else 0.0,
            "local_x": finite_or_zero(local_pose.pose.position.x) if local_pose is not None else 0.0,
            "local_y": finite_or_zero(local_pose.pose.position.y) if local_pose is not None else 0.0,
            "local_z": finite_or_zero(local_pose.pose.position.z) if local_pose is not None else 0.0,
            "voltage": finite_or_zero(battery.voltage) if battery is not None else 0.0,
            "current": finite_or_zero(battery.current) if battery is not None else 0.0,
            "percentage": finite_or_zero(battery.percentage) if battery is not None else 0.0,
        }

    def draw_overlay(self, image, stamp, data):
        height, width = image.shape[:2]
        lines = [
            "REC {}  frame={}  t={:.1f}s".format(
                "ON" if self.recording else "WAIT",
                self.frame_count,
                self.record_elapsed_sec(),
            ),
            "armed={} mode={}".format(data["armed"], data["mode"]),
            "lat={:.7f} lon={:.7f}".format(data["lat"], data["lon"]),
            "alt_global={:.2f}m rel_alt={:.2f}m".format(data["global_alt"], data["rel_alt"]),
            "local xyz=({:.2f}, {:.2f}, {:.2f})m".format(data["local_x"], data["local_y"], data["local_z"]),
            "battery={:.2f}V current={:.2f}A pct={:.0f}%".format(
                data["voltage"],
                data["current"],
                data["percentage"] * 100.0 if data["percentage"] <= 1.0 else data["percentage"],
            ),
            "ros_time={:.3f}".format(stamp.to_sec() if stamp is not None else rospy.Time.now().to_sec()),
        ]

        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = self.overlay_scale
        thickness = max(1, self.overlay_thickness)
        padding = 8
        gap = 7
        text_sizes = [cv2.getTextSize(line, font, scale, thickness)[0] for line in lines]
        panel_width = min(width - 8, max(size[0] for size in text_sizes) + padding * 2)
        line_height = max(size[1] for size in text_sizes)
        panel_height = len(lines) * line_height + (len(lines) - 1) * gap + padding * 2
        x0 = 4
        y0 = 4
        x1 = x0 + panel_width
        y1 = y0 + panel_height
        cv2.rectangle(image, (x0, y0), (x1, y1), (0, 0, 0), -1)
        border_color = (0, 0, 255) if self.recording else (0, 255, 255)
        cv2.rectangle(image, (x0, y0), (x1, y1), border_color, 2)

        y = y0 + padding + line_height
        for index, line in enumerate(lines):
            color = (0, 0, 255) if index == 0 and self.recording else (0, 255, 255)
            cv2.putText(image, line, (x0 + padding, y), font, scale, color, thickness, cv2.LINE_AA)
            y += line_height + gap
        return image

    def record_elapsed_sec(self):
        if self.start_time is None:
            return 0.0
        return max(0.0, (rospy.Time.now() - self.start_time).to_sec())

    def write_csv_row(self, stamp, data):
        self.csv_writer.writerow(
            [
                self.frame_count,
                stamp.to_sec() if stamp is not None else rospy.Time.now().to_sec(),
                "{:.3f}".format(self.record_elapsed_sec()),
                int(data["armed"]),
                data["mode"],
                "{:.9f}".format(data["lat"]),
                "{:.9f}".format(data["lon"]),
                "{:.3f}".format(data["global_alt"]),
                "{:.3f}".format(data["rel_alt"]),
                "{:.3f}".format(data["local_x"]),
                "{:.3f}".format(data["local_y"]),
                "{:.3f}".format(data["local_z"]),
                "{:.3f}".format(data["voltage"]),
                "{:.3f}".format(data["current"]),
                "{:.3f}".format(data["percentage"]),
            ]
        )

    def shutdown(self):
        self.stop_recording()
        if self.show_window:
            cv2.destroyWindow(self.window_name)


def main():
    rospy.init_node("flight_data_video_recorder_node")
    FlightDataVideoRecorderNode()
    rospy.spin()


if __name__ == "__main__":
    main()
