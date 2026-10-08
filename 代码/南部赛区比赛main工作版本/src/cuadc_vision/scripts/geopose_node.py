#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus

from cuadc_vision.msg import GeoTarget, YoloDetection


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def quaternion_rotate_vector(q, vector):
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return vx + w * tx + (y * tz - z * ty), \
           vy + w * ty + (z * tx - x * tz), \
           vz + w * tz + (x * ty - y * tx)


def enu_offset_to_geodetic(origin_lat, origin_lon, origin_alt, east_m, north_m, up_m):
    try:
        from geographiclib.geodesic import Geodesic
        horizontal_m = math.hypot(east_m, north_m)
        if horizontal_m > 0.0:
            azimuth_deg = math.degrees(math.atan2(east_m, north_m))
            result = Geodesic.WGS84.Direct(origin_lat, origin_lon, azimuth_deg, horizontal_m)
            return result["lat2"], result["lon2"], origin_alt + up_m
        return origin_lat, origin_lon, origin_alt + up_m
    except Exception:
        return enu_offset_to_geodetic_local(origin_lat, origin_lon, origin_alt, east_m, north_m, up_m)


def enu_offset_to_geodetic_local(origin_lat, origin_lon, origin_alt, east_m, north_m, up_m):
    lat_rad = math.radians(origin_lat)
    lon_rad = math.radians(origin_lon)
    semi_major = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_sq = flattening * (2.0 - flattening)
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - eccentricity_sq * sin_lat * sin_lat)
    prime_vertical_radius = semi_major / denom
    meridian_radius = semi_major * (1.0 - eccentricity_sq) / (denom ** 3)
    d_lat = north_m / (meridian_radius + origin_alt)
    cos_lat = max(1e-12, math.cos(lat_rad))
    d_lon = east_m / ((prime_vertical_radius + origin_alt) * cos_lat)
    return math.degrees(lat_rad + d_lat), math.degrees(lon_rad + d_lon), origin_alt + up_m


class GeoposeNode:
    def __init__(self):
        self.detection_topic = rospy.get_param("~detection_topic", "/vision/yolo/detection")
        self.global_position_topic = rospy.get_param(
            "~global_position_topic", "/mavros/global_position/global")
        self.local_pose_topic = rospy.get_param(
            "~local_pose_topic", "/mavros/local_position/pose")
        self.output_topic = rospy.get_param("~output_topic", "/vision/target_global")
        self.body_frame = rospy.get_param("~body_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame", "d435i_color_optical_frame")
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.30))
        self.transform_timeout_sec = float(rospy.get_param("~transform_timeout_sec", 0.10))
        self.publish_invalid = get_bool_param("~publish_invalid", True)

        self.lock = threading.Lock()
        self.latest_global = None
        self.latest_local_pose = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(self.output_topic, GeoTarget, queue_size=1)
        self.detection_sub = rospy.Subscriber(
            self.detection_topic, YoloDetection, self.detection_callback, queue_size=1)
        self.global_sub = rospy.Subscriber(
            self.global_position_topic, NavSatFix, self.global_callback, queue_size=1)
        self.local_pose_sub = rospy.Subscriber(
            self.local_pose_topic, PoseStamped, self.local_pose_callback, queue_size=1)

        rospy.loginfo("Geopose started. detection=%s output=%s body=%s camera=%s",
                       self.detection_topic, self.output_topic, self.body_frame, self.camera_frame)

    def global_callback(self, msg):
        with self.lock:
            self.latest_global = msg

    def local_pose_callback(self, msg):
        with self.lock:
            self.latest_local_pose = msg

    def detection_callback(self, msg):
        target = self.make_base_target(msg)
        if not msg.detected:
            self.publish_status(target, "no_detection")
            return
        if msg.confidence < self.min_confidence:
            self.publish_status(target, "low_confidence")
            return
        if not msg.position_valid:
            self.publish_status(target, "invalid_camera_position")
            return

        with self.lock:
            latest_global = self.latest_global
            latest_local_pose = self.latest_local_pose

        if not self.is_valid_global(latest_global):
            self.publish_status(target, "no_valid_global_position")
            return
        if latest_local_pose is None:
            self.publish_status(target, "no_local_pose")
            return

        body_point = self.transform_camera_to_body(msg)
        if body_point is None:
            self.publish_status(target, "tf_camera_to_body_failed")
            return

        bx, by, bz = body_point.point.x, body_point.point.y, body_point.point.z
        east_m, north_m, up_m = quaternion_rotate_vector(
            latest_local_pose.pose.orientation, (bx, by, bz))
        lat, lon, alt = enu_offset_to_geodetic(
            latest_global.latitude, latest_global.longitude, latest_global.altitude,
            east_m, north_m, up_m)

        target.valid = True
        target.status = "ok"
        target.body_x_m = float(bx)
        target.body_y_m = float(by)
        target.body_z_m = float(bz)
        target.enu_east_m = float(east_m)
        target.enu_north_m = float(north_m)
        target.enu_up_m = float(up_m)
        target.latitude = float(lat)
        target.longitude = float(lon)
        target.altitude = float(alt)
        self.pub.publish(target)

    def make_base_target(self, detection):
        target = GeoTarget()
        target.header = detection.header
        target.valid = False
        target.status = ""
        target.source_topic = self.detection_topic
        target.class_name = detection.class_name
        target.confidence = detection.confidence
        target.center_x = detection.center_x
        target.center_y = detection.center_y
        target.camera_x_m = detection.camera_x_m
        target.camera_y_m = detection.camera_y_m
        target.camera_z_m = detection.camera_z_m
        return target

    def publish_status(self, target, status):
        target.status = status
        if self.publish_invalid:
            self.pub.publish(target)
        rospy.logdebug("Geopose status: %s", status)

    def transform_camera_to_body(self, detection):
        point = PointStamped()
        point.header = detection.header
        if not point.header.frame_id:
            point.header.frame_id = self.camera_frame
        point.point.x = detection.camera_x_m
        point.point.y = detection.camera_y_m
        point.point.z = detection.camera_z_m

        timeout = rospy.Duration.from_sec(max(0.0, self.transform_timeout_sec))
        try:
            return self.tf_buffer.transform(point, self.body_frame, timeout)
        except Exception as exc1:
            try:
                point.header.stamp = rospy.Time(0)
                return self.tf_buffer.transform(point, self.body_frame, timeout)
            except Exception as exc2:
                rospy.logwarn_throttle(2.0,
                    "TF failed %s->%s: %s / %s",
                    point.header.frame_id, self.body_frame, exc1, exc2)
                return None

    def is_valid_global(self, msg):
        if msg is None:
            return False
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return False
        return all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude))


def main():
    rospy.init_node("geopose_node")
    GeoposeNode()
    rospy.spin()


if __name__ == "__main__":
    main()
