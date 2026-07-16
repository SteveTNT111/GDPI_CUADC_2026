#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import statistics
import time
from collections import deque

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from sensor_msgs.msg import CameraInfo, NavSatFix, NavSatStatus
from std_msgs.msg import Float64, String

from cuadc_vision.msg import GeoTarget, MissionStatus, YoloDetections


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
MAV_CMD_DO_SET_SERVO = 183


def now_s():
    return time.monotonic()


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def finite_number(value):
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def quaternion_rotate_vector(q, vector):
    """Rotate body vector into ENU using geometry_msgs/Quaternion."""
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    vx, vy, vz = vector

    # q * v * q^-1, expanded to avoid depending on tf at runtime.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return rx, ry, rz


def destination_wgs84(latitude, longitude, distance, heading_deg):
    lat_rad = math.radians(latitude)
    heading_rad = math.radians(heading_deg)
    flattening_term = WGS84_F * (2.0 - WGS84_F)
    sin_lat = math.sin(lat_rad)
    denominator = math.sqrt(1.0 - flattening_term * sin_lat * sin_lat)
    prime_vertical_radius = WGS84_A / denominator
    meridian_radius = WGS84_A * (1.0 - flattening_term) / denominator ** 3

    north = distance * math.cos(heading_rad)
    east = distance * math.sin(heading_rad)
    target_lat = latitude + math.degrees(north / meridian_radius)
    cos_lat = math.cos(lat_rad)
    if abs(cos_lat) < 1e-12:
        raise ValueError("latitude too close to pole")
    target_lon = longitude + math.degrees(east / (prime_vertical_radius * cos_lat))
    target_lon = (target_lon + 180.0) % 360.0 - 180.0
    return target_lat, target_lon


def horizontal_wgs84_distance(lat1, lon1, lat2, lon2):
    mean_lat = math.radians((lat1 + lat2) * 0.5)
    flattening_term = WGS84_F * (2.0 - WGS84_F)
    sin_lat = math.sin(mean_lat)
    denominator = math.sqrt(1.0 - flattening_term * sin_lat * sin_lat)
    prime_vertical_radius = WGS84_A / denominator
    meridian_radius = WGS84_A * (1.0 - flattening_term) / denominator ** 3
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians((lon2 - lon1 + 180.0) % 360.0 - 180.0)
    north = dlat * meridian_radius
    east = dlon * prime_vertical_radius * math.cos(mean_lat)
    return math.hypot(north, east)


class FlightIO:
    """Small wrapper around MAVROS primitives already used by verified scripts."""

    def __init__(self):
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_fix = NavSatFix()
        self.heading_deg = None
        self.relative_altitude = None

        self.state_received = False
        self.pose_received = False
        self.fix_received = False
        self.heading_received = False
        self.relative_altitude_received = False
        self.pose_received_at = 0.0
        self.fix_received_at = 0.0
        self.heading_received_at = 0.0
        self.relative_altitude_received_at = 0.0

        self.start_pose = None
        self.start_fix = None
        self.start_heading_deg = None
        self.ground_relative_altitude = None
        self.target_relative_altitude = None

        self.connection_timeout = float(rospy.get_param("~connection_timeout", 30.0))
        self.service_timeout = float(rospy.get_param("~service_timeout", 10.0))
        self.data_timeout = float(rospy.get_param("~data_timeout", 30.0))
        self.data_max_age = float(rospy.get_param("~data_max_age", 2.0))
        self.mode_timeout = float(rospy.get_param("~mode_timeout", 6.0))
        self.arm_timeout = float(rospy.get_param("~arm_timeout", 6.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 45.0))
        self.setpoint_rate_hz = float(rospy.get_param("~setpoint_rate", 10.0))

        self._set_mode_srv = None
        self._arming_srv = None
        self._takeoff_srv = None
        self._cmd_srv = None

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb, queue_size=10)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._fix_cb, queue_size=10)
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self._heading_cb, queue_size=10)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self._rel_alt_cb, queue_size=10)

        self.local_setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=20
        )
        self.global_setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=20
        )
        self.main_status_pub = rospy.Publisher("/mission/main_status", String, queue_size=1)
        self.mission_status_pub = rospy.Publisher("/vision/mission_status", MissionStatus, queue_size=1)

    def _state_cb(self, msg):
        self.current_state = msg
        self.state_received = True

    def _pose_cb(self, msg):
        self.current_pose = msg
        self.pose_received = True
        self.pose_received_at = now_s()

    def _fix_cb(self, msg):
        if self.valid_fix(msg):
            self.current_fix = msg
            self.fix_received = True
            self.fix_received_at = now_s()

    def _heading_cb(self, msg):
        if finite_number(msg.data):
            self.heading_deg = float(msg.data) % 360.0
            self.heading_received = True
            self.heading_received_at = now_s()

    def _rel_alt_cb(self, msg):
        if finite_number(msg.data):
            self.relative_altitude = float(msg.data)
            self.relative_altitude_received = True
            self.relative_altitude_received_at = now_s()

    def valid_fix(self, msg):
        if msg is None:
            return False
        if getattr(msg.status, "status", NavSatStatus.STATUS_NO_FIX) < NavSatStatus.STATUS_FIX:
            return False
        return all(finite_number(v) for v in (msg.latitude, msg.longitude, msg.altitude))

    def nav_fresh(self):
        t = now_s()
        return (
            self.pose_received
            and self.fix_received
            and self.heading_received
            and self.relative_altitude_received
            and t - self.pose_received_at <= self.data_max_age
            and t - self.fix_received_at <= self.data_max_age
            and t - self.heading_received_at <= self.data_max_age
            and t - self.relative_altitude_received_at <= self.data_max_age
        )

    def rel_alt_fresh(self):
        return (
            self.relative_altitude_received
            and now_s() - self.relative_altitude_received_at <= self.data_max_age
        )

    def wait_for_connection(self):
        rospy.loginfo("Waiting for MAVROS flight-controller connection...")
        deadline = now_s() + self.connection_timeout
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.state_received and self.current_state.connected:
                rospy.loginfo("Flight controller connected")
                return True
            if now_s() >= deadline:
                rospy.logerr("Timed out waiting for flight-controller connection")
                return False
            rate.sleep()
        return False

    def wait_for_navigation(self):
        rospy.loginfo("Waiting for local pose, WGS84, heading and relative altitude...")
        deadline = now_s() + self.data_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.nav_fresh():
                rospy.loginfo("Navigation data is fresh")
                return True
            if now_s() >= deadline:
                rospy.logerr(
                    "Timed out waiting for nav data: pose=%s fix=%s heading=%s rel_alt=%s",
                    self.pose_received,
                    self.fix_received,
                    self.heading_received,
                    self.relative_altitude_received,
                )
                return False
            rate.sleep()
        return False

    def ensure_services(self, need_servo=False):
        if self._set_mode_srv is not None and (not need_servo or self._cmd_srv is not None):
            return True
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=self.service_timeout)
            rospy.wait_for_service("/mavros/cmd/arming", timeout=self.service_timeout)
            rospy.wait_for_service("/mavros/cmd/takeoff", timeout=self.service_timeout)
            self._set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self._arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            self._takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
            if need_servo or self._cmd_srv is None:
                try:
                    rospy.wait_for_service("/mavros/cmd/command", timeout=3.0)
                    self._cmd_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
                except rospy.ROSException:
                    if need_servo:
                        rospy.logerr("/mavros/cmd/command unavailable")
                        return False
                    rospy.logwarn("/mavros/cmd/command unavailable; real drop disabled")
            return True
        except rospy.ROSException as exc:
            rospy.logerr("MAVROS service wait failed: %s", exc)
            return False

    def wait_for_mode(self, expected_mode):
        deadline = now_s() + self.mode_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state.mode == expected_mode:
                return True
            if now_s() >= deadline:
                return False
            rate.sleep()
        return False

    def set_mode(self, mode):
        if not self.ensure_services():
            return False
        try:
            response = self._set_mode_srv(custom_mode=mode)
        except rospy.ServiceException as exc:
            rospy.logerr("set_mode(%s) failed: %s", mode, exc)
            return False
        if not response.mode_sent or not self.wait_for_mode(mode):
            rospy.logerr("Mode switch failed: expected=%s current=%s", mode, self.current_state.mode)
            return False
        rospy.loginfo("Mode confirmed: %s", mode)
        return True

    def arm(self):
        if not self.ensure_services():
            return False
        if self.current_state.armed:
            rospy.logwarn("Vehicle was already armed")
            return True
        try:
            response = self._arming_srv(value=True)
        except rospy.ServiceException as exc:
            rospy.logerr("arming failed: %s", exc)
            return False
        if not response.success:
            rospy.logerr("Arming rejected: result=%s", getattr(response, "result", -1))
            return False
        deadline = now_s() + self.arm_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_state.armed:
                rospy.loginfo("Armed confirmed")
                return True
            if now_s() >= deadline:
                rospy.logerr("Timed out waiting for armed=True")
                return False
            rate.sleep()
        return False

    def capture_start_reference(self):
        if not self.nav_fresh():
            return False
        self.start_pose = self.current_pose
        self.start_fix = self.current_fix
        self.start_heading_deg = self.heading_deg
        return True

    def capture_ground_altitude_reference(self, takeoff_altitude, settle_s=2.0):
        deadline = now_s() + self.data_timeout
        fresh_since = None
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t = now_s()
            if not self.current_state.connected or not self.current_state.armed:
                rospy.logerr("Connection lost or disarmed while capturing ground altitude")
                return False
            if self.rel_alt_fresh():
                if fresh_since is None:
                    fresh_since = t
                if t - fresh_since >= settle_s:
                    self.ground_relative_altitude = self.relative_altitude
                    self.target_relative_altitude = self.ground_relative_altitude + takeoff_altitude
                    rospy.loginfo(
                        "Ground rel_alt=%.2f m, target rel_alt=%.2f m",
                        self.ground_relative_altitude,
                        self.target_relative_altitude,
                    )
                    return True
            else:
                fresh_since = None
            if t >= deadline:
                rospy.logerr("Timed out waiting for fresh relative altitude")
                return False
            rate.sleep()
        return False

    def takeoff(self):
        if self.target_relative_altitude is None:
            rospy.logerr("No target relative altitude captured")
            return False
        if self.current_state.mode != "GUIDED" and not self.set_mode("GUIDED"):
            return False
        try:
            response = self._takeoff_srv(
                altitude=self.target_relative_altitude,
                latitude=0.0,
                longitude=0.0,
                min_pitch=0.0,
                yaw=0.0,
            )
        except rospy.ServiceException as exc:
            rospy.logerr("takeoff failed: %s", exc)
            return False
        if not response.success:
            rospy.logerr("Takeoff rejected: result=%s", getattr(response, "result", -1))
            return False
        return True

    def wait_takeoff_altitude(self, tolerance=0.25):
        deadline = now_s() + self.takeoff_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.current_state.connected or not self.current_state.armed:
                return False
            if self.relative_altitude is not None and self.relative_altitude >= self.target_relative_altitude - tolerance:
                rospy.loginfo("Takeoff altitude reached: %.2f m", self.relative_altitude)
                return True
            if now_s() >= deadline:
                rospy.logerr("Timed out waiting for takeoff altitude")
                return False
            rospy.loginfo_throttle(
                2.0,
                "Climbing: rel_alt=%.2f / %.2f",
                self.relative_altitude or -999.0,
                self.target_relative_altitude,
            )
            rate.sleep()
        return False

    def start_offset_to_enu(self, offset_enu):
        if self.start_pose is None:
            return None
        p = self.start_pose.pose.position
        return (float(p.x) + offset_enu[0], float(p.y) + offset_enu[1], float(p.z) + offset_enu[2])

    def make_local_pose(self, enu):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(enu[0])
        msg.pose.position.y = float(enu[1])
        msg.pose.position.z = float(enu[2])
        msg.pose.orientation = self.current_pose.pose.orientation
        return msg

    def publish_local_target(self, enu):
        self.local_setpoint_pub.publish(self.make_local_pose(enu))

    def local_error(self, enu):
        p = self.current_pose.pose.position
        return math.sqrt((p.x - enu[0]) ** 2 + (p.y - enu[1]) ** 2 + (p.z - enu[2]) ** 2)

    def horizontal_error(self, enu):
        p = self.current_pose.pose.position
        return math.hypot(p.x - enu[0], p.y - enu[1])

    def make_global_target(self, latitude, longitude, rel_altitude):
        target = GlobalPositionTarget()
        target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        target.type_mask = (
            GlobalPositionTarget.IGNORE_VX
            | GlobalPositionTarget.IGNORE_VY
            | GlobalPositionTarget.IGNORE_VZ
            | GlobalPositionTarget.IGNORE_AFX
            | GlobalPositionTarget.IGNORE_AFY
            | GlobalPositionTarget.IGNORE_AFZ
            | GlobalPositionTarget.IGNORE_YAW
            | GlobalPositionTarget.IGNORE_YAW_RATE
        )
        target.latitude = float(latitude)
        target.longitude = float(longitude)
        target.altitude = float(rel_altitude)
        return target

    def publish_global_target(self, target):
        target.header.stamp = rospy.Time.now()
        self.global_setpoint_pub.publish(target)

    def set_servo_pwm(self, channel, pwm):
        if not self.ensure_services(need_servo=True):
            return False
        try:
            response = self._cmd_srv(
                broadcast=False,
                command=MAV_CMD_DO_SET_SERVO,
                confirmation=0,
                param1=float(channel),
                param2=float(pwm),
                param3=0.0,
                param4=0.0,
                param5=0.0,
                param6=0.0,
                param7=0.0,
            )
            return bool(response.success)
        except rospy.ServiceException as exc:
            rospy.logerr("MAV_CMD_DO_SET_SERVO(%d, %d) failed: %s", channel, pwm, exc)
            self._cmd_srv = None
            return False

    def pulse_servo(self, channel, open_pwm, close_pwm, hold_s):
        if not self.set_servo_pwm(channel, open_pwm):
            return False
        rospy.sleep(hold_s)
        return self.set_servo_pwm(channel, close_pwm)

    def publish_detector_status(self, ammo_a, ammo_b, aiming, last_drop=""):
        msg = MissionStatus()
        msg.ammo_a = int(max(0, ammo_a))
        msg.ammo_b = int(max(0, ammo_b))
        msg.aiming = bool(aiming)
        msg.last_drop = last_drop
        self.mission_status_pub.publish(msg)

    def publish_main_status(self, payload):
        self.main_status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))


class TargetTrack:
    def __init__(self, track_id, det, target_enu, body_xyz, center_norm, stamp):
        self.track_id = track_id
        self.class_name = det.class_name
        self.first_seen = stamp
        self.last_seen = stamp
        self.hits = 0
        self.confidence = 0.0
        self.centers = deque(maxlen=8)
        self.enu_samples = deque(maxlen=8)
        self.body_samples = deque(maxlen=8)
        self.center_norm = center_norm
        self.update(det, target_enu, body_xyz, center_norm, stamp)

    def update(self, det, target_enu, body_xyz, center_norm, stamp):
        self.hits += 1
        self.last_seen = stamp
        self.class_name = det.class_name
        self.confidence = max(self.confidence * 0.8, float(det.confidence))
        self.centers.append((float(det.center_x), float(det.center_y)))
        self.center_norm = float(center_norm)
        if target_enu is not None:
            self.enu_samples.append(target_enu)
        if body_xyz is not None:
            self.body_samples.append(body_xyz)

    def has_position(self):
        return len(self.enu_samples) > 0

    def stable_enu(self):
        if not self.enu_samples:
            return None
        xs = [p[0] for p in self.enu_samples]
        ys = [p[1] for p in self.enu_samples]
        zs = [p[2] for p in self.enu_samples]
        return (statistics.median(xs), statistics.median(ys), statistics.median(zs))

    def stable_body(self):
        if not self.body_samples:
            return None
        xs = [p[0] for p in self.body_samples]
        ys = [p[1] for p in self.body_samples]
        zs = [p[2] for p in self.body_samples]
        return (statistics.median(xs), statistics.median(ys), statistics.median(zs))

    def age_since_seen(self, stamp):
        return stamp - self.last_seen

    def score(self):
        position_bonus = 8.0 if self.has_position() else 0.0
        return position_bonus + self.hits * 2.0 + self.confidence * 5.0 - self.center_norm / 120.0


class TargetTracker:
    """Multi-frame bucket stabilizer built on detector_node outputs."""

    def __init__(self, flight):
        self.flight = flight
        self.camera_center_x = float(rospy.get_param("~image_center_x", 640.0))
        self.camera_center_y = float(rospy.get_param("~image_center_y", 360.0))
        self.camera_mount_z = float(rospy.get_param("~camera_mount_z", 0.40))
        self.min_conf = float(rospy.get_param("~target_min_confidence", 0.35))
        self.min_hits = int(rospy.get_param("~target_min_hits", 3))
        self.track_stale_s = float(rospy.get_param("~target_stale_s", 1.5))
        self.match_center_px = float(rospy.get_param("~target_match_center_px", 110.0))
        self.match_enu_m = float(rospy.get_param("~target_match_enu_m", 0.80))
        self.reject_jump_m = float(rospy.get_param("~target_reject_jump_m", 1.20))
        self.dropped_blacklist_m = float(rospy.get_param("~dropped_blacklist_m", 0.70))

        self.tracks = {}
        self.next_track_id = 1
        self.dropped_targets_enu = []
        self.latest_geo = None
        self.last_detections_stamp = 0.0

        rospy.Subscriber("/vision/yolo/detections", YoloDetections, self._detections_cb, queue_size=5)
        rospy.Subscriber("/vision/target_global", GeoTarget, self._geo_cb, queue_size=5)
        rospy.Subscriber("/vision/color/camera_info", CameraInfo, self._camera_info_cb, queue_size=1)

    def _camera_info_cb(self, msg):
        if msg.K and len(msg.K) >= 6:
            self.camera_center_x = float(msg.K[2])
            self.camera_center_y = float(msg.K[5])

    def _geo_cb(self, msg):
        self.latest_geo = msg

    def _detections_cb(self, msg):
        stamp = now_s()
        self.last_detections_stamp = stamp
        used_tracks = set()
        for det in msg.detections:
            if not getattr(det, "detected", True):
                continue
            if float(det.confidence) < self.min_conf:
                continue
            target_enu, body_xyz = self.detection_to_enu(det)
            if target_enu is not None and self.is_blacklisted(target_enu):
                continue
            center_norm = math.hypot(float(det.center_x) - self.camera_center_x, float(det.center_y) - self.camera_center_y)
            track = self._match_track(det, target_enu, used_tracks)
            if track is None:
                track = TargetTrack(self.next_track_id, det, target_enu, body_xyz, center_norm, stamp)
                self.tracks[track.track_id] = track
                self.next_track_id += 1
            else:
                current_enu = track.stable_enu()
                if current_enu is not None and target_enu is not None:
                    jump = math.dist(current_enu, target_enu)
                    if jump > self.reject_jump_m:
                        continue
                track.update(det, target_enu, body_xyz, center_norm, stamp)
            used_tracks.add(track.track_id)
        self.prune()

    def detection_to_body(self, det):
        if not det.position_valid:
            return None
        if not all(finite_number(v) for v in (det.camera_x_m, det.camera_y_m, det.camera_z_m)):
            return None
        # Same mapping as detector_node.transform_camera_to_body().
        return (-float(det.camera_y_m), float(det.camera_x_m), float(det.camera_z_m) + self.camera_mount_z)

    def detection_to_enu(self, det):
        body = self.detection_to_body(det)
        if body is None or not self.flight.pose_received:
            return None, body
        offset_enu = quaternion_rotate_vector(self.flight.current_pose.pose.orientation, body)
        p = self.flight.current_pose.pose.position
        return (p.x + offset_enu[0], p.y + offset_enu[1], p.z + offset_enu[2]), body

    def _match_track(self, det, target_enu, used_tracks):
        best = None
        best_cost = float("inf")
        det_center = (float(det.center_x), float(det.center_y))
        for track in self.tracks.values():
            if track.track_id in used_tracks:
                continue
            if track.centers:
                c = track.centers[-1]
                center_cost = math.hypot(det_center[0] - c[0], det_center[1] - c[1])
            else:
                center_cost = float("inf")
            enu_cost = float("inf")
            track_enu = track.stable_enu()
            if target_enu is not None and track_enu is not None:
                enu_cost = math.dist(target_enu, track_enu)
            if center_cost <= self.match_center_px or enu_cost <= self.match_enu_m:
                cost = min(center_cost / self.match_center_px, enu_cost / self.match_enu_m)
                if cost < best_cost:
                    best = track
                    best_cost = cost
        return best

    def prune(self):
        stamp = now_s()
        stale = [tid for tid, tr in self.tracks.items() if tr.age_since_seen(stamp) > self.track_stale_s]
        for tid in stale:
            del self.tracks[tid]

    def valid_tracks(self):
        self.prune()
        return [
            tr for tr in self.tracks.values()
            if tr.hits >= self.min_hits and tr.has_position() and not self.is_blacklisted(tr.stable_enu())
        ]

    def best_track(self):
        tracks = self.valid_tracks()
        if not tracks:
            return None
        return max(tracks, key=lambda tr: tr.score())

    def is_blacklisted(self, enu):
        if enu is None:
            return False
        return any(math.dist(enu[:2], old[:2]) <= self.dropped_blacklist_m for old in self.dropped_targets_enu)

    def mark_dropped(self, enu):
        if enu is not None:
            self.dropped_targets_enu.append(enu)


class DropperManager:
    def __init__(self, flight):
        self.flight = flight
        self.ammo_a = int(rospy.get_param("~ammo_a", 1))
        self.ammo_b = int(rospy.get_param("~ammo_b", 1))
        self.servo_a_channel = int(rospy.get_param("~servo_a_channel", 5))
        self.servo_b_channel = int(rospy.get_param("~servo_b_channel", 6))
        self.servo_open_pwm = int(rospy.get_param("~servo_open_pwm", 1500))
        self.servo_close_pwm = int(rospy.get_param("~servo_close_pwm", 1000))
        self.servo_hold_s = float(rospy.get_param("~servo_hold_s", 0.8))
        self.offset_a_body_x_m = float(rospy.get_param("~dropper_a_forward_offset_m", 0.05))
        self.offset_b_body_x_m = float(rospy.get_param("~dropper_b_forward_offset_m", -0.05))
        self.drop_cooldown_s = float(rospy.get_param("~drop_cooldown_s", 5.0))
        self.enable_servo_drop = get_bool_param("~enable_servo_drop", False)
        self.last_drop_at = 0.0
        self.last_drop_label = ""
        self.last_drop_display_until = 0.0

    def total_ammo(self):
        return max(0, self.ammo_a) + max(0, self.ammo_b)

    def current_label(self):
        if self.ammo_a > 0:
            return "A"
        if self.ammo_b > 0:
            return "B"
        return "NONE"

    def current_channel(self):
        return self.servo_a_channel if self.current_label() == "A" else self.servo_b_channel

    def current_body_offset(self):
        label = self.current_label()
        if label == "A":
            return (self.offset_a_body_x_m, 0.0, 0.0)
        if label == "B":
            return (self.offset_b_body_x_m, 0.0, 0.0)
        return (0.0, 0.0, 0.0)

    def compensated_target_enu(self, locked_target_enu):
        offset_body = self.current_body_offset()
        offset_enu = quaternion_rotate_vector(self.flight.current_pose.pose.orientation, offset_body)
        return (
            locked_target_enu[0] - offset_enu[0],
            locked_target_enu[1] - offset_enu[1],
            locked_target_enu[2],
        )

    def cooldown_ready(self):
        return now_s() - self.last_drop_at >= self.drop_cooldown_s

    def execute_drop(self):
        label = self.current_label()
        if label == "NONE":
            rospy.logwarn("No ammo left; drop skipped")
            return False
        if not self.cooldown_ready():
            rospy.logwarn("Drop cooldown active")
            return False

        ok = True
        if self.enable_servo_drop:
            ok = self.flight.pulse_servo(
                self.current_channel(),
                self.servo_open_pwm,
                self.servo_close_pwm,
                self.servo_hold_s,
            )
        else:
            rospy.logwarn("Dry-run drop: enable_servo_drop=false, not moving servo %s", label)

        if not ok:
            return False
        if label == "A":
            self.ammo_a -= 1
        elif label == "B":
            self.ammo_b -= 1
        self.last_drop_at = now_s()
        self.last_drop_label = label
        self.last_drop_display_until = now_s() + 3.0
        return True

    def detector_last_drop(self):
        if self.last_drop_label and now_s() <= self.last_drop_display_until:
            return self.last_drop_label
        return ""
