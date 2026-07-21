#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Autonomous forward search, visual descent, A/B drops, climb, and RTL.

This file is intentionally isolated from the existing project files.  It uses
the aircraft-local convention confirmed by flight testing:

    local +X = right, local +Y = forward, local +Z = up

The node consumes the existing detector's YoloDetections output, but derives
horizontal metric offsets directly from detection pixels, camera intrinsics,
and depth.  That avoids depending on the detector's configurable camera-X sign.

Real flight is guarded by both ``~execute_mission`` and
``~calibration_confirmed``.  Defaults never arm or move the aircraft.
"""

import json
import math
import statistics
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass

import rospy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong, CommandTOL, SetMode
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32, String

from cuadc_vision.msg import MissionStatus, YoloDetections

from mission_geometry import (
    angular_difference,
    distance_3d,
    finite,
    limit_vector_2d,
    pixel_to_right_forward,
    quaternion_yaw,
    ramped_progress,
    target_relative_to_aircraft,
)


MAV_CMD_DO_SET_SERVO = 183


def monotonic_s():
    return time.monotonic()


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def median(values):
    return float(statistics.median(values))


@dataclass
class TargetSample:
    stamp: float
    center_x: int
    center_y: int
    confidence: float
    depth_m: float
    local_x_right_m: float
    local_y_forward_m: float
    relative_x_right_m: float
    relative_y_forward_m: float
    relative_z_down_m: float
    distance_m: float


class TargetTrack:
    def __init__(self, track_id, sample, history_size):
        self.track_id = int(track_id)
        self.samples = deque(maxlen=max(3, int(history_size)))
        self.hits = 0
        self.last_seen = 0.0
        self.best_confidence = 0.0
        self.update(sample)

    def update(self, sample):
        self.samples.append(sample)
        self.hits += 1
        self.last_seen = float(sample.stamp)
        self.best_confidence = max(self.best_confidence, float(sample.confidence))

    def latest(self):
        return self.samples[-1] if self.samples else None

    def stable_xy(self):
        if not self.samples:
            return None
        return (
            median([sample.local_x_right_m for sample in self.samples]),
            median([sample.local_y_forward_m for sample in self.samples]),
        )

    def score(self, now):
        age_penalty = max(0.0, now - self.last_seen) * 10.0
        return self.hits * 2.0 + self.best_confidence * 5.0 - age_penalty


class VisionTargetTracker:
    """Nearest-neighbour multi-frame tracker in mission-local X/Y."""

    def __init__(self, params):
        self.min_confidence = params["min_confidence"]
        self.min_depth_m = params["min_depth_m"]
        self.max_depth_m = params["max_depth_m"]
        self.min_hits = params["min_hits"]
        self.history_size = params["history_size"]
        self.match_radius_m = params["match_radius_m"]
        self.stale_timeout_s = params["stale_timeout_s"]
        self.dropped_blacklist_radius_m = params["dropped_blacklist_radius_m"]
        self.camera_x_right_m = params["camera_x_right_m"]
        self.camera_y_forward_m = params["camera_y_forward_m"]
        self.camera_down_m = params["camera_down_m"]

        self.lock = threading.Lock()
        self.tracks = {}
        self.next_track_id = 1
        self.dropped_xy = []
        self.last_message_at = 0.0

    def update(self, detections_msg, pose_xyz, intrinsics):
        now = monotonic_s()
        pose_x, pose_y, _pose_z = pose_xyz
        fx, fy, cx, cy = intrinsics
        samples = []

        for det in detections_msg.detections:
            if not getattr(det, "detected", True):
                continue
            if not getattr(det, "position_valid", False):
                continue
            confidence = float(det.confidence)
            depth_m = float(det.camera_z_m if det.camera_z_m > 0.0 else det.depth_m)
            if confidence < self.min_confidence:
                continue
            if not finite(depth_m) or depth_m < self.min_depth_m or depth_m > self.max_depth_m:
                continue
            try:
                camera_right, camera_forward = pixel_to_right_forward(
                    det.center_x, det.center_y, depth_m, fx, fy, cx, cy
                )
            except ValueError:
                continue

            rel_right, rel_forward, rel_down = target_relative_to_aircraft(
                camera_right,
                camera_forward,
                depth_m,
                self.camera_x_right_m,
                self.camera_y_forward_m,
                self.camera_down_m,
            )
            samples.append(
                TargetSample(
                    stamp=now,
                    center_x=int(det.center_x),
                    center_y=int(det.center_y),
                    confidence=confidence,
                    depth_m=depth_m,
                    local_x_right_m=float(pose_x) + rel_right,
                    local_y_forward_m=float(pose_y) + rel_forward,
                    relative_x_right_m=rel_right,
                    relative_y_forward_m=rel_forward,
                    relative_z_down_m=rel_down,
                    distance_m=distance_3d(rel_right, rel_forward, rel_down),
                )
            )

        with self.lock:
            self.last_message_at = now
            self._prune_locked(now)
            used_tracks = set()
            for sample in samples:
                track = self._match_locked(sample, used_tracks)
                if track is None:
                    track = TargetTrack(self.next_track_id, sample, self.history_size)
                    self.tracks[track.track_id] = track
                    self.next_track_id += 1
                else:
                    track.update(sample)
                used_tracks.add(track.track_id)

    def _match_locked(self, sample, used_tracks):
        best = None
        best_distance = float("inf")
        for track in self.tracks.values():
            if track.track_id in used_tracks:
                continue
            stable = track.stable_xy()
            if stable is None:
                continue
            distance = math.hypot(
                sample.local_x_right_m - stable[0],
                sample.local_y_forward_m - stable[1],
            )
            if distance <= self.match_radius_m and distance < best_distance:
                best = track
                best_distance = distance
        return best

    def _prune_locked(self, now):
        stale_ids = [
            track_id
            for track_id, track in self.tracks.items()
            if now - track.last_seen > self.stale_timeout_s
        ]
        for track_id in stale_ids:
            del self.tracks[track_id]

    def prune(self):
        with self.lock:
            self._prune_locked(monotonic_s())

    def _blacklisted_locked(self, xy):
        return any(
            math.hypot(xy[0] - old[0], xy[1] - old[1])
            <= self.dropped_blacklist_radius_m
            for old in self.dropped_xy
        )

    def best_undropped(self):
        now = monotonic_s()
        with self.lock:
            self._prune_locked(now)
            candidates = []
            for track in self.tracks.values():
                xy = track.stable_xy()
                if track.hits < self.min_hits or xy is None or self._blacklisted_locked(xy):
                    continue
                candidates.append(track)
            if not candidates:
                return None
            return max(candidates, key=lambda track: track.score(now))

    def get(self, track_id):
        now = monotonic_s()
        with self.lock:
            self._prune_locked(now)
            return self.tracks.get(track_id)

    def mark_dropped(self, xy):
        if xy is None:
            return
        with self.lock:
            self.dropped_xy.append((float(xy[0]), float(xy[1])))

    def vision_alive(self, timeout_s):
        with self.lock:
            return self.last_message_at > 0.0 and monotonic_s() - self.last_message_at <= timeout_s


class AutonomousTwoDropMission:
    WAIT_READY = "WAIT_READY"
    TAKEOFF = "TAKEOFF"
    SEARCH_FORWARD = "SEARCH_FORWARD"
    AIM_DESCEND = "AIM_DESCEND"
    ALIGN_HOLD = "ALIGN_HOLD"
    DROP = "DROP"
    SHIFT_FOR_B = "SHIFT_FOR_B"
    CLIMB_RTL = "CLIMB_RTL"
    RTL = "RTL"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"
    PILOT_OVERRIDE = "PILOT_OVERRIDE"

    def __init__(self):
        self.execute_mission = get_bool_param("~execute_mission", False)
        self.calibration_confirmed = get_bool_param("~calibration_confirmed", False)
        self.authorization_required = get_bool_param("~authorization_required", True)
        self.authorization_phrase = str(rospy.get_param("~authorization_phrase", "FLY"))
        self.enable_servo_drop = get_bool_param("~enable_servo_drop", False)
        self.allow_inflight_dry_run = get_bool_param("~allow_inflight_dry_run", False)
        self.rtl_on_failure = get_bool_param("~rtl_on_failure", True)

        self.takeoff_altitude_m = float(rospy.get_param("~takeoff_altitude_m", 2.5))
        self.takeoff_tolerance_m = float(rospy.get_param("~takeoff_tolerance_m", 0.25))
        self.takeoff_timeout_s = float(rospy.get_param("~takeoff_timeout_s", 45.0))
        self.search_forward_distance_m = float(
            rospy.get_param("~search_forward_distance_m", 3.0)
        )
        self.search_forward_speed_mps = float(
            rospy.get_param("~search_forward_speed_mps", 0.20)
        )
        self.search_finish_timeout_s = float(
            rospy.get_param("~search_finish_timeout_s", 8.0)
        )
        self.aim_altitude_m = float(rospy.get_param("~aim_altitude_m", 1.2))
        self.rtl_climb_altitude_m = float(rospy.get_param("~rtl_climb_altitude_m", 3.0))

        self.arrival_tolerance_m = float(rospy.get_param("~arrival_tolerance_m", 0.35))
        self.arrival_hold_s = float(rospy.get_param("~arrival_hold_s", 1.0))
        self.align_threshold_x_m = float(rospy.get_param("~align_threshold_x_m", 0.20))
        self.align_threshold_y_m = float(rospy.get_param("~align_threshold_y_m", 0.20))
        self.align_threshold_z_m = float(rospy.get_param("~align_threshold_z_m", 0.15))
        self.align_stable_time_s = float(rospy.get_param("~align_stable_time_s", 2.0))
        self.max_visual_correction_step_m = float(
            rospy.get_param("~max_visual_correction_step_m", 0.60)
        )
        self.target_lost_timeout_s = float(rospy.get_param("~target_lost_timeout_s", 1.0))
        self.total_mission_timeout_s = float(rospy.get_param("~total_mission_timeout_s", 300.0))
        self.b_forward_shift_m = float(rospy.get_param("~b_forward_shift_m", 0.08))
        self.b_position_tolerance_m = float(
            rospy.get_param("~b_position_tolerance_m", 0.05)
        )
        self.b_stable_time_s = float(rospy.get_param("~b_stable_time_s", 2.0))
        self.loop_rate_hz = float(rospy.get_param("~loop_rate_hz", 10.0))

        self.connection_timeout_s = float(rospy.get_param("~connection_timeout_s", 30.0))
        self.vision_ready_timeout_s = float(rospy.get_param("~vision_ready_timeout_s", 30.0))
        self.service_timeout_s = float(rospy.get_param("~service_timeout_s", 10.0))
        self.pose_timeout_s = float(rospy.get_param("~pose_timeout_s", 1.0))
        self.mode_timeout_s = float(rospy.get_param("~mode_timeout_s", 6.0))
        self.arm_timeout_s = float(rospy.get_param("~arm_timeout_s", 6.0))
        self.max_yaw_deviation_deg = float(rospy.get_param("~max_yaw_deviation_deg", 12.0))

        self.camera_x_right_m = float(rospy.get_param("~camera_x_right_m", 0.0))
        self.camera_y_forward_m = float(rospy.get_param("~camera_y_forward_m", 0.0))
        self.camera_down_m = float(rospy.get_param("~camera_down_m", 0.40))

        raw_order = rospy.get_param("~drop_order", ["A", "B"])
        if isinstance(raw_order, str):
            raw_order = [part.strip() for part in raw_order.split(",") if part.strip()]
        self.drop_order = [str(label).strip().upper() for label in raw_order]

        self.servo_channels = {
            "A": int(rospy.get_param("~servo_a_channel", 5)),
            "B": int(rospy.get_param("~servo_b_channel", 6)),
        }
        self.servo_open_pwm = int(rospy.get_param("~servo_open_pwm", 1500))
        self.servo_close_pwm = int(rospy.get_param("~servo_close_pwm", 1000))
        self.servo_hold_s = float(rospy.get_param("~servo_hold_s", 0.8))
        self.servo_cooldown_s = float(rospy.get_param("~servo_cooldown_s", 2.0))

        tracker_params = {
            "min_confidence": float(rospy.get_param("~target_min_confidence", 0.60)),
            "min_depth_m": float(rospy.get_param("~target_min_depth_m", 0.30)),
            "max_depth_m": float(rospy.get_param("~target_max_depth_m", 8.0)),
            "min_hits": int(rospy.get_param("~target_min_hits", 4)),
            "history_size": int(rospy.get_param("~target_history_size", 7)),
            "match_radius_m": float(rospy.get_param("~target_match_radius_m", 0.70)),
            "stale_timeout_s": float(rospy.get_param("~target_stale_timeout_s", 1.5)),
            "dropped_blacklist_radius_m": float(
                rospy.get_param("~dropped_blacklist_radius_m", 0.55)
            ),
            "camera_x_right_m": self.camera_x_right_m,
            "camera_y_forward_m": self.camera_y_forward_m,
            "camera_down_m": self.camera_down_m,
        }
        self.tracker = VisionTargetTracker(tracker_params)

        self.data_lock = threading.Lock()
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.pose_received_at = 0.0
        self.state_received_at = 0.0
        self.camera_intrinsics = None

        rospy.Subscriber("/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self._pose_cb, queue_size=20)
        rospy.Subscriber("/vision/color/camera_info", CameraInfo, self._camera_info_cb, queue_size=2)
        rospy.Subscriber("/vision/yolo/detections", YoloDetections, self._detections_cb, queue_size=5)

        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=20
        )
        self.status_pub = rospy.Publisher("/autonomous_drop/status", String, queue_size=5)
        self.distance_pub = rospy.Publisher(
            "/autonomous_drop/target_distance_m", Float32, queue_size=5
        )
        self.offset_pub = rospy.Publisher(
            "/autonomous_drop/target_offset_xyz", Vector3Stamped, queue_size=5
        )
        self.mission_status_pub = rospy.Publisher(
            "/vision/mission_status", MissionStatus, queue_size=5
        )

        self._set_mode_srv = None
        self._arming_srv = None
        self._takeoff_srv = None
        self._command_srv = None

        self.state = self.WAIT_READY
        self.state_entered_at = monotonic_s()
        self.mission_started_at = 0.0
        self.home_xyz = None
        self.home_orientation = None
        self.home_yaw = None
        self.command_target_xyz = None
        self.arrived_since = None
        self.search_started_at = None
        self.search_start_xyz = None
        self.search_final_xyz = None
        self.search_command_finished_at = None
        self.locked_track_id = None
        self.aligned_since = None
        self.drop_index = 0
        self.completed_drops = []
        self.b_shift_target_xyz = None
        self.b_stable_since = None
        self.rtl_climb_target_xyz = None
        self.rtl_climb_arrived_since = None
        self.last_servo_action_at = 0.0
        self.last_drop_label = ""
        self.last_drop_display_until = 0.0
        self.failure_reason = ""
        self.last_alignment_error = None
        self.last_target_distance_m = None

    def _state_cb(self, msg):
        with self.data_lock:
            self.current_state = msg
            self.state_received_at = monotonic_s()

    def _pose_cb(self, msg):
        with self.data_lock:
            self.current_pose = msg
            self.pose_received_at = monotonic_s()

    def _camera_info_cb(self, msg):
        if len(msg.K) < 6:
            return
        fx, fy, cx, cy = float(msg.K[0]), float(msg.K[4]), float(msg.K[2]), float(msg.K[5])
        if finite(fx, fy, cx, cy) and fx > 0.0 and fy > 0.0:
            with self.data_lock:
                self.camera_intrinsics = (fx, fy, cx, cy)

    def _detections_cb(self, msg):
        with self.data_lock:
            pose_age = monotonic_s() - self.pose_received_at
            pose = self.current_pose
            intrinsics = self.camera_intrinsics
        if intrinsics is None or pose_age > self.pose_timeout_s:
            return
        p = pose.pose.position
        self.tracker.update(msg, (float(p.x), float(p.y), float(p.z)), intrinsics)

    def _snapshot(self):
        with self.data_lock:
            state = self.current_state
            pose = self.current_pose
            pose_age = monotonic_s() - self.pose_received_at
            state_age = monotonic_s() - self.state_received_at
            intrinsics = self.camera_intrinsics
        return state, pose, pose_age, state_age, intrinsics

    def _validate_parameters(self):
        errors = []
        if min(self.takeoff_altitude_m, self.aim_altitude_m, self.rtl_climb_altitude_m) <= 0.0:
            errors.append("takeoff, aim, and RTL climb altitudes must be positive")
        if self.aim_altitude_m >= self.takeoff_altitude_m:
            errors.append("aim_altitude_m must be below takeoff_altitude_m")
        if self.rtl_climb_altitude_m < self.takeoff_altitude_m:
            errors.append("rtl_climb_altitude_m must not be below takeoff altitude")
        if self.search_forward_distance_m <= 0.0:
            errors.append("search_forward_distance_m must be positive")
        if not 0.10 <= self.search_forward_speed_mps <= 0.20:
            errors.append("search_forward_speed_mps must stay in the documented 0.10-0.20 m/s range")
        if self.align_stable_time_s < 2.0:
            errors.append("align_stable_time_s must be at least the requested 2.0 seconds")
        elif self.align_stable_time_s > 2.0:
            rospy.logwarn(
                "align_stable_time_s=%.2f; this is longer than the requested 2.00 seconds",
                self.align_stable_time_s,
            )
        if self.b_forward_shift_m <= 0.0:
            errors.append("b_forward_shift_m must be positive")
        if self.b_stable_time_s <= 0.0:
            errors.append("b_stable_time_s must be positive")
        if len(self.drop_order) != 2 or sorted(self.drop_order) != ["A", "B"]:
            errors.append("drop_order must contain A and B exactly once")
        if self.execute_mission and not self.calibration_confirmed:
            errors.append("calibration_confirmed=false; refusing to arm")
        if self.execute_mission and not self.enable_servo_drop and not self.allow_inflight_dry_run:
            errors.append(
                "enable_servo_drop=false; set it true or explicitly allow_inflight_dry_run"
            )
        if self.max_visual_correction_step_m <= 0.0:
            errors.append("max_visual_correction_step_m must be positive")
        if self.max_yaw_deviation_deg <= 0.0:
            errors.append("max_yaw_deviation_deg must be positive")
        if errors:
            for error in errors:
                rospy.logerr("Configuration error: %s", error)
            return False
        return True

    def _summary_payload(self):
        return {
            "axis_convention": {"x": "right", "y": "forward", "z": "up"},
            "execute_mission": self.execute_mission,
            "calibration_confirmed": self.calibration_confirmed,
            "takeoff_altitude_m": self.takeoff_altitude_m,
            "search_forward_distance_m": self.search_forward_distance_m,
            "search_forward_speed_mps": self.search_forward_speed_mps,
            "aim_altitude_m": self.aim_altitude_m,
            "rtl_climb_altitude_m": self.rtl_climb_altitude_m,
            "drop_order": self.drop_order,
            "b_forward_shift_m": self.b_forward_shift_m,
            "camera_offset_xyz": [
                self.camera_x_right_m,
                self.camera_y_forward_m,
                -self.camera_down_m,
            ],
            "align_stable_time_s": self.align_stable_time_s,
            "align_xy_tolerance_m": [self.align_threshold_x_m, self.align_threshold_y_m],
            "real_servo": self.enable_servo_drop,
        }

    def _print_summary(self):
        print("\n================ autonomous two-drop mission ================", flush=True)
        print(json.dumps(self._summary_payload(), ensure_ascii=False, indent=2), flush=True)
        print("=============================================================\n", flush=True)

    def _wait_for_ready(self):
        deadline = monotonic_s() + max(self.connection_timeout_s, self.vision_ready_timeout_s)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            state, _pose, pose_age, state_age, intrinsics = self._snapshot()
            flight_ready = (
                state_age <= self.pose_timeout_s
                and bool(state.connected)
                and pose_age <= self.pose_timeout_s
            )
            vision_ready = intrinsics is not None and self.tracker.vision_alive(self.pose_timeout_s * 2.0)
            if flight_ready and vision_ready:
                return True
            if monotonic_s() >= deadline:
                rospy.logerr(
                    "Ready timeout: connected=%s pose_age=%.2f intrinsics=%s detector_alive=%s",
                    bool(state.connected),
                    pose_age,
                    intrinsics is not None,
                    self.tracker.vision_alive(self.pose_timeout_s * 2.0),
                )
                return False
            rospy.loginfo_throttle(
                2.0,
                "Waiting: FC=%s pose_fresh=%s camera_info=%s detector=%s",
                bool(state.connected),
                pose_age <= self.pose_timeout_s,
                intrinsics is not None,
                self.tracker.vision_alive(self.pose_timeout_s * 2.0),
            )
            rate.sleep()
        return False

    def _ensure_services(self):
        try:
            rospy.wait_for_service("/mavros/set_mode", timeout=self.service_timeout_s)
            rospy.wait_for_service("/mavros/cmd/arming", timeout=self.service_timeout_s)
            rospy.wait_for_service("/mavros/cmd/takeoff", timeout=self.service_timeout_s)
            self._set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
            self._arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
            self._takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
            if self.enable_servo_drop:
                rospy.wait_for_service("/mavros/cmd/command", timeout=self.service_timeout_s)
                self._command_srv = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
            return True
        except rospy.ROSException as exc:
            rospy.logerr("MAVROS service unavailable: %s", exc)
            return False

    def _authorize(self):
        if not self.authorization_required:
            rospy.logwarn("authorization_required=false")
            return True
        prompt = "Type %r to authorize ARM/TAKEOFF: " % self.authorization_phrase
        try:
            sys.stdout.write(prompt)
            sys.stdout.flush()
            try:
                with open("/dev/tty", "r", encoding="utf-8") as terminal:
                    response = terminal.readline()
            except OSError:
                response = sys.stdin.readline()
        except (EOFError, KeyboardInterrupt):
            return False
        return response.strip().casefold() == self.authorization_phrase.strip().casefold()

    def _wait_mode(self, expected):
        deadline = monotonic_s() + self.mode_timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            state, _pose, _pose_age, _state_age, _intrinsics = self._snapshot()
            if state.mode == expected:
                return True
            if monotonic_s() >= deadline:
                return False
            rate.sleep()
        return False

    def _set_mode(self, mode):
        if self._set_mode_srv is None:
            return False
        try:
            response = self._set_mode_srv(custom_mode=mode)
        except rospy.ServiceException as exc:
            rospy.logerr("set_mode(%s) failed: %s", mode, exc)
            return False
        if not response.mode_sent or not self._wait_mode(mode):
            rospy.logerr("Mode did not become %s", mode)
            return False
        rospy.loginfo("Mode confirmed: %s", mode)
        return True

    def _arm(self):
        try:
            response = self._arming_srv(value=True)
        except rospy.ServiceException as exc:
            rospy.logerr("Arming call failed: %s", exc)
            return False
        if not response.success:
            rospy.logerr("Arming rejected: result=%s", getattr(response, "result", -1))
            return False
        deadline = monotonic_s() + self.arm_timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            state, _pose, _pose_age, _state_age, _intrinsics = self._snapshot()
            if state.armed:
                rospy.loginfo("Armed confirmed")
                return True
            if monotonic_s() >= deadline:
                return False
            rate.sleep()
        return False

    def _takeoff(self):
        try:
            response = self._takeoff_srv(
                altitude=self.takeoff_altitude_m,
                latitude=0.0,
                longitude=0.0,
                min_pitch=0.0,
                yaw=0.0,
            )
        except rospy.ServiceException as exc:
            rospy.logerr("Takeoff call failed: %s", exc)
            return False
        if not response.success:
            rospy.logerr("Takeoff rejected: result=%s", getattr(response, "result", -1))
            return False

        self._transition(self.TAKEOFF)
        target_z = self.home_xyz[2] + self.takeoff_altitude_m
        deadline = monotonic_s() + self.takeoff_timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            state, pose, pose_age, _state_age, _intrinsics = self._snapshot()
            if not state.connected or not state.armed or pose_age > self.pose_timeout_s:
                return False
            if state.mode != "GUIDED":
                self._pilot_override("mode changed during takeoff")
                return False
            current_z = float(pose.pose.position.z)
            if current_z >= target_z - self.takeoff_tolerance_m:
                rospy.loginfo("Takeoff altitude reached: z=%.2f target=%.2f", current_z, target_z)
                return True
            if monotonic_s() >= deadline:
                rospy.logerr("Takeoff altitude timeout: z=%.2f target=%.2f", current_z, target_z)
                return False
            rospy.loginfo_throttle(2.0, "Climbing z=%.2f / %.2f", current_z, target_z)
            rate.sleep()
        return False

    def _transition(self, new_state, reason=""):
        old_state = self.state
        self.state = new_state
        self.state_entered_at = monotonic_s()
        rospy.loginfo(
            "MISSION STATE %s -> %s%s",
            old_state,
            new_state,
            (" | " + reason) if reason else "",
        )

    def _pilot_override(self, reason):
        self.failure_reason = reason
        self.command_target_xyz = None
        self._transition(self.PILOT_OVERRIDE, reason)
        rospy.logwarn("Pilot override detected; no further mode or setpoint commands will be sent")

    def _fail(self, reason):
        self.failure_reason = reason
        self.command_target_xyz = None
        self._transition(self.FAILED, reason)
        state, _pose, _pose_age, _state_age, _intrinsics = self._snapshot()
        if (
            self.rtl_on_failure
            and state.connected
            and state.armed
            and state.mode == "GUIDED"
            and self._set_mode_srv is not None
        ):
            if self._set_mode("RTL"):
                rospy.logwarn("Failsafe requested RTL")
        self._transition(self.COMPLETE, reason)

    def _aim_z(self):
        return self.home_xyz[2] + self.aim_altitude_m

    def _search_z(self):
        return self.home_xyz[2] + self.takeoff_altitude_m

    def _rtl_climb_z(self):
        return self.home_xyz[2] + self.rtl_climb_altitude_m

    def _pose_error(self, target_xyz, pose):
        p = pose.pose.position
        return (
            float(target_xyz[0]) - float(p.x),
            float(target_xyz[1]) - float(p.y),
            float(target_xyz[2]) - float(p.z),
        )

    def _publish_setpoint(self, target_xyz):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.current_pose.header.frame_id or "map"
        msg.pose.position.x = float(target_xyz[0])
        msg.pose.position.y = float(target_xyz[1])
        msg.pose.position.z = float(target_xyz[2])
        msg.pose.orientation = self.home_orientation
        self.setpoint_pub.publish(msg)
        self.command_target_xyz = tuple(target_xyz)

    def _current_dropper(self):
        if self.drop_index >= len(self.drop_order):
            return None
        return self.drop_order[self.drop_index]

    def _step_search_forward(self, pose):
        track = self.tracker.best_undropped()
        if track is not None:
            self.locked_track_id = track.track_id
            self.aligned_since = None
            self.last_alignment_error = None
            self._transition(
                self.AIM_DESCEND,
                "stable target detected, track=%d" % track.track_id,
            )
            return

        elapsed = max(0.0, monotonic_s() - self.search_started_at)
        commanded_forward_m = ramped_progress(
            elapsed,
            self.search_forward_speed_mps,
            self.search_forward_distance_m,
        )
        target = (
            self.search_start_xyz[0],
            self.search_start_xyz[1] + commanded_forward_m,
            self.search_start_xyz[2],
        )
        self._publish_setpoint(target)

        if commanded_forward_m >= self.search_forward_distance_m:
            if self.search_command_finished_at is None:
                self.search_command_finished_at = monotonic_s()
            error = self._pose_error(self.search_final_xyz, pose)
            arrived = (
                abs(error[0]) <= self.arrival_tolerance_m
                and abs(error[1]) <= self.arrival_tolerance_m
                and abs(error[2]) <= self.takeoff_tolerance_m
            )
            if arrived:
                if self.arrived_since is None:
                    self.arrived_since = monotonic_s()
                if monotonic_s() - self.arrived_since >= self.arrival_hold_s:
                    self._begin_climb_rtl("search ended with no target", pose)
                    return
            else:
                self.arrived_since = None
            if monotonic_s() - self.search_command_finished_at > self.search_finish_timeout_s:
                self._fail("could_not_finish_3m_forward_search")
                return

        rospy.loginfo_throttle(
            1.0,
            "SEARCH +Y command=%.2f/%.2fm reference_speed=%.2fm/s target=none",
            commanded_forward_m,
            self.search_forward_distance_m,
            self.search_forward_speed_mps,
        )

    def _step_aim(self, pose):
        track = self.tracker.get(self.locked_track_id)
        if track is None or monotonic_s() - track.last_seen > self.target_lost_timeout_s:
            self.locked_track_id = None
            self.aligned_since = None
            self._begin_climb_rtl("target lost during aiming", pose)
            return

        target_xy = track.stable_xy()
        latest = track.latest()
        if target_xy is None or latest is None:
            self._begin_climb_rtl("target position unavailable", pose)
            return

        p = pose.pose.position
        # Force the detector/camera-centred horizontal values toward zero.
        # target_xy contains the camera installation translation, therefore
        # the desired aircraft pose subtracts that translation again.
        control_error_x = target_xy[0] - self.camera_x_right_m - float(p.x)
        control_error_y = target_xy[1] - self.camera_y_forward_m - float(p.y)
        # Latest camera-centred values are used for the actual +/-0.20 m
        # release gate; the median world estimate above is used for control.
        error_x = latest.relative_x_right_m - self.camera_x_right_m
        error_y = latest.relative_y_forward_m - self.camera_y_forward_m
        error_z = self._aim_z() - float(p.z)
        self.last_alignment_error = (error_x, error_y, error_z)
        self.last_target_distance_m = latest.distance_m

        correction_x, correction_y = limit_vector_2d(
            control_error_x, control_error_y, self.max_visual_correction_step_m
        )
        command = (
            float(p.x) + correction_x,
            float(p.y) + correction_y,
            self._aim_z(),
        )
        self._publish_setpoint(command)

        aligned = (
            abs(error_x) <= self.align_threshold_x_m
            and abs(error_y) <= self.align_threshold_y_m
            and abs(error_z) <= self.align_threshold_z_m
        )
        if aligned:
            if self.aligned_since is None:
                self.aligned_since = monotonic_s()
                self._transition(self.ALIGN_HOLD, "alignment entered")
            stable_for = monotonic_s() - self.aligned_since
            if stable_for >= self.align_stable_time_s:
                self._transition(self.DROP, "x/y near zero and height 1.2m for %.2fs" % stable_for)
        else:
            if self.aligned_since is not None:
                rospy.loginfo("Alignment left tolerance; 2-second timer reset")
            self.aligned_since = None
            if self.state == self.ALIGN_HOLD:
                self._transition(self.AIM_DESCEND, "alignment error exceeded threshold")

        rospy.loginfo_throttle(
            0.5,
            "AIM_DESCEND track=%d err=(%+.3f,%+.3f,%+.3f)m distance=%.3fm stable=%.2f/%.2fs",
            track.track_id,
            error_x,
            error_y,
            error_z,
            latest.distance_m,
            0.0 if self.aligned_since is None else monotonic_s() - self.aligned_since,
            self.align_stable_time_s,
        )

    def _set_servo_pwm(self, channel, pwm):
        if self._command_srv is None:
            return False
        try:
            response = self._command_srv(
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
            rospy.logerr("Servo command failed: %s", exc)
            return False

    def _execute_drop(self, label):
        if monotonic_s() - self.last_servo_action_at < self.servo_cooldown_s:
            rospy.logerr("Servo cooldown has not elapsed")
            return False
        if not self.enable_servo_drop:
            rospy.logwarn("IN-FLIGHT DRY RUN: %s drop recorded without moving a servo", label)
            self.last_servo_action_at = monotonic_s()
            return self.allow_inflight_dry_run
        channel = self.servo_channels[label]
        if not self._set_servo_pwm(channel, self.servo_open_pwm):
            return False
        rospy.sleep(self.servo_hold_s)
        close_ok = self._set_servo_pwm(channel, self.servo_close_pwm)
        self.last_servo_action_at = monotonic_s()
        if not close_ok:
            rospy.logerr("Dropper %s opened but failed to close", label)
        return close_ok

    def _step_drop(self):
        state, pose, _pose_age, _state_age, _intrinsics = self._snapshot()
        if state.mode != "GUIDED" or not state.armed:
            self._pilot_override("vehicle not armed in GUIDED at drop")
            return
        label = self._current_dropper()
        if label == "A":
            track = self.tracker.get(self.locked_track_id)
            if track is None:
                self._begin_climb_rtl("target disappeared before A drop", pose)
                return
            p = pose.pose.position
            latest = track.latest()
            if latest is None:
                self._begin_climb_rtl("target sample unavailable before A drop", pose)
                return
            live_x = latest.relative_x_right_m - self.camera_x_right_m
            live_y = latest.relative_y_forward_m - self.camera_y_forward_m
            release_gate_ok = (
                abs(live_x) <= self.align_threshold_x_m
                and abs(live_y) <= self.align_threshold_y_m
                and abs(float(p.z) - self._aim_z()) <= self.align_threshold_z_m
            )
            if not release_gate_ok:
                self.aligned_since = None
                self._transition(self.AIM_DESCEND, "A release gate changed before servo command")
                return
        if not self._execute_drop(label):
            self._fail("dropper_%s_failed" % label.lower())
            return
        self.completed_drops.append(label)
        self.last_drop_label = label
        self.last_drop_display_until = monotonic_s() + 3.0
        self.drop_index += 1
        self.aligned_since = None
        if label == "A":
            p = pose.pose.position
            self.b_shift_target_xyz = (
                float(p.x),
                float(p.y) + self.b_forward_shift_m,
                self._aim_z(),
            )
            self.b_stable_since = None
            self._transition(self.SHIFT_FOR_B, "A complete; moving +Y %.2fm" % self.b_forward_shift_m)
        else:
            self.locked_track_id = None
            self._begin_climb_rtl("A and B drops complete", pose)

    def _step_shift_for_b(self, pose):
        self._publish_setpoint(self.b_shift_target_xyz)
        error = self._pose_error(self.b_shift_target_xyz, pose)
        stable = (
            abs(error[0]) <= self.b_position_tolerance_m
            and abs(error[1]) <= self.b_position_tolerance_m
            and abs(error[2]) <= self.align_threshold_z_m
        )
        if stable:
            if self.b_stable_since is None:
                self.b_stable_since = monotonic_s()
            if monotonic_s() - self.b_stable_since >= self.b_stable_time_s:
                self._transition(self.DROP, "B position stable for %.2fs" % self.b_stable_time_s)
        else:
            self.b_stable_since = None
        rospy.loginfo_throttle(
            0.5,
            "SHIFT_FOR_B err=(%+.3f,%+.3f,%+.3f)m stable=%.2f/%.2fs",
            error[0],
            error[1],
            error[2],
            0.0 if self.b_stable_since is None else monotonic_s() - self.b_stable_since,
            self.b_stable_time_s,
        )

    def _begin_climb_rtl(self, reason, pose):
        p = pose.pose.position
        self.rtl_climb_target_xyz = (float(p.x), float(p.y), self._rtl_climb_z())
        self.rtl_climb_arrived_since = None
        self._transition(self.CLIMB_RTL, reason)

    def _step_climb_rtl(self, pose):
        self._publish_setpoint(self.rtl_climb_target_xyz)
        error = self._pose_error(self.rtl_climb_target_xyz, pose)
        arrived = (
            abs(error[0]) <= self.arrival_tolerance_m
            and abs(error[1]) <= self.arrival_tolerance_m
            and abs(error[2]) <= self.takeoff_tolerance_m
        )
        if arrived:
            if self.rtl_climb_arrived_since is None:
                self.rtl_climb_arrived_since = monotonic_s()
            if monotonic_s() - self.rtl_climb_arrived_since >= self.arrival_hold_s:
                self._transition(self.RTL, "3.0m climb complete")
        else:
            self.rtl_climb_arrived_since = None
        rospy.loginfo_throttle(
            1.0,
            "CLIMB_RTL error=(%+.2f,%+.2f,%+.2f)m",
            error[0], error[1], error[2],
        )

    def _step_rtl(self):
        self.command_target_xyz = None
        if self._set_mode("RTL"):
            self._transition(self.COMPLETE, "RTL accepted")
        else:
            rospy.logerr("RTL request failed; requesting LAND")
            self._set_mode("LAND")
            self._transition(self.COMPLETE, "RTL failed, LAND requested")

    def _yaw_safe(self, pose):
        q = pose.pose.orientation
        current_yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
        deviation = abs(math.degrees(angular_difference(current_yaw, self.home_yaw)))
        if deviation > self.max_yaw_deviation_deg:
            self._fail("yaw_deviation_%.1f_deg" % deviation)
            return False
        return True

    def _flight_safety_ok(self, state, pose, pose_age):
        if not state.connected:
            self._fail("flight_controller_disconnected")
            return False
        if pose_age > self.pose_timeout_s:
            self._fail("local_pose_stale")
            return False
        if not state.armed:
            self._fail("unexpected_disarm")
            return False
        if state.mode != "GUIDED":
            self._pilot_override("mode changed from GUIDED to %s" % state.mode)
            return False
        if self.state in (
            self.SEARCH_FORWARD,
            self.AIM_DESCEND,
            self.ALIGN_HOLD,
            self.DROP,
            self.SHIFT_FOR_B,
        ):
            return self._yaw_safe(pose)
        return True

    def _publish_target_measurement(self, pose):
        if self.locked_track_id is None:
            return
        track = self.tracker.get(self.locked_track_id)
        if track is None:
            return
        xy = track.stable_xy()
        latest = track.latest()
        if xy is None or latest is None:
            return
        p = pose.pose.position
        right_m = xy[0] - float(p.x)
        forward_m = xy[1] - float(p.y)
        down_m = latest.relative_z_down_m
        distance_m = distance_3d(right_m, forward_m, down_m)
        self.last_target_distance_m = distance_m
        self.distance_pub.publish(Float32(data=float(distance_m)))
        vector = Vector3Stamped()
        vector.header.stamp = rospy.Time.now()
        vector.header.frame_id = "mission_local_x_right_y_forward_z_up"
        vector.vector.x = right_m
        vector.vector.y = forward_m
        vector.vector.z = -down_m
        self.offset_pub.publish(vector)

    def _publish_status(self, pose=None):
        current_dropper = self._current_dropper()
        status = {
            "state": self.state,
            "axis": "x_right_y_forward_z_up",
            "current_dropper": current_dropper,
            "completed_drops": list(self.completed_drops),
            "locked_track_id": self.locked_track_id,
            "target_distance_m": self.last_target_distance_m,
            "alignment_error_xyz_m": self.last_alignment_error,
            "failure_reason": self.failure_reason,
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False, sort_keys=True)))

        mission = MissionStatus()
        mission.ammo_a = 0 if "A" in self.completed_drops else 1
        mission.ammo_b = 0 if "B" in self.completed_drops else 1
        mission.aiming = self.state in (
            self.AIM_DESCEND,
            self.ALIGN_HOLD,
            self.SHIFT_FOR_B,
        )
        mission.last_drop = (
            self.last_drop_label
            if self.last_drop_label and monotonic_s() <= self.last_drop_display_until
            else ""
        )
        self.mission_status_pub.publish(mission)
        if pose is not None:
            self._publish_target_measurement(pose)

    def run(self):
        self._print_summary()
        if not self._validate_parameters():
            return 2
        if not self.execute_mission:
            rospy.logwarn(
                "SAFE PREVIEW ONLY: execute_mission=false. No mode, arm, setpoint, or servo command sent."
            )
            return 0
        if not self._wait_for_ready() or not self._ensure_services():
            return 3

        _state, pose, _pose_age, _state_age, _intrinsics = self._snapshot()
        p = pose.pose.position
        q = pose.pose.orientation
        self.home_xyz = (float(p.x), float(p.y), float(p.z))
        self.home_orientation = q
        self.home_yaw = quaternion_yaw(q.x, q.y, q.z, q.w)

        self._print_summary()
        rospy.logwarn(
            "Home=(%.3f, %.3f, %.3f); search will move +Y %.2fm at %.2fm/s",
            self.home_xyz[0],
            self.home_xyz[1],
            self.home_xyz[2],
            self.search_forward_distance_m,
            self.search_forward_speed_mps,
        )
        if not self._authorize():
            rospy.logwarn("Pilot did not authorize mission")
            return 4
        if not self._set_mode("GUIDED") or not self._arm() or not self._takeoff():
            if self.state != self.PILOT_OVERRIDE:
                self._fail("preflight_or_takeoff_failed")
            return 5

        self.mission_started_at = monotonic_s()
        self.arrived_since = None
        self.search_started_at = monotonic_s()
        self.search_start_xyz = (
            self.home_xyz[0],
            self.home_xyz[1],
            self._search_z(),
        )
        self.search_final_xyz = (
            self.search_start_xyz[0],
            self.search_start_xyz[1] + self.search_forward_distance_m,
            self.search_start_xyz[2],
        )
        self.search_command_finished_at = None
        self._transition(self.SEARCH_FORWARD)
        rate = rospy.Rate(self.loop_rate_hz)

        while not rospy.is_shutdown() and self.state not in (
            self.COMPLETE,
            self.PILOT_OVERRIDE,
        ):
            state, pose, pose_age, _state_age, _intrinsics = self._snapshot()
            if monotonic_s() - self.mission_started_at > self.total_mission_timeout_s:
                self._fail("total_mission_timeout")
                break
            if not self._flight_safety_ok(state, pose, pose_age):
                break

            if self.state == self.SEARCH_FORWARD:
                self._step_search_forward(pose)
            elif self.state in (self.AIM_DESCEND, self.ALIGN_HOLD):
                self._step_aim(pose)
            elif self.state == self.DROP:
                self._step_drop()
            elif self.state == self.SHIFT_FOR_B:
                self._step_shift_for_b(pose)
            elif self.state == self.CLIMB_RTL:
                self._step_climb_rtl(pose)
            elif self.state == self.RTL:
                self._step_rtl()

            self._publish_status(pose)
            rate.sleep()

        self._publish_status()
        if self.state == self.PILOT_OVERRIDE:
            return 6
        return 0 if self.state == self.COMPLETE and not self.failure_reason else 7


def main():
    rospy.init_node("autonomous_two_drop_mission")
    try:
        node = AutonomousTwoDropMission()
        exit_code = node.run()
    except (ValueError, TypeError) as exc:
        rospy.logerr("Mission configuration error: %s", exc)
        exit_code = 2
    except rospy.ROSInterruptException:
        exit_code = 130
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
