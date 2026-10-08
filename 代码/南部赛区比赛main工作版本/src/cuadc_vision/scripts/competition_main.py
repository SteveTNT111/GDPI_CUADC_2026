#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time

import rospy

from competition_mission_common import (
    DropperManager,
    FlightIO,
    TargetTracker,
    get_bool_param,
    now_s,
)


class CompetitionMain:
    INIT = "INIT"
    WAIT_READY = "WAIT_READY"
    PREARM = "PREARM"
    TAKEOFF = "TAKEOFF"
    GOTO_DROP_ZONE = "GOTO_DROP_ZONE"
    DROP_ZONE_SEARCH = "DROP_ZONE_SEARCH"
    TARGET_SELECT = "TARGET_SELECT"
    AIMING = "AIMING"
    DROP = "DROP"
    POST_DROP = "POST_DROP"
    GOTO_RECON_ZONE = "GOTO_RECON_ZONE"
    RECON_OBSERVE = "RECON_OBSERVE"
    RTL = "RTL"
    COMPLETE = "COMPLETE"
    FAILSAFE = "FAILSAFE"
    ABORT = "ABORT"

    def __init__(self):
        self.flight = FlightIO()
        self.tracker = TargetTracker(self.flight)
        self.dropper = DropperManager(self.flight)

        self.state = self.INIT
        self.state_entered_at = now_s()
        self.abort_reason = ""

        self.ground_test = get_bool_param("~ground_test", False)
        self.enable_takeoff = get_bool_param("~enable_takeoff", False)
        self.require_authorization = get_bool_param("~require_authorization", True)
        self.authorization_phrase = str(rospy.get_param("~authorization_phrase", "YES"))

        self.takeoff_altitude = float(rospy.get_param("~takeoff_altitude", 3.8))
        self.post_arm_altitude_settle_s = float(rospy.get_param("~post_arm_altitude_settle_s", 2.0))

        self.arrival_tolerance_m = float(rospy.get_param("~arrival_tolerance_m", 0.45))
        self.arrival_hold_s = float(rospy.get_param("~arrival_hold_s", 1.0))
        self.search_waypoint_hold_s = float(rospy.get_param("~search_waypoint_hold_s", 1.5))
        self.max_search_laps = int(rospy.get_param("~max_search_laps", 3))

        self.aiming_altitude_m = float(rospy.get_param("~aiming_altitude_m", 2.0))
        self.aiming_threshold_m = float(rospy.get_param("~aiming_threshold_m", 0.18))
        self.aiming_stable_time_s = float(rospy.get_param("~aiming_stable_time_s", 1.5))
        self.aiming_lost_timeout_s = float(rospy.get_param("~aiming_lost_timeout_s", 2.0))
        self.recon_observe_s = float(rospy.get_param("~recon_observe_s", 6.0))

        self.drop_zone_entry_offset_enu = self._param_vec("~drop_zone_entry_enu", [0.0, 0.0, self.takeoff_altitude])
        self.recon_zone_offset_enu = self._param_vec("~recon_zone_enu", [12.0, 0.0, max(5.0, self.takeoff_altitude)])
        self.search_offsets_enu = self._param_waypoints(
            "~search_waypoints_enu",
            [
                [-2.7, -1.5, self.takeoff_altitude],
                [0.0, 1.5, self.takeoff_altitude],
                [2.7, -1.5, self.takeoff_altitude],
            ],
        )

        self.use_wgs84_drop_zone = get_bool_param("~use_wgs84_drop_zone", False)
        self.drop_zone_lat = rospy.get_param("~drop_zone_lat", None)
        self.drop_zone_lon = rospy.get_param("~drop_zone_lon", None)
        self.drop_zone_rel_alt = float(rospy.get_param("~drop_zone_rel_alt", self.takeoff_altitude))
        self.use_wgs84_recon_zone = get_bool_param("~use_wgs84_recon_zone", False)
        self.recon_zone_lat = rospy.get_param("~recon_zone_lat", None)
        self.recon_zone_lon = rospy.get_param("~recon_zone_lon", None)
        self.recon_zone_rel_alt = float(rospy.get_param("~recon_zone_rel_alt", max(5.0, self.takeoff_altitude)))

        self.active_target_kind = None
        self.active_local_target = None
        self.active_global_target = None
        self.arrival_since = None

        self.search_index = 0
        self.search_laps = 0
        self.search_hold_until = 0.0

        self.locked_track_id = None
        self.locked_target_enu = None
        self.locked_at = 0.0
        self.aligned_since = None

    def _param_vec(self, name, default):
        value = rospy.get_param(name, default)
        if len(value) != 3:
            raise ValueError("%s must contain [east, north, up]" % name)
        return [float(value[0]), float(value[1]), float(value[2])]

    def _param_waypoints(self, name, default):
        value = rospy.get_param(name, default)
        return [self._coerce_waypoint(name, item) for item in value]

    def _coerce_waypoint(self, name, item):
        if len(item) != 3:
            raise ValueError("%s waypoint must contain [east, north, up]" % name)
        return [float(item[0]), float(item[1]), float(item[2])]

    def transition(self, new_state, reason=""):
        old = self.state
        self.state = new_state
        self.state_entered_at = now_s()
        self.arrival_since = None
        rospy.loginfo("MAIN STATE %s -> %s %s", old, new_state, ("| " + reason) if reason else "")

    def state_age(self):
        return now_s() - self.state_entered_at

    def run(self):
        rate = rospy.Rate(float(rospy.get_param("~main_rate", 10.0)))
        while not rospy.is_shutdown():
            try:
                self.step()
            except Exception as exc:
                rospy.logerr("Unhandled mission exception: %s", exc)
                self.abort_reason = str(exc)
                self.transition(self.FAILSAFE)
            self.publish_status()
            if self.state == self.COMPLETE:
                rospy.loginfo("Competition main complete")
                return 0
            rate.sleep()
        return 1

    def step(self):
        if self.state == self.INIT:
            if self.flight.wait_for_connection() and self.flight.ensure_services():
                self.transition(self.WAIT_READY)
            else:
                self.abort_reason = "mavros_not_ready"
                self.transition(self.ABORT)

        elif self.state == self.WAIT_READY:
            if not self.flight.wait_for_navigation():
                self.abort_reason = "navigation_not_ready"
                self.transition(self.ABORT)
                return
            if not self.flight.capture_start_reference():
                self.abort_reason = "start_reference_failed"
                self.transition(self.ABORT)
                return
            self.print_authorization_summary()
            if not self.request_authorization():
                self.abort_reason = "pilot_cancelled"
                self.transition(self.ABORT)
                return
            self.transition(self.PREARM)

        elif self.state == self.PREARM:
            if self.ground_test:
                rospy.logwarn("ground_test=true: no GUIDED/ARM/TAKEOFF/setpoint/drop commands will be sent")
                self.transition(self.DROP_ZONE_SEARCH)
                return
            if not self.enable_takeoff:
                self.abort_reason = "enable_takeoff_false"
                self.transition(self.ABORT)
                return
            if not self.flight.set_mode("GUIDED"):
                self.abort_reason = "guided_failed"
                self.transition(self.FAILSAFE)
                return
            if not self.flight.arm():
                self.abort_reason = "arm_failed"
                self.transition(self.ABORT)
                return
            if not self.flight.capture_ground_altitude_reference(
                self.takeoff_altitude, self.post_arm_altitude_settle_s
            ):
                self.abort_reason = "ground_altitude_failed"
                self.transition(self.FAILSAFE)
                return
            if not self.flight.takeoff():
                self.abort_reason = "takeoff_command_failed"
                self.transition(self.FAILSAFE)
                return
            self.transition(self.TAKEOFF)

        elif self.state == self.TAKEOFF:
            if self.flight.wait_takeoff_altitude():
                self.transition(self.GOTO_DROP_ZONE)
            else:
                self.abort_reason = "takeoff_altitude_failed"
                self.transition(self.FAILSAFE)

        elif self.state == self.GOTO_DROP_ZONE:
            if self.fly_zone("drop"):
                self.transition(self.DROP_ZONE_SEARCH)

        elif self.state == self.DROP_ZONE_SEARCH:
            if self.dropper.total_ammo() <= 0:
                self.transition(self.GOTO_RECON_ZONE, "ammo empty")
                return
            if self.tracker.best_track() is not None:
                self.transition(self.TARGET_SELECT)
                return
            if self.ground_test:
                rospy.loginfo_throttle(3.0, "Ground-test search: waiting for stable target")
                return
            if self.search_laps >= self.max_search_laps:
                self.transition(self.GOTO_RECON_ZONE, "search laps exhausted")
                return
            self.fly_search_waypoint()

        elif self.state == self.TARGET_SELECT:
            track = self.tracker.best_track()
            if track is None:
                self.transition(self.DROP_ZONE_SEARCH, "no stable target")
                return
            self.locked_track_id = track.track_id
            self.locked_target_enu = track.stable_enu()
            self.locked_at = now_s()
            self.aligned_since = None
            rospy.loginfo(
                "Locked target track=%s class=%s hits=%d enu=(%.2f, %.2f, %.2f)",
                track.track_id,
                track.class_name,
                track.hits,
                self.locked_target_enu[0],
                self.locked_target_enu[1],
                self.locked_target_enu[2],
            )
            self.transition(self.AIMING)

        elif self.state == self.AIMING:
            if self.locked_target_enu is None:
                self.transition(self.DROP_ZONE_SEARCH, "no locked target")
                return
            if now_s() - self.tracker.last_detections_stamp > self.aiming_lost_timeout_s:
                self.locked_target_enu = None
                self.transition(self.DROP_ZONE_SEARCH, "target lost")
                return
            if self.ground_test:
                rospy.loginfo_throttle(1.0, "Ground-test AIMING: target locked, no setpoint sent")
                return
            if self.flight.current_state.mode != "GUIDED" and not self.flight.set_mode("GUIDED"):
                self.abort_reason = "guided_failed_during_aim"
                self.transition(self.FAILSAFE)
                return
            aim_target = self.dropper.compensated_target_enu(self.locked_target_enu)
            start_z = self.flight.start_pose.pose.position.z if self.flight.start_pose else 0.0
            aim_target = (aim_target[0], aim_target[1], start_z + self.aiming_altitude_m)
            self.flight.publish_local_target(aim_target)
            horizontal_error = self.flight.horizontal_error(aim_target)
            if horizontal_error <= self.aiming_threshold_m and self.dropper.cooldown_ready():
                if self.aligned_since is None:
                    self.aligned_since = now_s()
                if now_s() - self.aligned_since >= self.aiming_stable_time_s:
                    self.transition(self.DROP)
            else:
                self.aligned_since = None
            rospy.loginfo_throttle(
                1.0,
                "AIMING dropper=%s h_error=%.2f threshold=%.2f stable=%.1f/%.1f",
                self.dropper.current_label(),
                horizontal_error,
                self.aiming_threshold_m,
                0.0 if self.aligned_since is None else now_s() - self.aligned_since,
                self.aiming_stable_time_s,
            )

        elif self.state == self.DROP:
            if self.flight.current_state.mode != "GUIDED":
                self.abort_reason = "not_guided_at_drop"
                self.transition(self.FAILSAFE)
                return
            if self.locked_target_enu is None or self.dropper.total_ammo() <= 0:
                self.transition(self.POST_DROP, "drop skipped")
                return
            if self.dropper.execute_drop():
                self.transition(self.POST_DROP)
            else:
                self.abort_reason = "drop_failed"
                self.transition(self.FAILSAFE)

        elif self.state == self.POST_DROP:
            self.tracker.mark_dropped(self.locked_target_enu)
            self.locked_track_id = None
            self.locked_target_enu = None
            self.aligned_since = None
            if self.dropper.total_ammo() <= 0:
                self.transition(self.GOTO_RECON_ZONE, "ammo empty")
            else:
                self.transition(self.DROP_ZONE_SEARCH)

        elif self.state == self.GOTO_RECON_ZONE:
            if self.ground_test:
                self.transition(self.COMPLETE, "ground test no recon flight")
                return
            if self.fly_zone("recon"):
                self.transition(self.RECON_OBSERVE)

        elif self.state == self.RECON_OBSERVE:
            if self.hold_active_target(self.recon_observe_s):
                self.transition(self.RTL)

        elif self.state == self.RTL:
            if self.ground_test:
                self.transition(self.COMPLETE)
            elif self.flight.set_mode("RTL"):
                self.transition(self.COMPLETE)
            else:
                self.flight.set_mode("LAND")
                self.transition(self.COMPLETE, "rtl_failed_land_requested")

        elif self.state == self.FAILSAFE:
            if not self.ground_test and self.flight.current_state.armed:
                if not self.flight.set_mode("RTL"):
                    self.flight.set_mode("LAND")
            self.transition(self.COMPLETE, self.abort_reason)

        elif self.state == self.ABORT:
            if not self.ground_test and self.flight.current_state.armed:
                self.flight.set_mode("LAND")
            self.transition(self.COMPLETE, self.abort_reason)

    def print_authorization_summary(self):
        pose = self.flight.current_pose.pose.position
        fix = self.flight.current_fix
        lines = [
            "",
            "================ CUADC competition main authorization ================",
            "state=%s mode=%s armed=%s" % (self.state, self.flight.current_state.mode, self.flight.current_state.armed),
            "local ENU: E=%.3f N=%.3f U=%.3f" % (pose.x, pose.y, pose.z),
            "WGS84: lat=%.9f lon=%.9f alt=%.3f" % (fix.latitude, fix.longitude, fix.altitude),
            "takeoff_altitude=%.2f enable_takeoff=%s ground_test=%s" % (
                self.takeoff_altitude,
                self.enable_takeoff,
                self.ground_test,
            ),
            "ammo: A=%d B=%d real_servo=%s" % (
                self.dropper.ammo_a,
                self.dropper.ammo_b,
                self.dropper.enable_servo_drop,
            ),
            "Type %r to authorize; any other input cancels before ARM/TAKEOFF." % self.authorization_phrase,
            "======================================================================",
        ]
        print("\n".join(lines), flush=True)

    def request_authorization(self):
        if not self.require_authorization:
            rospy.logwarn("require_authorization=false")
            return True
        try:
            sys.stdout.write("Authorization phrase: ")
            sys.stdout.flush()
            try:
                with open("/dev/tty", "r", encoding="utf-8") as terminal:
                    response = terminal.readline()
            except OSError:
                response = sys.stdin.readline()
        except (EOFError, KeyboardInterrupt):
            print("")
            return False
        return response.strip().casefold() == self.authorization_phrase.strip().casefold()

    def fly_zone(self, zone):
        if zone == "drop":
            use_global = self.use_wgs84_drop_zone and self.drop_zone_lat is not None and self.drop_zone_lon is not None
            local_offset = self.drop_zone_entry_offset_enu
            lat, lon, alt = self.drop_zone_lat, self.drop_zone_lon, self.drop_zone_rel_alt
        else:
            use_global = self.use_wgs84_recon_zone and self.recon_zone_lat is not None and self.recon_zone_lon is not None
            local_offset = self.recon_zone_offset_enu
            lat, lon, alt = self.recon_zone_lat, self.recon_zone_lon, self.recon_zone_rel_alt

        if self.active_target_kind != zone:
            self.active_target_kind = zone
            self.active_global_target = self.flight.make_global_target(lat, lon, alt) if use_global else None
            self.active_local_target = None if use_global else self.flight.start_offset_to_enu(local_offset)
            self.arrival_since = None
            rospy.loginfo("New %s target: global=%s local=%s", zone, bool(use_global), self.active_local_target)

        if use_global:
            self.flight.publish_global_target(self.active_global_target)
            distance = self.global_target_error(self.active_global_target)
            arrived = distance is not None and distance <= self.arrival_tolerance_m
        else:
            self.flight.publish_local_target(self.active_local_target)
            arrived = self.flight.local_error(self.active_local_target) <= self.arrival_tolerance_m

        if arrived:
            if self.arrival_since is None:
                self.arrival_since = now_s()
            return now_s() - self.arrival_since >= self.arrival_hold_s
        self.arrival_since = None
        return False

    def fly_search_waypoint(self):
        if self.search_hold_until > now_s():
            self.flight.publish_local_target(self.active_local_target)
            return

        if self.active_target_kind != "search_%d" % self.search_index:
            self.active_target_kind = "search_%d" % self.search_index
            self.active_local_target = self.flight.start_offset_to_enu(self.search_offsets_enu[self.search_index])
            self.arrival_since = None
            rospy.loginfo(
                "Search waypoint lap=%d index=%d target=%s",
                self.search_laps + 1,
                self.search_index,
                self.active_local_target,
            )

        self.flight.publish_local_target(self.active_local_target)
        if self.flight.local_error(self.active_local_target) <= self.arrival_tolerance_m:
            if self.arrival_since is None:
                self.arrival_since = now_s()
            if now_s() - self.arrival_since >= self.arrival_hold_s:
                self.search_hold_until = now_s() + self.search_waypoint_hold_s
                self.search_index += 1
                if self.search_index >= len(self.search_offsets_enu):
                    self.search_index = 0
                    self.search_laps += 1
                self.active_target_kind = None
        else:
            self.arrival_since = None

    def hold_active_target(self, duration_s):
        if self.active_global_target is not None:
            self.flight.publish_global_target(self.active_global_target)
        elif self.active_local_target is not None:
            self.flight.publish_local_target(self.active_local_target)
        return self.state_age() >= duration_s

    def global_target_error(self, target):
        if not self.flight.fix_received or self.flight.relative_altitude is None:
            return None
        horizontal = self.flight.valid_fix(self.flight.current_fix) and target is not None
        if not horizontal:
            return None
        from competition_mission_common import horizontal_wgs84_distance

        h = horizontal_wgs84_distance(
            self.flight.current_fix.latitude,
            self.flight.current_fix.longitude,
            target.latitude,
            target.longitude,
        )
        v = abs(self.flight.relative_altitude - target.altitude)
        return max(h, v)

    def publish_status(self):
        aiming = self.state == self.AIMING
        self.flight.publish_detector_status(
            self.dropper.ammo_a,
            self.dropper.ammo_b,
            aiming,
            self.dropper.detector_last_drop(),
        )
        phase = {
            "search": self.state == self.DROP_ZONE_SEARCH,
            "aiming": self.state == self.AIMING,
            "drop": self.state == self.DROP,
            "recon": self.state in (self.GOTO_RECON_ZONE, self.RECON_OBSERVE),
        }
        self.flight.publish_main_status(
            {
                "main_state": self.state,
                "flight_mode": self.flight.current_state.mode,
                "armed": bool(self.flight.current_state.armed),
                "ammo_a": int(self.dropper.ammo_a),
                "ammo_b": int(self.dropper.ammo_b),
                "current_dropper": self.dropper.current_label(),
                "locked_target_id": self.locked_track_id,
                "search": phase["search"],
                "aiming": phase["aiming"],
                "drop": phase["drop"],
                "recon": phase["recon"],
                "stable_targets": len(self.tracker.valid_tracks()),
                "abort_reason": self.abort_reason,
            }
        )


def main():
    rospy.init_node("competition_main")
    node = CompetitionMain()
    raise SystemExit(node.run())


if __name__ == "__main__":
    main()
