#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from competition_mission_common import (
    DropperManager,
    FlightIO,
    TargetTracker,
    get_bool_param,
    now_s,
)


class SingleBucketAimTest:
    WAIT_TARGET = "WAIT_TARGET"
    LOCK_TARGET = "LOCK_TARGET"
    AIMING = "AIMING"
    DROP_OR_REPORT = "DROP_OR_REPORT"
    FINISH = "FINISH"
    ABORT = "ABORT"

    def __init__(self):
        self.flight = FlightIO()
        self.tracker = TargetTracker(self.flight)
        self.dropper = DropperManager(self.flight)

        self.state = self.WAIT_TARGET
        self.state_entered_at = now_s()

        self.auto_guided = get_bool_param("~auto_guided", False)
        self.dry_run = get_bool_param("~dry_run", True)
        self.switch_loiter_on_finish = get_bool_param("~switch_loiter_on_finish", True)
        self.ground_test = get_bool_param("~ground_test", False)
        self.test_dropper = str(rospy.get_param("~test_dropper", "A")).strip().upper()

        self.aiming_altitude_m = float(rospy.get_param("~aiming_altitude_m", 2.0))
        self.aiming_threshold_m = float(rospy.get_param("~aiming_threshold_m", 0.15))
        self.aiming_stable_time_s = float(rospy.get_param("~aiming_stable_time_s", 1.5))
        self.target_lost_timeout_s = float(rospy.get_param("~target_lost_timeout_s", 2.0))
        self.max_test_duration_s = float(rospy.get_param("~max_test_duration_s", 180.0))

        self.locked_track_id = None
        self.locked_target_enu = None
        self.aligned_since = None
        self.started_at = now_s()

        self._force_test_dropper()

    def _force_test_dropper(self):
        if self.test_dropper == "B":
            self.dropper.ammo_a = 0
            self.dropper.ammo_b = max(1, self.dropper.ammo_b)
        else:
            self.test_dropper = "A"
            self.dropper.ammo_a = max(1, self.dropper.ammo_a)
            self.dropper.ammo_b = 0
        if self.dry_run:
            self.dropper.enable_servo_drop = False

    def transition(self, new_state, reason=""):
        old = self.state
        self.state = new_state
        self.state_entered_at = now_s()
        rospy.loginfo("SINGLE TEST STATE %s -> %s %s", old, new_state, ("| " + reason) if reason else "")

    def run(self):
        if not self.flight.wait_for_connection() or not self.flight.ensure_services():
            return 1
        if not self.flight.wait_for_navigation() or not self.flight.capture_start_reference():
            return 1

        rospy.logwarn(
            "Single-bucket test: no takeoff, no RTL. auto_guided=%s dry_run=%s dropper=%s real_servo=%s",
            self.auto_guided,
            self.dry_run,
            self.test_dropper,
            self.dropper.enable_servo_drop,
        )

        rate = rospy.Rate(float(rospy.get_param("~test_rate", 10.0)))
        while not rospy.is_shutdown():
            self.step()
            self.publish_status()
            if self.state == self.FINISH:
                self.finish_mode()
                return 0
            if self.state == self.ABORT:
                self.finish_mode()
                return 1
            if now_s() - self.started_at >= self.max_test_duration_s:
                self.transition(self.ABORT, "timeout")
            rate.sleep()
        return 1

    def step(self):
        if self.state == self.WAIT_TARGET:
            track = self.tracker.best_track()
            if track is None:
                rospy.loginfo_throttle(2.0, "Waiting for stable YOLO bucket target...")
                return
            self.locked_track_id = track.track_id
            self.locked_target_enu = track.stable_enu()
            self.transition(self.LOCK_TARGET)

        elif self.state == self.LOCK_TARGET:
            if self.auto_guided and not self.ground_test and self.flight.current_state.mode != "GUIDED":
                if not self.flight.set_mode("GUIDED"):
                    self.transition(self.ABORT, "guided_failed")
                    return
            self.transition(self.AIMING)

        elif self.state == self.AIMING:
            if self.locked_target_enu is None:
                self.transition(self.WAIT_TARGET, "lost_lock")
                return
            if now_s() - self.tracker.last_detections_stamp > self.target_lost_timeout_s:
                self.transition(self.WAIT_TARGET, "target_lost")
                self.locked_target_enu = None
                self.locked_track_id = None
                return

            if self.ground_test:
                rospy.loginfo_throttle(1.0, "ground_test=true: lock is valid; no setpoint sent")
                return
            if self.flight.current_state.mode != "GUIDED":
                rospy.logwarn_throttle(2.0, "Waiting for GUIDED mode before sending aim setpoints")
                return

            target = self.dropper.compensated_target_enu(self.locked_target_enu)
            start_z = self.flight.start_pose.pose.position.z if self.flight.start_pose else 0.0
            target = (target[0], target[1], start_z + self.aiming_altitude_m)
            self.flight.publish_local_target(target)
            h_error = self.flight.horizontal_error(target)

            if h_error <= self.aiming_threshold_m and self.dropper.cooldown_ready():
                if self.aligned_since is None:
                    self.aligned_since = now_s()
                stable_s = now_s() - self.aligned_since
                if stable_s >= self.aiming_stable_time_s:
                    rospy.loginfo(
                        "DROP CONDITION SATISFIED: dropper=%s h_error=%.3f stable=%.2fs dry_run=%s",
                        self.dropper.current_label(),
                        h_error,
                        stable_s,
                        self.dry_run,
                    )
                    self.transition(self.DROP_OR_REPORT)
            else:
                self.aligned_since = None

            rospy.loginfo_throttle(
                1.0,
                "Single aim: track=%s dropper=%s h_error=%.3f stable=%.2f/%.2f",
                self.locked_track_id,
                self.dropper.current_label(),
                h_error,
                0.0 if self.aligned_since is None else now_s() - self.aligned_since,
                self.aiming_stable_time_s,
            )

        elif self.state == self.DROP_OR_REPORT:
            if self.dry_run:
                rospy.logwarn("dry_run=true: not opening servo; test complete")
                self.transition(self.FINISH)
                return
            if self.dropper.execute_drop():
                self.tracker.mark_dropped(self.locked_target_enu)
                self.transition(self.FINISH)
            else:
                self.transition(self.ABORT, "drop_failed")

    def finish_mode(self):
        if self.ground_test:
            return
        if self.switch_loiter_on_finish and self.flight.current_state.mode == "GUIDED":
            self.flight.set_mode("LOITER")

    def publish_status(self):
        self.flight.publish_detector_status(
            self.dropper.ammo_a,
            self.dropper.ammo_b,
            self.state == self.AIMING,
            self.dropper.detector_last_drop(),
        )
        self.flight.publish_main_status(
            {
                "main_state": "SINGLE_BUCKET_" + self.state,
                "flight_mode": self.flight.current_state.mode,
                "armed": bool(self.flight.current_state.armed),
                "ammo_a": int(self.dropper.ammo_a),
                "ammo_b": int(self.dropper.ammo_b),
                "current_dropper": self.dropper.current_label(),
                "locked_target_id": self.locked_track_id,
                "search": self.state == self.WAIT_TARGET,
                "aiming": self.state == self.AIMING,
                "drop": self.state == self.DROP_OR_REPORT,
                "recon": False,
                "dry_run": self.dry_run,
            }
        )


def main():
    rospy.init_node("single_bucket_aim_test")
    node = SingleBucketAimTest()
    raise SystemExit(node.run())


if __name__ == "__main__":
    main()
