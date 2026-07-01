#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy

from d435i_yellow_circle_detector.msg import MissionTarget, YoloDetections


def parse_keywords(text):
    return [item.strip().lower() for item in text.split(",") if item.strip()]


def parse_float_list(value):
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    return [float(item.strip()) for item in str(value).split(",") if item.strip()]


class CompetitionTaskNode:
    def __init__(self):
        self.mission_stage = rospy.get_param("~mission_stage", "drop").strip().lower()
        self.yolo_detections_topic = rospy.get_param("~yolo_detections_topic", "/yolo/detections")
        self.target_topic = rospy.get_param("~target_topic", "/competition/target")

        self.drop_zone_distance_m = float(rospy.get_param("~drop_zone_distance_m", 30.0))
        self.disaster_zone_distance_m = float(rospy.get_param("~disaster_zone_distance_m", 55.0))
        self.zone_length_m = float(rospy.get_param("~zone_length_m", 8.0))
        self.zone_width_m = float(rospy.get_param("~zone_width_m", 5.0))
        self.drop_cylinder_diameters_m = parse_float_list(
            rospy.get_param("~drop_cylinder_diameters_m", "0.15,0.20,0.25")
        )
        self.disaster_cylinder_diameter_m = float(rospy.get_param("~disaster_cylinder_diameter_m", 0.20))
        self.hazard_label_size_m = float(rospy.get_param("~hazard_label_size_m", 0.12))
        self.b_zone_diameter_m = float(rospy.get_param("~b_zone_diameter_m", 1.0))

        self.cylinder_keywords = parse_keywords(
            rospy.get_param("~cylinder_keywords", "cylinder,tube,barrel,can,tong,yuantong,white_cylinder")
        )
        self.hazard_keywords = parse_keywords(
            rospy.get_param("~hazard_keywords", "hazard,chemical,danger,label,sign,biaoshi")
        )

        self.target_pub = rospy.Publisher(self.target_topic, MissionTarget, queue_size=1)
        self.sub = rospy.Subscriber(
            self.yolo_detections_topic,
            YoloDetections,
            self.detections_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "Competition task node started. stage=%s yolo=%s target=%s drop_zone=%.1fm disaster_zone=%.1fm zone=%.1fx%.1fm",
            self.mission_stage,
            self.yolo_detections_topic,
            self.target_topic,
            self.drop_zone_distance_m,
            self.disaster_zone_distance_m,
            self.zone_length_m,
            self.zone_width_m,
        )

    def detections_callback(self, msg):
        candidates = []
        for detection in msg.detections:
            if not detection.detected:
                continue
            target = self.classify_detection(msg.header, detection)
            if target.valid:
                candidates.append(target)

        if not candidates:
            self.target_pub.publish(self.empty_target(msg.header))
            return

        best = max(candidates, key=self.target_score)
        self.target_pub.publish(best)

    def empty_target(self, header):
        target = MissionTarget()
        target.header = header
        target.valid = False
        target.mission_stage = self.mission_stage
        target.target_type = "none"
        target.class_name = ""
        target.confidence = 0.0
        target.center_x = -1
        target.center_y = -1
        target.camera_x_m = 0.0
        target.camera_y_m = 0.0
        target.camera_z_m = 0.0
        target.distance_m = 0.0
        target.bbox_width_m = 0.0
        target.bbox_height_m = 0.0
        target.nominal_diameter_m = 0.0
        target.diameter_class = "unknown"
        target.zone_hint = "unknown"
        target.a_zone_radius_m = 0.0
        target.b_zone_radius_m = 0.0
        target.action_hint = "no_target"
        return target

    def classify_detection(self, header, detection):
        target = self.empty_target(header)
        name = detection.class_name.lower()
        is_hazard = self.contains_any(name, self.hazard_keywords)
        is_cylinder = self.contains_any(name, self.cylinder_keywords)

        if self.mission_stage == "disaster":
            if is_hazard:
                self.fill_common(target, detection, "hazard_label")
                target.nominal_diameter_m = self.hazard_label_size_m
                target.diameter_class = "hazard_label_12cm"
                target.zone_hint = "disaster_cylinder_inner"
                target.a_zone_radius_m = self.disaster_cylinder_diameter_m * 0.5
                target.b_zone_radius_m = 0.0
                target.action_hint = "inspect_or_report_hazard_label"
                return target
            if is_cylinder:
                self.fill_common(target, detection, "disaster_cylinder")
                target.nominal_diameter_m = self.disaster_cylinder_diameter_m
                target.diameter_class = "20cm"
                target.zone_hint = "disaster_cylinder"
                target.a_zone_radius_m = self.disaster_cylinder_diameter_m * 0.5
                target.b_zone_radius_m = 0.0
                target.action_hint = "search_for_hazard_label"
                return target

        if self.mission_stage == "drop":
            if is_cylinder:
                diameter = self.nearest_drop_diameter(detection.bbox_width_m)
                self.fill_common(target, detection, "drop_cylinder")
                target.nominal_diameter_m = diameter
                target.diameter_class = "{}cm".format(int(round(diameter * 100.0)))
                target.zone_hint = "A_center_or_B_ring"
                target.a_zone_radius_m = diameter * 0.5
                target.b_zone_radius_m = self.b_zone_diameter_m * 0.5
                target.action_hint = "drop_to_A_center_prefer_B_ring_if_needed"
                return target

        if self.mission_stage == "auto":
            if is_hazard:
                self.fill_common(target, detection, "hazard_label")
                target.nominal_diameter_m = self.hazard_label_size_m
                target.diameter_class = "hazard_label_12cm"
                target.zone_hint = "disaster"
                target.a_zone_radius_m = self.disaster_cylinder_diameter_m * 0.5
                target.action_hint = "inspect_or_report_hazard_label"
                return target
            if is_cylinder:
                diameter = self.nearest_drop_diameter(detection.bbox_width_m)
                self.fill_common(target, detection, "cylinder")
                target.nominal_diameter_m = diameter
                target.diameter_class = "{}cm_or_20cm".format(int(round(diameter * 100.0)))
                target.zone_hint = "drop_or_disaster"
                target.a_zone_radius_m = diameter * 0.5
                target.b_zone_radius_m = self.b_zone_diameter_m * 0.5
                target.action_hint = "use_mission_stage_to_disambiguate"
                return target

        return target

    def fill_common(self, target, detection, target_type):
        target.valid = True
        target.mission_stage = self.mission_stage
        target.target_type = target_type
        target.class_name = detection.class_name
        target.confidence = detection.confidence
        target.center_x = detection.center_x
        target.center_y = detection.center_y
        target.camera_x_m = detection.camera_x_m
        target.camera_y_m = detection.camera_y_m
        target.camera_z_m = detection.camera_z_m
        target.distance_m = detection.distance_m
        target.bbox_width_m = detection.bbox_width_m
        target.bbox_height_m = detection.bbox_height_m

    def contains_any(self, text, keywords):
        return any(keyword in text for keyword in keywords)

    def nearest_drop_diameter(self, measured_width_m):
        if measured_width_m <= 0.0 or not math.isfinite(measured_width_m):
            return 0.20
        return min(self.drop_cylinder_diameters_m, key=lambda value: abs(value - measured_width_m))

    def target_score(self, target):
        score = target.confidence
        if target.distance_m > 0.0:
            score += max(0.0, 2.0 - target.distance_m) * 0.05
        if self.mission_stage == "disaster" and target.target_type == "hazard_label":
            score += 1.0
        if self.mission_stage == "drop" and target.target_type == "drop_cylinder":
            score += 0.5
        return score


def main():
    rospy.init_node("competition_task_node")
    CompetitionTaskNode()
    rospy.spin()


if __name__ == "__main__":
    main()
