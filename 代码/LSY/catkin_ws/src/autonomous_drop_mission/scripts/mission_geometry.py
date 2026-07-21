#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Pure geometry helpers for the autonomous two-drop mission.

Mission-local axis convention (confirmed by the aircraft test):
    +X = aircraft right
    +Y = aircraft forward
    +Z = up

Camera optical convention:
    +u = image right
    +v = image down
    +depth = along the downward-looking optical axis

This module intentionally has no ROS dependency so its sign conventions can be
unit-tested on the NUC without a flight controller.
"""

import math


def finite(*values):
    """Return True only when every value can be represented by a finite float."""
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def limit_vector_2d(x_value, y_value, max_norm):
    """Limit a 2-D vector without changing its direction."""
    x_value = float(x_value)
    y_value = float(y_value)
    max_norm = max(0.0, float(max_norm))
    norm = math.hypot(x_value, y_value)
    if norm <= max_norm or norm <= 1e-12:
        return x_value, y_value
    scale = max_norm / norm
    return x_value * scale, y_value * scale


def ramped_progress(elapsed_s, speed_mps, distance_m):
    """Distance travelled by a constant-speed reference, capped at its endpoint."""
    if not finite(elapsed_s, speed_mps, distance_m):
        raise ValueError("ramped progress received a non-finite value")
    elapsed_s = max(0.0, float(elapsed_s))
    speed_mps = float(speed_mps)
    distance_m = float(distance_m)
    if speed_mps <= 0.0 or distance_m <= 0.0:
        raise ValueError("speed and distance must be positive")
    return min(distance_m, elapsed_s * speed_mps)


def pixel_to_right_forward(u, v, depth_m, fx, fy, cx, cy):
    """Convert a target pixel into aircraft right/forward displacement.

    The D435i looks downward and its physical top points to aircraft forward:
      image right (+u) -> aircraft right (+X)
      image up (-v)    -> aircraft forward (+Y)

    The computation deliberately uses pixel coordinates instead of the existing
    detector's camera_x_m because that node currently has a configurable X sign
    inversion whose default conflicts with parts of its documentation.
    """
    if not finite(u, v, depth_m, fx, fy, cx, cy):
        raise ValueError("pixel projection received a non-finite value")
    depth_m = float(depth_m)
    fx = float(fx)
    fy = float(fy)
    if depth_m <= 0.0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError("depth and focal lengths must be positive")
    right_m = (float(u) - float(cx)) * depth_m / fx
    forward_m = -(float(v) - float(cy)) * depth_m / fy
    return right_m, forward_m


def target_relative_to_aircraft(
    target_right_from_camera_m,
    target_forward_from_camera_m,
    target_down_from_camera_m,
    camera_x_right_m,
    camera_y_forward_m,
    camera_down_m,
):
    """Translate a camera-relative target vector to the aircraft reference point."""
    right_m = float(camera_x_right_m) + float(target_right_from_camera_m)
    forward_m = float(camera_y_forward_m) + float(target_forward_from_camera_m)
    down_m = float(camera_down_m) + float(target_down_from_camera_m)
    return right_m, forward_m, down_m


def alignment_error(
    target_right_from_aircraft_m,
    target_forward_from_aircraft_m,
    dropper_x_right_m,
    dropper_y_forward_m,
):
    """Aircraft translation needed to put one physical dropper above a target."""
    return (
        float(target_right_from_aircraft_m) - float(dropper_x_right_m),
        float(target_forward_from_aircraft_m) - float(dropper_y_forward_m),
    )


def distance_3d(right_m, forward_m, down_m):
    return math.sqrt(float(right_m) ** 2 + float(forward_m) ** 2 + float(down_m) ** 2)


def quaternion_yaw(x, y, z, w):
    """Return conventional yaw in radians from a normalized or near-normalized quaternion."""
    x = float(x)
    y = float(y)
    z = float(z)
    w = float(w)
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def wrapped_angle(angle_rad):
    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def angular_difference(a_rad, b_rad):
    return wrapped_angle(float(a_rad) - float(b_rad))
