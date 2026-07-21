#!/usr/bin/env python3

import math
import os
import sys
import unittest
from types import SimpleNamespace


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
VISION_SCRIPT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "cuadc_vision", "scripts")
)
if VISION_SCRIPT_DIR not in sys.path:
    sys.path.insert(0, VISION_SCRIPT_DIR)

from detector_node import frd_to_local_pose_axes
from semi_auto_drop_test import (
    LateralDivergenceGuard,
    SemiAutoDropTest,
    TargetLossGuard,
    limit_xy_step,
)


class CoordinateAxisTests(unittest.TestCase):
    def test_flight_calibration_preserves_positive_right(self):
        self.assertEqual(
            frd_to_local_pose_axes((1.0, 2.0, 3.0)),
            (1.0, 2.0, -3.0),
        )

    def test_vertical_axis_still_negates_down(self):
        self.assertEqual(
            frd_to_local_pose_axes((0.0, -2.0, 4.0), right_sign=-1.0),
            (0.0, 2.0, -4.0),
        )


class LimitedSetpointTests(unittest.TestCase):
    def test_five_centimetres_per_second_at_ten_hz(self):
        x, y = limit_xy_step(0.0, 0.0, 1.0, 0.0, 0.05 * 0.1)
        self.assertAlmostEqual(x, 0.005, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)

    def test_diagonal_step_uses_vector_magnitude_limit(self):
        x, y = limit_xy_step(0.0, 0.0, 1.0, 1.0, 0.005)
        self.assertAlmostEqual(math.hypot(x, y), 0.005, places=9)


class HeightCheckTests(unittest.TestCase):
    def test_height_over_surface_uses_ned_down_as_surface_z(self):
        node = SemiAutoDropTest.__new__(SemiAutoDropTest)
        node.target_drop_height_m = 1.5
        aim = SimpleNamespace(
            a_ned_valid=True,
            a_ned_n=0.0,
            a_ned_e=0.0,
            a_ned_d=2.0,
        )
        pose = SimpleNamespace(
            pose=SimpleNamespace(position=SimpleNamespace(z=0.0))
        )
        actual_height, height_error = node._height_over_surface(aim, "A", pose)
        self.assertAlmostEqual(actual_height, 2.0)
        self.assertAlmostEqual(height_error, -0.5)


class TargetLossGuardTests(unittest.TestCase):
    def test_ten_frames_then_four_second_timeout(self):
        guard = TargetLossGuard(10, 4.0, 30.0)
        for frame in range(9):
            self.assertEqual(
                guard.observe(False, frame / 30.0), TargetLossGuard.TOLERATING
            )
        self.assertEqual(guard.observe(False, 9.0 / 30.0), TargetLossGuard.LOST)
        self.assertEqual(guard.status(4.29), TargetLossGuard.LOST)
        self.assertEqual(guard.status(4.31), TargetLossGuard.TIMEOUT)

    def test_reacquire_inside_four_seconds_continues(self):
        guard = TargetLossGuard(10, 4.0, 30.0)
        for frame in range(10):
            guard.observe(False, frame / 30.0)
        self.assertEqual(guard.observe(True, 2.0), TargetLossGuard.REACQUIRED)
        self.assertEqual(guard.status(2.1), TargetLossGuard.VALID)


class LateralDivergenceGuardTests(unittest.TestCase):
    def test_sustained_growth_trips_guard(self):
        guard = LateralDivergenceGuard(0.10, 3.0, buffer_m=0.03)
        tripped = False
        for index in range(31):
            now = index * 0.1
            tripped = guard.observe(0.20 + 0.07 * now, now)
        self.assertTrue(tripped)

    def test_five_centimetres_per_second_has_three_centimetre_buffer(self):
        guard = LateralDivergenceGuard(0.10, 3.0, buffer_m=0.03)
        for index in range(31):
            now = index * 0.1
            self.assertFalse(guard.observe(0.20 + 0.05 * now, now))

    def test_total_xy_error_does_not_trip_when_one_axis_improves(self):
        guard = LateralDivergenceGuard(0.10, 3.0, buffer_m=0.03)
        for index in range(31):
            now = index * 0.1
            dx = -0.08 + 0.11 * now
            dy = -0.25 + (0.25 / 3.0) * now
            self.assertFalse(guard.observe(math.hypot(dx, dy), now))

    def test_noise_and_decreasing_error_do_not_trip(self):
        guard = LateralDivergenceGuard(0.10, 3.0, buffer_m=0.03)
        for index in range(61):
            now = index * 0.1
            noise = 0.015 if index % 2 else -0.015
            error = max(0.01, 0.50 - 0.03 * now + noise)
            self.assertFalse(guard.observe(error, now))


if __name__ == "__main__":
    unittest.main()
