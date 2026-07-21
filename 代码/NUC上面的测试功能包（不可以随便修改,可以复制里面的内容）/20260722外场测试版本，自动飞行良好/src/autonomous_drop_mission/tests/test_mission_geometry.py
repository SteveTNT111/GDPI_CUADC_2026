#!/usr/bin/env python3

import math
import os
import sys
import unittest


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPT_DIR)

from mission_geometry import (  # noqa: E402
    alignment_error,
    angular_difference,
    distance_3d,
    limit_vector_2d,
    pixel_to_right_forward,
    ramped_progress,
    target_relative_to_aircraft,
)


class MissionGeometryTest(unittest.TestCase):
    def test_optical_center_is_zero_horizontal_offset(self):
        right, forward = pixel_to_right_forward(320, 240, 2.0, 600, 600, 320, 240)
        self.assertAlmostEqual(right, 0.0)
        self.assertAlmostEqual(forward, 0.0)

    def test_image_right_is_aircraft_positive_x_right(self):
        right, forward = pixel_to_right_forward(380, 240, 2.0, 600, 600, 320, 240)
        self.assertAlmostEqual(right, 0.2)
        self.assertAlmostEqual(forward, 0.0)

    def test_image_up_is_aircraft_positive_y_forward(self):
        right, forward = pixel_to_right_forward(320, 180, 2.0, 600, 600, 320, 240)
        self.assertAlmostEqual(right, 0.0)
        self.assertAlmostEqual(forward, 0.2)

    def test_camera_offset_is_added_to_aircraft_relative_target(self):
        right, forward, down = target_relative_to_aircraft(
            0.2, 0.3, 2.0, 0.05, -0.10, 0.40
        )
        self.assertAlmostEqual(right, 0.25)
        self.assertAlmostEqual(forward, 0.20)
        self.assertAlmostEqual(down, 2.40)

    def test_dropper_is_aligned_when_target_matches_physical_offset(self):
        error_x, error_y = alignment_error(0.02, 0.05, 0.02, 0.05)
        self.assertAlmostEqual(error_x, 0.0)
        self.assertAlmostEqual(error_y, 0.0)

    def test_vector_limiter_preserves_direction(self):
        x_value, y_value = limit_vector_2d(3.0, 4.0, 1.0)
        self.assertAlmostEqual(x_value, 0.6)
        self.assertAlmostEqual(y_value, 0.8)

    def test_distance(self):
        self.assertAlmostEqual(distance_3d(3.0, 4.0, 12.0), 13.0)

    def test_forward_search_reference_uses_requested_speed_and_caps_at_3m(self):
        self.assertAlmostEqual(ramped_progress(5.0, 0.20, 3.0), 1.0)
        self.assertAlmostEqual(ramped_progress(20.0, 0.20, 3.0), 3.0)

    def test_wrapped_yaw_difference(self):
        delta = angular_difference(math.radians(-179), math.radians(179))
        self.assertAlmostEqual(math.degrees(delta), 2.0)


if __name__ == "__main__":
    unittest.main()
