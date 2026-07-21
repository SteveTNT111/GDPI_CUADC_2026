#!/usr/bin/env python3

import ast
import pathlib
import unittest


def load_target_loss_guard():
    script_path = (
        pathlib.Path(__file__).resolve().parents[1]
        / "scripts"
        / "semi_auto_drop_test.py"
    )
    tree = ast.parse(script_path.read_text(encoding="utf-8"), str(script_path))
    class_node = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "TargetLossGuard"
    )
    module = ast.Module(body=[class_node], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {}
    exec(compile(module, str(script_path), "exec"), namespace)
    return namespace["TargetLossGuard"]


TargetLossGuard = load_target_loss_guard()


class TargetLossGuardTest(unittest.TestCase):
    def test_nine_invalid_frames_are_tolerated(self):
        guard = TargetLossGuard(10, 4.0)
        for frame_index in range(9):
            state = guard.observe(False, frame_index / 30.0)
            self.assertEqual(TargetLossGuard.TOLERATING, state)
        self.assertEqual(9, guard.invalid_frames)
        self.assertIsNone(guard.lost_since)

    def test_tenth_invalid_frame_starts_reacquire_timer(self):
        guard = TargetLossGuard(10, 4.0)
        for frame_index in range(10):
            state = guard.observe(False, frame_index / 30.0)
        self.assertEqual(TargetLossGuard.LOST, state)
        self.assertEqual(10, guard.invalid_frames)
        self.assertAlmostEqual(9.0 / 30.0, guard.lost_since)

    def test_reacquire_within_timeout_continues(self):
        guard = TargetLossGuard(10, 4.0)
        for frame_index in range(10):
            guard.observe(False, frame_index / 30.0)
        state = guard.observe(True, guard.lost_since + 3.99)
        self.assertEqual(TargetLossGuard.REACQUIRED, state)
        self.assertEqual(0, guard.invalid_frames)
        self.assertIsNone(guard.lost_since)

    def test_no_reacquire_for_four_seconds_times_out(self):
        guard = TargetLossGuard(10, 4.0)
        for frame_index in range(10):
            guard.observe(False, frame_index / 30.0)
        state = guard.status(guard.lost_since + 4.0)
        self.assertEqual(TargetLossGuard.TIMEOUT, state)
        self.assertTrue(guard.timed_out)

    def test_recovery_after_deadline_cannot_restart_timed_out_flow(self):
        guard = TargetLossGuard(10, 4.0)
        for frame_index in range(10):
            guard.observe(False, frame_index / 30.0)
        state = guard.observe(True, guard.lost_since + 4.01)
        self.assertEqual(TargetLossGuard.TIMEOUT, state)
        self.assertTrue(guard.timed_out)

    def test_stream_stall_enters_same_reacquire_timeout(self):
        guard = TargetLossGuard(10, 4.0)
        self.assertEqual(TargetLossGuard.LOST, guard.force_lost(5.0))
        self.assertEqual(10, guard.invalid_frames)
        self.assertEqual(TargetLossGuard.TIMEOUT, guard.status(9.0))

    def test_time_equivalent_handles_detector_slower_than_camera(self):
        guard = TargetLossGuard(10, 4.0, 30.0)
        self.assertEqual(TargetLossGuard.TOLERATING, guard.observe(False, 2.0))
        self.assertEqual(TargetLossGuard.LOST, guard.status(2.0 + 10.0 / 30.0))


if __name__ == "__main__":
    unittest.main()
