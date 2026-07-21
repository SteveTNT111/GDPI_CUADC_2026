#!/usr/bin/env python3

import ast
import pathlib
import unittest
import xml.etree.ElementTree as ET


class FakeRospy:
    @staticmethod
    def logwarn(*_args, **_kwargs):
        pass

    @staticmethod
    def logerr(*_args, **_kwargs):
        pass


def load_run_harness():
    script_path = (
        pathlib.Path(__file__).resolve().parents[1]
        / "scripts"
        / "semi_auto_drop_test.py"
    )
    tree = ast.parse(script_path.read_text(encoding="utf-8"), str(script_path))
    source_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "SemiAutoDropTest"
    )
    run_node = next(
        node
        for node in source_class.body
        if isinstance(node, ast.FunctionDef) and node.name == "run"
    )
    harness_class = ast.ClassDef(
        name="RunHarness",
        bases=[],
        keywords=[],
        body=[run_node],
        decorator_list=[],
    )
    module = ast.Module(body=[harness_class], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {"rospy": FakeRospy}
    exec(compile(module, str(script_path), "exec"), namespace)
    return namespace["RunHarness"]


RunHarness = load_run_harness()


def make_harness(enable_b_dropper):
    node = RunHarness()
    node.ground_test = False
    node.enable_b_dropper = enable_b_dropper
    node.dry_run = True
    node.align_threshold_m = 0.15
    node.target_drop_height_m = 1.5
    node.height_tolerance_m = 0.15
    node.stable_time_s = 2.0
    node.min_bucket_count = 1
    node.lost_frame_threshold = 10
    node.vision_frame_rate_hz = 30.0
    node.target_reacquire_timeout_s = 4.0
    node.max_horizontal_speed_mps = 0.05
    node.lateral_divergence_growth_m = 0.10
    node.lateral_divergence_window_s = 3.0
    node.command_error_buffer_m = 0.03
    node.after_a_delay_s = 3.0
    node.calls = []

    node.start_dependencies = lambda: True
    node.wait_for_mavros = lambda: True
    node._wait_for_servo_subscriber = lambda: True
    node.wait_for_target_and_confirm = lambda: True

    def validate(dropper, expected_mode="LOITER", allow_target_reacquire=False):
        node.calls.append(
            ("validate", dropper, expected_mode, allow_target_reacquire)
        )
        return True

    def set_mode(mode):
        node.calls.append(("mode", mode))
        return True

    def aim(dropper, reset_loss_guard=True):
        node.calls.append(("aim", dropper, reset_loss_guard))
        return True

    def drop(dropper):
        node.calls.append(("drop", dropper))
        return True

    def wait_between():
        node.calls.append(("wait_between",))
        return True

    def safe_loiter(reason):
        node.calls.append(("safe_loiter", reason))
        return True

    node._validate_before_guided = validate
    node.set_mode_confirmed = set_mode
    node.aim_dropper = aim
    node.drop = drop
    node._wait_between_droppers = wait_between
    node.safe_loiter = safe_loiter
    return node


class SemiAutoAbFlowTest(unittest.TestCase):
    def test_ab_launch_has_safe_defaults_and_requested_loss_guard(self):
        launch_path = (
            pathlib.Path(__file__).resolve().parents[1]
            / "launch"
            / "semi_auto_ab_drop_test.launch"
        )
        root = ET.parse(str(launch_path)).getroot()
        defaults = {
            element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("arg")
        }
        include = root.find("include")
        include_args = {
            element.attrib["name"]: element.attrib.get("value")
            for element in include.findall("arg")
        }
        self.assertEqual("true", defaults["dry_run"])
        self.assertEqual("false", defaults["auto_start_servo_test"])
        self.assertEqual("10", defaults["lost_frame_threshold"])
        self.assertEqual("4.0", defaults["target_reacquire_timeout_s"])
        self.assertEqual("0.10", defaults["lateral_divergence_growth_m"])
        self.assertEqual("3.0", defaults["lateral_divergence_window_s"])
        self.assertEqual("0.03", defaults["command_error_buffer_m"])
        self.assertEqual("true", include_args["enable_b_dropper"])

    def test_ab_uses_one_guided_session_and_loiters_only_after_b(self):
        node = make_harness(True)
        self.assertEqual(0, node.run())
        self.assertEqual([("mode", "GUIDED")], [c for c in node.calls if c[0] == "mode"])
        self.assertEqual(
            [
                ("validate", "A", "LOITER", False),
                ("validate", "B", "GUIDED", True),
            ],
            [c for c in node.calls if c[0] == "validate"],
        )
        self.assertEqual(
            [("aim", "A", True), ("aim", "B", False)],
            [c for c in node.calls if c[0] == "aim"],
        )
        self.assertEqual(
            [("drop", "A"), ("drop", "B")],
            [c for c in node.calls if c[0] == "drop"],
        )
        self.assertEqual(1, sum(c[0] == "wait_between" for c in node.calls))
        loiter_calls = [c for c in node.calls if c[0] == "safe_loiter"]
        self.assertEqual(1, len(loiter_calls))
        self.assertIn("A/B", loiter_calls[0][1])

    def test_a_only_still_returns_loiter_after_a(self):
        node = make_harness(False)
        self.assertEqual(0, node.run())
        self.assertEqual([("drop", "A")], [c for c in node.calls if c[0] == "drop"])
        self.assertFalse(any(c[0] == "wait_between" for c in node.calls))
        loiter_calls = [c for c in node.calls if c[0] == "safe_loiter"]
        self.assertEqual([("safe_loiter", "A 投放完成")], loiter_calls)


if __name__ == "__main__":
    unittest.main()
