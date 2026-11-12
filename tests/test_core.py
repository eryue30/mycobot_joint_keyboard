# -*- coding: utf-8 -*-
"""离线测试：不导入 ROS，不打开相机、串口或机械臂。"""
import json
import math
from pathlib import Path
import tempfile
import unittest

from mycobot_joint_keyboard.core import (
    FeedbackWindow, JOINT_NAMES, JointLimits, KEY_MAP, KEY_PAIRS,
    jog_target, load_home, load_limits, save_home, six_finite,
    validate_signs, waypoints,
)


LIMITS = JointLimits((-170.0,) * 6, (170.0,) * 6)


class MappingTests(unittest.TestCase):
    def test_all_twelve_keys_change_only_the_requested_joint(self):
        current = (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
        for joint, pair in enumerate(KEY_PAIRS):
            for key, delta in zip(pair, (-1.0, 1.0)):
                with self.subTest(key=key):
                    expected = list(current)
                    expected[joint] += delta
                    self.assertEqual(jog_target(current, key, 1.0, [-1] * 6, LIMITS),
                                     tuple(expected))
        self.assertEqual(len(KEY_MAP), 12)
        self.assertNotIn('4', KEY_MAP)

    def test_direction_can_be_reversed_for_one_joint(self):
        result = jog_target([0] * 6, 'e', 0.5, [-1, -1, 1, -1, -1, -1], LIMITS)
        self.assertEqual(result, (0, 0, 0.5, 0, 0, 0))

    def test_every_step_uses_actual_angles_instead_of_a_previous_target(self):
        first = jog_target([0] * 6, 'a', 1.0, [-1] * 6, LIMITS)
        actual = (0.8, 0, 0, 0, 0, 0)
        second = jog_target(actual, 'a', 1.0, [-1] * 6, LIMITS)
        self.assertEqual(first[0], 1.0)
        self.assertAlmostEqual(second[0], 1.8)

    def test_over_limit_is_rejected_not_clipped_or_wrapped(self):
        with self.assertRaises(ValueError):
            jog_target([169.5, 0, 0, 0, 0, 0], 'a', 1.0, [-1] * 6, LIMITS)

    def test_invalid_input_rejected(self):
        for signs in ([1], [0] * 6, [1, 1, 1, 1, 1, 2]):
            with self.subTest(signs=signs), self.assertRaises(ValueError):
                validate_signs(signs)
        for values in ([0] * 5, [math.nan] + [0] * 5, [math.inf] + [0] * 5):
            with self.subTest(values=values), self.assertRaises(ValueError):
                six_finite(values)
        for step in (0, 6, math.nan):
            with self.subTest(step=step), self.assertRaises(ValueError):
                jog_target([0] * 6, 'q', step, [-1] * 6, LIMITS)

    def test_all_zero_pose_is_a_valid_pose(self):
        self.assertEqual(LIMITS.check([0] * 6), (0.0,) * 6)


class PathTests(unittest.TestCase):
    def test_joint_space_segments_end_exactly_at_target(self):
        start = (0.0,) * 6
        target = (7.0, -3.0, 0, 1, 5, -2)
        points = waypoints(start, target, LIMITS, 2.0)
        self.assertEqual(points[-1], target)
        self.assertEqual(len(points), 4)
        for previous, current in zip([start] + points, points):
            self.assertLessEqual(max(abs(a - b) for a, b in zip(previous, current)), 2.0)

    def test_invalid_endpoint_rejected_before_planning(self):
        with self.assertRaises(ValueError):
            waypoints([0] * 6, [180] * 6, LIMITS)

    def test_no_wrap_from_169_to_minus_169(self):
        points = waypoints([169, 0, 0, 0, 0, 0], [-169, 0, 0, 0, 0, 0], LIMITS)
        self.assertGreater(len(points), 160)
        self.assertLess(points[0][0], 169)
        self.assertGreater(points[0][0], 160)


class LimitsTests(unittest.TestCase):
    def test_urdf_intersection_and_margin(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'robot.urdf'
            parts = [f'<joint name="{name}" type="revolute">'
                     '<limit lower="-3.0" upper="3.0"/></joint>'
                     for name in JOINT_NAMES]
            path.write_text('<robot>' + ''.join(parts) + '</robot>', encoding='utf-8')
            result = load_limits(path, 2.0)
            self.assertAlmostEqual(result.lower[0], -166.0)
            self.assertAlmostEqual(result.upper[1], 133.0)

    def test_missing_urdf_joints_are_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'robot.urdf'
            path.write_text('<robot/>', encoding='utf-8')
            with self.assertRaises(ValueError):
                load_limits(path)


def window_with_samples(angles=(0.0,) * 6):
    window = FeedbackWindow()
    for index in range(11):
        received = 99.5 + 0.05 * index
        window.add(received, received + 1000.0, angles)
    return window


class FeedbackTests(unittest.TestCase):
    def test_fresh_stable_zero_feedback_is_accepted(self):
        window = window_with_samples()
        self.assertEqual(window.stationary(100.0, 1100.0).angles, (0.0,) * 6)

    def test_stale_receipt_time_is_rejected(self):
        with self.assertRaises(ValueError):
            window_with_samples().fresh(101.0, 1101.0)

    def test_old_source_time_is_rejected_even_if_receipt_is_recent(self):
        window = FeedbackWindow()
        window.add(100.0, 1000.0, [0] * 6)
        with self.assertRaises(ValueError):
            window.fresh(100.0, 1100.0)

    def test_future_source_time_is_rejected(self):
        with self.assertRaises(ValueError):
            window_with_samples().fresh(100.0, 1099.0)

    def test_duplicate_and_backward_timestamps_do_not_refresh_window(self):
        window = window_with_samples()
        for source in (1100.0, 1099.0):
            with self.assertRaises(ValueError):
                window.add(100.1, source, [0] * 6)
        self.assertEqual(window.sequence, 11)
        self.assertEqual(window.samples[-1].received, 100.0)

    def test_moving_joint_is_not_stationary(self):
        window = window_with_samples()
        window.add(100.05, 1100.05, [1, 0, 0, 0, 0, 0])
        with self.assertRaises(ValueError):
            window.stationary(100.05, 1100.05)

    def test_feedback_gap_is_not_stationary(self):
        window = FeedbackWindow()
        for instant in (99.0, 99.1, 99.2, 100.0):
            window.add(instant, instant + 1000.0, [0] * 6)
        with self.assertRaises(ValueError):
            window.stationary(100.0, 1100.0)

    def test_old_samples_cannot_prove_a_new_command_has_arrived(self):
        window = window_with_samples()
        with self.assertRaises(ValueError):
            window.stationary(100.0, 1100.0, after_sequence=window.sequence, target=[0] * 6)

    def test_stable_but_wrong_position_is_not_arrived(self):
        with self.assertRaises(ValueError):
            window_with_samples().stationary(100.0, 1100.0, target=[1, 0, 0, 0, 0, 0])

    def test_new_stable_target_feedback_can_prove_arrival(self):
        window = window_with_samples()
        previous = window.sequence
        target = (1.0, 0, 0, 0, 0, 0)
        for index in range(1, 12):
            instant = 100.0 + 0.05 * index
            window.add(instant, instant + 1000.0, target)
        sample = window.stationary(100.55, 1100.55, after_sequence=previous, target=target)
        self.assertEqual(sample.angles, target)


class HomeTests(unittest.TestCase):
    def test_first_start_uses_current_pose_without_writing_a_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'home.json'
            home, loaded = load_home(path, [1, 2, 3, 4, 5, 6], LIMITS)
            self.assertFalse(loaded)
            self.assertEqual(home, (1, 2, 3, 4, 5, 6))
            self.assertFalse(path.exists())

    def test_save_and_reload_home(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'new' / 'home.json'
            save_home(path, [1, 2, 3, 4, 5, 6], LIMITS)
            home, loaded = load_home(path, [0] * 6, LIMITS)
            self.assertTrue(loaded)
            self.assertEqual(home, (1, 2, 3, 4, 5, 6))
            self.assertFalse(path.with_name('home.json.tmp').exists())

    def test_bad_saved_home_never_silently_becomes_zero(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'home.json'
            path.write_text(json.dumps({'angles': [0] * 6}), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_home(path, [1] * 6, LIMITS)


if __name__ == '__main__':
    unittest.main()
