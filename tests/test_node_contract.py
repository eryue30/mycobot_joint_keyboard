# -*- coding: utf-8 -*-
"""用内存中的 ROS 替身验证客户端控制流程，不连接任何真实 ROS 域。

这不是 Ubuntu/rclpy 通信测试。目的是验证：不能把服务成功当到位、
没有有效反馈不发令、服务失败/超时不重发、反馈异常后锁住后续运动。
"""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from mycobot_joint_keyboard.core import FeedbackWindow, JOINT_NAMES, JointLimits


def load_node_with_ros_stubs():
    modules = {name: ModuleType(name) for name in (
        'rclpy', 'rclpy.node', 'rclpy.qos', 'ament_index_python',
        'ament_index_python.packages', 'mycobot_interfaces',
        'mycobot_interfaces.srv', 'sensor_msgs', 'sensor_msgs.msg')}
    modules['rclpy.node'].Node = type('Node', (), {})
    modules['rclpy.qos'].qos_profile_sensor_data = object()
    modules['rclpy'].ok = lambda: True
    modules['rclpy'].spin_once = lambda node, timeout_sec: node.fake_spin()
    modules['ament_index_python.packages'].get_package_share_directory = lambda name: ''
    modules['mycobot_interfaces.srv'].SetAngles = SimpleNamespace(Request=SimpleNamespace)
    modules['sensor_msgs.msg'].JointState = SimpleNamespace
    path = Path(__file__).resolve().parents[1] / 'mycobot_joint_keyboard' / 'keyboard_node.py'
    spec = importlib.util.spec_from_file_location('mycobot_joint_keyboard._node_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


NODE = load_node_with_ros_stubs()


class FakeFuture:
    def __init__(self, flag=True, pending=False):
        self.flag, self.pending, self.cancelled = flag, pending, False

    def done(self):
        return not self.pending

    def result(self):
        return SimpleNamespace(flag=self.flag)

    def cancel(self):
        self.cancelled = True


class FakeClient:
    def __init__(self, mode='reach'):
        self.mode = mode
        self.sent = []
        self.last_future = None

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.sent.append(tuple(getattr(request, f'joint_{index}') for index in range(1, 7)))
        self.last_future = FakeFuture(flag=self.mode != 'failed', pending=self.mode == 'no_reply')
        return self.last_future


class ControllerContractTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.clock_patch = patch.object(NODE.time, 'monotonic', lambda: self.now)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.output = contextlib.redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)
        self.node = NODE.JointKeyboard.__new__(NODE.JointKeyboard)
        self.node.options = {
            'expected_feedback_node': 'listen_real_service',
            'expected_feedback_namespace': '/', 'service_timeout_sec': 2.0,
            'feedback_timeout_sec': 0.5, 'move_timeout_sec': 3.0,
            'settle_sec': 0.35, 'settle_spread_deg': 0.2,
            'reach_tolerance_deg': 0.35, 'speed': 10, 'segment_deg': 2.0,
            'step_deg': 1.0, 'cw_signs': [-1] * 6,
        }
        self.node.limits = JointLimits((-170.0,) * 6, (170.0,) * 6)
        self.node.feedback = FeedbackWindow()
        self.start = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        for index in range(11):
            instant = 99.5 + index * 0.05
            self.node.feedback.add(instant, instant + 1000.0, self.start)
        self.node.faulted = False
        self.node.last_invalid_message = ''
        self.node.client = FakeClient()
        self.node.terminal = SimpleNamespace(poll=lambda timeout: None)
        self.node._ros_now = lambda: self.now + 1000.0
        self.node.fake_spin = self.spin
        self.publishers = [SimpleNamespace(node_name='listen_real_service', node_namespace='/')]
        self.node.get_publishers_info_by_topic = lambda topic: self.publishers
        self.node.get_node_names_and_namespaces = lambda: [('listen_real_service', '/')]
        self.node.get_service_names_and_types_by_node = lambda name, namespace: [
            ('/set_angles', ['mycobot_interfaces/srv/SetAngles'])]

    def spin(self):
        self.now += 0.1
        if self.node.client.mode == 'stale':
            return
        target = (self.node.client.sent[-1] if self.node.client.sent
                  and self.node.client.mode == 'reach' else self.start)
        self.node.feedback.add(self.now, self.now + 1000.0, target)

    def test_valid_command_waits_for_new_real_feedback(self):
        self.node.move_to((2, 2, 3, 4, 5, 6))
        self.assertEqual(len(self.node.client.sent), 1)
        self.assertGreaterEqual(self.now, 100.4)
        self.assertFalse(self.node.faulted)

    def test_acknowledged_but_unmoved_is_not_success_and_is_not_retried(self):
        self.node.client.mode = 'no_movement'
        with self.assertRaises(NODE.MotionFault):
            self.node.move_to((2, 2, 3, 4, 5, 6))
        self.assertEqual(len(self.node.client.sent), 1)
        self.assertTrue(self.node.faulted)

    def test_false_service_result_locks_motion(self):
        self.node.client.mode = 'failed'
        with self.assertRaises(NODE.MotionFault):
            self.node.move_to((2, 2, 3, 4, 5, 6))
        self.assertEqual(len(self.node.client.sent), 1)
        self.assertTrue(self.node.faulted)

    def test_service_timeout_does_not_retry(self):
        self.node.client.mode = 'no_reply'
        with self.assertRaises(NODE.MotionFault):
            self.node.move_to((2, 2, 3, 4, 5, 6))
        self.assertEqual(len(self.node.client.sent), 1)
        self.assertTrue(self.node.client.last_future.cancelled)
        self.assertTrue(self.node.faulted)

    def test_feedback_loss_after_send_prevents_remaining_segments(self):
        self.node.client.mode = 'stale'
        with self.assertRaises(NODE.MotionFault):
            self.node.move_to((7, 2, 3, 4, 5, 6))
        self.assertEqual(len(self.node.client.sent), 1)
        self.assertTrue(self.node.faulted)

    def test_stale_feedback_before_send_results_in_no_command(self):
        self.now = 101.0
        with self.assertRaises(ValueError):
            self.node.move_to((2, 2, 3, 4, 5, 6))
        self.assertEqual(self.node.client.sent, [])

    def test_duplicate_joint_publishers_prevent_command(self):
        self.publishers.append(SimpleNamespace(node_name='joint_state_publisher_gui', node_namespace='/'))
        with self.assertRaises(ValueError):
            self.node.move_to((2, 2, 3, 4, 5, 6))
        self.assertEqual(self.node.client.sent, [])

    def test_gui_only_feedback_prevents_command(self):
        self.publishers[0].node_name = 'joint_state_publisher_gui'
        with self.assertRaises(ValueError):
            self.node.move_to((2, 2, 3, 4, 5, 6))
        self.assertEqual(self.node.client.sent, [])

    def test_multiple_service_providers_prevent_command(self):
        self.node.get_node_names_and_namespaces = lambda: [('listen_real_service', '/'), ('another', '/')]
        with self.assertRaises(ValueError):
            self.node.move_to((2, 2, 3, 4, 5, 6))
        self.assertEqual(self.node.client.sent, [])

    def test_out_of_range_target_prevents_command(self):
        with self.assertRaises(ValueError):
            self.node.move_to((180, 2, 3, 4, 5, 6))
        self.assertEqual(self.node.client.sent, [])

    def test_joint_message_reordered_by_name(self):
        message = SimpleNamespace(
            name=list(reversed(JOINT_NAMES)),
            position=[math.radians(value) for value in (6, 5, 4, 3, 2, 1)],
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1100, nanosec=100000000)))
        self.node.on_joints(message)
        for actual, expected in zip(self.node.feedback.samples[-1].angles, self.start):
            self.assertAlmostEqual(actual, expected)

    def test_incomplete_message_does_not_refresh_feedback(self):
        before = self.node.feedback.sequence
        message = SimpleNamespace(name=['wrong_joint'], position=[0.0])
        self.node.on_joints(message)
        self.assertEqual(self.node.feedback.sequence, before)

    def keyboard_commands(self, keys):
        """把有限按键送入真实按键分支；以 4 结束，不调用真实硬件。"""
        iterator = iter(keys)
        commands = []
        self.node.init_pose = (0.0,) * 6
        self.node.home_pose = (2.0,) * 6
        self.node.move_to = lambda target: commands.append(tuple(target))
        self.node.wait_for_release = lambda: None

        def poll(timeout):
            key = next(iterator)
            if key == '4':
                raise NODE.ExitRequested()
            return key

        self.node.terminal.poll = poll
        with self.assertRaises(NODE.ExitRequested):
            self.node.run_keyboard()
        return commands

    def test_held_or_repeated_init_key_does_not_confirm_motion(self):
        self.assertEqual(self.keyboard_commands(['1', '1', '1', '4']), [])

    def test_init_requires_enter(self):
        self.assertEqual(self.keyboard_commands(['1', '\n', '4']), [(0.0,) * 6])

    def test_home_requires_enter(self):
        self.assertEqual(self.keyboard_commands(['2', '\r', '4']), [(2.0,) * 6])

    def test_q_is_joint_one_jog_and_cancels_a_pending_init(self):
        commands = self.keyboard_commands(['1', 'q', '\n', '4'])
        self.assertEqual(commands, [(0, 2, 3, 4, 5, 6)])

    def test_quit_during_motion_prevents_remaining_commands(self):
        def quit_now(timeout):
            raise NODE.ExitRequested()
        self.node.terminal.poll = quit_now
        with self.assertRaises(NODE.ExitRequested):
            self.node.move_to((7, 2, 3, 4, 5, 6))
        self.assertEqual(len(self.node.client.sent), 1)


if __name__ == '__main__':
    unittest.main()
