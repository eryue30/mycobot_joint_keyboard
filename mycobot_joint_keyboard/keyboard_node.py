# -*- coding: utf-8 -*-
"""六关节点动 ROS 2 客户端，不创建串口连接，不发布 /joint_states。

数据链：真实驱动 -> /joint_states -> 本客户端 -> /set_angles -> 同一真实驱动。
每条指令重新读取实际角度；服务 flag=True 之后还要等真实反馈到位。
"""
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mycobot_interfaces.srv import SetAngles
from sensor_msgs.msg import JointState

from .core import (FeedbackWindow, JOINT_NAMES, KEY_MAP, KEY_PAIRS, jog_target,
                   load_home, load_limits, save_home, six_finite,
                   validate_signs, waypoints)
from .terminal import ExitRequested, KeyboardTerminal


class MotionFault(RuntimeError):
    """下发结果或到位状态不确定：锁住后续运动，要求检查后重新启动。"""


def angle_text(values):
    return '[' + ', '.join(f'{value:.2f}' for value in values) + ']°'


class JointKeyboard(Node):
    def __init__(self, terminal):
        super().__init__('mycobot_joint_keyboard')
        self.terminal = terminal
        defaults = {
            'step_deg': 1.0, 'speed': 10, 'cw_signs': [-1] * 6,
            'soft_margin_deg': 2.0, 'segment_deg': 2.0,
            'feedback_timeout_sec': 0.5, 'service_timeout_sec': 2.0,
            'move_timeout_sec': 10.0, 'settle_sec': 0.35,
            'settle_spread_deg': 0.2, 'reach_tolerance_deg': 0.35,
            'expected_feedback_node': 'listen_real_service',
            'expected_feedback_namespace': '/',
            'init_angles': [0.0] * 6,
            'home_file': '~/.ros/mycobot_joint_keyboard/home.json',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.options = {name: self.get_parameter(name).value for name in defaults}
        self._validate_options()
        if self.get_parameter('use_sim_time').value:
            raise ValueError('实机键盘客户端不接受 use_sim_time=true')
        urdf = (Path(get_package_share_directory('mycobot_description'))
                / 'urdf/mycobot_280_m5/mycobot_280_m5.urdf')
        self.limits = load_limits(urdf, self.options['soft_margin_deg'])
        self.init_pose = self.limits.check(self.options['init_angles'])
        self.feedback = FeedbackWindow()
        self.last_invalid_message = ''
        self.faulted = False
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.client = self.create_client(SetAngles, '/set_angles')
        self.home_pose = None

    def _validate_options(self):
        """参数写错时拒绝启动，不能静默转换成一个意外的大步长。"""
        bounds = {
            'step_deg': (0.1, 5.0), 'soft_margin_deg': (0.0, 10.0),
            'segment_deg': (0.1, 2.0), 'feedback_timeout_sec': (0.1, 1.0),
            'service_timeout_sec': (0.2, 5.0), 'move_timeout_sec': (2.0, 60.0),
            'settle_sec': (0.2, 2.0), 'settle_spread_deg': (0.01, 1.0),
            'reach_tolerance_deg': (0.1, 1.0),
        }
        for name, (lower, upper) in bounds.items():
            value = self.options[name]
            if not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError(f'{name} 必须是有限数值')
            if not lower <= value <= upper:
                raise ValueError(f'{name} 应在 [{lower}, {upper}] 内')
        speed = self.options['speed']
        if not isinstance(speed, int) or not 1 <= speed <= 30:
            raise ValueError('本点动工具的 speed 必须是 1～30 的整数')
        validate_signs(self.options['cw_signs'])
        six_finite(self.options['init_angles'], 'init_angles')
        if not self.options['expected_feedback_node']:
            raise ValueError('expected_feedback_node 不能为空')

    def on_joints(self, msg):
        """按关节名称重排，不假定 msg.position 的数组顺序。"""
        try:
            if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
                raise ValueError('JointState 关节名重复或与位置数组长度不同')
            mapping = dict(zip(msg.name, msg.position))
            if not all(name in mapping for name in JOINT_NAMES):
                raise ValueError('JointState 缺少所需的六个关节')
            values = [math.degrees(mapping[name]) for name in JOINT_NAMES]
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.feedback.add(time.monotonic(), stamp, values)
            self.last_invalid_message = ''
        except (ValueError, TypeError) as exc:
            # 丢弃异常消息，不刷新“最近有效反馈”的时间，也不伪造全零姿态。
            self.last_invalid_message = str(exc)

    def _ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_graph(self):
        """只接受一个真实反馈发布者，且 /set_angles 应由同一驱动提供。

        这能发现 GUI/模拟器混入话题等常见配置错误，但不是安全认证或硬件互锁。
        其他控制客户端也应由操作者主动退出。
        """
        publishers = self.get_publishers_info_by_topic('/joint_states')
        expected = (self.options['expected_feedback_node'],
                    self.options['expected_feedback_namespace'])
        if len(publishers) != 1:
            raise ValueError(f'/joint_states 应只有一个真实发布者，当前 {len(publishers)} 个')
        actual = (publishers[0].node_name, publishers[0].node_namespace)
        if actual != expected:
            raise ValueError(f'反馈节点是 {actual}，预期 {expected}；拒绝用滑块/模拟反馈控制')
        providers = []
        for name, namespace in self.get_node_names_and_namespaces():
            services = self.get_service_names_and_types_by_node(name, namespace)
            if any(service == '/set_angles' and 'mycobot_interfaces/srv/SetAngles' in types
                   for service, types in services):
                providers.append((name, namespace))
        if providers != [expected] or not self.client.service_is_ready():
            raise ValueError('/set_angles 必须由同一个唯一实机驱动提供')

    def current(self, after_sequence=0, target=None):
        """取得新鲜、连续且停稳的实际角度。"""
        return self.feedback.stationary(
            time.monotonic(), self._ros_now(),
            settle_sec=self.options['settle_sec'],
            spread_deg=self.options['settle_spread_deg'],
            timeout=self.options['feedback_timeout_sec'],
            after_sequence=after_sequence, target=target,
            tolerance_deg=self.options['reach_tolerance_deg'])

    def spin_and_discard(self):
        """处理反馈；动作执行期间丢弃运动按键，但 4 随时退出客户端。"""
        rclpy.spin_once(self, timeout_sec=0.02)
        if not rclpy.ok():
            raise ExitRequested()
        self.terminal.poll(0.0)

    def wait_for_release(self):
        """终端没有 key-up 事件；要求 0.35 秒输入安静期，抑制长按重复。

        本工具是“一次点按一步”。正在动作时快速连按的后续键会被丢弃。
        """
        quiet_since = time.monotonic()
        while time.monotonic() - quiet_since < 0.35:
            rclpy.spin_once(self, timeout_sec=0.02)
            if not rclpy.ok():
                raise ExitRequested()
            if self.terminal.poll(0.0) is not None:
                quiet_since = time.monotonic()

    def prepare(self):
        """启动只读检查，不会自动回零，也不会发任何运动指令。"""
        print('等待唯一实机驱动和连续真实反馈；启动不会移动机械臂。', flush=True)
        deadline, reason = time.monotonic() + 15.0, ''
        while time.monotonic() < deadline:
            self.spin_and_discard()
            try:
                self.check_graph()
                initial = self.limits.check(self.current().angles)
                self.home_pose, loaded = load_home(
                    self.options['home_file'], initial, self.limits)
                print('当前实际角度：' + angle_text(initial))
                print(('已读取保存的 home：' if loaded else '尚无保存文件，本次 home=启动姿态：')
                      + angle_text(self.home_pose))
                self.print_help()
                return
            except (ValueError, RuntimeError) as exc:
                reason = str(exc)
        raise RuntimeError('启动检查未通过：' + reason + '；' + self.last_invalid_message)

    def print_help(self):
        print('\n一次点按一步，不需要回车：')
        for index, pair in enumerate(KEY_PAIRS):
            sign = self.options['cw_signs'][index]
            print(f'  J{index + 1}: {pair[0]} 顺时针({sign:+d}×步长) / '
                  f'{pair[1]} 逆时针({-sign:+d}×步长)')
        print('  1: 选择 init；2: 选择 home；这两项均需在 5 秒内按回车确认')
        print('  3: 把停稳的当前姿态保存为 home；4: 退出；Ctrl+C 也可退出')
        print(f"步长={self.options['step_deg']}°，速度参数={self.options['speed']}。")
        print('4 是退出客户端，无法通过现有服务保证中止已经发送的运动。')
        print('当前没有碰撞规划；先检查相机、夹爪和线缆的活动空间。\n', flush=True)

    def _send_segment(self, target):
        """每段只发送一次；无响应/失败时不自动重试，避免不确定运动叠加。"""
        self.check_graph()
        self.limits.check(self.current().angles)
        target = self.limits.check(target)
        before = self.feedback.sequence
        request = SetAngles.Request()
        for index, value in enumerate(target, start=1):
            setattr(request, f'joint_{index}', float(value))
        request.speed = int(self.options['speed'])
        future = self.client.call_async(request)
        deadline = time.monotonic() + self.options['service_timeout_sec']
        while not future.done():
            self.spin_and_discard()
            if time.monotonic() > deadline:
                future.cancel()  # 只取消本地等待，不能撤销服务器可能已执行的动作。
                raise MotionFault('服务响应超时，运动状态不确定；不重发，请检查实机')
        try:
            response = future.result()
        except Exception as exc:
            raise MotionFault('服务调用异常，运动状态不确定：' + str(exc)) from exc
        if response is None or not response.flag:
            raise MotionFault('驱动返回失败；请检查实机，不自动重复下发')

        # flag=True 仅表示驱动调用已发送；以下真实反馈才用于判断软件到位。
        deadline = time.monotonic() + self.options['move_timeout_sec']
        reason, next_graph_check = '', 0.0
        while time.monotonic() < deadline:
            self.spin_and_discard()
            now = time.monotonic()
            try:
                if now >= next_graph_check:
                    self.check_graph()
                    next_graph_check = now + 0.2
                self.feedback.fresh(now, self._ros_now(), self.options['feedback_timeout_sec'])
            except (ValueError, RuntimeError) as exc:
                raise MotionFault('动作中反馈检查失败：' + str(exc)) from exc
            try:
                return self.current(after_sequence=before, target=target).angles
            except ValueError as exc:
                reason = str(exc)
        raise MotionFault('到位等待超时：' + reason)

    def move_to(self, target):
        self.check_graph()
        start = self.limits.check(self.current().angles)
        goal = self.limits.check(target)
        if max(abs(a - b) for a, b in zip(start, goal)) < 0.01:
            print('当前已在目标附近，不发送运动。', flush=True)
            return
        points = waypoints(start, goal, self.limits, self.options['segment_deg'])
        for index, point in enumerate(points, start=1):
            print(f'发送第 {index}/{len(points)} 段：{angle_text(point)}', flush=True)
            try:
                actual = self._send_segment(point)
            except ExitRequested:
                raise
            except Exception as exc:
                # 只要一段执行结果不确定，就不继续后面的段，也不允许下一条运动。
                self.faulted = True
                raise MotionFault(str(exc)) from exc
        print('真实反馈已到位并停稳：' + angle_text(actual), flush=True)

    def run_keyboard(self):
        armed_key, armed_until = None, 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            key = self.terminal.poll(0.02)
            if key is None:
                continue
            try:
                # 用回车确认大幅回位，避免把“长按 1/2 的自动重复”当成二次确认。
                confirming = key in ('\r', '\n') and armed_key in ('1', '2')
                if key not in ('1', '2') and not confirming:
                    armed_key = None
                if self.faulted and (key in KEY_MAP or key in ('1', '2') or confirming):
                    print('运动已锁定。先检查实机停止和反馈，再按 4 退出并重新启动。')
                    continue
                if key in KEY_MAP:
                    self.check_graph()
                    current = self.current().angles
                    target = jog_target(current, key, self.options['step_deg'],
                                        self.options['cw_signs'], self.limits)
                    self.move_to(target)
                elif key in ('1', '2'):
                    self.check_graph()
                    self.limits.check(self.current().angles)
                    target = self.init_pose if key == '1' else self.home_pose
                    print(('init' if key == '1' else 'home') + ' 目标：' + angle_text(target))
                    print('检查整条动作的空间后，在 5 秒内按回车执行；其他功能键取消。', flush=True)
                    armed_key, armed_until = key, time.monotonic() + 5.0
                elif confirming:
                    selected, armed_key = armed_key, None
                    if time.monotonic() > armed_until:
                        print('确认已超时，请重新选择 1 或 2。', flush=True)
                    else:
                        self.move_to(self.init_pose if selected == '1' else self.home_pose)
                elif key == '3':
                    self.check_graph()
                    self.home_pose = self.limits.check(self.current().angles)
                    save_home(self.options['home_file'], self.home_pose, self.limits)
                    print('home 已保存：' + angle_text(self.home_pose), flush=True)
                else:
                    print('未定义的按键；q/a～y/h 点动，1/2 回位，3 保存，4 退出。', flush=True)
            except MotionFault as exc:
                self.faulted = True
                print('运动故障，后续运动已锁定：' + str(exc), flush=True)
            except (ValueError, RuntimeError, OSError) as exc:
                print('本次操作拒绝执行：' + str(exc), flush=True)
            finally:
                self.wait_for_release()


def main(args=None):
    node = None
    rclpy.init(args=args)
    try:
        with KeyboardTerminal() as terminal:
            node = JointKeyboard(terminal)
            node.prepare()
            node.run_keyboard()
    except (ExitRequested, KeyboardInterrupt):
        print('\n已退出键盘客户端。已发送的运动不保证因退出而停止。', flush=True)
    except Exception as exc:
        print('\n启动或运行失败：' + str(exc), flush=True)
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
