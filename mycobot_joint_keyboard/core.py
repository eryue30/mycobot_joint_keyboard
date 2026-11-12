# -*- coding: utf-8 -*-
"""不依赖 ROS 的计算与数据检查，便于在未连接机械臂时测试。

所有关节角在本模块内均使用“度”；仅在接收 JointState 时由节点转成度。
不要对关节角做 +/-180 度归一化：有限行程关节不能跨限位绕回。
"""
from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


# 与用户现有实机驱动、280 M5 URDF 的六个活动关节名称一致。
JOINT_NAMES = (
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
)

# 每对按键依次为“用户定义的顺时针 / 逆时针”。
# 具体顺时针对应角度增大还是减小，由 cw_signs 的各个元素决定。
KEY_PAIRS = ('qa', 'ws', 'ed', 'rf', 'tg', 'yh')
KEY_MAP = {key: (joint, 1 if direction == 0 else -1)
           for joint, pair in enumerate(KEY_PAIRS)
           for direction, key in enumerate(pair)}

# 本工具额外施加的保守软件范围，并不宣称为所有固件版本的硬件规格。
# 运行时会与实际安装的 M5 URDF 范围取交集，再留出 soft_margin_deg。
TOOL_MIN_DEG = (-168.0, -135.0, -150.0, -145.0, -155.0, -180.0)
TOOL_MAX_DEG = (168.0, 135.0, 150.0, 145.0, 160.0, 180.0)


def six_finite(values, label='关节角'):
    """统一检查六维数值；无效读取不能被当作六个零继续下发。"""
    if len(values) != 6:
        raise ValueError(f'{label}必须有 6 个元素')
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f'{label}含 NaN 或无穷大')
    return result


@dataclass(frozen=True)
class JointLimits:
    lower: tuple
    upper: tuple

    def __post_init__(self):
        lower = six_finite(self.lower, '下限')
        upper = six_finite(self.upper, '上限')
        if any(lo >= hi for lo, hi in zip(lower, upper)):
            raise ValueError('关节下限必须小于上限')

    def check(self, angles):
        """越界时拒绝整条命令；不裁剪，也不把角度绕回另一端。"""
        values = six_finite(angles)
        for index, (value, lo, hi) in enumerate(zip(values, self.lower, self.upper)):
            if not lo <= value <= hi:
                raise ValueError(
                    f'J{index + 1}={value:.2f}° 超出本工具允许范围 '
                    f'[{lo:.2f}, {hi:.2f}]°')
        return values


def load_limits(urdf_path, margin_deg=2.0):
    """读取各关节自身的 limit，避免把底座固定关节误认为 J1。"""
    if not math.isfinite(margin_deg) or not 0.0 <= margin_deg <= 10.0:
        raise ValueError('soft_margin_deg 应在 0～10 度之间')
    root = ET.parse(urdf_path).getroot()
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    lower, upper = [], []
    for index, name in enumerate(JOINT_NAMES):
        joint = joints.get(name)
        if joint is None or joint.get('type') != 'revolute':
            raise ValueError(f'URDF 缺少有限行程活动关节 {name}')
        limit = joint.find('limit')
        if limit is None:
            raise ValueError(f'URDF 缺少关节范围 {name}')
        lo = math.degrees(float(limit.attrib['lower']))
        hi = math.degrees(float(limit.attrib['upper']))
        lower.append(max(lo, TOOL_MIN_DEG[index]) + margin_deg)
        upper.append(min(hi, TOOL_MAX_DEG[index]) - margin_deg)
    return JointLimits(tuple(lower), tuple(upper))


def validate_signs(signs):
    """+1 为关节读数增大，-1 为减小；每个关节单独标定方向。"""
    if len(signs) != 6 or any(sign not in (-1, 1) for sign in signs):
        raise ValueError('cw_signs 必须是 6 个 +1 或 -1')
    return tuple(int(sign) for sign in signs)


def jog_target(current, key, step_deg, cw_signs, limits):
    """用这一次新鲜的真实角度生成目标，仅修改选中的一个分量。"""
    if key not in KEY_MAP:
        raise ValueError('不是关节点动键')
    if not math.isfinite(step_deg) or not 0.1 <= step_deg <= 5.0:
        raise ValueError('step_deg 应在 0.1～5.0 度之间')
    signs = validate_signs(cw_signs)
    target = list(limits.check(current))
    joint, direction = KEY_MAP[key]
    target[joint] += direction * signs[joint] * step_deg
    return limits.check(target)


def waypoints(start, target, limits, segment_deg=2.0):
    """沿关节空间直线分段，每一段的每个关节变化不超过 segment_deg。

    分段便于逐段核对反馈，不等同于碰撞规划，也不保证夹爪沿直线运动。
    不能把两组合法端点当作整条路径必然无碰撞的证明。
    """
    begin, end = limits.check(start), limits.check(target)
    if not math.isfinite(segment_deg) or not 0.1 <= segment_deg <= 2.0:
        raise ValueError('segment_deg 应在 0.1～2.0 度之间')
    maximum = max(abs(b - a) for a, b in zip(begin, end))
    count = max(1, int(math.ceil(maximum / segment_deg)))
    return [limits.check(tuple(a + (b - a) * number / count
                               for a, b in zip(begin, end)))
            for number in range(1, count + 1)]


@dataclass(frozen=True)
class Sample:
    received: float  # 接收端 monotonic 时间，避免系统时钟调整影响间隔判断。
    source: float    # JointState.header.stamp 的 ROS 时间，检查源消息是否过期。
    angles: tuple
    sequence: int


class FeedbackWindow:
    """保存真实反馈历史；同时检查新鲜程度、连续性和静止状态。"""
    def __init__(self):
        self.samples = deque(maxlen=500)
        self.sequence = 0

    def add(self, received, source, angles):
        values = six_finite(angles)
        if not math.isfinite(source) or source <= 0:
            raise ValueError('关节反馈时间戳无效')
        if self.samples and source <= self.samples[-1].source:
            raise ValueError('关节反馈时间戳重复或倒退')
        self.sequence += 1
        self.samples.append(Sample(received, source, values, self.sequence))

    def fresh(self, now, ros_now, timeout=0.5):
        if not self.samples:
            raise ValueError('尚未收到包含全部六关节的真实反馈')
        sample = self.samples[-1]
        if not 0.0 <= now - sample.received <= timeout:
            raise ValueError('真实关节反馈超过允许时间未更新')
        if not -0.05 <= ros_now - sample.source <= timeout:
            raise ValueError('JointState 源时间戳过旧或在未来')
        return sample

    def stationary(self, now, ros_now, settle_sec=0.35, spread_deg=0.2,
                   timeout=0.5, after_sequence=0, target=None, tolerance_deg=0.35):
        """检查连续静止窗口；到位判断只使用发令之后收到的新反馈。"""
        latest = self.fresh(now, ros_now, timeout)
        # 保留覆盖窗口开始时刻的一条样本，避免恰好少一个采样周期。
        candidates = [sample for sample in self.samples
                      if sample.sequence > after_sequence]
        if len(candidates) < 3:
            raise ValueError('等待更多连续反馈')
        threshold = now - settle_sec
        before = [sample for sample in candidates if sample.received <= threshold]
        if not before:
            raise ValueError('等待机械臂连续停稳')
        first = before[-1]
        window = [sample for sample in candidates if sample.sequence >= first.sequence]
        gaps = [b.received - a.received for a, b in zip(window, window[1:])]
        if any(gap > timeout for gap in gaps):
            raise ValueError('静止窗口内出现反馈中断')
        for joint in range(6):
            values = [sample.angles[joint] for sample in window]
            if max(values) - min(values) > spread_deg:
                raise ValueError('关节仍在运动，请停稳后重试')
        if target is not None:
            expected = six_finite(target)
            if any(abs(sample.angles[joint] - expected[joint]) > tolerance_deg
                   for sample in window for joint in range(6)):
                raise ValueError('尚未到达目标角度并停稳')
        return latest


def save_home(path, angles, limits):
    """先写临时文件再替换，避免关闭程序时留下写到一半的 home 文件。"""
    values = limits.check(angles)
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + '.tmp')
    temporary.write_text(json.dumps({'schema_version': 1, 'units': 'degree',
                                     'joint_names': list(JOINT_NAMES),
                                     'angles': list(values)},
                                    ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(destination)


def load_home(path, startup, limits):
    """第一次用当前启动姿态作 home；存在损坏文件时拒绝启动而不静默回退。"""
    source = Path(path).expanduser()
    if not source.exists():
        return limits.check(startup), False
    data = json.loads(source.read_text(encoding='utf-8'))
    if (data.get('schema_version') != 1 or data.get('units') != 'degree'
            or data.get('joint_names') != list(JOINT_NAMES)):
        raise ValueError('home 文件格式或关节顺序不匹配')
    return limits.check(data['angles']), True
