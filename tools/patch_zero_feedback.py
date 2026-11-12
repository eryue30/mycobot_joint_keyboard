# -*- coding: utf-8 -*-
"""只修复旧驱动在前三关节为零时不发布真实 JointState 的问题。

用法：python3 patch_zero_feedback.py /实际路径/listen_real_service.py
停止驱动后运行。脚本备份原文件，修补后还需 colcon build 并重新启动驱动。
不会启动机械臂，不会发送串口或 ROS 运动命令。
"""
import argparse
import ast
from pathlib import Path
import shutil


OLD = ('if not angles or not isinstance(angles, list) or '
       'angles[0:3] == [0.0, 0.0, 0.0] or len(angles) != 6:')
NEW = 'if not angles or not isinstance(angles, list) or len(angles) != 6:'


def patch(path):
    """只接受已核对过的原条件，遇到其他版本失败关闭，不能猜测替换。"""
    source = Path(path).resolve(strict=True)
    original = source.read_bytes()
    text = original.decode('utf-8')
    ast.parse(text)
    if OLD not in text:
        if text.count(NEW) == 1:
            return False
        raise ValueError('未找到本次已核对的过滤条件；文件版本不同，请人工检查')
    if text.count(OLD) != 1:
        raise ValueError('目标条件不唯一，拒绝修改')
    updated = text.replace(OLD, NEW, 1)
    ast.parse(updated)
    backup = source.with_name(source.name + '.before_joint_keyboard.bak')
    if backup.exists() and backup.read_bytes() != original:
        raise ValueError('已有不同内容的备份，拒绝覆盖，请先检查备份')
    if not backup.exists():
        shutil.copy2(source, backup)
    temporary = source.with_name(source.name + '.joint_keyboard.tmp')
    temporary.write_bytes(updated.encode('utf-8'))
    temporary.replace(source)
    if source.read_text(encoding='utf-8').count(NEW) != 1:
        raise RuntimeError('修补后读回检查失败，请用备份检查恢复')
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('driver', type=Path)
    args = parser.parse_args()
    changed = patch(args.driver)
    print('已修复并备份，请重新构建 mycobot_280。' if changed else '已是修复后的条件，无需修改。')


if __name__ == '__main__':
    main()
