# -*- coding: utf-8 -*-
"""Ubuntu 终端单字符读取；退出时恢复终端设置。"""
import os
import select
import sys


class ExitRequested(Exception):
    """用户按 4、输入关闭或按 Ctrl+C 时结束客户端。"""


class KeyboardTerminal:
    def __enter__(self):
        # Linux 专有模块在运行时导入，计算模块仍可以在 Windows 离线测试。
        import termios
        import tty
        if not sys.stdin.isatty():
            raise RuntimeError('请在 Ubuntu 的交互终端运行，不要使用输入重定向')
        self.termios = termios
        self.fd = sys.stdin.fileno()
        self.original = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # 单键立即生效，同时保留 Ctrl+C 信号处理。
        termios.tcflush(self.fd, termios.TCIFLUSH)
        return self

    def poll(self, timeout=0.0):
        """没有输入返回 None；始终优先识别 4，不把退出键积压到动作结束。"""
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return None
        value = os.read(self.fd, 1)
        if not value:
            raise ExitRequested()
        key = value.decode('ascii', errors='ignore').lower()
        if key == '4':
            raise ExitRequested()
        return key

    def __exit__(self, exc_type, exc_value, traceback):
        self.termios.tcsetattr(self.fd, self.termios.TCSANOW, self.original)
        return False
