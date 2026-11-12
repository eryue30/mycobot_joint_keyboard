# -*- coding: utf-8 -*-
"""只在临时目录验证补丁，不修改工作区内的真实驱动。"""
import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'tools' / 'patch_zero_feedback.py'
SPEC = importlib.util.spec_from_file_location('patch_zero_feedback', SCRIPT)
PATCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PATCH)


class PatchTests(unittest.TestCase):
    def test_patch_backup_readback_and_idempotence(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'driver.py'
            original = ('def publish_joint_states(angles):\r\n'
                        '    ' + PATCH.OLD + '\r\n        return\r\n')
            path.write_bytes(original.encode('utf-8'))
            self.assertTrue(PATCH.patch(path))
            self.assertEqual(path.with_name(path.name + '.before_joint_keyboard.bak').read_bytes(),
                             original.encode('utf-8'))
            self.assertEqual(path.read_bytes(), original.replace(PATCH.OLD, PATCH.NEW).encode('utf-8'))
            self.assertFalse(PATCH.patch(path))

    def test_unknown_driver_is_not_modified(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'driver.py'
            path.write_text('x = 1\n', encoding='utf-8')
            with self.assertRaises(ValueError):
                PATCH.patch(path)
            self.assertEqual(path.read_text(encoding='utf-8'), 'x = 1\n')


if __name__ == '__main__':
    unittest.main()
