import json
import platform
from pathlib import Path
import subprocess
import tempfile
import unittest


@unittest.skipUnless(platform.system() == 'Linux' and platform.machine() == 'x86_64',
                     'requires Linux x86-64 ptrace hardware breakpoints')
class PostLoadTraceTest(unittest.TestCase):
    def test_child_registers_capture_actual_load_after_snapshot(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temp:
            binary = str(Path(temp) / 'trace')
            subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-O0', '-no-pie',
                            str(root/'tools/diagnostics/mock_post_load_trace.cpp'), '-o', binary],
                           check=True, timeout=30)
            result = subprocess.run([binary], check=True, capture_output=True, text=True, timeout=10)
        unchanged, modified = [json.loads(line) for line in result.stdout.splitlines()]
        self.assertTrue(unchanged['passed'] and modified['passed'])
        self.assertEqual(unchanged['snapshot'], unchanged['loaded_xmm0'])
        self.assertEqual(unchanged['snapshot'], modified['snapshot'])
        self.assertAlmostEqual(modified['loaded_xmm0'][0], .9, places=6)
        self.assertNotEqual(modified['snapshot'][0], modified['loaded_xmm0'][0])


if __name__ == '__main__':
    unittest.main()
