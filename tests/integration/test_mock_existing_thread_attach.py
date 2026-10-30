import json
import platform
from pathlib import Path
import subprocess
import tempfile
import unittest


@unittest.skipUnless(platform.system() == 'Linux' and platform.machine() == 'x86_64',
                     'requires Linux x86-64 ptrace hardware breakpoints')
class ExistingThreadAttachTest(unittest.TestCase):
    def test_existing_owned_threads_are_enumerated_traced_and_detached(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temp:
            binary = str(Path(temp)/'trace')
            subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-O0', '-no-pie',
                            '-pthread', str(root/'tools/diagnostics/mock_existing_thread_attach.cpp'),
                            '-o', binary], check=True, timeout=30)
            result = subprocess.run([binary], check=True, capture_output=True, text=True, timeout=12)
        report = json.loads(result.stdout)
        self.assertTrue(report['passed'])
        self.assertEqual(report['enumerated_threads'], 3)
        self.assertEqual(report['configured_workers'], 2)
        self.assertEqual(report['detached_workers'], 2)
        self.assertEqual(report['hits_per_worker'], [3, 3])
        self.assertTrue(report['owned_child_exited'])


if __name__ == '__main__':
    unittest.main()
