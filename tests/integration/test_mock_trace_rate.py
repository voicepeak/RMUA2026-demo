import json
import platform
from pathlib import Path
import subprocess
import tempfile
import unittest


@unittest.skipUnless(platform.system() == 'Linux' and platform.machine() == 'x86_64',
                     'requires Linux x86-64 ptrace hardware breakpoints')
class TraceRateTest(unittest.TestCase):
    def test_rate_probe_records_baseline_and_owned_tracee(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temp:
            binary = str(Path(temp)/'trace')
            subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-O0', '-no-pie',
                            str(root/'tools/diagnostics/mock_trace_rate.cpp'), '-o', binary],
                           check=True, timeout=30)
            result = subprocess.run([binary], check=True, capture_output=True, text=True, timeout=15)
        baseline, traced = [json.loads(x) for x in result.stdout.splitlines()]
        self.assertEqual([baseline['case'], traced['case']], ['baseline', 'traced'])
        self.assertTrue(baseline['passed'] and traced['passed'])
        self.assertEqual([baseline['samples'], traced['samples']], [200, 200])
        self.assertEqual([baseline['hits'], traced['hits']], [0, 200])
        self.assertTrue(all(row[0] >= 0 for case in (baseline, traced) for row in case['rows']))


if __name__ == '__main__':
    unittest.main()
