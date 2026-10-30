import ctypes
import json
import os
import platform
import select
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest


@unittest.skipUnless(platform.system() == 'Linux' and platform.machine() == 'x86_64',
                     'requires Linux x86-64 ptrace hardware breakpoints')
class MultiThreadTraceTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.binary = str(Path(self.temp.name) / 'trace')
        source = Path(__file__).resolve().parents[2]/'tools/diagnostics/mock_multithread_post_load.cpp'
        subprocess.run(['g++', '-std=c++14', '-Wall', '-Wextra', '-Werror', '-O0', '-no-pie',
                        '-pthread', str(source), '-o', self.binary], check=True, timeout=30)

    def test_all_owned_child_threads_hit_the_post_load_breakpoint(self):
        result = subprocess.run([self.binary], check=True, capture_output=True, text=True, timeout=10)
        report = json.loads(result.stdout)
        self.assertTrue(report['passed'])
        self.assertEqual(report['threads_configured'], 2)
        self.assertEqual(report['hits_per_thread'], [3, 3])
        self.assertTrue(report['distinct_values'])
        self.assertTrue(report['owned_child_exited'])

    def test_exitkill_terminates_owned_tracee_on_tracer_death(self):
        libc = ctypes.CDLL(None, use_errno=True)
        previous = ctypes.c_int()
        self.assertEqual(libc.prctl(37, ctypes.byref(previous), 0, 0, 0), 0)  # PR_GET_CHILD_SUBREAPER
        self.assertEqual(libc.prctl(36, 1, 0, 0, 0), 0)  # PR_SET_CHILD_SUBREAPER
        process = None
        child = None
        try:
            process = subprocess.Popen([self.binary, '--exitkill-check'], stdout=subprocess.PIPE, text=True)
            self.assertTrue(select.select([process.stdout], [], [], 5)[0], 'tracer did not report child')
            line = process.stdout.readline()
            child = json.loads(line)['owned_child']
            process.kill()
            self.assertEqual(process.wait(timeout=5), -signal.SIGKILL)
            deadline = time.monotonic() + 5
            while True:
                pid, status = os.waitpid(child, os.WNOHANG)
                if pid:
                    break
                if time.monotonic() >= deadline:
                    self.fail('owned tracee did not exit after tracer death')
                time.sleep(.01)
            child = None
            self.assertTrue(os.WIFSIGNALED(status))
            self.assertEqual(os.WTERMSIG(status), signal.SIGKILL)
        finally:
            if process is not None:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=5)
                process.stdout.close()
            if child is not None:
                try:
                    os.kill(child, signal.SIGKILL)
                    os.waitpid(child, 0)
                except (ProcessLookupError, ChildProcessError):
                    pass
            libc.prctl(36, previous.value, 0, 0, 0)


if __name__ == '__main__':
    unittest.main()
