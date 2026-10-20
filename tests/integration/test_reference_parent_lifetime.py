"""Own-process cleanup check; no ROS master, simulator or external processes."""
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time
import unittest


@unittest.skipUnless(sys.platform.startswith('linux'),'Linux parent-death signal contract')
class ReferenceLifetimeTests(unittest.TestCase):
    def test_worker_does_not_outlive_a_terminated_flight_parent(self):
        script='''
import time
from control.reference_worker import BenchmarkSpec,ReferenceWorker
w=ReferenceWorker(BenchmarkSpec((0.,0.,-1.),0.))
w.start()
deadline=time.monotonic()+3
while time.monotonic()<deadline:
    w.submit(10.,0,0.)
    if w.snapshot(10.,0)[0] is not None:break
    time.sleep(.005)
else:raise RuntimeError('worker not ready')
print(w.process.pid,flush=True)
while True:time.sleep(.1)
'''
        root=Path(__file__).resolve().parents[2]
        parent=subprocess.Popen([sys.executable,'-c',script],cwd=root,stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,text=True)
        worker_pid=None
        def running(pid):
            path=Path('/proc')/str(pid)/'stat'
            try:return path.read_text().split(') ',1)[1].split()[0] not in ('Z','X')
            except FileNotFoundError:return False
        try:
            with selectors.DefaultSelector() as poll:
                poll.register(parent.stdout,selectors.EVENT_READ)
                self.assertTrue(poll.select(timeout=5.),'flight parent never became ready')
                line=parent.stdout.readline().strip()
            self.assertTrue(line.isdigit(),line)
            worker_pid=int(line)
            self.assertTrue(running(worker_pid))
            parent.terminate();parent.wait(timeout=2.)
            deadline=time.monotonic()+1.
            while running(worker_pid) and time.monotonic()<deadline:time.sleep(.01)
            self.assertFalse(running(worker_pid),'orphaned reference worker remains active')
        finally:
            if parent.poll() is None:parent.terminate();parent.wait(timeout=2.)
            if worker_pid and running(worker_pid):os.kill(worker_pid,signal.SIGTERM)
            parent.stdout.close();parent.stderr.close()


if __name__=='__main__':unittest.main()
