"""Explicit Docker integration test; uses two inert, UUID-identified sleep processes."""
import os
import subprocess
import unittest
import uuid
from evaluation.seed_runner.controller_cleanup import stop_bound_controller


@unittest.skipUnless(os.environ.get('RMUA_CONTAINER_TESTS')=='1','explicit container tests only')
class OwnedControllerCleanupTests(unittest.TestCase):
    def test_exact_run_path_cannot_terminate_a_neighboring_run(self):
        container=os.environ.get('RMUA_CONTAINER','rmua_noetic')
        module='adapters.ros_noetic.pwm_experiment'
        token='/workspace/main/runs/cleanup_test_'+uuid.uuid4().hex
        paths=(token,token+'_neighbor')
        children=[]
        try:
            for path in paths:
                child=subprocess.Popen(['docker','exec','-u',str(os.getuid())+':'+str(os.getgid()),
                        container,'python3','-c','import os,time;print(os.getpid(),flush=True);time.sleep(60)',
                        module,'--run-dir',path],stdout=subprocess.PIPE,text=True)
                pid=int(child.stdout.readline())
                children.append((child,pid,path))
            stopped=stop_bound_controller(container,module,paths[0])
            self.assertEqual(stopped,[children[0][1]])
            children[0][0].wait(timeout=5)
            self.assertIsNone(children[1][0].poll())
        finally:
            for child,_,path in children:
                stop_bound_controller(container,module,path)
                child.wait(timeout=5)
                child.stdout.close()


if __name__ == '__main__':
    unittest.main()
