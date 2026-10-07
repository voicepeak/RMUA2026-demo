from dataclasses import replace
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock,patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from spacetime_scenarios import known_snapshot
from spacetime_runtime import SpaceTimeRuntime
from lidar_points import LidarFrame


class SpaceTimeRuntimeTests(unittest.TestCase):
    def test_unknown_bridge_cannot_publish_motion_and_busy_perception_does_not_block(self):
        snapshot,_=known_snapshot();rows=[];diagnostics=[]
        started=time.monotonic()
        def observe():
            now=time.monotonic();stamp=100.+now-started
            frame=LidarFrame(stamp,np.array([[3.,0.,0.]]),np.zeros(3),0)
            return frame,np.array([.1,0.,0.]),np.zeros(3),stamp,now,0.,stamp
        planner=Mock(last_reason='PLANNING');planner.poll.return_value=None;planner.submit.return_value=False
        with patch('spacetime_runtime.PlannerProcess',return_value=planner):
            runtime=SpaceTimeRuntime(observe,lambda command,*args:rows.append((time.monotonic(),command.copy())),diagnostics.append)
            runtime.set_reference(snapshot.route,0)
            original=runtime.navigation.ingest
            def slow(*args):time.sleep(.2);return original(*args)
            runtime.navigation.ingest=slow
            runtime.start()
            try:time.sleep(.65)
            finally:runtime.close()
        self.assertGreaterEqual(len(rows),10)
        for _,command in rows:np.testing.assert_array_equal(command,np.zeros(3))
        self.assertLess(np.diff([row[0] for row in rows]).max(),.15)
        self.assertTrue(any(row.get('reason')=='BRIDGE_CONTRACT_UNVERIFIED' for row in diagnostics))

    def test_verified_contract_rejects_hold_longer_than_publication_period(self):
        with patch('spacetime_runtime.PlannerProcess'):
            with self.assertRaises(ValueError):
                SpaceTimeRuntime(lambda:None,Mock(),Mock(),bridge_verified=True,bridge_hold=.35)


if __name__=='__main__':unittest.main()
