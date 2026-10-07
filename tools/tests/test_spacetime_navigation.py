from dataclasses import replace
from pathlib import Path
import sys
import time
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from spacetime_scenarios import known_snapshot
from spacetime_navigation import SpaceTimeNavigation,PlannerProcess
from st_lattice import LatticeConfig
from lidar_points import LidarFrame
from local_occupancy import LocalOccupancy


class SpaceTimeNavigationTests(unittest.TestCase):
    def test_frozen_evidence_matches_owner_exact_ttls_and_is_immutable(self):
        mapping=LocalOccupancy();mapping.update([[3.,0.,0.],[4.,1.,0.]],[0.,0.,0.],10.)
        evidence=mapping.evidence()
        for now in (10.,10.49,10.5,11.1):
            a=mapping.snapshot(now);b=evidence.snapshot(now)
            for key in ('states','static_states','dynamic_owners'):
                np.testing.assert_array_equal(getattr(a,key),getattr(b,key))
            self.assertEqual(a.valid,b.valid)
        mapping.update([[1.,0.,0.]],[0.,0.,0.],10.1)
        self.assertEqual(evidence.stamp,10.);self.assertFalse(evidence.free.flags.writeable)

    def test_frame_origin_and_epoch_and_duplicate_are_preserved(self):
        snapshot,parameters=known_snapshot();nav=SpaceTimeNavigation()
        frame=LidarFrame(100.,np.array([[3.,0.,0.]]),np.array([1.,0.,0.]),4)
        self.assertTrue(nav.ingest(frame,snapshot.route,np.array([.1,0.,0.])))
        self.assertFalse(nav.ingest(frame,snapshot.route,np.array([.1,0.,0.])))
        # The executor can obtain the last immutable evidence during a write.
        with nav.lock:
            result=nav.snapshot(snapshot.response_state,snapshot.route)
            self.assertIsNotNone(result);self.assertEqual(result.occupancy.stamp,100.)
        nav.ingest(replace(frame,epoch=5),snapshot.route,np.array([.1,0.,0.]))
        self.assertGreater(nav.epoch,result.epoch)

    def test_unobserved_future_or_expired_map_never_becomes_free(self):
        snapshot,parameters=known_snapshot();nav=SpaceTimeNavigation()
        self.assertIsNone(nav.snapshot(snapshot.response_state,snapshot.route))
        nav.ingest(LidarFrame(100.,np.array([[3.,0.,0.]]),np.zeros(3),0),snapshot.route,np.zeros(3))
        state=replace(snapshot.response_state,stamp=100.6,next_control=np.full(3,100.6),next_physics=100.6)
        self.assertFalse(nav.snapshot(state,snapshot.route).occupancy.valid)

    def worker(self):
        snapshot,parameters=known_snapshot()
        return snapshot,PlannerProcess(parameters,LatticeConfig(budget=.1,forward_distance=2.,height_step=0.,lateral_rate=0.),timeout=.3)

    def ready(self,worker,snapshot,delay=0.):
        end=time.monotonic()+5.
        while not worker.submit(snapshot,delay):
            if time.monotonic()>end:self.fail('Worker failed to start')
            time.sleep(.01)

    def test_spawned_worker_returns_frozen_plan_and_persists_for_next_job(self):
        snapshot,worker=self.worker()
        try:
            self.ready(worker,snapshot);pid=worker.process.pid;end=time.monotonic()+2.
            reply=None
            while reply is None and time.monotonic()<end:reply=worker.poll();time.sleep(.005)
            self.assertIsNotNone(reply);self.assertIsNotNone(reply.result.trajectory)
            self.assertEqual(worker.process.pid,pid);self.assertTrue(worker.submit(snapshot))
        finally:worker.close()

    def test_hung_and_dead_worker_are_reaped_without_blocking_poller(self):
        snapshot,worker=self.worker()
        try:
            self.ready(worker,snapshot,2.2);worker.submitted-=1.
            start=time.monotonic();self.assertIsNone(worker.poll())
            self.assertLess(time.monotonic()-start,.15);self.assertEqual(worker.last_reason,'WORKER_TIMEOUT')
            self.assertIsNone(worker.process)
            self.ready(worker,snapshot,2.2);worker.process.terminate();worker.process.join(timeout=.1)
            self.assertIsNone(worker.poll());self.assertIn(worker.last_reason,('WORKER_EXITED','WORKER_EOF'))
        finally:worker.close()


if __name__=='__main__':unittest.main()
