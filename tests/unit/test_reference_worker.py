from dataclasses import replace
import time
import unittest
from control.reference_worker import BenchmarkSpec,ReferenceWorker,ReferencePacket
from control.safety_supervisor.reference_hold import ReferenceHold
from tests.unit.test_angular_recovery import state
from interfaces.contracts import Health


class ReferenceIsolationTests(unittest.TestCase):
    def test_blocked_process_cannot_refresh_old_output_or_block_control_reader(self):
        worker=ReferenceWorker(BenchmarkSpec((0.,0.,-1.),0.),stall_seconds=.3)
        worker.start()
        try:
            deadline=time.monotonic()+3
            while time.monotonic()<deadline:
                worker.submit(10.,0,0.)
                packet,reason=worker.snapshot(10.,0)
                if packet is not None:break
                time.sleep(.005)
            else:self.fail('reference process did not start')
            deadline=time.monotonic()+1.5
            durations=[]
            while time.monotonic()<deadline and worker.max_compute<.3:
                worker.submit(10.2,0,6.2)
                begin=time.monotonic()
                packet,reason=worker.snapshot(10.5,0)
                durations.append(time.monotonic()-begin)
                self.assertIsNone(packet)
                time.sleep(.005)
            self.assertTrue(worker.stall_observed)
            self.assertGreaterEqual(worker.max_compute,.3)
            self.assertLess(max(durations),.05)
            deadline=time.monotonic()+1.
            while time.monotonic()<deadline:
                worker.submit(10.5,0,6.5)
                packet,reason=worker.snapshot(10.5,0)
                if packet is not None:break
                time.sleep(.005)
            else:self.fail('worker did not recover after bounded stall')
            self.assertEqual(worker.snapshot(10.5,1)[1],'REFERENCE_EPOCH')
        finally:worker.close()
        self.assertFalse(worker.summary()['alive_after_close'])

    def test_recovered_reference_does_not_automatically_resume_after_hold(self):
        packet=ReferencePacket(1.,0,time.monotonic(),(1.,0.,-1.),(0.,0.,0.),(0.,0.,0.),0.,0.,0.,False)
        hold=ReferenceHold()
        self.assertEqual(hold.select(state(),packet).phase,'NORMAL')
        d=hold.select(state(1.1),None,'REFERENCE_STALE')
        self.assertEqual(d.position,(0.,0.,-1.))
        self.assertFalse(d.allow_integral)
        self.assertEqual(hold.select(state(1.2),packet).phase,'REFERENCE_HOLD')
        with self.assertRaises(RuntimeError):hold.select(state(1.3,health=Health.LOST),packet)
        with self.assertRaises(RuntimeError):hold.select(state(1.3,epoch=1),packet)
        with self.assertRaises(RuntimeError):hold.select(replace(state(1.3),position_ned=(1.,0.,-1.)),packet)


if __name__=='__main__':unittest.main()
