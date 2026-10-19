from dataclasses import replace
import threading
import unittest
from control.control_authority.graph_monitor import GraphMonitor
from control.safety_supervisor.hover_recovery import HoverRecovery
from interfaces.contracts import Health,Metadata,StateEstimate


class SupervisionTests(unittest.TestCase):
    def test_blocking_graph_rpc_cannot_block_freshness_check(self):
        now=[1.]
        entered,release=threading.Event(),threading.Event()
        calls=[0]
        def probe():
            calls[0]+=1
            if calls[0]>1:
                entered.set();release.wait(2.)
        monitor=GraphMonitor(probe,interval=.001,clock=lambda:now[0])
        monitor.start()
        try:
            self.assertTrue(entered.wait(1.))
            now[0]=1.4
            with self.assertRaisesRegex(RuntimeError,'ROS_GRAPH_STALE'):
                monitor.check()
        finally:
            release.set();monitor.close()

    def test_graph_conflict_is_latched_after_transient_failure(self):
        calls=[0]
        def probe():
            calls[0]+=1
            if calls[0]==2:raise RuntimeError('duplicate publisher')
        m=GraphMonitor(probe)
        m._once();m._once();m._once()
        with self.assertRaisesRegex(RuntimeError,'duplicate publisher'):m.check()

    def test_inflight_query_during_shutdown_does_not_create_a_fake_conflict(self):
        entered,release=threading.Event(),threading.Event()
        calls=[0]
        def probe():
            calls[0]+=1
            if calls[0]>1:
                entered.set();release.wait(2.)
                raise RuntimeError('publisher unregistered during shutdown')
        m=GraphMonitor(probe,interval=.001)
        m.start()
        try:
            self.assertTrue(entered.wait(1.))
            m.close()
        finally:
            release.set();m.thread.join(1.)
        self.assertIsNone(m.summary()['fault'])

    def test_gps_bridge_freezes_reference_and_integral_then_resumes(self):
        s=StateEstimate(Metadata(10.,11.,0,'test'),(1.,2.,-1.),(0.,0.,0.),
                        (1.,0.,0.,0.),(0.,0.,0.),Health.DEGRADED)
        nominal=((2.,2.,-1.),(.3,0.,0.),(0.,0.,0.))
        r=HoverRecovery()
        d=r.decide(s,.4,nominal)
        self.assertEqual(d.position,s.position_ned)
        self.assertEqual(d.velocity,(0.,0.,0.))
        self.assertFalse(d.allow_integral)
        d=r.decide(replace(s,position_ned=(1.1,2.,-1.)),.8,nominal)
        self.assertEqual(d.position,(1.,2.,-1.))
        with self.assertRaises(RuntimeError):r.decide(s,1.1,nominal)
        with self.assertRaises(RuntimeError):r.decide(replace(s,health=Health.OK),1.1,nominal)
        d=r.decide(replace(s,health=Health.OK),.01,nominal)
        self.assertEqual(d.phase,'NORMAL')
        self.assertEqual(d.position,nominal[0])
        self.assertEqual(len(r.transitions),2)


if __name__ == '__main__':
    unittest.main()
