from copy import deepcopy
from types import SimpleNamespace as Obj
import unittest
import numpy as np
from adapters.ros_noetic.flight_sensors import FlightSensors
from estimation.stationary_bootstrap import resting_window
from tests.unit.test_flight_sensors import message


class BootstrapTests(unittest.TestCase):
    def window(self):
        imu=[{'stamp':100+i*.01,'acceleration_body':[0,0,-9.81],
              'omega_body':[.001,-.002,.001],'imu_orientation_wxyz':[1,0,0,0]} for i in range(111)]
        gps=[(100+i*.1,(.01,-.02,.31),(1,0,0,0)) for i in range(12)]
        return imu,gps

    def test_motion_gap_zero_gps_and_missing_orientation_cannot_initialize(self):
        imu,gps=self.window()
        self.assertIsNotNone(resting_window(imu,gps,(1,0,0,0),101.1))
        changed=deepcopy(imu);changed[90]['omega_body']=[.1,0,0]
        self.assertIsNone(resting_window(changed,gps,(1,0,0,0),101.1))
        self.assertIsNone(resting_window(imu[:50]+imu[60:],gps,(1,0,0,0),101.1))
        moved=[(t,(i*.1,0,.31),q) for i,(t,p,q) in enumerate(gps)]
        self.assertIsNone(resting_window(imu,moved,(1,0,0,0),101.1))
        changed=deepcopy(imu);changed[-1].pop('imu_orientation_wxyz')
        self.assertIsNone(resting_window(changed,gps,(1,0,0,0),101.1))
        self.assertIsNone(resting_window(imu,[],(1,0,0,0),101.1))
        self.assertIsNone(resting_window(imu,[(t,(0,0,0),q) for t,p,q in gps],(1,0,0,0),101.1))

    def test_ground_offset_does_not_become_launch_velocity_after_rest_reset(self):
        s=FlightSensors(subscribe=False)
        s.on_initial(message(0.,position=(0,0,0)))
        for i in range(121):
            msg=message(100+i*.01)
            msg.orientation=Obj(w=1.,x=0.,y=0.,z=0.)
            s.on_imu(msg)
            if i%10==0:s.on_gps(message(100+i*.01,position=(.01,-.02,.31)))
        stamp,epoch=s.estimator.stamp,s.clock.epoch
        result=s.bootstrap_at_rest_before_control()
        self.assertIsNotNone(result)
        np.testing.assert_allclose(s.estimator.v,0)
        np.testing.assert_allclose(s.estimator.p,(.01,-.02,.31))
        self.assertEqual(s.estimator.stamp,stamp)
        self.assertEqual(s.clock.epoch,epoch)
        self.assertGreater(np.linalg.eigvalsh(s.estimator.P).min(),0)
        with self.assertRaises(RuntimeError):s.bootstrap_at_rest_before_control()


if __name__=='__main__':unittest.main()
