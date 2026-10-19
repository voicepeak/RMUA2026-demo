from types import SimpleNamespace as Obj
import unittest
from adapters.ros_noetic.flight_sensors import FlightSensors
from interfaces.contracts import Health


def message(stamp,position=(1.,2.,-1.),q=(1.,0.,0.,0.)):
    return Obj(header=Obj(stamp=Obj(to_sec=lambda:stamp)),
               pose=Obj(position=Obj(x=position[0],y=position[1],z=position[2]),
                        orientation=Obj(w=q[0],x=q[1],y=q[2],z=q[3])),
               linear_acceleration=Obj(x=0.,y=0.,z=-9.81),
               angular_velocity=Obj(x=0.,y=0.,z=0.))


class SensorAdapterTests(unittest.TestCase):
    def sensors(self):
        s=FlightSensors(subscribe=False)
        s.on_initial(message(0.))
        s.on_imu(message(100.))
        s.on_gps(message(100.))
        return s

    def test_future_gps_waits_for_imu_then_updates(self):
        s=self.sensors()
        s.on_gps(message(100.02))
        self.assertEqual(s.estimator.last_gps,100.)
        self.assertIsNotNone(s.pending_gps)
        s.on_imu(message(100.03))
        self.assertEqual(s.estimator.last_gps,100.02)
        self.assertIsNone(s.pending_gps)
        self.assertEqual(s.read()[0].health,Health.OK)

    def test_invalid_queued_gps_cannot_kill_imu_propagation(self):
        for position in ((0.,0.,0.),(1.,2.,-1.)):
            s=self.sensors()
            s.on_gps(message(100.02,position,(0.,0.,0.,0.)))
            s.on_imu(message(100.03))
            self.assertIsNone(s.fault)
            self.assertEqual(s.estimator.last_gps,100.)
            self.assertTrue(s.estimator.rejections)
            self.assertEqual(s.estimator.stamp,100.03)


if __name__ == '__main__':
    unittest.main()
