import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from motion_estimator import MotionEstimator
from speed_scheduler import SpeedScheduler


class MotionTests(unittest.TestCase):
    def test_sideways_avoidance_does_not_create_vertical_feed_forward(self):
        estimator=MotionEstimator(time_constant=0.)
        progress,up=estimator.update((0.,0.,0.),(0.,.5,-.1),.05,(1.,0.,0.))
        self.assertEqual(progress,0.)
        self.assertAlmostEqual(up,2.)

    def test_measured_motion_remains_consistent_when_command_changes(self):
        estimator=MotionEstimator()
        position=np.zeros(3)
        for _ in range(60):
            previous=position.copy();position+=np.array([.5,0.,.1])
            progress,up=estimator.update(previous,position,.05,(1.,0.,0.))
        self.assertAlmostEqual(progress,10.,places=5)
        self.assertAlmostEqual(up,-2.,places=5)
        estimator.reset()
        self.assertTrue(np.all(estimator.velocity==0.))

    def test_tracking_error_is_soft_and_does_not_instantly_brake(self):
        scheduler=SpeedScheduler(cruise_speed=18.,max_speed=40.,jerk_limit=20.)
        target,info=scheduler.target(0.,0.,True,None,0.,False,False,0.,
                                     v_tracking_cap=8.)
        self.assertEqual(target,8.)
        self.assertEqual(info['hard_cap'],18.)
        speed=scheduler.step(12.,target,.05,hard_cap=info['hard_cap'])
        self.assertGreaterEqual(speed,12.-scheduler.a_down*.05)

    def test_ordinary_acceleration_changes_are_jerk_limited(self):
        scheduler=SpeedScheduler(max_speed=40.,jerk_limit=20.)
        speed=previous_acceleration=0.
        for target in [10.]*150+[5.]*150:
            new=scheduler.step(speed,target,.05,hard_cap=40.)
            acceleration=(new-speed)/.05
            self.assertLessEqual(abs(acceleration-previous_acceleration),20.*.05+1e-7)
            previous_acceleration=acceleration;speed=new
        self.assertAlmostEqual(speed,5.,delta=.03)

    def test_emergency_stop_overrides_ordinary_smoothing(self):
        scheduler=SpeedScheduler(max_speed=40.,jerk_limit=20.)
        self.assertEqual(scheduler.step(15.,15.,.05,hard_cap=0.),0.)
