import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from motion_estimator import MotionEstimator
from speed_scheduler import SpeedScheduler


class MotionTests(unittest.TestCase):
    def test_endpoint_velocity_recovers_acceleration_at_irregular_pose_times(self):
        times=np.array([0.,.009,.021,.035,.049,.060,.077,.089,.102,.119,.133,.150,.169,.18])
        velocity=np.array([4.,-3.,.7]);acceleration=np.array([2.,1.,-4.])
        rows=[(1790896000.+t,np.array([900.,600.,-130.])+velocity*t+.5*acceleration*t*t) for t in times]
        fitted=MotionEstimator.terminal_velocity(rows,rows[-1][0])
        np.testing.assert_allclose(fitted,velocity+acceleration*.18,atol=2e-5)
        # Future observations must have no influence on a frozen cycle.
        future=rows+[(rows[-1][0]+.01,np.array([1e9]*3))]
        np.testing.assert_array_equal(MotionEstimator.terminal_velocity(future,rows[-1][0]),fitted)

    def test_endpoint_velocity_rejects_gap_teleport_and_short_history(self):
        rows=[(t,np.array([t,0.,0.])) for t in np.arange(0.,.181,.01)]
        self.assertIsNone(MotionEstimator.terminal_velocity(rows[-5:],.18))
        missing=[r for r in rows if not .04<r[0]<.12]
        self.assertIsNone(MotionEstimator.terminal_velocity(missing,.18))
        jump=[(t,p+([10.,0.,0.] if t>.1 else 0.)) for t,p in rows]
        self.assertIsNone(MotionEstimator.terminal_velocity(jump,.18))
        self.assertIsNone(MotionEstimator.terminal_velocity(rows,.185))

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
