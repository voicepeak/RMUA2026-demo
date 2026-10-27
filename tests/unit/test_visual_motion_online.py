import unittest
from dataclasses import dataclass
from types import SimpleNamespace

from interfaces.contracts import Metadata
from perception.visual_odometry.online import MotionWindow


@dataclass(frozen=True)
class MockMotion:
    previous_stamp: float
    current_stamp: float
    control_eligible: bool = False


def frame(stamp, epoch=0):
    return SimpleNamespace(metadata=Metadata(stamp, stamp + .15, epoch, 'test', 'front_stereo'))


class VisualMotionOnlineTests(unittest.TestCase):
    def test_warmup_and_timely_estimate_do_not_authorize_control(self):
        calls = []
        def estimator(a, b, rig, method):
            calls.append((a.metadata.stamp, b.metadata.stamp)); return MockMotion(a.metadata.stamp, b.metadata.stamp)
        window = MotionWindow(None, estimator=estimator)
        self.assertEqual(window.process(frame(1.), 10., lambda:(1.,0,True), lambda:10.)['status'], 'WARMUP')
        result = window.process(frame(1.1), 10.1, lambda:(1.11,0,True), lambda:10.11)
        self.assertTrue(result['timing_current']); self.assertFalse(result['control_eligible'])
        self.assertEqual(calls, [(1., 1.1)])

    def test_expiry_during_inference_clears_anchor(self):
        clocks = iter([(1.1,0,True), (1.3,0,True)])
        window = MotionWindow(None, estimator=lambda a,b,rig,method:MockMotion(a.metadata.stamp,b.metadata.stamp))
        window.process(frame(1.),10.,lambda:(1.,0,True),lambda:10.)
        result = window.process(frame(1.1),10.1,lambda:next(clocks),lambda:10.11)
        self.assertFalse(result['timing_current']); self.assertEqual(result['timing_rejection'],'SOURCE_LEASE_NOT_CURRENT')
        self.assertEqual(window.process(frame(1.4),10.4,lambda:(1.4,0,True),lambda:10.4)['status'],'WARMUP')

    def test_stale_epoch_and_dead_clock_clear_anchor(self):
        window = MotionWindow(None)
        window.process(frame(1.),10.,lambda:(1.,0,True),lambda:10.)
        result = window.process(frame(1.1),10.1,lambda:(1.1,1,True),lambda:10.1)
        self.assertEqual(result['reason'],'INPUT_OTHER_EPOCH')
        result = window.process(frame(2.,1),11.,lambda:(2.,1,False),lambda:11.)
        self.assertEqual(result['reason'],'INPUT_STALE_IMU_CLOCK')
        self.assertEqual(window.process(frame(2.1,1),11.1,lambda:(2.1,1,True),lambda:11.1)['status'],'WARMUP')


    def test_cli_forwards_settings_path(self):
        from unittest.mock import patch
        from adapters.ros_noetic.visual_motion_observer import main
        argv = ['observer', '--run-dir', '/tmp/unused', '--duration', '2', '--settings', '/tmp/settings']
        with patch('sys.argv', argv), patch('adapters.ros_noetic.visual_motion_observer.observe', return_value=0) as observe:
            self.assertEqual(main(), 0)
        self.assertEqual(observe.call_args.args[3], '/tmp/settings')


if __name__ == '__main__':
    unittest.main()
