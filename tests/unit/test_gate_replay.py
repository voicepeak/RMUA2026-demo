import unittest

from evaluation.pipeline.replay_gate_tracks import interpolate_pose


def row(stamp, position):
    return {'raw':{'stamp':stamp},'command':{'epoch':0},'state':{
        'health':'OK','position_ned':position,'velocity_ned':[1.,0.,0.],
        'orientation_wxyz':[1.,0.,0.,0.],'angular_velocity_body':[0.,0.,0.]}}


class GateReplayTests(unittest.TestCase):
    def test_interpolation_uses_recorded_estimator_and_no_extrapolation(self):
        states=[row(1.,[0.,0.,0.]),row(1.02,[.02,0.,0.])]
        pose,reason=interpolate_pose(states,1.01)
        self.assertIsNone(reason);self.assertAlmostEqual(pose.position_ned[0],.01)
        self.assertEqual(pose.bracket_stamps,(1.,1.02))
        self.assertEqual(interpolate_pose(states,.9)[1],'NO_STATE_BRACKET')
        self.assertEqual(interpolate_pose(states,1.03)[1],'NO_STATE_BRACKET')

    def test_bad_gap_and_estimator_discontinuity_rejected(self):
        self.assertEqual(interpolate_pose([row(1.,[0.,0.,0.]),row(1.1,[.1,0.,0.])],1.05)[1],'BAD_STATE_BRACKET')
        self.assertEqual(interpolate_pose([row(1.,[0.,0.,0.]),row(1.02,[1.,0.,0.])],1.01)[1],'STATE_DISCONTINUITY')


if __name__ == '__main__':unittest.main()
