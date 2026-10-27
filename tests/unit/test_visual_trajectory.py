from types import SimpleNamespace
import unittest
import numpy as np
from perception.visual_odometry.trajectory import VisualTrajectory
from evaluation.pipeline.visual_trajectory import build


def motion(t0, t1, translation, rotation, epoch=0):
    return SimpleNamespace(previous_stamp=t0, current_stamp=t1, translation_previous_body_m=translation,
                           rotation_previous_from_current_body=rotation, epoch=epoch)


class VisualTrajectoryTests(unittest.TestCase):
    def test_composes_body_translation_after_yaw(self):
        chain = VisualTrajectory(1., 0)
        yaw = ((0.,-1.,0.),(1.,0.,0.),(0.,0.,1.))
        chain.append(motion(1., 1.1, (1.,0.,0.), yaw))
        result = chain.append(motion(1.1, 1.2, (1.,0.,0.), np.eye(3)))
        np.testing.assert_allclose(result.position_initial_body_m, (1.,1.,0.))
        self.assertEqual(result.steps, 2)
        self.assertFalse(result.control_eligible)

    def test_gap_or_invalid_rotation_latches_break(self):
        for bad in (motion(1.1,1.2,(0.,0.,0.),np.eye(3)),
                    motion(1.,1.1,(0.,0.,0.),np.diag((1.,1.,-1.)))):
            chain = VisualTrajectory(1., 0)
            with self.assertRaises(ValueError): chain.append(bad)
            with self.assertRaises(ValueError): chain.append(motion(1.,1.1,(0.,0.,0.),np.eye(3)))
            self.assertEqual(chain.snapshot().steps, 0)

    def test_rejected_pair_creates_new_local_segment(self):
        def row(a,b):
            return {'status':'ESTIMATED','previous_stamp':a,'current_stamp':b,
                    'motion':vars(motion(a,b,(1.,0.,0.),np.eye(3)))}
        rejected = {'status':'REJECTED','previous_stamp':1.1,'current_stamp':1.2,'reason':'lost features'}
        result = build({'source':'synthetic','results':[row(1.,1.1), rejected, row(1.2,1.3)]})
        self.assertEqual([len(s['poses']) for s in result['segments']], [1,1])
        self.assertEqual(result['breaks'][0]['reason'], 'lost features')
        self.assertFalse(result['truth_accessed'])

    def test_explicit_failure_ends_chain(self):
        chain = VisualTrajectory(1.,0); chain.invalidate()
        with self.assertRaises(ValueError): chain.append(motion(1.,1.1,(0.,0.,0.),np.eye(3)))


if __name__ == '__main__': unittest.main()
