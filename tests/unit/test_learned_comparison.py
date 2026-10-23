import unittest

import numpy as np

from evaluation.metrics.learned_stereo import compare_reference


class LearnedComparisonTests(unittest.TestCase):
    def test_errors_use_identical_shared_points_without_filtering_large_residuals(self):
        uv=np.array(((3.2,3.2),(6.2,6.2),(10.2,10.2)))
        expected=np.full(3,10.)
        a=np.full((16,16),10.,np.float32);b=np.full((16,16),12.,np.float32)
        b[10:12,10:12]=100.
        va=np.ones((16,16),bool);vb=va.copy();va[10,10]=False;vb[3,3]=False
        a[~va]=np.nan;b[~vb]=np.nan
        result,masks=compare_reference(uv,expected,np.ones(3,bool),{'a':(a,va),'b':(b,vb)})
        np.testing.assert_array_equal(masks['a'],(True,True,False))
        np.testing.assert_array_equal(masks['b'],(False,True,True))
        self.assertEqual(result['reference_candidates'],3)
        self.assertEqual(result['methods']['a']['shared_support']['count'],1)
        self.assertEqual(result['methods']['b']['shared_support']['count'],1)
        self.assertEqual(result['methods']['a']['shared_support']['absolute_median_px'],0.)
        self.assertAlmostEqual(result['methods']['b']['shared_support']['absolute_median_px'],2.)
        self.assertGreater(result['methods']['b']['own_support']['absolute_p90_px'],80.)

    def test_empty_shared_support_is_reported_as_unknown_not_zero_error(self):
        uv=np.array(((3.2,3.2),));expected=np.array((10.,))
        disp=np.full((16,16),10.,np.float32);valid=np.ones((16,16),bool)
        invalid=np.zeros((16,16),bool);unknown=np.full((16,16),np.nan,np.float32)
        result,_=compare_reference(uv,expected,np.ones(1,bool),{'a':(disp,valid),'b':(unknown,invalid)})
        self.assertEqual(result['methods']['a']['own_support']['count'],1)
        self.assertEqual(result['methods']['a']['shared_support']['count'],0)
        self.assertIsNone(result['methods']['a']['shared_support']['absolute_median_px'])


if __name__=='__main__':
    unittest.main()
