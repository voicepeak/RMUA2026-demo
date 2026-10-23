import unittest

import numpy as np

from evaluation.metrics.trt_stereo_evaluate import numerical_agreement


class TrtAgreementTests(unittest.TestCase):
    def test_similar_mask_and_disparity_can_pass_numerical_gate_only(self):
        reference=np.full((20,20),8.,np.float32);mask=np.ones((20,20),bool)
        result=numerical_agreement(reference,mask,reference+.125,mask)
        self.assertTrue(result['compatible']);self.assertEqual(result['absolute_p99_px'],.125)

    def test_bad_disparity_or_lost_coverage_cannot_hide_behind_small_shared_error(self):
        reference=np.full((20,20),8.,np.float32);mask=np.ones((20,20),bool)
        self.assertFalse(numerical_agreement(reference,mask,reference+2.,mask)['compatible'])
        reduced=mask.copy();reduced[:4]=False
        self.assertFalse(numerical_agreement(reference,mask,reference,reduced)['compatible'])

    def test_empty_or_nonfinite_comparison_is_not_zero_error(self):
        reference=np.full((20,20),np.nan,np.float32);empty=np.zeros((20,20),bool)
        result=numerical_agreement(reference,empty,reference,empty)
        self.assertFalse(result['compatible']);self.assertIsNone(result['absolute_p99_px'])


if __name__=='__main__':unittest.main()
