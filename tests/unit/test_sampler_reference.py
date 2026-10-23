import json
import unittest

import numpy as np

from evaluation.metrics.stereo_sampler_probe import reference_1d,errors


class SamplerReferenceTests(unittest.TestCase):
    def test_align_corners_interpolates_fractional_and_zero_padded_edges(self):
        image=np.array([[[[2.,4.,8.]]]],np.float32)
        x=np.array([-.5,0.,.5,1.,1.5,2.,2.5],np.float32)
        grid=np.zeros((1,1,len(x),2),np.float32);grid[0,0,:,0]=x-1.
        np.testing.assert_array_equal(reference_1d(image,grid),[[[[1.,2.,3.,4.,6.,8.,4.]]]])

    def test_batches_and_channels_are_not_mixed(self):
        image=np.array([[[[2.,4.,8.]],[[1.,3.,5.]]],[[[10.,20.,30.]],[[-1.,0.,1.]]]],np.float32)
        grid=np.zeros((2,1,1,2),np.float32);grid[:,:,:,0]=-.5
        np.testing.assert_array_equal(reference_1d(image,grid),np.array([3.,2.,15.,-.5]).reshape(2,2,1,1))

    def test_nonfinite_or_two_dimensional_sampling_is_not_a_valid_oracle_input(self):
        image=np.ones((1,1,1,3),np.float32);grid=np.zeros((1,1,1,2),np.float32)
        grid[:,:,:,1]=.2
        with self.assertRaises(ValueError):reference_1d(image,grid)
        grid[:]=np.nan
        with self.assertRaises(ValueError):reference_1d(image,grid)

    def test_nonfinite_result_is_a_serializable_failure_not_nan_statistics(self):
        result=errors(np.array([np.inf]),np.array([0.]))
        self.assertFalse(result['passed']);self.assertIsNone(result['max_abs'])
        json.dumps(result,allow_nan=False)


if __name__=='__main__':unittest.main()
