import unittest

import numpy as np

from interfaces.contracts import Metadata
from perception.stereo_depth.pairing import ImageFrame
from perception.stereo_depth.learned import rgb_pixels,filter_learned_disparity,full_resolution_disparity


class LearnedStereoChecks(unittest.TestCase):
    def images(self):
        rng=np.random.RandomState(84)
        left=rng.randint(0,256,(40,80)).astype(np.uint8)
        right=np.roll(left,-8,axis=1)
        return left,right

    def test_rgb_conversion_preserves_declared_channel_order(self):
        metadata=Metadata(1.,1.2,0,'test','front_left_image')
        image=ImageFrame(metadata,'bgr8',np.array([[[1,2,3]]],dtype=np.uint8))
        np.testing.assert_array_equal(rgb_pixels(image),[[[3,2,1]]])
        mono=ImageFrame(metadata,'mono8',np.array([[[7]]],dtype=np.uint8))
        np.testing.assert_array_equal(rgb_pixels(mono),[[[7,7,7]]])

    def test_positive_reverse_magnitude_and_pixel_units_match_known_shift(self):
        left,right=self.images();forward=np.full(left.shape,8.,dtype=np.float32)
        filtered,common,lr=filter_learned_disparity(forward,forward,left,right)
        self.assertTrue(common[3:-3,10:-3].all())
        self.assertFalse(lr[:,:8].any())
        self.assertTrue(np.isnan(filtered[~common]).all())
        self.assertTrue((filtered[common]==8).all())

    def test_half_resolution_disparity_is_doubled_in_original_pixel_units(self):
        model_grid=np.array(((4.,8.,12.),(3.,6.,9.)),dtype=np.float32)
        restored=full_resolution_disparity(model_grid,6,4,.5)
        np.testing.assert_array_equal(restored,np.repeat(np.repeat(model_grid*2,2,axis=0),2,axis=1))
        np.testing.assert_array_equal(full_resolution_disparity(model_grid,3,2,1.),model_grid)

    def test_blank_texture_is_not_promoted_by_dense_model_output(self):
        blank=np.zeros((40,80),np.uint8);forward=np.full(blank.shape,8.,dtype=np.float32)
        result,common,lr=filter_learned_disparity(forward,forward,blank,blank)
        self.assertTrue(lr.any());self.assertFalse(common.any());self.assertTrue(np.isnan(result).all())

    def test_inconsistent_or_nonfinite_reverse_fields_stay_unknown(self):
        left,right=self.images();forward=np.full(left.shape,8.,dtype=np.float32)
        reverse=np.full(left.shape,10.,dtype=np.float32)
        self.assertFalse(filter_learned_disparity(forward,reverse,left,right)[1].any())
        reverse[:]=8.;reverse[:,20:40]=np.inf;forward[:,50:55]=np.nan
        with np.errstate(all='raise'):
            result,common,lr=filter_learned_disparity(forward,reverse,left,right)
        self.assertFalse(common[:,28:47].any());self.assertFalse(common[:,50:55].any())

    def test_incompatible_fields_are_rejected(self):
        left,right=self.images()
        with self.assertRaises(ValueError):filter_learned_disparity(np.ones((5,5)),np.ones((5,5)),left,right)

    def test_extreme_finite_predictions_fail_closed_without_overflow(self):
        left,right=self.images();forward=np.full(left.shape,8.,dtype=np.float32)
        reverse=forward.copy();forward[:,30:35]=np.finfo(np.float32).max
        reverse[:,40:45]=-np.finfo(np.float32).max
        with np.errstate(all='raise'):
            result,common,lr=filter_learned_disparity(forward,reverse,left,right)
        self.assertFalse(common[:,30:35].any());self.assertFalse(common[:,48:52].any())
        self.assertTrue(np.isnan(result[~common]).all())


if __name__=='__main__':
    unittest.main()
