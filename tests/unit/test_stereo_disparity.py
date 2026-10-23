import unittest

import numpy as np

from interfaces.contracts import Metadata
from perception.stereo_depth.pairing import ImageFrame, StereoPairBuffer
from perception.stereo_depth.sgbm import SgbmConfig, compute_diagnostic_disparity


def pair(left, right):
    buffer = StereoPairBuffer()
    for side, image in (('left', left), ('right', right)):
        frame = ImageFrame(Metadata(10., 10.2, 0, 'synthetic', 'front_' + side + '_image'),
                           'mono8', image[:, :, None])
        buffer.add(side, frame, 20.)
    return buffer.pop(10.01, 20.01)


def shifted(disparity=8):
    rng = np.random.RandomState(127)
    left = rng.randint(0, 256, (96, 320)).astype(np.uint8)
    right = rng.randint(0, 256, left.shape).astype(np.uint8)
    right[:, :-disparity] = left[:, disparity:]
    return left, right


class StereoDisparityTests(unittest.TestCase):
    def test_known_pixel_shift_recovered_without_metric_or_rectification_claim(self):
        for shift in (8, 16):
            result = compute_diagnostic_disparity(pair(*shifted(shift)), SgbmConfig(num_disparities=32))
            region = result.disparity_px[10:-10, 48:-48]
            valid = result.valid[10:-10, 48:-48]
            self.assertGreater(valid.mean(), .9)
            self.assertLess(np.sqrt(np.mean((region[valid] - shift) ** 2)), .2)
            self.assertFalse(result.metric_depth_verified)
            self.assertFalse(result.rectification_verified)
            self.assertFalse(result.acceptance_eligible)
            self.assertEqual(result.metadata.valid_until, 10.2)
            with self.assertRaises(ValueError): result.disparity_px.setflags(write=True)

    def test_uniform_and_occluded_regions_stay_unknown(self):
        blank = np.full((96, 320), 127, np.uint8)
        empty = compute_diagnostic_disparity(pair(blank, blank), SgbmConfig(num_disparities=32))
        self.assertFalse(empty.valid.any())
        self.assertTrue(np.isnan(empty.disparity_px).all())
        left, right = shifted()
        right[:, 120:180] = 0
        result = compute_diagnostic_disparity(pair(left, right), SgbmConfig(num_disparities=32))
        self.assertFalse(result.valid[10:-10, 135:175].any())
        self.assertTrue(np.isnan(result.disparity_px[10:-10, 135:175]).all())

    def test_inconsistent_vertical_correspondence_is_rejected(self):
        left, right = shifted()
        right = np.roll(right, 8, axis=0)
        result = compute_diagnostic_disparity(pair(left, right), SgbmConfig(num_disparities=32))
        self.assertLess(result.valid[10:-10, 48:-48].mean(), .05)

    def test_invalid_search_parameters_and_too_small_images_are_rejected(self):
        for kwargs in ({'num_disparities': 17}, {'block_size': 4}, {'consistency_px': 0},
                       {'min_texture_std': 0}, {'num_disparities': 512}):
            with self.assertRaises(ValueError): SgbmConfig(**kwargs)
        small = np.zeros((20, 20), np.uint8)
        with self.assertRaises(ValueError): compute_diagnostic_disparity(pair(small, small))


if __name__ == '__main__':
    unittest.main()
