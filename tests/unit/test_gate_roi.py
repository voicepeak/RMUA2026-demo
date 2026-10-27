import unittest
from dataclasses import replace

import cv2
import numpy as np

from interfaces.contracts import Metadata
from perception.gate_geometry.roi_disparity import gate_region_disparity
from perception.stereo_depth.pairing import ImageFrame, StereoPair
from perception.stereo_depth.sgbm import SgbmConfig, compute_diagnostic_disparity


def pair(left,right):
    frames=[]
    for side,image in (('left',left),('right',right)):
        frames.append(ImageFrame(Metadata(10.,10.2,0,side,'front_'+side+'_image'),'bgr8',image))
    return StereoPair(Metadata(10.,10.2,0,'pair','front_stereo'),frames[0],frames[1],0.)


class GateRoiTests(unittest.TestCase):
    def test_shift_preserved_at_full_pixel_coordinates(self):
        rng=np.random.RandomState(42)
        left=rng.randint(0,256,(240,480,3),dtype=np.uint8)
        cv2.rectangle(left,(200,60),(204,180),(0,0,255),-1)
        cv2.rectangle(left,(330,60),(334,180),(0,0,255),-1)
        right=np.zeros_like(left);right[:,:-12]=left[:,12:]
        p=pair(left,right);config=SgbmConfig(num_disparities=32)
        result=gate_region_disparity(p,config)
        self.assertGreater(int(result.valid.sum()),50)
        self.assertLess(abs(float(np.median(result.disparity_px[result.valid]))-12.),.5)
        self.assertEqual(result.metadata.stamp,p.metadata.stamp)
        self.assertEqual(result.metadata.valid_until,p.metadata.valid_until)
        self.assertFalse(result.valid[10,10]);self.assertTrue(np.isnan(result.disparity_px[10,10]))

    def test_skew_is_rejected_even_without_edges(self):
        image=np.zeros((240,480,3),np.uint8)
        p=pair(image,image)
        right=replace(p.right,metadata=replace(p.right.metadata,stamp=10.01,valid_until=10.21))
        shifted=StereoPair(replace(p.metadata,stamp=10.01),p.left,right,abs(10.01-10.))
        with self.assertRaises(ValueError):gate_region_disparity(shifted)

    def test_no_color_edges_cannot_create_depth(self):
        image=np.full((240,480,3),127,np.uint8); result=gate_region_disparity(pair(image,image))
        self.assertFalse(result.valid.any());self.assertTrue(np.isnan(result.disparity_px).all())
        self.assertFalse(result.acceptance_eligible)


if __name__ == '__main__':unittest.main()
