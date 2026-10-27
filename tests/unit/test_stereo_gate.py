import unittest
from dataclasses import replace
from pathlib import Path

import cv2
import numpy as np

from interfaces.contracts import Metadata
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.pairing import ImageFrame, StereoPair
from perception.stereo_depth.sgbm import DisparityObservation
from perception.gate_geometry.stereo_gate import stereo_gate_candidates


class StereoGateTests(unittest.TestCase):
    def setUp(self):
        self.rig = load_nominal_rig(Path(__file__).resolve().parents[2] / 'configs/simulator_settings.json')
        self.image = np.zeros((720,960,3),np.uint8)
        cv2.rectangle(self.image,(350,250),(355,450),(0,0,255),-1)
        cv2.rectangle(self.image,(600,250),(605,450),(0,0,255),-1)
        l = ImageFrame(Metadata(10.,10.2,0,'left','front_left_image'),'bgr8',self.image)
        r = ImageFrame(Metadata(10.,10.2,0,'right','front_right_image'),'bgr8',self.image)
        self.pair = StereoPair(Metadata(10.,10.2,0,'pair','front_stereo'),l,r,0.)
        valid = np.ones((720,960),dtype=bool); depth = np.full((720,960),12.,dtype=np.float32)
        self.disparity = DisparityObservation(Metadata(10.,10.2,0,'depth','front_left_image'),depth,valid,10.,10.)

    def test_valid_nominal_pair_and_no_control_eligibility(self):
        out=stereo_gate_candidates(self.pair,self.disparity,self.rig,10.05,0)
        self.assertEqual(len(out),1)
        self.assertGreater(out[0].center_body_frd_m[0],20.)
        self.assertFalse(out[0].control_eligible);self.assertFalse(out[0].aperture_verified)
        self.assertEqual(out[0].metadata.valid_until,10.2)

    def test_empty_depth_and_border_cropped_frame_are_not_promoted(self):
        empty=replace(self.disparity,disparity_px=np.full((720,960),np.nan,np.float32),valid=np.zeros((720,960),bool))
        self.assertEqual(stereo_gate_candidates(self.pair,empty,self.rig,10.,0),())
        image=self.image.copy();image[:, :]=0;cv2.rectangle(image,(0,250),(5,450),(0,0,255),-1);cv2.rectangle(image,(250,250),(255,450),(0,0,255),-1)
        self.assertEqual(stereo_gate_candidates(replace(self.pair,left=replace(self.pair.left,pixels=image)),self.disparity,self.rig,10.,0),())

    def test_depth_inconsistent_edges_and_plain_scene_do_not_produce_candidate(self):
        values = np.full((720,960), 12., dtype=np.float32)
        values[:, 590:620] = 24.
        mismatch = replace(self.disparity, disparity_px=values)
        self.assertEqual(stereo_gate_candidates(self.pair,mismatch,self.rig,10.05,0),())
        blank = np.zeros_like(self.image)
        blank_pair = replace(self.pair,left=replace(self.pair.left,pixels=blank))
        self.assertEqual(stereo_gate_candidates(blank_pair,self.disparity,self.rig,10.05,0),())

    def test_stale_epoch_and_cross_pair_disparity_are_rejected(self):
        for now,epoch in ((10.2,0),(10.05,1)):
            with self.assertRaises(ValueError):stereo_gate_candidates(self.pair,self.disparity,self.rig,now,epoch)
        with self.assertRaises(ValueError):stereo_gate_candidates(self.pair,replace(self.disparity,left_measurement_stamp=9.9),self.rig,10.05,0)


if __name__ == '__main__': unittest.main()
