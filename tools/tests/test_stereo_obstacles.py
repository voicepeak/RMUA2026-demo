import sys,unittest,json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'ros_ws/src/rmua_gate_vision/scripts'))
sys.path.insert(0,str(ROOT/'ros_ws/src/route_follower/scripts'))
from stereo_obstacles import downward_hits
from obstacle_cloud_fusion import ObstacleCloudFusion
from gate_stereo import FX,BASELINE,camera_to_body
from gate_map import GateMap
from gate_candidates import frame_candidates,gate_frame_support


class StereoObstacleTests(unittest.TestCase):
    def test_below_lidar_plane_is_measured_in_world_coordinates(self):
        disparity=np.zeros((720,960),dtype=np.float32)
        disparity[599:602,479:482]=FX*BASELINE/5.
        points=downward_hits(disparity,[100.,20.,-30.],[0.,0.,0.,1.])
        self.assertEqual(len(points),1)
        expected=camera_to_body(np.array([[0.,240.*5./FX,5.]]))+[100.,20.,-30.]
        np.testing.assert_allclose(points,expected,atol=1e-6)
        # Above the downward blind sector, invalid matches and far returns
        # must not create new obstacles from an unmeasured pixel.
        disparity[360,480]=FX*BASELINE/5.
        disparity[604,484]=np.nan
        disparity[608,488]=FX*BASELINE/40.
        disparity[612,492]=-1.
        self.assertEqual(len(downward_hits(disparity,[0.,0.,0.],[0.,0.,0.,1.])),1)

    def test_world_hits_follow_image_time_attitude(self):
        disparity=np.zeros((720,960),dtype=np.float32);disparity[599:602,479:482]=FX*BASELINE/5.
        z=np.sqrt(.5)
        points=downward_hits(disparity,[0.,0.,0.],[0.,0.,z,z])
        self.assertAlmostEqual(points[0,0],.15,places=5)
        self.assertAlmostEqual(points[0,1],5.175,places=5)

    def test_fusion_retains_raw_returns_caches_and_expires(self):
        fusion=ObstacleCloudFusion(max_age=.6);raw=np.array([[9.,1.,0.]])
        self.assertIs(fusion.combine(raw,1.,1.),raw)
        fusion.update(1.,[[3.,0.,1.]])
        combined=fusion.combine(raw,1.1,1.2)
        np.testing.assert_array_equal(combined,[[9.,1.,0.],[3.,0.,1.]])
        self.assertIs(fusion.combine(raw,1.1,1.3),combined)
        self.assertFalse(fusion.update(.9,[[1.,0.,0.]]))
        self.assertFalse(fusion.update(1.1,np.empty((0,3))))
        self.assertIs(fusion.combine(raw,1.1,1.7),raw)
        self.assertIs(fusion.combine(raw,1.1,.9),raw)
        fusion.reset();self.assertIs(fusion.combine(raw,1.1,1.2),raw)

    def test_stereo_cannot_replace_an_empty_lidar_frame(self):
        fusion=ObstacleCloudFusion();fusion.update(1.,[[1.,0.,1.]])
        raw=np.empty((0,3))
        self.assertIs(fusion.combine(raw,1.,1.1),raw)
        with self.assertRaises(ValueError):fusion.update(2.,[[np.nan,0.,0.]])

    def test_gate_world_map_rejects_another_road(self):
        mapping=GateMap(np.array([[0.,0.,0.],[100.,0.,0.]]))
        mapping.observe([10.,.5,-3.],1.);mapping.observe([10.,10.,-3.],1.)
        gates=json.loads(mapping.to_json(min_support=1,max_d_cross=5.))
        self.assertEqual(len(gates),1);self.assertAlmostEqual(gates[0]['y'],.5)

    def test_measured_gate_normal_keeps_tilt_and_follows_route_direction(self):
        mapping=GateMap(np.array([[0.,0.,0.],[100.,0.,0.]]))
        geometry=dict(width=3.,height=3.,plane_rmse=.05,geometry_valid=True,
                      normal_world=[-1.,0.,-.1],position_sigma=.3)
        mapping.observe([10.,.5,-3.],1.,geom=geometry)
        gate=mapping.gates(min_support=1)[0]
        self.assertGreater(gate['nx'],0.)
        self.assertGreater(gate['nz'],0.)
        self.assertEqual(gate['geometry_uncertainty'],.3)

    def test_white_buildings_cannot_become_colored_gate_proposals(self):
        image=np.zeros((720,960,3),dtype=np.uint8);image[100:400,200:500]=255
        self.assertEqual(frame_candidates(image,[]),[])

    def test_car_roof_does_not_confirm_a_gate_but_vertical_trim_does(self):
        image=np.zeros((200,200,3),dtype=np.uint8)
        image[20:50,20:180]=[30,85,230]
        self.assertFalse(gate_frame_support(image,(20,20,160,160)))
        image[20:180,20:25]=[30,85,230]
        self.assertTrue(gate_frame_support(image,(20,20,160,160)))
