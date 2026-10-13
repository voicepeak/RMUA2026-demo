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
from gate_candidates import frame_candidates,gate_frame_support,gate_opening_support,passable_gate_geometry,dense_candidates


class StereoObstacleTests(unittest.TestCase):
    def test_supported_color_frame_survives_bbox_only_cold_start(self):
        import cv2
        from gate_stereo import gate_center_from_frame
        from gate_detector_opencv import gate_frame_mask
        image=np.zeros((240,240,3),dtype=np.uint8)
        cv2.rectangle(image,(40,40),(200,200),(255,255,255),12)
        cv2.rectangle(image,(48,48),(192,192),(30,85,230),3)
        candidates=dense_candidates(frame_candidates(image,[]))
        self.assertEqual(len(candidates),1)
        self.assertFalse(candidates[0]['keypoints'])
        disparity=np.full(image.shape[:2],FX*BASELINE/8.,dtype=np.float32)
        center,depth,_=gate_center_from_frame(candidates[0],disparity,gate_frame_mask(image))
        self.assertAlmostEqual(depth,8.,places=5)
        gate_map=GateMap()
        for stamp in (1.,1.1,1.2,1.3):gate_map.observe(camera_to_body(center),stamp,depth=depth)
        gates=gate_map.gates(2)
        self.assertEqual(len(gates),1)
        self.assertFalse(gates[0]['geometry_valid'])

    def test_shaded_red_frame_is_valid_but_red_panel_has_no_opening(self):
        image=np.zeros((200,200,3),dtype=np.uint8)
        image[20:180,20:35]=150;image[20:180,165:180]=150
        image[20:180,35:40]=[12,25,110];image[20:180,160:165]=[12,25,110]
        self.assertTrue(gate_opening_support(image,(20,20,160,160)))
        panel=np.zeros_like(image);panel[20:180,20:180]=[12,25,110]
        self.assertFalse(gate_opening_support(panel,(20,20,160,160)))

    def test_frame_depth_ignores_white_background_and_requires_both_posts(self):
        from gate_detector_opencv import gate_trim_mask
        from gate_stereo import gate_center_from_frame
        image=np.zeros((240,240,3),dtype=np.uint8)
        image[40:200,40:46]=[12,25,110];image[40:200,194:200]=[12,25,110]
        trim=gate_trim_mask(image)
        disparity=np.full(image.shape[:2],FX*BASELINE/30.,dtype=np.float32)
        disparity[trim]=FX*BASELINE/8.
        cand=dict(bbox=(40,40,160,160),center=(120,120))
        result=gate_center_from_frame(cand,disparity,np.ones_like(trim),trim_mask=trim)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[1],8.,places=5)
        # A single nearer post cannot borrow a distant white wall as its pair.
        trim[:,150:]=False
        self.assertIsNone(gate_center_from_frame(cand,disparity,np.ones_like(trim),trim_mask=trim))

    def test_inner_detector_rectangle_can_find_surrounding_complete_posts(self):
        image=np.zeros((200,200,3),dtype=np.uint8)
        image[20:180,20:35]=150;image[20:180,165:180]=150
        image[20:180,35:40]=[12,25,110];image[20:180,160:165]=[12,25,110]
        inner=dict(bbox=(35,20,130,160),center=np.array([100.,100.]),conf=.9,keypoints=False)
        self.assertFalse(gate_opening_support(image,inner['bbox']))
        candidates=frame_candidates(image,[inner])
        self.assertEqual(len(candidates),1)
        self.assertTrue(gate_opening_support(image,candidates[0]['bbox']))

    def test_recessed_posts_can_supply_depth_without_a_single_plane(self):
        from gate_stereo import gate_center_from_frame
        trim=np.zeros((240,240),dtype=bool);trim[40:200,40:46]=True;trim[40:200,194:200]=True
        depths=np.full(trim.shape,30.,dtype=float)
        rng=np.random.default_rng(1)
        depths[trim]=rng.uniform(8.,10.,np.count_nonzero(trim))
        disparity=FX*BASELINE/depths
        result=gate_center_from_frame(dict(bbox=(40,40,160,160),center=(120,120)),
            disparity,np.ones_like(trim),trim_mask=trim)
        self.assertIsNotNone(result)
        self.assertGreaterEqual(result[1],8.)
        self.assertLessEqual(result[1],10.)

    def test_near_post_component_is_not_outvoted_by_nested_far_gates(self):
        from gate_stereo import gate_center_from_frame
        trim=np.zeros((240,240),dtype=bool);trim[40:200,40:46]=True;trim[40:200,194:200]=True
        depths=np.full(trim.shape,30.,dtype=float)
        depths[40:200,40]=8.;depths[40:200,199]=8.
        result=gate_center_from_frame(dict(bbox=(40,40,160,160),center=(120,120)),
            FX*BASELINE/depths,np.ones_like(trim),trim_mask=trim)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[1],8.,places=5)

    def test_cap_range_and_complete_opposite_post_share_vertical_center(self):
        from gate_stereo import gate_center_from_frame,FY,CY
        trim=np.zeros((240,240),dtype=bool)
        trim[40:43,40:46]=True;trim[40:200,194:200]=True
        disparity=np.full(trim.shape,FX*BASELINE/8.)
        cand=dict(bbox=(40,40,160,160))
        result=gate_center_from_frame(cand,disparity,np.ones_like(trim),trim_mask=trim)
        self.assertIsNotNone(result)
        self.assertAlmostEqual(result[0][1],(119.5-CY)*8./FY,places=4)
        trim[43:,:]=False
        self.assertIsNone(gate_center_from_frame(cand,disparity,np.ones_like(trim),trim_mask=trim))

    def test_blue_tinted_white_posts_keep_complete_frame_support(self):
        image=np.zeros((200,200,3),dtype=np.uint8)
        image[20:180,20:35]=[150,120,100];image[20:180,165:180]=[150,120,100]
        image[20:180,35:40]=[12,25,110];image[20:180,160:165]=[12,25,110]
        self.assertTrue(gate_opening_support(image,(20,20,160,160)))

    def test_half_resolution_projection_keeps_metric_depth_and_world_frame(self):
        disparity=np.zeros((360,480),dtype=np.float32)
        disparity[299:302,239:242]=.5*FX*BASELINE/5.
        points=downward_hits(disparity,[100.,20.,-30.],[0.,0.,0.,1.],stride=2,scale=.5)
        self.assertEqual(len(points),1)
        expected=camera_to_body(np.array([[0.,240.*5./FX,5.]]))+[100.,20.,-30.]
        np.testing.assert_allclose(points,expected,atol=1e-6)

    def test_gate_requires_both_white_posts_and_a_passable_physical_opening(self):
        image=np.zeros((200,200,3),dtype=np.uint8)
        image[20:180,20:35]=255;image[20:180,35:40]=[30,85,230]
        self.assertFalse(gate_opening_support(image,(20,20,160,160)))
        image[20:180,165:180]=255;image[20:180,160:165]=[30,85,230]
        self.assertTrue(gate_opening_support(image,(20,20,160,160)))
        panel=np.zeros_like(image);panel[20:180,20:180]=[30,85,230]
        self.assertTrue(gate_frame_support(panel,(20,20,160,160)))
        self.assertFalse(gate_opening_support(panel,(20,20,160,160)))
        self.assertFalse(passable_gate_geometry(dict(width=.8,height=.55,position_sigma=.27)))
        self.assertTrue(passable_gate_geometry(dict(width=3.,height=3.,position_sigma=.3)))

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
