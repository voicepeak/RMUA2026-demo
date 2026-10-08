import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from reference_planner import RouteGeometry,ReferencePlanner
from route_height_prior import RouteHeightPrior

class HeightPriorTests(unittest.TestCase):
    def setUp(self):
        pts=[(i*20.,.002*(i*20.)**2,.0005*(i*20.)**2) for i in range(31)]
        self.route=RouteGeometry(pts)
        self.prior=RouteHeightPrior(self.route)
        self.gates=[]
        for i in range(2,15):
            s=self.route.seg_s[i];x,y,_=self.route.point_at(s)
            z=-self.prior.height.center(s)-.05*x+.01*y+.8
            self.gates.append(dict(id=i,s=s,x=x,y=y,z=z,valid=True,trusted=True))

    def test_no_model_without_sufficient_geometry(self):
        self.assertFalse(self.prior.fit(self.gates[:4]))

    def test_calibrates_tilt_and_rejects_false_far_gate(self):
        self.assertTrue(self.prior.fit(self.gates))
        s=self.route.seg_s[17];x,y,_=self.route.point_at(s)
        true=-self.prior.height.center(s)-.05*x+.01*y+.8
        self.assertAlmostEqual(self.prior.center(s),true,delta=.1)
        self.assertFalse(self.prior.consistent(s,true-10.))

    def test_rejects_inconsistent_route_geometry(self):
        for i,g in enumerate(self.gates): g['z']+=5.*(-1)**i
        self.assertFalse(self.prior.fit(self.gates))

    def test_planner_does_not_follow_false_height_at_hilltop(self):
        planner=ReferencePlanner(self.route,snap_gate_to_route=False)
        s=self.gates[-1]['s']+50.
        self.prior.fit(self.gates)
        predicted=self.prior.center(s)
        _,profile=planner.build(self.gates,[],[(s,predicted-10.)],200.,0.)
        self.assertTrue(planner.height_prior.valid)
        self.assertAlmostEqual(profile.center(s),predicted,delta=.2)

    def test_reference_continues_beyond_speed_horizon(self):
        planner=ReferencePlanner(self.route,snap_gate_to_route=False)
        _,profile=planner.build(self.gates,[],[],200.,0.)
        h=planner.trend_horizon
        self.assertLess(profile.dz_ds(h+10.),-.1)

    def measured_guides(self):
        return [dict(s=g['s'],z=g['z'],source='measured_flight_pose') for g in self.gates]

    def test_sparse_gates_use_recorded_poses_with_bounded_horizon(self):
        planner=ReferencePlanner(self.route,snap_gate_to_route=False)
        guides=self.measured_guides()
        _,profile=planner.build(self.gates[:1],guides,[],0.,0.)
        self.assertTrue(planner.height_prior.valid)
        self.assertEqual(planner.height_prior.source,'recorded_poses')
        end=guides[-1]['s']
        self.assertGreater(planner.trend_horizon,end+30.)
        self.assertLessEqual(planner.trend_horizon,end+600.)
        self.assertAlmostEqual(profile.center(end+30.),planner.height_prior.center(end+30.),delta=.2)

    def test_pose_calibration_also_works_without_gates(self):
        planner=ReferencePlanner(self.route)
        planner.build([],self.measured_guides(),[],0.,0.)
        self.assertTrue(planner.height_prior.valid)
        self.assertGreater(planner.trend_horizon,self.gates[-1]['s'])

    def test_unlabelled_references_do_not_calibrate_model(self):
        planner=ReferencePlanner(self.route)
        guides=[dict(s=g['s'],z=g['z']) for g in self.gates]
        planner.build(self.gates[:1],guides,[],0.,0.)
        self.assertFalse(planner.height_prior.valid)
        self.assertIsNone(planner.trend_horizon)

    def test_measured_poses_cannot_mask_bad_gate_fit(self):
        guides=self.measured_guides()
        for i,g in enumerate(self.gates):g['z']+=5.*(-1)**i
        self.assertFalse(self.prior.fit(self.gates))
        self.assertFalse(self.prior.fit_recorded_poses(guides,self.gates))

    def test_sparse_conflicting_gate_rejects_pose_model(self):
        bad=dict(self.gates[0],z=self.gates[0]['z']+4.)
        self.assertFalse(self.prior.fit([bad]))
        self.assertFalse(self.prior.fit_recorded_poses(self.measured_guides(),[bad]))

    def test_gate_calibration_has_priority_and_resets_each_build(self):
        self.assertTrue(self.prior.fit(self.gates))
        shifted=[dict(g,z=g['z']+.5) for g in self.measured_guides()]
        self.assertTrue(self.prior.fit_recorded_poses(shifted,self.gates))
        self.assertEqual(self.prior.source,'gates')
        self.assertFalse(self.prior.fit([]))
        self.assertIsNone(self.prior.error)
        self.assertIsNone(self.prior.source)

    def test_corrupt_measured_guides_are_rejected(self):
        guides=[dict(g,z=g['z']+5.*(-1)**i) for i,g in enumerate(self.measured_guides())]
        self.assertFalse(self.prior.fit([]))
        self.assertFalse(self.prior.fit_recorded_poses(guides,[]))

    def test_calibration_poses_do_not_shift_covered_reference(self):
        references=[dict(s=g['s'],z=g['z'],source='recorded_reference') for g in self.gates]
        measurements=[dict(g,z=g['z']+.4,calibration_only=True) for g in self.measured_guides()]
        planner=ReferencePlanner(self.route,snap_gate_to_route=False)
        _,original=planner.build(self.gates[:1],references,[],0.,0.)
        _,calibrated=planner.build(self.gates[:1],references+measurements,[],0.,0.)
        self.assertTrue(planner.height_prior.valid)
        for g in references:
            self.assertAlmostEqual(calibrated.center(g['s']),original.center(g['s']))

if __name__=='__main__':unittest.main()
