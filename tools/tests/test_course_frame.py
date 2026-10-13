import sys,unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from course_frame import mapped_course_points
from reference_planner import RouteGeometry,ReferencePlanner


class CourseFrameTests(unittest.TestCase):
    def test_vertical_tilt_has_horizontal_effect_at_high_elevation(self):
        raw=np.array([[0.,0.,0.],[700.,0.,100.]])
        mapped=mapped_course_points(raw,[-.05,0.])
        self.assertAlmostEqual(mapped[1,0],700.*np.cos(.05)-100.*np.sin(.05))
        self.assertAlmostEqual(mapped[1,1],0.)
        np.testing.assert_array_equal(mapped[:,2],raw[:,2])
        np.testing.assert_array_equal(raw,[[0.,0.,0.],[700.,0.,100.]])

    def test_projection_and_tangent_use_mapped_xy_with_raw_station_labels(self):
        route=RouteGeometry([[0.,0.,0.],[100.,0.,20.],[180.,80.,40.]])
        raw=route.raw_point_at(80.)
        self.assertTrue(route.calibrate_frame([-.05,0.]))
        point=route.point_at(80.)
        self.assertLess(point[0],raw[0]-.8)
        i,t,d,_=route.project(point)
        self.assertAlmostEqual(route.seg_s[i]+t*route.seg_len[i],80.)
        self.assertAlmostEqual(d,0.)
        self.assertAlmostEqual(route.project_gate(point[0],point[1]),80.)
        before=np.array(route.frame_points)
        self.assertFalse(route.calibrate_frame([-.06,.01]))
        np.testing.assert_array_equal(route.frame_points,before)

    def test_no_frame_without_observable_height_calibration(self):
        route=RouteGeometry([[0.,0.,0.],[100.,0.,20.]])
        planner=ReferencePlanner(route,frame_calibration=True)
        planner.build([],[],[],0.,-2.)
        self.assertIsNone(route.frame_tilt)

    def test_nonfinite_and_unbounded_tilts_are_rejected(self):
        for tilt in ([float('nan'),0.],[.3,0.],[0.]):
            with self.assertRaises(ValueError):mapped_course_points([[0.,0.,0.]],tilt)

    def test_validated_current_run_measurements_enable_frame(self):
        points=[(i*20.,.002*(i*20.)**2,.0005*(i*20.)**2) for i in range(31)]
        route=RouteGeometry(points)
        planner=ReferencePlanner(route,frame_calibration=True,snap_gate_to_route=False)
        gates=[]
        for i in range(2,15):
            s=route.seg_s[i];x,y,_=route.raw_point_at(s)
            gates.append(dict(id=i,x=x,y=y,z=-planner.height_prior.height.center(s)-.05*x+.01*y+.8,
                              valid=True,trusted=True))
        planner.build(gates,[],[],0.,0.)
        self.assertTrue(planner.height_prior.valid)
        self.assertIsNotNone(route.frame_tilt)
        self.assertAlmostEqual(route.frame_tilt[0],-.05,delta=.005)

    def test_later_depth_error_cannot_shear_the_established_frame(self):
        from course_frame import course_rotation
        from route_height_prior import RouteHeightPrior
        points=[(i*20.,.002*(i*20.)**2,.0005*(i*20.)**2) for i in range(31)]
        route=RouteGeometry(points);prior=RouteHeightPrior(route)
        tilt=[-.05,.01];rotation=course_rotation(tilt);gates=[]
        for i in range(2,15):
            s=route.seg_s[i];x,y,_=route.raw_point_at(s)
            z=float(rotation[2]@np.array([x,y,-prior.height.center(s)]))+.8
            gates.append(dict(s=s,x=x,y=y,z=z,trusted=True))
        gates[-1]['z']+=.9
        self.assertTrue(prior.fit(gates))
        self.assertTrue(prior.constrain_to_frame(tilt,gates))
        future=route.seg_s[24];x,y,_=route.raw_point_at(future)
        expected=float(rotation[2]@np.array([x,y,-prior.height.center(future)]))+.8
        self.assertAlmostEqual(prior.center(future),expected,delta=.02)
        self.assertEqual(prior.source,'rigid_gates')

    def test_inconsistent_new_heights_cannot_hide_behind_fixed_frame(self):
        from route_height_prior import RouteHeightPrior
        route=RouteGeometry([(i*20.,.002*(i*20.)**2,.0005*(i*20.)**2) for i in range(31)])
        prior=RouteHeightPrior(route);gates=[]
        for i in range(2,15):
            s=route.seg_s[i];x,y,_=route.raw_point_at(s)
            gates.append(dict(s=s,x=x,y=y,z=-prior.height.center(s)-.05*x+.01*y+.8,trusted=True))
        self.assertTrue(prior.fit(gates))
        for i,g in enumerate(gates):g['z']+=3.*(-1)**i
        self.assertFalse(prior.constrain_to_frame([-.05,.01],gates))
        self.assertFalse(prior.valid)


if __name__=='__main__':unittest.main()
