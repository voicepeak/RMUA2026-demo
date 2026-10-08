import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from velocity_response import VelocityResponse
from lidar_scene import LidarScene
from point_index import PointIndex
from execution_guard import ExecutionGuard
from lidar_navigation import LidarNavigator,GeometryProcess
from response_native import NativeResponse


class ResponseTests(unittest.TestCase):
    def test_stationary_rejected_height_input_clears_for_both_response_policies(self):
        points=np.array([[x,.8,z] for x in np.arange(-2.,2.01,.2)
                         for z in np.arange(-2.,2.01,.1)])
        for limited in (False,True):
            with self.subTest(coupling_limited=limited):
                guard=ExecutionGuard()
                model=VelocityResponse(coupling_limited=limited)
                guard.response_model=model
                model.commit(np.array([0.,0.,-.001]),np.zeros(3),.95)
                model.stop_profile=LidarNavigator._stop_profile(np.array([[0.,0.,-.02],[10.,0.,-.02]]))
                command,info=guard.filter_command(np.zeros(3),np.zeros(3),
                    np.array([0.,0.,-.02]),points,1.,1.05)
                np.testing.assert_array_equal(command,np.zeros(3))
                self.assertEqual(info['command_reason'],'COMMAND_BLOCKED')
                self.assertTrue(info['blocked_neutral_settle'])
                self.assertLess(info['command_clearance'],1.25)
                # Conditional frozen replay: unchanged measured rest next
                # cycle, with the command actually issued in memory.
                model.commit(command,np.zeros(3),1.05)
                escape=guard.escape(np.zeros(3),np.zeros(3),np.array([0.,-1.,0.]),points,1.1,1.15)
                self.assertIsNotNone(escape)
                self.assertLess(escape[1],0.)

    def test_neutral_settle_still_requires_road_and_floor_constraint(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse(coupling_limited=False)
        guard.command_constraint=lambda samples:False
        _,info=guard.filter_command(np.zeros(3),np.zeros(3),np.array([0.,0.,-.02]),
                                  np.array([[0.,.8,0.]]),1.,1.05)
        self.assertEqual(info['command_reason'],'COMMAND_BLOCKED')
        self.assertFalse(info.get('blocked_neutral_settle',False))

    def test_lower_coupling_scenario_native_matches_python_and_extends_downward(self):
        parameters=dict(lift_gain=.095,coupling_gains=(.075,.11,.13),height_gain=2.,
                        coupling_limited=True,xy_error_max=4.5,control_periods=(.08,.16,.4))
        python=VelocityResponse(native=False,**parameters)
        native=VelocityResponse(**parameters)
        if native.native.library is None:self.skipTest('Native library unavailable')
        path=np.array([[0.,0.,0.],[10.,2.,-.5],[25.,5.,-2.]])
        velocity=np.array([10.,1.,0.]);command=np.array([6.,0.,1.])
        for model in (python,native):
            model.stop_profile=LidarNavigator._stop_profile(path,gain=2.)
            model.commit(velocity,velocity,1.)
        expected=python.envelope(np.zeros(3),velocity,command,.3)
        actual=native.envelope(np.zeros(3),velocity,command,.3)
        np.testing.assert_allclose(actual[0],expected[0],rtol=0.,atol=1e-8)
        np.testing.assert_allclose(actual[2],expected[2],rtol=0.,atol=1e-8)
        self.assertAlmostEqual(actual[1],expected[1],places=8)
        self.assertEqual(len(native.scenarios),9)
        narrow=VelocityResponse(**dict(parameters,coupling_gains=(.09,.11,.13)))
        narrow.stop_profile=LidarNavigator._stop_profile(path,gain=2.)
        narrow.commit(velocity,velocity,1.)
        self.assertGreater(actual[0][:,2].max(),narrow.envelope(np.zeros(3),velocity,command,.3)[0][:,2].max())

    def test_intermediate_speed_certifies_faster_stop_before_a_wall(self):
        position=np.zeros(3);velocity=np.array([10.5,0.,0.])
        guard=ExecutionGuard(reaction=.25)
        guard.response_model=VelocityResponse(lift_gain=.11,height_gain=2.,
            coupling_limited=True,xy_error_max=4.5,control_periods=(.08,.16,.4))
        guard.response_model.commit(velocity,velocity,.65)
        wall=np.array([[25.,y,z] for y in np.arange(-5.,5.01,.25)
                       for z in np.arange(-4.,4.01,.25)])
        guard._update(wall,1.,1.,position)
        nav=LidarNavigator(guard);xy=lambda station:(station,0.);center=lambda station:0.
        projection=nav._projection(0.,xy)
        nav.reference_stations=projection[0];nav.reference_coordinates=projection[1]
        nav.reference_heights=np.zeros(len(projection[0]))
        path=np.column_stack((np.arange(23.),np.zeros((23,2))))
        command,clearance,_=nav._commands(position,velocity,np.array([12.,0.,0.]),
            path,0.,0.,xy,center,None,projection)
        self.assertIsNotNone(command);self.assertGreater(command[0],8.5)
        self.assertLess(command[0],12.)
        samples,extent,times=guard.response_model.envelope(position,velocity,command,.25)
        guard.envelope_times=times
        self.assertLessEqual(extent,guard.horizon)
        self.assertGreaterEqual(guard.index.distance(samples).min(),1.25)
        self.assertTrue(nav._floor_ok(samples,nav._route_stations(samples,0.,xy,projection),
                                     center,None,timed=True))
        full=guard.response_model.prepare(np.array([12.,0.,0.]),velocity,1.,position)
        unsafe,extent,_=guard.response_model.envelope(position,velocity,full,.25)
        self.assertTrue(extent>guard.horizon or guard.index.distance(unsafe).min()<1.25)

    def test_fresh_publication_pose_does_not_erase_next_control_slew_interval(self):
        model=VelocityResponse(lift_gain=0.)
        model.commit(np.array([2.,0.,0.]),np.array([2.,0.,0.]),1.145,control_stamp=1.)
        command=model.prepare(np.array([5.,0.,0.]),np.array([2.,0.,0.]),1.15)
        self.assertAlmostEqual(command[0],2.+4.*.15)
        self.assertEqual(model.last_source_stamp,1.145)
        self.assertEqual(model.last_stamp,1.)

    def test_discrete_brake_uses_elapsed_hold_and_retains_command_between_updates(self):
        for native in (False,True):
            model=VelocityResponse(native=native,lift_gain=0.,coupling_gains=(0.,0.,0.),
                                   brake_feedback=0.,control_periods=(.08,.16,.4))
            if native and model.native.library is None:self.skipTest('Native library unavailable')
            if native:model.native.samples=lambda path,timing,position:(path,timing)
            else:model._sample_envelopes=lambda path,timing,position:(path,timing)
            path,timing=model._envelopes(np.zeros(3),np.array([10.,0.,0.]),
                                        np.array([[10.,0.,0.]]),0.,.35,None)
            # The first brake at0.4s uses prepare()'s0.35s interval cap.
            # For the0.16s case that command then remains held through0.56s.
            for finish in (.48,.56):
                index=int(np.argmin(abs(timing-finish)));interval=finish-.4
                expected=4.+7.2*interval+2.8*.8*(1.-np.exp(-interval/.8))
                self.assertAlmostEqual(path[index,0,1,0],expected,places=11)
            index=int(np.argmin(abs(timing-.56)))
            self.assertLess(path[index,0,0,0],path[index,0,1,0])

    def test_native_discrete_feedback_checks_every_response_period_combination(self):
        path=np.array([[0.,0.,0.],[10.,2.,-2.],[25.,8.,-4.]])
        python=VelocityResponse(native=False,lift_gain=.11,coupling_gains=(.09,.11,.13),
                                coupling_limited=True,xy_error_max=4.5,control_periods=(.08,.16,.4))
        native=VelocityResponse(lift_gain=.11,coupling_gains=(.09,.11,.13),
                                coupling_limited=True,xy_error_max=4.5,control_periods=(.08,.16,.4))
        if native.native.library is None:self.skipTest('Native library unavailable')
        self.assertEqual(len(native.scenarios),9)
        self.assertEqual(set(zip(native.scenarios,native.control_periods)),
                         {(scenario,period) for scenario in ((.8,.10,.09),(.95,.16,.11),(1.1,.22,.13))
                          for period in (.08,.16,.4)})
        for model in (python,native):
            model.stop_profile=LidarNavigator._stop_profile(path,gain=2.)
            model.commit(np.array([10.,1.,.3]),np.array([9.,0.,0.]),1.)
        commands=np.array([[10.,1.,.3],[6.,-1.,1.],[2.,0.,4.5]])
        a=python.envelopes(np.zeros(3),np.array([10.,0.,0.]),commands,.4)
        b=native.envelopes(np.zeros(3),np.array([10.,0.,0.]),commands,.4)
        for expected,actual in zip(a,b):
            np.testing.assert_allclose(actual[0],expected[0],rtol=0.,atol=1e-8)
            np.testing.assert_allclose(actual[2],expected[2],rtol=0.,atol=1e-8)
            self.assertAlmostEqual(actual[1],expected[1],places=8)

    def test_xy_error_budget_retains_height_authority_and_full_longer_stop(self):
        limited=VelocityResponse(lift_gain=.11,coupling_gains=(.09,.11,.13),
                                 coupling_limited=True,xy_error_max=4.5)
        velocity=np.array([10.,0.,0.])
        actual=limited.compensate(np.array([-3.,4.,1.]),velocity)
        self.assertLessEqual(np.linalg.norm(actual[:2]-velocity[:2]),4.5+1e-12)
        self.assertAlmostEqual(actual[2]-.11*np.sum((actual[:2]-velocity[:2])**2),1.)
        unrestricted=VelocityResponse(lift_gain=.11,coupling_gains=(.09,.11,.13),coupling_limited=True)
        a=unrestricted.envelope(np.zeros(3),velocity,velocity,.35)[0]
        b=limited.envelope(np.zeros(3),velocity,velocity,.35)[0]
        self.assertGreater(b[:,0].max(),a[:,0].max()+1.)
        wall=.5*(a[:,0].max()+b[:,0].max())
        # A longer physical stop must remain visible to collision checking.
        points=np.array([[wall,y,z] for y in np.arange(-3.,3.1,.2)
                         for z in np.arange(-4.,4.1,.2)])
        index=PointIndex(points)
        self.assertLess(index.distance(b).min(),1.25)

    def test_native_xy_error_budget_matches_python_on_curve_and_delayed_command(self):
        path=np.array([[0.,0.,0.],[10.,2.,-2.],[25.,8.,-4.]])
        python=VelocityResponse(native=False,lift_gain=.11,coupling_gains=(.09,.11,.13),
                                coupling_limited=True,xy_error_max=4.5)
        native=VelocityResponse(lift_gain=.11,coupling_gains=(.09,.11,.13),
                                coupling_limited=True,xy_error_max=4.5)
        if native.native.library is None:self.skipTest('Native library unavailable')
        for model in (python,native):
            model.stop_profile=LidarNavigator._stop_profile(path,gain=2.)
            model.commit(np.array([10.,1.,.3]),np.array([9.,0.,0.]),1.)
        commands=np.array([[10.,1.,.3],[6.,-1.,1.],[2.,0.,4.5]])
        a=python.envelopes(np.zeros(3),np.array([10.,0.,0.]),commands,.4)
        b=native.envelopes(np.zeros(3),np.array([10.,0.,0.]),commands,.4)
        for expected,actual in zip(a,b):
            np.testing.assert_allclose(actual[0],expected[0],rtol=0.,atol=1e-8)
            np.testing.assert_allclose(actual[2],expected[2],rtol=0.,atol=1e-8)
            self.assertAlmostEqual(actual[1],expected[1],places=8)

    def test_lift_budget_preserves_nominal_vertical_command_at_high_xy_error(self):
        model=VelocityResponse(lift_gain=.11,coupling_limited=True)
        velocity=np.array([[12.,0.,0.],[5.,-7.,0.],[1.,2.,0.]])
        desired=np.array([[-3.,0.,0.],[-4.,5.,2.],[6.,-3.,4.5]])
        actual=model.compensate(desired,velocity)
        effective=actual[:,2]-.11*np.sum((actual[:,:2]-velocity[:,:2])**2,axis=1)
        np.testing.assert_allclose(effective,desired[:,2],atol=1e-12)
        self.assertTrue(np.all(actual[:,2]<=4.5))
        np.testing.assert_array_equal(actual[-1,:2],velocity[-1,:2])
        legacy=VelocityResponse(lift_gain=.11).compensate(desired,velocity)
        self.assertLess(legacy[0,2]-.11*np.sum((legacy[0,:2]-velocity[0,:2])**2),-10.)

    def test_native_lift_budget_matches_python_with_slope_curve_and_applied_delay(self):
        paths=[np.array([[0.,0.,0.],[20.,0.,0.],[40.,0.,0.]]),
               np.array([[0.,0.,0.],[10.,2.,-2.],[25.,8.,-4.]])]
        for path in paths:
            python=VelocityResponse(native=False,lift_gain=.11,coupling_gains=(.09,.11,.13),coupling_limited=True)
            native=VelocityResponse(lift_gain=.11,coupling_gains=(.09,.11,.13),coupling_limited=True)
            if native.native.library is None:self.skipTest('Native library unavailable')
            for model in (python,native):
                model.stop_profile=LidarNavigator._stop_profile(path,gain=2.)
                model.commit(np.array([10.,1.,.3]),np.array([9.,0.,0.]),1.)
            commands=np.array([[10.,1.,.3],[6.,-1.,1.],[2.,0.,4.5]])
            a=python.envelopes(np.zeros(3),np.array([10.,0.,0.]),commands,.4)
            b=native.envelopes(np.zeros(3),np.array([10.,0.,0.]),commands,.4)
            for expected,actual in zip(a,b):
                np.testing.assert_allclose(actual[0],expected[0],rtol=0.,atol=1e-8)
                np.testing.assert_allclose(actual[2],expected[2],rtol=0.,atol=1e-8)
                self.assertAlmostEqual(actual[1],expected[1],places=8)

    def test_native_height_projection_can_differ_from_lateral_braking_path(self):
        path=np.array([[0.,2.,0.],[5.,1.,-2.],[10.,-1.,-4.]])
        height_path=np.array([[0.,0.,0.],[5.,0.,-2.],[10.,0.,-4.]])
        for gain in (1.,2.,2.5):
            python=VelocityResponse(native=False);native=VelocityResponse()
            if native.native.library is None:self.skipTest('Native library unavailable')
            for model in (python,native):
                profile=LidarNavigator._stop_profile(height_path,gain=gain)
                profile.path=path;profile.height_path=height_path;model.stop_profile=profile
                model.commit(np.array([3.,-.2,.3]),np.array([2.,0.,0.]),1.)
            p=np.array([0.,1.,.2]);v=np.array([2.,0.,-.1]);commands=np.array([[3.,-.2,-.5],[0.,0.,-.2]])
            a=python.envelopes(p,v,commands,.4);b=native.envelopes(p,v,commands,.4)
            for expected,actual in zip(a,b):
                np.testing.assert_allclose(actual[0],expected[0],rtol=0.,atol=1e-8)
                np.testing.assert_allclose(actual[2],expected[2],rtol=0.,atol=1e-8)
                self.assertAlmostEqual(actual[1],expected[1],places=8)

    def test_delayed_braking_uses_observed_interval_up_to_command_hold_horizon(self):
        velocity=np.array([6.,0.,0.])
        for interval in (.05,.25,2.):
            model=VelocityResponse()
            model.commit(velocity,velocity,1.)
            command=model.prepare(np.zeros(3),velocity,1.+interval)
            expected=6.-8.*min(interval,.35)
            self.assertAlmostEqual(command[0],expected)
            self.assertAlmostEqual(command[2],model.lift_gain*(6.-expected)**2)

    def test_coupled_escape_checks_previous_driving_command_and_full_stopping_tail(self):
        points=np.array([[.8,y,z] for y in np.arange(-4.,4.01,.2)
                         for z in np.arange(-4.,4.01,.2)])
        guard=ExecutionGuard();guard.response_model=VelocityResponse(height_gain=2.)
        target=np.array([-1.,0.,0.])
        command=guard.escape(np.zeros(3),np.zeros(3),target,points,1.,1.05)
        self.assertIsNotNone(command)
        samples,_,times=guard.response_model.envelope(np.zeros(3),np.zeros(3),command,.4)
        initial=np.minimum(np.sum(points**2,axis=1),1.25**2)
        for q in samples:
            self.assertTrue(np.all(np.sum((points-q)**2,axis=1)>=initial-1e-7))
        tail=samples[times>=times.max()-.15]
        self.assertGreater(guard.index.distance(tail).min(),.81)
        # An old command toward the wall makes the delay unsafe despite
        # zero measured velocity and a geometrically safe retreat ray.
        original=guard.response_model.stop_profile
        guard.response_model.commit(np.array([.6,0.,0.]),np.zeros(3),1.)
        self.assertIsNone(guard.escape(np.zeros(3),np.zeros(3),target,points,1.,1.05))
        self.assertIs(guard.response_model.stop_profile,original)

    def test_coupled_escape_cannot_relax_bounds_or_moving_object_clearance(self):
        points=np.array([[.8,y,z] for y in np.arange(-4.,4.01,.2)
                         for z in np.arange(-4.,4.01,.2)])
        guard=ExecutionGuard();guard.response_model=VelocityResponse(height_gain=2.)
        guard.command_constraint=lambda q:np.all(q[:,0]>=-.05)
        self.assertIsNone(guard.escape(np.zeros(3),np.zeros(3),np.array([-1.,0.,0.]),points,1.,1.05))
        guard.command_constraint=None
        guard.dynamic_scene=LidarScene();guard.dynamic_scene.pose_stamp=1.05
        guard.dynamic_scene.tracks=[dict(support=4,velocity=np.array([2.,0.,0.]),stamp=1.,
            center=np.array([-1.,0.,0.]),half=np.array([.3,.3,.3]))]
        self.assertIsNone(guard.escape(np.zeros(3),np.zeros(3),np.array([-1.,0.,0.]),points,1.,1.05))

    def test_height_feedback_gain_matches_native_response_for_grade_and_vertical_recovery(self):
        paths=[np.array([[0.,0.,0.],[5.,0.,-1.5],[10.,2.,-2.]]),
               np.array([[0.,0.,0.],[0.,0.,-.8]])]
        for path in paths:
            for gain in (1.,2.,2.5):
                python=VelocityResponse(native=False);native=VelocityResponse()
                if native.native.library is None:self.skipTest('Native library unavailable')
                for model in (python,native):
                    model.stop_profile=LidarNavigator._stop_profile(path,gain=gain)
                    model.commit(np.array([2.,0.,.1]),np.array([1.,0.,-.1]),1.)
                p=np.array([0.,0.,.2]);v=np.array([1.,0.,-.1]);commands=np.array([[2.,0.,-.3],[0.,0.,-.5]])
                a=python.envelopes(p,v,commands,.3);b=native.envelopes(p,v,commands,.3)
                for expected,actual in zip(a,b):
                    np.testing.assert_allclose(actual[0],expected[0],rtol=0.,atol=1e-8)
                    np.testing.assert_allclose(actual[2],expected[2],rtol=0.,atol=1e-8)
                    self.assertAlmostEqual(actual[1],expected[1],places=8)

    def test_near_cross_road_face_can_certify_short_retreat_before_replanning(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        nav=LidarNavigator(guard)
        points=np.array([[.8,y,z] for y in np.arange(-4.,4.01,.1) for z in np.arange(-4.,4.01,.1)])
        command,info=nav.select(np.zeros(3),np.zeros(3),np.array([6.,0.,0.]),0.,
            lambda s:(s,0.),lambda s:0.,points,1.,1.05)
        self.assertEqual(info['command_reason'],'LIDAR_ESCAPE')
        self.assertLess(command[0],-.05)
        self.assertLess(info['recovery_target'][0],-.5)
        self.assertLessEqual(np.linalg.norm(command),.6)

    def test_uncertified_near_wall_recovery_settles_instead_of_full_z_bursts(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        points=np.array([[x,.8-.1*z,z] for x in np.arange(-3.,3.01,.2) for z in np.arange(-4.,4.01,.1)])
        command,info=guard.filter_command(np.zeros(3),np.array([0.,0.,.2]),np.zeros(3),points,1.,1.05)
        self.assertEqual(info['command_reason'],'COMMAND_BLOCKED')
        self.assertTrue(info.get('blocked_settle'))
        self.assertAlmostEqual(command[2],-.12)
        self.assertTrue(info['blocked_brake'])
        self.assertLess(info['command_clearance'],1.25)

    def test_blocked_ceiling_floor_never_rank_rejected_height_bursts(self):
        for limited in (False,True):
            for vz in (-2.,-.2,0.,.2,2.):
                with self.subTest(limited=limited,vz=vz):
                    guard=ExecutionGuard();guard.response_model=VelocityResponse(coupling_limited=limited)
                    points=np.array([[0.,0.,-.8],[0.,0.,.9]])
                    commands=[]
                    for desired_z in (-4.,4.5,-4.,4.5):
                        command,info=guard.filter_command(np.zeros(3),np.array([0.,0.,vz]),
                            np.array([0.,0.,desired_z]),points,1.,1.05)
                        self.assertEqual(info['command_reason'],'COMMAND_BLOCKED')
                        self.assertTrue(info['blocked_brake'])
                        import json
                        json.dumps(info,allow_nan=False)
                        self.assertLessEqual(abs(command[2]),.8)
                        self.assertLessEqual(command[2]*vz,0.)
                        commands.append(command)
                    for command in commands[1:]:np.testing.assert_array_equal(command,commands[0])

    def test_safe_height_motion_is_still_certified_with_response_model(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        command,info=guard.filter_command(np.zeros(3),np.zeros(3),np.array([0.,0.,-.5]),
            np.array([[0.,20.,0.]]),1.,1.05)
        self.assertEqual(info['command_reason'],'COMMAND_BRAKING')
        self.assertAlmostEqual(command[2],-.5)

    def test_blocked_high_speed_brake_retains_vertical_authority(self):
        guard=ExecutionGuard();model=VelocityResponse(lift_gain=.11,coupling_limited=True,xy_error_max=4.5)
        guard.response_model=model;velocity=np.array([12.,0.,-1.])
        model.commit(np.zeros(3),velocity,1.)
        command,info=guard.filter_command(np.zeros(3),velocity,np.array([0.,0.,4.]),
            np.array([[0.,.8,0.]]),1.,1.3)
        self.assertEqual(info['command_reason'],'COMMAND_BLOCKED')
        error=np.linalg.norm(command[:2]-velocity[:2])
        self.assertAlmostEqual(error,4.5)
        nominal=command[2]-model.lift_gain*error*error
        self.assertAlmostEqual(nominal,.6)
        self.assertLess(command[2],4.5)

    def test_rejected_brake_retains_grade_until_horizontal_inertia_stops(self):
        for slope in (-.5,.5):
            for native in (False,True):
                with self.subTest(slope=slope,native=native):
                    guard=ExecutionGuard()
                    model=VelocityResponse(lift_gain=.095,coupling_limited=True,xy_error_max=4.5,native=native)
                    guard.response_model=model
                    model.stop_profile=LidarNavigator._stop_profile(np.array([[0.,0.,0.],[20.,0.,20.*slope]]))
                    velocity=np.array([6.,0.,6.*slope]);model.commit(np.array([6.,0.,6.*slope]),velocity,1.)
                    command,info=guard.filter_command(np.zeros(3),velocity,np.zeros(3),
                        np.array([[0.,.8,0.]]),1.25,1.3)
                    self.assertEqual(info['command_reason'],'COMMAND_BLOCKED')
                    self.assertTrue(info['blocked_grade_follow'])
                    nominal=command[2]-model.lift_gain*np.sum((command[:2]-velocity[:2])**2)
                    self.assertAlmostEqual(nominal,6.*slope)
                    self.assertLess(command[0],6.)
                    # A rejected height target must not keep pumping Z once
                    # horizontal motion has settled beneath a vehicle.
                    self.assertAlmostEqual(model.fallback_vertical(np.zeros(3),np.array([0.,0.,.2])),-.12)

    def test_native_timed_sampling_preserves_duplicate_points_and_endpoints(self):
        native=NativeResponse()
        if native.library is None:self.skipTest('Native library unavailable')
        rng=np.random.default_rng(827)
        path=np.cumsum(rng.uniform(-.3,.3,(70,4,3,3)),axis=0)
        path[12:15]=path[11]
        times=np.arange(70)*.08;origin=np.array([1.,2.,3.])
        reference=VelocityResponse._sample_envelopes(path,times,origin)
        actual=native.samples(path,times,origin)
        for a,b in zip(actual,reference):
            np.testing.assert_allclose(a[0],b[0],rtol=0.,atol=1e-12)
            np.testing.assert_allclose(a[2],b[2],rtol=0.,atol=1e-12)
            self.assertAlmostEqual(a[1],b[1],places=12)

    def test_delay_keeps_applied_command_and_covers_coasting_case(self):
        model=VelocityResponse(native=False)
        old=np.array([0.,6.,.086*36.])
        model.commit(old,np.zeros(3),1.)
        np.testing.assert_array_equal(model.applied_command,old)
        self.assertAlmostEqual(model.previous[2],0.)
        points=np.array([[0.,2.,z] for z in np.arange(-20.,20.01,.1)])
        index=PointIndex(points)
        samples,_,times=model.envelope(np.zeros(3),np.zeros(3),np.zeros(3),.4)
        # An accelerating old command can hit the wall although a zero-
        # velocity coast predicts staying safely at the origin.
        self.assertLess(index.distance(samples).min(),1.25)
        self.assertGreater(samples[times<=.4,1].max(),.25)
        self.assertTrue(np.any(np.linalg.norm(samples,axis=1)<1e-10))
        model.reset()
        self.assertIsNone(model.applied_command)
        coast,_,_=model.envelope(np.zeros(3),np.zeros(3),np.zeros(3),.4)
        self.assertGreater(index.distance(coast).min(),1.25)

    def test_native_surface_queries_match_supported_hulls_and_outside_points(self):
        from unittest.mock import patch
        native=NativeResponse()
        if native.library is None:self.skipTest('Native library unavailable')
        rng=np.random.default_rng(234)
        queries=rng.uniform([-3.,-3.,-2.],[3.,3.,7.],(5000,3))
        for slope in [0.,.3]:
            points=np.array([(x,y,slope*x) for x in np.arange(-2.,2.01,.2) for y in np.arange(-2.,2.01,.2)])
            index=PointIndex(points,origin=np.array([0.,0.,1.5]),continuous_surfaces=True,road_height=5.)
            self.assertIsNotNone(index.patches)
            # Supply a supported roof hull explicitly; roof detection has
            # separate tests and is unchanged by this numerical optimization.
            hull=np.array([[-2.,-2.],[2.,-2.],[2.,2.],[-2.,2.]])
            normals=np.array([[0.,1.],[-1.,0.],[0.,-1.],[1.,0.]])
            index.roof=(np.zeros(3),np.array([slope,0.,0.]),hull,normals)
            with patch('point_index.native_patch_backend',return_value=None):
                expected=[index.patch_distance(queries),index.surface_distance(queries),index.floor_limit(queries,1.25)]
            with patch('point_index.native_patch_backend',return_value=native):
                actual=[index.patch_distance(queries),index.surface_distance(queries),index.floor_limit(queries,1.25)]
            for a,b in zip(actual,expected):np.testing.assert_allclose(a,b,rtol=0.,atol=1e-12)

    def test_replanned_measured_connector_cannot_erase_altitude_feedback(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse();guard.pose_stamp=1.
        guard.index=PointIndex(np.array([[0.,20.,-2.2]]))
        nav=LidarNavigator(guard);nav.reference_stations=np.arange(25.)
        nav.reference_heights=np.full(25,-2.2);nav.reference_coordinates=np.c_[np.arange(25.),np.zeros(25)]
        position=np.array([0.,0.,-1.])
        path=np.array([[0.,0.,-1.],[1.,0.,-2.2],[2.,0.,-2.2],[24.,0.,-2.2]])
        command,_,_=nav._commands(position,np.zeros(3),np.zeros(3),path,0.,0.,lambda s:(s,0.),lambda s:-2.2,None)
        self.assertLess(command[2],-.8)
        # A true graded profile is preserved, rather than aiming one whole
        # forward station's grade change above the aircraft.
        graded=np.array([[0.,0.,0.],[1.,0.,-.3],[2.,0.,-.6]])
        np.testing.assert_allclose(nav._command_profile(graded),graded,atol=1e-12)

    def test_rejected_new_path_does_not_replace_the_committed_braking_height(self):
        from unittest.mock import patch
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        nav=LidarNavigator(guard)
        nav.braking_path=np.array([[0.,0.,0.],[24.,0.,0.]])
        nav.last_path=np.array([[0.,0.,1.],[24.,0.,1.]])
        with patch.object(nav,'path',return_value=(None,{})):
            command,info=nav.select(np.array([1.,0.,.8]),np.zeros(3),np.zeros(3),1.,
                lambda s:(s,0.),lambda s:0.,np.array([[0.,20.,0.]]),1.,1.05)
        self.assertEqual(info['command_reason'],'COMMAND_BRAKING')
        self.assertLess(command[2],-.5)

    def test_native_projection_preserves_duplicate_segments_and_first_ties(self):
        native=NativeResponse()
        if native.library is None:self.skipTest('Native library unavailable')
        nav=LidarNavigator(ExecutionGuard())
        queries=np.random.default_rng(81).uniform([-1.,-3.,-2.],[30.,5.,3.],(5000,3))
        for xy in (lambda s:(s,2.*np.sin(s/8.)),lambda s:(3.,1.)):
            projection=nav._projection(3.,xy)
            stations,road,segments,length=projection
            road[7]=road[6];segments[:]=np.diff(road,axis=0);length[:]=np.sum(segments**2,axis=1)
            expected=nav._route_stations(queries,3.,xy,projection)
            np.testing.assert_allclose(native.project(queries,projection),expected,rtol=0.,atol=1e-12)

    def test_curved_brake_steers_and_still_stops(self):
        radius=60.;arc=np.arange(0.,40.1,.2)
        path=np.column_stack([radius*np.sin(arc/radius),radius*(1.-np.cos(arc/radius)),np.zeros(len(arc))])
        model=VelocityResponse(native=False);model.stop_profile=LidarNavigator._stop_profile(path)
        samples,_,_=model.envelope(np.zeros(3),np.array([10.,0.,0.]),np.array([10.,0.,0.]),.4)
        error=abs(np.linalg.norm(samples[:,:2]-[0.,radius],axis=1)-radius)
        self.assertLess(error.max(),2.1)
        np.testing.assert_array_equal(model.braking_target(np.zeros(2),np.array([10.,3.,0.])),np.zeros(2))
        self.assertGreater(model.braking_target(np.array([10.,0.]),np.array([7.5,0.,0.]))[1],0.)

    def test_native_integration_matches_python_for_slopes_curves_and_duplicate_points(self):
        native=NativeResponse()
        if native.library is None:self.skipTest('Native library not built; Python fallback remains available')
        paths=[None,np.array([[0.,0.,0.],[0.,0.,.1],[3.,0.,-.5],[8.,2.,-1.],[30.,5.,-2.]]),
               np.array([[0.,0.,0.],[0.,0.,1.]])]
        rng=np.random.default_rng(231)
        for path in paths:
            compiled=VelocityResponse();reference=VelocityResponse(native=False)
            if path is not None:
                compiled.stop_profile=LidarNavigator._stop_profile(path)
                reference.stop_profile=LidarNavigator._stop_profile(path)
                applied=rng.uniform(-3.,8.,3)
                compiled.commit(applied,np.zeros(3),1.)
                reference.commit(applied,np.zeros(3),1.)
            for _ in range(3):
                position=rng.uniform(-.3,.3,3);velocity=rng.uniform(-3.,8.,3)
                velocity[2]*=.1
                commands=rng.uniform(-3.,10.,(5,3));commands[:,2]*=.1
                a=compiled.envelopes(position,velocity,commands,.4)
                b=reference.envelopes(position,velocity,commands,.4)
                for first,second in zip(a,b):
                    np.testing.assert_allclose(first[0],second[0],rtol=0.,atol=2e-10)
                    np.testing.assert_allclose(first[2],second[2],rtol=0.,atol=2e-10)
                    self.assertAlmostEqual(first[1],second[1],places=9)

    def test_braking_keeps_slew_and_compensates_horizontal_error(self):
        model=VelocityResponse();model.previous=np.array([6.,0.,0.]);model.last_stamp=1.
        command=model.prepare(np.zeros(3),np.array([6.,0.,0.]),1.05)
        self.assertGreater(command[0],5.5)
        self.assertGreater(command[2],0.)
        samples,extent,times=model.envelope(np.zeros(3),np.array([6.,0.,0.]),command,.4)
        # A smooth stop cannot be certified using the old v*0.8 coast length.
        self.assertGreater(extent,10.)
        self.assertEqual(len(samples),len(times))
        self.assertGreater(times.max(),2.)

    def test_vertical_compensation_is_not_clipped_to_the_base_descent_speed(self):
        model=VelocityResponse(vertical_command_max=12.)
        model.previous=np.array([-3.,0.,0.]);model.last_stamp=1.
        velocity=np.array([7.,0.,0.])
        command=model.prepare(np.array([-3.,0.,4.5]),velocity,1.05)
        self.assertEqual(command[2],12.)
        # The declared flight command includes coupling compensation;
        # commit recovers the smaller base altitude-controller command.
        model.commit(command,velocity,1.05)
        self.assertLess(model.previous[2],4.5)

    def test_compensated_braking_tracks_a_downhill_road(self):
        path=np.array([(x,0.,.3*x) for x in np.arange(31.)])
        model=VelocityResponse(vertical_command_max=12.)
        model.stop_profile=LidarNavigator._stop_profile(path)
        v=np.array([8.,0.,2.4]);command=np.array([8.,0.,2.4])
        samples,_,_=model.envelope(np.zeros(3),v,command,0.)
        self.assertLess(np.max(abs(samples[:,2]-.3*samples[:,0])),1.0)

    def test_counter_braking_stops_before_wall_at_operating_speed(self):
        wall=np.array([(27.,y,z) for y in np.arange(-5.,5.1,.25) for z in np.arange(-4.,4.1,.25)])
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        guard.response_model.previous=np.array([10.,0.,0.]);guard.response_model.last_stamp=1.
        guard.response_model.stop_profile=LidarNavigator._stop_profile(np.array([[0.,0.,0.],[40.,0.,0.]]))
        command,info=guard.filter_command(np.zeros(3),np.array([10.,0.,0.]),np.array([10.,0.,0.]),wall,1.,1.05)
        samples,extent=guard.command_envelope(np.zeros(3),np.array([10.,0.,0.]),command,.05)
        self.assertEqual(info['command_reason'],'COMMAND_BRAKING')
        self.assertGreater(command[0],9.9)
        self.assertLess(extent,26.)
        self.assertGreaterEqual(guard.index.distance(samples).min(),1.25)

    def test_batched_and_single_stop_predictions_agree(self):
        model=VelocityResponse();v=np.array([4.,1.,-.2]);p=np.array([0.,0.,-2.])
        commands=np.array([[4.,1.,0.],[3.,0.,.2]])
        batch=model.envelopes(p,v,commands,.4)
        for i,c in enumerate(commands):
            single=model.envelope(p,v,c,.4)
            self.assertLess(abs(batch[i][1]-single[1]),.01)
            self.assertLess(np.linalg.norm(batch[i][0][-1]-single[0][-1]),.02)

    def test_continuous_patch_fills_return_gap_without_extrapolating(self):
        plane=np.array([(x,y,0.) for x in np.arange(-2.,2.01,.5) for y in np.arange(-2.,2.01,.5)])
        index=PointIndex(plane,origin=np.array([0.,0.,1.5]),continuous_surfaces=True)
        self.assertLess(index.distance([[.25,.25,1.24]])[0],1.25)
        self.assertTrue(np.isinf(index.patch_distance([[5.,0.,1.]])[0]))

    def test_different_supported_polygon_sizes_preserve_edges_and_open_space(self):
        triangle=np.array([[-.5,-.4],[.5,-.4],[0.,.6]])
        interior=np.array([[0.,0.],[-.1,0.],[.1,0.],[0.,.1],[0.,-.1],
                           [-.2,-.2],[.2,-.2],[-.1,.2],[.1,.2],[0.,.3]])
        rectangle=np.array([[x,y] for x in (-.45,0.,.45) for y in (-.45,0.,.45)])
        angles=np.arange(8)*np.pi/4.
        octagon=np.vstack([radius*np.column_stack((np.cos(angles),np.sin(angles)))
                           for radius in (.5,.25)])
        centers=np.array([[0.,0.,1.],[4.,0.,1.],[8.,0.,1.]])
        points=np.vstack([np.column_stack((polygon,np.zeros(len(polygon))))+center
                          for polygon,center in zip((np.vstack((triangle,interior)),rectangle,octagon),centers)])
        index=PointIndex(points,origin=np.zeros(3),continuous_surfaces=True,
                         surface_seeds=np.repeat(centers,6,axis=0))
        self.assertIsNotNone(index.patches)
        np.testing.assert_array_equal(np.isfinite(index.patches[4]).sum(axis=1),[3,4,8])
        np.testing.assert_allclose(index.patch_distance(centers+[0.,0.,.2]),.2,atol=1e-6)
        outside=centers+[1.1,0.,.2]
        self.assertTrue(np.isinf(index.patch_distance(outside)).all())
        self.assertTrue((index.distance(outside)>.5).all())

    def test_sloped_road_stop_keeps_height_instead_of_coasting_below_road(self):
        model=VelocityResponse()
        path=np.array([(x,0.,-.3*x) for x in np.arange(0.,31.)])
        model.stop_profile=LidarNavigator._stop_profile(path)
        samples,_,_=model.envelope(np.zeros(3),np.array([4.,0.,-1.2]),np.array([4.,0.,-1.2]),.4)
        self.assertLess(np.max(abs(samples[:,2]+.3*samples[:,0])),.6)

    def test_grade_command_uses_slewed_speed_during_startup(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        nav=LidarNavigator(guard)
        guard.pose_stamp=1.
        path=np.array([(x,0.,-.3*x) for x in np.arange(25.)])
        guard.index=PointIndex(np.array([[0.,20.,0.]]))
        nav.reference_stations=np.arange(25.);nav.reference_heights=path[:,2]
        command,_,_=nav._commands(np.zeros(3),np.zeros(3),np.array([10.,0.,-3.]),path,0.,0.,lambda s:(s,0.),lambda s:-.3*s,None)
        self.assertIsNotNone(command)
        self.assertGreater(command[0],.08)
        # Climbing at the target's 10 m/s grade would demand -3 m/s
        # while startup sends only 0.1 m/s horizontally.
        self.assertLess(abs(command[2]),.06)

    def test_short_cached_connector_cannot_command_full_vertical_speed(self):
        path=np.array([[0.,0.,0.],[.02,0.,-.1],[3.,0.,-.1],[10.,0.,0.]])
        target=LidarNavigator._stop_profile(path)(np.array([[0.,0.,0.]]),np.array([[6.,0.,0.]]))
        self.assertLess(abs(target[0]),.5)

    def test_timed_out_geometry_worker_is_discarded_and_replaced(self):
        import os,signal
        if os.name!='posix':self.skipTest('POSIX worker suspension test')
        process=GeometryProcess(timeout=.15)
        old_pid=process.process.pid
        index=PointIndex(np.array([[0.,20.,0.]]))
        ss=np.arange(-1.,26.,.5)
        snapshot=(np.zeros(3),0.,ss,np.column_stack([ss,np.zeros(len(ss))]),np.zeros(len(ss)),None,index,1.15,24.,.06,True)
        try:
            os.kill(old_pid,signal.SIGSTOP)
            with self.assertRaisesRegex(RuntimeError,'deadline'):
                process.submit(None,snapshot).result(timeout=3.)
            process.timeout=4.
            path,_=process.submit(None,snapshot).result(timeout=6.)
            self.assertIsNotNone(path)
            self.assertNotEqual(process.process.pid,old_pid)
        finally:process.shutdown()

    def test_empty_space_below_road_is_not_a_legal_command_corridor(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        nav=LidarNavigator(guard)
        nav.reference_stations=np.array([0.,30.]);nav.reference_heights=np.zeros(2)
        self.assertFalse(nav._floor_ok(np.array([[2.,0.,2.]]),np.array([2.]),lambda s:0.,None))

    def test_empty_space_outside_road_cannot_certify_a_stopping_path(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        nav=LidarNavigator(guard)
        nav.reference_stations=np.array([0.,30.]);nav.reference_heights=np.zeros(2)
        nav.reference_coordinates=np.array([[0.,0.],[30.,0.]])
        self.assertFalse(nav._floor_ok(np.array([[2.,3.,0.]]),np.array([2.]),lambda s:0.,None))
        self.assertTrue(nav._floor_ok(np.array([[2.,2.,0.]]),np.array([2.]),lambda s:0.,None))

    def test_small_measured_road_overshoot_can_reenter_without_worsening_it(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        nav=LidarNavigator(guard)
        command,info=nav.select(np.array([0.,2.4,0.]),np.zeros(3),np.array([4.,0.,0.]),0.,
            lambda s:(s,0.),lambda s:0.,np.array([[0.,20.,0.]]),1.,1.05)
        self.assertTrue(info['road_recovery'])
        self.assertTrue(info['feasible'])
        self.assertLess(command[1],0.)
        self.assertLessEqual(np.max(abs(np.array(info['path'])[1:,1])),2.25)

    def test_offset_grid_floor_uses_the_actual_projected_station(self):
        angle=.35
        xy=lambda s:(s,0.) if s<=3. else (3.+(s-3.)*np.cos(angle),(s-3.)*np.sin(angle))
        center=lambda s:-.1*s
        points=np.array([[xy(s)[0]-y*(0. if s<3 else np.sin(angle)),
                          xy(s)[1]+y*(1. if s<3 else np.cos(angle)),
                          center(s)-(1.08 if 3<=s<=8 else 1.5)]
                         for s in np.arange(0.,20.,.2) for y in np.arange(-3.,3.1,.2)])
        guard=ExecutionGuard();guard.response_model=VelocityResponse();guard.index=PointIndex(points)
        nav=LidarNavigator(guard,budget=0.)
        path,info=nav.path(np.array([0.,-2.,.112]),0.,xy,center,.2577)
        self.assertIsNotNone(path)
        self.assertEqual(info['grid_attempts'],1)

    def test_chunked_projection_preserves_exact_nearest_segments(self):
        nav=LidarNavigator(ExecutionGuard())
        xy=lambda s:(s,2.*np.sin(s/8.))
        projection=nav._projection(3.,xy)
        stations,road,segments,length=projection
        # Include a duplicate route segment and more than two chunks.
        road[7]=road[6];segments[:]=np.diff(road,axis=0)
        length[:]=np.sum(segments**2,axis=1)
        queries=np.random.default_rng(81).uniform([-1.,-3.,-2.],[30.,5.,3.],(5000,3))
        delta=queries[:,None,:2]-road[None,:-1]
        fractions=np.clip(np.sum(delta*segments,axis=2)/np.maximum(1e-8,length),0.,1.)
        distances=np.sum((delta-fractions[:,:,None]*segments)**2,axis=2)
        distances[:,length<=1e-8]=np.inf
        nearest=np.argmin(distances,axis=1)
        expected=stations[nearest]+.5*fractions[np.arange(len(queries)),nearest]
        np.testing.assert_allclose(nav._route_stations(queries,3.,xy,projection),expected,rtol=0.,atol=1e-12)

    def test_grid_minimization_keeps_unreachable_cells_and_first_equal_parent(self):
        previous=np.array([[1.,np.inf],[2.,3.]])
        neighbors=np.array([[[0,2,4],[1,4,4]],[[2,0,3],[3,2,0]]])
        shift=np.array([[[1.,0.,np.inf],[0.,np.inf,np.inf]],[[0.,1.,0.],[0.,1.,2.]]])
        costs,parent=LidarNavigator._advance_grid(previous,neighbors,shift)
        np.testing.assert_array_equal(costs,[[2.,np.inf],[2.,3.]])
        self.assertEqual(parent[0,0],0)
        self.assertEqual(parent[1,0],2)
        self.assertEqual(parent[1,1],3)

    def test_changed_far_obstacle_preserves_only_clear_near_prefix(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        wall=np.array([(18.,y,z) for y in np.arange(-4.,4.1,.25) for z in np.arange(-3.,3.1,.25)])
        guard.index=PointIndex(wall)
        nav=LidarNavigator(guard)
        xy=lambda s:(s,0.);center=lambda s:0.
        projection=nav._projection(0.,xy)
        nav.reference_stations=projection[0];nav.reference_coordinates=projection[1]
        nav.reference_heights=np.zeros(len(projection[0]))
        path=np.array([(x,0.,0.) for x in np.arange(25.)])
        prefix,info=nav._usable_path(path,np.zeros(3),0.,xy,center,None,projection)
        self.assertTrue(info['truncated'])
        self.assertGreater(prefix[-1,0],15.)
        self.assertLessEqual(prefix[-1,0],16.75)
        from path_sampling import swept_samples
        samples,_,_=swept_samples(prefix)
        self.assertGreaterEqual(guard.index.distance(samples).min(),1.25)
        guard.pose_stamp=1.
        command,clearance,_=nav._commands(np.zeros(3),np.zeros(3),np.array([8.,0.,0.]),prefix,.05,0.,xy,center,None,projection)
        self.assertIsNotNone(command)
        self.assertGreaterEqual(clearance,1.25)

    def test_cached_prefix_still_checks_height_and_rejects_near_obstacles(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        nav=LidarNavigator(guard)
        xy=lambda s:(s,0.);center=lambda s:0.;projection=nav._projection(0.,xy)
        nav.reference_stations=projection[0];nav.reference_coordinates=projection[1]
        nav.reference_heights=np.zeros(len(projection[0]))
        guard.index=PointIndex(np.array([[0.,20.,0.]]))
        path=np.array([(x,0.,0. if x<10 else .25*(x-10)) for x in np.arange(25.)])
        prefix,info=nav._usable_path(path,np.zeros(3),0.,xy,center,None,projection)
        self.assertTrue(info['truncated'])
        self.assertLessEqual(prefix[-1,2],1.25)
        guard.index=PointIndex(np.array([[2.,0.,0.]]))
        prefix,_=nav._usable_path(path,np.zeros(3),0.,xy,center,None,projection)
        self.assertIsNone(prefix)

    def test_floor_disagreement_can_recover_but_cannot_certify_hovering(self):
        from predictive_avoidance import departure_floor_limits
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        nav=LidarNavigator(guard)
        position=np.array([0.,0.,.4])
        command,info=nav.select(position,np.zeros(3),np.array([4.,0.,0.]),0.,
            lambda s:(s,0.),lambda s:0.,np.array([[0.,20.,0.]]),1.,1.05,.25)
        self.assertNotEqual(info['command_reason'],'COMMAND_BLOCKED')
        self.assertLess(command[2],0.)
        self.assertFalse(nav._floor_ok(np.array([position,position]),np.zeros(2),lambda s:0.,.25))
        samples,_=guard.command_envelope(position,np.zeros(3),command,.05)
        stations=nav._route_stations(samples,0.,lambda s:(s,0.))
        limits=departure_floor_limits(stations,samples,lambda s:0.,.25,guard.index,1.25)
        error=samples[:,2]-limits
        self.assertLessEqual(error.max(),.15+1e-8)
        tails=guard.envelope_times>=guard.envelope_times.max()-.15
        self.assertLessEqual(error[tails].max(),.14+1e-8)

    def test_inserted_height_samples_do_not_break_physical_neighbors(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse();guard.index=PointIndex(np.array([[0.,20.,0.]]))
        nav=LidarNavigator(guard,budget=0.)
        path,_=nav.path(np.array([0.,0.,1.24]),0.,lambda s:(s,0.),lambda s:0.,1.27)
        self.assertIsNotNone(path)
        self.assertLessEqual(path[1,2],1.+1e-8)
        self.assertLessEqual(np.max(abs(np.diff(path[:,2]))),.25+1e-8)

    def test_process_planner_uses_a_serializable_geometry_snapshot(self):
        guard=ExecutionGuard();guard.response_model=VelocityResponse()
        nav=LidarNavigator(guard,async_planning=True,process_planning=True)
        walls=np.array([(x,y,z) for x in np.arange(-5.,35.,1.) for y in (-6.,6.) for z in (-3.,0.,3.)])
        try:
            nav.select(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),0.,lambda s:(s,0.),lambda s:0.,walls,1.,1.05)
            path,info=nav.future.result(timeout=8.)
            self.assertIsNotNone(path)
            self.assertGreater(info['path_distance'],20.)
        finally:nav.close()

    def test_static_cluster_does_not_acquire_drone_velocity(self):
        cloud=np.array([(x,y,z) for x in np.arange(8.,9.01,.2) for y in np.arange(-.5,.51,.2) for z in (-.5,.5)])
        scene=LidarScene()
        for i in range(6):scene.update(cloud,np.array([i*.3,0.,0.]),1.+i*.1,1.05+i*.1,np.array([1.,0.]),lambda s:0.,i*.3)
        self.assertEqual(scene.summary()['moving_obstacles'],0)

    def test_rear_approaching_cluster_blocks_future_hover(self):
        shape=np.array([(x,y,z) for x in np.arange(-.5,.51,.2) for y in np.arange(-.5,.51,.2) for z in (-.5,.5)])
        scene=LidarScene()
        for i in range(8):scene.update(shape+[-6.+i*.2,0.,0.],np.zeros(3),1.+i*.1,1.05+i*.1,np.array([1.,0.]),lambda s:0.,0.)
        self.assertGreaterEqual(scene.summary()['moving_obstacles'],1)
        self.assertGreater(scene.distance([[0.,0.,0.]],[0.])[0],1.25)
        self.assertLess(scene.distance([[0.,0.,0.]],[3.])[0],1.25)


if __name__=='__main__':unittest.main()
