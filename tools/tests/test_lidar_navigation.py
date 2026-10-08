import sys
import unittest
from concurrent.futures import Future
from pathlib import Path
import numpy as np
from unittest.mock import patch,Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from execution_guard import ExecutionGuard
from lidar_navigation import LidarNavigator
from path_sampling import swept_samples


class LidarNavigationTests(unittest.TestCase):
    def test_future_occupancy_cost_starts_bypass_earlier_without_changing_buffer(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse()
        obstacle=self.car+np.array([7.,0.,0.])
        self.guard._update(obstacle,1.,1.05,np.zeros(3))
        paths=[]
        for distance in (0.,9.):
            navigator=LidarNavigator(self.guard,budget=.5,anticipation_distance=distance)
            path,_=navigator.path(np.zeros(3),0.,self.xy,self.center,None)
            self.assertIsNotNone(path)
            samples,_,_=swept_samples(path)
            self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)
            paths.append(path)
        first=[p[np.flatnonzero(abs(p[:,1])>.5)[0],0] for p in paths]
        self.assertLess(first[1],first[0]-2.)
        # The real geometry-worker entry must produce the same early bypass.
        from lidar_navigation import _plan_snapshot
        stations=np.arange(-1.,35.1,.5)
        coordinates=np.column_stack((stations,np.zeros(len(stations))))
        worker_path,_=_plan_snapshot((np.zeros(3),0.,stations,coordinates,
            np.zeros(len(stations)),None,self.guard.index,self.guard.margin,24.,.5,True,9.))
        np.testing.assert_allclose(worker_path,paths[1],atol=1e-8,rtol=0.)

    def test_pending_full_planner_can_certify_fresh_local_motion(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse(coupling_limited=True)
        self.nav=LidarNavigator(self.guard,local_replan=True)
        self.nav.executor=Mock();self.nav.future=Future()
        import time
        self.nav.submitted_at=time.monotonic()
        command,info=self.nav.select(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),
            0.,self.xy,self.center,self.car,1.,1.05)
        self.assertTrue(info['local_replan'])
        self.assertEqual(info['command_reason'],'LIDAR_TRACK')
        self.assertGreater(command[0],.1)
        self.assertLessEqual(info['path_distance'],12.)
        samples,_=self.guard.command_envelope(np.zeros(3),np.zeros(3),command,.05)
        self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)
        self.assertTrue(self.guard.command_constraint(samples))

    def test_local_replan_does_not_certify_motion_through_a_new_wall(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse(coupling_limited=True)
        self.nav=LidarNavigator(self.guard,local_replan=True)
        self.nav.executor=Mock();self.nav.future=Future()
        import time
        self.nav.submitted_at=time.monotonic()
        self.nav.last_path=np.column_stack((np.arange(25.),np.zeros((25,2))))
        self.nav.last_plan_stamp=1.
        wall=np.array([(2.,y,z) for y in np.arange(-5.,5.1,.25)
                       for z in np.arange(-4.,4.1,.25)])
        command,info=self.nav.select(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),
            0.,self.xy,self.center,wall,1.,1.05)
        self.assertTrue(info['local_replan'])
        self.assertNotEqual(info['command_reason'],'LIDAR_TRACK')
        samples,_=self.guard.command_envelope(np.zeros(3),np.zeros(3),command,.05)
        self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)
        self.assertTrue(self.guard.command_constraint(samples))

    def setUp(self):
        self.guard=ExecutionGuard()
        self.nav=LidarNavigator(self.guard,budget=.5)
        self.xy=lambda s:(s,0.,0.)
        self.center=lambda s:0.
        self.car=np.array([(x,y,z) for x in np.arange(7.,11.1,.5)
                          for y in np.arange(-.75,.76,.25) for z in np.arange(-.75,.76,.25)])

    def test_unclassified_point_obstacle_produces_swept_detour(self):
        self.guard._update(self.car,1.,1.05,np.zeros(3))
        path,info=self.nav.path(np.zeros(3),0.,self.xy,self.center,None)
        self.assertIsNotNone(path)
        samples,_,_=swept_samples(path)
        self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)
        around=path[(path[:,0]>=7.)&(path[:,0]<=11.)]
        self.assertGreater(np.max(np.linalg.norm(around[:,1:],axis=1)),1.5)
        self.assertGreaterEqual(info['path_distance'],20.)

    def test_curved_inner_band_rejection_does_not_retry_same_xy_at_every_height(self):
        from velocity_response import VelocityResponse
        xy=lambda s:(20.*np.sin(s/20.),20.*(1.-np.cos(s/20.)))
        cloud=np.array([[*(np.array(xy(s))+y*np.array([-np.sin(s/20.),np.cos(s/20.)])),z]
                        for s in np.arange(8.,16.1,.5) for y in np.arange(-3.,.61,.3)
                        for z in np.arange(-2.,2.1,.4)])
        self.guard.response_model=VelocityResponse()
        self.guard._update(cloud,1.,1.05,np.zeros(3))
        path,info=self.nav.path(np.zeros(3),0.,xy,self.center,None)
        self.assertIsNotNone(path)
        self.assertLess(info['grid_attempts'],12)
        samples,_,_=swept_samples(path)
        projection=self.nav._projection(0.,xy)
        stations=self.nav._route_stations(samples,0.,xy,projection)
        road=np.column_stack([np.interp(stations,projection[0],projection[1][:,axis]) for axis in (0,1)])
        self.assertLessEqual(np.linalg.norm(samples[:,:2]-road,axis=1).max(),2.25+1e-8)
        self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)

    def test_blocked_first_forward_step_can_stage_sideways_with_certified_connector(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse(height_gain=2.)
        self.nav.budget=0.
        wall=np.array([[1.7,y,z] for y in np.arange(-.75,.76,.15)
                       for z in np.arange(-4.,4.01,.2)])
        command,info=self.nav.select(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),
            0.,self.xy,self.center,wall,1.,1.05)
        self.assertTrue(info['staging'])
        self.assertTrue(info['feasible'])
        path=np.array(info['path'])
        np.testing.assert_array_equal(path[0],np.zeros(3))
        self.assertAlmostEqual(path[1,0],0.)
        self.assertGreaterEqual(abs(path[1,1]),1.)
        samples,_,_=swept_samples(path)
        self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)
        actual,_,times=self.guard.response_model.envelope(np.zeros(3),np.zeros(3),command,.4)
        self.guard.envelope_times=times
        stations=self.nav._route_stations(actual,0.,self.xy)
        self.assertTrue(self.nav._floor_ok(actual,stations,self.center,None,timed=True))
        self.assertGreaterEqual(self.guard.index.distance(actual).min(),1.25)

    def test_safe_in_place_nodes_cannot_certify_a_completely_closed_road(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse(height_gain=2.)
        wall=np.array([[1.7,y,z] for y in np.arange(-4.,4.01,.2)
                       for z in np.arange(-4.,4.01,.2)])
        self.guard._update(wall,1.,1.05,np.zeros(3))
        path,info=self.nav.path(np.zeros(3),0.,self.xy,self.center,None)
        self.assertIsNone(path)
        self.assertGreaterEqual(info['grid_attempts'],2)

    def test_occupancy_cost_cannot_skip_the_first_search_attempt(self):
        self.nav.budget=0.
        self.guard._update(self.car,1.,1.05,np.zeros(3))
        path,info=self.nav.path(np.zeros(3),0.,self.xy,self.center,None)
        self.assertGreaterEqual(info['grid_attempts'],1)
        if path is not None:
            self.assertGreaterEqual(self.guard.index.distance(swept_samples(path)[0]).min(),1.25)

    def test_final_moving_command_uses_the_same_full_stopping_envelope(self):
        command,info=self.nav.select(np.zeros(3),np.array([2.,0.,0.]),np.array([4.,0.,0.]),
            0.,self.xy,self.center,self.car,1.,1.05)
        queries,extent=self.guard.command_envelope(np.zeros(3),np.array([2.,0.,0.]),command,.05)
        self.assertGreaterEqual(self.guard.index.distance(queries).min(),1.25)
        self.assertLessEqual(extent,30.)
        self.assertTrue(info['feasible'])

    def test_thin_wall_between_grid_stations_is_not_crossed(self):
        wall=np.array([(5.5,y,z) for y in np.arange(-5.,5.01,.25) for z in np.arange(-4.,4.01,.25)])
        self.guard._update(wall,1.,1.05,np.zeros(3))
        path,info=self.nav.path(np.zeros(3),0.,self.xy,self.center,None)
        if path is not None:
            self.assertLess(path[:,0].max(),5.5)
            self.assertGreaterEqual(self.guard.index.distance(swept_samples(path)[0]).min(),1.25)

    def test_stale_lidar_cannot_authorize_motion(self):
        command,info=self.nav.select(np.zeros(3),np.zeros(3),np.array([8.,0.,0.]),
            0.,self.xy,self.center,self.car,1.,2.)
        np.testing.assert_array_equal(command,np.zeros(3))
        self.assertEqual(info['command_reason'],'LIDAR_STALE')

    def test_clear_road_does_not_brake_to_land_on_grid_vertices(self):
        walls=np.array([(x,y,z) for x in np.arange(-5.,35.,1.)
                        for y in (-6.,6.) for z in (-3.,0.,3.)])
        command,info=self.nav.select(np.zeros(3),np.array([6.,0.,0.]),np.array([6.,0.,0.]),
            0.,self.xy,self.center,walls,1.,1.05)
        self.assertGreaterEqual(command[0],5.5)
        self.assertLess(abs(command[1]),.1)

    def test_zero_requested_speed_does_not_gain_forward_motion_from_lookaheads(self):
        from velocity_response import VelocityResponse
        walls=np.array([(x,y,z) for x in np.arange(-5.,35.,1.)
                        for y in (-6.,6.) for z in (-3.,0.,3.)])
        for coupled in (False,True):
            with self.subTest(coupled=coupled):
                self.guard.response_model=VelocityResponse() if coupled else None
                command,info=self.nav.select(np.zeros(3),np.zeros(3),np.zeros(3),
                    0.,self.xy,self.center,walls,1.,1.05)
                self.assertTrue(info['feasible'])
                np.testing.assert_allclose(command[:2],0.,atol=1e-12)

    def test_no_path_low_speed_height_recovery_checks_full_stop_before_replacing_profile(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse(lift_gain=.110,coupling_gains=(.09,.110,.13))
        position=np.array([0.,0.,.8])
        self.nav.braking_path=np.array([[0.,0.,.8],[5.,0.,.8]])
        walls=np.array([(x,y,z) for x in np.arange(-5.,35.,1.)
                        for y in (-6.,6.) for z in (-3.,0.,3.)])
        with patch.object(self.nav,'path',return_value=(None,{})):
            command,info=self.nav.select(position,np.zeros(3),np.array([5.,0.,0.]),
                0.,self.xy,self.center,walls,1.,1.05)
        self.assertEqual(info['command_reason'],'LIDAR_HEIGHT_RECOVERY')
        self.assertTrue(info['feasible'])
        self.assertLess(command[2],-.4)
        self.assertEqual(self.nav.braking_path[-1,2],0.)
        samples,_,_=self.guard.response_model.envelope(position,np.zeros(3),command,.4)
        self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)

    def test_safe_departure_path_does_not_extrapolate_stopping_height_below_floor(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse(lift_gain=.110,coupling_gains=(.09,.110,.13))
        position=np.array([0.,0.,-1.5]);center=lambda s:-2.5
        floor=np.array([(x,y,0.) for x in np.arange(-8.,30.,.3) for y in np.arange(-5.,5.,.3)])
        self.guard._update(floor,1.,1.05,position)
        path=np.array([position,[1.,0.,-1.3],[2.,0.,-1.5],[8.,0.,-2.]])
        self.assertGreater(self.nav._command_profile(path)[0,2],-1.25)
        projection=self.nav._projection(0.,self.xy)
        self.nav.reference_stations=projection[0];self.nav.reference_coordinates=projection[1]
        self.nav.reference_heights=np.full(len(projection[0]),-2.5)
        command,clearance,_=self.nav._commands(position,np.zeros(3),np.array([1.,0.,-.1]),
            path,.05,0.,self.xy,center,1.25,projection)
        self.assertIsNotNone(command)
        self.assertGreater(command[0],.1)
        self.assertGreaterEqual(clearance,1.25)
        self.assertLessEqual(self.guard.response_model.stop_profile.path[0,2],-1.27)
        samples,_,times=self.guard.response_model.envelope(position,np.zeros(3),command,.4)
        self.guard.envelope_times=times
        stations=self.nav._route_stations(samples,0.,self.xy,projection)
        self.assertTrue(self.nav._floor_ok(samples,stations,center,1.25,timed=True))
        self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)

    def test_height_reserve_keeps_full_stop_clearance_and_falls_back_under_low_roof(self):
        from velocity_response import VelocityResponse
        position=np.array([0.,0.,1.]);velocity=np.array([6.,0.,0.])
        path=np.column_stack((np.arange(25.),np.zeros(25),np.ones(25)))
        for low_roof in (False,True):
            with self.subTest(low_roof=low_roof):
                guard=ExecutionGuard()
                guard.response_model=VelocityResponse(lift_gain=.110,coupling_gains=(.09,.110,.13),height_gain=2.)
                guard.response_model.commit(velocity,velocity,1.)
                cloud=(np.array([(x,y,-.5) for x in np.arange(-5.,35.,.3)
                                 for y in np.arange(-5.,5.1,.3)]) if low_roof else
                       np.array([(x,y,z) for x in np.arange(-5.,35.,1.) for y in (-6.,6.) for z in (-3.,0.,3.)]))
                guard._update(cloud,1.,1.05,position);nav=LidarNavigator(guard)
                projection=nav._projection(0.,self.xy)
                nav.reference_stations=projection[0];nav.reference_coordinates=projection[1]
                nav.reference_heights=np.zeros(len(projection[0]))
                command,clearance,_=nav._commands(position,velocity,velocity,path,.05,
                    0.,self.xy,self.center,None,projection)
                self.assertIsNotNone(command)
                self.assertAlmostEqual(guard.response_model.stop_profile.path[0,2],1. if low_roof else .75)
                samples,_,times=guard.response_model.envelope(position,velocity,command,.4)
                guard.envelope_times=times
                self.assertGreaterEqual(guard.command_clearance(samples),1.25)
                self.assertTrue(nav._floor_ok(samples,nav._route_stations(samples,0.,self.xy),self.center,None,timed=True))

    def test_bypass_height_tracks_road_station_while_braking_keeps_actual_bypass(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse(height_gain=2.)
        projection=self.nav._projection(0.,self.xy)
        self.nav.reference_stations=projection[0];self.nav.reference_coordinates=projection[1]
        self.nav.reference_heights=-projection[0]
        path=np.array([[0.,2.,0.],[3.,-2.,-3.],[6.,2.,-6.]])
        position=np.array([[2.,2.,-2.]]);velocity=np.array([[1.,0.,0.]])
        original=self.nav._stop_profile(path,gain=2.)(position,velocity)[0]
        profile=self.nav._tracking_profile(path)
        self.assertGreater(original,1.)
        self.assertAlmostEqual(profile(position,velocity)[0],-1.)
        np.testing.assert_array_equal(profile.path,path)
        np.testing.assert_allclose(profile.height_path[:,1],0.)

    def test_height_extrapolation_cannot_cross_visible_surface_to_invent_reference(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse()
        roof=np.array([(x,y,1.5) for x in np.arange(-8.,30.,.3) for y in np.arange(-5.,5.,.3)])
        self.guard._update(roof,1.,1.05,np.zeros(3))
        path=np.array([[0.,0.,0.],[2.,0.,0.],[3.,0.,-.5],[8.,0.,-.5]])
        self.assertGreaterEqual(self.guard.index.distance(swept_samples(path)[0]).min(),1.25)
        projection=self.nav._projection(0.,self.xy)
        self.nav.reference_stations=projection[0];self.nav.reference_coordinates=projection[1]
        self.nav.reference_heights=np.zeros(len(projection[0]))
        profile=self.nav._bounded_command_profile(path,0.,self.xy,self.center,None,projection)
        self.assertEqual(profile[0,2],0.)

    def test_blocked_height_recovery_keeps_last_certified_stopping_profile(self):
        from velocity_response import VelocityResponse
        self.guard.response_model=VelocityResponse(lift_gain=.110,coupling_gains=(.09,.110,.13))
        position=np.array([0.,0.,.8])
        previous=np.array([[0.,0.,.8],[5.,0.,.8]])
        self.nav.braking_path=previous.copy()
        roof=np.array([(x,y,-.8) for x in np.arange(-8.,30.,.3) for y in np.arange(-5.,5.,.3)])
        with patch.object(self.nav,'path',return_value=(None,{})):
            _,info=self.nav.select(position,np.zeros(3),np.array([5.,0.,0.]),
                0.,self.xy,self.center,roof,1.,1.05)
        self.assertNotEqual(info['command_reason'],'LIDAR_HEIGHT_RECOVERY')
        np.testing.assert_array_equal(self.nav.braking_path,previous)

    def test_score_pruning_preserves_exhaustive_choice_with_static_and_moving_obstacles(self):
        from velocity_response import VelocityResponse
        from lidar_scene import LidarScene
        walls=np.array([(x,y,z) for x in np.arange(-5.,35.,1.)
                        for y in (-6.,6.) for z in (-3.,0.,3.)])
        wall=np.array([(11.,y,z) for y in np.arange(-5.,5.1,.3) for z in np.arange(-3.,3.1,.3)])
        for case in ('clear','wall','moving'):
            answers=[];counts=[]
            for pruning in (False,True):
                guard=ExecutionGuard();guard.response_model=VelocityResponse(lift_gain=.110,coupling_gains=(.09,.110,.13))
                guard.response_model.commit(np.array([6.,0.,0.]),np.array([6.,0.,0.]),1.)
                cloud=np.concatenate((walls,wall)) if case=='wall' else walls
                guard._update(cloud,1.,1.05,np.zeros(3))
                if case=='moving':
                    guard.dynamic_scene=LidarScene();guard.dynamic_scene.pose_stamp=1.05
                    guard.dynamic_scene.tracks=[dict(support=4,velocity=np.array([-2.,0.,0.]),
                        stamp=1.,center=np.array([14.,0.,0.]),half=np.array([.5,.75,.5]))]
                nav=LidarNavigator(guard);nav.prune_candidates=pruning
                projection=nav._projection(0.,self.xy)
                nav.reference_stations=projection[0];nav.reference_coordinates=projection[1]
                nav.reference_heights=np.zeros(len(projection[0]))
                path=np.column_stack((np.arange(25.),np.zeros((25,2))))
                with patch.object(guard.index,'distance',wraps=guard.index.distance) as queries:
                    result=nav._commands(np.zeros(3),np.array([6.,0.,0.]),np.array([6.,0.,0.]),
                        path,.05,0.,self.xy,self.center,None,projection)
                answers.append(result);counts.append(queries.call_count)
            with self.subTest(case=case):
                if answers[0][0] is None:self.assertIsNone(answers[1][0])
                else:np.testing.assert_array_equal(answers[0][0],answers[1][0])
                self.assertEqual(answers[0][1:],answers[1][1:])
                self.assertLessEqual(counts[1],counts[0])
                if case=='clear':self.assertLess(counts[1],counts[0])

    def test_shared_departure_floor_limits_path_and_emitted_xyz(self):
        command,info=self.nav.select(np.zeros(3),np.zeros(3),np.array([2.,0.,1.]),
            0.,self.xy,self.center,self.car,1.,1.05,floor_offset=.483)
        if info.get('path'):
            self.assertTrue(np.all(np.array(info['path'])[:,2]<=.483))
        if info.get('command_reason')!='LIDAR_ESCAPE':
            samples,_=self.guard.command_envelope(np.zeros(3),np.zeros(3),command,.05)
            self.assertLessEqual(samples[:,2].max(),.483)

    def test_async_route_snapshot_survives_cached_path_validation(self):
        self.nav=LidarNavigator(self.guard,budget=.5,async_planning=True)
        try:
            args=(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),0.,self.xy,self.center,self.car,1.,1.05)
            self.nav.select(*args)
            self.nav.future.result(timeout=3.)
            _,info=self.nav.select(*args)
            self.assertEqual(info['command_reason'],'LIDAR_TRACK')
            # This call submits with a cached path, then replaces local ss in
            # path validation. Background interpolation must keep its snapshot.
            self.nav.submitted_at=None
            self.nav.select(*args)
            path,_=self.nav.future.result(timeout=3.)
            self.assertIsNotNone(path)
            wall=np.array([(2.,y,z) for y in np.arange(-5.,5.01,.5) for z in np.arange(-4.,4.01,.5)])
            command,info=self.nav.select(*args[:6],wall,1.1,1.15)
            samples,_=self.guard.command_envelope(args[0],args[1],command,.05)
            self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)
        finally:self.nav.close()

    def test_background_failure_does_not_kill_control(self):
        self.nav=LidarNavigator(self.guard,async_planning=True)
        try:
            self.nav.future=Future();self.nav.future.set_exception(ValueError('failed snapshot'))
            _,info=self.nav.select(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),
                0.,self.xy,self.center,self.car,1.,1.05)
            self.assertEqual(info['planner_error'],'failed snapshot')
        finally:self.nav.close()

    def test_cached_path_and_command_share_floor_station_with_offset_start(self):
        self.nav=LidarNavigator(self.guard,async_planning=True)
        try:
            center=lambda s:-.1*max(0.,s)
            position=np.array([-.8,0.,.4])
            self.nav.last_path=np.array([position,*[(t,0.,center(t)+.4) for t in range(1,25)]])
            self.nav.last_plan_stamp=1.;self.nav.future=Future()
            points=np.array([(x,y,z) for x in np.arange(-5.,35.,1.) for y in (-6.,6.) for z in (-3.,0.,3.)])
            command,info=self.nav.select(position,np.zeros(3),np.array([2.,0.,-.2]),
                0.,self.xy,center,points,1.,1.05,.45)
            self.assertTrue(info.get('cached'))
            self.assertTrue(info['feasible'])
            self.assertGreater(command[0],.1)
        finally:self.nav.close()

    def test_close_pass_geometry_side_vs_closing(self):
        guard=ExecutionGuard();guard._update(np.array([[1.,.95,0.]]),1.,1.05,np.zeros(3))
        side=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.],[3.,0.,0.]])
        self.assertTrue(guard.close_pass_ok(side,.9))
        guard._update(np.array([[4.,0.,0.]]),1.1,1.15,np.zeros(3))
        closing=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
        self.assertFalse(guard.close_pass_ok(closing,.9))
        guard._update(np.array([[1.5,.05,0.]]),1.2,1.25,np.zeros(3))
        self.assertFalse(guard.close_pass_ok(side,.9))

    def test_side_buffer_admits_only_non_closing_raw_minimum(self):
        self.guard.response_model=None
        path=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
        self.guard.command_clearance_components=lambda samples,limit=None:(1.0,float('inf'))
        closed={'value':False}
        self.guard.close_pass_ok=lambda samples,floor: closed['value']
        nav=LidarNavigator(self.guard,budget=.5);nav.side_buffer=.9
        with patch.object(nav,'path',return_value=(path,{})):
            _,info=nav.select(np.zeros(3),np.zeros(3),np.array([2.,0.,0.]),
                0.,self.xy,self.center,self.car,1.,1.05)
        self.assertNotEqual(info['command_reason'],'LIDAR_TRACK')
        closed['value']=True
        nav=LidarNavigator(self.guard,budget=.5);nav.side_buffer=.9
        with patch.object(nav,'path',return_value=(path,{})):
            _,info=nav.select(np.zeros(3),np.zeros(3),np.array([2.,0.,0.]),
                0.,self.xy,self.center,self.car,1.,1.05)
        self.assertEqual(info['command_reason'],'LIDAR_TRACK')
        # Moving predictions keep the full buffer even for a side pass.
        self.guard.command_clearance_components=lambda samples,limit=None:(1.0,1.1)
        nav=LidarNavigator(self.guard,budget=.5);nav.side_buffer=.9
        with patch.object(nav,'path',return_value=(path,{})):
            _,info=nav.select(np.zeros(3),np.zeros(3),np.array([2.,0.,0.]),
                0.,self.xy,self.center,self.car,1.,1.05)
        self.assertNotEqual(info['command_reason'],'LIDAR_TRACK')

    def test_envelope_margin_admits_model_lag_against_raw_returns_only(self):
        self.guard.response_model=None
        path=np.array([[0.,0.,0.],[1.,0.,0.],[2.,0.,0.]])
        self.guard.command_clearance=lambda samples,limit=None:1.2
        self.guard.command_clearance_components=lambda samples,limit=None:(1.2,float('inf'))
        nav=LidarNavigator(self.guard,budget=.5)
        with patch.object(nav,'path',return_value=(path,{})):
            _,info=nav.select(np.zeros(3),np.zeros(3),np.array([2.,0.,0.]),
                0.,self.xy,self.center,self.car,1.,1.05)
        self.assertNotEqual(info['command_reason'],'LIDAR_TRACK')
        nav=LidarNavigator(self.guard,budget=.5);nav.envelope_margin=1.15
        with patch.object(nav,'path',return_value=(path,{})):
            _,info=nav.select(np.zeros(3),np.zeros(3),np.array([2.,0.,0.]),
                0.,self.xy,self.center,self.car,1.,1.05)
        self.assertEqual(info['command_reason'],'LIDAR_TRACK')
        # Moving predictions retain the full buffer.
        self.guard.command_clearance_components=lambda samples,limit=None:(1.2,1.0)
        nav=LidarNavigator(self.guard,budget=.5);nav.envelope_margin=1.15
        with patch.object(nav,'path',return_value=(path,{})):
            _,info=nav.select(np.zeros(3),np.zeros(3),np.array([2.,0.,0.]),
                0.,self.xy,self.center,self.car,1.,1.05)
        self.assertNotEqual(info['command_reason'],'LIDAR_TRACK')


if __name__=='__main__':unittest.main()
