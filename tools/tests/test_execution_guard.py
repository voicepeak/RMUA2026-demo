import sys
import unittest
from pathlib import Path
import numpy as np
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from execution_guard import ExecutionGuard
from predictive_avoidance import Detour, JoinedDetour, PredictiveAvoidance
from lattice_detour import LatticeDetour, search
from point_index import PointIndex, measured_car_faces
from reference_planner import RouteGeometry
from path_sampling import swept_samples


class ExecutionGuardTests(unittest.TestCase):
    def test_narrow_certified_shift_gap_between_half_meter_grid_cells(self):
        walls=np.array([(x,y,z) for x in np.arange(-2.,8.1,.25)
                        for y in (-.9,2.35) for z in np.arange(-3.,3.1,.25)])
        path=np.array([(x,0.,0.) for x in np.arange(0.,4.1,.2)])
        result=ExecutionGuard().find_shift(np.zeros(3),np.zeros(3),np.zeros(3),
                         np.array([0.,1.,0.]),walls,10.,10.1,forward_path=path)
        self.assertIsNotNone(result)
        target,_=result
        self.assertGreater(target[1],.5);self.assertLess(target[1],1.)

    def test_measured_road_ceiling_restores_the_unseen_floor_bound(self):
        cloud=self.roof_cloud()
        index=PointIndex(cloud,origin=np.zeros(3),road_height=5.)
        self.assertAlmostEqual(index.floor_limit([[0.,0.,0.]],1.25)[0],1.75,places=5)
        self.assertTrue(np.isnan(index.floor_limit([[20.,0.,0.]],1.25)[0]))
        self.assertLess(index.distance([[0.,0.,4.]])[0],0.)
        command,info=self.guard.filter_command(np.zeros(3),np.zeros(3),np.array([0.,0.,4.]),cloud,10.,10.1)
        self.assertLess(command[2],2.)
        self.assertGreaterEqual(info['command_clearance'],1.25)

    def test_shift_cannot_approach_close_floor_while_leaving_ceiling(self):
        cloud=self.roof_cloud()
        position=np.array([0.,0.,1.9])
        self.assertIsNone(ExecutionGuard().escape(position,np.zeros(3),np.array([0.,0.,2.5]),cloud,10.,10.1))
        self.assertIsNotNone(ExecutionGuard().escape(position,np.zeros(3),np.array([0.,0.,1.]),cloud,10.,10.1))

    def test_retreat_requires_an_accepted_complete_new_corridor(self):
        verified=dict(execution_verified=True,verified_distance=30.,execution_clearance=1.5)
        plan=dict(source='PREDICTIVE',feasible=True,join_accepted=True)
        self.assertTrue(ExecutionGuard.resume_after_retreat(verified,plan))
        for bad in (dict(verified,verified_distance=4.),dict(verified,execution_clearance=1.2)):
            self.assertFalse(ExecutionGuard.resume_after_retreat(bad,plan))
        for bad in (dict(plan,source='LIDAR_RECOVERY'),dict(plan,join_accepted=False)):
            self.assertFalse(ExecutionGuard.resume_after_retreat(verified,bad))

    def test_shift_target_reserves_clearance_for_measured_tracking_tolerance(self):
        cloud=np.array([[.8,0.,0.]])
        self.assertIsNone(ExecutionGuard().escape(np.zeros(3),np.zeros(3),np.array([-.5,0.,0.]),cloud,10.,10.1))
        self.assertIsNotNone(ExecutionGuard().escape(np.zeros(3),np.zeros(3),np.array([-1.,0.,0.]),cloud,10.,10.1))

    def test_blocked_side_shifts_can_retreat_to_restore_turning_room(self):
        wall=np.array([(.8,y,z) for y in np.arange(-5.,5.1,.5) for z in np.arange(-3.,3.1,.5)])
        path=np.array([(x,0.,0.) for x in np.arange(0.,8.1,.2)])
        result=ExecutionGuard().find_shift(np.zeros(3),np.zeros(3),np.zeros(3),
                         np.array([0.,1.,0.]),wall,10.,10.1,forward_path=path)
        self.assertIsNotNone(result)
        target,command=result
        self.assertLess(target[0],0.);self.assertLess(command[0],0.)
        self.assertGreaterEqual(np.min(np.linalg.norm(wall-target,axis=1)),1.25)

    def car_face(self,x=8.):
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        track=dict(world=np.array([x,0.,0.]),stamp=10.,uncertainty=.35,half_width=1.,half_height=.8)
        return measured_car_faces([track],route,10.1)

    def test_fresh_car_face_stops_commands_through_unsampled_body(self):
        cloud=np.array([[50.,0.,0.]])
        desired=np.array([8.,0.,0.])
        clear,_=self.guard.filter_command(np.zeros(3),np.zeros(3),desired,cloud,10.,10.1)
        self.assertEqual(clear[0],8.)
        self.guard.set_faces(self.car_face())
        command,info=self.guard.filter_command(np.zeros(3),np.zeros(3),desired,cloud,10.,10.1)
        self.assertLess(command[0],8.)
        self.assertGreaterEqual(info['command_clearance'],self.guard.margin+.1)
        self.assertEqual(PointIndex([],faces=self.car_face()).distance([[8.,0.,0.]])[0],0.)

    def test_uncertain_or_stale_stereo_faces_do_not_seal_corridors(self):
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        track=dict(world=np.array([8.,0.,0.]),stamp=10.,uncertainty=2.,half_width=1.,half_height=.8)
        self.assertFalse(measured_car_faces([track],route,10.1))
        track['uncertainty']=.35
        self.assertTrue(measured_car_faces([track],route,10.6))
        self.assertFalse(measured_car_faces([track],route,12.1))
        self.assertFalse(measured_car_faces([track],route,9.9))

    def test_shift_cannot_enter_car_face_hidden_between_laser_returns(self):
        self.guard.set_faces(self.car_face(x=2.))
        cloud=np.array([[50.,0.,0.]])
        self.assertIsNone(self.guard.escape(np.zeros(3),np.zeros(3),np.array([1.5,0.,0.]),cloud,10.,10.1))
        self.assertIsNotNone(self.guard.escape(np.zeros(3),np.zeros(3),np.array([-1.,0.,0.]),cloud,10.,10.1))

    def test_nearest_clearance_matches_every_point_across_tree_seeds(self):
        rng=np.random.default_rng(16)
        points=rng.normal(size=(600,3));queries=rng.normal(size=(80,3))
        expected=np.min(np.linalg.norm(points[None,:,:]-queries[:,None,:],axis=2),axis=1)
        for seed in range(16):
            cv2.setRNGSeed(seed)
            np.testing.assert_allclose(PointIndex(points).distance(queries),expected,atol=1e-6)

    def test_road_car_detour_reserves_vertical_tracking_clearance(self):
        cloud=np.array([(x,y,z) for x in np.arange(12.,16.1,.5)
                        for y in np.arange(-.8,.81,.4) for z in (-.3,0.,.3)])
        planner=PredictiveAvoidance(margin=1.15,budget=3.,vertical_limit=.5)
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        info=planner.evaluate(0.,np.zeros(3),np.zeros(3),lambda s:(s,0.,0.),
                              lambda s:0.,route,cloud,[],10.,8.)
        self.assertTrue(info['feasible'])
        offsets=planner.plan.offsets(np.arange(0.,60.,.1))
        self.assertLessEqual(np.max(abs(offsets[:,1])),.5+1e-6)
        self.assertGreater(np.max(abs(offsets[:,0])),1.)

    def roof_cloud(self):
        return np.array([(x,y,-2.) for x in np.arange(-8.,8.1,1.)
                         for y in np.arange(-5.,5.1,1.) if abs(x)>3. or abs(y)>3.])

    def test_ceiling_between_laser_returns_limits_climb(self):
        cloud=self.roof_cloud()
        self.assertGreater(PointIndex(cloud).distance([[0.,0.,-2.]])[0],3.)
        index=PointIndex(cloud,origin=np.zeros(3))
        self.assertIsNotNone(index.roof)
        self.assertAlmostEqual(index.distance([[0.,0.,-2.]])[0],0.,places=5)
        command,info=self.guard.filter_command(np.zeros(3),np.zeros(3),np.array([0.,0.,-4.]),cloud,10.,10.1)
        self.assertLess(abs(command[2]),1.)
        self.assertGreaterEqual(info['command_clearance'],self.guard.margin+.1)

    def test_ceiling_patch_does_not_extend_outside_measured_support(self):
        index=PointIndex(self.roof_cloud(),origin=np.zeros(3))
        self.assertTrue(np.isinf(index.surface_distance([[20.,0.,-2.]])[0]))
        sparse=PointIndex([[x,0.,-2.] for x in range(-9,10)],origin=np.zeros(3))
        self.assertIsNone(sparse.roof)

    def test_escape_must_not_enter_unseen_ceiling_between_returns(self):
        cloud=self.roof_cloud()
        self.assertIsNone(self.guard.escape(np.zeros(3),np.zeros(3),np.array([0.,0.,-1.5]),cloud,10.,10.1))
        self.assertIsNotNone(self.guard.escape(np.zeros(3),np.zeros(3),np.array([0.,0.,.6]),cloud,10.,10.1))

    def setUp(self):
        self.guard=ExecutionGuard()
        self.xy=lambda s:(s,0.,0.)

    def check(self,points=(),plan=None,cloud=10.,pose=10.1,velocity=(0.,0.,0.),position=(0.,0.,0.)):
        return self.guard.evaluate(0.,np.array(position),np.array(velocity),self.xy,lambda s:0.,
                                   plan,np.array(points).reshape(-1,3),cloud,pose)

    def test_old_plan_can_continue_only_after_latest_geometry_validation(self):
        plan=Detour(-20.,(0.,0.),(2.,0.),10.,20.)
        safe=self.check([(12.,0.,0.)],plan=plan,position=(0.,2.,0.))
        self.assertTrue(safe['execution_verified'])
        self.assertGreater(safe['cap'],10.)
        self.guard=ExecutionGuard()
        blocked=self.check([(1.,2.,0.)],plan=plan,position=(0.,2.,0.))
        self.assertEqual(blocked['cap'],0.)

    def test_stale_missing_or_future_cloud_does_not_authorize_motion(self):
        for cloud in (None,9.,11.):
            result=self.check(cloud=cloud)
            self.assertEqual(result['cap'],0.)
            self.assertEqual(result['guard_reason'],'LIDAR_STALE')

    def test_empty_cloud_is_not_a_measured_free_corridor(self):
        result=self.check()
        self.assertEqual(result['cap'],0.)
        self.assertEqual(result['guard_reason'],'LIDAR_EMPTY')

    def test_braking_begins_before_obstacle_without_unnecessary_zero(self):
        result=self.check([(15.,0.,0.)])
        self.assertEqual(result['guard_reason'],'BRAKING_OBSTACLE')
        self.assertGreater(result['cap'],0.)
        d=result['verified_distance'];v=result['cap'];latency=.45
        self.assertLessEqual(v*latency+v*v/(2.*self.guard.braking),d+1e-6)

    def test_actual_tracking_connector_is_certified(self):
        self.assertEqual(self.check([(0.,1.8,0.)],position=(0.,2.,0.))['cap'],0.)

    def test_measured_response_segment_cannot_cross_surface(self):
        result=self.check([(0.,2.,0.)],velocity=(0.,8.,0.))
        self.assertEqual(result['cap'],0.)

    def test_brakes_for_car_on_actual_motion_beyond_old_response_segment(self):
        result=self.check([(0.,10.,0.)],velocity=(0.,8.,0.))
        self.assertEqual(result['guard_reason'],'BRAKING_MOTION')
        self.assertGreater(result['cap'],0.)
        self.assertLess(result['motion_speed_cap'],8.)
        self.assertGreater(result['measured_stopping_distance'],10.)
        self.assertGreater(result['response_clearance'],self.guard.margin+.1)

    def test_safe_steering_is_not_capped_by_a_different_coasting_ray(self):
        cloud=np.array([[0.,10.,0.]])
        result=self.check(cloud,velocity=(0.,8.,0.))
        self.assertGreater(result['cap'],8.)
        command,info=self.guard.filter_command(np.zeros(3),np.array([0.,8.,0.]),
                                              np.array([8.,0.,0.]),cloud,10.,10.1)
        self.assertGreater(command[0],0.)
        self.assertGreaterEqual(info['command_clearance'],self.guard.margin+.1)

    def test_final_xyz_command_cannot_climb_into_car_after_horizontal_stop(self):
        cloud=np.array([[0.,0.,-2.]])
        command,info=self.guard.filter_command(np.zeros(3),np.zeros(3),np.array([0.,0.,-4.]),cloud,10.,10.1)
        self.assertLess(info['command_scale'],1.)
        travel=command*(.45+np.linalg.norm(command)/(2.*self.guard.braking))
        self.assertGreaterEqual(np.linalg.norm(travel-cloud[0]),self.guard.margin+.1)

    def test_blocked_start_stops_vertical_and_horizontal_commands_together(self):
        command,info=self.guard.filter_command(np.zeros(3),np.zeros(3),np.array([3.,0.,-2.]),
                                               np.array([[.5,0.,0.]]),10.,10.1)
        np.testing.assert_array_equal(command,[0.,0.,0.])
        self.assertEqual(info['command_reason'],'COMMAND_BLOCKED')

    def test_certified_shift_can_exit_existing_buffer_without_approaching_surface(self):
        cloud=np.array([[.8,0.,0.],[5.,0.,0.]])
        command=self.guard.escape(np.zeros(3),np.zeros(3),np.array([-1.,0.,0.]),cloud,10.,10.1)
        self.assertIsNotNone(command)
        self.assertLess(command[0],0.)
        self.assertLessEqual(np.linalg.norm(command),.6)
        continuing=self.guard.escape(np.zeros(3),np.array([-.5,0.,0.]),np.array([-1.,0.,0.]),cloud,10.,10.1)
        self.assertIsNotNone(continuing)
        self.assertIsNone(self.guard.escape(np.zeros(3),np.zeros(3),np.array([2.,0.,0.]),cloud,10.,10.1))

    def test_shift_rechecks_raw_points_momentum_and_destination(self):
        for cloud,velocity in (([[.8,0.,0.],[-1.,0.,0.]],np.zeros(3)),
                               ([[.8,0.,0.]],np.array([.5,0.,0.])),
                               ([[.8,0.,0.]],np.array([.1,0.,0.]))):
            self.assertIsNone(self.guard.escape(np.zeros(3),velocity,np.array([-1.,0.,0.]),
                                               np.array(cloud),10.,10.1))

    def test_shift_selects_offset_that_clears_near_car_and_respects_floor(self):
        cloud=np.array([(x,y,z) for x in (4.,5.,6.) for y in (-.5,0.,.5) for z in (-.5,0.,.5)])
        path=np.column_stack([np.arange(0.,8.1,.15),np.zeros((54,2))])
        result=self.guard.find_shift(np.zeros(3),np.zeros(3),np.zeros(3),np.array([0.,1.,0.]),
                                    cloud,10.,10.1,forward_path=path,max_z=-.1)
        self.assertIsNotNone(result)
        target,command=result
        self.assertLessEqual(target[2],-.1)
        self.assertGreaterEqual(np.min(self.guard.index.distance(path+target)),self.guard.margin+.1)
        self.assertLessEqual(np.linalg.norm(command),.6)

    def test_measured_settling_delay_is_included_in_braking_budget(self):
        result=self.check([(3.,0.,0.)])
        v=result['cap'];latency=self.guard.reaction+.1
        stopping=max(v*v/(2.*self.guard.braking),v*self.guard.settling)
        self.assertLessEqual(v*latency+stopping,result['verified_distance']+1e-6)

    def test_stopping_models_are_not_added_twice_on_clear_straight_road(self):
        result=self.check([(10.,10.,0.)])
        self.assertGreater(result['cap'],13.)

    def test_fallback_zero_cap_cannot_freeze_different_certified_detour(self):
        guard=dict(cap=3.2,execution_verified=True)
        for info in (dict(source='LIDAR_RECOVERY',feasible=True,cap=0.),
                     dict(source='PREDICTIVE',feasible=True,join_accepted=False,cap=0.),
                     dict(source='PREDICTIVE',feasible=True,cap=0.)):
            self.assertEqual(self.guard.route_cap(guard,info),3.2)
        self.assertEqual(self.guard.route_cap(guard,dict(source='PREDICTIVE',feasible=True,
                                                       join_accepted=True,cap=2.)),2.)

    def test_zero_target_still_checks_full_measured_stop_and_counter_brakes(self):
        command,info=self.guard.filter_command(np.zeros(3),np.array([8.,0.,0.]),np.zeros(3),
                                               np.array([[12.,0.,0.]]),10.,10.1)
        self.assertEqual(info['command_reason'],'COUNTER_BRAKE')
        self.assertLess(command[0],0.)
        self.assertGreaterEqual(info['command_clearance'],self.guard.margin+.1)

    def test_vertical_correction_is_preserved_when_only_horizontal_command_is_blocked(self):
        command,info=self.guard.filter_command(np.zeros(3),np.zeros(3),np.array([8.,0.,-1.]),
                                               np.array([[3.,0.,0.]]),10.,10.1)
        self.assertEqual(info['command_reason'],'COMMAND_BRAKING')
        self.assertEqual(command[2],-1.)
        self.assertLess(command[0],8.)

    def test_roundoff_does_not_label_a_blocked_start_as_verified(self):
        s=51.2
        result=self.guard.evaluate(s,np.array([s,0.,0.]),np.zeros(3),self.xy,lambda t:0.,None,
                                   np.array([[s+1.4,0.,0.]]),10.,10.1)
        self.assertFalse(result['execution_verified'])
        self.assertEqual(result['guard_reason'],'EXECUTION_BLOCKED')
        self.assertEqual(result['cap'],0.)

    def test_delayed_plan_join_preserves_position_slope_and_acceleration(self):
        old=Detour(0.,(0.,0.),(2.,-1.),20.,30.)
        new=Detour(2.,old.offset(2.),(-2.,0.),20.,40.,start_slope=old.slope(2.))
        s=8.;accel=(old.slope(s+.1)-old.slope(s-.1))/.2
        joined=JoinedDetour(s,old.offset(s),old.slope(s),accel,new,8.)
        np.testing.assert_allclose(joined.offset(s),old.offset(s),atol=1e-10)
        np.testing.assert_allclose(joined.slope(s),old.slope(s),atol=1e-10)
        np.testing.assert_allclose(joined.offset(16.),new.offset(16.),atol=1e-10)
        np.testing.assert_allclose(joined.slope(16.),new.slope(16.),atol=1e-10)
        self.assertLess(np.linalg.norm(joined.slope(16.001)-joined.slope(15.999)),.001)

    def test_lattice_recenters_when_reachable_and_preserves_endpoint_slopes(self):
        stations=np.arange(16.)
        nominal=np.column_stack([stations,np.zeros((16,2))]);sides=np.tile([0.,1.,0.],(16,1))
        options=search(stations,nominal,sides,np.array([2.,1.]),PointIndex([]),1.,[],start_slope=(.12,-.08))
        for plan in options:
            np.testing.assert_allclose(plan.offset(15.),[0.,0.],atol=1e-10)
            np.testing.assert_allclose(plan.slope(15.),[0.,0.],atol=1e-10)
            np.testing.assert_allclose(plan.slope(0.),[.12,-.08],atol=1e-10)

    def test_finished_nonzero_lattice_is_returned_continuously(self):
        planner=PredictiveAvoidance()
        planner.plan=LatticeDetour([0.,5.],np.array([[1.,1.],[1.,1.]]))
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        result=planner.evaluate(6.,np.array([6.,1.,1.]),np.array([8.,0.,0.]),self.xy,lambda s:0.,
                                route,np.empty((0,3)),[],10.,8.)
        self.assertTrue(result['feasible'])
        np.testing.assert_allclose(planner.plan.offset(6.),[1.,1.],atol=1e-10)
        np.testing.assert_allclose(planner.plan.offset(45.),[0.,0.],atol=1e-10)

    def test_nonzero_terminal_is_retained_when_nominal_return_is_blocked(self):
        planner=PredictiveAvoidance()
        previous=LatticeDetour([0.,5.],np.array([[2.,0.],[2.,0.]]));planner.plan=previous
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        result=planner.evaluate(6.,np.array([6.,2.,0.]),np.array([8.,0.,0.]),self.xy,lambda s:0.,
                                route,np.array([[16.,0.,0.]]),[],10.,8.)
        self.assertTrue(result['feasible']);self.assertIs(planner.plan,previous)

    def test_shared_swept_samples_cover_short_and_long_connectors(self):
        path=np.array([[0.,0.,0.],[0.,2.,0.],[.2,2.,0.],[10.,2.,1.]])
        samples,segments,fraction=swept_samples(path)
        self.assertLessEqual(np.max(np.linalg.norm(np.diff(samples,axis=0),axis=1)),.18+1e-12)
        np.testing.assert_allclose(samples,path[segments]+fraction[:,None]*(path[segments+1]-path[segments]))

    def test_raw_dense_candidate_passes_the_execution_validator(self):
        # Offset surfaces inside the same voxel are all retained. The runtime
        # checker must agree with the planner's actual dense executable path.
        points=np.array([(x,y,z) for x in (10.,10.12,12.)
                         for y in np.arange(-.8,.9,.14) for z in np.arange(-.8,.9,.14)])
        planner=PredictiveAvoidance(margin=1.15,braking=4.)
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        result=planner.evaluate(0.,np.zeros(3),np.zeros(3),self.xy,lambda s:0.,route,points,[],10.,5.)
        self.assertTrue(result['feasible'])
        check=self.guard.evaluate(0.,np.zeros(3),np.zeros(3),self.xy,lambda s:0.,planner.plan,points,10.,10.1)
        self.assertEqual(check['guard_reason'],'EXECUTION_CLEAR')


if __name__=='__main__':unittest.main()
