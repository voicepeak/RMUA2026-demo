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
