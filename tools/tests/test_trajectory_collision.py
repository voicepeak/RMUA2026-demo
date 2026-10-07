from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from spacetime_scenarios import known_snapshot,obstacle,set_box
from trajectory_types import ResponseState,ResponseTrace,readonly
from trajectory_collision import TrajectoryCollision,CollisionConfig,VoxelBoxIndex
from dynamic_tracker import TrackerConfig
from local_occupancy import UNKNOWN,OCCUPIED


def linear_trace(snapshot,path,times=None,scenario_path=None):
    count=len(snapshot.response_state.position);path=np.asarray(path,dtype=float)
    positions=np.tile(path[:,None,:],(1,count,1))
    if scenario_path is not None:positions[:,scenario_path[0],:]=scenario_path[1]
    times=snapshot.pose_stamp+np.arange(len(path))*.1 if times is None else np.asarray(times)
    velocities=np.zeros_like(positions)
    if len(path)>1:
        velocities[:-1]=np.diff(positions,axis=0)/np.diff(times)[:,None,None];velocities[-1]=velocities[-2]
    zero=np.zeros((count,3));end=ResponseState(times[-1],snapshot.response_state.model_key,positions[-1],velocities[-1],zero,zero,
        np.full(count,times[-1]),np.full(count,times[-1]))
    return ResponseTrace(times,positions,velocities,np.zeros_like(positions),np.zeros_like(positions),end,
                         np.zeros((len(times)-1,count,3)))


class TrajectoryCollisionTests(unittest.TestCase):
    def config(self,**options):
        defaults=dict(body_half_extent=(.1,.1,.1),margin=0.,max_horizontal_speed=30.,max_vertical_speed=20.,max_acceleration=(100.,100.,100.))
        defaults.update(options);return CollisionConfig(**defaults)

    def test_full_swept_box_hits_a_wall_between_clear_endpoints(self):
        snapshot,_=known_snapshot();snapshot=set_box(snapshot,[2.,-.5,-.5],[2.1,.5,.5],OCCUPIED)
        trace=linear_trace(snapshot,[[1.,0.,0.],[3.,0.,0.]])
        self.assertEqual(snapshot.occupancy.query([[1.,0.,0.],[3.,0.,0.]],'static').tolist(),[1,1])
        self.assertEqual(TrajectoryCollision(snapshot,self.config()).check(trace).reason,'STATIC_COLLISION')

    def test_unknown_is_hard_rejected_interior_and_beyond_window(self):
        snapshot,_=known_snapshot();snapshot=set_box(snapshot,[2.,-.5,-.5],[2.1,.5,.5],UNKNOWN)
        check=TrajectoryCollision(snapshot,self.config()).check(linear_trace(snapshot,[[1.,0.,0.],[3.,0.,0.]]))
        self.assertEqual(check.reason,'UNKNOWN_BLOCKED')
        snapshot,_=known_snapshot()
        self.assertEqual(TrajectoryCollision(snapshot,self.config()).check(linear_trace(snapshot,[[24.,0.,0.],[26.,0.,0.]])).reason,'UNKNOWN_BLOCKED')

    def test_body_footprint_hits_adjacent_voxel_while_center_is_free(self):
        snapshot,_=known_snapshot();snapshot=set_box(snapshot,[1.,.5,0.],[1.1,.6,.1],OCCUPIED)
        trace=linear_trace(snapshot,[[1.1,.4,.1],[1.2,.4,.1]])
        self.assertTrue(TrajectoryCollision(snapshot,self.config(body_half_extent=(0.,0.,0.))).check(trace).safe)
        self.assertFalse(TrajectoryCollision(snapshot,self.config(body_half_extent=(.2,.2,.2))).check(trace).safe)

    def test_fast_crossing_vehicle_collision_between_temporally_clear_samples(self):
        vehicle=obstacle([2.,-2.,0.],[0.,40.,0.],size=(.2,.2,.2))
        snapshot,_=known_snapshot([vehicle])
        trace=linear_trace(snapshot,[[2.,0.,0.],[2.,0.,0.]])
        check=TrajectoryCollision(snapshot,self.config()).check(trace)
        self.assertEqual(check.reason,'DYNAMIC_COLLISION');self.assertEqual(check.track_id,1)
        self.assertGreater(check.stamp,100.);self.assertLess(check.stamp,100.1)

    def test_vehicle_passed_by_future_arrival_and_cloud_age_counted_once(self):
        vehicle=obstacle([2.,0.,0.],[0.,4.,0.],stamp=99.8)
        snapshot,_=known_snapshot([vehicle]);trace=linear_trace(snapshot,[[2.,0.,0.],[2.,0.,0.]],times=[100.4,100.5])
        self.assertTrue(TrajectoryCollision(snapshot,self.config()).check(trace).safe)
        # 0.2s from track snapshot gives y=.8, not y=0 or y=1.6.
        now=linear_trace(snapshot,[[2.,.8,0.],[2.,.8,0.]])
        self.assertEqual(TrajectoryCollision(snapshot,self.config()).check(now).reason,'DYNAMIC_COLLISION')

    def test_stationary_tentative_and_coasting_tracks_are_not_ignored(self):
        for status in ('TENTATIVE','CONFIRMED','COASTING'):
            snapshot,_=known_snapshot([obstacle([2.,0.,0.],status=status)])
            trace=linear_trace(snapshot,[[2.,0.,0.],[2.1,0.,0.]])
            self.assertEqual(TrajectoryCollision(snapshot,self.config()).check(trace).reason,'DYNAMIC_COLLISION')

    def test_uncertainty_growth_rejects_nominally_separate_vehicle(self):
        config=TrackerConfig(acceleration_std=.5,base_margin=0.,uncertainty_sigma=2.)
        vehicle=obstacle([2.,3.,0.],config=config)
        snapshot,_=known_snapshot([vehicle]);checker=TrajectoryCollision(snapshot,self.config())
        self.assertTrue(checker.check(linear_trace(snapshot,[[2.,0.,0.],[2.,0.,0.]])).safe)
        future=linear_trace(snapshot,[[2.,0.,0.],[2.,0.,0.]],times=[103.,103.1])
        self.assertEqual(checker.check(future).reason,'DYNAMIC_COLLISION')

    def test_only_one_response_scenario_collides_and_is_reported(self):
        snapshot,_=known_snapshot([obstacle([2.,1.,0.],size=(.2,.2,.2))])
        trace=linear_trace(snapshot,[[2.,0.,0.],[2.1,0.,0.]],scenario_path=(2,[[2.,1.,0.],[2.1,1.,0.]]))
        result=TrajectoryCollision(snapshot,self.config()).check(trace)
        self.assertEqual(result.reason,'DYNAMIC_COLLISION');self.assertEqual(result.scenario,2)

    def test_road_floor_and_dynamics_are_shared_hard_constraints(self):
        snapshot,_=known_snapshot(lateral_limit=.5)
        trace=linear_trace(snapshot,[[1.,.45,0.],[1.1,.45,0.]])
        self.assertEqual(TrajectoryCollision(snapshot,self.config()).check(trace).reason,'ROAD_BOUNDARY')
        snapshot,_=known_snapshot();trace=linear_trace(snapshot,[[1.,0.,0.],[2.,0.,0.]])
        self.assertEqual(TrajectoryCollision(snapshot,self.config(max_horizontal_speed=2.)).check(trace).reason,'DYNAMICS_LIMIT')

    def test_stale_or_inconsistent_scene_and_missing_dynamic_owner_fail_closed(self):
        snapshot,_=known_snapshot()
        self.assertEqual(TrajectoryCollision(snapshot,now=100.6).freshness().reason,'SENSOR_STALE')
        self.assertEqual(TrajectoryCollision(snapshot,now=100.1).freshness().reason,'SCENE_TIME_MISMATCH')
        owners=snapshot.occupancy.dynamic_owners.copy();owners[10,8,6]=7
        snapshot=replace(snapshot,occupancy=replace(snapshot.occupancy,dynamic_owners=readonly(owners,dtype=np.int64)))
        self.assertEqual(TrajectoryCollision(snapshot).freshness().reason,'MISSING_TRACK')

    def test_outdated_or_future_track_timestamps_are_rejected(self):
        for stamp,expected in ((98.,'TRACK_STALE'),(101.,'TRACK_TIME_MISMATCH')):
            snapshot,_=known_snapshot([obstacle([10.,0.,0.],stamp=stamp)])
            self.assertEqual(TrajectoryCollision(snapshot).freshness().reason,expected)

    def test_integral_volume_queries_match_public_map_queries(self):
        snapshot,_=known_snapshot();snapshot=set_box(snapshot,[2.,-.5,-.5],[3.,.5,.5],OCCUPIED)
        snapshot=set_box(snapshot,[5.,0.,0.],[6.,1.,1.],UNKNOWN)
        rng=np.random.default_rng(42);low=rng.uniform([-7.,-5.,-4.],[26.,5.,4.],(100,3));high=low+rng.uniform(0.,1.,(100,3))
        index=VoxelBoxIndex(snapshot.occupancy)
        np.testing.assert_array_equal(index.query(low,high),snapshot.occupancy.query_boxes(low,high,'static'))

    def test_velocity_change_invalidates_prior_certified_motion(self):
        vehicle=obstacle([3.,3.,0.],[0.,1.,0.])
        snapshot,_=known_snapshot([vehicle]);trace=linear_trace(snapshot,[[3.,0.,0.],[3.,0.,0.]],times=[101.,101.1])
        self.assertTrue(TrajectoryCollision(snapshot,self.config()).check(trace).safe)
        changed=replace(snapshot,obstacles=(obstacle([3.,3.,0.],[0.,-3.,0.]),))
        self.assertEqual(TrajectoryCollision(changed,self.config()).check(trace).reason,'DYNAMIC_COLLISION')

    def test_batch_prediction_matches_full_covariance_for_correlated_states(self):
        rng=np.random.default_rng(3);matrix=rng.normal(size=(6,6));covariance=matrix@matrix.T
        vehicle=replace(obstacle([3.,2.,0.],[1.,-2.,.1],config=TrackerConfig()),covariance=readonly(covariance),truncated=True)
        stamps=np.array([100.,100.2,101.,104.])
        positions,uncertainties=vehicle.predict_arrays(stamps)
        for i,stamp in enumerate(stamps):
            prediction=vehicle.predict_at(stamp)
            np.testing.assert_allclose(positions[i],prediction.position,atol=1e-12)
            np.testing.assert_allclose(uncertainties[i],prediction.uncertainty,atol=1e-12)
        with self.assertRaises(ValueError):positions[0,0]=5.
        with self.assertRaises(ValueError):vehicle.predict_arrays([99.])


if __name__=='__main__':unittest.main()
