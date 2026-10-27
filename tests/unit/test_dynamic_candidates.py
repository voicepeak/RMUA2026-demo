import unittest
from dataclasses import replace

from interfaces.contracts import Metadata, Trajectory, TrajectorySample
from planning.trajectory_validator.static_sweep import SweepBounds
from planning.trajectory_validator.dynamic_candidates import DynamicBounds, DynamicScene, check_dynamic_candidates
from world_model.dynamic_tracks.tracker import ObjectTrack
from tests.unit.test_static_candidates import scene
from planning.spacetime_local.static_candidates import plan_static_local


class DynamicCandidateTests(unittest.TestCase):
    def setUp(self):
        self.meta=Metadata(10.,10.5,0,'synthetic')
        self.body=SweepBounds((.1,.1,.1),0.,0.)
        self.bounds=DynamicBounds(0.,0.,0.,3.)
    def trajectory(self,a=(0.,0.,0.),b=(0.,0.,0.),end=10.4):
        return Trajectory(Metadata(10.,end,0,'trajectory'),tuple(TrajectorySample(t,p,(0.,0.,0.),(0.,0.,0.),0.) for t,p in ((10.,a),(end,b))))
    def track(self,center=(0.,-2.,0.),velocity=(0.,10.,0.),observations=2):
        return ObjectTrack(1,0,center,(.2,.2,.2),velocity,observations,10.,10.5)
    def test_detects_crossing_between_samples(self):
        result=check_dynamic_candidates(self.trajectory(),DynamicScene(self.meta,(self.track(),)),10.,0,self.body,self.bounds)
        self.assertFalse(result.allowed);self.assertEqual(result.reason,'DYNAMIC_CANDIDATE_CONFLICT')
        self.assertEqual(result.track_id,1);self.assertFalse(result.safety_certified)
    def test_no_collision_is_explicitly_uncertified(self):
        track=self.track(center=(10.,-2.,0.))
        result=check_dynamic_candidates(self.trajectory(),DynamicScene(self.meta,(track,)),10.,0,self.body,self.bounds)
        self.assertTrue(result.allowed);self.assertFalse(result.safety_certified)
        self.assertEqual(result.reason,'NO_DETECTED_DYNAMIC_CONFLICT_UNCERTIFIED')
    def test_tangent_contact_counts_as_conflict(self):
        track=self.track(center=(.3,0.,0.),velocity=(0.,0.,0.))
        result=check_dynamic_candidates(self.trajectory(),DynamicScene(self.meta,(track,)),10.,0,self.body,self.bounds)
        self.assertFalse(result.allowed);self.assertEqual(result.reason,'DYNAMIC_CANDIDATE_CONFLICT')

    def test_excess_pair_budget_is_not_silently_skipped(self):
        trajectory=self.trajectory();one=self.track(center=(10.,0.,0.))
        scene=DynamicScene(self.meta,(one,replace(one,track_id=2)))
        bounds=replace(self.bounds,max_pair_tests=1)
        self.assertEqual(check_dynamic_candidates(trajectory,scene,10.,0,self.body,bounds).reason,'DYNAMIC_BUDGET')

    def test_single_observation_expands_unknown_velocity(self):
        track=self.track(center=(0.,1.,0.),velocity=(0.,0.,0.),observations=1)
        result=check_dynamic_candidates(self.trajectory(),DynamicScene(self.meta,(track,)),10.,0,self.body,self.bounds)
        self.assertFalse(result.allowed);self.assertEqual(result.reason,'DYNAMIC_CANDIDATE_CONFLICT')
    def test_stale_wrong_epoch_and_horizon_reject(self):
        scene=DynamicScene(self.meta,(self.track(),))
        self.assertEqual(check_dynamic_candidates(self.trajectory(),scene,10.5,0,self.body,self.bounds).reason,'DYNAMIC_SOURCE_UNAVAILABLE')
        self.assertEqual(check_dynamic_candidates(self.trajectory(end=10.6),scene,10.,0,self.body,self.bounds).reason,'DYNAMIC_HORIZON')
        other=replace(self.track(),epoch=1)
        self.assertEqual(check_dynamic_candidates(self.trajectory(),DynamicScene(self.meta,(other,)),10.,0,self.body,self.bounds).reason,'DYNAMIC_TRACK_UNAVAILABLE')
    def test_unknown_or_empty_scene_never_claims_certification(self):
        empty=DynamicScene(self.meta,())
        result=check_dynamic_candidates(self.trajectory(),empty,10.,0,self.body,self.bounds)
        self.assertTrue(result.allowed);self.assertFalse(result.safety_certified)
        self.assertFalse(empty.coverage_verified)
        _,_,_,limits=scene()
        with self.assertRaises(ValueError):
            plan_static_local(*scene()[:2],(.501,.5,.5),scene()[2],10.,0,limits,dynamic_scene=empty)

    def test_planner_uses_veto_and_bounds_source_lease(self):
        state,initial,grid,limits=scene()
        limits=replace(limits,segment_durations_s=(.1,),lateral_offsets_m=(),speed_limit_m_s=1.,sweep=replace(limits.sweep,acceleration_bound_m_s2=2.))
        obstacle=self.track(center=(.5,.5,.5),velocity=(0.,0.,0.))
        result=plan_static_local(state,initial,(.501,.5,.5),grid,10.,0,limits,DynamicScene(self.meta,(obstacle,)),self.bounds)
        self.assertIsNone(result.trajectory);self.assertEqual(result.checks[0].reason,'DYNAMIC_CANDIDATE_CONFLICT')
        far=replace(obstacle,center_ned=(10.,10.,10.),valid_until=10.04)
        result=plan_static_local(state,initial,(.501,.5,.5),grid,10.,0,limits,DynamicScene(self.meta,(far,)),self.bounds)
        self.assertIsNotNone(result.trajectory);self.assertLessEqual(result.trajectory.metadata.valid_until,10.04)
        self.assertEqual(result.checks[0].reason,'NO_DETECTED_DYNAMIC_CONFLICT_UNCERTIFIED')
        self.assertEqual(result.source_dynamic,self.meta);self.assertFalse(result.dynamic_coverage_certified)


if __name__=='__main__':unittest.main()
