from dataclasses import replace
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from spacetime_scenarios import known_snapshot,obstacle,cases,set_box,advance_oracle
from st_lattice import STLattice,LatticeConfig
from trajectory_collision import TrajectoryCollision
from trajectory_types import readonly
from local_occupancy import UNKNOWN,OCCUPIED


class STLatticeTests(unittest.TestCase):
    def planner(self,parameters,**options):
        config=dict(budget=2.,forward_distance=2.,height_step=0.,lateral_rate=0.,max_expansions=100)
        config.update(options);return STLattice(parameters,LatticeConfig(**config))

    def assert_certified(self,snapshot,result):
        self.assertIsNotNone(result.trajectory)
        trajectory=result.trajectory;checker=TrajectoryCollision(snapshot)
        self.assertTrue(checker.check(trajectory.trace).safe)
        self.assertTrue(checker.check(trajectory.backup).safe)
        self.assertAlmostEqual(trajectory.valid_until,trajectory.backup.times[0])
        self.assertEqual(trajectory.epoch,snapshot.epoch)
        self.assertLess(np.linalg.norm(trajectory.backup.velocities[-1],axis=1).max(),.01)

    def test_known_narrow_corridor_progress_uses_calibrated_response(self):
        snapshot,parameters=cases()['F_narrow_corridor']
        result=self.planner(parameters).plan(snapshot)
        self.assertEqual(result.reason,'GOAL_REACHED');self.assert_certified(snapshot,result)
        self.assertTrue(np.all(result.trajectory.trace.positions[-1,:,0]>=2.1))
        self.assertLess(result.trajectory.valid_until,result.trajectory.trace.times[-1])

    def test_temporary_full_block_returns_certified_wait_then_resumes(self):
        snapshot,parameters=cases()['D_temporary_block'];planner=self.planner(parameters,forward_distance=8.)
        result=planner.plan(snapshot)
        self.assertEqual(result.reason,'WAIT_FOR_DYNAMIC');self.assertEqual(result.blocking_track_id,1)
        self.assertEqual(result.trajectory.primitives[0].name,'WAIT');self.assert_certified(snapshot,result)
        cleared=replace(snapshot,obstacles=(obstacle([3.,6.4,0.],[0.,.8,0.],size=(1.,8.,2.)),))
        resumed=planner.plan(cleared)
        self.assertEqual(resumed.reason,'GOAL_REACHED');self.assert_certified(cleared,resumed)

    def test_sideways_inertia_command_memory_and_control_phase_cannot_merge(self):
        snapshot,parameters=known_snapshot();planner=self.planner(parameters)
        base=snapshot.response_state;changed=base.velocity.copy();changed[:,1]=.7
        a=replace(base,velocity=changed);b=replace(base,velocity=-changed)
        self.assertNotEqual(planner.label_key(a,snapshot.route,100.),planner.label_key(b,snapshot.route,100.))
        command=base.previous.copy();command[:,0]=1.
        c=replace(base,previous=command)
        self.assertNotEqual(planner.label_key(base,snapshot.route,100.),planner.label_key(c,snapshot.route,100.))
        d=replace(base,next_control=np.full(3,100.08))
        self.assertNotEqual(planner.label_key(base,snapshot.route,100.),planner.label_key(d,snapshot.route,100.))

    def test_unknown_map_cannot_certify_move_or_stop(self):
        snapshot,parameters=known_snapshot();mapping=snapshot.occupancy
        states=readonly(np.full(mapping.states.shape,UNKNOWN),dtype=np.uint8)
        snapshot=replace(snapshot,occupancy=replace(mapping,states=states,static_states=states))
        result=self.planner(parameters).plan(snapshot)
        self.assertEqual(result.reason,'UNKNOWN_BLOCKED');self.assertIsNone(result.trajectory)
        self.assertEqual(result.first_conflict.reason,'UNKNOWN_BLOCKED')

    def test_static_block_is_not_called_dynamic_wait(self):
        snapshot,parameters=known_snapshot();snapshot=set_box(snapshot,[1.,-4.,-3.],[1.5,4.,3.],OCCUPIED)
        result=self.planner(parameters).plan(snapshot)
        self.assertEqual(result.reason,'NO_PATH');self.assertIsNone(result.trajectory)
        self.assertEqual(result.first_conflict.reason,'STATIC_COLLISION')

    def test_expansion_budget_returns_only_previously_certified_short_plan(self):
        snapshot,parameters=known_snapshot(velocity=(2.,0.,0.))
        result=self.planner(parameters,max_expansions=1).plan(snapshot)
        self.assertEqual(result.reason,'SEARCH_TIMEOUT');self.assertTrue(result.budget_exhausted)
        self.assert_certified(snapshot,result)

    def test_elapsed_deadline_with_no_certificate_returns_no_plan(self):
        snapshot,parameters=known_snapshot()
        result=self.planner(parameters,budget=1e-8).plan(snapshot)
        self.assertEqual(result.reason,'SEARCH_TIMEOUT');self.assertIsNone(result.trajectory)
        self.assertTrue(result.budget_exhausted)

    def test_backup_stop_is_not_truncated_by_search_horizon(self):
        snapshot,parameters=known_snapshot(velocity=(4.,0.,0.))
        result=self.planner(parameters,horizon=.25,max_expansions=10).plan(snapshot)
        self.assert_certified(snapshot,result)
        self.assertGreater(result.trajectory.backup.times[-1]-100.,.25)

    def test_backup_preserves_chosen_height_reference_instead_of_prediction_bias(self):
        snapshot,parameters=known_snapshot(velocity=(4.,0.,0.))
        result=self.planner(parameters,max_expansions=1).plan(snapshot)
        self.assert_certified(snapshot,result)
        self.assertEqual(result.trajectory.backup_profile.height,result.trajectory.profiles[0].height)
        self.assertFalse(np.allclose(result.trajectory.backup.positions[0,:,2],result.trajectory.backup_profile.height))

    def test_stale_scene_and_mismatched_response_model_are_rejected(self):
        snapshot,parameters=known_snapshot();invalid=replace(snapshot,occupancy=replace(snapshot.occupancy,valid=False))
        self.assertEqual(self.planner(parameters).plan(invalid).reason,'SENSOR_STALE')
        self.assertEqual(self.planner(replace(parameters,lift_gain=.1)).plan(snapshot).reason,'MODEL_MISMATCH')

    def test_reaction_delay_is_included_once_and_drives_old_command(self):
        snapshot,parameters=known_snapshot(velocity=(2.,0.,0.));snapshot=replace(snapshot,reaction_delay=.2)
        result=self.planner(parameters,max_expansions=1).plan(snapshot)
        self.assert_certified(snapshot,result)
        self.assertAlmostEqual(result.trajectory.trace.times[0],100.)
        self.assertAlmostEqual(result.trajectory.valid_until,100.45)
        np.testing.assert_allclose(result.trajectory.trace.positions[0,:,0],.1)

    def test_search_exposes_zero_forward_side_motion_before_a_static_obstacle(self):
        snapshot,parameters=known_snapshot([obstacle([6.,0.,0.])]);planner=self.planner(parameters,lateral_rate=1.5)
        actions=planner.actions(snapshot.response_state,snapshot.route)
        self.assertTrue(any(p.name=='MOVE' and np.linalg.norm(p.target[:2])==0 and abs(profile.lateral)>1. for p,profile in actions))

    def test_invalid_options_are_rejected(self):
        for values in ({'budget':0.},{'horizon':.1},{'speed_levels':(0.,4.,2.)},{'forward_distance':26.},{'max_expansions':2.5}):
            with self.assertRaises(ValueError):LatticeConfig(**values)


if __name__=='__main__':unittest.main()
