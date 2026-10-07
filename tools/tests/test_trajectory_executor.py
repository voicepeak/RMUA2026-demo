"""Actual final-command certificates, deadline handling and latest observations."""
from dataclasses import replace
from pathlib import Path
import sys
import time
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from spacetime_scenarios import known_snapshot,obstacle,set_box
from st_lattice import STLattice,LatticeConfig
from trajectory_executor import TrajectoryExecutor,collision_key
from execution_guard import certify_spacetime_command
from trajectory_collision import TrajectoryCollision,CollisionConfig
from trajectory_types import Primitive
from route_coordinates import HeightProfile
from local_occupancy import OCCUPIED


def fixture(velocity=(0.,0.,0.),obstacles=()):
    snapshot,parameters=known_snapshot(velocity=velocity,obstacles=obstacles)
    parameters=replace(parameters,periods=(.05,)*len(parameters.periods),integration_step=.05)
    snapshot=replace(snapshot,response_state=replace(snapshot.response_state,model_key=parameters.model_key))
    return snapshot,parameters


def plan(snapshot,parameters):
    return STLattice(parameters,LatticeConfig(forward_distance=2.,height_step=0.,lateral_rate=0.,budget=.3)).plan(snapshot).trajectory


class TrajectoryExecutorTests(unittest.TestCase):
    def executor(self,parameters):return TrajectoryExecutor(parameters,guard_budget=.3)

    def test_planner_safe_final_command_guard_passes_same_definition(self):
        snapshot,parameters=fixture();trajectory=plan(snapshot,parameters)
        executor=self.executor(parameters);executor.reset(snapshot.epoch)
        self.assertTrue(executor.accept(trajectory,snapshot,collision_key(executor.collision_config),time.monotonic()))
        decision=executor.tick(snapshot,pose_arrival=time.monotonic(),bridge_verified=True)
        self.assertTrue(decision.certified);self.assertEqual(decision.mode,'NORMAL')
        self.assertTrue(TrajectoryCollision(snapshot).check(decision.trace).safe)
        self.assertTrue(TrajectoryCollision(snapshot).check(decision.backup).safe)
        np.testing.assert_allclose(decision.command,decision.trace.applied_commands[0,0])

    def test_expired_plan_only_brakes_without_executing_search_suffix(self):
        snapshot,parameters=fixture(velocity=(2.,0.,0.));trajectory=plan(snapshot,parameters)
        executor=self.executor(parameters);executor.reset(snapshot.epoch);executor.plan=trajectory
        state=replace(snapshot.response_state,stamp=trajectory.valid_until,
            next_control=np.full(3,trajectory.valid_until),next_physics=trajectory.valid_until)
        newer=replace(snapshot,pose_stamp=state.stamp,response_state=state,
                      occupancy=replace(snapshot.occupancy,stamp=state.stamp,evaluated_at=state.stamp))
        decision=executor.tick(newer,pose_arrival=time.monotonic(),bridge_verified=True)
        self.assertTrue(decision.certified);self.assertEqual(decision.mode,'EMERGENCY_STOP')
        self.assertEqual(decision.reason,'PLAN_EXPIRED');self.assertIsNone(executor.plan)
        self.assertLess(decision.command[0],2.)

    def test_delayed_results_are_rejected_by_monotonic_age(self):
        snapshot,parameters=fixture();trajectory=plan(snapshot,parameters);executor=self.executor(parameters)
        for delay in (.2,.6,1.2,2.2):
            accepted=executor.accept(trajectory,snapshot,collision_key(executor.collision_config),10.,10.+delay)
            self.assertEqual(accepted,delay<.25)
        self.assertEqual(executor.rejections['RESULT_EXPIRED'],3)

    def test_epoch_and_parameter_versions_reject_old_results(self):
        snapshot,parameters=fixture();trajectory=plan(snapshot,parameters);executor=self.executor(parameters)
        self.assertFalse(executor.accept(trajectory,replace(snapshot,epoch=1),collision_key(executor.collision_config),time.monotonic()))
        self.assertFalse(executor.accept(trajectory,snapshot,'different',time.monotonic()))
        self.assertEqual(set(executor.rejections),{'EPOCH_MISMATCH','COLLISION_CONFIG_MISMATCH'})

    def test_missing_pose_map_or_bridge_contract_is_uncertified_failsafe(self):
        snapshot,parameters=fixture();executor=self.executor(parameters)
        for args,reason in ((dict(snapshot=None,pose_arrival=10.,now_monotonic=10.,bridge_verified=True),'SNAPSHOT_UNAVAILABLE'),
                            (dict(snapshot=snapshot,pose_arrival=9.,now_monotonic=10.,bridge_verified=True),'POSE_STALE'),
                            (dict(snapshot=snapshot,pose_arrival=10.,now_monotonic=10.),'BRIDGE_CONTRACT_UNVERIFIED'),
                            (dict(snapshot=replace(snapshot,occupancy=replace(snapshot.occupancy,valid=False)),
                                  pose_arrival=10.,now_monotonic=10.,bridge_verified=True),'SENSOR_STALE')):
            decision=executor.tick(**args);self.assertFalse(decision.certified);self.assertEqual(decision.reason,reason)
            np.testing.assert_array_equal(decision.command,np.zeros(3))

    def test_new_obstacle_and_new_velocity_invalidate_final_command(self):
        snapshot,parameters=fixture(velocity=(2.,0.,0.));trajectory=plan(snapshot,parameters)
        executor=self.executor(parameters);executor.reset(snapshot.epoch);executor.plan=trajectory
        newer=replace(snapshot,obstacles=(obstacle([.9,0.,0.]),))
        decision=executor.tick(newer,pose_arrival=time.monotonic(),bridge_verified=True)
        self.assertFalse(decision.certified);self.assertTrue(decision.reason.startswith('EMERGENCY_BLOCK:'))
        executor.plan=trajectory
        faster=replace(snapshot,response_state=replace(snapshot.response_state,velocity=np.tile([8.,0.,0.],(3,1))))
        decision=executor.tick(faster,pose_arrival=time.monotonic(),bridge_verified=True)
        self.assertNotEqual(decision.mode,'NORMAL')

    def test_executor_long_gap_cannot_keep_forward_command(self):
        snapshot,parameters=fixture();executor=self.executor(parameters);executor.reset(snapshot.epoch)
        executor.plan=plan(snapshot,parameters)
        self.assertTrue(executor.tick(snapshot,10.,10.,True).certified)
        decision=executor.tick(snapshot,12.2,12.2,True)
        self.assertEqual(decision.mode,'EMERGENCY_STOP');self.assertEqual(decision.reason,'EXECUTOR_DELAY')

    def test_clock_reset_invalidates_plan(self):
        snapshot,parameters=fixture();executor=self.executor(parameters);executor.reset(snapshot.epoch)
        executor.last_stamp=101.;executor.plan=plan(snapshot,parameters)
        decision=executor.tick(snapshot,pose_arrival=time.monotonic(),bridge_verified=True)
        self.assertEqual(decision.reason,'CLOCK_RESET');self.assertIsNone(executor.plan)

    def test_guard_deadline_does_not_certify_partial_stop(self):
        snapshot,parameters=fixture();profile=HeightProfile(snapshot.route)
        decision=certify_spacetime_command(snapshot,parameters,Primitive([2.,0.,0.],.05),profile,
            CollisionConfig(),.05,time.monotonic()-1.)
        self.assertFalse(decision.certified);self.assertEqual(decision.reason,'GUARD_TIMEOUT')

    def test_delay_checks_coast_and_drive_without_double_cloud_age(self):
        snapshot,parameters=fixture();snapshot=replace(snapshot,reaction_delay=.1)
        profile=HeightProfile(snapshot.route)
        decision=certify_spacetime_command(snapshot,parameters,Primitive([0.,0.,0.],.05,'WAIT'),profile,
            CollisionConfig(),.05,time.monotonic()+1.)
        self.assertTrue(decision.certified)
        self.assertAlmostEqual(decision.backup.times[0],snapshot.pose_stamp+.15)

    def test_changed_route_brakes_in_latest_road(self):
        snapshot,parameters=fixture();executor=self.executor(parameters);executor.reset(snapshot.epoch)
        trajectory=plan(snapshot,parameters)
        changed=replace(snapshot,route=replace(snapshot.route,lateral_limit=1.))
        executor.plan=trajectory
        decision=executor.tick(changed,pose_arrival=time.monotonic(),bridge_verified=True)
        self.assertEqual(decision.mode,'EMERGENCY_STOP');self.assertEqual(decision.reason,'ROUTE_CHANGED')

    def test_unpickled_equal_route_does_not_invalidate_plan(self):
        import pickle
        snapshot,parameters=fixture();executor=self.executor(parameters);executor.reset(snapshot.epoch)
        executor.plan=pickle.loads(pickle.dumps(plan(snapshot,parameters)))
        decision=executor.tick(snapshot,pose_arrival=time.monotonic(),bridge_verified=True)
        self.assertEqual(decision.mode,'NORMAL')

    def test_map_ttl_uses_actual_now_with_an_older_sensor_pose(self):
        snapshot,parameters=fixture();snapshot=replace(snapshot,reaction_delay=.3,evaluation_stamp=100.3,
            occupancy=replace(snapshot.occupancy,stamp=99.79,evaluated_at=100.3))
        result=TrajectoryCollision(snapshot).freshness()
        self.assertEqual(result.reason,'SENSOR_STALE')

    def test_a_tick_cannot_extend_the_certified_prefix_deadline(self):
        snapshot,parameters=fixture();executor=self.executor(parameters)
        executor.reset(snapshot.epoch);executor.plan=plan(snapshot,parameters)
        state=replace(snapshot.response_state,stamp=100.24,next_control=np.full(3,100.24),next_physics=100.24)
        newer=replace(snapshot,pose_stamp=100.24,response_state=state,
                      occupancy=replace(snapshot.occupancy,stamp=100.24,evaluated_at=100.24))
        result=executor.tick(newer,pose_arrival=time.monotonic(),bridge_verified=True)
        self.assertEqual(result.mode,'EMERGENCY_STOP');self.assertEqual(result.reason,'PLAN_EXPIRED')


if __name__=='__main__':unittest.main()
