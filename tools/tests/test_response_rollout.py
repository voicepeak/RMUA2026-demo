import sys
from pathlib import Path
from dataclasses import replace
import time
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from velocity_response import VelocityResponse
from trajectory_types import Primitive,join_traces
from response_rollout import rollout_primitive,rollout_sequence,rollout_delay,rollout_stop,ResponseParameters
from route_coordinates import RouteCoordinates,HeightProfile


class ResponseRolloutTests(unittest.TestCase):
    def model(self,**options):return VelocityResponse(native=False,**options)

    def test_first_command_equals_prepare_and_commit_memory(self):
        model=self.model(lift_gain=.095,coupling_limited=True,xy_error_max=4.5)
        velocity=np.array([10.,1.,.2]);model.commit([8.,1.,.5],velocity,99.92)
        parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],velocity,100.)
        target=np.array([0.,0.,.4]);expected=model.prepare(target,velocity,100.,np.zeros(3))
        trace=rollout_primitive(parameters,state,Primitive(target,.25,'WAIT'))
        np.testing.assert_allclose(trace.applied_commands[0],np.tile(expected,(3,1)),atol=1e-12)
        remembered=expected.copy();remembered[2]-=model.lift_gain*np.sum((expected[:2]-velocity[:2])**2)
        np.testing.assert_allclose(trace.nominal_commands[0],np.tile(remembered,(3,1)),atol=1e-12)

    def test_full_sequence_matches_chained_primitive_without_live_mutation(self):
        model=self.model();model.commit(np.array([2.,0.,0.]),np.array([2.,0.,0.]),99.9)
        parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],[2.,0.,0.],100.)
        old=(model.previous.copy(),model.applied_command.copy(),model.last_stamp)
        actions=[Primitive([4.,1.,0.],.25),Primitive([2.,-1.,0.],.25),Primitive([0.,0.,0.],.25,'WAIT')]
        full=model.rollout_sequence(state,actions)
        pieces=[];current=state
        for action in actions:
            trace=rollout_primitive(parameters,current,action);pieces.append(trace);current=trace.end_state
        expected=join_traces(pieces)
        np.testing.assert_array_equal(full.positions,expected.positions)
        np.testing.assert_array_equal(full.applied_commands,expected.applied_commands)
        np.testing.assert_array_equal(model.previous,old[0]);np.testing.assert_array_equal(model.applied_command,old[1])
        self.assertEqual(model.last_stamp,old[2])
        with self.assertRaises(ValueError):full.positions[0,0,0]=1.

    def test_wait_brakes_real_inertia_then_settles(self):
        model=self.model(lift_gain=0.,coupling_gains=(0.,0.,0.));model.commit(np.array([4.,0.,0.]),np.array([4.,0.,0.]),99.92)
        parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],[4.,0.,0.],100.)
        wait=rollout_primitive(parameters,state,Primitive([0.,0.,0.],.25,'WAIT'))
        self.assertTrue(np.all(wait.end_state.velocity[:,0]>0.));self.assertTrue(np.all(wait.end_state.position[:,0]>0.))
        stop,settled=rollout_stop(parameters,wait.end_state)
        self.assertTrue(settled);self.assertLess(np.linalg.norm(stop.velocities[-1],axis=1).max(),.01)
        self.assertGreater(stop.times[-1]-stop.times[0],1.)

    def test_delayed_command_integrates_previous_drive_or_coasting(self):
        model=self.model(lift_gain=0.,coupling_gains=(0.,0.,0.));model.commit(np.array([6.,0.,0.]),np.array([2.,0.,0.]),99.9)
        parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],[2.,0.,0.],100.)
        driven=rollout_delay(parameters,state,.16)
        taus=np.array(parameters.scenarios)[:,0]
        np.testing.assert_allclose(driven.positions[-1,:,0],6.*.16+(2.-6.)*taus*(1.-np.exp(-.16/taus)))
        model.reset();coasting=rollout_delay(parameters,model.snapshot_state([0.,0.,0.],[2.,0.,0.],100.),.16)
        np.testing.assert_allclose(coasting.positions[-1,:,0],.32)
        self.assertFalse(coasting.end_state.has_applied)

    def test_feedback_phase_and_previous_command_survive_action_boundary(self):
        model=self.model();parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],[0.,0.,0.],100.)
        a=rollout_primitive(parameters,state,Primitive([4.,1.,0.],.25))
        b=rollout_primitive(parameters,a.end_state,Primitive([0.,0.,0.],.25,'WAIT'))
        np.testing.assert_array_equal(b.applied_commands[0],a.applied_commands[-1])
        self.assertAlmostEqual(b.times[1],100.32)
        self.assertFalse(np.allclose(b.applied_commands[1],b.applied_commands[0]))

    def test_scenario_velocities_are_retained_instead_of_averaged(self):
        model=self.model();state=model.snapshot_state([0.,0.,0.],[0.,0.,0.],100.)
        trace=model.rollout_primitive(state,[4.,1.,0.],.5)
        self.assertGreater(np.ptp(trace.velocities[-1,:,0]),.01)
        continuation=model.rollout_primitive(trace.end_state,[2.,0.,0.],.25)
        np.testing.assert_array_equal(continuation.velocities[0],trace.velocities[-1])

    def test_fixed_drive_matches_python_and_native_legacy_integration(self):
        for native in (False,True):
            model=VelocityResponse(native=native,lift_gain=.095,coupling_gains=(.075,.11,.13),control_periods=(.08,.16,.4))
            if native and model.native.library is None:continue
            position=np.array([1.,2.,3.]);velocity=np.array([4.,1.,.2]);command=np.array([6.,0.,1.])
            model.commit(command,velocity,99.9)
            delay=rollout_delay(model.freeze_parameters(),model.snapshot_state(position,velocity,100.),.24)
            if native:model.native.samples=lambda path,timing,position:(path,timing)
            else:model._sample_envelopes=lambda path,timing,position:(path,timing)
            path,timing=model._envelopes(position,velocity,[command],0.,.32,None)
            for i,t in enumerate(delay.times[1:]):
                match=np.flatnonzero(abs(timing-(t-100.))<1e-8)[0]
                np.testing.assert_allclose(delay.positions[i+1],path[match,0],atol=1e-10)

    def test_first_order_acceleration_bound_contains_chord_deviation(self):
        model=self.model(lift_gain=0.,coupling_gains=(0.,0.,0.));model.commit(np.array([4.,0.,0.]),np.array([0.,0.,0.]),99.9)
        parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],[0.,0.,0.],100.)
        trace=rollout_delay(parameters,state,.08);tau=np.array(parameters.scenarios)[:,0]
        midpoint=4.*.04-4.*tau*(1.-np.exp(-.04/tau))
        chord=.5*trace.positions[-1,:,0]
        self.assertTrue(np.all(abs(midpoint-chord)<=trace.acceleration_bounds[0,:,0]*.08**2/8.))

    def test_model_mismatch_and_invalid_actions_are_rejected(self):
        model=self.model();parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],[0.,0.,0.],100.)
        with self.assertRaises(ValueError):rollout_primitive(replace(parameters,lift_gain=.1),state,Primitive([1.,0.,0.],.25))
        with self.assertRaises(ValueError):Primitive([1.,0.,0.],.25,'WAIT')
        with self.assertRaises(ValueError):ResponseParameters(periods=(.05,))

    def test_stop_deadline_and_duration_never_claim_a_truncated_stop(self):
        model=self.model();model.commit(np.array([10.,0.,0.]),np.array([10.,0.,0.]),99.9)
        parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],[10.,0.,0.],100.)
        trace,stopped=rollout_stop(parameters,state,max_duration=.25)
        self.assertIsNotNone(trace);self.assertFalse(stopped)
        trace,stopped=rollout_stop(parameters,state,deadline=time.monotonic()-1.)
        self.assertIsNone(trace);self.assertFalse(stopped)

    def test_sideways_and_vertical_move_at_zero_forward_speed_are_not_wait(self):
        model=self.model();parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],[0.,0.,0.],100.)
        route=RouteCoordinates([-2.,10.],[[-2.,0.,0.],[10.,0.,0.]])
        sideways=rollout_primitive(parameters,state,Primitive([0.,0.,0.],.75),HeightProfile(route,lateral=1.,lateral_gain=2.))
        self.assertTrue(np.all(sideways.end_state.position[:,1]>.1))
        np.testing.assert_allclose(sideways.end_state.position[:,0],0.)
        vertical=rollout_primitive(parameters,state,Primitive([0.,0.,0.],.5),HeightProfile(route,height=.5))
        self.assertTrue(np.all(vertical.end_state.position[:,2]>.1))

    def test_nine_feedback_scenarios_keep_different_event_schedules(self):
        model=self.model(control_periods=(.08,.16,.4));parameters=model.freeze_parameters()
        state=model.snapshot_state([0.,0.,0.],[0.,0.,0.],100.)
        trace=rollout_primitive(parameters,state,Primitive([8.,0.,0.],.25))
        self.assertEqual(trace.positions.shape[1],9)
        self.assertGreater(trace.end_state.applied[0,0],trace.end_state.applied[2,0])
        self.assertAlmostEqual(trace.end_state.next_control[0],100.32)
        self.assertAlmostEqual(trace.end_state.next_control[2],100.4)

    def test_unchanged_action_is_invariant_to_a_primitive_boundary(self):
        model=self.model();model.commit(np.array([2.,0.,.5]),np.array([1.,0.,0.]),99.92)
        parameters=model.freeze_parameters();state=model.snapshot_state([0.,0.,0.],[1.,0.,0.],100.)
        single=rollout_primitive(parameters,state,Primitive([4.,1.,0.],.5))
        split=rollout_sequence(parameters,state,[Primitive([4.,1.,0.],.25),Primitive([4.,1.,0.],.25)])
        np.testing.assert_allclose(single.end_state.position,split.end_state.position,atol=1e-10)
        np.testing.assert_allclose(single.end_state.velocity,split.end_state.velocity,atol=1e-10)
        np.testing.assert_allclose(single.end_state.effective,split.end_state.effective,atol=1e-10)


if __name__=='__main__':unittest.main()
