"""Exercise the real ROS publication gateway without a live simulator."""
import importlib.util
import json
import sys
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np


class LegacyPublicationTests(unittest.TestCase):
    def setUp(self):
        script=Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts/route_follower.py'
        sys.path.insert(0,str(script.parent))
        modules={name:types.ModuleType(name) for name in ('rospy','tf','tf.transformations',
            'geometry_msgs','geometry_msgs.msg','sensor_msgs','sensor_msgs.msg',
            'std_msgs','std_msgs.msg','airsim_ros','airsim_ros.msg')}
        modules['geometry_msgs.msg'].PoseStamped=object
        modules['sensor_msgs.msg'].PointCloud2=object
        modules['std_msgs.msg'].String=types.SimpleNamespace
        modules['airsim_ros.msg'].VelCmd=lambda:types.SimpleNamespace(header=types.SimpleNamespace())
        modules['rospy'].Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:2.))
        modules['tf.transformations'].euler_from_quaternion=lambda q:(0.,0.,0.)
        self.modules=patch.dict(sys.modules,modules);self.modules.start();self.addCleanup(self.modules.stop)
        spec=importlib.util.spec_from_file_location('legacy_publication_controller',script)
        self.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(self.module)
        from command_deadline import CommandDeadline
        from velocity_response import VelocityResponse
        c=self.module.RouteFollower.__new__(self.module.RouteFollower)
        c.command_deadline=CommandDeadline();c._publication_lock=threading.RLock()
        c.stopping=False;c.spacetime_runtime=None;c._deadline_model=None;c._deadline_certified=False;c._deadline_commit=None
        c.clearance_info={};c.cmd_pub=Mock();c.command_trace_pub=Mock();c.accel=8
        c.execution_guard=types.SimpleNamespace(response_model=VelocityResponse(native=False))
        c.motion=types.SimpleNamespace(velocity=np.array([8.,0.,0.]),terminal_velocity=lambda rows,stamp:None)
        c.yaw=0.;c.pose_stamp=1.;c._pose_lock=threading.RLock()
        c.pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=0.))
        c.pose_history=types.SimpleNamespace(rows=[])
        self.controller=c

    def test_failed_certificate_preserves_compensated_brake_and_inertial_history(self):
        c=self.controller;model=c.execution_guard.response_model
        model.commit(np.array([8.,0.,3.]),c.motion.velocity,.9)
        c.publish(4.,0.,-2.,started=1.,publication_check={'replacement_certified':False})
        issued=c.cmd_pub.publish.call_args.args[0]
        self.assertEqual(issued.stop,0)
        self.assertEqual((issued.vx,issued.vy,issued.vz),(4.,0.,-2.))
        np.testing.assert_allclose(model.applied_command,[4.,0.,2.])
        self.assertEqual(json.loads(c.command_trace_pub.publish.call_args.args[0].data)['stop'],0)

    def test_watchdog_brakes_with_fresh_feedback_without_mutating_rollout_model(self):
        c=self.controller;model=c.execution_guard.response_model
        model.coupling_limited=True;model.xy_error_max=4.5
        c.publish(8.,0.,0.,started=1.)
        original=model.applied_command.copy()
        c.pose_stamp=1.26;c.yaw=np.pi/2
        c._watchdog_stop=Mock();c._watchdog_stop.wait.side_effect=[False,True]
        with patch.object(self.module.time,'monotonic',return_value=1.26):
            c.command_deadline.record(1.,False)
            c._watch_commands()
        issued=c.cmd_pub.publish.call_args.args[0]
        self.assertEqual(issued.stop,0)
        self.assertAlmostEqual(issued.vx,0.)
        self.assertGreater(-issued.vy,0.)
        self.assertLess(-issued.vy,8.)
        self.assertLess(issued.vz,0.,'Physical braking requires lift compensation')
        np.testing.assert_allclose(model.applied_command,original)
        count=c.cmd_pub.publish.call_count
        c.publish(20.,0.,0.,started=1.1,publication_check={'accepted_original':True})
        self.assertEqual(c.cmd_pub.publish.call_count,count,'Late planner reply must be dropped')
        c._reconcile_deadline_model()
        np.testing.assert_allclose(model.applied_command,[-issued.vy,0.,-issued.vz],atol=1e-10)
        self.assertIsNone(c._deadline_commit)
        c.publish(1.,0.,0.,started=1.3,frame_yaw=0.)
        self.assertEqual(c.cmd_pub.publish.call_args.args[0].vx,1.)
        np.testing.assert_allclose(model.applied_command,[1.,0.,0.])

    def test_deadline_repeats_stopping_profile_until_planner_recovers(self):
        c=self.controller;model=c.execution_guard.response_model
        model.stop_profile=lambda p,v:np.full(len(p),-.3)
        c.publish(8.,0.,0.,started=1.,publication_check={'accepted_original':True})
        c._watchdog_stop=Mock();c._watchdog_stop.wait.side_effect=[False,False,True]
        c.pose_stamp=1.3
        with patch.object(self.module.time,'monotonic',side_effect=[1.3,1.3,1.6,1.6]):
            c.command_deadline.record(1.,False)
            c._watch_commands()
        self.assertEqual(c.cmd_pub.publish.call_count,3)
        self.assertTrue(c.command_deadline.armed)
        self.assertIs(model.stop_profile,c._deadline_model.stop_profile)
        self.assertGreater(c._deadline_model.applied_command[2],-.3)

    def test_shutdown_uses_hardware_hover_even_after_deadline(self):
        c=self.controller;c.stopping=True;c.command_deadline.record(1.,False)
        c.command_deadline.expire(1.3)
        c.publish(5.,3.,2.)
        self.assertEqual(c.cmd_pub.publish.call_args.args[0].stop,1)
