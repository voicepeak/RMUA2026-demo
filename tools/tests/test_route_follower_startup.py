"""Initialize the actual controller with ROS I/O stubbed, before flying."""
import importlib.util
import sys
import types
import unittest
import math
import threading
import time
import numpy as np
from pathlib import Path
from unittest.mock import Mock,patch


class RouteFollowerStartupTests(unittest.TestCase):
    def test_controller_initializes_with_race_and_legacy_parameters(self):
        script=Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts/route_follower.py'
        sys.path.insert(0,str(script.parent))
        modules={}
        for name in ('rospy','tf','tf.transformations','geometry_msgs','geometry_msgs.msg',
                     'sensor_msgs','sensor_msgs.msg','std_msgs','std_msgs.msg','airsim_ros','airsim_ros.msg'):
            modules[name]=types.ModuleType(name)
        for module,message in [('geometry_msgs.msg','PoseStamped'),('sensor_msgs.msg','PointCloud2'),
                               ('std_msgs.msg','String'),('airsim_ros.msg','VelCmd')]:
            setattr(modules[module],message,object)
        modules['airsim_ros.msg'].VelCmd=lambda:types.SimpleNamespace(header=types.SimpleNamespace())
        modules['std_msgs.msg'].String=types.SimpleNamespace
        ros=modules['rospy']
        for name in ('Publisher','Subscriber','on_shutdown','Timer','loginfo','logwarn','logwarn_throttle'):
            setattr(ros,name,Mock())
        ros.Duration=lambda seconds:seconds
        ros.Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:2.))
        modules['tf.transformations'].euler_from_quaternion=lambda q:(0.,0.,0.)
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('startup_route_follower',script)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            for adaptive in (False,True):
                with self.subTest(adaptive=adaptive):
                    params={'~adaptive_speed':adaptive,'~cruise_speed':40.,'~max_speed':40.,'~lidar_braking':8.,'~lidar_coupling_limited':adaptive,'~lidar_anticipation_distance':9.,'~lidar_xy_error_max':4.5,'~lidar_discrete_feedback':adaptive}
                    ros.get_param=lambda key,default=None:params.get(key,default)
                    if adaptive:params.update({'~lidar_lift_gain':.095,'~lidar_coupling_gain_min':.075})
                    controller=module.RouteFollower()
                    controller._watchdog_stop.set()  # Publication deadlines have their own clock tests.
                    try:
                        self.assertEqual(controller.execution_guard.braking,4.)
                        self.assertIsNone(controller.executing_plan)
                        self.assertEqual(controller.navigator.anticipation_distance,9.)
                        if controller.execution_guard.response_model is not None:
                            self.assertEqual(controller.execution_guard.response_model.lift_gain,.095 if adaptive else .11)
                            self.assertEqual(controller.execution_guard.response_model.scenarios[0][2],.075 if adaptive else .09)
                            self.assertEqual(controller.execution_guard.response_model.xy_error_max,4.5)
                            self.assertEqual(len(controller.execution_guard.response_model.scenarios),9 if adaptive else 3)
                        self.assertTrue(ros.Timer.called)
                        self.assertGreater(controller.route.total_s,1000.)
                        pose_calls=[call for call in ros.Subscriber.call_args_list
                                    if call.args[0]=='/airsim_node/drone_1/debug/pose_gt']
                        self.assertTrue(pose_calls)
                        self.assertEqual(pose_calls[-1].kwargs['queue_size'],1)
                        # Busy control/map computation must not starve the
                        # actual pose callback and create a false stale stop.
                        p=types.SimpleNamespace(x=1.,y=2.,z=-2.)
                        q=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.)
                        message=types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:1.2)),
                            pose=types.SimpleNamespace(position=p,orientation=q))
                        receiver=threading.Thread(target=controller.pose_cb,args=(message,))
                        with controller._lock:
                            receiver.start();receiver.join(timeout=1.)
                            self.assertFalse(receiver.is_alive(),'Pose ingestion stalled behind planner/control lock')
                            self.assertEqual(controller.pose_stamp,1.2)
                        receiver.join(timeout=1.)
                        previous_arrival=controller.pose_arrival
                        previous_history=list(controller.pose_history.rows)
                        for invalid in ((float('nan'),1.),(float('inf'),1.),(1.,.5)):
                            bad_pose=types.SimpleNamespace(
                                position=types.SimpleNamespace(x=invalid[0],y=2.,z=-2.),
                                orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=invalid[1]))
                            bad=types.SimpleNamespace(header=types.SimpleNamespace(
                                stamp=types.SimpleNamespace(to_sec=lambda:1.3)),pose=bad_pose)
                            controller.pose_cb(bad)
                            self.assertEqual(controller.pose_stamp,1.2)
                            self.assertIs(controller.pose,message.pose)
                            self.assertEqual(controller.pose_arrival,previous_arrival)
                            self.assertEqual(len(controller.pose_history.rows),len(previous_history))
                        if adaptive:
                            self.assertTrue(controller.execution_guard.response_model.coupling_limited)
                            from velocity_response import VelocityResponse
                            # Run real control cycles beyond init_once. A
                            # one-time entry update would stay at gain one.
                            saved={key:getattr(controller,key) for key in
                                   ('seg','chain','pose_stamp','last_pose_stamp','prev_s')}
                            controller.seg=0;controller.prev_s=None;controller.gate_idx=0
                            controller.chain=types.SimpleNamespace(gates=[])
                            for configured in (1.,2.):
                                controller.execution_guard.response_model=VelocityResponse(height_gain=configured)
                                controller.execution_guard.response_model.configured_height_gain=configured
                                for station in (0.,10.,25.,5.):
                                    controller.pose_stamp+=.1
                                    with patch.object(controller.route,'project',return_value=
                                            (0,station/controller.route.seg_len[0],0.,(0.,0.,0.))),\
                                         patch.object(controller.task_state,'step',side_effect=RuntimeError('after tracking update')):
                                        with self.assertRaisesRegex(RuntimeError,'after tracking update'):
                                            controller._control_loop(None)
                                    blend=min(1.,station/20.)
                                    self.assertAlmostEqual(controller.execution_guard.response_model.height_gain,
                                                           1.+(configured-1.)*blend)
                                    self.assertAlmostEqual(controller.navigator.height_reserve,
                                                           .5*blend if configured>1. else 0.)
                            for key,value in saved.items():setattr(controller,key,value)
                            model=controller.execution_guard.response_model=VelocityResponse()
                            controller.pose_stamp=1.5;controller.yaw=2.4
                            frozen_yaw=.4
                            controller.publish(2.,-3.,1.,source_pose_stamp=1.7,frame_yaw=frozen_yaw)
                            expected=[2.*math.cos(frozen_yaw)+3.*math.sin(frozen_yaw),
                                      2.*math.sin(frozen_yaw)-3.*math.cos(frozen_yaw),-1.]
                            np.testing.assert_allclose(model.applied_command,expected,atol=1e-12)
                            self.assertEqual(model.last_stamp,1.7)
                            # A wall that was far from the planning pose is
                            # close to the independent, latest pose. Recheck
                            # the actual command, then certify a fresh brake.
                            controller.pose.position=types.SimpleNamespace(x=3.7,y=0.,z=0.)
                            controller.pose_stamp=2.;controller.yaw=.5
                            controller.lidar_stamp=1.95
                            model.reset()
                            controller.lidar_points=np.array([(5.,y,z) for y in np.arange(-5.,5.1,.3)
                                                              for z in np.arange(-4.,4.1,.3)])
                            proposed=controller.arbiter.finalize(np.array([2.,0.]),0.,0.,0.,safety=controller.safety)
                            with patch.object(controller.motion,'terminal_velocity',return_value=np.zeros(3)):
                                result=controller._recertify_publication(proposed,0.,1.5,time.monotonic()-1.)
                            actual,fresh_yaw,stamp,velocity,checked=result
                            self.assertFalse(checked['accepted_original'])
                            self.assertEqual(checked['replacement']['command_reason'],'COMMAND_BRAKING')
                            self.assertEqual(stamp,2.)
                            self.assertEqual(fresh_yaw,.5)
                            self.assertLess(math.hypot(actual.vx,actual.vy),.1)
                            # A short computation preserves its frozen frame;
                            # already clamped commands are never prepared twice.
                            fast=controller._recertify_publication(proposed,0.,1.5,time.monotonic())
                            self.assertIs(fast[0],proposed)
                            self.assertIsNone(fast[-1])
                            # Vehicle mode rechecks even a short computation
                            # against the independent latest pose/cloud.
                            controller.fresh_publication_check=True
                            with patch.object(controller.motion,'terminal_velocity',return_value=np.zeros(3)):
                                fresh=controller._recertify_publication(proposed,0.,1.5,time.monotonic())
                            self.assertFalse(fresh[-1]['accepted_original'])
                            self.assertEqual(fresh[2],2.)
                            self.assertLess(math.hypot(fresh[0].vx,fresh[0].vy),.1)
                            controller.publish(fresh[0].vx,fresh[0].vy,fresh[0].vz,
                                source_pose_stamp=2.,frame_yaw=.5,control_stamp=1.5)
                            self.assertEqual(model.last_source_stamp,2.)
                            self.assertEqual(model.last_stamp,1.5)
                            controller.fresh_publication_check=False
                            controller.publish(actual.vx,actual.vy,actual.vz,source_pose_stamp=stamp,
                                frame_yaw=fresh_yaw,measured_velocity=np.array([2.,0.,0.]),publication_check=checked)
                            self.assertEqual(model.last_stamp,2.)
                            self.assertAlmostEqual(model.previous[2],model.applied_command[2]-.086*4.)
                            # Full-buffer rejection at an already close wall
                            # must use the separately certified nonapproach
                            # recovery, preserving its stopping height profile.
                            controller.pose.position=types.SimpleNamespace(x=0.,y=0.,z=0.)
                            controller.pose_stamp=3.;controller.lidar_stamp=2.95
                            controller.lidar_points=np.array([(.8,y,z) for y in np.arange(-4.,4.1,.2)
                                                               for z in np.arange(-4.,4.1,.2)])
                            controller.clearance_info={'command_reason':'LIDAR_ESCAPE'}
                            model.reset()
                            from lidar_navigation import LidarNavigator
                            model.stop_profile=LidarNavigator._stop_profile(np.array([[0.,0.,0.],[-1.,0.,0.]]))
                            recovery=controller.arbiter.finalize(np.array([-.2,0.]),-.00344,0.,0.,safety=controller.safety)
                            with patch.object(controller.motion,'terminal_velocity',return_value=np.zeros(3)):
                                result=controller._recertify_publication(recovery,0.,2.5,time.monotonic()-1.)
                            self.assertTrue(result[-1]['conditional_recovery'])
                            self.assertTrue(result[-1]['accepted_original'])
                            self.assertNotIn('replacement',result[-1])
                            # Shutdown overrides must replace the applied
                            # history too, instead of keeping the old drive.
                            controller.stopping=True
                            controller.publish(2.,-3.,1.,source_pose_stamp=1.8)
                            self.assertIsNone(model.applied_command)
                            self.assertEqual(controller.cmd_pub.publish.call_args_list[-2].args[0].stop,1)
                    finally:
                        controller._watchdog_stop.set()
                        controller.planning.close();controller.navigator.close()


if __name__=='__main__':unittest.main()
