"""Exercise the real opt-in mission entry and enforce unique publication."""
import importlib.util
import sys
import types
import unittest
import threading
from pathlib import Path
from unittest.mock import Mock,patch
import numpy as np


class SpaceTimeStartupTests(unittest.TestCase):
    def test_new_mode_builds_reference_and_publishes_only_through_executor(self):
        script=Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts/route_follower.py'
        sys.path.insert(0,str(script.parent))
        modules={name:types.ModuleType(name) for name in ('rospy','tf','tf.transformations',
            'geometry_msgs','geometry_msgs.msg','sensor_msgs','sensor_msgs.msg','std_msgs','std_msgs.msg','airsim_ros','airsim_ros.msg',
            'visualization_msgs','visualization_msgs.msg')}
        modules['visualization_msgs.msg'].MarkerArray=types.SimpleNamespace
        for name,message in (('geometry_msgs.msg','PoseStamped'),('sensor_msgs.msg','PointCloud2')):
            setattr(modules[name],message,object)
        modules['std_msgs.msg'].String=types.SimpleNamespace
        modules['airsim_ros.msg'].VelCmd=lambda:types.SimpleNamespace(header=types.SimpleNamespace())
        ros=modules['rospy']
        for name in ('Publisher','Subscriber','on_shutdown','Timer','loginfo','logwarn','logwarn_throttle'):
            setattr(ros,name,Mock())
        ros.Publisher.side_effect=lambda *args,**kwargs:Mock()
        params={'~planner_mode':'spacetime'};ros.get_param=lambda key,default=None:params.get(key,default)
        ros.Duration=lambda seconds:seconds
        ros.Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_sec=lambda:2.))
        modules['tf.transformations'].euler_from_quaternion=lambda q:(0.,0.,0.)
        runtime=Mock(last_decision=None,publisher_ident=-1)
        with patch.dict(sys.modules,modules),patch('spacetime_runtime.SpaceTimeRuntime',return_value=runtime):
            spec=importlib.util.spec_from_file_location('spacetime_startup_controller',script)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            controller=module.RouteFollower()
            try:
                self.assertTrue(runtime.start.called)
                for stamp in np.arange(1.,1.181,.02):
                    pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.,y=0.,z=-1.5),
                        orientation=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
                    message=types.SimpleNamespace(pose=pose,header=types.SimpleNamespace(
                        stamp=types.SimpleNamespace(to_sec=lambda stamp=stamp:float(stamp))))
                    controller.pose_cb(message)
                controller.control_loop(None)
                self.assertTrue(runtime.set_reference.called)
                route=runtime.set_reference.call_args.args[0]
                self.assertGreater(len(route.stations),20)
                with self.assertRaisesRegex(RuntimeError,'Only the space-time executor'):
                    controller.publish(1.,0.,0.)
                runtime.publisher_ident=threading.get_ident()
                from trajectory_executor import ExecutionDecision
                decision=ExecutionDecision(np.array([2.,3.,-1.]),'NORMAL','PASS',True)
                controller._spacetime_publish(decision.command,0.,0.,1.18,np.zeros(3),decision)
                issued=controller.cmd_pub.publish.call_args.args[0]
                np.testing.assert_allclose([issued.vx,issued.vy,issued.vz],[2.,3.,1.])
                controller.stopping=True
                params['~planner_mode']='invalid'
                with self.assertRaises(ValueError):module.RouteFollower()
            finally:controller.planning.close();controller.navigator.close()


if __name__=='__main__':unittest.main()
