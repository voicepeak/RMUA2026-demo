"""Initialize the actual controller with ROS I/O stubbed, before flying."""
import importlib.util
import sys
import types
import unittest
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
        ros=modules['rospy']
        for name in ('Publisher','Subscriber','on_shutdown','Timer','loginfo','logwarn'):
            setattr(ros,name,Mock())
        ros.Duration=lambda seconds:seconds
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('startup_route_follower',script)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            for adaptive in (False,True):
                with self.subTest(adaptive=adaptive):
                    params={'~adaptive_speed':adaptive,'~cruise_speed':40.,'~max_speed':40.,'~lidar_braking':8.}
                    ros.get_param=lambda key,default=None:params.get(key,default)
                    controller=module.RouteFollower()
                    try:
                        self.assertEqual(controller.execution_guard.braking,4.)
                        self.assertIsNone(controller.executing_plan)
                        self.assertTrue(ros.Timer.called)
                        self.assertGreater(controller.route.total_s,1000.)
                        pose_calls=[call for call in ros.Subscriber.call_args_list
                                    if call.args[0]=='/airsim_node/drone_1/debug/pose_gt']
                        self.assertTrue(pose_calls)
                        self.assertEqual(pose_calls[-1].kwargs['queue_size'],1)
                    finally:controller.planning.close()


if __name__=='__main__':unittest.main()
