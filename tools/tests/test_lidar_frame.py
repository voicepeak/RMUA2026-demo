import importlib.util
from pathlib import Path
import sys
import threading
from types import ModuleType,SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

SCRIPTS=Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'
sys.path.insert(0,str(SCRIPTS))
from lidar_points import LidarFrame,decode_point_cloud
from pose_history import PoseHistory


def message(points,endian='<',width=2,padding=12):
    height=len(points)//width;step=16;row_step=width*step+padding
    data=bytearray(b'\xff'*(height*row_step))
    for i,point in enumerate(points):
        offset=(i//width)*row_step+(i%width)*step
        data[offset:offset+12]=np.asarray(point,dtype=endian+'f4').tobytes()
    fields=[SimpleNamespace(name=key,offset=i*4,datatype=7,count=1) for i,key in enumerate(('x','y','z'))]
    return SimpleNamespace(fields=fields,width=width,height=height,point_step=step,row_step=row_step,
                           data=bytes(data),is_bigendian=endian=='>',
                           header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:1.)))


class LidarFrameTests(unittest.TestCase):
    def test_organized_padding_and_endianness_do_not_produce_false_returns(self):
        points=[[1.,2.,3.],[-1.,0.,.5],[0.,0.,0.],[np.nan,0.,0.]]
        for endian in ('<','>'):
            decoded=decode_point_cloud(message(points,endian))
            np.testing.assert_allclose(decoded,points[:2],atol=0.)

    def test_bad_layout_and_truncated_buffer_are_rejected(self):
        original=message([[1.,2.,3.]]*4)
        for change in ({'row_step':1},{'data':original.data[:10]},{'point_step':0}):
            malformed=SimpleNamespace(**dict(vars(original),**change))
            with self.assertRaises(ValueError):decode_point_cloud(malformed)

    def test_exposure_frame_owns_immutable_points_and_origin(self):
        points=np.array([[1.,2.,3.]]);origin=np.array([0.,0.,-.05])
        frame=LidarFrame(1.,points,origin,3)
        points[:]=10.;origin[:]=20.
        np.testing.assert_array_equal(frame.points,[[1.,2.,3.]])
        np.testing.assert_array_equal(frame.sensor_origin,[0.,0.,-.05])
        with self.assertRaises(ValueError):frame.points[0,0]=0.
        with self.assertRaises(ValueError):frame.sensor_origin[0]=0.
        with self.assertRaises(ValueError):LidarFrame(np.nan,points,origin)

    def test_actual_controller_transform_pairs_raw_frame_with_sensor_time_rotated_mount(self):
        modules={name:ModuleType(name) for name in ('rospy','tf','tf.transformations',
                 'geometry_msgs','geometry_msgs.msg','sensor_msgs','sensor_msgs.msg',
                 'std_msgs','std_msgs.msg','airsim_ros','airsim_ros.msg')}
        for package,name in (('geometry_msgs.msg','PoseStamped'),('sensor_msgs.msg','PointCloud2'),
                             ('std_msgs.msg','String'),('airsim_ros.msg','VelCmd')):
            setattr(modules[package],name,object)
        rotation=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]])
        homogeneous=np.eye(4);homogeneous[:3,:3]=rotation
        modules['tf.transformations'].quaternion_matrix=lambda q:homogeneous
        with patch.dict(sys.modules,modules):
            spec=importlib.util.spec_from_file_location('frame_route_follower',SCRIPTS/'route_follower.py')
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            controller=module.RouteFollower.__new__(module.RouteFollower)
            controller._cloud_lock=threading.RLock();controller._pose_lock=threading.RLock()
            controller.pose_history=PoseHistory()
            q=np.array([0.,np.sqrt(.5),0.,np.sqrt(.5)])
            controller.pose_history.add(.9,[1.,2.,3.],q)
            controller.pose_history.add(1.1,[3.,4.,5.],q)
            controller.lidar_stamp=None;controller.lidar_frame=None;controller.lidar_epoch=0
            controller.cloud_history=[];controller.coupled_navigation=False
            points=np.array([[1.,0.,0.],[0.,2.,0.],[0.,0.,1.]])
            controller.pending_cloud=(1.,points)
            controller._transform_cloud()
            frame=controller.lidar_frame
            np.testing.assert_allclose(frame.sensor_origin,[1.95,3.,4.])
            np.testing.assert_allclose(frame.points,points@rotation.T+frame.sensor_origin)
            self.assertEqual(frame.stamp,1.);self.assertIsNone(controller.pending_cloud)
            controller.pending_cloud=(1.05,np.array([[2.,0.,0.]]));controller._transform_cloud()
            self.assertEqual(len(controller.lidar_points),4)
            self.assertEqual(len(controller.lidar_frame.points),1)
            self.assertEqual(controller.lidar_frame.stamp,1.05)
            np.testing.assert_allclose(controller.lidar_frame.sensor_origin,[2.45,3.5,4.5])
            before=controller.lidar_frame
            controller.pending_cloud=(2.,points);controller._transform_cloud()
            self.assertIs(controller.lidar_frame,before)
            self.assertIsNotNone(controller.pending_cloud)
            controller.pending_cloud=(1.,points);controller._transform_cloud()
            self.assertEqual(controller.lidar_frame.epoch,1)


if __name__=='__main__':unittest.main()
