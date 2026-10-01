import sys
import unittest
from concurrent.futures import Future
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from execution_guard import ExecutionGuard
from lidar_navigation import LidarNavigator
from path_sampling import swept_samples


class LidarNavigationTests(unittest.TestCase):
    def setUp(self):
        self.guard=ExecutionGuard()
        self.nav=LidarNavigator(self.guard,budget=.5)
        self.xy=lambda s:(s,0.,0.)
        self.center=lambda s:0.
        self.car=np.array([(x,y,z) for x in np.arange(7.,11.1,.5)
                          for y in np.arange(-.75,.76,.25) for z in np.arange(-.75,.76,.25)])

    def test_unclassified_point_obstacle_produces_swept_detour(self):
        self.guard._update(self.car,1.,1.05,np.zeros(3))
        path,info=self.nav.path(np.zeros(3),0.,self.xy,self.center,None)
        self.assertIsNotNone(path)
        samples,_,_=swept_samples(path)
        self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)
        around=path[(path[:,0]>=7.)&(path[:,0]<=11.)]
        self.assertGreater(np.max(np.linalg.norm(around[:,1:],axis=1)),1.5)
        self.assertGreaterEqual(info['path_distance'],20.)

    def test_occupancy_cost_cannot_skip_the_first_search_attempt(self):
        self.nav.budget=0.
        self.guard._update(self.car,1.,1.05,np.zeros(3))
        path,info=self.nav.path(np.zeros(3),0.,self.xy,self.center,None)
        self.assertGreaterEqual(info['grid_attempts'],1)
        if path is not None:
            self.assertGreaterEqual(self.guard.index.distance(swept_samples(path)[0]).min(),1.25)

    def test_final_moving_command_uses_the_same_full_stopping_envelope(self):
        command,info=self.nav.select(np.zeros(3),np.array([2.,0.,0.]),np.array([4.,0.,0.]),
            0.,self.xy,self.center,self.car,1.,1.05)
        queries,extent=self.guard.command_envelope(np.zeros(3),np.array([2.,0.,0.]),command,.05)
        self.assertGreaterEqual(self.guard.index.distance(queries).min(),1.25)
        self.assertLessEqual(extent,30.)
        self.assertTrue(info['feasible'])

    def test_thin_wall_between_grid_stations_is_not_crossed(self):
        wall=np.array([(5.5,y,z) for y in np.arange(-5.,5.01,.25) for z in np.arange(-4.,4.01,.25)])
        self.guard._update(wall,1.,1.05,np.zeros(3))
        path,info=self.nav.path(np.zeros(3),0.,self.xy,self.center,None)
        if path is not None:
            self.assertLess(path[:,0].max(),5.5)
            self.assertGreaterEqual(self.guard.index.distance(swept_samples(path)[0]).min(),1.25)

    def test_stale_lidar_cannot_authorize_motion(self):
        command,info=self.nav.select(np.zeros(3),np.zeros(3),np.array([8.,0.,0.]),
            0.,self.xy,self.center,self.car,1.,2.)
        np.testing.assert_array_equal(command,np.zeros(3))
        self.assertEqual(info['command_reason'],'LIDAR_STALE')

    def test_clear_road_does_not_brake_to_land_on_grid_vertices(self):
        walls=np.array([(x,y,z) for x in np.arange(-5.,35.,1.)
                        for y in (-6.,6.) for z in (-3.,0.,3.)])
        command,info=self.nav.select(np.zeros(3),np.array([6.,0.,0.]),np.array([6.,0.,0.]),
            0.,self.xy,self.center,walls,1.,1.05)
        self.assertGreaterEqual(command[0],5.5)
        self.assertLess(abs(command[1]),.1)

    def test_shared_departure_floor_limits_path_and_emitted_xyz(self):
        command,info=self.nav.select(np.zeros(3),np.zeros(3),np.array([2.,0.,1.]),
            0.,self.xy,self.center,self.car,1.,1.05,floor_offset=.483)
        if info.get('path'):
            self.assertTrue(np.all(np.array(info['path'])[:,2]<=.483))
        if info.get('command_reason')!='LIDAR_ESCAPE':
            samples,_=self.guard.command_envelope(np.zeros(3),np.zeros(3),command,.05)
            self.assertLessEqual(samples[:,2].max(),.483)

    def test_async_route_snapshot_survives_cached_path_validation(self):
        self.nav=LidarNavigator(self.guard,budget=.5,async_planning=True)
        try:
            args=(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),0.,self.xy,self.center,self.car,1.,1.05)
            self.nav.select(*args)
            self.nav.future.result(timeout=3.)
            _,info=self.nav.select(*args)
            self.assertEqual(info['command_reason'],'LIDAR_TRACK')
            # This call submits with a cached path, then replaces local ss in
            # path validation. Background interpolation must keep its snapshot.
            self.nav.submitted_at=None
            self.nav.select(*args)
            path,_=self.nav.future.result(timeout=3.)
            self.assertIsNotNone(path)
            wall=np.array([(2.,y,z) for y in np.arange(-5.,5.01,.5) for z in np.arange(-4.,4.01,.5)])
            command,info=self.nav.select(*args[:6],wall,1.1,1.15)
            samples,_=self.guard.command_envelope(args[0],args[1],command,.05)
            self.assertGreaterEqual(self.guard.index.distance(samples).min(),1.25)
        finally:self.nav.close()

    def test_background_failure_does_not_kill_control(self):
        self.nav=LidarNavigator(self.guard,async_planning=True)
        try:
            self.nav.future=Future();self.nav.future.set_exception(ValueError('failed snapshot'))
            _,info=self.nav.select(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),
                0.,self.xy,self.center,self.car,1.,1.05)
            self.assertEqual(info['planner_error'],'failed snapshot')
        finally:self.nav.close()

    def test_cached_path_and_command_share_floor_station_with_offset_start(self):
        self.nav=LidarNavigator(self.guard,async_planning=True)
        try:
            center=lambda s:-.1*max(0.,s)
            position=np.array([-.8,0.,.4])
            self.nav.last_path=np.array([position,*[(t,0.,center(t)+.4) for t in range(1,25)]])
            self.nav.last_plan_stamp=1.;self.nav.future=Future()
            points=np.array([(x,y,z) for x in np.arange(-5.,35.,1.) for y in (-6.,6.) for z in (-3.,0.,3.)])
            command,info=self.nav.select(position,np.zeros(3),np.array([2.,0.,-.2]),
                0.,self.xy,center,points,1.,1.05,.45)
            self.assertTrue(info.get('cached'))
            self.assertTrue(info['feasible'])
            self.assertGreater(command[0],.1)
        finally:self.nav.close()


if __name__=='__main__':unittest.main()
