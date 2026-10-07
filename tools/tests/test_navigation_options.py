import sys,time,unittest,numpy as np
from pathlib import Path
from concurrent.futures import Future
from unittest.mock import Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from execution_guard import ExecutionGuard
from lidar_navigation import LidarNavigator,_plan_snapshot
from velocity_response import VelocityResponse
from path_sampling import swept_samples
class OptionsTests(unittest.TestCase):
 def create(self):
  guard=ExecutionGuard(reaction=.25);guard.response_model=VelocityResponse(lift_gain=.11,height_gain=2.,coupling_limited=True,xy_error_max=4.5,control_periods=(.08,.16,.4))
  nav=LidarNavigator(guard,path_options=True);nav.executor=Mock();nav.future=Future();nav.submitted_at=time.monotonic();nav.future_stamp=1.
  primary=np.column_stack((np.arange(16.),np.zeros((16,2))))
  alternative=np.array([[0.,0.,0.],[0.,2.2,0.],[4.,2.2,0.],[15.,2.2,0.]])
  nav.future.set_result((primary,dict(path_distance=15.,path_options=[dict(path=alternative.tolist(),path_preference=1.25,path_distance=15.)])))
  return nav,guard
 def test_reset_discards_previous_geometry_alternatives(self):
  nav,guard=self.create();nav.last_path_options=[dict(path=[[0.,0.,0.],[5.,1.,0.]])]
  nav.reset();self.assertEqual(nav.last_path_options,[])
 def test_cached_alternative_certifies_actual_motion_after_primary_is_blocked(self):
  nav,guard=self.create()
  wall=np.array([[1.7,y,z] for y in np.arange(-.75,.76,.15) for z in np.arange(-1.,1.01,.2)])
  command,info=nav.select(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),0.,lambda s:(s,0.),lambda s:0.,wall,1.,1.03)
  self.assertEqual(info['command_reason'],'LIDAR_TRACK');self.assertTrue(info['alternative_path_used']);self.assertGreater(command[1],0.)
  samples,_,times=guard.response_model.envelope(np.zeros(3),np.zeros(3),command,.28);guard.envelope_times=times
  self.assertGreaterEqual(guard.index.distance(samples).min(),1.25);self.assertTrue(guard.command_constraint(samples))
 def test_latest_wall_rejects_both_cached_paths(self):
  nav,guard=self.create()
  wall=np.array([[1.7,y,z] for y in np.arange(-5.,5.01,.25) for z in np.arange(-4.,4.01,.25)])
  wall=np.vstack((wall,np.array([[x,1.7,z] for x in np.arange(-5.,16.,.25) for z in np.arange(-4.,4.01,.25)])))
  command,info=nav.select(np.zeros(3),np.zeros(3),np.array([4.,0.,0.]),0.,lambda s:(s,0.),lambda s:0.,wall,1.,1.03)
  self.assertNotEqual(info['command_reason'],'LIDAR_TRACK');self.assertFalse(info['feasible'])
 def test_worker_alternatives_keep_continuous_observed_clearance(self):
  guard=ExecutionGuard();guard.response_model=VelocityResponse()
  wall=np.array([[x,y,z] for x in np.arange(7.,11.01,.5) for y in np.arange(-.75,.76,.25) for z in np.arange(-.75,.76,.25)])
  guard._update(wall,1.,1.03,np.zeros(3))
  ss=np.arange(-1.,35.1,.5);coords=np.column_stack((ss,np.zeros(len(ss))))
  path,info=_plan_snapshot((np.zeros(3),0.,ss,coords,np.zeros(len(ss)),None,guard.index,guard.margin,24.,.5,True,9.,True))
  self.assertIsNotNone(path);self.assertGreater(len(info['path_options']),0)
  for option in [path]+[np.array(v['path']) for v in info['path_options']]:
   samples=swept_samples(option)[0];self.assertGreaterEqual(guard.index.distance(samples).min(),1.25)
   self.assertLessEqual(abs(samples[:,1]).max(),2.25)
if __name__=='__main__':unittest.main()
