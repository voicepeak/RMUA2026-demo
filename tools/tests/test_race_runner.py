import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from race_runner import build_leg,controller_command,goal_matches_road,recorded_return_guides,measured_height_pose,outbound_response_arguments
from reference_planner import RouteGeometry,ReferencePlanner


class RaceLegTests(unittest.TestCase):
    def test_marker_lowering_is_not_replayed_through_departure_window(self):
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        trace=[(7.,0.,-31.7),(15.,0.,-31.7),(24.,0.,-32.4),
               (41.,0.,-34.5),(50.,0.,-34.8),(70.,30.,-99.)]
        guides=recorded_return_guides(route,route,trace)
        self.assertEqual([g['s'] for g in guides],[41.,50.])
        self.assertTrue(all(g['source']=='recorded_reference' for g in guides))
        measured=recorded_return_guides(route,route,trace,source='measured_flight_pose',calibration_only=True)
        self.assertTrue(all(g['source']=='measured_flight_pose' and g['calibration_only'] for g in measured))
        planner=ReferencePlanner(route)
        _,profile=planner.build([dict(id=0,x=28.,y=0.,s=28.,z=-34.,
            source='static_yaml')],guides,[],0.,-33.,verified_ids={0})
        self.assertLess(profile.center(14.),-33.3)

    def test_only_actual_near_reference_pose_becomes_height_evidence(self):
        data=dict(position_world=[41.,.2,-34.6],path_xy=[41.,0.],z_ref_raw=-34.5)
        self.assertEqual(measured_height_pose(data),(41.,.2,-34.6))
        self.assertIsNone(measured_height_pose(dict(data,position_world=[41.,.2,-35.2])))
        self.assertIsNone(measured_height_pose(dict(data,position_world=[41.,1.1,-34.6])))
        self.assertIsNone(measured_height_pose(dict(path_xy=[41.,0.],z_ref_raw=-34.5)))
        self.assertIsNone(measured_height_pose(dict(data,position_world=[41.,0.,float('nan')])))

    def test_fifteen_cruise_raises_total_speed_limit_for_return_leg(self):
        command=controller_command('route.yaml','gates.yaml','guides.yaml',15.,True)
        self.assertIn('cruise_speed:=15.0',command)
        self.assertIn('max_speed:=15.0',command)

    def test_coupled_outbound_enables_brake_authority_and_fresh_checks(self):
        args=outbound_response_arguments('coupled')
        for name in ('coupled_response','lidar_coupling_limited','lidar_discrete_feedback','lidar_fresh_publication_check'):
            self.assertIn(name+':=true',args)
        self.assertIn('lidar_xy_error_max:=4.5',args)
        self.assertIn('lidar_coupling_limited:=false',outbound_response_arguments('legacy'))
        with self.assertRaises(ValueError):outbound_response_arguments('invalid')

    def test_startup_zero_goal_cannot_advance_road3_mission(self):
        roads=[[(0.,0.,0.)],[(100.,0.,0.)],[(550.,520.,7.)]]
        self.assertFalse(goal_matches_road(roads,3,(0.,0.,0.)))
        self.assertTrue(goal_matches_road(roads,3,(545.,519.,-30.)))
    def test_controller_command_uses_declared_launch_arguments(self):
        import xml.etree.ElementTree as ET
        launch=Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/launch/route_follower.launch'
        allowed={n.attrib['name'] for n in ET.parse(launch).findall('arg')}
        for adaptive in (False,True):
            command=controller_command('route.yaml','gates.yaml','guides.yaml',12.,adaptive_speed=adaptive)
            args={s.split(':=')[0] for s in command if ':=' in s}
            self.assertLessEqual(args,allowed)
            self.assertIn('route:=race_leg',command)

    def test_adaptive_policy_arguments_reach_private_node_parameters(self):
        import xml.etree.ElementTree as ET
        launch=Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/launch/route_follower.launch'
        tree=ET.parse(launch)
        params={n.attrib['name']:n.attrib.get('value') for n in tree.findall('node/param')}
        command=controller_command('route.yaml','gates.yaml','guides.yaml',12.,adaptive_speed=True)
        arguments=dict(s.split(':=',1) for s in command if ':=' in s)
        expected={'lidar_geometry_margin':'0.65','lidar_gate_opening':'true',
                  'lidar_departure_floor_distance':'8','lidar_envelope_margin':'0.75',
                  'lidar_xy_error_max':'2.0'}
        for name,value in expected.items():
            self.assertEqual(arguments[name],value)
            self.assertEqual(params[name],'$(arg '+name+')')

    def test_published_goal_must_match_road_before_starting(self):
        roads=[[(i,0.,0.) for i in range(30)],
               [(0.,i,0.) for i in range(30)]]
        with self.assertRaises(ValueError):
            build_leg(roads,1,2,(500.,500.,0.),[])

    def test_return_leg_reverses_scoring_normal(self):
        roads=[[(i,0.,0.) for i in range(30)],
               [(0.,i,0.) for i in range(30)]]
        measured=[dict(id=7,x=10.,y=0.,z=0.,nx=1.,ny=0.,nz=0.)]
        points,gates=build_leg(roads,2,1,(0.,0.,0.),measured)
        self.assertEqual(points[0],roads[1][0])
        self.assertEqual(points[-1],roads[0][0])
        self.assertLess(gates[0]['nx'],-.9)

    def test_truncated_export_is_not_a_complete_course(self):
        with self.assertRaises(ValueError):
            build_leg([[(0,0,0),(1,0,0)],[(0,i,0) for i in range(30)]],
                      1,2,(0,0,0),[])
