import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from race_runner import build_leg,controller_command,goal_matches_road,recorded_return_guides
from reference_planner import RouteGeometry,ReferencePlanner


class RaceLegTests(unittest.TestCase):
    def test_marker_lowering_is_not_replayed_through_departure_window(self):
        route=RouteGeometry([(0.,0.,0.),(100.,0.,0.)])
        trace=[(7.,0.,-31.7),(15.,0.,-31.7),(24.,0.,-32.4),
               (41.,0.,-34.5),(50.,0.,-34.8),(70.,30.,-99.)]
        guides=recorded_return_guides(route,route,trace)
        self.assertEqual([g['s'] for g in guides],[41.,50.])
        planner=ReferencePlanner(route)
        _,profile=planner.build([dict(id=0,x=28.,y=0.,s=28.,z=-34.,
            source='static_yaml')],guides,[],0.,-33.,verified_ids={0})
        self.assertLess(profile.center(14.),-33.3)

    def test_fifteen_cruise_raises_total_speed_limit_for_return_leg(self):
        command=controller_command('route.yaml','gates.yaml','guides.yaml',15.,True)
        self.assertIn('cruise_speed:=15.0',command)
        self.assertIn('max_speed:=15.0',command)

    def test_startup_zero_goal_cannot_advance_road3_mission(self):
        roads=[[(0.,0.,0.)],[(100.,0.,0.)],[(550.,520.,7.)]]
        self.assertFalse(goal_matches_road(roads,3,(0.,0.,0.)))
        self.assertTrue(goal_matches_road(roads,3,(545.,519.,-30.)))
    def test_controller_command_uses_declared_launch_arguments(self):
        import xml.etree.ElementTree as ET
        launch=Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/launch/route_follower.launch'
        allowed={n.attrib['name'] for n in ET.parse(launch).findall('arg')}
        command=controller_command('route.yaml','gates.yaml','guides.yaml',12.)
        args={s.split(':=')[0] for s in command if ':=' in s}
        self.assertLessEqual(args,allowed)
        self.assertIn('route:=race_leg',command)

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
