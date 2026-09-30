import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from race_runner import build_leg


class RaceLegTests(unittest.TestCase):
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
