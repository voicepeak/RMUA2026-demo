import sys
import math
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from terminal_handoff import approach_target


class TerminalHandoffTests(unittest.TestCase):
    def test_low_approach_stays_outside_trigger_before_next_leg(self):
        target,ready=approach_target((20.,0.,-.6),(0.,0.,0.))
        self.assertFalse(ready)
        self.assertEqual(target,(9.,0.))
        # The 10m by 7m trigger fits inside this standoff in any heading.
        self.assertGreater(math.hypot(*target),math.hypot(5.,3.5))

    def test_safe_flight_height_releases_trigger_without_exact_height_deadlock(self):
        for z in (-1.5,-2.,-3.):
            target,ready=approach_target((20.,0.,z),(0.,0.,0.))
            self.assertTrue(ready)
            self.assertEqual(target,(0.,0.))

    def test_ceiling_height_and_lost_height_restore_staging(self):
        for z in (-.5,-4.5):
            target,ready=approach_target((0.,20.,z),(0.,0.,0.))
            self.assertFalse(ready)
            self.assertEqual(target,(0.,9.))

    def test_goal_coincident_pose_does_not_invent_a_retreat_direction(self):
        target,ready=approach_target((3.,4.,0.),(3.,4.,0.))
        self.assertFalse(ready)
        self.assertEqual(target,(3.,4.))
