import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from race_monitor_policy import should_stop_controller_event


class MonitorPolicyTests(unittest.TestCase):
    def test_route_end_and_official_goal_handoff_do_not_stop_recording(self):
        for reason in ('ROUTE_END','RACE_GOAL_CHANGED','ROS_SHUTDOWN'):
            self.assertFalse(should_stop_controller_event(dict(kind='TERMINATION',reason=reason)))

    def test_abort_still_stops_and_regular_events_do_not(self):
        self.assertTrue(should_stop_controller_event(dict(kind='TERMINATION',reason='STUCK')))
        self.assertFalse(should_stop_controller_event(dict(kind='GATE',reason='PASSED')))
