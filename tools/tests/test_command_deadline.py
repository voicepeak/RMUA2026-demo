import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from command_deadline import CommandDeadline


class CommandDeadlineTests(unittest.TestCase):
    def test_blocked_calculation_expires_and_cannot_restart_old_drive(self):
        lease=CommandDeadline()
        lease.record(1.,False)
        self.assertFalse(lease.expire(1.24))
        self.assertTrue(lease.expire(1.25))
        self.assertFalse(lease.accepts(1.1))
        self.assertFalse(lease.accepts(None))
        self.assertFalse(lease.expire(1.3))
        lease.record(1.3,True)
        self.assertFalse(lease.accepts(1.1))
        self.assertTrue(lease.accepts(1.31))
        lease.record(1.4,False)
        self.assertFalse(lease.expire(1.64))
        self.assertTrue(lease.expire(1.65))

    def test_current_publications_renew_until_a_hardware_stop_disarms(self):
        lease=CommandDeadline()
        lease.record(1.,False);lease.record(1.2,False)
        self.assertFalse(lease.expire(1.3))
        lease.record(1.35,True)
        self.assertFalse(lease.expire(10.))
        self.assertTrue(lease.accepts(None))

    def test_invalid_deadline_cannot_exceed_prediction_hold(self):
        for value in (0.,-.1,.35,float('nan'),float('inf')):
            with self.assertRaises(ValueError):CommandDeadline(value)
