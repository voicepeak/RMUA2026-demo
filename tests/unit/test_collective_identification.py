from dataclasses import replace
import math
import unittest
from control.collective_identification import CollectivePulse
from interfaces.contracts import Health
from tests.unit.test_angular_recovery import state


class CollectiveExcitationTests(unittest.TestCase):
    def test_exact_endpoints_are_available_only_once_in_settled_window(self):
        for value in (0.,.4,1.):
            pulse=CollectivePulse(value)
            self.assertIsNone(pulse.request(state(),4.9))
            self.assertIsNone(pulse.request(state(omega=(0.,.2,0.)),5.))
            self.assertIsNone(pulse.request(replace(state(),velocity_ned=(.3,0.,0.)),5.))
            self.assertIsNone(pulse.request(state(health=Health.DEGRADED),5.))
            self.assertEqual(pulse.request(state(),5.),(value,)*4)
            pulse.published(10.)
            self.assertIsNone(pulse.request(state(),5.1))
            with self.assertRaises(RuntimeError):pulse.published(10.1)
        for value in (-.01,1.01,math.nan):
            with self.assertRaises(ValueError):CollectivePulse(value)


if __name__=='__main__':unittest.main()
