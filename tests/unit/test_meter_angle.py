import math
import unittest
from inspection.meter_reader.angular_scale import read_angle


def tip(angle):return (100+20*math.cos(angle),100+20*math.sin(angle))


class MeterAngleTests(unittest.TestCase):
    def test_signed_arc_and_wrap(self):
        for zero,sweep in ((2.8,2.),(-2.8,-2.)):
            result=read_angle((100,100),tip(zero+sweep*.2),zero,sweep,.005)
            self.assertEqual(result.value,10);self.assertEqual(result.status,'CANDIDATE')
            self.assertFalse(result.report_eligible);self.assertFalse(result.scale_calibrated)

    def test_boundary_uncertainty_refuses_guess(self):
        result=read_angle((100,100),tip(.42),0.,2.,.01)
        self.assertEqual(result.status,'AMBIGUOUS');self.assertIsNone(result.value)

    def test_out_of_arc_and_bad_geometry(self):
        self.assertEqual(read_angle((100,100),tip(-.2),0.,2.,.01).status,'OUTSIDE_CALIBRATED_ARC')
        for args in (((0,0),(0,0),0,1,.01),((0,0),(10,0),0,0,.01),((0,0),(10,0),0,2*math.pi,.01)):
            with self.assertRaises(ValueError):read_angle(*args)


if __name__=='__main__':unittest.main()
