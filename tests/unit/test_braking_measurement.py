import unittest
import numpy as np
from evaluation.metrics.braking import measure_braking
from control.reference import benchmark_reference


class BrakingMeasurementTests(unittest.TestCase):
    def truth(self):
        rows=[]
        for t in np.arange(3.,11.,.01):
            if t<4:x=0.
            elif t<6:x=.4*(t-4)
            else:
                dt=min(.8,t-6)
                x=.8+.4*dt-.25*dt*dt
            rows.append({'stamp':float(t),'position':(float(x),0.,-1.)})
        return rows

    def test_constant_deceleration_response_and_finite_stop(self):
        command=[{'sim_elapsed':6.,'command':{'stamp':6.}}]
        r=measure_braking(command,self.truth())
        self.assertTrue(r['valid'])
        self.assertFalse(r['certified_deceleration_bound'])
        self.assertAlmostEqual(r['forward_excursion_m'],.16,delta=.01)
        self.assertAlmostEqual(r['stop_time_s'],.64,delta=.05)
        self.assertGreater(r['effective_average_deceleration_m_s2'],.4)
        self.assertLess(r['effective_average_deceleration_m_s2'],.55)

    def test_gaps_and_missing_braking_phase_cannot_create_a_bound(self):
        command=[{'sim_elapsed':6.,'command':{'stamp':6.}}]
        self.assertFalse(measure_braking([],self.truth())['valid'])
        rows=[r for r in self.truth() if not 6.<r['stamp']<6.3]
        self.assertFalse(measure_braking(command,rows)['valid'])

    def test_reference_velocity_changes_at_stop_with_continuous_position(self):
        before=benchmark_reference((0.,0.,0.),5.999,braking=True)
        after=benchmark_reference((0.,0.,0.),6.,braking=True)
        self.assertAlmostEqual(after[0][0],.8)
        self.assertAlmostEqual(before[1][0],.4)
        self.assertEqual(after[1][0],0.)
        self.assertLess(abs(before[0][0]-after[0][0]),.001)


if __name__ == '__main__':
    unittest.main()
