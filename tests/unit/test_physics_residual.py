import unittest
import numpy as np
from evaluation.metrics.physics_residual import interval_wrench,evaluate


class PhysicsResidualTests(unittest.TestCase):
    def test_feedback_is_integrated_across_changes_and_missing_coverage_rejected(self):
        stamps=np.array([1.,1.01,1.02])
        values=np.array([[4.,0,0,0],[8.,0,0,0],[12.,0,0,0]])
        np.testing.assert_allclose(interval_wrench(stamps,values,1.005,1.015),[6,0,0,0])
        self.assertIsNone(interval_wrench(stamps,values,.99,1.01))
        self.assertIsNone(interval_wrench(stamps,values,1.015,1.2))

    def test_uncommanded_impulse_is_retained_and_not_called_collision(self):
        config={'arm_radius_m':.18,'max_rpm':60,'thrust_coefficient_rps2':1,
                'torque_coefficient_rps2':.01,'local_pwm_ceiling':.6,
                'yaw_torque_signs':[1,1,-1,-1],'inertia_kg_m2':[.01,.02,.03],
                'mass_kg':1,'parameter_status':'provisional'}
        raw=[{'stamp':1.+i*.01,'omega_body':[0.,2.*(i==2),0.],
              'acceleration_body':[0.,0.,-1.]} for i in range(3)]
        feedback=[{'data':{'header':{'stamp':{'secs':1,'nsecs':i*10000000}},
                   **{'rotorPWM'+str(j):.25 for j in range(4)}}} for i in range(3)]
        result=evaluate(raw,feedback,config)
        self.assertEqual(result['matched_intervals'],2)
        self.assertAlmostEqual(result['angular_residual_p50_p95_p99_max_Nm'][-1],4.)
        self.assertFalse(result['cause_identified'])
        np.testing.assert_allclose(result['largest_angular_discrepancies'][0]['specific_force_residual_m_s2'],0)


if __name__=='__main__':unittest.main()
