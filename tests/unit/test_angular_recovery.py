from dataclasses import replace
import json
from pathlib import Path
import unittest
import numpy as np
from control.safety_supervisor.angular_recovery import AngularRecovery
from control.motor_allocator.allocator import MotorAllocator
from control.se3_controller.controller import Se3Controller
from estimation.geometry import delta_quaternion,multiply,unit,rotation
from interfaces.contracts import StateEstimate,Metadata,Health


def state(stamp=1.,omega=(0.,0.,0.),health=Health.OK,epoch=0):
    return StateEstimate(Metadata(stamp,stamp+.05,epoch,'test'),(0.,0.,-1.),(0.,0.,0.),
                         (1.,0.,0.,0.),omega,health)


def simulate_recovery(axis,impulse,lag=.02,delay=0.,inertia_scale=1.,profile='standard'):
    cfg=json.loads((Path(__file__).resolve().parents[2]/'configs/vehicle_identification.json').read_text())
    inertia=np.array(cfg['inertia_kg_m2'])*inertia_scale;mass=cfg['mass_kg']
    controller=Se3Controller(cfg);allocator=MotorAllocator(cfg);supervisor=AngularRecovery(profile)
    q=np.array([1.,0.,0.,0.]);p=np.array([0.,0.,-1.]);v=np.zeros(3)
    direction=np.eye(3)[axis] if axis<3 else np.array([1.,1.,0.])/np.sqrt(2)
    omega=direction*impulse
    actual=np.array([mass*9.81,0,0,0]);request=actual.copy()
    dt=.0025;max_tilt=0.;max_drift=0.
    delay_steps=round(delay/dt)
    history=[(p.copy(),v.copy(),q.copy(),np.zeros(3)) for _ in range(delay_steps)]
    for i in range(1600):
        history.append((p.copy(),v.copy(),q.copy(),omega.copy()))
        if i%5==0:
            observed=history[len(history)-1-delay_steps]
            s=StateEstimate(Metadata(1+i*dt,1.05+i*dt,0,'model'),tuple(observed[0]),tuple(observed[1]),
                            tuple(observed[2]),tuple(observed[3]),Health.OK)
            decision=supervisor.decide(s)
            wrench=controller.step(s,decision.hold_position,yaw=decision.hold_yaw,
                dt=.0125,allow_integral=False,rate_gain_scale=decision.rate_gain_scale,
                attitude_gain_scale=decision.attitude_gain_scale)
            request=np.array(allocator.allocate(wrench.thrust,wrench.moment).achieved_wrench)
        actual+=(1-np.exp(-dt/lag))*(request-actual)
        R=rotation(q)
        acceleration=np.array([0,0,9.81])-R[:,2]*actual[0]/mass
        alpha=(actual[1:]-np.cross(omega,inertia*omega))/inertia
        p+=v*dt+.5*acceleration*dt*dt;v+=acceleration*dt
        q=unit(multiply(q,delta_quaternion(omega*dt+.5*alpha*dt*dt)));omega+=alpha*dt
        max_tilt=max(max_tilt,float(np.arccos(np.clip(R[2,2],-1,1))))
        max_drift=max(max_drift,float(np.linalg.norm(p-[0,0,-1])))
    return max_tilt,max_drift,supervisor.phase,float(np.linalg.norm(omega))


class AngularRecoveryTests(unittest.TestCase):
    def test_loss_epoch_hard_bound_and_timeout_do_not_resume_normal_commands(self):
        for s in (state(health=Health.LOST),state(omega=(0,5.,0))):
            with self.assertRaises(RuntimeError):AngularRecovery().decide(s)
        r=AngularRecovery();r.decide(state(omega=(0,1.,0)))
        with self.assertRaises(RuntimeError):r.decide(state(1.1,epoch=1))
        r=AngularRecovery();r.decide(state(omega=(0,1.,0)))
        with self.assertRaises(RuntimeError):r.decide(state(4.1,omega=(0,1.,0)))

    def test_fast_profile_is_explicit_and_does_not_relax_the_hard_bound(self):
        standard=AngularRecovery().decide(state(omega=(0,.6,0)))
        fast=AngularRecovery('fast').decide(state(omega=(0,.6,0)))
        self.assertEqual((standard.rate_gain_scale,standard.attitude_gain_scale),(2.,1.))
        self.assertEqual((fast.rate_gain_scale,fast.attitude_gain_scale),(3.,4.))
        with self.assertRaises(RuntimeError):AngularRecovery('fast').decide(state(omega=(0,2.6,0)))
        with self.assertRaises(ValueError):AngularRecovery('unsupported')

    def test_settling_requires_sustained_window_and_keeps_hold_target(self):
        r=AngularRecovery();d=r.decide(state(omega=(0,.6,0)))
        self.assertEqual(d.phase,'ANGULAR_RECOVERY')
        r.decide(state(1.1));r.decide(state(1.3,omega=(0,.3,0)))
        self.assertEqual(r.decide(state(1.4)).phase,'ANGULAR_RECOVERY')
        self.assertEqual(r.decide(state(1.76)).phase,'RECOVERY_HOLD')
        d=r.decide(replace(state(2.),position_ned=(.1,0.,-1.)))
        self.assertEqual(d.hold_position,(0.,0.,-1.))
        self.assertEqual(d.phase,'RECOVERY_HOLD')

    def test_nominal_rigid_body_impulses_with_80hz_control_and_20ms_motor_lag(self):
        for axis in (0,1,2):
            for impulse in (1.8,2.3):
                with self.subTest(axis=axis,impulse=impulse):
                    max_tilt,max_drift,phase,rate=simulate_recovery(axis,impulse)
                    self.assertLess(max_tilt,.35)
                    self.assertLess(max_drift,.8)
                    self.assertEqual(phase,'RECOVERY_HOLD')
                    self.assertLess(rate,.2)

    def test_recovery_with_delayed_sensors_slow_motors_and_inertia_uncertainty(self):
        for axis in (0,1,3):
            for lag in (.02,.04):
                for delay in (0.,.02):
                    for inertia_scale in (.8,1.2):
                        with self.subTest(axis=axis,lag=lag,delay=delay,inertia_scale=inertia_scale):
                            tilt,drift,phase,rate=simulate_recovery(axis,2.3,lag,delay,inertia_scale,'fast')
                            self.assertLess(tilt,.35)
                            self.assertLess(drift,.8)
                            self.assertEqual(phase,'RECOVERY_HOLD')
                            self.assertLess(rate,.2)


if __name__=='__main__':unittest.main()
