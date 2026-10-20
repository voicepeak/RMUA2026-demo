"""Experimental local attitude recovery, requiring fresh healthy localization.

This does not reverse a competition collision or authorize continued mission
flight. After settling, hold at the trigger position until the experiment ends.
"""
from dataclasses import dataclass
import math
import numpy as np
from estimation.geometry import rotation
from interfaces.contracts import Health


@dataclass(frozen=True)
class AngularDecision:
    phase: str
    hold_position: tuple
    hold_yaw: float
    rate_gain_scale: float
    attitude_gain_scale: float


class AngularRecovery:
    def __init__(self,profile='standard'):
        if profile not in ('standard','fast'):raise ValueError('unknown angular recovery profile')
        self.profile=profile
        self.phase='NORMAL'
        self.position=None
        self.yaw=0.
        self.trigger_stamp=None
        self.settled_since=None
        self.previous_stamp=None
        self.epoch=None
        self.transitions=[]

    def decide(self,state):
        stamp=state.metadata.stamp
        if state.health!=Health.OK:raise RuntimeError('ANGULAR_RECOVERY_LOCALIZATION')
        if self.epoch is not None and state.metadata.epoch!=self.epoch:
            raise RuntimeError('ANGULAR_RECOVERY_EPOCH')
        if self.previous_stamp is not None and stamp<self.previous_stamp:
            raise RuntimeError('ANGULAR_RECOVERY_CLOCK')
        self.previous_stamp=stamp
        self.epoch=state.metadata.epoch
        R=rotation(state.orientation_wxyz)
        tilt=math.acos(float(np.clip(R[2,2],-1,1)))
        rate=float(np.linalg.norm(state.angular_velocity_body))
        if tilt>.35 or rate>2.5:raise RuntimeError('ANGULAR_RECOVERY_HARD_BOUND')
        previous=self.phase
        if self.phase=='NORMAL' and (rate>.35 or tilt>.18):
            self.phase='ANGULAR_RECOVERY'
            self.position=state.position_ned
            self.yaw=math.atan2(R[1,0],R[0,0])
            self.trigger_stamp=stamp
        if self.phase!='NORMAL':
            if np.linalg.norm(np.array(state.position_ned)-self.position)>.8:
                raise RuntimeError('ANGULAR_RECOVERY_POSITION_BOUND')
            settled=(rate<.2 and tilt<.12 and np.linalg.norm(state.velocity_ned)<.25)
            if self.phase=='ANGULAR_RECOVERY':
                if settled:
                    if self.settled_since is None:self.settled_since=stamp
                    if stamp-self.settled_since>=.35:self.phase='RECOVERY_HOLD'
                else:self.settled_since=None
                if self.phase!='RECOVERY_HOLD' and stamp-self.trigger_stamp>3.:
                    raise RuntimeError('ANGULAR_RECOVERY_TIMEOUT')
            elif rate>.35 or tilt>.18:
                # A second disturbance gets a new bounded recovery interval,
                # while keeping the original safe hold target.
                self.phase='ANGULAR_RECOVERY'
                self.trigger_stamp=stamp
                self.settled_since=None
        if previous!=self.phase:
            self.transitions.append({'stamp':stamp,'from':previous,'to':self.phase,
                'rate_rad_s':rate,'tilt_rad':tilt})
        return AngularDecision(self.phase,self.position or state.position_ned,self.yaw,
                               (3. if self.profile=='fast' else 2.) if self.phase=='ANGULAR_RECOVERY' else 1.,
                               4. if self.phase=='ANGULAR_RECOVERY' and self.profile=='fast' else 1.)
