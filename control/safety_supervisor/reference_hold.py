"""Fresh-state local hold when isolated reference production becomes unavailable."""
from dataclasses import dataclass
import math
import numpy as np
from estimation.geometry import rotation
from interfaces.contracts import Health


@dataclass(frozen=True)
class ReferenceDecision:
    position: tuple
    velocity: tuple
    acceleration: tuple
    yaw: float
    yaw_rate: float
    phase: str
    allow_integral: bool


class ReferenceHold:
    def __init__(self):
        self.position=None
        self.yaw=0.
        self.epoch=None
        self.reason=None
        self.transitions=[]

    def select(self,state,packet,reason=None):
        if state.health!=Health.OK:raise RuntimeError('REFERENCE_HOLD_LOCALIZATION')
        if self.epoch is None:self.epoch=state.metadata.epoch
        if state.metadata.epoch!=self.epoch:raise RuntimeError('REFERENCE_HOLD_EPOCH')
        if packet is None and self.position is None:
            self.position=state.position_ned
            R=rotation(state.orientation_wxyz)
            self.yaw=math.atan2(R[1,0],R[0,0])
            self.reason=reason or 'REFERENCE_UNAVAILABLE'
            self.transitions.append({'stamp':state.metadata.stamp,'from':'NORMAL','to':'REFERENCE_HOLD',
                                     'reason':self.reason})
        if self.position is not None:
            if np.linalg.norm(np.array(state.position_ned)-self.position)>.8:
                raise RuntimeError('REFERENCE_HOLD_POSITION_BOUND')
            return ReferenceDecision(self.position,(0.,0.,0.),(0.,0.,0.),self.yaw,0.,'REFERENCE_HOLD',False)
        return ReferenceDecision(packet.position,packet.velocity,packet.acceleration,packet.yaw,
                                 packet.yaw_rate,'NORMAL',True)
