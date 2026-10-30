"""Limited GPS outage bridge for bounded local hover only, not indoor navigation."""
from dataclasses import dataclass
import math
from interfaces.contracts import Health


@dataclass(frozen=True)
class HoverDecision:
    phase: str
    position: tuple
    velocity: tuple
    acceleration: tuple
    allow_integral: bool


class HoverRecovery:
    def __init__(self, maximum_gps_age=1.):
        if not math.isfinite(maximum_gps_age) or maximum_gps_age <= .3:
            raise ValueError('invalid hover outage bound')
        self.maximum_gps_age = maximum_gps_age
        self.hold_position = None
        self.phase = 'NORMAL'
        self.transitions = []
        self.fault = None

    def fail(self, reason):
        self.fault = self.fault or reason
        raise RuntimeError(self.fault)

    def decide(self, state, gps_age, nominal):
        if self.fault:
            self.fail(self.fault)
        if not math.isfinite(gps_age) or gps_age < 0 or state.health == Health.LOST:
            self.fail('HOVER_LOCALIZATION_LOST')
        if gps_age > self.maximum_gps_age:
            self.fail('HOVER_OUTAGE_BOUND_EXCEEDED')
        previous = self.phase
        if state.health == Health.OK and gps_age <= .3:
            self.phase = 'NORMAL'
            self.hold_position = None
            decision = HoverDecision(self.phase,*nominal,True)
        elif .3 < gps_age <= self.maximum_gps_age:
            self.phase = 'GPS_BRIDGE'
            if self.hold_position is None:
                self.hold_position = state.position_ned
            decision = HoverDecision(self.phase,self.hold_position,(0.,0.,0.),(0.,0.,0.),False)
        else:
            self.fail('HOVER_OUTAGE_BOUND_EXCEEDED')
        if previous != self.phase:
            self.transitions.append({'stamp':state.metadata.stamp,'from':previous,'to':self.phase})
        return decision
