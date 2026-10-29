"""Explicit local experiment injection; disabled in all normal sensor adapters."""
from dataclasses import dataclass
import math


@dataclass
class LocalSensorFault:
    kind: str
    start_stamp: float
    duration: float
    injected_samples: int = 0

    def __post_init__(self):
        if (self.kind not in ('GPS_ZERO', 'IMU_DROP', 'IMU_ROLLBACK', 'IMU_RATE_SPIKE')
                or not math.isfinite(self.start_stamp) or self.start_stamp <= 0
                or not math.isfinite(self.duration) or not 0 < self.duration <= 1.):
            raise ValueError('invalid local sensor fault')

    def active(self, stamp):
        return self.start_stamp <= stamp < self.start_stamp+self.duration

    def affects(self, kind, stamp):
        if self.kind == kind and self.active(stamp):
            self.injected_samples += 1
            return True
        return False
