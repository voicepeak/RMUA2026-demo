"""ROS-independent immutable contracts. Time is simulator time, coordinates NED."""
from dataclasses import dataclass
from enum import Enum
import math
from typing import Tuple

Vec3 = Tuple[float, float, float]


class Health(str, Enum):
    OK = "OK"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


@dataclass(frozen=True)
class Metadata:
    stamp: float
    valid_until: float
    epoch: int
    source: str
    frame_id: str = "world_ned"
    covariance_or_bounds: Tuple[float, ...] = ()

    def __post_init__(self):
        if not all(math.isfinite(v) for v in (self.stamp, self.valid_until)):
            raise ValueError("timestamps must be finite")
        if self.stamp < 0 or self.valid_until < self.stamp or self.epoch < 0:
            raise ValueError("invalid validity interval or epoch")
        if not self.source or not self.frame_id:
            raise ValueError("source and frame required")
        if not isinstance(self.covariance_or_bounds, tuple) or not all(
                math.isfinite(v) for v in self.covariance_or_bounds):
            raise ValueError("bounds must be an immutable finite tuple")

    def current(self, now: float, epoch: int) -> bool:
        return (math.isfinite(now) and self.epoch == epoch
                and self.stamp <= now < self.valid_until)


def finite_vec3(value):
    if not isinstance(value, tuple) or len(value) != 3 or not all(math.isfinite(v) for v in value):
        raise ValueError("expected immutable finite Vec3")


@dataclass(frozen=True)
class StateEstimate:
    metadata: Metadata
    position_ned: Vec3
    velocity_ned: Vec3
    orientation_wxyz: Tuple[float, float, float, float]
    angular_velocity_body: Vec3
    health: Health

    def __post_init__(self):
        finite_vec3(self.position_ned)
        finite_vec3(self.velocity_ned)
        finite_vec3(self.angular_velocity_body)
        q = self.orientation_wxyz
        if (not isinstance(q, tuple) or len(q) != 4 or not all(math.isfinite(v) for v in q)
                or abs(sum(v * v for v in q) - 1.0) > 1e-6):
            raise ValueError("orientation must be an immutable unit quaternion (w,x,y,z)")
        if not isinstance(self.health, Health):
            raise ValueError("invalid health")


@dataclass(frozen=True)
class TrajectorySample:
    stamp: float
    position_ned: Vec3
    velocity_ned: Vec3
    acceleration_ned: Vec3
    yaw: float

    def __post_init__(self):
        for value in (self.position_ned, self.velocity_ned, self.acceleration_ned):
            finite_vec3(value)
        if not math.isfinite(self.stamp) or not math.isfinite(self.yaw):
            raise ValueError("nonfinite trajectory")


@dataclass(frozen=True)
class Trajectory:
    metadata: Metadata
    samples: Tuple[TrajectorySample, ...]

    def __post_init__(self):
        if not isinstance(self.samples, tuple) or len(self.samples) < 2:
            raise ValueError("trajectory needs at least two immutable samples")
        times = tuple(s.stamp for s in self.samples)
        if any(a >= b for a, b in zip(times, times[1:])):
            raise ValueError("trajectory timestamps must increase")
        if times[0] > self.metadata.stamp or times[-1] < self.metadata.valid_until:
            raise ValueError("samples must cover the validity interval")
