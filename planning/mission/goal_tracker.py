"""Stable official end-goal observations; does not infer mission completion."""
from dataclasses import dataclass
import math
import threading

from interfaces.contracts import finite_vec3


@dataclass(frozen=True)
class GoalSnapshot:
    status: str
    position: tuple = None
    epoch: int = 0
    observation_count: int = 0
    last_arrival_wall: float = None
    source_stamp: float = None
    source_frame: str = ''
    # Goal updates alone do not confirm an official stage completion or score.
    official_stage_confirmed: bool = False
    control_eligible: bool = False


class GoalTracker:
    """Debounce /end_goal observations with wall freshness and epoch reset.

    The message may have stamp zero; never replace it with a fresh IMU stamp.
    A changed or unstable candidate suppresses the prior goal until confirmed.
    """
    def __init__(self, epoch=0, confirmations=3, tolerance_m=.05, wall_ttl_s=.5):
        if (isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0
                or isinstance(confirmations, bool) or not isinstance(confirmations, int) or not 2 <= confirmations <= 10
                or not math.isfinite(tolerance_m) or not 0 < tolerance_m <= 1
                or not math.isfinite(wall_ttl_s) or not 0 < wall_ttl_s <= 2):
            raise ValueError('invalid goal tracker settings')
        self.epoch, self.confirmations = epoch, confirmations
        self.tolerance_m, self.wall_ttl_s = tolerance_m, wall_ttl_s
        self._lock = threading.RLock()
        self._candidate = self._stable = None
        self._count = 0
        self._arrival = self._stamp = None
        self._last_timed_stamp = None
        self._source_frame = ''

    def reset(self, epoch):
        with self._lock:
            if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch <= self.epoch:
                raise ValueError('goal epoch must increase')
            self.epoch = epoch
            self._candidate = self._stable = None
            self._count = 0; self._arrival = self._stamp = None; self._last_timed_stamp = None; self._source_frame = ''

    def observe(self, position, source_stamp, source_frame, arrival_wall, imu_stamp, epoch):
        with self._lock:
            finite_vec3(position)
            if (epoch != self.epoch or not isinstance(source_frame, str)
                    or not all(math.isfinite(t) and t >= 0 for t in (source_stamp, arrival_wall, imu_stamp))
                    or imu_stamp == 0 or (self._arrival is not None and arrival_wall <= self._arrival)
                    or (source_stamp != 0 and not 0 <= imu_stamp - source_stamp <= .3)
                    or (source_stamp != 0 and self._last_timed_stamp is not None
                        and source_stamp <= self._last_timed_stamp)):
                raise ValueError('invalid goal source, clock, or epoch')
            if (self._arrival is not None and arrival_wall - self._arrival >= self.wall_ttl_s) or source_frame != self._source_frame:
                self._candidate = self._stable = None; self._count = 0
            if self._candidate is None or math.dist(position, self._candidate) > self.tolerance_m:
                self._candidate = position; self._count = 1
            else:
                self._count += 1
            self._arrival, self._stamp, self._source_frame = arrival_wall, source_stamp, source_frame
            if source_stamp != 0:
                self._last_timed_stamp = source_stamp
            if self._count >= self.confirmations:
                self._stable = self._candidate
            return self.snapshot(arrival_wall, epoch, True)

    def snapshot(self, wall_now, epoch, imu_fresh):
        with self._lock:
            if (epoch != self.epoch or not imu_fresh or not math.isfinite(wall_now)
                    or self._arrival is None or not 0 <= wall_now - self._arrival < self.wall_ttl_s):
                return GoalSnapshot('UNAVAILABLE', epoch=self.epoch)
            if self._count < self.confirmations:
                return GoalSnapshot('PENDING', epoch=self.epoch, observation_count=self._count,
                                    last_arrival_wall=self._arrival, source_stamp=self._stamp,
                                    source_frame=self._source_frame)
            return GoalSnapshot('OBSERVED', self._stable, self.epoch, self._count,
                                self._arrival, self._stamp, self._source_frame)
