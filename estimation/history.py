"""Bounded historical ESKF poses, separate from current control-state leases."""
from collections import deque
from dataclasses import dataclass, field
import math
import threading

import numpy as np

from estimation.geometry import multiply, unit
from interfaces.contracts import Health, finite_vec3


class HistoryUnavailable(ValueError):
    pass


@dataclass(frozen=True)
class HistoricalPose:
    stamp: float
    epoch: int
    position_ned: tuple
    orientation_wxyz: tuple
    bracket_stamps: tuple
    bracket_covariance_diagonals: tuple
    interpolation_fraction: float
    position_correction_residual_m: float
    rotation_correction_residual_rad: float
    frame_id: str = field(default='world_ned', init=False)
    uncertainty_calibrated: bool = field(default=False, init=False)
    current_control_eligible: bool = field(default=False, init=False)

    def __post_init__(self):
        finite_vec3(self.position_ned)
        q = self.orientation_wxyz
        if (not isinstance(q, tuple) or len(q) != 4 or not all(math.isfinite(x) for x in q)
                or abs(sum(x * x for x in q) - 1) > 1e-6
                or not math.isfinite(self.stamp) or self.stamp < 0 or self.epoch < 0
                or not isinstance(self.bracket_stamps, tuple) or len(self.bracket_stamps) != 2
                or not all(math.isfinite(x) for x in self.bracket_stamps)
                or not self.bracket_stamps[0] <= self.stamp <= self.bracket_stamps[1]
                or not math.isfinite(self.interpolation_fraction) or not 0 <= self.interpolation_fraction <= 1
                or not isinstance(self.bracket_covariance_diagonals, tuple)
                or len(self.bracket_covariance_diagonals) != 2
                or any(not isinstance(d, tuple) or len(d) != 15
                       or not all(math.isfinite(x) and x >= 0 for x in d) for d in self.bracket_covariance_diagonals)
                or not all(math.isfinite(x) and x >= 0 for x in
                           (self.position_correction_residual_m, self.rotation_correction_residual_rad))):
            raise ValueError('invalid immutable historical pose')


def slerp(first, second, fraction):
    a, b = unit(first), unit(second)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    dot = min(1., max(-1., dot))
    if dot > .9995:
        return unit((1 - fraction) * a + fraction * b)
    angle = math.acos(dot)
    return unit((math.sin((1 - fraction) * angle) * a + math.sin(fraction * angle) * b) / math.sin(angle))


class PoseHistory:
    def __init__(self, epoch=0, horizon_s=1., capacity=256, max_bracket_s=.03):
        if (not isinstance(epoch, int) or epoch < 0 or not math.isfinite(horizon_s)
                or not .2 <= horizon_s <= 2 or not isinstance(capacity, int)
                or not 4 <= capacity <= 512 or not math.isfinite(max_bracket_s)
                or not .005 <= max_bracket_s <= .05):
            raise ValueError('invalid historical pose limits')
        self.epoch, self.horizon_s = epoch, horizon_s
        self.capacity, self.max_bracket_s = capacity, max_bracket_s
        self._states = deque()
        self._arrivals = deque()
        self._last_arrival = None
        self._lock = threading.RLock()

    def reset(self, epoch):
        with self._lock:
            if not isinstance(epoch, int) or epoch <= self.epoch:
                raise ValueError('history epoch must increase')
            self.epoch = epoch
            self._states.clear()
            self._arrivals.clear()
            self._last_arrival = None

    def append(self, state, arrival_wall):
        with self._lock:
            if state.metadata.epoch != self.epoch or state.metadata.frame_id != 'world_ned':
                raise ValueError('historical source frame or epoch mismatch')
            if (not math.isfinite(arrival_wall) or arrival_wall < 0
                    or (self._last_arrival is not None and arrival_wall < self._last_arrival)):
                raise ValueError('invalid historical source arrival')
            bounds = state.metadata.covariance_or_bounds
            if len(bounds) != 225 or any(bounds[i * 15 + i] < 0 for i in range(15)):
                raise ValueError('history requires a finite 15-state ESKF covariance')
            if self._states and state.metadata.stamp <= self._states[-1].metadata.stamp:
                return False  # Duplicates never refresh the wall deadline.
            self._states.append(state)
            self._arrivals.append(arrival_wall)
            self._last_arrival = arrival_wall
            cutoff = state.metadata.stamp - self.horizon_s
            while len(self._states) > 1 and self._states[1].metadata.stamp < cutoff:
                self._states.popleft()
                self._arrivals.popleft()
            while len(self._states) > self.capacity:
                self._states.popleft()
                self._arrivals.popleft()
            return True

    def snapshot_sources(self):
        with self._lock:
            return tuple(zip(self._states, self._arrivals))

    def lookup(self, stamp, now, wall_now, epoch):
        with self._lock:
            if not all(math.isfinite(x) and x >= 0 for x in (stamp, now, wall_now)):
                raise HistoryUnavailable('INVALID_QUERY_TIME')
            if epoch != self.epoch:
                raise HistoryUnavailable('OTHER_EPOCH')
            if not self._states:
                raise HistoryUnavailable('NO_HISTORY')
            latest = self._states[-1]
            if not latest.metadata.current(now, epoch):
                raise HistoryUnavailable('LATEST_STATE_TIME')
            if not 0 <= wall_now - self._last_arrival <= .1:
                raise HistoryUnavailable('STALE_STATE_ARRIVAL')
            if latest.health != Health.OK:
                raise HistoryUnavailable('LATEST_STATE_HEALTH')
            if not 0 <= now - stamp <= self.horizon_s:
                raise HistoryUnavailable('OUTSIDE_HISTORY_HORIZON')
            lower = upper = None
            for state in self._states:
                if state.metadata.stamp <= stamp:
                    lower = state
                if state.metadata.stamp >= stamp:
                    upper = state
                    break
            if lower is None or upper is None:
                raise HistoryUnavailable('NO_BRACKET')
            if lower.health != Health.OK or upper.health != Health.OK:
                raise HistoryUnavailable('BRACKET_HEALTH')
            gap = upper.metadata.stamp - lower.metadata.stamp
            if gap > self.max_bracket_s or lower.metadata.valid_until <= stamp:
                raise HistoryUnavailable('BRACKET_GAP')
            fraction = (stamp - lower.metadata.stamp) / gap if gap else 0.
            p0, p1 = np.array(lower.position_ned), np.array(upper.position_ned)
            predicted = .5 * (np.array(lower.velocity_ned) + upper.velocity_ned) * gap
            position_residual = float(np.linalg.norm(p1 - p0 - predicted))
            q0, q1 = unit(lower.orientation_wxyz), unit(upper.orientation_wxyz)
            delta = unit(multiply(q0 * (1., -1., -1., -1.), q1))
            if delta[0] < 0:
                delta = -delta
            length = float(np.linalg.norm(delta[1:]))
            angle = delta[1:] * (2 * math.atan2(length, float(delta[0])) / length) if length > 1e-12 else np.zeros(3)
            expected = .5 * (np.array(lower.angular_velocity_body) + upper.angular_velocity_body) * gap
            rotation_residual = float(np.linalg.norm(angle - expected))
            # Local plausibility guards, not certified interpolation error bounds.
            if position_residual > .1 or rotation_residual > .05:
                raise HistoryUnavailable('ESTIMATE_DISCONTINUITY')
            diagonals = tuple(tuple(s.metadata.covariance_or_bounds[i * 15 + i] for i in range(15))
                              for s in (lower, upper))
            return HistoricalPose(stamp, epoch, tuple((1 - fraction) * p0 + fraction * p1),
                tuple(slerp(q0, q1, fraction)), (lower.metadata.stamp, upper.metadata.stamp), diagonals,
                fraction, position_residual, rotation_residual)
