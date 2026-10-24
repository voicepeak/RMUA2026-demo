"""Quintic reference candidates, continuous kinematic bounds and static map checks."""
from dataclasses import dataclass, replace
import math
from typing import Optional, Tuple

from interfaces.contracts import Health, Metadata, Trajectory, TrajectorySample, finite_vec3
from planning.trajectory_validator.static_sweep import SweepBounds, check_static_sweep


def _norm(value):
    return math.hypot(*value)


def _subtract(first, second):
    return tuple(a - b for a, b in zip(first, second))


def _evaluate(controls, fraction):
    values = controls
    while len(values) > 1:
        values = tuple(tuple((1 - fraction) * a + fraction * b for a, b in zip(left, right))
                       for left, right in zip(values, values[1:]))
    return values[0]


def _derivative(controls, duration):
    factor = (len(controls) - 1) / duration
    return tuple(tuple(factor * (b - a) for a, b in zip(left, right))
                 for left, right in zip(controls, controls[1:]))


def _continuous_bound(controls, depth=3):
    # Each Bezier curve lies in its control hull. Subdivision tightens the bound
    # while retaining coverage of all times, including between emitted samples.
    if depth == 0:
        return max(_norm(value) for value in controls)
    levels = controls
    left, right = [levels[0]], [levels[-1]]
    while len(levels) > 1:
        levels = tuple(tuple(.5 * a + .5 * b for a, b in zip(first, second))
                       for first, second in zip(levels, levels[1:]))
        left.append(levels[0]); right.append(levels[-1])
    return max(_continuous_bound(tuple(left), depth - 1),
               _continuous_bound(tuple(reversed(right)), depth - 1))


@dataclass(frozen=True)
class QuinticReference:
    stamp: float
    duration_s: float
    controls: tuple
    yaw: float

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.stamp, self.duration_s, self.yaw))
                or self.stamp < 0 or not .05 <= self.duration_s <= 10
                or not math.isfinite(self.end_stamp) or self.end_stamp <= self.stamp
                or not isinstance(self.controls, tuple) or len(self.controls) != 6):
            raise ValueError('invalid quintic reference')
        for point in self.controls:
            finite_vec3(point)

    @classmethod
    def to_rest(cls, initial, target, duration_s):
        finite_vec3(target)
        elapsed = initial.stamp + duration_s - initial.stamp
        p, v, a = initial.position_ned, initial.velocity_ned, initial.acceleration_ned
        first = tuple(x + elapsed * speed / 5 for x, speed in zip(p, v))
        second = tuple(x + 2 * elapsed * speed / 5 + elapsed**2 * accel / 20
                       for x, speed, accel in zip(p, v, a))
        return cls(initial.stamp, duration_s, (p, first, second, target, target, target), initial.yaw)

    @property
    def end_stamp(self):
        return self.stamp + self.duration_s

    def derivative_bounds(self):
        elapsed = self.end_stamp - self.stamp
        velocity = _derivative(self.controls, elapsed)
        acceleration = _derivative(velocity, elapsed)
        for value in velocity + acceleration:
            finite_vec3(value)
        # Small outward padding covers floating-point hull arithmetic.
        return tuple(0. if value == 0 else value + 1e-12 * max(1., value)
                     for value in (_continuous_bound(velocity), _continuous_bound(acceleration)))

    def sample(self, stamp):
        if not math.isfinite(stamp) or not self.stamp <= stamp <= self.end_stamp:
            raise ValueError('reference query outside segment')
        elapsed = self.end_stamp - self.stamp
        fraction = (stamp - self.stamp) / elapsed
        velocity = _derivative(self.controls, elapsed)
        return TrajectorySample(stamp, _evaluate(self.controls, fraction),
            _evaluate(velocity, fraction), _evaluate(_derivative(velocity, elapsed), fraction), self.yaw)


@dataclass(frozen=True)
class LocalPlannerLimits:
    sweep: SweepBounds
    speed_limit_m_s: float
    segment_durations_s: tuple = (2., 4.)
    lateral_offsets_m: tuple = (1., 2.)
    sample_dt_s: float = .1
    lease_s: float = .08
    max_attempts: int = 16
    max_start_velocity_error_m_s: float = .2

    def __post_init__(self):
        if (not isinstance(self.sweep, SweepBounds)
                or not math.isfinite(self.speed_limit_m_s) or self.speed_limit_m_s <= 0
                or not isinstance(self.segment_durations_s, tuple) or not 1 <= len(self.segment_durations_s) <= 8
                or any(not math.isfinite(v) or not .05 <= v <= 10 for v in self.segment_durations_s)
                or not isinstance(self.lateral_offsets_m, tuple) or len(self.lateral_offsets_m) > 4
                or any(not math.isfinite(v) or not 0 < v <= 10 for v in self.lateral_offsets_m)
                or not math.isfinite(self.sample_dt_s) or not .005 <= self.sample_dt_s <= .1
                or not math.isfinite(self.lease_s) or not .01 <= self.lease_s <= .2
                or not math.isfinite(self.max_start_velocity_error_m_s) or self.max_start_velocity_error_m_s < 0
                or isinstance(self.max_attempts, bool) or not isinstance(self.max_attempts, int)
                or not 1 <= self.max_attempts <= 64):
            raise ValueError('invalid local planner limits')


@dataclass(frozen=True)
class CandidateCheck:
    offset_m: float
    segment_duration_s: float
    reason: str
    speed_bound_m_s: float = 0.
    acceleration_bound_m_s2: float = 0.
    voxel_tests: int = 0


@dataclass(frozen=True)
class LocalPlan:
    trajectory: Optional[Trajectory]
    pieces: tuple
    checks: Tuple[CandidateCheck, ...]
    reason: str
    source_state: Metadata
    source_map: Metadata

    def reference_at(self, stamp, epoch):
        if self.trajectory is None or not self.trajectory.metadata.current(stamp, epoch):
            raise ValueError('local plan lease is not current')
        for piece in self.pieces:
            if piece.stamp <= stamp <= piece.end_stamp:
                return piece.sample(stamp)
        raise ValueError('reference has no covering segment')


def plan_static_local(state, initial_reference, goal, snapshot, now, epoch, limits):
    """Select a fully checked stop-at-waypoint reference with a short output lease.

    Candidate paths include direct and left/right midpoints relative to the
    current goal, without seed maps. All pieces preserve endpoint p/v/a; each
    waypoint ends at rest. The WHOLE stopping reference is checked in the map,
    then only a source-bounded short lease is issued. This is a static/kinematic
    candidate, not proof of actual braking, dynamic clearance or control health.
    """
    finite_vec3(goal)
    def result(reason, trajectory=None, pieces=(), checks=()):
        return LocalPlan(trajectory, pieces, tuple(checks), reason, state.metadata, snapshot.metadata)
    if not state.metadata.current(now, epoch) or state.health != Health.OK:
        return result('STATE_UNAVAILABLE')
    if state.metadata.frame_id != 'world_ned' or snapshot.metadata.frame_id != 'world_ned':
        return result('WORLD_FRAME_REQUIRED')
    if not snapshot.metadata.current(now, epoch):
        return result('MAP_UNAVAILABLE')
    if (initial_reference.stamp != now
            or _norm(_subtract(initial_reference.position_ned, state.position_ned)) > limits.sweep.tracking_error_m + 1e-9
            or _norm(_subtract(initial_reference.velocity_ned, state.velocity_ned)) > limits.max_start_velocity_error_m_s):
        return result('REFERENCE_START_MISMATCH')
    start = initial_reference.position_ned
    delta = _subtract(goal, start)
    horizontal = math.hypot(delta[0], delta[1])
    routes = [(0., (goal,))]
    if horizontal > 1e-6:
        perpendicular = (-delta[1] / horizontal, delta[0] / horizontal, 0.)
        middle = tuple(.5 * a + .5 * b for a, b in zip(start, goal))
        for magnitude in limits.lateral_offsets_m:
            for offset in (magnitude, -magnitude):
                via = tuple(value + offset * axis for value, axis in zip(middle, perpendicular))
                routes.append((offset, (via, goal)))
    best = None
    checks = []
    for offset, waypoints in routes:
        for duration in limits.segment_durations_s:
            if len(checks) >= limits.max_attempts:
                break
            end_stamp = now + duration * len(waypoints)
            if not snapshot.metadata.current(end_stamp, epoch):
                checks.append(CandidateCheck(offset, duration, 'MAP_STOP_HORIZON'))
                continue
            steps = math.ceil(duration / limits.sample_dt_s)
            if steps * len(waypoints) > limits.sweep.max_segments:
                checks.append(CandidateCheck(offset, duration, 'SAMPLE_BUDGET'))
                continue
            pieces, samples = [], []
            current = initial_reference
            try:
                for waypoint in waypoints:
                    piece = QuinticReference.to_rest(current, waypoint, duration)
                    pieces.append(piece)
                    first_index = 0 if not samples else 1
                    samples.extend(piece.sample(piece.end_stamp if i == steps else
                        piece.stamp + i * (piece.end_stamp - piece.stamp) / steps)
                        for i in range(first_index, steps + 1))
                    current = piece.sample(piece.end_stamp)
                bounds = [piece.derivative_bounds() for piece in pieces]
                speed = max(v[0] for v in bounds)
                acceleration = max(v[1] for v in bounds)
                if not all(math.isfinite(v) for v in (speed, acceleration)):
                    raise ValueError('nonfinite derivative bounds')
            except (ValueError, OverflowError):
                checks.append(CandidateCheck(offset, duration, 'NUMERICAL_RANGE'))
                continue
            if speed > limits.speed_limit_m_s or acceleration > limits.sweep.acceleration_bound_m_s2:
                checks.append(CandidateCheck(offset, duration, 'KINEMATIC_LIMIT', speed, acceleration))
                continue
            full = Trajectory(Metadata(now, pieces[-1].end_stamp, epoch, 'static_local_quintic'), tuple(samples))
            sweep = check_static_sweep(full, snapshot, now, epoch, limits.sweep)
            checks.append(CandidateCheck(offset, duration, sweep.reason, speed, acceleration, sweep.voxel_tests))
            if not sweep.clear:
                continue
            length = sum(_norm(_subtract(b, a)) for piece in pieces
                         for a, b in zip(piece.controls, piece.controls[1:]))
            cost = length + .05 * duration * len(pieces)
            if best is None or cost < best[0]:
                best = cost, full, tuple(pieces)
    if best is None:
        return result('NO_STATIC_CANDIDATE', checks=checks)
    _, full, pieces = best
    lease_end = min(now + limits.lease_s, state.metadata.valid_until, full.metadata.valid_until)
    leased = replace(full, metadata=replace(full.metadata, valid_until=lease_end))
    return result('STATIC_CANDIDATE', leased, pieces, checks)
