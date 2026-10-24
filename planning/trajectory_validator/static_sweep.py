"""Continuous box sweep against three-state voxels; static geometry only."""
from dataclasses import dataclass
from itertools import product
import math
import sys
from typing import Optional, Tuple

from interfaces.contracts import finite_vec3
from world_model.occupancy.grid import Occupancy


@dataclass(frozen=True)
class SweepBounds:
    # Must enclose the body/rotors in world axes at every permitted attitude.
    half_extents_m: Tuple[float, float, float]
    tracking_error_m: float
    acceleration_bound_m_s2: float
    max_voxel_tests: int = 20000
    max_segments: int = 256

    def __post_init__(self):
        finite_vec3(self.half_extents_m)
        if (any(v <= 0 for v in self.half_extents_m)
                or not all(math.isfinite(v) and v >= 0 for v in
                           (self.tracking_error_m, self.acceleration_bound_m_s2))
                or isinstance(self.max_voxel_tests, bool) or not isinstance(self.max_voxel_tests, int)
                or not 1 <= self.max_voxel_tests <= 1000000
                or isinstance(self.max_segments, bool) or not isinstance(self.max_segments, int)
                or not 1 <= self.max_segments <= 2048):
            raise ValueError('invalid swept-volume bounds or work budget')


@dataclass(frozen=True)
class StaticSweepResult:
    clear: bool
    reason: str
    voxel_tests: int
    intersected_voxels: int
    segment_index: Optional[int] = None
    voxel_key: Optional[Tuple[int, int, int]] = None
    contact_time: Optional[float] = None


def _contact_interval(start, end, lower, upper):
    """Slab intersection on [0,1], including tangency at closed voxel faces."""
    enter, leave = 0., 1.
    for a, b, lo, hi in zip(start, end, lower, upper):
        delta = b - a
        if delta == 0:
            if a < lo or a > hi:
                return None
        else:
            first, last = (lo - a) / delta, (hi - a) / delta
            enter, leave = max(enter, min(first, last)), min(leave, max(first, last))
            if enter > leave:
                return None
    return enter, leave


def check_static_sweep(trajectory, snapshot, now, epoch, bounds):
    """Check the remaining trajectory including its terminal pose.

    A world-axis box is swept continuously along each position chord. Expand it
    by the tracking bound plus a_max * sample_dt**2 / 8: the componentwise chord
    deviation bound for any C2 reference obeying that continuous acceleration
    bound. Sampled accelerations alone do not prove this producer guarantee.

    FREE must cover the whole contact interval. Occupied, absent, future, stale
    or mismatched cells reject. This neither creates FREE evidence nor checks
    dynamic objects, road boundaries, braking or executable control authority.
    """
    tested = intersected = 0
    def result(clear, reason, segment=None, key=None, stamp=None):
        return StaticSweepResult(clear, reason, tested, intersected, segment, key, stamp)
    if not trajectory.metadata.current(now, epoch):
        return result(False, 'TRAJECTORY_TIME_OR_EPOCH')
    if not snapshot.metadata.current(now, epoch):
        return result(False, 'MAP_TIME_OR_EPOCH')
    if trajectory.metadata.frame_id != 'world_ned' or snapshot.metadata.frame_id != 'world_ned':
        return result(False, 'WORLD_FRAME_REQUIRED')
    horizon = trajectory.metadata.valid_until
    if not snapshot.metadata.current(horizon, epoch):
        return result(False, 'MAP_EXPIRES_BEFORE_HORIZON')
    samples = trajectory.samples
    if len(samples) - 1 > bounds.max_segments or len(snapshot.cells) > 250000:
        return result(False, 'INPUT_BUDGET')
    if any(math.hypot(*sample.acceleration_ned) > bounds.acceleration_bound_m_s2 + 1e-9
           for sample in samples):
        return result(False, 'ACCELERATION_BOUND_CONTRADICTION')
    cells = {cell.key: cell for cell in snapshot.cells}
    resolution = snapshot.resolution_m
    for index, (first, second) in enumerate(zip(samples, samples[1:])):
        begin, finish = max(now, first.stamp), min(horizon, second.stamp)
        if begin > finish:
            continue
        duration = second.stamp - first.stamp
        if not math.isfinite(duration):
            return result(False, 'NUMERICAL_RANGE', index)
        # Use the original sample interval, even when checking only its suffix.
        padding = bounds.tracking_error_m + bounds.acceleration_bound_m_s2 * duration * duration / 8
        extents = tuple(v + padding for v in bounds.half_extents_m)
        if not all(math.isfinite(v) for v in extents):
            return result(False, 'NUMERICAL_RANGE', index)
        def at(stamp):
            fraction = (stamp - first.stamp) / duration
            return tuple((1 - fraction) * a + fraction * b
                         for a, b in zip(first.position_ned, second.position_ned))
        start, end = at(begin), at(finish)
        if not all(math.isfinite(a) and math.isfinite(b) and math.isfinite(b - a)
                   for a, b in zip(start, end)):
            return result(False, 'NUMERICAL_RANGE', index)
        # Round outward rather than turn a grazing contact into a clear result.
        scale = max(1., *(abs(v) for v in start + end + extents))
        roundoff = 32 * sys.float_info.epsilon * scale
        extents = tuple(v + roundoff for v in extents)
        ranges = []
        for a, b, extent in zip(start, end, extents):
            low, high = (min(a, b) - extent) / resolution, (max(a, b) + extent) / resolution
            if not all(math.isfinite(v) and abs(v) <= 2**45 for v in (low, high)):
                return result(False, 'NUMERICAL_RANGE', index)
            # Include the cell below an exact lower face: tangency is a hit.
            ranges.append(range(math.floor(low) - 1, math.floor(high) + 1))
        count = math.prod(len(axis) for axis in ranges)
        if tested + count > bounds.max_voxel_tests:
            return result(False, 'VOXEL_BUDGET', index)
        for key in product(*ranges):
            tested += 1
            lower = tuple(k * resolution - e for k, e in zip(key, extents))
            upper = tuple((k + 1) * resolution + e for k, e in zip(key, extents))
            if not all(math.isfinite(v) for v in lower + upper):
                return result(False, 'NUMERICAL_RANGE', index)
            interval = _contact_interval(start, end, lower, upper)
            if interval is None:
                continue
            intersected += 1
            enter, leave = (begin + t * (finish - begin) for t in interval)
            cell = cells.get(key)
            if cell is None or cell.status == Occupancy.UNKNOWN:
                return result(False, 'UNKNOWN_SPACE', index, key, enter)
            meta = cell.metadata
            if meta.frame_id != 'world_ned':
                return result(False, 'CELL_FRAME', index, key, enter)
            if meta.stamp > now or not meta.current(enter, epoch) or not meta.current(leave, epoch):
                return result(False, 'CELL_TIME_OR_EPOCH', index, key, enter)
            if cell.status != Occupancy.FREE:
                return result(False, 'OCCUPIED_SPACE', index, key, enter)
    return result(True, 'CLEAR_STATIC_MODEL')
