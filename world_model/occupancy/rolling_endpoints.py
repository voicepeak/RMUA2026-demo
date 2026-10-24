"""Nominal world-frame endpoint observations and bounded occupied-only fusion."""
from dataclasses import dataclass, field, replace
import math
import threading

import numpy as np

from estimation.geometry import rotation
from interfaces.contracts import Metadata, finite_vec3
from world_model.occupancy.grid import Cell, Occupancy, OccupancySnapshot


@dataclass(frozen=True)
class WorldEndpointObservation:
    metadata: Metadata
    source_metadata: Metadata
    world_points: np.ndarray
    sensor_origin_ned: tuple
    pose_bracket_stamps: tuple
    metric_calibration_verified: bool = field(default=False, init=False)
    per_point_deskew_applied: bool = field(default=False, init=False)
    current_control_eligible: bool = field(default=False, init=False)

    def __post_init__(self):
        points = np.asarray(self.world_points, dtype=np.float64)
        finite_vec3(self.sensor_origin_ned)
        if (points.ndim != 2 or points.shape[1] != 3 or len(points) > 250000
                or not np.isfinite(points).all() or self.metadata.frame_id != 'world_ned'
                or self.source_metadata.frame_id != 'lidar'
                or (self.metadata.stamp, self.metadata.valid_until, self.metadata.epoch)
                   != (self.source_metadata.stamp, self.source_metadata.valid_until, self.source_metadata.epoch)
                or not isinstance(self.pose_bracket_stamps, tuple) or len(self.pose_bracket_stamps) != 2
                or not all(math.isfinite(v) for v in self.pose_bracket_stamps)
                or not self.pose_bracket_stamps[0] <= self.metadata.stamp <= self.pose_bracket_stamps[1]):
            raise ValueError('invalid nominal world endpoint observation')
        immutable = np.frombuffer(points.tobytes(), dtype=np.float64).reshape(-1, 3)
        object.__setattr__(self, 'world_points', immutable)


def world_endpoints(cloud, pose, sensor_pose, max_range_m=30.):
    """Apply mount and estimated scan-header pose, preserving the source lease.

    This is NOT per-return scan deskew or verified metric calibration. Invalid,
    zero and maximum-range returns do not provide hit or free-space evidence.
    """
    if (cloud.metadata.frame_id != 'lidar' or pose.frame_id != 'world_ned'
            or pose.stamp != cloud.metadata.stamp or pose.epoch != cloud.metadata.epoch
            or pose.position_correction_residual_m > .1 or pose.rotation_correction_residual_rad > .05
            or not math.isfinite(max_range_m) or not 0 < max_range_m <= 30):
        raise ValueError('incompatible scan and historical pose')
    ranges = np.hypot(np.hypot(cloud.points[:, 0], cloud.points[:, 1]), cloud.points[:, 2])
    hits = cloud.points[(ranges > 0) & (ranges < max_range_m)]
    body_to_world = rotation(pose.orientation_wxyz)
    world = sensor_pose.to_body(hits) @ body_to_world.T + pose.position_ned
    origin = sensor_pose.to_body(np.zeros((1, 3)))[0] @ body_to_world.T + pose.position_ned
    metadata = replace(cloud.metadata, frame_id='world_ned', source='nominal_lidar_header_pose_no_deskew')
    return WorldEndpointObservation(metadata, cloud.metadata, world, tuple(origin), pose.bracket_stamps)


class RollingEndpointMap:
    """Thread-safe occupied-only map. Expired/cropped cells become UNKNOWN."""
    def __init__(self, epoch=0, resolution_m=.25, capacity=50000, window_radius_m=40.):
        if (isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0
                or not math.isfinite(resolution_m) or not .05 <= resolution_m <= 2
                or isinstance(capacity, bool) or not isinstance(capacity, int) or not 1 <= capacity <= 100000
                or not math.isfinite(window_radius_m) or not 1 <= window_radius_m <= 100):
            raise ValueError('invalid rolling endpoint map limits')
        self.epoch, self.resolution_m = epoch, resolution_m
        self.capacity, self.window_radius_m = capacity, window_radius_m
        self._cells = {}
        self._latest = None
        self._last_now = None
        self._lock = threading.RLock()

    def reset(self, epoch):
        with self._lock:
            if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch <= self.epoch:
                raise ValueError('map epoch must increase')
            self.epoch = epoch
            self._cells.clear()
            self._latest = self._last_now = None

    def _check_time(self, now, epoch):
        if (not math.isfinite(now) or now < 0 or epoch != self.epoch
                or (self._last_now is not None and now < self._last_now)):
            raise ValueError('map clock or epoch mismatch')

    def integrate(self, observation, now, epoch):
        with self._lock:
            self._check_time(now, epoch)
            meta = observation.metadata
            if not meta.current(now, epoch) or meta.frame_id != 'world_ned':
                raise ValueError('endpoint observation is not current')
            if observation.pose_bracket_stamps[1] > now:
                raise ValueError('historical pose not yet available at query time')
            if self._latest is not None and meta.stamp <= self._latest.stamp:
                return False  # No change to data, lease or clock on duplicates.
            origin = np.asarray(observation.sensor_origin_ned)
            with np.errstate(over='ignore', invalid='ignore'):
                normalized = observation.world_points / self.resolution_m
            if not np.isfinite(normalized).all() or np.any(np.abs(normalized) > 2**50):
                raise ValueError('endpoint voxel coordinates exceed numerical range')
            keys = (np.unique(np.floor(normalized).astype(np.int64), axis=0)
                    if len(normalized) else np.empty((0, 3), dtype=np.int64))
            def within_window(values):
                centers = (values.astype(float) + .5) * self.resolution_m
                return np.linalg.norm(centers - origin, axis=1) <= self.window_radius_m
            live = [(key, value) for key, value in self._cells.items() if value.current(now, epoch)]
            if live:
                keep = within_window(np.array([key for key, _ in live], dtype=float))
                candidate = {key: value for (key, value), selected in zip(live, keep) if selected}
            else:
                candidate = {}
            for raw in keys[within_window(keys)]:
                key = tuple(int(value) for value in raw)
                candidate[key] = meta
            if len(candidate) > self.capacity:
                raise ValueError('rolling endpoint map capacity exceeded')
            # Commit only after complete validation; never publish a partial scan.
            self._cells = candidate
            self._latest, self._last_now = meta, now
            return True

    def snapshot(self, now, epoch):
        with self._lock:
            self._check_time(now, epoch)
            metadata = self._latest or Metadata(now, now, epoch, 'empty_rolling_endpoints')
            cells = tuple(Cell(key, Occupancy.OCCUPIED, value)
                          for key, value in sorted(self._cells.items()) if value.current(now, epoch))
            self._last_now = now
            return OccupancySnapshot(metadata, self.resolution_m, cells)
