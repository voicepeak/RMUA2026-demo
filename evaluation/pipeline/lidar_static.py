"""Read-only LiDAR-to-static-planner probe; never grants control authority."""
from dataclasses import dataclass
import threading

from estimation.history import HistoryUnavailable
from interfaces.contracts import Health
from planning.spacetime_local.static_candidates import plan_static_local
from world_model.occupancy.rolling_endpoints import RollingEndpointMap, world_endpoints


@dataclass(frozen=True)
class ProbeResult:
    reason: str
    map_cells: int = 0
    candidate_reasons: tuple = ()
    # A nominal endpoint map cannot certify FREE or an executable trajectory.
    control_eligible: bool = False


class LidarStaticProbe:
    """Join retained scan, history, rolling occupied map and static planner.

    The caller owns the history and serializes its updates with this probe.
    A scan is never re-used after rejection and no control/reference is returned.
    """
    def __init__(self, history, sensor_pose, limits, epoch=0, **map_limits):
        if history.epoch != epoch:
            raise ValueError('history and map epoch mismatch')
        self.history = history
        self.sensor_pose = sensor_pose
        self.limits = limits
        self.map = RollingEndpointMap(epoch=epoch, **map_limits)
        self._lock = threading.RLock()

    def reset(self, epoch):
        with self._lock:
            # History is reset by its owner before the map, so they cannot silently diverge.
            if self.history.epoch != epoch:
                raise ValueError('history must be reset before probe')
            self.map.reset(epoch)

    def evaluate(self, cloud, state, initial_reference, goal, now, wall_now, epoch):
        with self._lock:
            if epoch != self.map.epoch or epoch != self.history.epoch:
                return ProbeResult('EPOCH_MISMATCH')
            if not cloud.metadata.current(now, epoch) or cloud.metadata.frame_id != 'lidar':
                return ProbeResult('SCAN_UNAVAILABLE')
            sources = self.history.snapshot_sources()
            if (not sources or sources[-1][0] != state or not state.metadata.current(now, epoch)
                    or state.health != Health.OK):
                return ProbeResult('STATE_UNAVAILABLE')
            try:
                pose = self.history.lookup(cloud.metadata.stamp, now, wall_now, epoch)
            except HistoryUnavailable as exc:
                return ProbeResult('POSE_' + str(exc))
            try:
                observation = world_endpoints(cloud, pose, self.sensor_pose)
                if not self.map.integrate(observation, now, epoch):
                    return ProbeResult('SCAN_DUPLICATE')
                snapshot = self.map.snapshot(now, epoch)
            except ValueError:
                return ProbeResult('MAP_REJECTED')
            try:
                plan = plan_static_local(state, initial_reference, goal, snapshot, now, epoch, self.limits)
            except (ValueError, OverflowError):
                return ProbeResult('PLANNER_INPUT_REJECTED', len(snapshot.cells))
            return ProbeResult(plan.reason, len(snapshot.cells), tuple(check.reason for check in plan.checks))
