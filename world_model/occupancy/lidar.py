"""Single-scan endpoint evidence. No free-space carving or cross-frame fusion."""
import math

import numpy as np

from world_model.occupancy.grid import Cell, Occupancy, OccupancySnapshot


def endpoint_snapshot(cloud, now, epoch, resolution_m=.25, max_range_m=30.):
    """Voxelize returns in the scan's frozen sensor frame, never world_ned.

    No per-point times or calibrated poses are available in this contract.
    This snapshot cannot certify collision clearance for a moving vehicle.
    Missing returns, out-of-range returns and all unvisited cells stay UNKNOWN.
    """
    if not math.isfinite(resolution_m) or not .05 <= resolution_m <= 2.:
        raise ValueError('endpoint resolution must be in [.05,2] meters')
    if not math.isfinite(max_range_m) or not 0 < max_range_m <= 30.:
        raise ValueError('endpoint range must be in (0,30] meters')
    if not cloud.metadata.current(now, epoch):
        raise ValueError('stale, future or other-epoch lidar cloud')
    if cloud.metadata.frame_id != 'lidar':
        raise ValueError('endpoint map requires the frozen lidar sensor frame')
    ranges = np.hypot(np.hypot(cloud.points[:, 0], cloud.points[:, 1]), cloud.points[:, 2])
    # Zero and max-range returns have no confirmed hit semantics.
    hits = cloud.points[(ranges > 0) & (ranges < max_range_m)]
    # Noetic's NumPy 1.17 cannot unique(axis=0) an empty (0,3) array.
    keys = (np.unique(np.floor(hits / resolution_m).astype(np.int64), axis=0)
            if len(hits) else np.empty((0, 3), dtype=np.int64))
    cells = tuple(Cell(tuple(int(k) for k in key), Occupancy.OCCUPIED, cloud.metadata)
                  for key in keys)
    return OccupancySnapshot(cloud.metadata, resolution_m, cells)
