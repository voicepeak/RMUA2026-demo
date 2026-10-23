"""Bounded historical LiDAR association with an already causal stereo pair."""
from collections import Counter, deque
from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class LidarAssociation:
    cloud: object
    arrival_wall: float
    left_skew_s: float
    right_skew_s: float
    lidar_current_at_selection: bool
    current_control_eligible: bool = field(default=False, init=False)
    per_point_motion_compensation_applied: bool = field(default=False, init=False)


class LidarAssociationBuffer:
    def __init__(self, epoch=0, capacity=8, retention_s=.6, max_skew_s=.02):
        if (not isinstance(epoch, int) or epoch < 0 or not isinstance(capacity, int)
                or not 1 <= capacity <= 16 or not math.isfinite(retention_s)
                or not .2 <= retention_s <= 1. or not math.isfinite(max_skew_s)
                or not 0 < max_skew_s <= .05):
            raise ValueError('invalid joint association limits')
        self.epoch, self.capacity = epoch, capacity
        self.retention_s, self.max_skew_s = retention_s, max_skew_s
        self.pending = deque()
        self.last_stamp = self.last_arrival = None
        self.last_pair = None
        self.events = Counter()

    def reset(self, epoch):
        if not isinstance(epoch, int) or epoch <= self.epoch:
            raise ValueError('association epoch must increase')
        self.epoch = epoch
        self.pending.clear()
        self.last_stamp = self.last_arrival = self.last_pair = None

    def add(self, cloud, arrival_wall):
        if cloud.metadata.frame_id != 'lidar' or not math.isfinite(arrival_wall) or arrival_wall < 0:
            raise ValueError('invalid LiDAR source or arrival')
        if cloud.metadata.epoch != self.epoch:
            self.events['OTHER_EPOCH'] += 1
            return False
        if self.last_stamp is not None and cloud.metadata.stamp <= self.last_stamp:
            self.events['NONINCREASING'] += 1
            return False
        if self.last_arrival is not None and arrival_wall < self.last_arrival:
            raise ValueError('LiDAR arrivals must not go backwards')
        self.last_stamp, self.last_arrival = cloud.metadata.stamp, arrival_wall
        self.pending.append((cloud, arrival_wall))
        while len(self.pending) > self.capacity:
            self.pending.popleft()
            self.events['CAPACITY'] += 1
        return True

    def select(self, pair, now, wall_now, epoch):
        if not all(math.isfinite(v) and v >= 0 for v in (now, wall_now)):
            raise ValueError('invalid joint query clock')
        if epoch != self.epoch or not pair.metadata.current(now, epoch):
            self.events['INVALID_STEREO_TIME_OR_EPOCH'] += 1
            return None
        key = (pair.left.metadata.stamp, pair.right.metadata.stamp)
        if self.last_pair is not None and (key[0] <= self.last_pair[0] or key[1] <= self.last_pair[1]):
            self.events['REUSED_OR_OLDER_PAIR'] += 1
            return None
        self.pending = deque((cloud, arrival) for cloud, arrival in self.pending
            if 0 <= wall_now - arrival < self.retention_s
            and 0 <= now - cloud.metadata.stamp < self.retention_s)
        candidates = []
        for index, (cloud, arrival) in enumerate(self.pending):
            dl = abs(cloud.metadata.stamp - key[0])
            dr = abs(cloud.metadata.stamp - key[1])
            if max(dl, dr) <= self.max_skew_s:
                candidates.append((max(dl, dr), index, dl, dr))
        if not candidates:
            self.events['NO_LIDAR_WITHIN_WINDOW'] += 1
            return None
        _, index, dl, dr = min(candidates)
        cloud, arrival = self.pending[index]
        del self.pending[index]
        self.last_pair = key
        return LidarAssociation(cloud, arrival, dl, dr, cloud.metadata.current(now, epoch))
