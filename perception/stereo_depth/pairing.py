"""Causal, bounded approximate stereo pairing on an external IMU clock."""
from collections import Counter, deque
from dataclasses import dataclass
import math

import numpy as np

from interfaces.contracts import Metadata


@dataclass(frozen=True)
class ImageFrame:
    metadata: Metadata
    encoding: str
    pixels: np.ndarray

    def __post_init__(self):
        image = np.asarray(self.pixels)
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
        count = channels.get(self.encoding)
        if (image.dtype != np.uint8 or image.ndim != 3 or count is None
                or image.shape[2] != count or not 0 < image.shape[0] <= 2160
                or not 0 < image.shape[1] <= 3840):
            raise ValueError('invalid byte image or encoding')
        immutable = np.frombuffer(image.tobytes(), dtype=np.uint8).reshape(image.shape)
        object.__setattr__(self, 'pixels', immutable)


@dataclass(frozen=True)
class StereoPair:
    metadata: Metadata
    left: ImageFrame
    right: ImageFrame
    skew_s: float

    def __post_init__(self):
        if (self.metadata.frame_id != 'front_stereo'
                or self.left.metadata.frame_id != 'front_left_image'
                or self.right.metadata.frame_id != 'front_right_image'
                or self.left.pixels.shape != self.right.pixels.shape
                or self.left.encoding != self.right.encoding
                or self.left.metadata.epoch != self.right.metadata.epoch
                or self.metadata.epoch != self.left.metadata.epoch
                or self.metadata.stamp != max(self.left.metadata.stamp, self.right.metadata.stamp)
                or self.metadata.valid_until != min(self.left.metadata.valid_until, self.right.metadata.valid_until)
                or not math.isfinite(self.skew_s)
                or self.skew_s != abs(self.left.metadata.stamp - self.right.metadata.stamp)):
            raise ValueError('inconsistent immutable stereo pair')


class StereoPairBuffer:
    def __init__(self, epoch=0, max_skew_s=.005, capacity=4, wall_ttl_s=.2):
        if (not isinstance(epoch, int) or epoch < 0 or not isinstance(capacity, int)
                or not 1 <= capacity <= 8 or not math.isfinite(max_skew_s)
                or not 0 <= max_skew_s <= .02 or not math.isfinite(wall_ttl_s)
                or not 0 < wall_ttl_s <= .5):
            raise ValueError('invalid stereo pairing limits')
        self.epoch = epoch
        self.max_skew_s = max_skew_s
        self.wall_ttl_s = wall_ttl_s
        self.capacity = capacity
        self.pending = {'left': deque(), 'right': deque()}
        self.last = {'left': None, 'right': None}
        self.events = Counter()

    def reset(self, epoch):
        if not isinstance(epoch, int) or epoch <= self.epoch:
            raise ValueError('new epoch must increase')
        for queue in self.pending.values(): queue.clear()
        self.last = {'left': None, 'right': None}
        self.epoch = epoch
        self.events['EPOCH_RESET'] += 1

    def add(self, side, frame, arrival):
        if side not in self.pending or not math.isfinite(arrival) or arrival < 0:
            raise ValueError('invalid stereo source or arrival')
        expected = 'front_' + side + '_image'
        if frame.metadata.frame_id != expected:
            raise ValueError('stereo source frame mismatch')
        if frame.metadata.epoch != self.epoch:
            self.events['OTHER_EPOCH'] += 1
            return False
        previous = self.last[side]
        if previous is not None and frame.metadata.stamp <= previous:
            self.events['NONINCREASING_' + side] += 1
            return False
        self.last[side] = frame.metadata.stamp
        queue = self.pending[side]
        if len(queue) == self.capacity:
            queue.popleft()
            self.events['CAPACITY_' + side] += 1
        queue.append((frame, arrival))
        return True

    def pop(self, now, wall_now):
        if not all(math.isfinite(x) and x >= 0 for x in (now, wall_now)):
            raise ValueError('invalid pairing clock')
        for side, queue in self.pending.items():
            keep = deque()
            for frame, arrival in queue:
                if frame.metadata.epoch != self.epoch:
                    self.events['OTHER_EPOCH_' + side] += 1
                elif frame.metadata.valid_until <= now:
                    self.events['SENSOR_EXPIRED_' + side] += 1
                elif not 0 <= wall_now - arrival < self.wall_ttl_s:
                    self.events['WALL_EXPIRED_' + side] += 1
                elif frame.metadata.stamp > now + .15:
                    self.events['FUTURE_' + side] += 1
                else:
                    keep.append((frame, arrival))
            self.pending[side] = keep
        candidates = []
        for i, (left, _) in enumerate(self.pending['left']):
            for j, (right, _) in enumerate(self.pending['right']):
                skew = abs(left.metadata.stamp - right.metadata.stamp)
                if (left.metadata.current(now, self.epoch) and right.metadata.current(now, self.epoch)
                        and skew <= self.max_skew_s and left.pixels.shape == right.pixels.shape
                        and left.encoding == right.encoding):
                    candidates.append((skew, i, j))
        if not candidates:
            return None
        skew, i, j = min(candidates)
        selected = {}
        for side, index in (('left', i), ('right', j)):
            self.events['SUPERSEDED_' + side] += index
            for _ in range(index + 1):
                selected[side], _ = self.pending[side].popleft()
        left, right = selected['left'], selected['right']
        metadata = Metadata(max(left.metadata.stamp, right.metadata.stamp),
                            min(left.metadata.valid_until, right.metadata.valid_until), self.epoch,
                            'front_stereo_approximate_pair', 'front_stereo', (skew,))
        return StereoPair(metadata, left, right, skew)
