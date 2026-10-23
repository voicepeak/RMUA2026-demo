"""Bounded PointCloud2 XYZ decoding; no ROS import or implicit frame transform."""
from dataclasses import dataclass
import math
import numbers

import numpy as np

from interfaces.contracts import Metadata


@dataclass(frozen=True)
class LidarCloud:
    metadata: Metadata
    points: np.ndarray
    input_points: int
    nonfinite_points: int

    def __post_init__(self):
        points = np.asarray(self.points, dtype=np.float64)
        if (points.ndim != 2 or points.shape[1] != 3
                or len(points) > 250000 or not np.isfinite(points).all()
                or self.nonfinite_points < 0
                or self.input_points != len(points) + self.nonfinite_points):
            raise ValueError('invalid finite XYZ cloud')
        # A bytes owner prevents callers from enabling writes on the array.
        immutable = np.frombuffer(points.tobytes(), dtype=np.float64).reshape(-1, 3)
        object.__setattr__(self, 'points', immutable)


def integer(value, name, minimum=0):
    if isinstance(value, bool) or not isinstance(value, numbers.Integral) or value < minimum:
        raise ValueError('invalid ' + name)
    return int(value)


def decode_xyz(message, epoch, ttl_s=.2, expected_frame='lidar'):
    """Preserve the sensor frame and stamp; drop nonfinite returns as UNKNOWN.

    Handles row/point padding and both byte orders. No dense/is_dense assumption,
    ray timing, no-return inference, or body/world coordinate conversion.
    """
    stamp = message.header.stamp.to_sec()
    if not math.isfinite(stamp) or stamp <= 0:
        raise ValueError('invalid lidar timestamp')
    if not math.isfinite(ttl_s) or not 0 < ttl_s <= .5:
        raise ValueError('invalid lidar lifetime')
    if not expected_frame or message.header.frame_id != expected_frame:
        raise ValueError('unexpected lidar frame')
    width = integer(message.width, 'width')
    height = integer(message.height, 'height')
    step = integer(message.point_step, 'point_step', 1)
    row_step = integer(message.row_step, 'row_step')
    if (width * height > 250000 or step > 1024 or row_step < width * step
            or row_step * height > 16 * 1024 * 1024):
        raise ValueError('oversized or inconsistent cloud layout')
    if len(message.data) != row_step * height:
        raise ValueError('truncated or oversized cloud data')
    data = bytes(message.data)
    fields = {}
    sizes = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}
    for field in message.fields:
        offset = integer(field.offset, 'field offset')
        count = integer(field.count, 'field count', 1)
        datatype = integer(field.datatype, 'field datatype', 1)
        if (not field.name or field.name in fields or datatype not in sizes
                or offset + count * sizes[datatype] > step):
            raise ValueError('invalid or duplicate point field')
        fields[field.name] = field
    endian = '>' if message.is_bigendian else '<'
    points = np.empty((height * width, 3), dtype=np.float64)
    for axis, name in enumerate(('x', 'y', 'z')):
        field = fields.get(name)
        if field is None or field.count != 1 or field.datatype not in (7, 8):
            raise ValueError('XYZ requires scalar FLOAT32 or FLOAT64 fields')
        if width * height:
            view = np.ndarray((height, width), dtype=endian + ('f4' if field.datatype == 7 else 'f8'),
                              buffer=data, offset=field.offset, strides=(row_step, step))
            points[:, axis] = view.reshape(-1)
    finite = np.isfinite(points).all(axis=1)
    metadata = Metadata(stamp, stamp + ttl_s, integer(epoch, 'epoch'),
                        'official_lidar_xyz', message.header.frame_id)
    return LidarCloud(metadata, points[finite], len(points), int((~finite).sum()))
