import math
import struct
from types import SimpleNamespace as NS
import unittest

import numpy as np

from adapters.ros_noetic.lidar_cloud import decode_xyz
from world_model.occupancy.grid import Occupancy
from world_model.occupancy.lidar import endpoint_snapshot


def message(points=((1., 2., 3.),), big=False, dtype=7, width=None):
    """Owned cloud with reordered fields, point padding, and row padding."""
    width = len(points) if width is None else width
    height = len(points) // width if width else 1
    size = 4 if dtype == 7 else 8
    step = 4 * size
    row_step = step * width + 8
    data = bytearray(row_step * height)
    fields = [NS(name=n, offset=o * size, datatype=dtype, count=1)
              for n, o in (('z', 0), ('x', 2), ('y', 1))]
    for i, (x, y, z) in enumerate(points):
        struct.pack_into(('>' if big else '<') + ('4f' if dtype == 7 else '4d'), data,
                         (i // width) * row_step + (i % width) * step, z, y, x, 999.)
    return NS(header=NS(stamp=NS(to_sec=lambda: 10.), frame_id='lidar'), width=width,
              height=height, point_step=step, row_step=row_step, fields=fields,
              is_bigendian=big, data=bytes(data), is_dense=True)


class LidarOccupancyTests(unittest.TestCase):
    def test_organized_padding_reordered_fields_and_both_byte_orders(self):
        points = ((1., 2., 3.), (-4., 5., -6.), (7., -8., 9.), (10., 11., -12.))
        for big in (False, True):
            for dtype in (7, 8):
                cloud = decode_xyz(message(points, big, dtype, width=2), 2)
                np.testing.assert_array_equal(cloud.points, points)
                self.assertEqual(cloud.metadata.frame_id, 'lidar')
                self.assertEqual(cloud.metadata.epoch, 2)
                with self.assertRaises(ValueError):
                    cloud.points.setflags(write=True)

    def test_nonfinite_returns_are_dropped_even_when_is_dense_is_true(self):
        cloud = decode_xyz(message(((1., 2., 3.), (math.nan, 0., 0.), (0., math.inf, 0.))), 0)
        self.assertEqual(cloud.input_points, 3)
        self.assertEqual(cloud.nonfinite_points, 2)
        self.assertEqual(len(cloud.points), 1)
        self.assertFalse(endpoint_snapshot(cloud, 10., 0).query((0., 0., 0.), 10., 0) == Occupancy.FREE)

    def test_malformed_layout_frame_and_time_are_rejected(self):
        for key, value in (('point_step', 0), ('row_step', 1), ('data', b''),
                           ('width', -1), ('height', 250001), ('width', True)):
            msg = message()
            setattr(msg, key, value)
            with self.assertRaises(ValueError):
                decode_xyz(msg, 0)
        for mutation in ('duplicate', 'missing', 'overrun', 'array', 'integer'):
            msg = message()
            if mutation == 'duplicate': msg.fields.append(msg.fields[0])
            if mutation == 'missing': msg.fields.pop()
            if mutation == 'overrun': msg.fields[0].offset = msg.point_step
            if mutation == 'array': msg.fields[0].count = 2
            if mutation == 'integer': msg.fields[0].datatype = 5
            with self.assertRaises(ValueError):
                decode_xyz(msg, 0)
        msg = message()
        msg.header.frame_id = 'world_ned'
        with self.assertRaises(ValueError): decode_xyz(msg, 0)
        msg = message()
        msg.header.stamp.to_sec = lambda: math.nan
        with self.assertRaises(ValueError): decode_xyz(msg, 0)

    def test_empty_cloud_and_unconfirmed_returns_do_not_create_free_cells(self):
        for points in ((), ((0., 0., 0.), (30., 0., 0.), (31., 0., 0.))):
            cloud = decode_xyz(message(points), 0)
            snapshot = endpoint_snapshot(cloud, 10., 0)
            self.assertEqual(snapshot.cells, ())
            self.assertEqual(snapshot.query((1., 0., 0.), 10., 0), Occupancy.UNKNOWN)

    def test_hit_voxels_use_floor_and_leave_ray_occlusion_and_neighbor_unknown(self):
        cloud = decode_xyz(message(((2., 0., 0.), (-.01, 0., 0.), (2.01, 0., 0.))), 4)
        snapshot = endpoint_snapshot(cloud, 10., 4)
        self.assertEqual(len(snapshot.cells), 2)
        for position in ((2., 0., 0.), (-.01, 0., 0.)):
            self.assertEqual(snapshot.query(position, 10., 4), Occupancy.OCCUPIED)
        for position in ((1., 0., 0.), (3., 0., 0.), (2., .3, 0.)):
            self.assertEqual(snapshot.query(position, 10., 4), Occupancy.UNKNOWN)

    def test_stale_future_and_epoch_mismatch_cannot_be_fused(self):
        cloud = decode_xyz(message(), 3)
        for now, epoch in ((9.99, 3), (10.2, 3), (10., 4)):
            with self.assertRaises(ValueError): endpoint_snapshot(cloud, now, epoch)
        snapshot = endpoint_snapshot(cloud, 10., 3)
        self.assertEqual(snapshot.query((1., 2., 3.), 10.2, 3), Occupancy.UNKNOWN)
        self.assertEqual(snapshot.query((1., 2., 3.), 10., 4), Occupancy.UNKNOWN)
        next_scan = endpoint_snapshot(decode_xyz(message(()), 3), 10., 3)
        self.assertEqual(next_scan.query((1., 2., 3.), 10., 3), Occupancy.UNKNOWN)


if __name__ == '__main__':
    unittest.main()
