import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

import numpy as np

from adapters.ros_noetic.capture_io import save_message, read_payload
from adapters.ros_noetic.lidar_cloud import LidarCloud
from adapters.ros_noetic.joint_sensor_observer import raw_record
from interfaces.contracts import Metadata
from perception.sensor_sync import LidarAssociationBuffer
from perception.stereo_depth.pairing import ImageFrame, StereoPairBuffer


def pair(stamp=10.1, skew=.003, epoch=0):
    buffer = StereoPairBuffer(epoch=epoch)
    for side, time in (('left', stamp), ('right', stamp + skew)):
        buffer.add(side, ImageFrame(Metadata(time, time + .2, epoch, 'test', 'front_' + side + '_image'),
                                   'mono8', np.zeros((4, 4, 1), np.uint8)), 20.)
    return buffer.pop(stamp + .01, 20.01)


def cloud(stamp, epoch=0, ttl=.2):
    return LidarCloud(Metadata(stamp, stamp + ttl, epoch, 'test', 'lidar'), np.array(((1., 0., 0.),)), 1, 0)


class JointAssociationTests(unittest.TestCase):
    def test_closest_scan_within_both_image_windows_consumed_once(self):
        buffer = LidarAssociationBuffer()
        buffer.add(cloud(10.08), 20.)
        buffer.add(cloud(10.1), 20.01)
        p = pair()
        chosen = buffer.select(p, 10.15, 20.02, 0)
        self.assertEqual(chosen.cloud.metadata.stamp, 10.1)
        self.assertFalse(chosen.current_control_eligible)
        self.assertFalse(chosen.per_point_motion_compensation_applied)
        self.assertIsNone(buffer.select(p, 10.15, 20.03, 0))

    def test_header_window_does_not_extend_a_lidar_lease(self):
        buffer = LidarAssociationBuffer()
        original = cloud(10.09, ttl=.01)
        buffer.add(original, 20.)
        association = buffer.select(pair(), 10.15, 20.02, 0)
        self.assertIsNotNone(association)
        self.assertFalse(association.lidar_current_at_selection)
        self.assertEqual(association.cloud.metadata.valid_until, original.metadata.valid_until)

    def test_stale_future_wrong_epoch_and_expired_stereo_cannot_associate(self):
        for stamp, now, wall, epoch in ((10.04, 10.15, 20.02, 0), (10.11, 10.105, 20.02, 0),
                                        (10.1, 10.15, 20.7, 0), (10.1, 10.31, 20.02, 0),
                                        (10.1, 10.15, 20.02, 1)):
            buffer = LidarAssociationBuffer()
            buffer.add(cloud(stamp), 20.)
            self.assertIsNone(buffer.select(pair(), now, wall, epoch))

    def test_capacity_duplicates_and_epoch_reset_are_bounded(self):
        buffer = LidarAssociationBuffer(capacity=2)
        for stamp in (10., 10.1, 10.2):buffer.add(cloud(stamp), 20.)
        self.assertEqual(len(buffer.pending), 2)
        self.assertFalse(buffer.add(cloud(10.2), 20.1))
        buffer.reset(1)
        self.assertEqual(len(buffer.pending), 0)
        self.assertFalse(buffer.add(cloud(10.3), 20.2))
        self.assertTrue(buffer.add(cloud(1., 1), 21.))


class CapturePayloadTests(unittest.TestCase):
    def message(self):
        stamp = NS(secs=10, nsecs=123456789, to_sec=lambda: 10.123456789)
        return NS(header=NS(stamp=stamp, seq=7, frame_id=''), data=bytes(range(24)),
                  width=2, height=4, step=6, encoding='rgb8', is_bigendian=0)

    def test_payload_and_integer_timestamp_retained_exactly_and_exclusive(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name);message = self.message()
            document = save_message(root, 'left', message, 'image')
            loaded, payload = read_payload(root / 'left.json')
            self.assertEqual(payload, message.data)
            self.assertEqual(loaded['stamp_nsecs'], 123456789)
            self.assertEqual(loaded['raw_sha256'], hashlib.sha256(message.data).hexdigest())
            with self.assertRaises(FileExistsError):save_message(root, 'left', message, 'image')

    def test_bad_size_hash_or_external_payload_path_is_rejected(self):
        with tempfile.TemporaryDirectory() as name:
            root = Path(name);original = save_message(root, 'left', self.message(), 'image')
            for key, value in (('raw_bytes', 10), ('raw_sha256', '0' * 64), ('payload_file', '../left.bin.gz')):
                changed = dict(original);changed[key] = value
                (root / 'left.json').write_text(json.dumps(changed))
                with self.assertRaises(ValueError):read_payload(root / 'left.json')

    def test_nonfinite_raw_measurement_is_explicitly_null_without_invalid_json(self):
        message = self.message()
        message.linear_acceleration = NS(x=float('nan'), y=0., z=-9.81)
        message.angular_velocity = NS(x=0.,y=0.,z=0.)
        message.orientation = NS(w=1.,x=0.,y=0.,z=0.)
        message.linear_acceleration_covariance = (0.,)*9
        message.angular_velocity_covariance = (0.,)*9
        message.orientation_covariance = (0.,)*9
        record = raw_record('imu',message,20.)
        self.assertFalse(record['raw_fields_finite'])
        self.assertIsNone(record['acceleration_body'][0])
        json.dumps(record,allow_nan=False)


if __name__ == '__main__':
    unittest.main()
