from dataclasses import replace
from types import SimpleNamespace as NS
import unittest

import numpy as np

from adapters.ros_noetic.stereo_images import decode_image
from interfaces.contracts import Metadata
from perception.stereo_depth.pairing import ImageFrame, StereoPairBuffer


def frame(side, stamp=10., epoch=0, encoding='rgb8', shape=(3, 4, 3)):
    return ImageFrame(Metadata(stamp, stamp + .15, epoch, 'test', 'front_' + side + '_image'),
                      encoding, np.zeros(shape, np.uint8))


class StereoPairingTests(unittest.TestCase):
    def test_pair_is_causal_in_sensor_time_and_consumed_once(self):
        buffer = StereoPairBuffer()
        buffer.add('left', frame('left'), 20.)
        buffer.add('right', frame('right', 10.003), 20.001)
        self.assertIsNone(buffer.pop(10.001, 20.002))
        pair = buffer.pop(10.004, 20.003)
        self.assertAlmostEqual(pair.skew_s, .003)
        self.assertEqual(pair.metadata.valid_until, 10.15)
        with self.assertRaises(ValueError):
            replace(pair, metadata=replace(pair.metadata, valid_until=11.))
        with self.assertRaises(ValueError):
            replace(pair, metadata=replace(pair.metadata, epoch=1))
        self.assertIsNone(buffer.pop(10.004, 20.004))
        with self.assertRaises(ValueError): pair.left.pixels.setflags(write=True)

    def test_large_skew_expiry_and_wall_stall_do_not_produce_pairs(self):
        for right_stamp, now, wall in ((10.02, 10.03, 20.01), (10.001, 10.16, 20.01),
                                       (10.001, 10.01, 20.21)):
            buffer = StereoPairBuffer()
            buffer.add('left', frame('left'), 20.)
            buffer.add('right', frame('right', right_stamp), 20.)
            self.assertIsNone(buffer.pop(now, wall))
            if now >= 10.15:
                self.assertGreater(buffer.events['SENSOR_EXPIRED_left'], 0)
            elif wall >= 20.2:
                self.assertGreater(buffer.events['WALL_EXPIRED_left'], 0)

    def test_queues_bounded_old_frames_not_replayed_and_reset_clears_pending(self):
        buffer = StereoPairBuffer(capacity=2)
        for stamp in (10., 10.01, 10.02): buffer.add('left', frame('left', stamp), 20.)
        self.assertEqual(len(buffer.pending['left']), 2)
        self.assertFalse(buffer.add('left', frame('left', 10.01), 20.))
        buffer.reset(1)
        self.assertFalse(buffer.add('right', frame('right', 10.02), 20.))
        self.assertIsNone(buffer.pop(10.03, 20.01))
        self.assertTrue(buffer.add('left', frame('left', 1., 1), 21.))
        self.assertTrue(buffer.add('right', frame('right', 1., 1), 21.))
        self.assertIsNotNone(buffer.pop(1.01, 21.01))

    def test_nearest_available_pair_discards_superseded_unpaired_images(self):
        buffer = StereoPairBuffer()
        buffer.add('left', frame('left', 10.), 20.)
        buffer.add('left', frame('left', 10.004), 20.)
        buffer.add('right', frame('right', 10.003), 20.)
        pair = buffer.pop(10.005, 20.01)
        self.assertEqual(pair.left.metadata.stamp, 10.004)
        self.assertEqual(buffer.events['SUPERSEDED_left'], 1)
        self.assertEqual(len(buffer.pending['left']), 0)

    def test_incompatible_images_wrong_source_and_future_leap_are_rejected(self):
        buffer = StereoPairBuffer()
        buffer.add('left', frame('left'), 20.)
        buffer.add('right', frame('right', shape=(4, 4, 3)), 20.)
        self.assertIsNone(buffer.pop(10.01, 20.01))
        with self.assertRaises(ValueError): buffer.add('left', frame('right'), 20.)
        buffer = StereoPairBuffer()
        buffer.add('left', frame('left', 11.), 20.)
        buffer.add('right', frame('right', 11.), 20.)
        self.assertIsNone(buffer.pop(10., 20.01))
        self.assertEqual(len(buffer.pending['left']), 0)

    def test_padded_image_bytes_and_empty_header_frame_are_preserved_without_tf(self):
        message = NS(header=NS(stamp=NS(to_sec=lambda: 10.), frame_id=''),
                     height=2, width=2, step=8, encoding='rgb8', data=bytes(range(16)))
        image = decode_image(message, 'left', 0)
        np.testing.assert_array_equal(image.pixels, [[[0, 1, 2], [3, 4, 5]],
                                                   [[8, 9, 10], [11, 12, 13]]])
        self.assertEqual(image.metadata.frame_id, 'front_left_image')
        delayed = decode_image(message, 'left', 0, ttl_s=.2)
        self.assertTrue(delayed.metadata.current(10.18, 0))
        self.assertFalse(image.metadata.current(10.18, 0))
        for key, value in (('encoding', 'mono16'), ('step', 5), ('data', b''), ('height', -1)):
            original = getattr(message, key)
            setattr(message, key, value)
            with self.assertRaises(ValueError): decode_image(message, 'left', 0)
            setattr(message, key, original)


if __name__ == '__main__':
    unittest.main()
