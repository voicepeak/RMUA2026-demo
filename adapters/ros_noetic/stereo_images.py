"""Decode ROS byte images without inferring empty frame_ids as calibrated TF."""
import math

import numpy as np

from adapters.ros_noetic.lidar_cloud import integer
from interfaces.contracts import Metadata
from perception.stereo_depth.pairing import ImageFrame


def decode_image(message, side, epoch, ttl_s=.15):
    if side not in ('left', 'right'):
        raise ValueError('unsupported stereo source')
    stamp = message.header.stamp.to_sec()
    if not math.isfinite(stamp) or stamp <= 0 or not math.isfinite(ttl_s) or not 0 < ttl_s <= .5:
        raise ValueError('invalid image time')
    height = integer(message.height, 'image height', 1)
    width = integer(message.width, 'image width', 1)
    step = integer(message.step, 'image step', 1)
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(message.encoding)
    if (channels is None or height > 2160 or width > 3840 or step < width * channels
            or step * height > 32 * 1024 * 1024 or len(message.data) != step * height):
        raise ValueError('unsupported or inconsistent byte image')
    # The source is the subscribed topic, not header.frame_id: local images
    # advertise an empty frame_id. Neither case supplies a calibrated TF.
    metadata = Metadata(stamp, stamp + ttl_s, integer(epoch, 'image epoch'),
                        '/airsim_node/drone_1/front_' + side + '/Scene',
                        'front_' + side + '_image')
    pixels = np.ndarray((height, width, channels), dtype=np.uint8,
                        buffer=bytes(message.data), strides=(step, channels, 1))
    return ImageFrame(metadata, message.encoding, pixels)
