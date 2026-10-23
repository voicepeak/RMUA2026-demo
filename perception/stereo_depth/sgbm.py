"""Diagnostic pixel disparity with left/right and texture rejection.

Input rectification and exposures are unverified; this does not return depth.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from interfaces.contracts import Metadata


@dataclass(frozen=True)
class SgbmConfig:
    num_disparities: int = 128
    block_size: int = 5
    uniqueness_ratio: int = 10
    consistency_px: float = 1.
    min_texture_std: float = 2.

    def __post_init__(self):
        if (not isinstance(self.num_disparities, int) or not 16 <= self.num_disparities <= 256
                or self.num_disparities % 16 or not isinstance(self.block_size, int)
                or not 3 <= self.block_size <= 11 or self.block_size % 2 != 1
                or not isinstance(self.uniqueness_ratio, int) or not 5 <= self.uniqueness_ratio <= 30
                or not math.isfinite(self.consistency_px) or not 0 < self.consistency_px <= 1.5
                or not math.isfinite(self.min_texture_std) or not 1 <= self.min_texture_std <= 20):
            raise ValueError('invalid bounded SGBM parameters')


@dataclass(frozen=True)
class DisparityObservation:
    metadata: Metadata
    disparity_px: np.ndarray
    valid: np.ndarray
    left_measurement_stamp: float
    right_measurement_stamp: float
    rectification_verified: bool = field(default=False, init=False)
    metric_depth_verified: bool = field(default=False, init=False)
    acceptance_eligible: bool = field(default=False, init=False)

    def __post_init__(self):
        values, valid = np.asarray(self.disparity_px), np.asarray(self.valid)
        if (values.ndim != 2 or not np.issubdtype(values.dtype, np.floating)
                or valid.shape != values.shape or valid.dtype != np.bool_
                or not np.isfinite(values[valid]).all() or np.any(values[valid] <= 0)
                or np.any(values[valid] > np.finfo(np.float32).max)
                or not np.isnan(values[~valid]).all()):
            raise ValueError('invalid disparity mask or unknown pixels')
        object.__setattr__(self, 'disparity_px', np.frombuffer(
            values.astype(np.float32).tobytes(), dtype=np.float32).reshape(values.shape))
        object.__setattr__(self, 'valid', np.frombuffer(valid.tobytes(), dtype=np.bool_).reshape(valid.shape))


def _gray(frame, cv2):
    if frame.encoding == 'mono8':
        return frame.pixels[:, :, 0].copy()
    conversion = {'bgr8': cv2.COLOR_BGR2GRAY, 'rgb8': cv2.COLOR_RGB2GRAY,
                  'bgra8': cv2.COLOR_BGRA2GRAY, 'rgba8': cv2.COLOR_RGBA2GRAY}[frame.encoding]
    return cv2.cvtColor(frame.pixels, conversion)


def compute_diagnostic_disparity(pair, config=None):
    import cv2

    config = config or SgbmConfig()
    left, right = _gray(pair.left, cv2), _gray(pair.right, cv2)
    if (left.shape != right.shape or left.shape[1] <= config.num_disparities + config.block_size
            or left.shape[0] <= 2 * config.block_size
            or pair.left.metadata.epoch != pair.right.metadata.epoch
            or pair.left.metadata.epoch != pair.metadata.epoch
            or abs(pair.left.metadata.stamp - pair.right.metadata.stamp) > .005):
        raise ValueError('incompatible diagnostic stereo input')

    def matcher(minimum):
        block = config.block_size
        return cv2.StereoSGBM_create(minDisparity=minimum,
            numDisparities=config.num_disparities, blockSize=block,
            P1=8 * block * block, P2=32 * block * block,
            disp12MaxDiff=1, preFilterCap=31, uniquenessRatio=config.uniqueness_ratio,
            speckleWindowSize=100, speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM)

    minimum_right = 1 - config.num_disparities
    forward = matcher(0).compute(left, right).astype(np.float32) / 16.
    reverse = matcher(minimum_right).compute(right, left).astype(np.float32) / 16.
    width = left.shape[1]
    right_x = np.arange(width, dtype=np.float32)[None, :] - forward
    lower = np.floor(right_x).astype(np.int32)
    clipped = np.clip(lower, 0, width - 2)
    r0 = np.take_along_axis(reverse, clipped, axis=1)
    r1 = np.take_along_axis(reverse, clipped + 1, axis=1)
    weight = right_x - lower
    interpolated = (1. - weight) * r0 + weight * r1

    def texture(image):
        image = image.astype(np.float32)
        size = (config.block_size, config.block_size)
        mean = cv2.boxFilter(image, cv2.CV_32F, size, borderType=cv2.BORDER_REPLICATE)
        square = cv2.boxFilter(image * image, cv2.CV_32F, size, borderType=cv2.BORDER_REPLICATE)
        return np.maximum(square - mean * mean, 0) >= config.min_texture_std ** 2

    right_texture = texture(right)
    valid = ((forward > 0) & (forward < config.num_disparities)
             & (lower >= 0) & (lower + 1 < width)
             & (r0 >= minimum_right) & (r0 < 0) & (r1 >= minimum_right) & (r1 < 0)
             & (np.abs(forward + interpolated) <= config.consistency_px)
             & texture(left) & np.take_along_axis(right_texture, clipped, axis=1)
             & np.take_along_axis(right_texture, clipped + 1, axis=1))
    radius = config.block_size // 2
    valid[:radius] = False
    valid[-radius:] = False
    valid[:, :radius] = False
    valid[:, -radius:] = False
    disparity = np.where(valid, forward, np.nan).astype(np.float32)
    metadata = Metadata(pair.metadata.stamp, pair.metadata.valid_until, pair.metadata.epoch,
                        'opencv_sgbm_diagnostic_pixels', 'front_left_image')
    return DisparityObservation(metadata, disparity, valid,
                                pair.left.metadata.stamp, pair.right.metadata.stamp)
