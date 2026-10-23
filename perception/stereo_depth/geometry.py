"""Versioned nominal camera/LiDAR geometry from public simulator settings.

Body/sensor axes are FRD; image optical axes are right/down/forward.
This is a configuration model, not a completed metric calibration.
"""
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from interfaces.contracts import finite_vec3


def points(value, columns):
    array = np.asarray(value, dtype=float)
    if array.ndim != 2 or array.shape[1] != columns:
        raise ValueError('invalid point array shape')
    return array


@dataclass(frozen=True)
class SensorPose:
    translation_body_frd: tuple
    rotation_body_from_sensor_frd: tuple

    def __post_init__(self):
        finite_vec3(self.translation_body_frd)
        rows = self.rotation_body_from_sensor_frd
        if not isinstance(rows, tuple) or len(rows) != 3:
            raise ValueError('immutable rotation rows required')
        for row in rows:
            finite_vec3(row)
        rotation = np.array(rows)
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-10, rtol=0) or abs(np.linalg.det(rotation) - 1) > 1e-10:
            raise ValueError('proper sensor rotation required')

    def to_body(self, sensor_points):
        return points(sensor_points, 3) @ np.array(self.rotation_body_from_sensor_frd).T + self.translation_body_frd

    def from_body(self, body_points):
        return (points(body_points, 3) - self.translation_body_frd) @ np.array(self.rotation_body_from_sensor_frd)


def sensor_pose(settings):
    translation = tuple(float(settings.get(k, 0.)) for k in ('X', 'Y', 'Z'))
    roll, pitch, yaw = (math.radians(float(settings.get(k, 0.))) for k in ('Roll', 'Pitch', 'Yaw'))
    if not all(math.isfinite(v) for v in (roll, pitch, yaw)):
        raise ValueError('nonfinite sensor angle')
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                (-sp, cp*sr, cp*cr))
    return SensorPose(translation, rotation)


@dataclass(frozen=True)
class NominalCamera:
    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    pose: SensorPose

    def __post_init__(self):
        if (not self.name or not isinstance(self.width, int) or not 1 <= self.width <= 3840
                or not isinstance(self.height, int) or not 1 <= self.height <= 2160
                or not all(math.isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy))
                or min(self.fx, self.fy) <= 0 or not 0 <= self.cx < self.width or not 0 <= self.cy < self.height):
            raise ValueError('invalid nominal camera')

    def body_to_optical(self, body_points):
        frd = self.pose.from_body(body_points)
        return frd[:, (1, 2, 0)]

    def optical_to_body(self, optical_points):
        return self.pose.to_body(points(optical_points, 3)[:, (2, 0, 1)])

    def project_body(self, body_points):
        body_points = points(body_points, 3)
        finite = np.isfinite(body_points).all(axis=1)
        optical = np.full(body_points.shape, np.nan)
        optical[finite] = self.body_to_optical(body_points[finite])
        z = optical[:, 2]
        valid = np.zeros(len(z), dtype=bool)
        valid[finite] = z[finite] > 0
        uv = np.full((len(z), 2), np.nan)
        uv[valid, 0] = self.fx * optical[valid, 0] / z[valid] + self.cx
        uv[valid, 1] = self.fy * optical[valid, 1] / z[valid] + self.cy
        valid[valid] = ((uv[valid, 0] >= 0) & (uv[valid, 0] < self.width)
                       & (uv[valid, 1] >= 0) & (uv[valid, 1] < self.height))
        uv[~valid] = np.nan
        return uv, z, valid


@dataclass(frozen=True)
class NominalStereoRig:
    left: NominalCamera
    right: NominalCamera
    lidar_pose: SensorPose
    settings_sha256: str
    metric_calibration_verified: bool = field(default=False, init=False)
    current_control_eligible: bool = field(default=False, init=False)

    def __post_init__(self):
        a, b = self.left, self.right
        ra = np.array(a.pose.rotation_body_from_sensor_frd)
        rb = np.array(b.pose.rotation_body_from_sensor_frd)
        offset = ra.T @ (np.array(b.pose.translation_body_frd) - a.pose.translation_body_frd)
        if (not np.allclose(ra, rb, atol=1e-10, rtol=0)
                or abs(offset[0]) > 1e-10 or abs(offset[2]) > 1e-10 or offset[1] <= 0
                or (a.width, a.height, a.fx, a.fy, a.cx, a.cy) != (b.width, b.height, b.fx, b.fy, b.cx, b.cy)
                or len(self.settings_sha256) != 64
                or any(c not in '0123456789abcdef' for c in self.settings_sha256)):
            raise ValueError('nominal stereo requires parallel matched cameras and a positive right baseline')

    @property
    def baseline_m(self):
        rotation = np.array(self.left.pose.rotation_body_from_sensor_frd)
        offset = np.array(self.right.pose.translation_body_frd) - self.left.pose.translation_body_frd
        return float((rotation.T @ offset)[1])

    def triangulate_body(self, left_uv, right_uv, max_depth_m=30., max_vertical_px=1.):
        """Nominal points and |dZ/dd|, not calibrated uncertainty or safe space.

        Depth is optical forward Z, not Euclidean distance. Invalid/too distant
        correspondences stay NaN. Range and vertical limits are explicit.
        """
        left_uv, right_uv = points(left_uv, 2), points(right_uv, 2)
        if (left_uv.shape != right_uv.shape or not math.isfinite(max_depth_m) or not 0 < max_depth_m <= 100
                or not math.isfinite(max_vertical_px) or not 0 <= max_vertical_px <= 2):
            raise ValueError('invalid nominal triangulation limits')
        valid = np.isfinite(left_uv).all(axis=1) & np.isfinite(right_uv).all(axis=1)
        for uv, camera in ((left_uv, self.left), (right_uv, self.right)):
            valid[valid] = ((uv[valid, 0] >= 0) & (uv[valid, 0] < camera.width)
                            & (uv[valid, 1] >= 0) & (uv[valid, 1] < camera.height))
        disparity = np.full(len(left_uv), np.nan)
        disparity[valid] = left_uv[valid, 0] - right_uv[valid, 0]
        valid[valid] = ((disparity[valid] > 0)
                       & (np.abs(left_uv[valid, 1] - right_uv[valid, 1]) <= max_vertical_px))
        z = np.full(len(left_uv), np.nan)
        z[valid] = self.left.fx * self.baseline_m / disparity[valid]
        valid[valid] = np.isfinite(z[valid]) & (z[valid] <= max_depth_m)
        optical = np.full((len(z), 3), np.nan)
        optical[valid, 2] = z[valid]
        optical[valid, 0] = (left_uv[valid, 0] - self.left.cx) * z[valid] / self.left.fx
        optical[valid, 1] = (left_uv[valid, 1] - self.left.cy) * z[valid] / self.left.fy
        sensitivity = np.full(len(z), np.nan)
        sensitivity[valid] = z[valid] ** 2 / (self.left.fx * self.baseline_m)
        body = np.full(optical.shape, np.nan)
        body[valid] = self.left.optical_to_body(optical[valid])
        return body, valid, sensitivity


def load_nominal_rig(path, direction='front'):
    if direction not in ('front', 'back'):
        raise ValueError('unsupported stereo direction')
    data = Path(path).read_bytes()
    settings = json.loads(data)
    vehicle = settings['Vehicles']['drone_1']

    def scene(entries):
        matches = [s for s in entries if s['ImageType'] == 0]
        if len(matches) != 1:
            raise ValueError('exactly one Scene capture setting required')
        return matches[0]

    defaults = scene(settings['CameraDefaults']['CaptureSettings'])
    def camera(side):
        name = direction + '_' + side
        entry = vehicle['Cameras'][name]
        capture = {**defaults, **scene(entry['CaptureSettings'])}
        width, height, fov = capture['Width'], capture['Height'], float(capture['FOV_Degrees'])
        if not math.isfinite(fov) or not 5 < fov < 175:
            raise ValueError('invalid camera horizontal FOV')
        focal = width / (2 * math.tan(math.radians(fov) / 2))
        return NominalCamera(name, width, height, focal, focal, width / 2, height / 2, sensor_pose(entry))

    lidar = vehicle['Sensors']['lidar']
    if lidar['DataFrame'] != 'SensorLocalFrame':
        raise ValueError('nominal lidar transform requires SensorLocalFrame')
    return NominalStereoRig(camera('left'), camera('right'), sensor_pose(lidar), hashlib.sha256(data).hexdigest())
