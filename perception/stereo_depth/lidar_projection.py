"""Nominal LiDAR/camera header-pose alignment, without per-point timing."""
import numpy as np

from estimation.geometry import rotation
from perception.stereo_depth.geometry import points


def project_lidar_into_stereo(sensor_points, scan_pose, left_pose, right_pose, rig):
    poses = (scan_pose, left_pose, right_pose)
    if (len({pose.epoch for pose in poses}) != 1
            or any(pose.frame_id != 'world_ned' for pose in poses)
            or max(abs(scan_pose.stamp - pose.stamp) for pose in poses[1:]) > .05):
        raise ValueError('incompatible historical poses for nominal projection')
    sensor_points = points(sensor_points, 3)
    ranges = np.hypot(np.hypot(sensor_points[:, 0], sensor_points[:, 1]), sensor_points[:, 2])
    finite = np.isfinite(ranges)
    keep = np.zeros(len(sensor_points), dtype=bool)
    keep[finite] = (ranges[finite] > 0) & (ranges[finite] < 30)
    world = np.full(sensor_points.shape, np.nan)
    body_at_scan = rig.lidar_pose.to_body(sensor_points[keep])
    world[keep] = body_at_scan @ rotation(scan_pose.orientation_wxyz).T + scan_pose.position_ned
    outputs = []
    valid = keep.copy()
    for camera, pose in ((rig.left, left_pose), (rig.right, right_pose)):
        body = np.full(sensor_points.shape, np.nan)
        body[keep] = (world[keep] - pose.position_ned) @ rotation(pose.orientation_wxyz)
        uv, depth, inside = camera.project_body(body)
        valid &= inside
        outputs.extend((uv, depth))
    return outputs[0], outputs[1], outputs[2], outputs[3], valid


def sparse_visible(uv, depth, valid, width, height):
    """Nearest retained return per pixel; not a complete visibility proof."""
    visible = np.zeros(len(uv), dtype=bool)
    indices = np.flatnonzero(valid)
    if not len(indices):return visible
    pixel = np.floor(uv[indices]).astype(np.int64)
    if np.any(pixel < 0) or np.any(pixel[:, 0] >= width) or np.any(pixel[:, 1] >= height):
        raise ValueError('out-of-view samples cannot enter sparse z-buffer')
    keys = pixel[:, 1] * width + pixel[:, 0]
    nearest = np.full(width * height, np.inf)
    np.minimum.at(nearest, keys, depth[indices])
    visible[indices] = depth[indices] <= nearest[keys] + 1e-8
    return visible


def sample_disparity(uv, disparity, valid_pixels, candidates):
    """Bilinear sampling only when all four neighbors are valid."""
    if disparity.ndim != 2 or disparity.shape != valid_pixels.shape:
        raise ValueError('disparity and mask shapes differ')
    h, w = disparity.shape
    valid = np.zeros(len(uv), dtype=bool)
    initial = np.flatnonzero(candidates)
    if len(initial):
        valid[initial] = ((uv[initial, 0] >= 0) & (uv[initial, 0] < w - 1)
                          & (uv[initial, 1] >= 0) & (uv[initial, 1] < h - 1))
    indices = np.flatnonzero(valid)
    values = np.full(len(uv), np.nan)
    if not len(indices):return values, valid
    xy = np.floor(uv[indices]).astype(np.int64)
    x, y = xy[:, 0], xy[:, 1]
    good = valid_pixels[y, x] & valid_pixels[y, x + 1] & valid_pixels[y + 1, x] & valid_pixels[y + 1, x + 1]
    valid[indices] = good
    indices, x, y = indices[good], x[good], y[good]
    dx, dy = uv[indices, 0] - x, uv[indices, 1] - y
    values[indices] = ((1-dx)*(1-dy)*disparity[y,x] + dx*(1-dy)*disparity[y,x+1]
                       + (1-dx)*dy*disparity[y+1,x] + dx*dy*disparity[y+1,x+1])
    return values, valid
