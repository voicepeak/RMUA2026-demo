"""Sparse stereo visual odometry; nominal geometry, no control authorization."""
from dataclasses import dataclass, field
import math

import numpy as np

from perception.stereo_depth.sgbm import _gray


@dataclass(frozen=True)
class RelativeMotion:
    # Transform from the current body frame to the previous body frame.
    translation_previous_body_m: tuple
    rotation_previous_from_current_body: tuple
    stereo_matches: int
    temporal_matches: int
    inliers: int
    reprojection_p95_px: float
    previous_stamp: float
    current_stamp: float
    epoch: int
    metric_calibration_verified: bool = field(default=False, init=False)
    control_eligible: bool = field(default=False, init=False)


def _reciprocal(first, second, cv2):
    if first is None or second is None or len(first) < 2 or len(second) < 2:
        return []
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    def good(a, b):
        return {m.queryIdx: m for group in matcher.knnMatch(a, b, k=2) if len(group) == 2
                for m, n in [group] if m.distance < .75 * n.distance}
    forward, backward = good(first, second), good(second, first)
    return [m for m in forward.values() if m.trainIdx in backward
            and backward[m.trainIdx].trainIdx == m.queryIdx]


def solve_relative(points_body_previous, pixels_current, rig, *, min_inliers=12):
    """Estimate body-current pose in body-previous, using nominal stereo geometry.

    PnP maps previous optical points into the current optical frame; we invert
    that transform, then conjugate through the fixed camera-to-body mount. A
    RANSAC result is rejected for poor coverage, inliers or reprojection error.
    The result is diagnostic until calibration, timing and moving-scene checks.
    """
    import cv2
    body = np.asarray(points_body_previous, dtype=np.float64)
    pixels = np.asarray(pixels_current, dtype=np.float64)
    if (body.ndim != 2 or body.shape[1:] != (3,) or pixels.shape != (len(body), 2)
            or len(body) < min_inliers or not np.isfinite(body).all() or not np.isfinite(pixels).all()
            or not isinstance(min_inliers, int) or isinstance(min_inliers, bool) or min_inliers < 6):
        raise ValueError('invalid visual odometry correspondences')
    camera = rig.left
    if (np.any(pixels[:, 0] < 0) or np.any(pixels[:, 0] >= camera.width)
            or np.any(pixels[:, 1] < 0) or np.any(pixels[:, 1] >= camera.height)):
        raise ValueError('image points outside camera')
    optical = camera.body_to_optical(body)
    if np.any(optical[:, 2] <= 0):
        raise ValueError('nonpositive previous optical depth')
    # Reject concentrated or effectively collinear observations.
    singular = np.linalg.svd(optical - optical.mean(axis=0), compute_uv=False)
    if len(singular) < 2 or singular[1] < .05:
        raise ValueError('degenerate geometry')
    k = np.array(((camera.fx, 0., camera.cx), (0., camera.fy, camera.cy), (0., 0., 1.)))
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(optical, pixels, k, None,
        iterationsCount=150, reprojectionError=2., confidence=.99, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None or len(inliers) < min_inliers or len(inliers) / len(body) < .5:
        raise ValueError('insufficient PnP inliers')
    indices = inliers.ravel()
    bins = {(min(3, int(u * 4 / camera.width)), min(2, int(v * 3 / camera.height)))
            for u, v in pixels[indices]}
    if len(bins) < 4:
        raise ValueError('insufficient image coverage')
    rotation, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(optical[indices], rvec, tvec, k, None)
    errors = np.linalg.norm(projected.reshape(-1, 2) - pixels[indices], axis=1)
    p95 = float(np.percentile(errors, 95))
    if not math.isfinite(p95) or p95 > 2. or np.any((optical[indices] @ rotation.T + tvec.ravel())[:, 2] <= 0):
        raise ValueError('invalid PnP reprojection')
    # Optical (right, down, forward) -> sensor FRD (forward, right, down).
    optical_to_frd = np.array(((0., 0., 1.), (1., 0., 0.), (0., 1., 0.)))
    b_from_c = np.array(camera.pose.rotation_body_from_sensor_frd) @ optical_to_frd
    mount = np.asarray(camera.pose.translation_body_frd)
    prev_from_cur_opt = rotation.T
    prev_from_cur_body = b_from_c @ prev_from_cur_opt @ b_from_c.T
    # Invert x_cur = R x_prev + t; account for the rotating camera lever arm.
    displacement = b_from_c @ (-rotation.T @ tvec.ravel()) + mount - prev_from_cur_body @ mount
    return tuple(displacement), tuple(tuple(float(v) for v in row) for row in prev_from_cur_body), len(indices), p95


def estimate_stereo_motion(previous, current, rig, max_gap_s=2.):
    """Extract stereo/temporal ORB tracks and estimate nominal relative motion."""
    import cv2
    if (previous.metadata.epoch != current.metadata.epoch
            or previous.skew_s > .005 or current.skew_s > .005
            or not math.isfinite(max_gap_s) or not 0 < max_gap_s <= 5
            or not 0 < current.metadata.stamp - previous.metadata.stamp <= max_gap_s):
        raise ValueError('stereo pair epoch or time mismatch')
    shape = (rig.left.height, rig.left.width)
    if any(frame.pixels.shape[:2] != shape for frame in (previous.left, previous.right, current.left)):
        raise ValueError('stereo dimensions differ from camera model')
    orb = cv2.ORB_create(nfeatures=2000, fastThreshold=10)
    features = [orb.detectAndCompute(_gray(frame, cv2), None)
                for frame in (previous.left, previous.right, current.left)]
    (kl, dl), (kr, dr), (kc, dc) = features
    stereo, temporal = _reciprocal(dl, dr, cv2), _reciprocal(dl, dc, cv2)
    stereo_by_left = {m.queryIdx: m.trainIdx for m in stereo}
    joined = [m for m in temporal if m.queryIdx in stereo_by_left]
    left = np.array([kl[m.queryIdx].pt for m in joined], dtype=np.float64).reshape(-1, 2)
    right = np.array([kr[stereo_by_left[m.queryIdx]].pt for m in joined], dtype=np.float64).reshape(-1, 2)
    target = np.array([kc[m.trainIdx].pt for m in joined], dtype=np.float64).reshape(-1, 2)
    body, valid, _ = rig.triangulate_body(left, right, max_depth_m=30., max_vertical_px=1.)
    translation, rotation, inliers, p95 = solve_relative(body[valid], target[valid], rig)
    return RelativeMotion(translation, rotation, len(stereo), len(joined), inliers, p95,
                          previous.metadata.stamp, current.metadata.stamp, current.metadata.epoch)
