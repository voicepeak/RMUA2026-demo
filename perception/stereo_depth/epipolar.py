"""Sparse image correspondence diagnostics without a metric calibration claim."""
import numpy as np


def measure_sparse_epipolar(left_gray, right_gray):
    import cv2

    if (left_gray.dtype != np.uint8 or right_gray.dtype != np.uint8
            or left_gray.ndim != 2 or left_gray.shape != right_gray.shape):
        raise ValueError('matching grayscale byte images required')
    orb = cv2.ORB_create(nfeatures=2000, fastThreshold=10)
    kl, dl = orb.detectAndCompute(left_gray, None)
    kr, dr = orb.detectAndCompute(right_gray, None)
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)

    def reciprocal_candidates(a, b):
        if a is None or b is None or len(b) < 2:
            return {}
        return {m.queryIdx: m for group in matcher.knnMatch(a, b, k=2) if len(group) == 2
                for m, n in [group] if m.distance < .75 * n.distance}

    forward, backward = reciprocal_candidates(dl, dr), reciprocal_candidates(dr, dl)
    matches = [m for m in forward.values() if m.trainIdx in backward and backward[m.trainIdx].trainIdx == m.queryIdx]
    left = np.asarray([kl[m.queryIdx].pt for m in matches], dtype=np.float32).reshape(-1, 2)
    right = np.asarray([kr[m.trainIdx].pt for m in matches], dtype=np.float32).reshape(-1, 2)
    # Do not prefilter vertical residuals: that would make the check circular.
    fundamental, mask = (None, None)
    if len(matches) >= 8:
        fundamental, mask = cv2.findFundamentalMat(left, right, cv2.FM_RANSAC, 1., .999)
    inliers = mask.ravel().astype(bool) if fundamental is not None and mask is not None else np.zeros(len(matches), dtype=bool)
    disparity = left[:, 0] - right[:, 0]
    vertical = np.abs(left[:, 1] - right[:, 1])
    def quantiles(values):
        return [float(x) for x in np.percentile(values, (50, 95, 99, 100))] if len(values) else None
    h, w = left_gray.shape
    bins = {(min(3, int(x * 4 / w)), min(2, int(y * 3 / h))) for x, y in left[inliers]}
    report = {'left_features': len(kl), 'right_features': len(kr), 'reciprocal_ratio_matches': len(matches),
              'fundamental_inliers': int(inliers.sum()), 'inlier_spatial_bins_4x3': len(bins),
              'vertical_residual_all_p50_p95_p99_max_px': quantiles(vertical),
              'vertical_residual_inliers_p50_p95_p99_max_px': quantiles(vertical[inliers]),
              'positive_disparity_inliers': int(((disparity > 0) & inliers).sum()),
              'zero_or_negative_disparity_inliers': int(((disparity <= 0) & inliers).sum()),
              'vertical_prefilter_applied': False, 'rectification_verified': False,
              'metric_calibration_verified': False, 'acceptance_eligible': False}
    return report, left, right, inliers, fundamental
