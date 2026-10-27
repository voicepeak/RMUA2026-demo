"""Experimental orange-edge gate candidates with sparse stereo depth support."""
from dataclasses import dataclass, field
import math

import numpy as np

from interfaces.contracts import Metadata


@dataclass(frozen=True)
class GateCandidate:
    metadata: Metadata
    image_edges_px: tuple
    center_body_frd_m: tuple
    nominal_width_m: float
    nominal_height_m: float
    edge_disparities_px: tuple
    edge_support: tuple
    # Single-frame color/geometry proposals never certify aperture or clearance.
    aperture_verified: bool = field(default=False, init=False)
    metric_calibration_verified: bool = field(default=False, init=False)
    control_eligible: bool = field(default=False, init=False)


def _vertical_edges(image, encoding, cv2):
    if encoding == 'bgr8': bgr = image
    elif encoding == 'rgb8': bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    elif encoding == 'bgra8': bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    elif encoding == 'rgba8': bgr = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    else: raise ValueError('gate candidate requires color image')
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 100, 75), (18, 255, 255)) | cv2.inRange(hsv, (170, 100, 75), (179, 255, 255))
    vertical = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (1, 9)))
    _, _, stats, _ = cv2.connectedComponentsWithStats(vertical, 8)
    # Image borders can truncate a frame; never promote such an edge.
    h, w = image.shape[:2]
    edges = []
    for x, y, width, height, area in stats[1:]:
        x, y, width, height, area = map(int, (x, y, width, height, area))
        if (height < 20 or area < 18 or height < 2 * width or x < 4 or y < 4
                or x + width >= w - 4 or y + height >= h - 4):
            continue
        edges.append((x + (width - 1) / 2., float(y), float(y + height - 1), width))
    return sorted(edges, key=lambda e: (e[0], e[1]))[:128]



@dataclass(frozen=True)
class PreparedGateEdges:
    """Edges tied by identity to one immutable left image observation."""
    source_frame: object
    edges: tuple


def prepare_gate_edges(pair):
    import cv2
    return PreparedGateEdges(pair.left, tuple(_vertical_edges(pair.left.pixels, pair.left.encoding, cv2)))


def checked_gate_edges(pair, prepared):
    if not isinstance(prepared, PreparedGateEdges) or prepared.source_frame is not pair.left:
        raise ValueError('prepared gate edges belong to another image')
    return prepared.edges


def stereo_gate_candidates(pair, disparity, rig, now, epoch, prepared=None):
    """Produce nominal 3D proposals, never a gate passage or obstacle clearance.

    Conservative source-age checks apply to the original image/derived disparity.
    Edge depth is a regional median, not a measured precise opening polygon.
    """
    import cv2
    if (not pair.metadata.current(now, epoch) or not disparity.metadata.current(now, epoch)
            or disparity.metadata.epoch != pair.metadata.epoch
            or disparity.metadata.stamp != pair.metadata.stamp
            or disparity.metadata.valid_until != pair.metadata.valid_until
            or disparity.left_measurement_stamp != pair.left.metadata.stamp
            or disparity.right_measurement_stamp != pair.right.metadata.stamp
            or disparity.metadata.frame_id != 'front_left_image'
            or pair.left.pixels.shape[:2] != (rig.left.height, rig.left.width)
            or disparity.disparity_px.shape != (rig.left.height, rig.left.width)):
        raise ValueError('gate inputs do not share a current source pair')
    edges = (_vertical_edges(pair.left.pixels, pair.left.encoding, cv2) if prepared is None
             else checked_gate_edges(pair, prepared))
    h, w = disparity.disparity_px.shape
    supported = []
    for x, top, bottom, width in edges:
        x0, x1 = max(0, int(x - width / 2 - 4)), min(w, int(x + width / 2 + 5))
        y0, y1 = max(0, int(top + .15 * (bottom-top))), min(h, int(bottom - .15 * (bottom-top)) + 1)
        mask = disparity.valid[y0:y1, x0:x1]
        values = disparity.disparity_px[y0:y1, x0:x1][mask]
        if len(values) < 8: continue
        median = float(np.median(values)); deviation = float(np.median(np.abs(values - median)))
        if not math.isfinite(median) or median <= 0 or deviation > max(.75, .15 * median): continue
        supported.append((x, top, bottom, width, median, len(values)))
    candidates = []
    for index, left in enumerate(supported):
        for right in supported[index+1:]:
            lx, lt, lb, _, ld, lc = left; rx, rt, rb, _, rd, rc = right
            lh, rh = lb-lt, rb-rt
            if (rx-lx < .7 * max(lh, rh) or rx-lx > 6 * min(lh, rh)
                    or min(lh, rh) / max(lh, rh) < .65
                    or abs((lt+lb)-(rt+rb)) > .4 * max(lh, rh)
                    or abs(ld-rd) > max(.75, .2 * max(ld, rd))):
                continue
            left_pixels = np.array(((lx, lt), (lx, lb), (rx, rt), (rx, rb)))
            right_pixels = left_pixels.copy()
            right_pixels[:, 0] -= (ld, ld, rd, rd)
            body, valid, _ = rig.triangulate_body(left_pixels, right_pixels, max_depth_m=30.)
            if not valid.all(): continue
            lcenter, rcenter = (body[0]+body[1])/2., (body[2]+body[3])/2.
            width_m = float(np.linalg.norm(rcenter-lcenter))
            height_m = float((np.linalg.norm(body[1]-body[0])+np.linalg.norm(body[3]-body[2])) / 2.)
            if not (1. <= width_m <= 30. and 1. <= height_m <= 20.): continue
            meta = Metadata(pair.metadata.stamp, pair.metadata.valid_until, epoch, 'nominal_stereo_gate_candidate', 'body_frd')
            candidates.append(GateCandidate(meta, ((lx, lt, lb), (rx, rt, rb)),
                tuple((lcenter+rcenter)/2.), width_m, height_m, (ld, rd), (lc, rc)))
    return tuple(sorted(candidates, key=lambda item: (item.center_body_frd_m[0], -item.nominal_width_m))[:32])
