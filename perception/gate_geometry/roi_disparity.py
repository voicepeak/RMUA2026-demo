"""Optional edge-local stereo disparity for gate proposals only."""
import numpy as np

from interfaces.contracts import Metadata
from perception.gate_geometry.stereo_gate import _vertical_edges
from perception.stereo_depth.pairing import ImageFrame, StereoPair
from perception.stereo_depth.sgbm import DisparityObservation, SgbmConfig, compute_diagnostic_disparity


def gate_region_disparity(pair, config=None):
    """Match in an edge-bounded crop, returning UNKNOWN everywhere else.

    Both images use the same crop origin so disparity remains in original pixel
    units. SGBM may differ from a full-image computation due to image context.
    The ROI is only a diagnostic gate proposal input, never free-space evidence.
    """
    import cv2
    config = config or SgbmConfig()
    if pair.skew_s > .005:
        raise ValueError('incompatible diagnostic stereo input')
    edges = _vertical_edges(pair.left.pixels, pair.left.encoding, cv2)
    height, width = pair.left.pixels.shape[:2]
    values = np.full((height, width), np.nan, dtype=np.float32)
    valid = np.zeros((height, width), dtype=bool)
    if edges:
        # Give both directions a full disparity search margin in x, and avoid
        # letting ROI edges pretend to be image borders in y.
        padding = config.num_disparities + config.block_size + 5
        ypad = max(16, config.block_size * 3)
        x0 = max(0, int(min(x-w/2 for x,_,_,w in edges)) - padding)
        x1 = min(width, int(max(x+w/2 for x,_,_,w in edges)) + padding + 1)
        y0 = max(0, int(min(top for _,top,_,_ in edges)) - ypad)
        y1 = min(height, int(max(bottom for _,_,bottom,_ in edges)) + ypad + 1)
        if x1-x0 > config.num_disparities + config.block_size and y1-y0 > 2*config.block_size:
            def crop(frame):
                return ImageFrame(frame.metadata, frame.encoding, frame.pixels[y0:y1, x0:x1])
            cropped = StereoPair(pair.metadata, crop(pair.left), crop(pair.right), pair.skew_s)
            sub = compute_diagnostic_disparity(cropped, config)
            # Only the actual edge support windows can be consumed. Other
            # pixels, including the rest of the crop, stay explicitly UNKNOWN.
            keep = np.zeros((height, width), dtype=bool)
            for x, top, bottom, edge_width in edges:
                ax0=max(0,int(x-edge_width/2-4)); ax1=min(width,int(x+edge_width/2+5))
                ay0=max(0,int(top+.15*(bottom-top))); ay1=min(height,int(bottom-.15*(bottom-top))+1)
                keep[ay0:ay1,ax0:ax1]=True
            crop_keep = keep[y0:y1,x0:x1] & sub.valid
            valid[y0:y1,x0:x1] = crop_keep
            region = values[y0:y1,x0:x1]
            region[crop_keep] = sub.disparity_px[crop_keep]
    meta=Metadata(pair.metadata.stamp,pair.metadata.valid_until,pair.metadata.epoch,
                  'gate_roi_sgbm_diagnostic_pixels','front_left_image')
    return DisparityObservation(meta,values,valid,pair.left.metadata.stamp,pair.right.metadata.stamp)
