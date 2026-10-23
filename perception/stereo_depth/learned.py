"""Optional headless Fast-FoundationStereo research inference and pixel masks."""
import hashlib
import math
from pathlib import Path
import sys
import time

import numpy as np


def rgb_pixels(frame):
    image = frame.pixels
    if frame.encoding == 'mono8':
        return np.repeat(image, 3, axis=2)
    if frame.encoding in ('bgr8', 'bgra8'):
        return np.ascontiguousarray(image[:, :, (2, 1, 0)])
    if frame.encoding in ('rgb8', 'rgba8'):
        return np.ascontiguousarray(image[:, :, :3])
    raise ValueError('unsupported learned stereo encoding')


def full_resolution_disparity(disparity, width, height, scale):
    """Resize a model-grid disparity and convert its units to original pixels."""
    import cv2

    if scale not in (.5,1.) or disparity.shape != (int(height*scale),int(width*scale)):
        raise ValueError('incompatible model-grid disparity')
    if scale == 1:
        return disparity.astype(np.float32)
    return cv2.resize(disparity.astype(np.float32),(width,height),interpolation=cv2.INTER_NEAREST)/scale


def filter_learned_disparity(forward, reverse_positive, left_gray, right_gray,
                             consistency_px=1., min_texture_std=2.):
    """Full-resolution pixel units; right map has positive disparity magnitude.

    Returns masked disparity, shared texture/LR mask, and LR-only mask.
    None of these masks is a calibrated confidence or free-space certificate.
    """
    import cv2

    forward = np.asarray(forward, dtype=np.float32)
    reverse_positive = np.asarray(reverse_positive, dtype=np.float32)
    if (forward.ndim != 2 or forward.shape != reverse_positive.shape
            or forward.shape != left_gray.shape or left_gray.shape != right_gray.shape
            or left_gray.dtype != np.uint8 or right_gray.dtype != np.uint8
            or min(forward.shape) < 5 or not math.isfinite(consistency_px)
            or not 0 < consistency_px <= 1.5 or not math.isfinite(min_texture_std)
            or not 1 <= min_texture_std <= 20):
        raise ValueError('invalid learned disparity fields or rejection limits')
    h, w = forward.shape
    finite_forward = np.isfinite(forward)
    finite_forward[finite_forward] = (forward[finite_forward] > 0) & (forward[finite_forward] < w)
    safe = np.where(finite_forward, forward, 0.)
    right_x = np.arange(w, dtype=np.float32)[None, :] - safe
    # Only potentially in-image coordinates enter integer conversion.
    inside = (right_x >= 0) & (right_x < w - 1)
    lower = np.floor(np.where(inside, right_x, 0.)).astype(np.int64)
    weight = np.where(inside, right_x - lower, 0.)
    reverse_valid = np.isfinite(reverse_positive)
    reverse_valid[reverse_valid] = ((reverse_positive[reverse_valid] > 0)
                                  & (reverse_positive[reverse_valid] < w))
    reverse_safe = np.where(reverse_valid, reverse_positive, 0.)
    r0 = np.take_along_axis(reverse_safe, lower, axis=1)
    r1 = np.take_along_axis(reverse_safe, lower + 1, axis=1)
    right_ok = (np.take_along_axis(reverse_valid, lower, axis=1)
                & np.take_along_axis(reverse_valid, lower + 1, axis=1))
    difference = np.abs(safe - ((1-weight)*r0 + weight*r1))
    lr_valid = finite_forward & (safe > 0) & inside & right_ok & (difference <= consistency_px)
    lr_valid[:2] = False;lr_valid[-2:] = False
    lr_valid[:, :2] = False;lr_valid[:, -2:] = False

    def texture(gray):
        gray = gray.astype(np.float32)
        mean = cv2.boxFilter(gray, cv2.CV_32F, (5,5), borderType=cv2.BORDER_REPLICATE)
        square = cv2.boxFilter(gray*gray, cv2.CV_32F, (5,5), borderType=cv2.BORDER_REPLICATE)
        return np.maximum(square-mean*mean,0.) >= min_texture_std**2
    right_texture = texture(right_gray)
    common_valid = (lr_valid & texture(left_gray)
        & np.take_along_axis(right_texture,lower,axis=1)
        & np.take_along_axis(right_texture,lower+1,axis=1))
    return np.where(common_valid,forward,np.nan).astype(np.float32), common_valid, lr_valid


class FastFoundationStereo:
    """CUDA-only optional backend. No ROS, GUI, reference input or control output."""
    def __init__(self, vendor_root, checkpoint, checkpoint_sha256, scale=.5, iterations=4, max_disp=192, mixed_precision=True):
        if scale not in (.5,1.) or iterations not in (4,8) or max_disp != 192 or not isinstance(mixed_precision,bool):
            raise ValueError('unsupported frozen learned stereo profile')
        vendor_root, checkpoint = Path(vendor_root).resolve(), Path(checkpoint).resolve()
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != checkpoint_sha256:
            raise ValueError('learned checkpoint differs from its manifest')
        sys.path.insert(0,str(vendor_root))
        import torch
        import core.foundation_stereo  # Registers upstream checkpoint module aliases.
        from core.utils.utils import InputPadder

        if not torch.cuda.is_available():
            raise RuntimeError('CUDA is unavailable; a CPU run cannot substitute for this GPU benchmark')
        torch.manual_seed(0);torch.cuda.manual_seed_all(0)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False
        # Official serialized module checkpoint, pinned by publisher SHA256.
        self.model = torch.load(str(checkpoint),map_location='cpu',weights_only=False)
        # Official bp2 checkpoint omits this field. NVIDIA's TAO FFS docs require
        # normalized group-wise correlation; the upstream exporter defaults True.
        self.normalization_compatibility_default_applied = 'normalize' not in self.model.args
        if self.normalization_compatibility_default_applied:
            self.model.args.normalize = True
        self.model.args.valid_iters = iterations
        self.model.args.max_disp = max_disp
        self.model.args.mixed_precision = mixed_precision
        self.model.cuda().eval()
        self.torch, self.padder = torch, InputPadder
        self.scale, self.iterations, self.max_disp = scale, iterations, max_disp
        self.mixed_precision = mixed_precision
        self.checkpoint_sha256 = checkpoint_sha256

    def predict(self, left_rgb, right_rgb):
        import cv2

        if (left_rgb.dtype != np.uint8 or right_rgb.dtype != np.uint8
                or left_rgb.ndim != 3 or left_rgb.shape[2] != 3 or left_rgb.shape != right_rgb.shape):
            raise ValueError('matching RGB byte images required')
        torch = self.torch
        torch.cuda.synchronize()
        begin = time.monotonic()
        height, width = left_rgb.shape[:2]
        size = (int(width*self.scale),int(height*self.scale))
        if min(size) < 32 or size[0]/width != self.scale or size[1]/height != self.scale:
            raise ValueError('profile requires exact image scaling')
        left = cv2.resize(left_rgb,size,interpolation=cv2.INTER_AREA) if self.scale != 1 else left_rgb
        right = cv2.resize(right_rgb,size,interpolation=cv2.INTER_AREA) if self.scale != 1 else right_rgb
        def tensor(image):
            return torch.as_tensor(np.ascontiguousarray(image),device='cuda').float()[None].permute(0,3,1,2).contiguous()
        a,b = tensor(left),tensor(right)
        padder = self.padder(a.shape,divis_by=32,force_square=False)
        def one(x,y):
            x,y = padder.pad(x,y)
            output = self.model.forward(x,y,iters=self.iterations,test_mode=True,optimize_build_volume='pytorch1')
            return padder.unpad(output.float())
        with torch.inference_mode(),torch.amp.autocast('cuda',enabled=self.mixed_precision,dtype=torch.float16):
            forward = one(a,b)
            reverse = one(torch.flip(b,(-1,)),torch.flip(a,(-1,))).flip(-1)
        forward = forward.cpu().numpy().reshape(size[1],size[0])
        reverse = reverse.cpu().numpy().reshape(size[1],size[0])
        forward = full_resolution_disparity(forward,width,height,self.scale)
        reverse = full_resolution_disparity(reverse,width,height,self.scale)
        torch.cuda.synchronize()
        return forward.astype(np.float32),reverse.astype(np.float32),time.monotonic()-begin
