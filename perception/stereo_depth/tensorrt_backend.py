"""Optional pinned static-shape TensorRT stereo candidate, no ROS or control."""
from pathlib import Path
import time

import numpy as np

from evaluation.seed_runner.manifest import file_hash
from perception.stereo_depth.learned import full_resolution_disparity


class TensorRTStereo:
    def __init__(self,engine_path,engine_sha256):
        import torch
        import tensorrt as trt
        from core.utils.utils import InputPadder

        engine_path=Path(engine_path)
        if file_hash(engine_path)!=engine_sha256:raise ValueError('engine differs from the frozen build')
        self.torch,self.trt,self.padder=torch,trt,InputPadder
        self.logger=trt.Logger(trt.Logger.WARNING);self.runtime=trt.Runtime(self.logger)
        self.engine=self.runtime.deserialize_cuda_engine(engine_path.read_bytes())
        if self.engine is None:raise RuntimeError('TensorRT engine cannot be deserialized')
        names={self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)}
        if names!={'left_image','right_image','disparity'}:raise ValueError('unexpected TensorRT IO names')
        for name in ('left_image','right_image'):
            if (self.engine.get_tensor_mode(name)!=trt.TensorIOMode.INPUT
                    or tuple(self.engine.get_tensor_shape(name))!=(1,3,384,480)
                    or self.engine.get_tensor_dtype(name)!=trt.float32):raise ValueError('unexpected TensorRT input contract')
        if (self.engine.get_tensor_mode('disparity')!=trt.TensorIOMode.OUTPUT
                or tuple(self.engine.get_tensor_shape('disparity'))!=(1,1,384,480)
                or self.engine.get_tensor_dtype('disparity')!=trt.float32):raise ValueError('unexpected TensorRT output contract')
        self.context=self.engine.create_execution_context()
        self.output=torch.empty((1,1,384,480),device='cuda',dtype=torch.float32)
        self.mean=torch.tensor([.485,.456,.406],device='cuda',dtype=torch.float32).view(1,3,1,1)
        self.std=torch.tensor([.229,.224,.225],device='cuda',dtype=torch.float32).view(1,3,1,1)
        self.engine_sha256=engine_sha256

    def predict(self,left_rgb,right_rgb):
        import cv2

        if left_rgb.shape!=(720,960,3) or right_rgb.shape!=left_rgb.shape or left_rgb.dtype!=np.uint8 or right_rgb.dtype!=np.uint8:
            raise ValueError('TensorRT candidate requires frozen 960x720 RGB byte input')
        torch=self.torch;torch.cuda.synchronize();begin=time.monotonic()
        def tensor(image):
            small=cv2.resize(image,(480,360),interpolation=cv2.INTER_AREA)
            value=torch.as_tensor(small.copy(),device='cuda').float()[None].permute(0,3,1,2).contiguous()
            return ((value/255.)-self.mean)/self.std
        a,b=tensor(left_rgb),tensor(right_rgb)
        padder=self.padder(a.shape,divis_by=32,force_square=False)
        def one(left,right):
            left,right=padder.pad(left,right)
            self.context.set_tensor_address('left_image',int(left.data_ptr()))
            self.context.set_tensor_address('right_image',int(right.data_ptr()))
            self.context.set_tensor_address('disparity',int(self.output.data_ptr()))
            if not self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream):
                raise RuntimeError('TensorRT execution failed')
            # The second invocation reuses its output buffer; own this result now.
            return padder.unpad(self.output).clone()
        with torch.inference_mode():
            forward=one(a,b)
            reverse=one(torch.flip(b,(-1,)),torch.flip(a,(-1,))).flip(-1)
        forward=forward.cpu().numpy().reshape(360,480)
        reverse=reverse.cpu().numpy().reshape(360,480)
        forward=full_resolution_disparity(forward,960,720,.5)
        reverse=full_resolution_disparity(reverse,960,720,.5)
        torch.cuda.synchronize()
        return forward,reverse,time.monotonic()-begin
