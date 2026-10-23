"""Pinned upstream single-ONNX export and bounded TensorRT engine construction."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import time

from evaluation.seed_runner.manifest import file_hash


def load_exporter(vendor):
    spec=importlib.util.spec_from_file_location('official_single_onnx',vendor/'scripts/make_single_onnx.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def export(run,vendor,checkpoint,sample):
    os.environ['TORCH_COMPILE_DISABLE']='1';os.environ['TORCHDYNAMO_DISABLE']='1'
    import numpy as np
    import torch
    import onnx

    from perception.stereo_depth.learned import FastFoundationStereo
    profile=json.loads((run/'profile.json').read_text())
    official=load_exporter(vendor)
    backend=FastFoundationStereo(vendor,checkpoint,profile['checkpoint']['sha256'])
    model=backend.model;model.args.mixed_precision=False
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    import cv2
    cv2.setNumThreads(1);torch.set_num_threads(1)
    with np.load(sample,allow_pickle=False) as saved:
        images=[cv2.resize(saved[side],(480,360),interpolation=cv2.INTER_AREA) for side in ('left','right')]
    raw=[torch.as_tensor(image.copy(),device='cuda').float()[None].permute(0,3,1,2).contiguous() for image in images]
    padder=backend.padder(raw[0].shape,divis_by=32,force_square=False);left,right=padder.pad(*raw)
    import core.foundation_stereo as core
    normalize=core.normalize_image
    start=time.monotonic()
    with torch.inference_mode():
        original=model.forward(left,right,iters=4,test_mode=True,optimize_build_volume='pytorch1').float()
        normalized_left,normalized_right=normalize(left),normalize(right)
        core.normalize_image=lambda image:image
        core.build_gwc_volume_optimized_pytorch1=official._build_gwc_volume_onnx
        core.build_concat_volume_optimized_pytorch1=official._build_concat_volume_onnx
        wrapper=official.FastFoundationStereoSingleOnnx(model).cuda().eval()
        patched=wrapper(normalized_left,normalized_right).float()
    error=(patched-original).abs().cpu().numpy()
    comparison={'mean_abs_model_px':float(error.mean()),'p99_abs_model_px':float(np.percentile(error,99)),
        'max_abs_model_px':float(error.max()),'finite':bool(np.isfinite(error).all()),
        'p99_gate_model_px':.05,'max_gate_model_px':.5}
    comparison['passed']=comparison['finite'] and comparison['p99_abs_model_px']<=.05 and comparison['max_abs_model_px']<=.5
    (run/'export_parity.json').write_text(json.dumps(comparison,indent=2,allow_nan=False)+'\n')
    if not comparison['passed']:raise RuntimeError('upstream ONNX-compatible replacement differs from FP32 reference')
    # Upstream wrapper replaces unsupported volume construction and strips normalization.
    # Inputs are already normalized FP32. No original vendor bytes are modified.
    torch.onnx.export(wrapper,(normalized_left,normalized_right),str(run/'stereo.onnx'),
        opset_version=17,input_names=['left_image','right_image'],output_names=['disparity'],
        do_constant_folding=True,dynamo=False)
    graph=onnx.load(str(run/'stereo.onnx'));onnx.checker.check_model(graph)
    report={'onnx_sha256':file_hash(run/'stereo.onnx'),'opset':17,'image_shape':[1,3,384,480],
        'external_imagenet_normalization':True,'volume_normalize':bool(model.args.normalize),
        'mixed_precision_at_export':False,'valid_iters':4,'max_disp':192,
        'sample_sha256':file_hash(sample),'exporter_sha256':file_hash(vendor/'scripts/make_single_onnx.py'),
        'export_s':time.monotonic()-start,'torch':torch.__version__,'onnx':onnx.__version__,
        'normalization_compatibility_default_applied':backend.normalization_compatibility_default_applied,
        'source_weights_sha256':profile['checkpoint']['sha256'],'acceptance_eligible':False}
    (run/'export.json').write_text(json.dumps(report,indent=2)+'\n')


def build(run,precision):
    import tensorrt as trt
    import torch

    logger=trt.Logger(trt.Logger.INFO);start=time.monotonic()
    builder=trt.Builder(logger);network=builder.create_network(1<<int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser=trt.OnnxParser(network,logger)
    if not parser.parse((run/'stereo.onnx').read_bytes()):
        raise RuntimeError('ONNX parse failed: '+str([str(parser.get_error(i)) for i in range(parser.num_errors)]))
    config=builder.create_builder_config();config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE,512*1024*1024)
    config.clear_flag(trt.BuilderFlag.TF32);config.max_aux_streams=0
    if precision=='fp16':config.set_flag(trt.BuilderFlag.FP16)
    serialized=builder.build_serialized_network(network,config)
    if serialized is None:raise RuntimeError('TensorRT engine construction failed')
    engine_path=run/('stereo_'+precision+'.engine');engine_path.write_bytes(bytes(serialized))
    report={'engine_sha256':file_hash(engine_path),'engine_file':engine_path.name,'onnx_sha256':file_hash(run/'stereo.onnx'),
        'tensorrt':trt.__version__,'precision':precision,'tf32_enabled':False,'max_aux_streams':0,
        'workspace_limit_bytes':512*1024*1024,'build_s':time.monotonic()-start,
        'gpu':torch.cuda.get_device_name(0),'compute_capability':list(torch.cuda.get_device_capability(0)),
        'cuda_runtime':torch.version.cuda,'acceptance_eligible':False,'current_control_eligible':False}
    (run/('build_'+precision+'.json')).write_text(json.dumps(report,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--stage',choices=('export','build'),required=True)
    parser.add_argument('--vendor',type=Path);parser.add_argument('--checkpoint',type=Path);parser.add_argument('--sample',type=Path)
    parser.add_argument('--precision',choices=('fp16','fp32'),default='fp16');args=parser.parse_args()
    if args.stage=='export':
        if any(x is None for x in (args.vendor,args.checkpoint,args.sample)):parser.error('export requires vendor, checkpoint and sample')
        export(args.run_dir,args.vendor,args.checkpoint,args.sample)
    else:build(args.run_dir,args.precision)


if __name__=='__main__':main()
