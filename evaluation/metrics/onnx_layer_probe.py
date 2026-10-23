"""Extract marked ONNX subgraphs and compare TensorRT/ORT intermediates."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time

import numpy as np

from evaluation.metrics.onnx_stereo_probe import normalized_input
from evaluation.seed_runner.manifest import file_hash


def difference(a,b):
    values=np.abs(a.astype(np.float64)-b.astype(np.float64))
    return {'mean_abs':float(values.mean()),'p99_abs':float(np.percentile(values,99)),
        'max_abs':float(values.max()),'reference_abs_max':float(np.abs(b).max()),
        'finite':bool(np.isfinite(a).all() and np.isfinite(b).all())}


def main():
    import torch
    import onnx
    import onnxruntime as ort
    import tensorrt as trt

    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();run=args.run_dir;root=Path(__file__).resolve().parents[2]
    manifest=json.loads((run/'manifest.json').read_text());cfg=manifest['config']['onnx_probe']
    source=root/cfg['source_run'];onnx_path=root/cfg['onnx_run']/'stereo.onnx';layers=cfg['layers']
    if file_hash(onnx_path)!=cfg['onnx_sha256']:raise ValueError('original graph changed')
    started=time.monotonic()
    logger=trt.Logger(trt.Logger.INFO)
    if cfg.get('reuse_subgraph_run'):
        reused=root/cfg['reuse_subgraph_run']
        for name in ('subgraph.onnx','subgraph.engine'):shutil.copyfile(reused/name,run/name)
    else:
        onnx.utils.extract_model(str(onnx_path),str(run/'subgraph.onnx'),['left_image','right_image'],layers)
        builder=trt.Builder(logger);network=builder.create_network(1<<int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser_trt=trt.OnnxParser(network,logger)
        if not parser_trt.parse((run/'subgraph.onnx').read_bytes()):raise RuntimeError('subgraph parse failed')
        config=builder.create_builder_config();config.clear_flag(trt.BuilderFlag.TF32);config.max_aux_streams=0
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE,512*1024*1024)
        config.profiling_verbosity=trt.ProfilingVerbosity.DETAILED
        serialized=builder.build_serialized_network(network,config)
        if serialized is None:raise RuntimeError('subgraph build failed')
        (run/'subgraph.engine').write_bytes(bytes(serialized))
        del serialized,parser_trt,network,config,builder
    build_s=time.monotonic()-started
    options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
    options.graph_optimization_level=ort.GraphOptimizationLevel.ORT_DISABLE_ALL
    session=ort.InferenceSession(str(run/'subgraph.onnx'),sess_options=options,
        providers=[('CUDAExecutionProvider',{'use_tf32':'0','cudnn_conv_algo_search':'HEURISTIC',
            'cudnn_conv_use_max_workspace':'0'}),'CPUExecutionProvider'])
    if session.get_providers()[0]!='CUDAExecutionProvider':raise RuntimeError('CUDA provider unavailable')
    torch.set_num_threads(1);results=[];feeds={};references={}
    # ORT's expanded cost volume and TRT's context do not fit concurrently on 6GiB.
    # Retain CPU references first, release ORT, then allocate the TRT context.
    for key in cfg['keys']:
        with np.load(source/'inputs'/(key+'.npz'),allow_pickle=False) as saved:
            feed={'left_image':normalized_input(saved['left']),'right_image':normalized_input(saved['right'])}
        feeds[key]=feed;references[key]=session.run(layers,feed)
    del session
    import gc
    gc.collect();torch.cuda.empty_cache()
    runtime=trt.Runtime(logger);engine=runtime.deserialize_cuda_engine((run/'subgraph.engine').read_bytes())
    context=engine.create_execution_context()
    if context is None:raise RuntimeError('subgraph context creation failed')
    inspector=engine.create_engine_inspector()
    (run/'engine_inspector.json').write_text(inspector.get_engine_information(trt.LayerInformationFormat.JSON))
    for key in cfg['keys']:
        feed=feeds[key];expected=references[key]
        stream=torch.cuda.Stream();holders={}
        with torch.cuda.stream(stream):
            for i in range(engine.num_io_tensors):
                name=engine.get_tensor_name(i)
                if engine.get_tensor_dtype(name)!=trt.float32:raise ValueError('subgraph probe expects FP32 tensors')
                shape=tuple(engine.get_tensor_shape(name))
                if any(v<=0 for v in shape):raise ValueError('static subgraph tensor required')
                holders[name]=(torch.as_tensor(feed[name],device='cuda') if engine.get_tensor_mode(name)==trt.TensorIOMode.INPUT
                    else torch.empty(shape,device='cuda',dtype=torch.float32))
                context.set_tensor_address(name,int(holders[name].data_ptr()))
            if not context.execute_async_v3(stream.cuda_stream):raise RuntimeError('subgraph execute failed')
        stream.synchronize();arrays={};comparisons=[]
        for i,(name,reference) in enumerate(zip(layers,expected)):
            actual=holders[name].cpu().numpy();arrays['ort_%02d'%i]=reference;arrays['trt_%02d'%i]=actual
            comparisons.append({'tensor':name,'shape':list(reference.shape),'difference':difference(actual,reference)})
        np.savez_compressed(run/(key+'_layers.npz'),**arrays)
        results.append({'key':key,'normalized_input_sha256':hashlib.sha256(feed['left_image'].tobytes()+feed['right_image'].tobytes()).hexdigest(),
            'layers':comparisons});print(json.dumps(results[-1]),flush=True)
    report={'results':results,'build_s':None if cfg.get('reuse_subgraph_run') else build_s,
        'reused_from':cfg.get('reuse_subgraph_run'),'subgraph_sha256':file_hash(run/'subgraph.onnx'),
        'engine_sha256':file_hash(run/'subgraph.engine'),'ort':ort.__version__,'tensorrt':trt.__version__,
        'global_tf32_override':'0','marking_outputs_changes_optimization':True,
        'control_published':False,'current_control_eligible':False,'acceptance_eligible':False}
    (run/'layer_probe.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
