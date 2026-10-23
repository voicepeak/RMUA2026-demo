"""Controlled single-row GridSample comparison with the actual geometry shapes."""
import argparse
import json
from pathlib import Path
import time

import numpy as np


def reference_1d(image,grid):
    """Float64 analytic zero-padded bilinear interpolation for H=1, y=0."""
    if (image.ndim!=4 or image.shape[2]!=1 or min(image.shape[:2])<1 or image.shape[3]<2
            or grid.ndim!=4 or grid.shape[0]!=image.shape[0] or grid.shape[1]!=1 or grid.shape[2]<1 or grid.shape[3]!=2
            or not np.isfinite(image).all() or not np.isfinite(grid).all() or np.any(grid[:,:,:,1]!=0)):
        raise ValueError('finite single-row, zero-y geometry sampler inputs required')
    n,c,_,width=image.shape
    x=(grid[:,0,:,0].astype(np.float64)+1)*(width-1)/2
    floor=np.floor(x).astype(np.int64);fraction=x-floor
    rows=np.arange(n)[:,None,None];channels=np.arange(c)[None,:,None]
    low=image[rows,channels,0,np.clip(floor,0,width-1)[:,None,:]].astype(np.float64)
    high=image[rows,channels,0,np.clip(floor+1,0,width-1)[:,None,:]].astype(np.float64)
    low*=((floor>=0)&(floor<width))[:,None,:]
    high*=((floor+1>=0)&(floor+1<width))[:,None,:]
    return ((1-fraction[:,None,:])*low+fraction[:,None,:]*high)[:,:,None,:]


def errors(value,expected):
    if not np.isfinite(value).all() or not np.isfinite(expected).all():
        return {'mean_abs':None,'p99_abs':None,'max_abs':None,'finite':False,'absolute_gate':.0005,'passed':False}
    error=np.abs(value.astype(np.float64)-expected)
    return {'mean_abs':float(error.mean()),'p99_abs':float(np.percentile(error,99)),
        'max_abs':float(error.max()),'finite':bool(np.isfinite(value).all()),
        'absolute_gate':.0005,'passed':bool(np.isfinite(value).all() and error.max()<=.0005)}


def main():
    import torch
    import onnx
    import onnxruntime as ort
    import tensorrt as trt
    from onnx import helper,TensorProto

    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();run=args.run_dir
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    torch.set_num_threads(1);random=np.random.RandomState(4097);results=[]
    logger=trt.Logger(trt.Logger.WARNING)
    for batch in (1,11520):
        for channels,width in ((28,48),(28,24),(1,120),(1,60)):
            key='n%d_c%d_w%d'%(batch,channels,width);started=time.monotonic()
            image=random.normal(size=(batch,channels,1,width)).astype(np.float32)
            grid=np.zeros((batch,1,9,2),np.float32);grid[:,:,:,0]=random.uniform(-1.5,1.5,(batch,1,9))
            expected=reference_1d(image,grid)
            node=helper.make_node('GridSample',['image','grid'],['result'],mode='bilinear',padding_mode='zeros',align_corners=1)
            graph=helper.make_graph([node],key,[helper.make_tensor_value_info('image',TensorProto.FLOAT,list(image.shape)),
                helper.make_tensor_value_info('grid',TensorProto.FLOAT,list(grid.shape))],
                [helper.make_tensor_value_info('result',TensorProto.FLOAT,[batch,channels,1,9])])
            model=helper.make_model(graph,opset_imports=[helper.make_opsetid('',17)]);model.ir_version=8
            onnx.checker.check_model(model);onnx.save(model,str(run/(key+'.onnx')))
            options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
            options.graph_optimization_level=ort.GraphOptimizationLevel.ORT_DISABLE_ALL
            session=ort.InferenceSession(model.SerializeToString(),sess_options=options,
                providers=[('CUDAExecutionProvider',{'use_tf32':'0'}),'CPUExecutionProvider'])
            if session.get_providers()[0]!='CUDAExecutionProvider':raise RuntimeError('CUDA provider unavailable')
            ort_value=session.run(None,{'image':image,'grid':grid})[0]
            a=torch.as_tensor(image,device='cuda');b=torch.as_tensor(grid,device='cuda')
            torch_value=torch.nn.functional.grid_sample(a,b,align_corners=True).cpu().numpy()
            builder=trt.Builder(logger);network=builder.create_network(1<<int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
            parser_trt=trt.OnnxParser(network,logger)
            if not parser_trt.parse(model.SerializeToString()):raise RuntimeError('GridSample parse failed')
            config=builder.create_builder_config();config.clear_flag(trt.BuilderFlag.TF32);config.max_aux_streams=0
            config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE,128*1024*1024)
            serialized=builder.build_serialized_network(network,config)
            if serialized is None:raise RuntimeError('GridSample engine build failed')
            (run/(key+'.engine')).write_bytes(bytes(serialized))
            runtime=trt.Runtime(logger);engine=runtime.deserialize_cuda_engine(serialized);context=engine.create_execution_context()
            if context is None:raise RuntimeError('GridSample context creation failed')
            out=torch.empty((batch,channels,1,9),device='cuda',dtype=torch.float32)
            stream=torch.cuda.Stream();stream.wait_stream(torch.cuda.current_stream())
            context.set_tensor_address('image',int(a.data_ptr()));context.set_tensor_address('grid',int(b.data_ptr()))
            context.set_tensor_address('result',int(out.data_ptr()))
            if not context.execute_async_v3(stream.cuda_stream):raise RuntimeError('GridSample execute failed')
            stream.synchronize();trt_value=out.cpu().numpy()
            row={'key':key,'shape':list(image.shape),'torch':errors(torch_value,expected),
                'onnxruntime':errors(ort_value,expected),'tensorrt':errors(trt_value,expected),
                'tensorrt_vs_torch':errors(trt_value,torch_value.astype(np.float64)),
                'elapsed_s':time.monotonic()-started}
            np.savez_compressed(run/(key+'.npz'),image=image,grid=grid,expected=expected,
                pytorch=torch_value,onnxruntime=ort_value,tensorrt=trt_value)
            results.append(row);print(json.dumps(row),flush=True)
            del context,engine,runtime,serialized,parser_trt,network,config,builder,session,a,b,out
    (run/'sampler_probe.json').write_text(json.dumps({'results':results,'torch':torch.__version__,
        'onnxruntime':ort.__version__,'tensorrt':trt.__version__,'seed':4097,
        'control_published':False,'acceptance_eligible':False,'current_control_eligible':False},indent=2)+'\n')


if __name__=='__main__':main()
