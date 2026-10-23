"""Freeze shape-only Fourier grids, preserving the original learned projection."""
import hashlib


def freeze_fourier_inputs(model,left,right):
    import torch

    selected=[(name,module) for name,module in model.named_modules()
        if type(module).__name__=='PositionalEncodingFourier' and hasattr(module,'token_projection')]
    if not selected:raise ValueError('no supported shape-only Fourier modules in checkpoint')
    captured={};hooks=[]
    for name,module in selected:
        def shape_hook(_,args,key=name):
            captured.setdefault(key,{})['shape']=tuple(int(v) for v in args[0])
        def grid_hook(_,args,key=name):
            captured.setdefault(key,{})['grid']=args[0].detach().clone()
        hooks.append(module.register_forward_pre_hook(shape_hook))
        hooks.append(module.token_projection.register_forward_pre_hook(grid_hook))
    try:
        original=model.forward(left,right,iters=4,test_mode=True,optimize_build_volume='pytorch1')
    finally:
        for hook in hooks:hook.remove()

    class FrozenFourierInput(torch.nn.Module):
        def __init__(self,projection,shape,grid):
            super().__init__();self.token_projection=projection;self.expected_shape=shape
            # clone() preserves the original channels-last strides and dtype.
            self.register_buffer('fourier_input',grid)

        def forward(self,shape):
            if not torch.jit.is_tracing() and tuple(int(v) for v in shape)!=self.expected_shape:
                raise ValueError('frozen Fourier input shape changed')
            return self.token_projection(self.fourier_input)

    records=[]
    for name,module in selected:
        data=captured[name];grid=data['grid'];shape=data['shape']
        replacement=FrozenFourierInput(module.token_projection,shape,grid)
        parent,attribute=name.rsplit('.',1);setattr(model.get_submodule(parent),attribute,replacement)
        try:replacement((shape[0],shape[1]+1,shape[2]))
        except ValueError:shape_rejected=True
        else:shape_rejected=False
        if not shape_rejected:raise RuntimeError('frozen shape guard did not reject a different height')
        records.append({'module':name,'shape':list(shape),'input_shape':list(grid.shape),
            'input_stride':list(grid.stride()),'input_dtype':str(grid.dtype),
            'input_sha256':hashlib.sha256(grid.cpu().numpy().tobytes()).hexdigest(),
            'original_projection_preserved':replacement.token_projection is module.token_projection,
            'shape_mismatch_rejected':True})
    model.eval()
    return original,records
