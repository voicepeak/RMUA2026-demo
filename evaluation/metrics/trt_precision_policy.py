"""Optional, explicit FP32 constraints for a low-precision diagnostic build."""


POLICIES = ('default', 'sensitive_fp32')


def apply_precision_policy(network, config, trt, policy, precision):
    if policy not in POLICIES:
        raise ValueError('unknown TensorRT precision policy')
    if policy == 'default':
        return []
    if precision != 'fp16':
        raise ValueError('sensitive_fp32 is a mixed FP16/FP32 experiment')

    selected = []
    for index in range(network.num_layers):
        layer = network.get_layer(index)
        layer_type = layer.type
        sensitive = layer_type in (trt.LayerType.NORMALIZATION, trt.LayerType.REDUCE, trt.LayerType.SOFTMAX)
        if layer_type == trt.LayerType.ACTIVATION:
            # get_layer() exposes ILayer in the pinned Python bindings.
            layer.__class__ = trt.IActivationLayer
            sensitive = layer.type in (trt.ActivationType.SIGMOID, trt.ActivationType.TANH)
        if not sensitive:
            continue
        outputs = [i for i in range(layer.num_outputs)
            if layer.get_output(i).dtype in (trt.float32, trt.float16)]
        if not outputs:
            continue  # Integer shape calculations must keep their type.
        layer.precision = trt.float32
        for output in outputs:
            layer.set_output_type(output, trt.float32)
        if layer_type == trt.LayerType.NORMALIZATION:
            layer.__class__ = trt.INormalizationLayer
            layer.compute_precision = trt.float32
        if not layer.precision_is_set or layer.precision != trt.float32:
            raise RuntimeError('FP32 layer constraint not retained: ' + layer.name)
        if any(not layer.output_type_is_set(i) or layer.get_output_type(i) != trt.float32 for i in outputs):
            raise RuntimeError('FP32 output constraint not retained: ' + layer.name)
        selected.append({'index': index, 'name': layer.name, 'type': str(layer_type),
            'fp32_output_indices': outputs, 'requested_precision': 'fp32',
            'normalization_compute_precision': 'fp32' if layer_type == trt.LayerType.NORMALIZATION else None})
    if not selected:
        raise RuntimeError('precision policy did not match any floating-point layers')
    # Fail a build when constraints cannot be honored; no silent precision fallback.
    config.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
    config.profiling_verbosity = trt.ProfilingVerbosity.DETAILED
    return selected
