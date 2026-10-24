"""Optional, explicit FP32 constraints for a low-precision diagnostic build."""


POLICIES = ('default', 'sensitive_fp32', 'sensitive_geometry_fp32')


def geometry_precision_scope(network, trt):
    """Follow sampling inputs back to learned convolution/matmul boundaries.

    Include coordinate arithmetic before the sampling layer: promoting an already
    rounded grid at the final sampler cannot recover its original coordinates.
    No learned convolution/matmul is promoted by this traversal.
    """
    layers = [network.get_layer(i) for i in range(network.num_layers)]
    producer = {layer.get_output(j).name: i for i, layer in enumerate(layers)
        for j in range(layer.num_outputs)}
    reasons = {}
    def walk(tensor, reason, seen):
        index = producer.get(tensor.name)
        if index is None or index in seen:
            return
        seen.add(index)
        layer = layers[index]
        if layer.type in (trt.LayerType.CONVOLUTION, trt.LayerType.MATRIX_MULTIPLY):
            return
        reasons.setdefault(index, set()).add(reason)
        for j in range(layer.num_inputs):
            value = layer.get_input(j)
            if value is not None and value.dtype in (trt.float32, trt.float16):
                walk(value, reason, seen)
    for index, layer in enumerate(layers):
        if layer.type != trt.LayerType.GRID_SAMPLE:
            continue
        reasons.setdefault(index, set()).add('geometry_sampling')
        walk(layer.get_input(1), 'geometry_coordinates', set())
        walk(layer.get_input(0), 'geometry_values', set())
    if not any('geometry_sampling' in value for value in reasons.values()):
        raise RuntimeError('geometry policy requires actual GridSample layers')
    return reasons


def apply_precision_policy(network, config, trt, policy, precision):
    if policy not in POLICIES:
        raise ValueError('unknown TensorRT precision policy')
    if policy == 'default':
        return []
    if precision != 'fp16':
        raise ValueError('non-default policies are mixed FP16/FP32 experiments')

    geometry = geometry_precision_scope(network, trt) if policy == 'sensitive_geometry_fp32' else {}
    selected = []
    for index in range(network.num_layers):
        layer = network.get_layer(index)
        layer_type = layer.type
        sensitive = layer_type in (trt.LayerType.NORMALIZATION, trt.LayerType.REDUCE, trt.LayerType.SOFTMAX)
        if layer_type == trt.LayerType.ACTIVATION:
            # get_layer() exposes ILayer in the pinned Python bindings.
            layer.__class__ = trt.IActivationLayer
            sensitive = layer.type in (trt.ActivationType.SIGMOID, trt.ActivationType.TANH)
        reasons = set(geometry.get(index, ()))
        if sensitive:
            reasons.add('sensitive_operation')
        if not reasons or layer_type == trt.LayerType.CONSTANT:
            continue
        outputs = [i for i in range(layer.num_outputs)
            if layer.get_output(i).dtype in (trt.float32, trt.float16)]
        if not outputs:
            continue  # Integer shape calculations must keep their type.
        # Constrain input precision even for movement layers: otherwise the
        # builder can round a grid to Half before casting its output to Float.
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
            'selection_reasons': sorted(reasons),
            'coordinate_shape': list(layer.get_input(1).shape) if layer_type == trt.LayerType.GRID_SAMPLE else None,
            'normalization_compute_precision': 'fp32' if layer_type == trt.LayerType.NORMALIZATION else None})
    if not selected:
        raise RuntimeError('precision policy did not match any floating-point layers')
    # Fail a build when constraints cannot be honored; no silent precision fallback.
    config.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
    config.profiling_verbosity = trt.ProfilingVerbosity.DETAILED
    return selected
