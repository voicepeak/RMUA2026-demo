"""Audit visible grid formats without mistaking a final Half-to-Float cast for preservation."""


def inspect_geometry_precision(inspector, constraints):
    samplers = [row for row in constraints if row['type'] == 'LayerType.GRID_SAMPLE']
    if not samplers:
        raise ValueError('sampler constraints required')
    shapes = {tuple(row['coordinate_shape']) for row in samplers}
    expected = {row['name'] for row in samplers}
    matched = set()
    coordinates = []
    for layer in inspector['Layers']:
        metadata = layer.get('Metadata', '')
        names = {name for name in expected if '[ONNX Layer: ' + name + ']' in metadata}
        if not names:
            continue
        matched.update(names)
        for side in ('Inputs', 'Outputs'):
            for tensor in layer.get(side, []):
                if tuple(tensor.get('Dimensions', ())) in shapes:
                    coordinates.append({'engine_layer': layer['Name'], 'side': side,
                        'tensor': tensor['Name'], 'format': tensor.get('Format/Datatype'),
                        'sampler_metadata': sorted(names)})
    rejected = [row for row in coordinates if row['format'] != 'Float']
    return {'expected_samplers': sorted(expected), 'matched_samplers': sorted(matched),
        'visible_coordinates': coordinates, 'non_fp32_coordinates': rejected,
        'visible_coordinate_formats_verified': bool(coordinates) and not rejected and matched == expected,
        'per_operation_internal_precision_trace': False,
        'interpretation': 'Visible Float coordinates and metadata coverage are necessary evidence; fused internals are not an instruction trace.'}
