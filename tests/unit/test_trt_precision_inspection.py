import unittest

from evaluation.metrics.trt_precision_inspection import inspect_geometry_precision


class GeometryPrecisionInspectionTests(unittest.TestCase):
    constraints = [{'type': 'LayerType.GRID_SAMPLE', 'name': '/GridSample',
        'coordinate_shape': [11520, 1, 9, 2]}]

    def inspector(self, source='Float', output='Float'):
        return {'Layers': [{'Name': 'sample_fusion', 'Metadata': '[ONNX Layer: /GridSample]',
            'Inputs': [{'Name': 'grid', 'Dimensions': [11520, 1, 9, 2], 'Format/Datatype': source}],
            'Outputs': [{'Name': 'grid_cast', 'Dimensions': [11520, 1, 9, 2], 'Format/Datatype': output},
                {'Name': 'features', 'Dimensions': [1, 522, 96, 120], 'Format/Datatype': 'Half'}]}]}

    def test_float_coordinate_evidence_does_not_reject_half_feature_output(self):
        result = inspect_geometry_precision(self.inspector(), self.constraints)
        self.assertTrue(result['visible_coordinate_formats_verified'])
        self.assertFalse(result['per_operation_internal_precision_trace'])

    def test_half_source_cannot_be_certified_by_a_final_float_cast(self):
        result = inspect_geometry_precision(self.inspector('Half'), self.constraints)
        self.assertFalse(result['visible_coordinate_formats_verified'])
        self.assertEqual(result['non_fp32_coordinates'][0]['side'], 'Inputs')

    def test_missing_sampler_metadata_does_not_verify_precision(self):
        result = inspect_geometry_precision({'Layers': []}, self.constraints)
        self.assertFalse(result['visible_coordinate_formats_verified'])

    def test_missing_format_is_not_treated_as_float(self):
        result = inspect_geometry_precision(self.inspector(None), self.constraints)
        self.assertFalse(result['visible_coordinate_formats_verified'])


if __name__ == '__main__':
    unittest.main()
