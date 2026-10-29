import sys
import types
import unittest
from unittest.mock import patch

from adapters.ros_noetic.service_contract_audit import observe


class ServiceInventoryTest(unittest.TestCase):
    def test_inventory_only_probes_advertised_candidates(self):
        queried = []
        rosservice = types.ModuleType('rosservice')
        rosservice.get_service_uri = lambda name: 'rosrpc://localhost:1'
        rosservice.get_service_headers = lambda name, uri: {'type': 'airsim_ros/MeterReport',
                                                             'md5sum': '40e1aa17dd96ff6a8c88396581d6a491'}
        rosservice.get_service_list = lambda: ['/airsim_node/meter_report', '/airsim_node/drone_1/land']
        def get_type(name):
            queried.append(name)
            return 'airsim_ros/Land'
        rosservice.get_service_type = get_type
        roslib = types.ModuleType('roslib')
        message = types.ModuleType('roslib.message')
        message.get_service_class = lambda name: types.SimpleNamespace(_md5sum='40e1aa17dd96ff6a8c88396581d6a491')
        roslib.message = message
        with patch.dict(sys.modules, {'rosservice': rosservice, 'roslib': roslib, 'roslib.message': message}):
            result = observe()
        self.assertTrue(result['compatible'])
        self.assertEqual(queried, ['/airsim_node/drone_1/land'])
        self.assertEqual(result['local_candidate_types']['/airsim_node/drone_1/land'], 'airsim_ros/Land')
        self.assertIsNone(result['local_candidate_types']['/airsim_node/reset'])
        self.assertFalse(result['candidate_service_requests_sent'])
        self.assertFalse(result['candidate_service_semantics_verified'])


if __name__ == '__main__':
    unittest.main()
