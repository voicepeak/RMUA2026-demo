"""Run inside Noetic after building interfaces; never contacts a ROS service."""
import io
import struct
import unittest
try:
    from airsim_ros.srv import MeterReport,MeterReportRequest,MeterReportResponse
except ImportError:
    MeterReport=MeterReportRequest=MeterReportResponse=None


@unittest.skipUnless(MeterReport is not None,'run inside built ROS Noetic environment')
class MeterWireContractTests(unittest.TestCase):
    def test_request_layout_matches_observed_two_signed_64_bit_fields(self):
        self.assertEqual(MeterReport._md5sum,'40e1aa17dd96ff6a8c88396581d6a491')
        request=MeterReportRequest(index=1,value=50)
        buffer=io.BytesIO();request.serialize(buffer)
        self.assertEqual(buffer.getvalue(),struct.pack('<qq',1,50))

    def test_response_is_a_single_boolean(self):
        buffer=io.BytesIO();MeterReportResponse(success=True).serialize(buffer)
        self.assertEqual(buffer.getvalue(),b'\x01')
        self.assertTrue(MeterReportResponse().deserialize(buffer.getvalue()).success)


if __name__ == '__main__':
    unittest.main()
