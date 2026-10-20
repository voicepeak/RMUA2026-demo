"""Verify official wire precision in the built Noetic message package."""
import io
import struct
import unittest
try:
    from airsim_ros.msg import RotorPWM
except ImportError:
    RotorPWM=None


@unittest.skipUnless(RotorPWM is not None,'run inside built ROS Noetic environment')
class PwmSerializationTests(unittest.TestCase):
    def test_payload_is_four_float64_values_not_float32(self):
        values=(.123456789123,.234567891234,.345678912345,.456789123456)
        message=RotorPWM()
        message.rotorPWM0,message.rotorPWM1,message.rotorPWM2,message.rotorPWM3=values
        buffer=io.BytesIO();message.serialize(buffer)
        self.assertEqual(RotorPWM._md5sum,'4967432b608c06d70b17be20124fa258')
        self.assertEqual(len(buffer.getvalue()),16+32)
        self.assertEqual(buffer.getvalue()[-32:],struct.pack('<4d',*values))
        decoded=RotorPWM().deserialize(buffer.getvalue())
        self.assertEqual(tuple(getattr(decoded,'rotorPWM'+str(i)) for i in range(4)),values)


if __name__=='__main__':unittest.main()
