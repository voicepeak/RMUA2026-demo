import socket
import struct
import threading
import unittest

import numpy as np

from interfaces.contracts import Metadata
from perception.stereo_depth.transport import (LatestSlot,MAX_HEADER,MAX_PAYLOAD,
    send_packet,receive_packet,encode_images,decode_images,encode_disparity,decode_disparity,digest,freshness_reason)


class StereoTransportTests(unittest.TestCase):
    def test_pending_replacement_does_not_replace_consumed_inflight_item(self):
        slot=LatestSlot();slot.put('first');inflight=slot.take()
        for n in range(100):slot.put(n)
        self.assertEqual(inflight,'first');self.assertEqual(slot.take(),99)
        self.assertIsNone(slot.take());self.assertEqual(slot.stats()['superseded'],99)
        slot.put('stop');slot.close();self.assertFalse(slot.put('late'));self.assertIsNone(slot.take())
        self.assertEqual(slot.stats()['discarded_on_close'],1)

    def test_stream_packet_handles_fragmented_large_payload_without_pickle(self):
        left,right=socket.socketpair();left.settimeout(2);right.settimeout(2)
        body=b'abc123'*100000
        thread=threading.Thread(target=send_packet,args=(left,{'tag':'image'},body));thread.start()
        try:
            header,received=receive_packet(right);self.assertEqual(header,{'tag':'image'});self.assertEqual(received,body)
        finally:left.close();right.close();thread.join(2)

    def test_oversized_descriptors_and_truncated_packet_are_rejected_before_allocation(self):
        for sizes in ((MAX_HEADER+1,0),(1,MAX_PAYLOAD+1)):
            left,right=socket.socketpair()
            try:
                left.sendall(struct.pack('!II',*sizes))
                with self.assertRaises(ValueError):receive_packet(right)
            finally:left.close();right.close()
        left,right=socket.socketpair();left.sendall(b'abc');left.close()
        try:
            with self.assertRaises(EOFError):receive_packet(right)
        finally:right.close()

    def test_image_bytes_are_exact_and_corruption_is_rejected(self):
        a=np.arange(720*960*3,dtype=np.uint8).reshape(720,960,3);b=a[:,:,::-1]
        payload=encode_images(a,b);header={'image_shape':[720,960,3],'input_sha256':digest(payload)}
        x,y=decode_images(header,payload);np.testing.assert_array_equal(x,a);np.testing.assert_array_equal(y,b)
        with self.assertRaises(ValueError):decode_images(header,payload[:-1]+b'x')

    def test_disparity_transport_preserves_unknown_and_rejects_nonfinite_valid_values(self):
        valid=np.zeros((720,960),bool);valid[10:20,10:20]=True
        disparity=np.where(valid,8.,np.nan).astype(np.float32)
        restored,mask=decode_disparity(encode_disparity(disparity,valid))
        np.testing.assert_array_equal(mask,valid);self.assertTrue(np.isnan(restored[~mask]).all())
        self.assertTrue((restored[mask]==8).all())
        disparity[11,11]=np.inf
        with self.assertRaises(ValueError):encode_disparity(disparity,valid)

    def test_late_result_cannot_refresh_lease_and_clock_reset_or_stall_is_rejected(self):
        meta=Metadata(10.,10.2,4,'image','front_stereo')
        self.assertIsNone(freshness_reason(meta,10.19,4,True,30.,30.19))
        self.assertEqual(freshness_reason(meta,10.2,4,True,30.,30.19),'SOURCE_LEASE_NOT_CURRENT')
        self.assertEqual(freshness_reason(meta,10.19,5,True,30.,30.19),'OTHER_EPOCH')
        self.assertEqual(freshness_reason(meta,10.19,4,False,30.,30.19),'STALE_IMU_CLOCK')
        self.assertEqual(freshness_reason(meta,10.19,4,True,30.,30.25),'WALL_EXPIRED')
        self.assertEqual(meta.valid_until,10.2)


if __name__=='__main__':unittest.main()
