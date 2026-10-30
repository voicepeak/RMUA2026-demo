import struct
import unittest
from evaluation.metrics.hud_capture import decode_xwd
from evaluation.metrics.hud_readout import parse_text


class HudCaptureTests(unittest.TestCase):
    def test_ocr_fields_require_unique_exact_labels(self):
        self.assertEqual(parse_text('Time:0\nScore: 0\nState:Pending1\n'),
                         {'time_s':0.,'score':0,'state':'Pending1'})
        result=parse_text('OutTime:4\nScore: 1\nScore: 2\nStateTime:5\n')
        self.assertEqual(result,{'time_s':None,'score':None,'state':None})

    def test_decodes_rgb_from_supported_xwd_and_rejects_corruption(self):
        h=[100,7,2,24,255,320,0,0,32,0,32,32,1020,5,0xff0000,0xff00,0xff,8,256,0,255,320,0,0,0]
        data=struct.pack('>25I',*h) + bytes([3,2,1,0])*(255*320)
        image=decode_xwd(data)
        self.assertEqual(image.size,(255,320))
        self.assertEqual(image.getpixel((0,0)),(1,2,3))
        with self.assertRaises(ValueError): decode_xwd(data[:-1])
        h[7]=1
        with self.assertRaises(ValueError): decode_xwd(struct.pack('>25I',*h)+data[100:])


if __name__ == '__main__': unittest.main()
