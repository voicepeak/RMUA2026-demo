import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/rmua_gate_vision/scripts'))
from latest_frame import LatestFrame

class LatestFrameTests(unittest.TestCase):
    def test_reset_discards_queued_image_from_previous_flight(self):
        box=LatestFrame()
        box.put(('old_frame','old_pose'))
        box.clear()
        self.assertIsNone(box.take(timeout=0.))
        box.put(('new_frame','new_pose'))
        self.assertEqual(box.take(),('new_frame','new_pose'))

    def test_slow_inference_receives_latest_frame_with_its_original_pose(self):
        box=LatestFrame()
        box.put(('frame1','pose1'))
        self.assertEqual(box.take(),('frame1','pose1'))
        for i in range(2,100):box.put(('frame'+str(i),'pose'+str(i)))
        self.assertEqual(box.take(),('frame99','pose99'))
        self.assertEqual(box.dropped,97)
        self.assertIsNone(box.take(timeout=0.))
