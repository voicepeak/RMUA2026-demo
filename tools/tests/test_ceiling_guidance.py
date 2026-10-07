import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from point_index import PointIndex
from ceiling_guidance import ceiling_guidance


class CeilingGuidanceTests(unittest.TestCase):
    def setUp(self):
        self.xy=lambda s:(s,0.,0.)
        self.base=lambda s:0.
        points=np.array([(x,y,-3.+.1*x) for x in np.arange(-9.,10.,.3)
                        for y in np.arange(-3.,3.1,.3)])
        self.index=PointIndex(points,np.zeros(3),road_height=5.)

    def test_measured_slope_is_used_only_inside_supported_width_and_hull(self):
        result=ceiling_guidance(self.index,1.,1.05,0.,self.xy,self.base)
        self.assertIsNotNone(result)
        stations,heights=result
        self.assertGreaterEqual(stations[-1],4.)
        self.assertLess(stations[-1],10.)
        np.testing.assert_allclose(heights,-.5+.1*stations,atol=1e-5)
        self.assertIsNone(ceiling_guidance(self.index,1.,1.05,8.,self.xy,self.base))

    def test_stale_scan_or_inconsistent_height_cannot_extend_map(self):
        self.assertIsNone(ceiling_guidance(self.index,1.,1.31,0.,self.xy,self.base))
        self.assertIsNone(ceiling_guidance(self.index,1.,.99,0.,self.xy,self.base))
        self.assertIsNone(ceiling_guidance(self.index,1.,1.05,0.,self.xy,lambda s:2.))

    def test_vertical_walls_do_not_create_ceiling_evidence(self):
        walls=np.array([(x,y,z) for x in np.arange(-10.,10.,.3)
                        for y in (-3.,3.) for z in np.arange(-4.,1.,.3)])
        index=PointIndex(walls,np.zeros(3),road_height=5.)
        self.assertIsNone(ceiling_guidance(index,1.,1.05,0.,self.xy,self.base))

    def test_tapered_near_hull_needs_continuous_measured_center_connector(self):
        points=np.array([(x,y,-3.) for x in np.arange(0.,9.1,.2)
                        for y in np.arange(-min(3.,.1+x),min(3.,.1+x)+.01,.1)])
        index=PointIndex(points,np.zeros(3),road_height=5.)
        self.assertIsNotNone(ceiling_guidance(index,1.,1.05,.2,self.xy,self.base))
        # Removing the near support forbids extending the hull back to us.
        index=PointIndex(points[points[:,0]>=1.],np.zeros(3),road_height=5.)
        self.assertIsNone(ceiling_guidance(index,1.,1.05,.2,self.xy,self.base))


if __name__=='__main__':unittest.main()
