import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from persistent_guides import PersistentGuides
from reference_planner import RouteGeometry, ReferencePlanner

class PersistentGuidesTests(unittest.TestCase):
    def observation(self,s,z,tid=1):
        return dict(id=tid,x=s,y=0.,z=z,last_seen=10.,support=8,sigma_z=.5,confidence=.8)

    def test_dropout_and_hard_promotion_preserve_height_trail(self):
        c=PersistentGuides();g=self.observation(400.,-50.)
        c.update([g],10.,350.,lambda g:g['x'])
        self.assertFalse(c.update([],30.,410.,lambda g:g['x']))
        self.assertEqual(c.guides,[(400.,-50.)])
        g.update(hard_anchor=True,last_seen=30.,z=-40.)
        self.assertFalse(c.update([g],30.,410.,lambda g:g['x']))
        self.assertEqual(c.guides,[(400.,-50.)])

    def test_duplicate_tracks_merge_spatially(self):
        c=PersistentGuides()
        c.update([self.observation(400,-50),self.observation(402,-51,2)],10.,350.,lambda g:g['x'])
        self.assertEqual(c.guides,[(401.,-50.5)])

    def test_dropout_does_not_revert_uphill_profile_to_old_hard_gate(self):
        c=PersistentGuides();r=RouteGeometry([(0,0,0),(1000,0,0)])
        planner=ReferencePlanner(r)
        gates=[dict(id=1,x=320.,y=0.,z=-30.,valid=True),dict(id=2,x=340.,y=0.,z=-34.,valid=True)]
        c.update([self.observation(380.,-45.),self.observation(420.,-60.,2)],10.,350.,lambda g:g['x'])
        _,before=planner.build(gates,[],c.guides,370.,0.)
        c.update([],30.,400.,lambda g:g['x'])
        _,after=planner.build(gates,[],c.guides,400.,0.)
        self.assertAlmostEqual(before.center(410.),after.center(410.))
        self.assertLess(after.dz_ds(410.),0.)

    def test_supported_gate_beyond_sixty_metre_gap_keeps_climb(self):
        r=RouteGeometry([(0,0,0),(1000,0,0)])
        planner=ReferencePlanner(r)
        gates=[dict(id=1,x=414.,y=0.,z=-55.,valid=True),dict(id=2,x=434.,y=0.,z=-61.,valid=True)]
        _,profile=planner.build(gates,[],[(454.,-68.5),(514.7,-94.1)],495.,0.)
        self.assertAlmostEqual(profile.center(514.7),-94.1)
        self.assertLess(profile.dz_ds(505.),-.2)

if __name__=='__main__': unittest.main()
