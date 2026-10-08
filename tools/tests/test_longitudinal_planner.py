import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros_ws/src/route_follower/scripts'))
from longitudinal_planner import (LongitudinalConfig, emergency_stop_distance, car_box_conflict,
                                  plan_longitudinal, choose_one_shot,
                                  commitment_should_switch, CarVelocityStore,
                                  project_cars)


class LongitudinalPlannerTests(unittest.TestCase):
    def setUp(self):
        self.cfg = LongitudinalConfig()

    def rollout(self, car, v0, cfg=None, seconds=8.):
        cfg = cfg or self.cfg
        s = 0.; v = v0; car_s = float(car['s']); trace = []
        for _ in range(int(seconds/cfg.dt)):
            moving = dict(car, s=car_s)
            out = plan_longitudinal(s, v, [moving], cfg, cruise=12., free_soft=None)
            v2 = float(out['target_speed'])
            s = s+.5*(v+v2)*cfg.dt
            v = v2
            car_s = car_s+float(car['v_s'])*cfg.dt
            trace.append((s, v, out, car_s))
        return trace

    def simulate(self, profile, v0, dt=.25):
        s = 0.; v = v0; trace = []
        for v2 in profile:
            s += .5*(v+v2)*dt
            v = v2
            trace.append((s, v))
        return trace

    def test_static_car_ahead_decelerates_before_follow_gap(self):
        car = dict(s=20., y=0., v_s=0., half_s=2., half_y=.8, uncertainty=.3, id=1)
        trace = self.rollout(car, 8.)
        stop_s, stop_v, _, _ = trace[-1]
        self.assertLess(stop_v, .5)
        limit = 20.-(2.+.3+self.cfg.follow_gap)
        self.assertLessEqual(stop_s, limit+.25)
        for s, v, out, car_s in trace:
            self.assertLessEqual(s, limit+.25)

    def test_stationary_car_ten_meters_waits_or_crawls(self):
        car = dict(s=14., y=0., v_s=0., half_s=2., half_y=.8, uncertainty=.3, id=1)
        trace = self.rollout(car, 10.)
        stop_s, stop_v, _, _ = trace[-1]
        self.assertLess(stop_v, .5)
        self.assertLessEqual(stop_s, 14.-(2.+.3+self.cfg.follow_gap)+.25)

    def test_no_stop_distance_brakes_as_hard_as_allowed(self):
        car = dict(s=10., y=0., v_s=0., half_s=2., half_y=.8, uncertainty=.3, id=1)
        out = plan_longitudinal(0., 10., [car], self.cfg, cruise=10., free_soft=None)
        self.assertLessEqual(out['target_speed'], 9.)
        trace = self.rollout(car, 10.)
        self.assertLessEqual(trace[-1][0], 10.-2.-self.cfg.absolute_margin)

    def test_crossing_car_in_another_lane_is_ignored(self):
        car = dict(s=30., y=3., v_s=0., half_s=2., half_y=.8, uncertainty=.3, id=1)
        out = plan_longitudinal(0., 8., [car], self.cfg, cruise=8., free_soft=None)
        self.assertGreater(out['target_speed'], 7.)

    def test_moving_car_ahead_allows_follow_at_its_speed(self):
        car = dict(s=12., y=0., v_s=2., half_s=2., half_y=.8, uncertainty=.3, id=1)
        trace = self.rollout(car, 8., seconds=4.)
        self.assertGreater(trace[-1][1], 1.5)
        for s, v, out, car_s in trace:
            self.assertGreaterEqual(car_s-s, 2.+.3+self.cfg.follow_gap-.6)

    def test_static_soft_stop_limits_progress(self):
        s = 0.; v = 8.; trace = []
        for _ in range(32):
            out = plan_longitudinal(s, v, [], self.cfg, cruise=8., free_soft=5.-s)
            v2 = float(out['target_speed'])
            s = s+.5*(v+v2)*self.cfg.dt
            v = v2
            trace.append((s, v))
        self.assertLess(v, .5)
        self.assertLessEqual(s, 5.+1e-6)

    def test_emergency_stop_distance_formula(self):
        distance = emergency_stop_distance(12., self.cfg)
        self.assertAlmostEqual(distance, 12.*.25+12.*12/12.+.45, places=6)
        self.assertAlmostEqual(emergency_stop_distance(0., self.cfg), .45, places=6)

    def test_one_shot_prefers_larger_free_with_smaller_offset(self):
        current = 2.
        offsets = {-1.5: (7., 7., 1.), -1.: (9., 9., 1.), 1.: (9., 9., 1.)}
        pick = choose_one_shot(current, offsets, self.cfg)
        self.assertEqual(pick['kind'], 'lateral')
        self.assertEqual(pick['y'], -1.)
        self.assertEqual(choose_one_shot(2., {-1.: (3., 3., 1.)}, self.cfg)['kind'], None)
        self.assertEqual(choose_one_shot(2., {}, self.cfg, vertical_ok=True)['kind'],
                         'vertical')

    def test_commitment_keeps_old_decision_until_clear_improvement(self):
        old = dict(cost=10.)
        self.assertFalse(commitment_should_switch(old, dict(cost=9.), 1., 2., self.cfg))
        self.assertTrue(commitment_should_switch(old, dict(cost=8.), 1., 2., self.cfg))
        self.assertTrue(commitment_should_switch(old, dict(cost=9.), 3., 2., self.cfg))
        self.assertTrue(commitment_should_switch(old, dict(cost=9., unsafe=True),
                                                  1., 2., self.cfg))

    def test_car_velocity_store_measures_and_smooths(self):
        store = CarVelocityStore(smoothing=1.)
        track = dict(world=np.array([0., 0., 0.]))
        self.assertIsNone(store.velocity(track, 0.))
        track = dict(world=np.array([1., 0., 0.]))
        measured = store.velocity(track, .5)
        self.assertIsNotNone(measured)
        self.assertAlmostEqual(measured[0], 2., places=6)

    def test_car_box_conflict_detects_unseen_roof_on_flight_plane(self):
        cfg = self.cfg
        car = dict(s=8., y=0., z=-1., v_s=0., half_s=2., half_y=.8, half_z=.7,
                   uncertainty=.3, id=3)
        # Aircraft at the car's altitude and within the stop distance: blocked.
        self.assertIsNotNone(car_box_conflict([car], 0., 0., -1., 7., cfg))
        # Above the roof by more than the hard margin: ignored.
        self.assertIsNone(car_box_conflict([car], 0., 0., -2.5, 7., cfg))
        # Different lane: ignored.
        self.assertIsNone(car_box_conflict([car], 0., 3., -1., 7., cfg))
        # Behind the aircraft: ignored.
        self.assertIsNone(car_box_conflict([car], 20., 0., -1., 5., cfg))

    def test_project_cars_route_coordinates(self):
        cfg = self.cfg
        tracks = [dict(world=np.array([10., 1., 0.]), stamp=1., half_width=.8,
                       half_height=.6, uncertainty=.3, id=7)]
        cars = project_cars(tracks, project_xy=lambda x, y: float(x),
                            center_at=lambda s: (s, 0., 0.),
                            tangent_at=lambda s: (1., 0., 0.),
                            now=1., cfg=cfg, state_store=CarVelocityStore())
        self.assertEqual(len(cars), 1)
        self.assertAlmostEqual(cars[0]['s'], 10., places=6)
        self.assertAlmostEqual(cars[0]['y'], 1., places=6)
        self.assertAlmostEqual(cars[0]['half_y'], .8, places=6)


if __name__ == '__main__':
    unittest.main()
