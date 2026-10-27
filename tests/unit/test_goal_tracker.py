import unittest

from planning.mission.goal_tracker import GoalTracker


class GoalTrackerTests(unittest.TestCase):
    def test_zero_stamp_stability_change_and_no_claimed_completion(self):
        tracker = GoalTracker()
        self.assertEqual(tracker.observe((1.,2.,3.),0.,'',10.,100.,0).status,'PENDING')
        tracker.observe((1.01,2.,3.),0.,'',10.05,100.05,0)
        observed = tracker.observe((1.,2.,3.),0.,'',10.1,100.1,0)
        self.assertEqual(observed.status,'OBSERVED'); self.assertEqual(observed.position,(1.,2.,3.))
        self.assertEqual(observed.source_stamp,0.); self.assertFalse(observed.control_eligible)
        self.assertFalse(observed.official_stage_confirmed)
        self.assertEqual(tracker.observe((4.,2.,3.),0.,'',10.15,100.15,0).status,'PENDING')
        tracker.observe((4.,2.,3.),0.,'',10.2,100.2,0)
        self.assertEqual(tracker.observe((4.,2.,3.),0.,'',10.25,100.25,0).position,(4.,2.,3.))

    def test_stale_wall_clock_dead_imu_and_epoch_reset(self):
        t = GoalTracker()
        for i in range(3): t.observe((1.,2.,3.),0.,'',10.+i*.1,100.+i*.1,0)
        self.assertEqual(t.snapshot(10.7,0,True).status,'UNAVAILABLE')
        self.assertEqual(t.snapshot(10.25,0,False).status,'UNAVAILABLE')
        t.reset(1); self.assertEqual(t.snapshot(10.25,1,True).status,'UNAVAILABLE')
        with self.assertRaises(ValueError): t.observe((1.,2.,3.),0.,'',10.3,100.3,0)

    def test_stale_source_must_reconfirm(self):
        t = GoalTracker()
        for i in range(3): t.observe((1.,2.,3.),0.,'',10.+i*.1,100.+i*.1,0)
        self.assertEqual(t.observe((1.,2.,3.),0.,'',11.,101.,0).status,'PENDING')
        self.assertEqual(t.observe((1.,2.,3.),0.,'new_frame',11.1,101.1,0).observation_count,1)

    def test_stamped_duplicates_and_rollbacks_cannot_reconfirm(self):
        t = GoalTracker()
        t.observe((1.,2.,3.),100.,'',10.,100.,0)
        for stamp in (100., 99.99):
            with self.assertRaises(ValueError): t.observe((1.,2.,3.),stamp,'',10.1,100.1,0)
        self.assertEqual(t.snapshot(10.1,0,True).observation_count,1)
        self.assertEqual(t.observe((1.,2.,3.),100.1,'',10.2,100.1,0).observation_count,2)

    def test_reject_invalid_inputs_and_no_duplicate_wall_refresh(self):
        t = GoalTracker(); t.observe((1.,2.,3.),0.,'',10.,100.,0)
        for position,stamp,arrival in [((float('nan'),2.,3.),0.,10.1), ((1.,2.,3.),100.2,10.1),
                                       ((1.,2.,3.),99.,10.1), ((1.,2.,3.),0.,10.)]:
            with self.assertRaises(ValueError): t.observe(position,stamp,'',arrival,100.,0)
        self.assertEqual(t.snapshot(10.5,0,True).status,'UNAVAILABLE')


if __name__ == '__main__': unittest.main()
