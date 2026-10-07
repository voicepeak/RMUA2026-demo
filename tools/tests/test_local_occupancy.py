"""Evidence and geometry tests: unknown space must never become implicit free."""
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros_ws/src/route_follower/scripts'))
from local_occupancy import LocalOccupancy,OccupancyConfig,owned_dynamic_returns,UNKNOWN,FREE,OCCUPIED


def local_map(**options):
    return LocalOccupancy(OccupancyConfig(resolution=1.,forward=8.,backward=1.,lateral=4.,
                                         vertical_min=-2.,vertical_max=2.,**options))


class OccupancyEvidenceTests(unittest.TestCase):
    def test_ray_prefix_free_hit_occupied_and_hidden_space_unknown(self):
        mapping=local_map();mapping.update([[4.1,.1,.1]],[.1,.1,.1],1.)
        snapshot=mapping.snapshot()
        self.assertEqual(snapshot.query([[.2,.1,.1],[2.1,.1,.1],[4.2,.1,.1],[5.1,.1,.1]]).tolist(),
                         [FREE,FREE,OCCUPIED,UNKNOWN])
        self.assertEqual(snapshot.query([[2.1,1.1,.1]]).item(),UNKNOWN)

    def test_sensor_origin_not_a_later_aircraft_position_controls_ray(self):
        mapping=local_map();mapping.update([[4.1,.1,.1]],[2.1,.1,.1],1.,center=[.1,.1,.1])
        snapshot=mapping.snapshot()
        self.assertEqual(snapshot.query([[1.1,.1,.1],[2.2,.1,.1],[3.1,.1,.1]]).tolist(),[UNKNOWN,FREE,FREE])

    def test_all_frame_hits_override_other_ray_free_evidence(self):
        mapping=local_map();mapping.update([[2.1,.1,.1],[4.1,.1,.1]],[.1,.1,.1],1.)
        self.assertEqual(mapping.snapshot().query([[1.1,.1,.1],[2.1,.1,.1],[3.1,.1,.1],[4.1,.1,.1]]).tolist(),
                         [FREE,OCCUPIED,UNKNOWN,OCCUPIED])

    def test_grid_corner_ray_does_not_clear_side_cells_touched_only_at_a_corner(self):
        mapping=local_map();mapping.update([[3.,3.,0.]],[0.,0.,0.],1.)
        self.assertEqual(mapping.snapshot().query([[.5,.5,0.],[1.5,1.5,0.],[2.5,2.5,0.],
                                                  [1.5,.5,0.],[.5,1.5,0.],[3.1,3.1,0.]]).tolist(),
                         [FREE,FREE,FREE,UNKNOWN,UNKNOWN,OCCUPIED])

    def test_negative_grid_boundary_and_axis_aligned_ray(self):
        mapping=local_map();mapping.update([[-3.,0.,0.]],[0.,0.,0.],1.,center=[-3.,0.,0.])
        self.assertEqual(mapping.snapshot().query([[-.5,0.,0.],[-1.5,0.,0.],[-3.,0.,0.]]).tolist(),
                         [FREE,FREE,OCCUPIED])

    def test_hit_beyond_local_window_clears_only_clipped_ray_inside_window(self):
        mapping=local_map();mapping.update([[30.,.1,.1]],[.1,.1,.1],1.)
        self.assertEqual(mapping.snapshot().query([[7.5,.1,.1],[8.2,.1,.1],[25.,.1,.1]]).tolist(),[FREE,UNKNOWN,UNKNOWN])

    def test_sensor_outside_map_is_clipped_without_filling_unobserved_sides(self):
        mapping=local_map();mapping.update([[6.1,.1,.1]],[-5.1,.1,.1],1.,center=[.1,.1,.1])
        self.assertEqual(mapping.snapshot().query([[.5,.1,.1],[6.1,.1,.1],[.5,1.1,.1]]).tolist(),[FREE,OCCUPIED,UNKNOWN])

    def test_ray_cap_retains_every_hit_even_when_some_rays_are_skipped(self):
        mapping=local_map(max_rays=1)
        info=mapping.update([[2.1,.1,.1],[4.1,1.1,.1],[6.1,2.1,.1]],[.1,.1,.1],1.)
        self.assertEqual(info['rays'],1);self.assertEqual(info['rays_skipped'],2)
        self.assertEqual(mapping.snapshot().query([[2.1,.1,.1],[4.1,1.1,.1],[6.1,2.1,.1]]).tolist(),[OCCUPIED]*3)

    def test_free_and_occupied_evidence_age_independently(self):
        mapping=local_map();mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.)
        mapping.update([[-.8,.1,.1]],[.1,.1,.1],1.4)
        self.assertEqual(mapping.snapshot().query([[2.1,.1,.1],[3.1,.1,.1]]).tolist(),[FREE,OCCUPIED])
        mapping.update([[-.8,.1,.1]],[.1,.1,.1],1.6)
        self.assertEqual(mapping.snapshot().query([[2.1,.1,.1],[3.1,.1,.1]]).tolist(),[UNKNOWN,OCCUPIED])
        mapping.update([[-.8,.1,.1]],[.1,.1,.1],2.1)
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]]).item(),UNKNOWN)

    def test_missing_empty_or_invalid_scan_never_authorizes_free_space(self):
        mapping=local_map();self.assertEqual(mapping.snapshot().query([[0.,0.,0.]]).item(),UNKNOWN)
        mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.)
        self.assertEqual(mapping.snapshot(now=1.6).query([[1.1,.1,.1]]).item(),UNKNOWN)
        mapping.update(np.empty((0,3)),[.1,.1,.1],1.1)
        self.assertFalse(mapping.snapshot().valid)
        self.assertEqual(mapping.snapshot().query([[1.1,.1,.1]]).item(),UNKNOWN)
        mapping.update([[np.nan,0.,0.],[.1,.1,.1]],[.1,.1,.1],1.2)
        self.assertFalse(mapping.snapshot().valid)

    def test_fresh_ray_clears_a_departed_hit_but_motion_label_alone_does_not(self):
        mapping=local_map();mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.)
        mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.1,dynamic_ids=np.array([1]))
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]],layer='static').item(),OCCUPIED)
        mapping.update([[5.1,.1,.1]],[.1,.1,.1],1.2)
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]],layer='static').item(),FREE)

    def test_dynamic_hit_is_separate_and_does_not_invent_underlying_static_free(self):
        mapping=local_map();mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.,dynamic_ids=np.array([7]))
        snapshot=mapping.snapshot()
        self.assertEqual(snapshot.query([[3.1,.1,.1]]).item(),OCCUPIED)
        self.assertEqual(snapshot.query([[3.1,.1,.1]],'static').item(),UNKNOWN)
        self.assertEqual(mapping.summary()['static_hit_voxels'],0)
        self.assertIn(7,snapshot.dynamic_owners)

    def test_mixed_voxel_static_return_survives_dynamic_ownership(self):
        mapping=local_map();mapping.update([[3.1,.1,.1],[3.2,.1,.1]],[.1,.1,.1],1.,dynamic_ids=np.array([7,0]))
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]],'static').item(),OCCUPIED)
        self.assertEqual(mapping.summary()['mixed_hit_voxels'],1)

    def test_dynamic_only_voxel_can_reuse_still_fresh_prior_free_evidence(self):
        mapping=local_map();mapping.update([[5.1,.1,.1]],[.1,.1,.1],1.)
        mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.1,dynamic_ids=np.array([7]))
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]],'static').item(),FREE)
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]]).item(),OCCUPIED)

    def test_multiple_dynamic_owners_in_one_voxel_are_explicit(self):
        mapping=local_map();mapping.update([[3.1,.1,.1],[3.2,.1,.1]],[.1,.1,.1],1.,dynamic_ids=np.array([7,8]))
        self.assertIn(-2,mapping.snapshot().dynamic_owners)
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]]).item(),OCCUPIED)

    def test_rolling_window_preserves_world_cells_and_evicts_departed_region(self):
        mapping=local_map();mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.)
        mapping.recenter([1.1,.1,.1],[1.,0.])
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]]).item(),OCCUPIED)
        mapping.recenter([20.1,.1,.1],[1.,0.]);mapping.recenter([.1,.1,.1],[1.,0.])
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]]).item(),UNKNOWN)
        self.assertLess(mapping.snapshot().states.size,1000)

    def test_rotation_changes_roi_without_rotating_world_evidence(self):
        mapping=local_map();mapping.update([[1.1,1.1,.1]],[.1,.1,.1],1.)
        mapping.recenter([.1,.1,.1],[0.,1.])
        self.assertEqual(mapping.snapshot().query([[1.1,1.1,.1]]).item(),OCCUPIED)
        self.assertEqual(mapping.snapshot().query([[7.,0.,.1]]).item(),UNKNOWN)

    def test_clock_and_source_epoch_reset_clear_old_evidence(self):
        mapping=local_map();mapping.update([[3.1,.1,.1]],[.1,.1,.1],2.,source_epoch=0)
        mapping.update([[-.8,.1,.1]],[.1,.1,.1],1.,source_epoch=0)
        self.assertEqual(mapping.epoch,1);self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]]).item(),UNKNOWN)
        mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.1,source_epoch=0)
        mapping.update([[-.8,.1,.1]],[.1,.1,.1],1.2,source_epoch=1)
        self.assertEqual(mapping.epoch,2);self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]]).item(),UNKNOWN)

    def test_snapshot_is_immutable_and_duplicate_frame_does_not_add_evidence(self):
        mapping=local_map();mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.)
        before=mapping.snapshot();version=mapping.version
        mapping.update([[5.1,.1,.1]],[.1,.1,.1],1.)
        self.assertEqual(mapping.version,version)
        mapping.update([[5.1,.1,.1]],[.1,.1,.1],1.1)
        self.assertEqual(before.query([[3.1,.1,.1]]).item(),OCCUPIED)
        self.assertEqual(mapping.snapshot().query([[3.1,.1,.1]]).item(),FREE)
        with self.assertRaises(ValueError):before.states.flat[0]=FREE

    def test_body_footprint_detects_obstacle_or_unknown_missed_by_center(self):
        mapping=local_map();mapping.update([[3.1,.1,.1]],[.1,.1,.1],1.)
        snapshot=mapping.snapshot()
        self.assertEqual(snapshot.query([[2.5,.1,.1]]).item(),FREE)
        self.assertEqual(snapshot.query_boxes([[2.1,.05,.05]],[[3.2,.15,.15]]).item(),OCCUPIED)
        self.assertEqual(snapshot.query_boxes([[1.1,.05,.05]],[[1.8,1.2,.15]]).item(),UNKNOWN)
        self.assertEqual(snapshot.query_boxes([[1.1,.05,.05]],[[1.8,.15,.15]]).item(),FREE)

    def test_invalid_geometry_parameters_and_queries_fail_closed(self):
        for parameters in ({'resolution':0.},{'vertical_min':3.},{'max_rays':1.5},
                           {'max_rays':2.},{'free_ttl':np.nan}):
            with self.assertRaises(ValueError):OccupancyConfig(**parameters)
        mapping=local_map(max_voxels=10)
        with self.assertRaises(ValueError):mapping.update([[1.,0.,0.]],[0.,0.,0.],1.)
        mapping=local_map()
        with self.assertRaises(ValueError):mapping.update([[1.,0.,0.]],[np.nan,0.,0.],1.)
        with self.assertRaises(ValueError):mapping.update([[1.,0.,0.]],[0.,0.,0.],1.,dynamic_ids=np.array([-1]))
        with self.assertRaises(ValueError):mapping.snapshot().query([[0.,0.,0.]],'missing')
        self.assertEqual(mapping.snapshot().query([[np.nan,0.,0.]]).item(),UNKNOWN)

    def test_invalid_reset_frame_cannot_erase_existing_evidence(self):
        mapping=local_map();mapping.update([[3.1,.1,.1]],[.1,.1,.1],2.,source_epoch=0)
        before=mapping.snapshot()
        with self.assertRaises(ValueError):
            mapping.update([[4.,0.,0.]],[0.,0.,0.],1.,forward=[0.,0.],source_epoch=1)
        after=mapping.snapshot()
        self.assertEqual((before.epoch,before.version,before.stamp),(after.epoch,after.version,after.stamp))
        np.testing.assert_array_equal(before.states,after.states)

    def test_random_rays_match_independent_segment_box_intersection_oracle(self):
        # A scalar slab-intersection oracle, independent of the incremental DDA.
        rng=np.random.default_rng(20261007)
        center=np.array([.1,.1,.1])
        for _ in range(12):
            mapping=LocalOccupancy(OccupancyConfig(resolution=1.,forward=4.,backward=1.,
                lateral=2.,vertical_min=-1.,vertical_max=1.))
            mapping.recenter(center,[1.,0.])
            origin=rng.uniform([-3.,-3.,-2.],[5.,3.,2.])
            points=rng.uniform([-2.,-3.,-2.],[7.,3.,2.],size=(8,3))
            snapshot=mapping.snapshot();cells=np.indices(snapshot.states.shape).reshape(3,-1).T
            lower=(cells+snapshot.cell_origin).astype(float);upper=lower+1.;middle=lower+.5
            active=np.all((middle-center>=[-1.,-2.,-1.])&(middle-center<[4.,2.,1.]),axis=1)
            inside=np.all((points-center>=[-1.,-2.,-1.])&(points-center<[4.,2.,1.]),axis=1)
            hit_cells={tuple(np.floor(p).astype(int)-snapshot.cell_origin) for p in points[inside]}
            expected=np.full(len(cells),UNKNOWN,dtype=np.uint8)
            for endpoint in points:
                direction=endpoint-origin;intersections=[]
                for index,(lo,hi) in enumerate(zip(lower,upper)):
                    enter=0.;leave=1.
                    for axis in range(3):
                        if abs(direction[axis])<1e-14:
                            if not lo[axis]<=origin[axis]<hi[axis]:leave=-1.;break
                        else:
                            a=(lo[axis]-origin[axis])/direction[axis]
                            b=(hi[axis]-origin[axis])/direction[axis]
                            enter=max(enter,min(a,b));leave=min(leave,max(a,b))
                    if leave-enter>1e-10:intersections.append((enter,index))
                for enter,index in sorted(intersections):
                    if tuple(cells[index]) in hit_cells:break
                    if active[index]:expected[index]=FREE
            for index,cell in enumerate(cells):
                if tuple(cell) in hit_cells:expected[index]=OCCUPIED
            mapping.update(points,origin,1.,center=center)
            np.testing.assert_array_equal(mapping.snapshot().states.ravel(),expected)


class DynamicOwnershipTests(unittest.TestCase):
    def track(self,**options):
        parameters=dict(point_indices=np.array([0,1]),timestamp=1.,observed_stamp=1.,track_id=7,
                        status='CONFIRMED',age=0.,motion_confirmed=True,truncated=False,
                        confidence=.8,velocity=np.array([1.,0.,0.]))
        parameters.update(options);return SimpleNamespace(**parameters)

    def test_only_credible_current_moving_returns_leave_static_layer(self):
        self.assertEqual(owned_dynamic_returns(3,[self.track()],1.).tolist(),[7,7,0])
        for options in ({'status':'TENTATIVE'},{'status':'COASTING','age':.1},
                        {'truncated':True},{'motion_confirmed':False},{'confidence':.1},
                        {'velocity':np.zeros(3)},{'velocity':np.array([.1,0.,0.])}):
            self.assertEqual(owned_dynamic_returns(3,[self.track(**options)],1.).tolist(),[0,0,0])

    def test_conflicting_and_outdated_ownership_cannot_erase_static_hits(self):
        a=self.track();b=self.track(track_id=8,point_indices=np.array([1,2]))
        self.assertEqual(owned_dynamic_returns(3,[a,b],1.).tolist(),[7,0,8])
        with self.assertRaises(ValueError):owned_dynamic_returns(3,[self.track(timestamp=.9)],1.)
        with self.assertRaises(ValueError):owned_dynamic_returns(3,[self.track(point_indices=np.array([3]))],1.)


if __name__=='__main__':unittest.main()
