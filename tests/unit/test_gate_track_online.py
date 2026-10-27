from dataclasses import asdict
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace
import unittest

from estimation.history import HistoryUnavailable
from perception.gate_geometry.track_online import OnlineGateTracks
from tests.unit.test_gate_tracker import candidate, pose


class History:
    epoch=0
    def __init__(self, unavailable=False):self.unavailable=unavailable
    def lookup(self, stamp, now, wall, epoch):
        if self.unavailable:raise HistoryUnavailable('NO_BRACKET')
        return pose(stamp, epoch=epoch)


class GateTrackOnlineTests(unittest.TestCase):
    def pair(self, stamp=10.):return SimpleNamespace(metadata=candidate(stamp).metadata)
    def row(self, stamp=10.):return {'status':'GATE_CANDIDATES','timing_current':True,'candidates':[asdict(candidate(stamp))]}

    def test_causal_pose_and_candidate_produce_noncertified_track(self):
        pipeline=OnlineGateTracks(History())
        row=pipeline.process(self.row(),self.pair(),20.,lambda:(10.1,0,True),lambda:20.1)
        self.assertEqual(row['status'],'TRACKED');self.assertEqual(row['pose_stamp'],10.)
        self.assertEqual(row['tracks'][0]['observations'],1)
        self.assertFalse(row['tracks'][0]['control_eligible']);self.assertEqual(row['source_valid_until'],10.2)

    def test_missing_historical_pose_and_expired_source_are_rejected(self):
        pipeline=OnlineGateTracks(History(unavailable=True))
        self.assertEqual(pipeline.process(self.row(),self.pair(),20.,lambda:(10.1,0,True),lambda:20.1)['reason'],'POSE_NO_BRACKET')
        pipeline=OnlineGateTracks(History())
        self.assertEqual(pipeline.process(self.row(),self.pair(),20.,lambda:(10.2,0,True),lambda:20.1)['reason'],'INPUT_SOURCE_LEASE_NOT_CURRENT')
        self.assertEqual(pipeline.process({'status':'REJECTED'},self.pair(),20.,lambda:(10.1,0,True),lambda:20.1)['status'],'SOURCE_UNAVAILABLE')

    def test_output_expiry_after_association_suppresses_tracks(self):
        pipeline=OnlineGateTracks(History());clocks=iter([(10.1,0,True),(10.2,0,True)])
        row=pipeline.process(self.row(),self.pair(),20.,lambda:next(clocks),lambda:20.1)
        self.assertEqual(row['reason'],'OUTPUT_SOURCE_LEASE_NOT_CURRENT')
        self.assertNotIn('tracks',row)


    def test_audit_rejects_expired_track_even_when_candidate_is_current(self):
        from evaluation.metrics.visual_motion import audit
        pipeline=OnlineGateTracks(History())
        tracking=pipeline.process(self.row(),self.pair(),20.,lambda:(10.1,0,True),lambda:20.1)
        tracking['tracks'][0]['last_valid_until']=10.05
        row={'status':'GATE_CANDIDATES','timing_current':True,'timing_rejection':None,
             'source_stamp':10.,'source_valid_until':10.2,'source_epoch':0,'source_pair_stamp':10.,
             'source_arrival_wall':20.,'check_wall':20.1,'check_imu_stamp':10.1,'check_epoch':0,
             'clock_fresh_at_check':True,'control_eligible':False,'source_skew_s':0.,
             'candidates':self.row()['candidates'],'tracking':tracking}
        summary={'results':1,'statuses':{'GATE_CANDIDATES':1},'pipeline':'gate_track',
                 'tracking_statuses':{'TRACKED':1},'timing_current_results':1,'faults':[],
                 'control_published':False,'debug_inputs_used':False,
                 'node_graph':{'subscriptions':sorted(['/airsim_node/drone_1/front_left/Scene',
                   '/airsim_node/drone_1/front_right/Scene','/airsim_node/drone_1/imu/imu',
                   '/airsim_node/initial_pose','/airsim_node/drone_1/gps']),'publishers':['/rosout']}}
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder);(path/'visual_motion_summary.json').write_text(json.dumps(summary))
            (path/'visual_motion_results.jsonl').write_text(json.dumps(row)+'\n')
            result=audit(path)
        self.assertFalse(result['audit_valid']);self.assertIn('TRACK_TIMING_OR_IDENTITY',result['errors'])


if __name__ == '__main__':unittest.main()
