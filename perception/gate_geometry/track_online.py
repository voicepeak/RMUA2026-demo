"""Link timely online gate proposals to historical pose and world tracks."""
from dataclasses import asdict

from estimation.history import HistoryUnavailable
from interfaces.contracts import Metadata
from perception.gate_geometry.stereo_gate import GateCandidate
from perception.stereo_depth.transport import freshness_reason
from world_model.gate_landmarks.tracker import GateTracker


class OnlineGateTracks:
    def __init__(self, history, tracker=None):
        self.history = history
        self.tracker = tracker or GateTracker(epoch=history.epoch)

    def process(self, row, pair, arrival, clock_snapshot, wall_clock):
        """Associate only an unexpired candidate using a causal history lookup."""
        if row['status'] != 'GATE_CANDIDATES' or not row['timing_current']:
            return {'status':'SOURCE_UNAVAILABLE','control_eligible':False}
        now, epoch, fresh = clock_snapshot(); wall = wall_clock()
        reason = freshness_reason(pair.metadata, now, epoch, fresh, arrival, wall)
        if reason:
            return {'status':'REJECTED','reason':'INPUT_'+reason,'control_eligible':False}
        try:
            pose = self.history.lookup(pair.metadata.stamp, now, wall, epoch)
        except HistoryUnavailable as error:
            return {'status':'REJECTED','reason':'POSE_'+str(error),'control_eligible':False}
        candidates=[]
        try:
            for data in row['candidates']:
                meta = dict(data['metadata']); meta['covariance_or_bounds']=tuple(meta['covariance_or_bounds'])
                candidates.append(GateCandidate(Metadata(**meta),tuple(tuple(x) for x in data['image_edges_px']),
                    tuple(data['center_body_frd_m']),data['nominal_width_m'],data['nominal_height_m'],
                    tuple(data['edge_disparities_px']),tuple(data['edge_support'])))
            result = self.tracker.ingest(tuple(candidates), pose, now, epoch)
            # Recheck time after pose lookup and association; do not publish a
            # snapshot based on a lease that elapsed while processing.
            check_now, check_epoch, check_fresh = clock_snapshot(); check_wall = wall_clock()
            reason = freshness_reason(pair.metadata, check_now, check_epoch, check_fresh, arrival, check_wall)
            if reason:
                return {'status':'REJECTED','reason':'OUTPUT_'+reason,'control_eligible':False}
            tracks=self.tracker.snapshot(check_now,check_epoch)
        except (ValueError,ArithmeticError) as error:
            return {'status':'REJECTED','reason':'TRACK_'+str(error),'control_eligible':False}
        return {'status':'TRACKED','association':result,'tracks':[asdict(track) for track in tracks],
                'pose_stamp':pose.stamp,'pose_bracket_stamps':pose.bracket_stamps,
                'source_stamp':pair.metadata.stamp,'source_valid_until':pair.metadata.valid_until,
                'source_epoch':pair.metadata.epoch,'source_arrival_wall':arrival,
                'check_imu_stamp':check_now,'check_epoch':check_epoch,'check_wall':check_wall,
                'clock_fresh_at_check':check_fresh,'control_eligible':False}
