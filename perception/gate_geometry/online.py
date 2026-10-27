"""Read-only gate candidate pipeline with source-bounded output timing."""
from dataclasses import asdict

from perception.gate_geometry.roi_disparity import gate_region_disparity
from perception.gate_geometry.stereo_gate import stereo_gate_candidates
from perception.stereo_depth.transport import freshness_reason


class GateWindow:
    def __init__(self, rig, depth_fn=gate_region_disparity, candidates_fn=stereo_gate_candidates):
        self.rig, self.depth_fn, self.candidates_fn = rig, depth_fn, candidates_fn

    def process(self, pair, source_arrival, clock_snapshot, wall_clock):
        def check():
            now, epoch, fresh = clock_snapshot(); wall = wall_clock()
            return now, epoch, fresh, wall, freshness_reason(pair.metadata, now, epoch, fresh, source_arrival, wall)
        now, epoch, fresh, wall, reason = check()
        if reason:
            return {'status':'REJECTED','reason':'INPUT_'+reason,'control_eligible':False}
        try:
            depth = self.depth_fn(pair)
            now, epoch, fresh, wall, reason = check()
            if reason:
                return {'status':'REJECTED','reason':'AFTER_DEPTH_'+reason,'control_eligible':False}
            candidates = self.candidates_fn(pair, depth, self.rig, now, epoch)
        except (ValueError, ArithmeticError) as error:
            return {'status':'REJECTED','reason':'GATE_'+str(error),'control_eligible':False}
        now, epoch, fresh, wall, reason = check()
        return {'status':'GATE_CANDIDATES','candidates':[asdict(candidate) for candidate in candidates],
                'timing_current':reason is None,'timing_rejection':reason,'check_imu_stamp':now,
                'check_epoch':epoch,'source_stamp':pair.metadata.stamp,
                'source_valid_until':pair.metadata.valid_until,'source_arrival_wall':source_arrival,
                'check_wall':wall,'clock_fresh_at_check':fresh,'control_eligible':False}
