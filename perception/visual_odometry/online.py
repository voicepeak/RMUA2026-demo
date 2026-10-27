"""Causal, bounded visual-odometry window for an isolated online observer."""
from dataclasses import asdict

from perception.stereo_depth.transport import freshness_reason
from perception.visual_odometry.stereo import estimate_stereo_motion


class MotionWindow:
    def __init__(self, rig, method='orb_lk', estimator=estimate_stereo_motion):
        if method not in ('orb', 'orb_lk'):
            raise ValueError('unknown motion method')
        self.rig, self.method, self.estimator = rig, method, estimator
        self.previous = None

    def process(self, pair, source_arrival, clock_snapshot, wall_clock):
        """Consume one pair; previous pair is historical and never gets a new lease.

        clock_snapshot() returns (sim_time, epoch, fresh), using an independent
        IMU clock. A returned estimate is diagnostic even if timing is current.
        """
        now, epoch, healthy = clock_snapshot()
        reason = freshness_reason(pair.metadata, now, epoch, healthy, source_arrival, wall_clock())
        if reason:
            self.previous = None
            return {'status': 'REJECTED', 'reason': 'INPUT_' + reason, 'control_eligible': False}
        previous, self.previous = self.previous, pair
        if previous is None or previous.metadata.epoch != pair.metadata.epoch:
            return {'status': 'WARMUP', 'control_eligible': False}
        try:
            motion = self.estimator(previous, pair, self.rig, method=self.method)
        except (ValueError, ArithmeticError) as error:
            return {'status': 'REJECTED', 'reason': 'MOTION_' + str(error), 'control_eligible': False}
        now, epoch, healthy = clock_snapshot()
        checked_wall = wall_clock()
        reason = freshness_reason(pair.metadata, now, epoch, healthy, source_arrival, checked_wall)
        # A stale current frame must not serve as an anchor for the next estimate.
        if reason:
            self.previous = None
        return {'status': 'ESTIMATED', 'motion': asdict(motion), 'timing_current': reason is None,
                'timing_rejection': reason, 'check_imu_stamp': now, 'check_epoch': epoch,
                'source_stamp': pair.metadata.stamp, 'source_valid_until': pair.metadata.valid_until,
                'source_arrival_wall': source_arrival, 'check_wall': checked_wall,
                'clock_fresh_at_check': healthy, 'control_eligible': False}
