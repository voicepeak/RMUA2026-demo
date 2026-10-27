"""Read-only LiDAR cluster, tracking and dynamic-veto diagnostic pipeline."""
from dataclasses import asdict

from estimation.history import HistoryUnavailable
from interfaces.contracts import Health, Metadata, Trajectory, TrajectorySample
from perception.object_motion.lidar_clusters import cluster_endpoints
from perception.stereo_depth.transport import freshness_reason
from planning.trajectory_validator.dynamic_candidates import DynamicBounds, DynamicScene, check_dynamic_candidates
from planning.trajectory_validator.static_sweep import SweepBounds
from world_model.dynamic_tracks.tracker import DynamicTracker
from world_model.occupancy.rolling_endpoints import world_endpoints


class OnlineDynamicLidar:
    def __init__(self, history, sensor_pose):
        self.history=history;self.sensor_pose=sensor_pose;self.tracker=DynamicTracker(epoch=history.epoch)
        # Explicit uncalibrated assumptions for a stationary diagnostic probe.
        self.body=SweepBounds((.2,.2,.1),.05,0.)
        self.bounds=DynamicBounds(.2,2.,4.,20.,max_horizon_s=.3)

    def process(self,cloud,arrival,snapshot,wall_clock):
        """Snapshot returns (imu stamp, epoch, fresh, latest state)."""
        def check():
            now,epoch,fresh,state=snapshot();wall=wall_clock()
            return now,epoch,fresh,state,wall,freshness_reason(cloud.metadata,now,epoch,fresh,arrival,wall)
        now,epoch,fresh,state,wall,reason=check()
        if reason:return {'status':'REJECTED','reason':'INPUT_'+reason,'control_eligible':False,
                          'source_stamp':cloud.metadata.stamp,'source_valid_until':cloud.metadata.valid_until,
                          'check_imu_stamp':now,'check_epoch':epoch,'source_epoch':cloud.metadata.epoch,
                          'source_arrival_wall':arrival,'check_wall':wall}
        try:
            pose=self.history.lookup(cloud.metadata.stamp,now,wall,epoch)
        except HistoryUnavailable as error:
            return {'status':'REJECTED','reason':'POSE_'+str(error),'control_eligible':False}
        try:
            world=world_endpoints(cloud,pose,self.sensor_pose)
            clusters=cluster_endpoints(world,now,epoch)
            now,epoch,fresh,state,wall,reason=check()
            if reason:return {'status':'REJECTED','reason':'AFTER_CLUSTER_'+reason,'control_eligible':False}
            if epoch > self.tracker.epoch:self.tracker.reset(epoch)
            association=self.tracker.ingest(clusters,now,epoch)
            tracks=self.tracker.snapshot(now,epoch)
            if state is None or state.health!=Health.OK or not state.metadata.current(now,epoch):
                return {'status':'REJECTED','reason':'STATE_UNAVAILABLE','control_eligible':False}
            # This is a held-position geometry probe, not a command or claim
            # that the vehicle can stop here. No candidate establishes FREE.
            sample=lambda stamp:TrajectorySample(stamp,state.position_ned,(0.,0.,0.),(0.,0.,0.),0.)
            trajectory=Trajectory(Metadata(now,now+.2,epoch,'stationary_dynamic_probe'),(sample(now),sample(now+.2)))
            dynamic=check_dynamic_candidates(trajectory,DynamicScene(clusters.metadata,tracks),now,epoch,self.body,self.bounds)
            check_now,check_epoch,check_fresh,_,check_wall,reason=check()
            if reason:return {'status':'REJECTED','reason':'OUTPUT_'+reason,'control_eligible':False}
            if not state.metadata.current(check_now,check_epoch) or any(
                    t.epoch != check_epoch or not t.stamp <= check_now < t.valid_until for t in tracks):
                return {'status':'REJECTED','reason':'OUTPUT_STATE_OR_TRACK_EXPIRED','control_eligible':False}
            return {'status':'PROCESSED','source_stamp':cloud.metadata.stamp,'source_valid_until':cloud.metadata.valid_until,
                    'source_epoch':cloud.metadata.epoch,'source_arrival_wall':arrival,'check_imu_stamp':check_now,
                    'check_epoch':check_epoch,'check_wall':check_wall,'clock_fresh_at_check':check_fresh,
                    'pose_stamp':pose.stamp,'pose_bracket_stamps':pose.bracket_stamps,
                    'occupied_voxels':clusters.occupied_voxels,'cluster_count':len(clusters.clusters),
                    'rejected_components':clusters.rejected_components,'association':association,
                    'tracks':[asdict(track) for track in tracks],'dynamic_check':asdict(dynamic),
                    'control_eligible':False}
        except (ValueError,ArithmeticError) as error:
            return {'status':'REJECTED','reason':'PIPELINE_'+str(error),'control_eligible':False}
