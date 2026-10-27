"""Read-only LiDAR cluster, tracking and dynamic-veto diagnostic pipeline."""
from dataclasses import asdict
import math

from estimation.history import HistoryUnavailable
from interfaces.contracts import Health, Metadata, Trajectory, TrajectorySample
from perception.object_motion.lidar_clusters import cluster_endpoints
from perception.stereo_depth.transport import freshness_reason
from planning.spacetime_local.static_candidates import LocalPlannerLimits, plan_static_local
from planning.trajectory_validator.dynamic_candidates import DynamicBounds, DynamicScene, check_dynamic_candidates
from planning.trajectory_validator.static_sweep import SweepBounds
from world_model.dynamic_tracks.tracker import DynamicTracker
from world_model.occupancy.rolling_endpoints import RollingEndpointMap, world_endpoints


class OnlineDynamicLidar:
    def __init__(self, history, sensor_pose, planning_diagnostic=False):
        self.history=history;self.sensor_pose=sensor_pose;self.tracker=DynamicTracker(epoch=history.epoch)
        # Explicit uncalibrated assumptions for a stationary diagnostic probe.
        self.body=SweepBounds((.2,.2,.1),.05,0.)
        self.bounds=DynamicBounds(.2,2.,4.,20.,max_horizon_s=.3)
        self.map=RollingEndpointMap(epoch=history.epoch) if planning_diagnostic else None
        self.plan_limits=LocalPlannerLimits(SweepBounds((.2,.2,.1),.05,2.),1.,
                         segment_durations_s=(.05,.1),lateral_offsets_m=())

    def process(self,cloud,arrival,snapshot,wall_clock,goal_snapshot=None):
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
            planner = None
            if self.map is not None:
                if epoch > self.map.epoch:self.map.reset(epoch)
                if not self.map.integrate(world,now,epoch):raise ValueError('duplicate map scan')
                grid=self.map.snapshot(now,epoch)
                goal=goal_snapshot() if goal_snapshot is not None else None
                if goal is None or goal.status!='OBSERVED' or goal.epoch!=epoch:
                    planner={'reason':'GOAL_UNAVAILABLE','map_cells':len(grid.cells)}
                else:
                    delta=tuple(b-a for a,b in zip(state.position_ned,goal.position))
                    length=math.hypot(*delta);factor=min(1.,1./length) if length else 0.
                    target=tuple(a+factor*d for a,d in zip(state.position_ned,delta))
                    initial=TrajectorySample(now,state.position_ned,state.velocity_ned,(0.,0.,0.),0.)
                    plan=plan_static_local(state,initial,target,grid,now,epoch,self.plan_limits,
                                           DynamicScene(clusters.metadata,tracks),self.bounds)
                    planner={'reason':plan.reason,'attempts':[asdict(c) for c in plan.checks],
                             'map_cells':len(grid.cells),'goal':asdict(goal),'target':target,
                             'trajectory_produced':plan.trajectory is not None,
                             'goal_frame_calibrated':False,'control_eligible':False}
            check_now,check_epoch,check_fresh,_,check_wall,reason=check()
            if reason:return {'status':'REJECTED','reason':'OUTPUT_'+reason,'control_eligible':False}
            if planner and 'goal' in planner and (planner['goal']['epoch'] != check_epoch
                    or not 0 <= check_wall-planner['goal']['last_arrival_wall'] < .5):
                return {'status':'REJECTED','reason':'OUTPUT_GOAL_STALE','control_eligible':False}
            if not state.metadata.current(check_now,check_epoch) or any(
                    t.epoch != check_epoch or not t.stamp <= check_now < t.valid_until for t in tracks):
                return {'status':'REJECTED','reason':'OUTPUT_STATE_OR_TRACK_EXPIRED','control_eligible':False}
            return {'status':'PROCESSED','source_stamp':cloud.metadata.stamp,'source_valid_until':cloud.metadata.valid_until,
                    'source_epoch':cloud.metadata.epoch,'source_arrival_wall':arrival,'check_imu_stamp':check_now,
                    'check_epoch':check_epoch,'check_wall':check_wall,'clock_fresh_at_check':check_fresh,
                    'pose_stamp':pose.stamp,'pose_bracket_stamps':pose.bracket_stamps,
                    'occupied_voxels':clusters.occupied_voxels,'cluster_count':len(clusters.clusters),
                    'rejected_components':clusters.rejected_components,'association':association,
                    'tracks':[asdict(track) for track in tracks],'dynamic_check':asdict(dynamic), 'planner':planner,
                    'control_eligible':False}
        except (ValueError,ArithmeticError) as error:
            return {'status':'REJECTED','reason':'PIPELINE_'+str(error),'control_eligible':False}
