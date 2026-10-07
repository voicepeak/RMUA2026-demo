#!/usr/bin/env python3
"""One frozen collision definition for planner and future execution guard.

Every response scenario is checked. Whole swept voxel boxes reject unknown;
moving CV boxes use relative-motion segment intersection, including between
samples. Bounded inter-sample curvature uses the configured acceleration limit.
These checks certify the empirical/discretized model, not physical flight.
"""
from dataclasses import dataclass
import numpy as np
from local_occupancy import UNKNOWN,FREE,OCCUPIED,_inside
from trajectory_types import CollisionResult


@dataclass(frozen=True)
class CollisionConfig:
    body_half_extent: tuple = (.25,.25,.25)
    margin: float = 1.
    max_horizontal_speed: float = 12.
    max_vertical_speed: float = 4.5
    max_acceleration: tuple = (12.,12.,50.)
    max_cloud_age: float = .5
    max_track_age: float = .75

    def __post_init__(self):
        object.__setattr__(self,'body_half_extent',tuple(map(float,self.body_half_extent)))
        object.__setattr__(self,'max_acceleration',tuple(map(float,self.max_acceleration)))
        if len(self.body_half_extent)!=3 or len(self.max_acceleration)!=3:raise ValueError('Collision bounds must be XYZ')
        values=(*self.body_half_extent,self.margin,self.max_horizontal_speed,self.max_vertical_speed,
                *self.max_acceleration,self.max_cloud_age,self.max_track_age)
        if not np.all(np.isfinite(values)) or any(v<0 for v in values):raise ValueError('Invalid collision limits')
        if any(v<=0 for v in (self.max_horizontal_speed,self.max_vertical_speed,*self.max_acceleration,self.max_cloud_age,self.max_track_age)):
            raise ValueError('Collision limits must be positive')


class VoxelBoxIndex:
    """Summed-volume queries exactly match OccupancySnapshot.query_boxes."""
    def __init__(self,snapshot):
        self.snapshot=snapshot
        prefix=lambda mask:np.pad(mask.astype(np.int64),((1,0),(1,0),(1,0))).cumsum(0).cumsum(1).cumsum(2)
        self.occupied=prefix(snapshot.static_states==OCCUPIED)
        self.not_free=prefix(snapshot.static_states!=FREE)

    @staticmethod
    def _counts(prefix,low,high):
        total=np.zeros(len(low),dtype=np.int64)
        for x in (0,1):
            for y in (0,1):
                for z in (0,1):
                    corner=np.column_stack([high[:,0] if x else low[:,0],high[:,1] if y else low[:,1],high[:,2] if z else low[:,2]])
                    total+=(-1 if (x+y+z)%2==0 else 1)*prefix[tuple(corner.T)]
        return total

    def query(self,lows,highs):
        snapshot=self.snapshot
        result=np.full(len(lows),UNKNOWN,dtype=np.uint8)
        if not snapshot.valid:return result
        first=np.floor(lows/snapshot.config.resolution).astype(np.int64)-snapshot.cell_origin
        last=np.floor(highs/snapshot.config.resolution).astype(np.int64)-snapshot.cell_origin
        shape=np.asarray(snapshot.static_states.shape)
        overlap=np.all(last>=0,axis=1)&np.all(first<shape,axis=1)
        ids=np.flatnonzero(overlap)
        if not len(ids):return result
        a=np.maximum(first[ids],0);b=np.minimum(last[ids]+1,shape)
        occupied=self._counts(self.occupied,a,b)>0
        unknown=self._counts(self.not_free,a,b)>0
        corners=np.array([[x,y,z] for x in (0,1) for y in (0,1) for z in (0,1)])
        inside=np.all(_inside(lows[ids,None,:]+corners*(highs-lows)[ids,None,:],
                              snapshot.center,snapshot.forward,snapshot.config),axis=1)
        complete=np.all(first[ids]>=0,axis=1)&np.all(last[ids]<shape,axis=1)&inside
        result[ids[~unknown&complete]]=FREE;result[ids[occupied]]=OCCUPIED
        return result


class TrajectoryCollision:
    def __init__(self,snapshot,config=None,now=None):
        self.snapshot=snapshot;self.config=CollisionConfig() if config is None else config
        self.now=(snapshot.pose_stamp if snapshot.evaluation_stamp is None else snapshot.evaluation_stamp) if now is None else float(now)
        if not np.isfinite(self.now):raise ValueError('Invalid collision evaluation time')
        self.index=VoxelBoxIndex(snapshot.occupancy)

    def freshness(self):
        snapshot=self.snapshot;mapping=snapshot.occupancy
        if self.now<snapshot.pose_stamp-1e-8:return CollisionResult(False,'CLOCK_MISMATCH')
        if mapping.stamp is None or not mapping.valid:return CollisionResult(False,'SENSOR_STALE')
        if not np.all(np.isfinite([mapping.stamp,mapping.evaluated_at])):return CollisionResult(False,'SCENE_INVALID')
        if (mapping.stamp>snapshot.pose_stamp+1e-8 or self.now-mapping.stamp>=min(self.config.max_cloud_age,mapping.config.free_ttl)):
            return CollisionResult(False,'SENSOR_STALE')
        # Individual evidence TTLs must be refreshed by the map owner at now.
        if abs(mapping.evaluated_at-self.now)>1e-8 or snapshot.scene_version!=mapping.version:
            return CollisionResult(False,'SCENE_TIME_MISMATCH')
        ids=set()
        for obstacle in snapshot.obstacles:
            if (not np.all(np.isfinite(np.r_[obstacle.timestamp,obstacle.observed_stamp,obstacle.age,
                    obstacle.position,obstacle.velocity,obstacle.bbox_size,obstacle.covariance.ravel()])) or
                    obstacle.position.shape!=(3,) or obstacle.velocity.shape!=(3,) or obstacle.bbox_size.shape!=(3,) or
                    obstacle.covariance.shape!=(6,6) or np.any(obstacle.bbox_size<0.) or
                    obstacle.observed_stamp>obstacle.timestamp+1e-8 or obstacle.age<0.):
                return CollisionResult(False,'TRACK_INVALID',track_id=obstacle.track_id)
            if (not np.allclose(obstacle.covariance,obstacle.covariance.T,rtol=0.,atol=1e-8) or
                    np.min(np.linalg.eigvalsh(obstacle.covariance))<-1e-8):
                return CollisionResult(False,'TRACK_INVALID',track_id=obstacle.track_id)
            if obstacle.timestamp>snapshot.pose_stamp+1e-8:return CollisionResult(False,'TRACK_TIME_MISMATCH',track_id=obstacle.track_id)
            age_limit=min(self.config.max_track_age,obstacle.config.max_coast)
            if self.now-obstacle.observed_stamp>age_limit+1e-8:
                return CollisionResult(False,'TRACK_STALE',track_id=obstacle.track_id)
            if obstacle.track_id in ids:return CollisionResult(False,'DUPLICATE_TRACK',track_id=obstacle.track_id)
            ids.add(obstacle.track_id)
        owners=np.unique(mapping.dynamic_owners)
        if -2 in owners or any(owner>0 and owner not in ids for owner in owners):
            return CollisionResult(False,'MISSING_TRACK')
        return CollisionResult(True,'PASS')

    def check(self,trace):
        fresh=self.freshness()
        if not fresh.safe:return fresh
        if trace.end_state.model_key!=self.snapshot.response_state.model_key:return CollisionResult(False,'MODEL_MISMATCH')
        if trace.times[0]<self.snapshot.pose_stamp-1e-8:return CollisionResult(False,'TRAJECTORY_TIME_MISMATCH')
        count=len(trace.end_state.position)
        if count!=len(self.snapshot.response_state.position):return CollisionResult(False,'SCENARIO_MISMATCH')
        if trace.acceleration_bounds is None:return CollisionResult(False,'DYNAMICS_UNVERIFIED')
        p0=trace.positions[:-1].reshape(-1,3);p1=trace.positions[1:].reshape(-1,3)
        v0=trace.velocities[:-1].reshape(-1,3);v1=trace.velocities[1:].reshape(-1,3)
        t0=np.repeat(trace.times[:-1],count);t1=np.repeat(trace.times[1:],count);dt=t1-t0
        scenario=np.tile(np.arange(count),len(trace.times)-1)
        acceleration=trace.acceleration_bounds.reshape(-1,3)
        curvature=dt[:,None]**2*acceleration/8.
        body=np.asarray(self.config.body_half_extent);inflation=body+self.config.margin+curvature
        low=np.minimum(p0,p1)-inflation;high=np.maximum(p0,p1)+inflation
        conflicts=[]

        def report(mask,reason,stamps=None,track_id=None,separation=None):
            ids=np.flatnonzero(mask)
            if not len(ids):return
            clock=t0 if stamps is None else stamps
            i=ids[np.argmin(clock[ids])]
            fraction=np.clip((clock[i]-t0[i])/dt[i],0.,1.)
            position=tuple(map(float,p0[i]+fraction*(p1[i]-p0[i])))
            conflicts.append(CollisionResult(False,reason,float(clock[i]),position,int(scenario[i]),track_id,separation))

        values=self.index.query(low,high)
        report(values==OCCUPIED,'STATIC_COLLISION');report(values==UNKNOWN,'UNKNOWN_BLOCKED')
        corners=np.array([[x,y,z] for x in (0,1) for y in (0,1) for z in (0,1)])
        road_low=np.minimum(p0,p1)-body-curvature;road_high=np.maximum(p0,p1)+body+curvature
        footprint=road_low[:,None,:]+corners*(road_high-road_low)[:,None,:]
        report(np.any(self.snapshot.route.violations(footprint.reshape(-1,3)).reshape(-1,8),axis=1),'ROAD_BOUNDARY')
        horizontal=np.maximum(np.linalg.norm(v0[:,:2],axis=1),np.linalg.norm(v1[:,:2],axis=1))
        vertical=np.maximum(abs(v0[:,2]),abs(v1[:,2]))
        report((horizontal>self.config.max_horizontal_speed+1e-8)|(vertical>self.config.max_vertical_speed+1e-8)|
               np.any(acceleration>np.asarray(self.config.max_acceleration)+1e-8,axis=1),'DYNAMICS_LIMIT')
        for obstacle in self.snapshot.obstacles:
            # Prediction is tied to each absolute sample time, not cloud age + t.
            unique,inverse=np.unique(np.r_[t0,t1],return_inverse=True)
            positions,uncertainty=obstacle.predict_arrays(unique)
            start,end=np.split(inverse,2)
            q0=p0-positions[start];q1=p1-positions[end];delta=q1-q0
            half=obstacle.bbox_size/2.+np.maximum(uncertainty[start],uncertainty[end])+inflation
            inverse_delta=np.zeros_like(delta);nonzero=abs(delta)>1e-12
            np.divide(1.,delta,out=inverse_delta,where=nonzero)
            a=(-half-q0)*inverse_delta;b=(half-q0)*inverse_delta
            near=np.minimum(a,b);far=np.maximum(a,b);near[~nonzero]=-np.inf;far[~nonzero]=np.inf
            outside=np.any((~nonzero)&(abs(q0)>half),axis=1)
            entry=np.maximum(0.,near.max(axis=1));exit=np.minimum(1.,far.min(axis=1))
            hit=(entry<=exit+1e-12)&~outside
            report(hit,'DYNAMIC_COLLISION',t0+entry*dt,obstacle.track_id,0.)
        return min(conflicts,key=lambda c:(c.stamp,c.scenario)) if conflicts else CollisionResult(True,'PASS')
