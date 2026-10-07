#!/usr/bin/env python3
"""Persistent nonsemantic LiDAR tracks with CV Kalman prediction.

No ROS or controller state. All public obstacle snapshots own immutable arrays;
prediction takes an absolute sensor-clock timestamp. Tentative/cropped tracks
remain visible, and are never discarded merely for moving slowly.
motion_confirmed describes return-ownership quality, not permission to ignore
other tracks during collision checking; coasting/tentative predictions matter.
"""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import linear_sum_assignment
from lidar_scene import extract_clusters


def _readonly(value):
    result=np.asarray(value,dtype=float).copy()
    result.setflags(write=False)
    return result


@dataclass(frozen=True)
class TrackerConfig:
    measurement_std: float = .15
    acceleration_std: float = 1.5
    initial_velocity_std: float = 5.
    association_distance: float = 2.
    association_mahalanobis: float = 16.
    size_cost: float = .5
    confirmation_hits: int = 3
    max_coast: float = .75
    tentative_lifetime: float = .4
    bbox_consistency: float = .35
    bbox_noise_scale: float = 4.
    base_margin: float = .15
    uncertainty_sigma: float = 2.
    truncated_margin: float = .3
    max_velocity_std: float = 2.
    moving_speed: float = .3

    def __post_init__(self):
        for key,value in vars(self).items():
            if not np.isfinite(value) or value<0:
                raise ValueError('Tracker parameter must be finite and nonnegative: '+key)
        for key in ('measurement_std','acceleration_std','initial_velocity_std',
                    'association_distance','association_mahalanobis','max_coast',
                    'tentative_lifetime','bbox_consistency','max_velocity_std'):
            if getattr(self,key)<=0:raise ValueError('Tracker parameter must be positive: '+key)
        if (isinstance(self.confirmation_hits,bool) or
                int(self.confirmation_hits)!=self.confirmation_hits or self.confirmation_hits<2):
            raise ValueError('confirmation_hits must be an integer >= 2')


def _predict(state,covariance,dt,acceleration_std):
    transition=np.eye(6);transition[:3,3:]=np.eye(3)*dt
    noise=np.vstack((np.eye(3)*dt*dt/2.,np.eye(3)*dt))
    return (transition@state,
            transition@covariance@transition.T+acceleration_std**2*(noise@noise.T))


@dataclass(frozen=True)
class ObstaclePrediction:
    track_id: int
    epoch: int
    timestamp: float
    position: np.ndarray
    bbox_size: np.ndarray
    covariance: np.ndarray
    uncertainty: np.ndarray

    @property
    def low(self):return self.position-self.bbox_size/2.-self.uncertainty

    @property
    def high(self):return self.position+self.bbox_size/2.+self.uncertainty


@dataclass(frozen=True)
class DynamicObstacle:
    track_id: int
    epoch: int
    timestamp: float
    observed_stamp: float
    created_stamp: float
    position: np.ndarray
    observed_position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    bbox_size: np.ndarray
    covariance: np.ndarray
    age: float
    observations: int
    confidence: float
    status: str
    truncated: bool
    motion_confirmed: bool
    point_indices: np.ndarray
    config: TrackerConfig

    @property
    def position_covariance(self):return self.covariance[:3,:3]

    @property
    def velocity_covariance(self):return self.covariance[3:,3:]

    def predict_at(self,stamp):
        if not np.isfinite(stamp) or stamp<self.timestamp-1e-9:
            raise ValueError('Prediction timestamp precedes track snapshot')
        dt=max(0.,stamp-self.timestamp)
        state,covariance=_predict(np.r_[self.position,self.velocity],self.covariance,
                                  dt,self.config.acceleration_std)
        uncertainty=(self.config.base_margin+
                     self.config.uncertainty_sigma*np.sqrt(np.maximum(0.,np.diag(covariance)[:3]))+
                     (self.config.truncated_margin if self.truncated else 0.))
        return ObstaclePrediction(self.track_id,self.epoch,float(stamp),_readonly(state[:3]),
                                  self.bbox_size,_readonly(covariance),_readonly(uncertainty))

    def predict(self,dt):
        """Relative to this immutable snapshot's timestamp, NOT last detection."""
        if not np.isfinite(dt) or dt<0:raise ValueError('Prediction dt must be nonnegative')
        return self.predict_at(self.timestamp+dt)

    def predict_arrays(self,stamps):
        """Batch CV centers/uncertainty; exactly the diagonals of predict_at.

        Avoid constructing a full 6x6 prediction for every collision segment.
        This does not reduce uncertainty or discard coasting/tentative tracks.
        """
        stamps=np.asarray(stamps,dtype=float)
        if stamps.ndim!=1 or not np.all(np.isfinite(stamps)) or np.any(stamps<self.timestamp-1e-9):
            raise ValueError('Prediction timestamps precede track snapshot')
        dt=np.maximum(0.,stamps-self.timestamp)
        position=self.position+dt[:,None]*self.velocity
        variance=(np.diag(self.covariance)[:3]+2.*dt[:,None]*np.diag(self.covariance[:3,3:])+
                  dt[:,None]**2*np.diag(self.covariance)[3:]+
                  self.config.acceleration_std**2*dt[:,None]**4/4.)
        uncertainty=(self.config.base_margin+self.config.uncertainty_sigma*np.sqrt(np.maximum(0.,variance))+
                     (self.config.truncated_margin if self.truncated else 0.))
        return _readonly(position),_readonly(uncertainty)

    def summary(self):
        future=[self.predict(dt) for dt in (1.,2.)]
        return dict(track_id=self.track_id,epoch=self.epoch,status=self.status,
                    timestamp=self.timestamp,observed_stamp=self.observed_stamp,age=self.age,
                    observations=self.observations,confidence=self.confidence,
                    position=self.position.tolist(),velocity=self.velocity.tolist(),
                    observed_position=self.observed_position.tolist(),
                    bbox_size=self.bbox_size.tolist(),truncated=self.truncated,
                    motion_confirmed=self.motion_confirmed,
                    position_covariance=self.position_covariance.tolist(),
                    velocity_covariance=self.velocity_covariance.tolist(),
                    predictions=[dict(dt=dt,position=p.position.tolist(),
                                      low=p.low.tolist(),high=p.high.tolist(),
                                      uncertainty=p.uncertainty.tolist())
                                 for dt,p in zip((1.,2.),future)])


class DynamicTracker:
    def __init__(self,config=None):
        self.config=TrackerConfig() if config is None else config
        self.epoch=0;self.next_id=1;self.stamp=None;self._tracks=[]
        self.last_info={}

    def reset(self):
        self.epoch+=1;self.stamp=None;self._tracks=[];self.last_info={}
        # IDs remain unique across resets as an extra protection for consumers.

    def update(self,points,position,stamp,forward,center,s,**cluster_parameters):
        return self.update_clusters(extract_clusters(points,position,forward,center,s,
                                                     **cluster_parameters),stamp)

    def _measurement(self,track,cluster):
        low_shift=cluster.low-track['low']
        high_shift=cluster.high-track['high']
        variation=float(np.linalg.norm(low_shift-high_shift))
        inconsistent=variation>self.config.bbox_consistency or cluster.truncated
        scale=1.+(self.config.bbox_noise_scale if inconsistent else 0.)
        # Crop/asymmetric changes can move the observed center without motion.
        # Inflate position noise and withhold motion ownership in that case.
        covariance=np.eye(3)*(self.config.measurement_std*scale)**2
        innovation=cluster.center-track['state'][:3]
        total=track['covariance'][:3,:3]+covariance
        mahalanobis=float(innovation@np.linalg.solve(total,innovation))
        distance=float(np.linalg.norm(innovation))
        size_error=float(np.linalg.norm(cluster.size-(track['high']-track['low'])))
        return covariance,inconsistent,mahalanobis,distance,size_error,float(np.linalg.slogdet(total)[1])

    def update_clusters(self,clusters,stamp):
        if not np.isfinite(stamp):raise ValueError('Nonfinite observation timestamp')
        clusters=list(clusters)
        for c in clusters:
            if (np.shape(c.center)!=(3,) or np.shape(c.low)!=(3,) or np.shape(c.high)!=(3,) or
                    not np.all(np.isfinite(np.r_[c.center,c.low,c.high])) or np.any(c.high<c.low) or
                    np.any(c.center<c.low) or np.any(c.center>c.high) or
                    np.ndim(c.point_indices)!=1 or not np.issubdtype(np.asarray(c.point_indices).dtype,np.integer) or
                    np.any(np.asarray(c.point_indices)<0)):
                raise ValueError('Invalid cluster bounds')
        if self.stamp is not None and stamp==self.stamp:return self.snapshot()
        if self.stamp is not None and stamp<self.stamp:self.reset()
        dt=0. if self.stamp is None else stamp-self.stamp
        expired=0;remaining=[]
        for track in self._tracks:
            lifetime=self.config.max_coast if track['confirmed'] else self.config.tentative_lifetime
            if stamp-track['observed_stamp']>lifetime:
                expired+=1;continue
            track['state'],track['covariance']=_predict(track['state'],track['covariance'],
                                                        dt,self.config.acceleration_std)
            track['point_indices']=np.empty(0,dtype=int)
            remaining.append(track)
        self._tracks=remaining
        costs=np.full((len(remaining),len(clusters)),1e9)
        measurements={}
        for i,track in enumerate(remaining):
            for j,cluster in enumerate(clusters):
                measurement=self._measurement(track,cluster)
                R,inconsistent,mahalanobis,distance,size_error,logdet=measurement
                if distance>self.config.association_distance or mahalanobis>self.config.association_mahalanobis:
                    continue
                # The innovation likelihood also penalizes a noisy/cropped
                # observation; large R must not win just by reducing Mahalanobis.
                costs[i,j]=mahalanobis+logdet+self.config.size_cost*size_error
                measurements[i,j]=measurement
        rows,cols=linear_sum_assignment(costs)
        matched_tracks=set();matched_clusters=set()
        for i,j in zip(rows,cols):
            if (i,j) not in measurements:continue
            track=remaining[i];cluster=clusters[j]
            R,inconsistent,_,_,_,_=measurements[i,j]
            covariance=track['covariance']
            gain=np.linalg.solve(covariance[:3,:3]+R,covariance[:3,:]).T
            track['state']+=gain@(cluster.center-track['state'][:3])
            residual=np.eye(6);residual[:,:3]-=gain
            track['covariance']=residual@covariance@residual.T+gain@R@gain.T
            track['covariance']=(track['covariance']+track['covariance'].T)/2.
            track['observations']+=1;track['hits']+=1
            track['consistent_hits']=0 if inconsistent else track['consistent_hits']+1
            track['confirmed']|=track['hits']>=self.config.confirmation_hits
            # Retain recent maximum dimensions while the track lives: a
            # changing visible face must not silently shrink a collision box.
            track['size']=np.maximum(track['size'],cluster.size)
            track.update(observed_stamp=float(stamp),low=cluster.low.copy(),high=cluster.high.copy(),
                         observed_center=cluster.center.copy(),truncated=bool(cluster.truncated),
                         point_indices=np.asarray(cluster.point_indices,dtype=int).copy())
            matched_tracks.add(i);matched_clusters.add(j)
        for i,track in enumerate(remaining):
            if i not in matched_tracks:
                track['hits']=0;track['consistent_hits']=0
        created=0
        for j,cluster in enumerate(clusters):
            if j in matched_clusters:continue
            covariance=np.diag([self.config.measurement_std**2]*3+
                                [self.config.initial_velocity_std**2]*3)
            self._tracks.append(dict(track_id=self.next_id,state=np.r_[cluster.center,np.zeros(3)],
                covariance=covariance,observed_stamp=float(stamp),created_stamp=float(stamp),
                observations=1,hits=1,consistent_hits=0,confirmed=False,
                low=cluster.low.copy(),high=cluster.high.copy(),size=cluster.size.copy(),
                observed_center=cluster.center.copy(),truncated=bool(cluster.truncated),
                point_indices=np.asarray(cluster.point_indices,dtype=int).copy()))
            self.next_id+=1;created+=1
        self.stamp=float(stamp)
        self.last_info=dict(clusters=len(clusters),matched=len(matched_clusters),created=created,
                            expired=expired,epoch=self.epoch)
        return self.snapshot()

    def snapshot(self):
        result=[]
        for track in self._tracks:
            age=self.stamp-track['observed_stamp']
            covariance=track['covariance']
            velocity_std=np.sqrt(np.maximum(0.,np.diag(covariance)[3:])).max()
            status=('COASTING' if age>0. else 'CONFIRMED') if track['confirmed'] else 'TENTATIVE'
            confidence=(min(1.,track['observations']/self.config.confirmation_hits)*
                        np.exp(-age/self.config.max_coast)/(1.+velocity_std))
            motion_confirmed=(track['confirmed'] and not track['truncated'] and
                              track['consistent_hits']>=self.config.confirmation_hits-1 and
                              velocity_std<=self.config.max_velocity_std)
            indices=track['point_indices'].copy();indices.setflags(write=False)
            result.append(DynamicObstacle(track['track_id'],self.epoch,self.stamp,
                track['observed_stamp'],track['created_stamp'],_readonly(track['state'][:3]),
                _readonly(track['observed_center']),
                _readonly(track['state'][3:]),_readonly(np.zeros(3)),_readonly(track['size']),
                _readonly(covariance),age,track['observations'],float(confidence),status,
                track['truncated'],bool(motion_confirmed),indices,self.config))
        return tuple(result)

    def summary(self):
        obstacles=self.snapshot()
        return dict(self.last_info,track_count=len(obstacles),
                    confirmed_count=sum(o.status=='CONFIRMED' for o in obstacles),
                    coasting_count=sum(o.status=='COASTING' for o in obstacles),
                    moving_count=sum(o.motion_confirmed and bool(np.linalg.norm(o.velocity)>=self.config.moving_speed)
                                     for o in obstacles),tracks=[o.summary() for o in obstacles])
