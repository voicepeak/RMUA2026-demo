#!/usr/bin/env python3
"""Bounded Weighted A* over route progress, offset, height, speed and time.

Nodes also own ALL response-scenario velocities/command memory/control phases.
Only the first primitive is executable: it has a full certified feedback stop
tail. A 4s search horizon never truncates that tail. No reverse/recovery search.
"""
from dataclasses import dataclass
from collections import Counter
import heapq
import time
import math
import numpy as np
from trajectory_types import Primitive,TimedTrajectory,PlanResult,CollisionResult,join_traces
from route_coordinates import HeightProfile
from response_rollout import rollout_primitive,rollout_delay,rollout_stop
from trajectory_collision import TrajectoryCollision,CollisionConfig


@dataclass(frozen=True)
class LatticeConfig:
    time_step: float = .25
    horizon: float = 4.
    forward_distance: float = 12.
    speed_levels: tuple = (0.,2.,4.,6.,8.)
    lateral_rate: float = 1.5
    lateral_step: float = .5
    lateral_target_margin: float = .1
    lateral_damping: float = 2.
    height_step: float = .5
    height_gain: float = 1.
    s_resolution: float = 1.
    lateral_resolution: float = .5
    height_resolution: float = .5
    velocity_resolution: float = .25
    command_resolution: float = .25
    heuristic_weight: float = 1.5
    budget: float = .1
    max_expansions: int = 1000
    stop_max_duration: float = 12.
    minimum_progress: float = .01
    goal_offset: float = 2.25

    def __post_init__(self):
        object.__setattr__(self,'speed_levels',tuple(map(float,self.speed_levels)))
        for key,value in vars(self).items():
            if key=='speed_levels':continue
            if not np.isfinite(value) or value<0:raise ValueError('Invalid lattice option: '+key)
        positive=('time_step','horizon','forward_distance','height_gain','s_resolution','lateral_resolution',
                  'height_resolution','velocity_resolution','command_resolution','heuristic_weight','budget','stop_max_duration')
        if any(getattr(self,k)<=0 for k in positive):raise ValueError('Lattice parameters must be positive')
        if self.horizon<self.time_step or self.forward_distance>25. or self.heuristic_weight<1.:
            raise ValueError('Invalid bounded search horizon/weight')
        if (isinstance(self.max_expansions,bool) or not isinstance(self.max_expansions,int) or self.max_expansions<1 or
                not self.speed_levels or self.speed_levels[0]!=0. or self.speed_levels[-1]<=0. or
                any(not np.isfinite(v) or v<0 for v in self.speed_levels) or np.any(np.diff(self.speed_levels)<=0.)):
            raise ValueError('Invalid lattice speeds/expansion budget')


@dataclass
class _Node:
    state: object
    parent: object
    edge: object
    primitive: object
    profile: object
    cost: float
    backup: object = None
    backup_profile: object = None
    first_end: object = None
    stop_risk: float = 0.


class STLattice:
    def __init__(self,parameters,config=None,collision_config=None):
        self.parameters=parameters;self.config=LatticeConfig() if config is None else config
        self.collision_config=CollisionConfig() if collision_config is None else collision_config
        if self.config.speed_levels[-1]>self.collision_config.max_horizontal_speed:
            raise ValueError('Planner command speed exceeds configured collision limit')
        self.plan_id=0

    def coordinates(self,state,route):
        coordinates=route.project(state.position)
        return np.median(coordinates,axis=0),coordinates

    def label_key(self,state,route,start_stamp):
        config=self.config;coord,_=self.coordinates(state,route)
        tangent,_=route.frame(coord[0]);v=float(np.median(state.velocity[:,:2]@tangent))
        displayed=tuple(np.rint([coord[0]/config.s_resolution,coord[1]/config.lateral_resolution,
                      coord[2]/config.height_resolution,v/config.velocity_resolution,
                      (state.stamp-start_stamp)/config.time_step]).astype(np.int64))
        # A coarse (s,y,z,v,t) key alone would silently discard lateral inertia.
        memory=np.r_[state.position.ravel()/config.velocity_resolution,state.velocity.ravel()/config.velocity_resolution,
                     state.previous.ravel()/config.command_resolution,state.applied.ravel()/config.command_resolution,
                     state.effective.ravel()/config.command_resolution,
                     (state.next_control-state.stamp)/.001,(state.stamp-state.last_control)/.001,
                     (state.next_physics-state.stamp)/.001]
        return displayed+tuple(np.rint(memory).astype(np.int64))

    def actions(self,state,route):
        config=self.config;coord,_=self.coordinates(state,route)
        tangent,side=route.frame(coord[0]);speed=float(np.median(state.velocity[:,:2]@tangent))
        levels=np.asarray(config.speed_levels)
        height=float(np.rint(coord[2]/config.height_resolution)*config.height_resolution)
        nearest=int(np.argmin(abs(levels-max(0.,speed))))
        choices=sorted(set([0,nearest,max(0,nearest-1),min(len(levels)-1,nearest+1),len(levels)-1]),reverse=True)
        actions=[]
        usable=max(0.,route.lateral_limit-float(abs(side)@np.asarray(self.collision_config.body_half_extent)[:2])-config.lateral_target_margin)
        targets=sorted(set([float(coord[1]),float(np.clip(coord[1]-config.lateral_step,-usable,usable)),
                           float(np.clip(coord[1]+config.lateral_step,-usable,usable)),-usable,usable])) if config.lateral_rate>0 else [float(coord[1])]
        for i in choices:
            along=levels[i]
            if along==0.:
                actions.append((Primitive([0.,0.,0.],config.time_step,'WAIT'),HeightProfile(route,coord[1],coord[2],config.height_gain)))
                for lateral in targets:
                    if config.lateral_rate>0 and abs(lateral-coord[1])>1e-8:
                        actions.append((Primitive([0.,0.,0.],config.time_step),HeightProfile(route,lateral,height,
                            config.height_gain,2.,config.lateral_rate,config.lateral_damping)))
                if config.height_step>0.:
                    for offset in (-config.height_step,config.height_step):
                        actions.append((Primitive([0.,0.,0.],config.time_step),HeightProfile(route,coord[1],height+offset,
                            config.height_gain,2.,config.lateral_rate,config.lateral_damping)))
                continue
            for lateral in targets:
                target=np.r_[along*tangent,0.]
                profile=HeightProfile(route,lateral,height,config.height_gain,2.,config.lateral_rate,config.lateral_damping)
                actions.append((Primitive(target,config.time_step),profile))
            if config.height_step>0:
                for offset in (-config.height_step,config.height_step):
                    profile=HeightProfile(route,coord[1],height+offset,config.height_gain,2.,config.lateral_rate,config.lateral_damping)
                    actions.append((Primitive(np.r_[along*tangent,0.],config.time_step),profile))
        return actions

    def _trajectory(self,node,delay,checker,snapshot):
        nodes=[];current=node
        while current.parent is not None:nodes.append(current);current=current.parent
        nodes.reverse()
        trace=join_traces(([delay] if delay is not None else [])+[n.edge for n in nodes])
        validation=checker.check(trace)
        if not validation.safe:return None,validation
        self.plan_id+=1
        from trajectory_executor import collision_key
        trajectory=TimedTrajectory(self.plan_id,snapshot.epoch,snapshot.scene_version,snapshot.pose_stamp,
            node.first_end,trace,tuple(n.primitive for n in nodes),tuple(n.profile for n in nodes),
            node.backup,node.backup_profile,node.cost,validation,collision_key(self.collision_config))
        return trajectory,validation

    def _sustained_blocker(self,state,snapshot):
        """A moving box covers the straight local road throughout the horizon.

        This is evidence for choosing a certified wait, not a NO_PATH proof for
        arbitrary curved roads or a reason to ignore the backup collision check.
        CV centers are linear; their physical boxes must cover both endpoints.
        Uncertainty is omitted here, making this early wait decision stricter.
        """
        route=snapshot.route;median,_=self.coordinates(state,route)
        tangent,side=route.frame(median[0]);directions,_=route.frame(route.stations)
        if np.any(directions@tangent<1.-1e-6) or np.ptp(route.points[:,2])>1e-6:return None
        body=np.asarray(self.collision_config.body_half_extent)
        lateral=route.lateral_limit-float(abs(side)@body[:2]);vertical=route.vertical_limit-body[2]
        for obstacle in snapshot.obstacles:
            if abs(obstacle.velocity[:2]@tangent)>.05 or abs(obstacle.velocity[2])>.05 or abs(obstacle.velocity[:2]@side)<=.05:continue
            centers=obstacle.position+(np.array([state.stamp,state.stamp+self.config.horizon])-obstacle.timestamp)[:,None]*obstacle.velocity
            coordinates=route.project(centers)
            if not median[0]<coordinates[0,0]<median[0]+self.config.forward_distance:continue
            half=obstacle.bbox_size/2.+body+self.collision_config.margin
            width=float(abs(side)@half[:2])
            if (np.all(coordinates[:,1]-width<=-lateral) and np.all(coordinates[:,1]+width>=lateral) and
                    np.all(coordinates[:,2]-half[2]<=-vertical) and np.all(coordinates[:,2]+half[2]>=vertical)):
                return obstacle.track_id
        return None

    def plan(self,snapshot):
        started=time.monotonic();deadline=started+self.config.budget
        stats=Counter();expanded=0;candidates=0;first_conflict=None;exhausted=False;blocking_track_id=None
        checker=TrajectoryCollision(snapshot,self.collision_config)

        def reject(result):
            nonlocal first_conflict
            stats[result.reason]+=1
            if first_conflict is None:first_conflict=result

        def result(reason,trajectory=None):
            return PlanResult(reason,trajectory,expanded,candidates,tuple(sorted(stats.items())),
                              1000.*(time.monotonic()-started),exhausted,first_conflict,blocking_track_id)

        fresh=checker.freshness()
        if not fresh.safe:reject(fresh);return result(fresh.reason)
        if snapshot.response_state.model_key!=self.parameters.model_key:return result('MODEL_MISMATCH')
        delay=None;state=snapshot.response_state
        if snapshot.reaction_delay>0.:
            delay=rollout_delay(self.parameters,state,snapshot.reaction_delay)
            check=checker.check(delay)
            if not check.safe:reject(check);return result(check.reason)
            state=delay.end_state
        root=_Node(state,None,None,None,None,0.)
        initial,coords=self.coordinates(state,snapshot.route);initial_s=float(coords[:,0].min())
        goal=min(initial_s+self.config.forward_distance,float(snapshot.route.stations[-1])-.5)
        best=None;fallback=None;best_score=-np.inf;queue=[];serial=0;seen={}
        obstacle_frames=[(obstacle,snapshot.route.project(obstacle.position[None,:])[0]) for obstacle in snapshot.obstacles]
        def heuristic(n):
            median,coordinates=self.coordinates(n.state,snapshot.route)
            distance=max(0.,goal-float(coordinates[:,0].min()))
            tangent,_=snapshot.route.frame(median[0])
            speed=max(0.,float(np.max(n.state.velocity[:,:2]@tangent)))
            maximum=max(speed,self.config.speed_levels[-1]);tau=min(s[0] for s in self.parameters.scenarios)
            # Optimistic command slew + fastest lag. Permit one feedback-period
            # command jump, so the continuous ramp bounds discrete preparation.
            rate=self.parameters.acceleration
            command=min(maximum,max(0.,float(np.max(n.state.previous[:,:2]@tangent)))+rate*max(self.parameters.periods))
            cap=max(0.,(maximum-command)/rate)
            def ramp(t):
                return (command*t+rate*(.5*t*t-tau*t)+
                        (speed-command+rate*tau)*tau*(1.-math.exp(-t/tau)))
            cap_distance=ramp(cap)
            cap_speed=command+rate*(cap-tau)+(speed-command+rate*tau)*math.exp(-cap/tau)
            def reach(t):
                if t<=cap:return ramp(t)
                extra=t-cap
                return cap_distance+maximum*extra+(cap_speed-maximum)*tau*(1.-math.exp(-extra/tau))
            lo=0.;hi=distance/max(maximum,1e-8)+tau+cap
            for _ in range(16):
                mid=(lo+hi)/2.
                if reach(mid)<distance:lo=mid
                else:hi=mid
            travel=hi
            side=np.array([-tangent[1],tangent[0]])
            for obstacle,observation in obstacle_frames:
                if observation[0]<=median[0] or observation[0]>goal:continue
                front=obstacle.bbox_size/2.+np.asarray(self.collision_config.body_half_extent)+self.collision_config.margin
                ahead=max(0.,observation[0]-median[0]-float(abs(tangent)@front[:2]))
                lo=0.;hi=ahead/max(maximum,1e-8)+tau+cap
                for _ in range(12):
                    mid=(lo+hi)/2.
                    if reach(mid)<ahead:lo=mid
                    else:hi=mid
                arrival=hi
                predicted,uncertainty=obstacle.predict_arrays([n.state.stamp+arrival])
                future=snapshot.route.project(predicted)[0]
                half=obstacle.bbox_size/2.+uncertainty[0]+np.asarray(self.collision_config.body_half_extent)+self.collision_config.margin
                if abs(future[2]-median[2])>half[2]:continue
                lateral_velocity=float(np.median(n.state.velocity[:,:2]@side))
                lateral_command=float(np.median(n.state.applied[:,:2]@side))
                projected=median[1]+lateral_command*arrival+(lateral_velocity-lateral_command)*tau*(1.-math.exp(-arrival/tau))
                extent=float(abs(side)@half[:2])
                if abs(projected-future[1])>extent:continue
                escape=np.inf;usable=snapshot.route.lateral_limit-float(abs(side)@np.asarray(self.collision_config.body_half_extent)[:2])
                if self.config.lateral_rate>0.:
                    for target in (future[1]-extent,future[1]+extent):
                        if abs(target)<usable:escape=min(escape,abs(target-projected)/self.config.lateral_rate)
                motion=float(obstacle.velocity[:2]@side);waiting=np.inf
                current=snapshot.route.project((obstacle.position+(n.state.stamp-obstacle.timestamp)*obstacle.velocity)[None,:])[0]
                if abs(motion)>.05:
                    boundary=projected+(extent if motion>0 else -extent)
                    waiting=max(0.,(boundary-current[1])/motion-arrival)
                penalty=min(escape,waiting,self.config.horizon)
                travel+=penalty
            return travel
        heapq.heappush(queue,(self.config.heuristic_weight*heuristic(root),serial,root))
        while queue:
            if time.monotonic()>=deadline or expanded>=self.config.max_expansions:exhausted=True;break
            _,_,node=heapq.heappop(queue)
            if node.parent is not None:
                key=self.label_key(node.state,snapshot.route,snapshot.pose_stamp)
                if node.cost>seen.get(key,np.inf)+1e-9:continue
                median,coords=self.coordinates(node.state,snapshot.route)
                if float(coords[:,0].min())>=goal and abs(median[1])<=self.config.goal_offset:
                    trajectory,check=self._trajectory(node,delay,checker,snapshot)
                    if trajectory is not None:return result('GOAL_REACHED',trajectory)
                    reject(check)
            if node.state.stamp-state.stamp+self.config.time_step>self.config.horizon+1e-8:continue
            expanded+=1
            actions=self.actions(node.state,snapshot.route)
            # Certify braking first, so a deadline can retain a safe waiting plan.
            if node.parent is None:actions.sort(key=lambda item:item[0].name!='WAIT')
            for primitive,profile in actions:
                if time.monotonic()>=deadline:exhausted=True;break
                candidates+=1
                edge=rollout_primitive(self.parameters,node.state,primitive,profile)
                median,coords=self.coordinates(edge.end_state,snapshot.route)
                delta_v=edge.end_state.velocity-node.state.velocity
                delta_u=edge.end_state.applied-node.state.applied
                cost=(node.cost+primitive.duration*(1.+.05*abs(median[1])+.05*abs(median[2]))+
                      .002*float(np.mean(np.sum(delta_v**2,axis=1)))+.001*float(np.mean(np.sum(delta_u**2,axis=1))))
                key=self.label_key(edge.end_state,snapshot.route,snapshot.pose_stamp)
                if cost>=seen.get(key,np.inf)-1e-9:
                    stats['DOMINATED_LABEL']+=1;continue
                check=checker.check(edge)
                if not check.safe:reject(check);continue
                backup=node.backup;backup_profile=node.backup_profile;first_end=node.first_end
                if node.parent is None:
                    # Preserve the chosen height reference. Using predicted Z
                    # as the next reference accumulates coupling/lag bias on
                    # every replan and can invalidate a previously viable stop.
                    backup_lateral=float(self.coordinates(edge.end_state,snapshot.route)[0][1])
                    backup_profile=HeightProfile(snapshot.route,backup_lateral,profile.height,self.config.height_gain)
                    backup,stopped=rollout_stop(self.parameters,edge.end_state,backup_profile,self.config.stop_max_duration,
                                               chunk=self.config.time_step,deadline=deadline)
                    if not stopped:
                        reject(CollisionResult(False,'STOP_UNSETTLED'))
                        if time.monotonic()>=deadline:exhausted=True;break
                        continue
                    check=checker.check(backup)
                    if not check.safe:reject(check);continue
                    first_end=edge.times[-1]
                stop_risk=node.stop_risk
                if node.parent is None and snapshot.obstacles:
                    # A certified tail ending millimetres before an obstacle
                    # is fragile under the next measured response update.
                    # Prefer a braking reserve as a SOFT search objective;
                    # hard geometry and uncertainty remain unchanged.
                    separations=[]
                    for obstacle in snapshot.obstacles:
                        predicted,uncertainty=obstacle.predict_arrays([backup.times[-1]])
                        half=obstacle.bbox_size/2.+uncertainty[0]+np.asarray(self.collision_config.body_half_extent)+self.collision_config.margin
                        separations.append(np.linalg.norm(np.maximum(0.,abs(backup.positions[-1]-predicted[0])-half),axis=1).min())
                    stop_risk=min(2.,.25/(min(separations)+.1))
                    cost+=stop_risk
                child=_Node(edge.end_state,node,edge,primitive,profile,cost,backup,backup_profile,first_end,stop_risk)
                if node.parent is None and primitive.name=='WAIT':
                    blocking_track_id=self._sustained_blocker(node.state,snapshot)
                    if blocking_track_id is not None:
                        trajectory,check=self._trajectory(child,delay,checker,snapshot)
                        if trajectory is not None:return result('WAIT_FOR_DYNAMIC',trajectory)
                        reject(check)
                if float(coords[:,0].min())>=goal and abs(median[1])<=self.config.goal_offset:
                    trajectory,check=self._trajectory(child,delay,checker,snapshot)
                    if trajectory is not None:return result('GOAL_REACHED',trajectory)
                    reject(check)
                seen[key]=cost;serial+=1
                heapq.heappush(queue,(cost+self.config.heuristic_weight*heuristic(child),serial,child))
                progress=float(coords[:,0].min())-initial_s
                score=-heuristic(child)-.01*cost-child.stop_risk
                movement=max(progress,abs(median[1]-initial[1]),abs(median[2]-initial[2]))
                if movement>=self.config.minimum_progress and score>best_score:best=child;best_score=score
                if fallback is None and primitive.name=='WAIT' and node.parent is None:fallback=child
            if exhausted:break
        chosen=best if best is not None else fallback
        if chosen is not None:
            trajectory,check=self._trajectory(chosen,delay,checker,snapshot)
            if trajectory is None:reject(check);return result(check.reason)
            if exhausted:return result('SEARCH_TIMEOUT',trajectory)
            if best is not None:return result('SAFE_PARTIAL',trajectory)
            if stats['DYNAMIC_COLLISION']:return result('WAIT_FOR_DYNAMIC',trajectory)
            if stats['UNKNOWN_BLOCKED']:return result('UNKNOWN_BLOCKED',trajectory)
            return result('NO_PATH',trajectory)
        if exhausted:return result('SEARCH_TIMEOUT')
        if stats['UNKNOWN_BLOCKED']:return result('UNKNOWN_BLOCKED')
        return result('NO_PATH')
