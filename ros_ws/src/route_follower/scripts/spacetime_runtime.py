#!/usr/bin/env python3
"""Opt-in ROS adapter: perception/search coordinator and unique publisher.

Publication runs on a monotonic scheduler independently of RouteFollower's
mission lock. A verified external bridge hold contract is required for motion.
No in-repository VelCmd duration field exists, so its default remains false.
"""
from dataclasses import replace
import threading
import time
import math
import numpy as np
from velocity_response import VelocityResponse
from response_rollout import ResponseParameters
from trajectory_executor import TrajectoryExecutor
from spacetime_navigation import SpaceTimeNavigation,PlannerProcess
from st_lattice import LatticeConfig
from trajectory_collision import CollisionConfig


class SpaceTimeRuntime:
    def __init__(self,observe,publish,diagnostic,period=.05,planner_config=None,collision_config=None,
                 bridge_hold=None,bridge_verified=False,pose_timeout=.3):
        self.observe=observe;self.publish=publish;self.diagnostic=diagnostic;self.period=period
        self.model=VelocityResponse(native=False,lift_gain=.095,coupling_gains=(.075,.11,.13),
                                    coupling_limited=True,xy_error_max=4.5)
        # Actual fastest feedback equals the publication period. Slow feedback
        # scenarios remain to cover .16/.4s observed response uncertainty.
        original=self.model.freeze_parameters()
        self.parameters=replace(original,scenarios=tuple(s for s in original.scenarios for _ in (period,.16,.4)),
                                periods=tuple(t for _ in original.scenarios for t in (period,.16,.4)),
                                integration_step=min(period,.08))
        self.bridge_verified=bool(bridge_verified and bridge_hold is not None and np.isfinite(bridge_hold)
                                  and 0.<bridge_hold<=period)
        if bridge_verified and not self.bridge_verified:raise ValueError('Verified bridge hold must be in (0, executor period]')
        self.navigation=SpaceTimeNavigation()
        self.planner=PlannerProcess(self.parameters,planner_config,collision_config)
        self.executor=TrajectoryExecutor(self.parameters,collision_config,period,pose_timeout,guard_budget=.04)
        self.stop_event=threading.Event();self.reference=None;self.reference_epoch=0
        self.reply=None;self.last_submission=0.;self.last_decision=None;self.publisher_ident=None
        self.owner_lock=threading.Lock();self.model_lock=threading.Lock()
        self.threads=[]
        self.archive_dir=None;self.last_archive=0.;self.archive_sequence=0

    def start(self):
        for target in (self._coordinate,self._execute):
            thread=threading.Thread(target=target,daemon=True);self.threads.append(thread);thread.start()

    def set_reference(self,route,epoch):
        with self.owner_lock:
            if epoch!=self.reference_epoch:
                self.reference_epoch=epoch;self.reply=None
            self.reference=route

    def _coordinate(self):
        previous_epoch=None
        while not self.stop_event.wait(.02):
            try:
                with self.owner_lock:route=self.reference;epoch=self.reference_epoch
                if route is None:continue
                if epoch!=previous_epoch:
                    self.planner.close();self.navigation.reset();previous_epoch=epoch
                observation=self.observe()
                if observation is None:continue
                frame,position,velocity,stamp,arrival,yaw,ros_now=observation
                if frame is not None:self.navigation.ingest(frame,route,position)
                with self.model_lock:state=self.model.snapshot_state(position,velocity,stamp,self.parameters)
                if not 0.<=ros_now-stamp<self.executor.pose_timeout:continue
                snapshot=self.navigation.snapshot(state,route,ros_now-stamp,ros_now)
                if snapshot is None:continue
                if self.stop_event.is_set():break
                reply=self.planner.poll()
                if reply is not None:
                    with self.owner_lock:
                        if epoch==self.reference_epoch:self.reply=reply
                if time.monotonic()-self.last_submission>=.1:
                    if self.planner.submit(snapshot):self.last_submission=time.monotonic()
                if self.archive_dir and time.monotonic()-self.last_archive>=1.:
                    from pathlib import Path
                    from planning_snapshot_io import write_snapshot
                    self.last_archive=time.monotonic();self.archive_sequence+=1
                    write_snapshot(snapshot,self.parameters,Path(self.archive_dir)/('%06d.json'%self.archive_sequence),
                        dict(kind='live_frozen_input',sensor_origin=None if frame is None else frame.sensor_origin.tolist(),
                             cloud_stamp=None if frame is None else frame.stamp,
                             source_epoch=None if frame is None else frame.epoch,
                             bridge_verified=self.bridge_verified,planner_reason=self.planner.last_reason))
            except Exception as error:
                self.diagnostic(dict(mode='FAILSAFE',reason='COORDINATOR_ERROR',error=repr(error)))

    def _execute(self):
        self.publisher_ident=threading.get_ident();due=time.monotonic();previous_epoch=None;last_emit=0.
        while not self.stop_event.is_set():
            now=time.monotonic()
            if now<due:
                self.stop_event.wait(due-now);continue
            started=now;due=max(due+self.period,now+self.period)
            try:
                with self.owner_lock:route=self.reference;epoch=self.reference_epoch;reply=self.reply;self.reply=None
                observation=self.observe()
                if epoch!=previous_epoch:
                    self.executor.reset();previous_epoch=epoch
                    with self.model_lock:self.model.reset()
                snapshot=None;arrival=None;position=np.zeros(3);velocity=np.zeros(3);stamp=None;yaw=0.
                if observation is not None:
                    frame,position,velocity,stamp,arrival,yaw,ros_now=observation
                    if route is not None:
                        with self.model_lock:
                            if self.model.last_stamp is not None and stamp<self.model.last_stamp:
                                self.model.reset();self.executor.reset()
                            state=self.model.snapshot_state(position,velocity,stamp,self.parameters)
                        if 0.<=ros_now-stamp<self.executor.pose_timeout:
                            snapshot=self.navigation.snapshot(state,route,ros_now-stamp,ros_now)
                        # A newer unprocessed scan may overturn the old plan.
                        # Never certify a fresh drive against an older map.
                        if snapshot is not None and frame is not None and (
                                snapshot.occupancy.stamp<frame.stamp-1e-8 or
                                self.navigation.source_epoch!=frame.epoch):snapshot=None
                if snapshot is not None and reply is not None and reply.result.trajectory is not None:
                    self.executor.accept(reply.result.trajectory,snapshot,reply.collision_key,reply.submitted_monotonic)
                decision=self.executor.tick(snapshot,pose_arrival=arrival,bridge_verified=self.bridge_verified)
                # No slow certification result may be published after its deadline.
                if time.monotonic()-started>=self.period:
                    from trajectory_executor import ExecutionDecision
                    decision=ExecutionDecision(np.zeros(3),'FAILSAFE','PUBLICATION_DEADLINE',False)
                    self.executor.plan=None
                command=decision.command
                yaw_rate=0.
                if route is not None and observation is not None and decision.certified:
                    s=route.project([position])[0,0];tangent,_=route.frame(s)
                    target=math.atan2(tangent[1],tangent[0]);error=math.atan2(math.sin(target-yaw),math.cos(target-yaw))
                    yaw_rate=float(np.clip(error,-1.,1.))
                with self.model_lock:
                    self.publish(command,yaw_rate,yaw,stamp,velocity,decision)
                    if stamp is not None:self.model.commit(command,velocity,stamp)
                self.last_decision=decision
                if now-last_emit>=.2:
                    last_emit=now
                    self.diagnostic(dict(mode=decision.mode,reason=decision.reason,certified=decision.certified,
                        plan_id=decision.plan_id,epoch=self.navigation.epoch,
                        compute_ms=1000.*(time.monotonic()-started),planner_reason=self.planner.last_reason,
                        model_key=self.parameters.model_key,
                        conflict=None if decision.conflict is None else vars(decision.conflict)))
            except Exception as error:
                # An adapter failure must clear an already issued drive.
                self.executor.plan=None
                from trajectory_executor import ExecutionDecision
                decision=ExecutionDecision(np.zeros(3),'FAILSAFE','EXECUTOR_ERROR',False)
                try:self.publish(decision.command,0.,0.,None,np.zeros(3),decision)
                except Exception as publication_error:
                    self.diagnostic(dict(mode='FAILSAFE',reason='PUBLICATION_FAILED',error=repr(publication_error)))
                self.diagnostic(dict(mode='FAILSAFE',reason='EXECUTOR_ERROR',error=repr(error)))

    def close(self):
        self.stop_event.set()
        for thread in self.threads:thread.join(timeout=.5)
        self.planner.close()
