#!/usr/bin/env python3
"""Frozen perception owner and a bounded, disposable space-time search process."""
from dataclasses import dataclass
import multiprocessing as mp
import threading
import time
from dynamic_tracker import DynamicTracker,TrackerConfig
from local_occupancy import LocalOccupancy,OccupancyConfig,owned_dynamic_returns
from trajectory_types import PlanningSnapshot
from trajectory_executor import collision_key
from st_lattice import STLattice,LatticeConfig
from trajectory_collision import CollisionConfig


@dataclass(frozen=True)
class PlannerReply:
    result: object
    epoch: int
    collision_key: str
    submitted_monotonic: float


def _plan_worker(connection,parameters,planner_config,collision_config):
    try:
        planner=STLattice(parameters,planner_config,collision_config)
        connection.send(('READY',None))
        while True:
            snapshot,delay=connection.recv()
            if delay:time.sleep(delay)
            result=planner.plan(snapshot)
            connection.send(('OK',result))
    except Exception as error:
        connection.send(('ERROR',type(error).__name__+': '+str(error)))
    finally:connection.close()


class PlannerProcess:
    """One bounded job. Hung/crashed jobs cannot block the publication owner."""
    def __init__(self,parameters,planner_config=None,collision_config=None,timeout=.3,context='spawn'):
        self.parameters=parameters;self.planner_config=planner_config or LatticeConfig()
        self.collision_config=collision_config or CollisionConfig()
        if timeout<=0 or timeout<self.planner_config.budget:raise ValueError('Worker timeout must cover search budget')
        self.timeout=timeout;self.context=mp.get_context(context);self.process=None;self.connection=None
        self.submitted=None;self.epoch=None;self.last_reason='IDLE';self.sequence=0
        self.ready=False;self._start()

    def _start(self):
        parent,child=self.context.Pipe()
        self.connection=parent
        self.process=self.context.Process(target=_plan_worker,args=(child,self.parameters,self.planner_config,
                                             self.collision_config),daemon=True)
        self.process.start();child.close();self.ready=False

    def _ready(self):
        if self.process is None:self._start()
        if not self.ready and self.connection.poll():
            try:status,_=self.connection.recv()
            except (EOFError,OSError):status='ERROR'
            self.ready=status=='READY'
            if not self.ready:self.last_reason='WORKER_START_FAILED';self.close()
        return self.ready

    def submit(self,snapshot,delay=0.):
        if self.submitted is not None or not self._ready():return False
        self.submitted=time.monotonic();self.epoch=snapshot.epoch
        try:self.connection.send((snapshot,delay))
        except (EOFError,OSError,BrokenPipeError):self.close();self.last_reason='WORKER_SEND_FAILED';return False
        self.last_reason='PLANNING';return True

    def poll(self):
        if self.process is None or self.submitted is None:return None
        # Age is checked before accepting a buffered completion too.
        if time.monotonic()-self.submitted>=self.timeout:
            self.last_reason='WORKER_TIMEOUT';self.close();return None
        if self.connection.poll():
            try:status,value=self.connection.recv()
            except (EOFError,OSError):status,value='ERROR','WORKER_EOF'
            epoch,submitted=self.epoch,self.submitted
            self.submitted=None
            if status!='OK':self.last_reason=str(value);return None
            self.last_reason=value.reason;self.sequence+=1
            if value.trajectory is not None:
                from dataclasses import replace
                value=replace(value,trajectory=replace(value.trajectory,plan_id=self.sequence))
            return PlannerReply(value,epoch,collision_key(self.collision_config),submitted)
        if not self.process.is_alive():self.last_reason='WORKER_EXITED';self.close()
        return None

    def close(self):
        process,self.process=self.process,None
        if process is not None:
            if process.is_alive():process.terminate()
            process.join(timeout=.02)
            if process.is_alive():process.kill();process.join(timeout=.02)
            process.close()
        if self.connection is not None:self.connection.close();self.connection=None
        self.submitted=None;self.ready=False


class SpaceTimeNavigation:
    """Map mutation belongs to the perception thread. Executor never waits.

    The caller supplies an immutable route reference and the true exposure
    origin. Map TTLs are evaluated by the owning map, at each execution time.
    """
    def __init__(self,tracker_config=None,occupancy_config=None):
        self.tracker=DynamicTracker(tracker_config or TrackerConfig())
        self.mapping=LocalOccupancy(occupancy_config or OccupancyConfig())
        self.lock=threading.Lock();self.epoch=0;self.source_epoch=None;self.last_frame=None
        self.last_reason='NO_CLOUD'
        self.published=None

    def reset(self):
        with self.lock:
            self.tracker.reset();self.mapping.reset();self.epoch+=1;self.source_epoch=None;self.last_frame=None
            self.published=None

    def ingest(self,frame,route,position):
        with self.lock:
            identity=(frame.epoch,frame.stamp)
            if identity==self.last_frame:return False
            if (self.source_epoch is not None and frame.epoch!=self.source_epoch) or (
                    self.last_frame is not None and frame.stamp<self.last_frame[1]):
                self.tracker.reset();self.mapping.reset();self.epoch+=1
                self.published=None
            self.source_epoch=frame.epoch
            coord=route.project([position])[0];forward,_=route.frame(coord[0])
            center=lambda s:route.world(s,0.,0.)[2]
            obstacles=self.tracker.update(frame.points,position,frame.stamp,forward,center,coord[0])
            owners=owned_dynamic_returns(len(frame.points),obstacles,frame.stamp,self.mapping.config)
            self.mapping.update(frame.points,frame.sensor_origin,frame.stamp,position,forward,owners,frame.epoch)
            # Publish evidence and tracks atomically only after a complete frame.
            self.published=(self.epoch,self.mapping.evidence(),tuple(obstacles),frame.sensor_origin)
            self.last_frame=identity;self.last_reason='PASS';return True

    def snapshot(self,response_state,route,reaction_delay=0.,evaluation_now=None):
        published=self.published
        if published is None:self.last_reason='NO_CLOUD';return None
        epoch,evidence,obstacles,sensor_origin=published
        if evidence.stamp>response_state.stamp+1e-8:self.last_reason='NO_SYNCHRONIZED_CLOUD';return None
        now=response_state.stamp if evaluation_now is None else evaluation_now
        mapping=evidence.snapshot(now)
        return PlanningSnapshot(epoch,mapping.version,response_state.stamp,response_state,route,mapping,
                                obstacles,reaction_delay,evaluation_now,sensor_origin)
