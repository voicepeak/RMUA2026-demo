"""Bounded latest-only reference IPC. The worker never creates ROS publishers.

Perception/planning may later replace this development reference generator.
Worker liveness and freshness do not certify obstacle clearance or localization.
"""
from dataclasses import dataclass
import math
import multiprocessing as mp
import os
from queue import Empty,Full
import time
from control.reference import benchmark_reference,benchmark_yaw
from interfaces.contracts import finite_vec3


@dataclass(frozen=True)
class BenchmarkSpec:
    origin: tuple
    yaw: float
    step: bool=False
    braking: bool=False
    yaw_motion: bool=False


@dataclass(frozen=True)
class ReferencePacket:
    stamp: float
    epoch: int
    requested_wall: float
    position: tuple
    velocity: tuple
    acceleration: tuple
    yaw: float
    yaw_rate: float
    max_compute_wall_s: float
    stall_performed: bool

    def __post_init__(self):
        for value in (self.position,self.velocity,self.acceleration):finite_vec3(value)
        if (not all(math.isfinite(x) for x in (self.stamp,self.requested_wall,self.yaw,
                    self.yaw_rate,self.max_compute_wall_s)) or self.stamp<=0 or self.epoch<0
                    or self.max_compute_wall_s<0):raise ValueError('invalid reference packet')


def offer_latest(queue,item):
    try:
        queue.put_nowait(item)
        return True
    except Full:
        try:queue.get_nowait()
        except Empty:return False
        try:
            queue.put_nowait(item)
            return True
        except Full:return False


def process_main(spec,requests,results,stop,stall_seconds,parent_pid):
    # Linux process lifetime belongs to the flight invocation even if the host
    # has to terminate its parent before Python finally blocks can execute.
    import ctypes
    import signal
    libc=ctypes.CDLL(None,use_errno=True)
    if libc.prctl(1,signal.SIGTERM,0,0,0)!=0:
        offer_latest(results,{'error':'PARENT_DEATH_SIGNAL_UNAVAILABLE'})
        return
    if os.getppid()!=parent_pid:return
    stalled=False
    max_compute=0.
    while not stop.is_set() and os.getppid()==parent_pid:
        try:request=requests.get(timeout=.05)
        except Empty:continue
        stamp,epoch,elapsed,requested_wall=request
        begin=time.monotonic()
        try:
            if stall_seconds and not stalled and elapsed>=6.:
                stalled=True
                # Deliberately block only this worker. The PWM loop receives no
                # special injection notification and must detect stale output.
                time.sleep(stall_seconds)
            p,v,a=benchmark_reference(spec.origin,elapsed,spec.step,spec.braking)
            yaw,rate=benchmark_yaw(spec.yaw,elapsed,spec.yaw_motion)
            max_compute=max(max_compute,time.monotonic()-begin)
            packet=ReferencePacket(stamp,epoch,requested_wall,tuple(p),tuple(v),tuple(a),
                                   yaw,rate,max_compute,stalled)
            offer_latest(results,packet)
        except Exception as error:
            offer_latest(results,{'error':str(error)})
            return


class ReferenceWorker:
    def __init__(self,spec,stall_seconds=0.,sensor_ttl=.08,wall_ttl=.15):
        finite_vec3(spec.origin)
        if (not math.isfinite(spec.yaw) or not 0<=stall_seconds<=1.
                or not 0<sensor_ttl<=.1 or not 0<wall_ttl<=.3):
            raise ValueError('invalid reference worker configuration')
        context=mp.get_context('spawn')
        self.requests=context.Queue(maxsize=1)
        self.results=context.Queue(maxsize=1)
        self.stop=context.Event()
        self.process=context.Process(target=process_main,args=(spec,self.requests,self.results,self.stop,stall_seconds,os.getpid()),
                                     name='rmua_reference_worker',daemon=True)
        self.sensor_ttl,self.wall_ttl=sensor_ttl,wall_ttl
        self.latest=None
        self.error=None
        self.submitted=self.dropped=self.received=self.stale_observations=0
        self.max_compute=0.
        self.stall_observed=False

    def start(self):self.process.start()

    def submit(self,stamp,epoch,elapsed):
        if not all(math.isfinite(v) for v in (stamp,elapsed)) or stamp<=0 or epoch<0:
            raise ValueError('invalid reference request')
        if offer_latest(self.requests,(stamp,epoch,elapsed,time.monotonic())):self.submitted+=1
        else:self.dropped+=1

    def snapshot(self,stamp,epoch):
        # Each mailbox holds one result. Never drain an unbounded producer loop.
        try:
            value=self.results.get_nowait()
            if isinstance(value,ReferencePacket):
                self.latest=value;self.received+=1
                self.max_compute=max(self.max_compute,value.max_compute_wall_s)
                self.stall_observed=self.stall_observed or value.stall_performed
            else:self.error=value.get('error','INVALID_REFERENCE_RESULT')
        except Empty:pass
        if self.error:return None,'WORKER_ERROR'
        if not self.process.is_alive():return None,'WORKER_EXITED'
        p=self.latest
        if p is None:return None,'NO_REFERENCE'
        if p.epoch!=epoch:return None,'REFERENCE_EPOCH'
        if not 0<=stamp-p.stamp<=self.sensor_ttl or not 0<=time.monotonic()-p.requested_wall<=self.wall_ttl:
            self.stale_observations+=1
            return None,'REFERENCE_STALE'
        return p,None

    def close(self):
        self.stop.set()
        if self.process.pid is not None:
            self.process.join(timeout=.2)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=1.)
        for queue in (self.requests,self.results):
            queue.cancel_join_thread();queue.close()

    def summary(self):
        return {'pid':self.process.pid,'submitted':self.submitted,'requests_dropped':self.dropped,
                'received':self.received,'stale_observations':self.stale_observations,
                'max_compute_wall_s':self.max_compute,'stall_observed':self.stall_observed,
                'fault':self.error,'sensor_ttl_s':self.sensor_ttl,'wall_ttl_s':self.wall_ttl,
                'alive_after_close':self.process.is_alive()}
