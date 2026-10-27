"""Bounded multi-frame candidate consensus, without report authorization."""
from collections import deque
from dataclasses import dataclass, field
import math
from interfaces.contracts import Metadata
from inspection.meter_reader.angular_scale import MeterCandidate


@dataclass(frozen=True)
class MeterConsensus:
    status: str
    device_track_id: str = None
    value: int = None
    observations: int = 0
    source_stamp: float = None
    source_valid_until: float = None
    epoch: int = 0
    report_eligible: bool = field(default=False, init=False)
    device_identity_verified: bool = field(default=False, init=False)


class ConsensusTracker:
    def __init__(self, epoch=0, required=3, history_s=.8, wall_ttl_s=.2):
        if (isinstance(epoch,bool) or not isinstance(epoch,int) or epoch<0
                or isinstance(required,bool) or not isinstance(required,int) or not 2<=required<=10
                or not math.isfinite(history_s) or not .1<=history_s<=2
                or not math.isfinite(wall_ttl_s) or not 0<wall_ttl_s<=.5):
            raise ValueError('invalid consensus limits')
        self.epoch,self.required,self.history_s,self.wall_ttl_s=epoch,required,history_s,wall_ttl_s
        self._items=deque(maxlen=required);self._last_stamp=None;self._last_wall=None

    def reset(self,epoch):
        if isinstance(epoch,bool) or not isinstance(epoch,int) or epoch<=self.epoch:raise ValueError('epoch must increase')
        self.epoch=epoch;self._items.clear();self._last_stamp=self._last_wall=None

    def observe(self,meta,arrival,now,wall,device,lights,candidate):
        if (not isinstance(meta,Metadata) or not meta.current(now,self.epoch)
                or not isinstance(device,str) or not device or len(device)>128
                or not isinstance(lights,tuple) or len(lights)!=3 or any(x is not True and x is not False and x is not None for x in lights)
                or not isinstance(candidate,MeterCandidate) or not all(math.isfinite(x) for x in (arrival,wall))
                or arrival<0 or not 0<=wall-arrival<self.wall_ttl_s or (self._last_wall is not None and arrival<=self._last_wall)
                or (self._last_stamp is not None and meta.stamp<=self._last_stamp)):
            raise ValueError('invalid or duplicate observation')
        self._last_stamp,self._last_wall=meta.stamp,arrival
        if (lights != (True,True,True) or candidate.status!='CANDIDATE'
                or isinstance(candidate.value,bool) or not isinstance(candidate.value,int) or not 0<=candidate.value<=50):
            self._items.clear()
            return MeterConsensus('UNCONFIRMED',epoch=self.epoch)
        if self._items and (self._items[-1][2]!=device or self._items[-1][3]!=candidate.value
                            or self._items[-1][0].frame_id != meta.frame_id):self._items.clear()
        self._items.append((meta,arrival,device,candidate.value))
        while self._items and (meta.stamp-self._items[0][0].stamp>self.history_s
                               or arrival-self._items[0][1]>self.history_s):self._items.popleft()
        return self.snapshot(now,wall)

    def snapshot(self,now,wall):
        if (not self._items or not all(math.isfinite(x) for x in (now,wall))):
            return MeterConsensus('UNAVAILABLE',epoch=self.epoch)
        meta,arrival,device,value=self._items[-1]
        if (not meta.current(now,self.epoch) or not 0<=wall-arrival<self.wall_ttl_s
                or now-self._items[0][0].stamp>self.history_s
                or wall-self._items[0][1]>self.history_s):
            return MeterConsensus('UNAVAILABLE',epoch=self.epoch)
        return MeterConsensus('CONSENSUS' if len(self._items)>=self.required else 'PENDING',
                              device,value,len(self._items),meta.stamp,meta.valid_until,self.epoch)
