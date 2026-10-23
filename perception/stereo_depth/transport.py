"""Bounded image/disparity IPC and a single latest pending item, without pickle."""
import hashlib
import json
import math
import struct
import threading

import numpy as np


MAX_HEADER = 16384
MAX_PAYLOAD = 16 * 1024 * 1024


def digest(data):
    return hashlib.sha256(data).hexdigest()


def send_packet(connection, header, payload):
    encoded = json.dumps(header,allow_nan=False,separators=(',',':')).encode()
    if not 0 < len(encoded) <= MAX_HEADER or len(payload) > MAX_PAYLOAD:
        raise ValueError('oversized stereo IPC packet')
    connection.sendall(struct.pack('!II',len(encoded),len(payload)))
    connection.sendall(encoded)
    connection.sendall(payload)


def receive_packet(connection):
    def exact(size):
        parts=[]
        while size:
            part=connection.recv(min(size,1024*1024))
            if not part:raise EOFError('stereo IPC closed mid-packet')
            parts.append(part);size-=len(part)
        return b''.join(parts)
    header_size,payload_size=struct.unpack('!II',exact(8))
    if not 0 < header_size <= MAX_HEADER or payload_size > MAX_PAYLOAD:
        raise ValueError('oversized stereo IPC descriptor')
    header=json.loads(exact(header_size))
    if not isinstance(header,dict):raise ValueError('stereo IPC requires object header')
    return header,exact(payload_size)


def encode_images(left,right):
    if left.shape != (720,960,3) or right.shape != left.shape or left.dtype!=np.uint8 or right.dtype!=np.uint8:
        raise ValueError('frozen online profile requires 960x720 RGB byte images')
    return left.tobytes()+right.tobytes()


def decode_images(header,payload):
    if header.get('image_shape') != [720,960,3] or len(payload)!=2*720*960*3 or digest(payload)!=header.get('input_sha256'):
        raise ValueError('invalid image IPC payload')
    images=np.frombuffer(payload,dtype=np.uint8).reshape(2,720,960,3)
    return images[0],images[1]


def encode_disparity(disparity,valid):
    if (disparity.shape!=(720,960) or valid.shape!=disparity.shape or valid.dtype!=np.bool_
            or not np.isfinite(disparity[valid]).all() or np.any(disparity[valid]<=0)
            or not np.isnan(disparity[~valid]).all()):
        raise ValueError('invalid diagnostic disparity IPC output')
    return disparity.astype('<f4').tobytes()+valid.astype(np.uint8).tobytes()


def decode_disparity(payload):
    count=720*960
    if len(payload)!=count*5:raise ValueError('invalid diagnostic disparity IPC size')
    disparity=np.frombuffer(payload[:count*4],dtype='<f4').reshape(720,960)
    mask=np.frombuffer(payload[count*4:],dtype=np.uint8).reshape(720,960)
    if np.any(mask>1):raise ValueError('invalid diagnostic validity mask')
    valid=mask.astype(bool)
    # Same finite/UNKNOWN contract as the sender; no implicit conversion to FREE.
    encode_disparity(disparity,valid)
    return disparity,valid


def freshness_reason(metadata,now,epoch,clock_fresh,source_arrival,wall_now,wall_ttl=.2):
    """Timing only. A timely diagnostic still has no control eligibility."""
    if not clock_fresh:return 'STALE_IMU_CLOCK'
    if metadata.epoch!=epoch:return 'OTHER_EPOCH'
    if not metadata.current(now,epoch):return 'SOURCE_LEASE_NOT_CURRENT'
    if not all(math.isfinite(v) for v in (source_arrival,wall_now)) or not 0<=wall_now-source_arrival<wall_ttl:
        return 'WALL_EXPIRED'
    return None


class LatestSlot:
    """One consumer owns its in-flight item; producers replace only the pending one."""
    def __init__(self):
        self._lock=threading.Lock();self._pending=None;self.closed=False
        self.submitted=0;self.superseded=0;self.taken=0;self.discarded_on_close=0

    def put(self,item):
        if item is None:raise ValueError('None is reserved for an empty pending slot')
        with self._lock:
            if self.closed:return False
            self.submitted+=1
            if self._pending is not None:self.superseded+=1
            self._pending=item
            return True

    def take(self):
        with self._lock:
            item,self._pending=self._pending,None
            if item is not None:self.taken+=1
            return item

    def close(self):
        with self._lock:
            self.closed=True
            if self._pending is not None:self.discarded_on_close+=1
            self._pending=None

    def stats(self):
        with self._lock:
            return {'capacity':1,'pending':int(self._pending is not None),'submitted':self.submitted,
                'superseded':self.superseded,'taken':self.taken,'discarded_on_close':self.discarded_on_close}
