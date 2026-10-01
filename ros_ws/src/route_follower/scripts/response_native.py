#!/usr/bin/env python3
"""Optional compiled integrator; the Python model remains the reference."""
import ctypes
import os
from pathlib import Path
import numpy as np


class NativeResponse:
    def __init__(self):
        self.library=None
        source=Path(__file__).resolve()
        candidates=[os.environ.get('RMUA_RESPONSE_LIBRARY',''),
                    str(source.parents[3]/'devel/lib/libvelocity_response_native.so'),
                    str(source.parents[5]/'rmua_ws/devel/lib/libvelocity_response_native.so'),
                    str(source.parents[1]/'libvelocity_response_native.so'),
                    'libvelocity_response_native.so']
        for name in candidates:
            if not name:continue
            try:library=ctypes.CDLL(name)
            except OSError:continue
            library.rmua_response_abi.restype=ctypes.c_int
            if library.rmua_response_abi()!=2:continue
            pointer=ctypes.POINTER(ctypes.c_double)
            library.rmua_response_integrate.argtypes=[pointer,pointer,pointer,ctypes.c_int,pointer,ctypes.c_int,
                pointer,ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,
                ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_int,pointer,pointer]
            library.rmua_response_integrate.restype=ctypes.c_int
            self.library=library;self.path=name;break

    def integrate(self,position,velocity,commands,scenarios,path,latency,hold,deceleration,feedback,lift_gain,command_max,duration):
        arrays=[np.ascontiguousarray(a,dtype=np.float64) for a in
                (position,velocity,commands,scenarios,np.empty((0,3)) if path is None else path)]
        p,v,u,scenario,profile=arrays
        capacity=int(np.ceil(duration/.08))+4
        output=np.empty((capacity,len(u),len(scenario),3));times=np.empty(capacity)
        pointer=lambda a:a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        count=self.library.rmua_response_integrate(pointer(p),pointer(v),pointer(u),len(u),
            pointer(scenario),len(scenario),pointer(profile),len(profile),latency,hold,deceleration,
            feedback,lift_gain,command_max,duration,capacity,pointer(output),pointer(times))
        if count<0:raise RuntimeError('Native response integration failed: %d'%count)
        return output[:count],times[:count]
