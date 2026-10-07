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
            if library.rmua_response_abi()!=12:continue
            pointer=ctypes.POINTER(ctypes.c_double)
            library.rmua_response_integrate.argtypes=[pointer,pointer,pointer,ctypes.c_int,pointer,ctypes.c_int,
                pointer,ctypes.c_int,pointer,ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,
                ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_int,ctypes.c_double,pointer,pointer,ctypes.c_int,ctypes.c_int,pointer,pointer]
            library.rmua_response_integrate.restype=ctypes.c_int
            library.rmua_response_samples.argtypes=[pointer,pointer,ctypes.c_int,ctypes.c_int,ctypes.c_int,
                ctypes.c_double,pointer,pointer,pointer,ctypes.POINTER(ctypes.c_int64),pointer]
            library.rmua_response_samples.restype=ctypes.c_int64
            library.rmua_route_stations.argtypes=[pointer,ctypes.c_int,pointer,pointer,pointer,pointer,ctypes.c_int,pointer]
            library.rmua_route_stations.restype=None
            library.rmua_roof_queries.argtypes=[pointer,ctypes.c_int,pointer,pointer,pointer,pointer,
                ctypes.c_int,ctypes.c_double,ctypes.c_int,ctypes.c_double,pointer,pointer]
            library.rmua_roof_queries.restype=None
            library.rmua_patch_distances.argtypes=[pointer,ctypes.c_int,ctypes.POINTER(ctypes.c_int64),
                ctypes.c_int,pointer,pointer,pointer,pointer,pointer,ctypes.c_int,pointer]
            library.rmua_patch_distances.restype=None
            self.library=library;self.path=name;break

    def integrate(self,position,velocity,commands,scenarios,path,latency,hold,deceleration,feedback,lift_gain,command_max,duration,applied=None,height_gain=1.,height_path=None,coupling_limited=False,xy_error_max=np.inf,control_periods=None):
        arrays=[np.ascontiguousarray(a,dtype=np.float64) for a in
                (position,velocity,commands,scenarios,np.empty((0,3)) if path is None else path)]
        p,v,u,scenario,profile=arrays
        vertical_profile=np.ascontiguousarray(profile if height_path is None else height_path,dtype=np.float64)
        periods=np.ascontiguousarray(np.zeros(len(scenario)) if control_periods is None else control_periods,dtype=np.float64)
        old=np.ascontiguousarray(np.zeros(3) if applied is None else applied,dtype=np.float64)
        capacity=int(np.ceil((duration+latency)/.08))+5
        output=np.empty((capacity,len(u),len(scenario),3));times=np.empty(capacity)
        pointer=lambda a:a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        count=self.library.rmua_response_integrate(pointer(p),pointer(v),pointer(u),len(u),
            pointer(scenario),len(scenario),pointer(profile),len(profile),pointer(vertical_profile),len(vertical_profile),latency,hold,deceleration,
            feedback,lift_gain,command_max,duration,height_gain,coupling_limited,xy_error_max,pointer(periods),pointer(old),applied is not None,capacity,pointer(output),pointer(times))
        if count<0:raise RuntimeError('Native response integration failed: %d'%count)
        return output[:count],times[:count]

    def project(self,queries,projection):
        queries=np.ascontiguousarray(queries,dtype=np.float64)
        stations,road,segments,lengths=[np.ascontiguousarray(a,dtype=np.float64) for a in projection]
        output=np.empty(len(queries))
        pointer=lambda a:a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.library.rmua_route_stations(pointer(queries),len(queries),pointer(stations),pointer(road),
            pointer(segments),pointer(lengths),len(lengths),pointer(output))
        return output

    def samples(self,path,times,position):
        path,times,position=[np.ascontiguousarray(a,dtype=np.float64) for a in (path,times,position)]
        offsets=np.empty(path.shape[1]+1,dtype=np.int64);extents=np.empty(path.shape[1])
        pointer=lambda a:a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        offsets_pointer=offsets.ctypes.data_as(ctypes.POINTER(ctypes.c_int64))
        arguments=[pointer(path),pointer(times),*path.shape[:3],.18,pointer(position)]
        count=self.library.rmua_response_samples(*arguments,None,None,offsets_pointer,None)
        if count<0:raise RuntimeError('Native response sample count failed')
        queries=np.empty((count,3));query_times=np.empty(count)
        filled=self.library.rmua_response_samples(*arguments,pointer(queries),pointer(query_times),offsets_pointer,pointer(extents))
        if filled!=count:raise RuntimeError('Native response sample allocation mismatch')
        return [(queries[offsets[i]:offsets[i+1]],float(extents[i]),query_times[offsets[i]:offsets[i+1]])
                for i in range(len(extents))]

    def roof_queries(self,queries,roof,road_height,margin=0.):
        queries,origin,coeff,hull,normals=[np.ascontiguousarray(a,dtype=np.float64) for a in (queries,*roof)]
        distances=np.empty(len(queries));floors=np.empty(len(queries))
        pointer=lambda a:a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.library.rmua_roof_queries(pointer(queries),len(queries),pointer(origin),pointer(coeff),pointer(hull),
            pointer(normals),len(hull),0. if road_height is None else road_height,road_height is not None,
            margin,pointer(distances),pointer(floors))
        return distances,floors

    def patch_distances(self,queries,ids,patches):
        queries,centers,normals,bases,boundaries,offsets=[np.ascontiguousarray(a,dtype=np.float64) for a in (queries,*patches[:5])]
        ids=np.ascontiguousarray(ids,dtype=np.int64);output=np.empty(len(queries))
        pointer=lambda a:a.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        self.library.rmua_patch_distances(pointer(queries),len(queries),ids.ctypes.data_as(ctypes.POINTER(ctypes.c_int64)),
            ids.shape[1],pointer(centers),pointer(normals),pointer(bases),pointer(boundaries),pointer(offsets),len(centers),pointer(output))
        return output
