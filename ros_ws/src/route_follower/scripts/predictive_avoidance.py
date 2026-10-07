#!/usr/bin/env python3
"""Continuous moving detours from stereo car tracks, certified by point clouds.

Car rectangles describe a face, not a complete vehicle. Tracks therefore use
an inflated longitudinal volume; lidar checks every segment of the executable
trajectory including entry and return. No instantaneous lateral displacement.
"""
import math
import time
import numpy as np
from lidar_clearance import LidarClearance
from point_index import PointIndex, measured_car_faces
from lattice_detour import search as lattice_search
from path_sampling import swept_samples


def departure_floor_limits(stations, positions, center, floor_offset, index, margin):
    """Shared NED floor bound during departure; measured floor takes priority."""
    stations=np.asarray(stations)
    limits=np.full(len(stations),np.inf)
    if floor_offset is None:return limits
    departure=stations<20.
    unique,inverse=np.unique(stations[departure],return_inverse=True)
    values=(center.batch(unique)+floor_offset if hasattr(center,'batch') else
            np.array([center(s)+floor_offset for s in unique]))
    limits[departure]=values[inverse]
    if index is not None:
        measured=index.floor_limit(positions,margin)
        supported=departure & np.isfinite(measured)
        limits[supported]=measured[supported]
    return limits


def smooth(t):
    t=np.clip(t,0.,1.)
    return t*t*t*(10.+t*(-15.+6.*t))


class Detour:
    def __init__(self,s,start,target,length,hold,return_length=22.,start_slope=(0.,0.)):
        self.s=float(s); self.start=np.asarray(start,dtype=float)
        self.target=np.asarray(target,dtype=float); self.length=float(length)
        self.hold=max(self.s+self.length,float(hold)); self.end=self.hold+return_length
        self.return_length=float(return_length)
        self.start_slope=np.asarray(start_slope,dtype=float)

    def offsets(self,stations):
        stations=np.asarray(stations,dtype=float)
        t=np.clip((stations-self.s)/self.length,0.,1.)
        # Quintic Hermite: measured initial direction, zero final slope and
        # acceleration. This also preserves a previous detour's entry tangent.
        initial=t-6*t**3+8*t**4-3*t**5
        value=self.start+(self.target-self.start)*smooth(t)[...,None]+self.length*self.start_slope*initial[...,None]
        value=np.where((stations>self.hold)[...,None],self.target*(1.-smooth((stations-self.hold)/self.return_length))[...,None],value)
        return value

    def offset(self,s):return self.offsets(float(s))

    def slope(self,s):return (self.offset(s+.1)-self.offset(s-.1))/.2


class JoinedDetour:
    """C2 spatial join at the application station, including planner delay."""
    def __init__(self,s,start,start_slope,start_accel,following,length):
        self.s=float(s);self.length=float(length);self.following=following
        self.join=self.s+self.length
        self.end=max(self.join,following.end if following is not None else self.join)
        self.target=following.target if following is not None else np.zeros(2)
        value=following.offset(self.join) if following is not None else np.zeros(2)
        slope=following.slope(self.join) if following is not None else np.zeros(2)
        accel=((following.slope(self.join+.1)-following.slope(self.join-.1))/.2
               if following is not None else np.zeros(2))
        self.coefficients=np.zeros((6,2))
        self.coefficients[:3]=[start,self.length*np.asarray(start_slope),.5*self.length**2*np.asarray(start_accel)]
        a,b,c=self.coefficients[:3]
        self.coefficients[3:]=np.linalg.solve(np.array([[1.,1.,1.],[3.,4.,5.],[6.,12.,20.]]),
            np.array([value-a-b-c,self.length*slope-b-2*c,self.length**2*accel-2*c]))

    def offset(self,s):
        if s>self.join:return self.following.offset(s) if self.following is not None else np.zeros(2)
        t=np.clip((s-self.s)/self.length,0.,1.)
        return np.array([t**i for i in range(6)])@self.coefficients

    def offsets(self,ss):return np.array([self.offset(s) for s in np.atleast_1d(ss)])

    def slope(self,s):
        if s>self.join:return self.following.slope(s) if self.following is not None else np.zeros(2)
        t=np.clip((s-self.s)/self.length,0.,1.)
        return np.array([0.]+[i*t**(i-1)/self.length for i in range(1,6)])@self.coefficients


class CarTracks:
    def __init__(self):self.tracks=[]; self.last_stamp=None

    def update(self,stamp,detections):
        if not math.isfinite(stamp) or (self.last_stamp is not None and stamp<=self.last_stamp):return
        self.last_stamp=stamp
        self.tracks=[t for t in self.tracks if 0.<=stamp-t['stamp']<2.]
        used=set()
        for d in detections:
            if not d.get('geometry_valid') or d.get('confidence',0.)<.5:continue
            p=np.asarray(d.get('world',[]),dtype=float)
            if p.shape!=(3,) or not np.all(np.isfinite(p)):continue
            uncertainty=max(.35,min(5.,float(d['depth'])-float(d.get('depth_near',d['depth']))))
            near=[(np.linalg.norm(p-t['world']),i) for i,t in enumerate(self.tracks) if i not in used]
            distance,index=min(near,default=(float('inf'),-1))
            if distance<max(3.,uncertainty):
                t=self.tracks[index]; used.add(index)
                t['samples'].append(p);t['samples']=t['samples'][-5:]
                t['world']=np.median(t['samples'],axis=0)
                t['hits']+=1
            else:
                t=dict(world=p,samples=[p],hits=1)
                self.tracks.append(t);used.add(len(self.tracks)-1)
            t.update(stamp=stamp,uncertainty=uncertainty,
                     half_width=max(.4,min(2.,float(d.get('face_half_width',.8)))),
                     half_height=max(.4,min(2.,float(d.get('face_half_height',.8)))))

    def snapshot(self,stamp):
        return [dict(t) for t in self.tracks if t['hits']>=2 and 0.<=stamp-t['stamp']<2.]


class PredictiveAvoidance:
    def __init__(self,margin=1.,braking=4.,budget=.5,vertical_limit=1.5):
        self.vertical_limit=vertical_limit
        self.margin=float(margin);self.braking=float(braking);self.plan=None
        self.budget=budget

    swept_samples=staticmethod(swept_samples)

    @staticmethod
    def positions(stations,xy,center,plan):
        rows=[]
        for s in stations:
            p=np.array([*xy(s)[:2],center(s)],dtype=float)
            a,b=np.asarray(xy(s-.3)[:2]),np.asarray(xy(s+.3)[:2])
            f=(b-a)/max(1e-6,np.linalg.norm(b-a))
            y,z=plan.offset(s) if plan is not None else (0.,0.)
            p[:2]+=y*np.array([-f[1],f[0]]);p[2]+=z;rows.append(p)
        return np.asarray(rows)

    def certify(self,path,points,cars,initial):
        minimum=float('inf'); first=None
        queries,segments,_=self.swept_samples(path)
        if hasattr(self,'index'):
            distances=self.index.distance(queries)
            minimum=float(np.min(distances))
            close=np.flatnonzero(distances<self.margin+.1)
            if len(close):first=int(segments[close[0]])
        if first is not None:return False,minimum,first
        if hasattr(self,'index') and not cars:return True,minimum,None
        if hasattr(self,'index'):
            for car in cars:
                delta=queries-car['world']
                dims=np.column_stack([delta[:,:2]@car['forward'],delta[:,:2]@car['side'],delta[:,2]])
                close=np.flatnonzero(np.all(abs(dims)<car['extent'],axis=1))
                if len(close):first=min(first if first is not None else len(path),int(segments[close[0]]))
            return first is None,minimum,first
        for j,(a,b) in enumerate(zip(path[:-1],path[1:])):
            lo=np.minimum(a,b)-self.margin;hi=np.maximum(a,b)+self.margin
            near=points[np.all((points>=lo)&(points<=hi),axis=1)]
            if len(near) and not hasattr(self,'index'):
                clearance=float(np.sqrt(np.min(LidarClearance._segment_squared(near,a[None,:],b[None,:]))))
                minimum=min(minimum,clearance)
                if clearance<self.margin:
                    first=j;break
            # Oriented, inflated whole-body box projected to the road frame.
            for car in cars:
                f,side=car['forward'],car['side']
                # Samples every .5m also cover thin entry/exit intersections.
                count=max(3,int(np.linalg.norm(b-a)/.35)+2)
                samples=a+np.linspace(0.,1.,count)[:,None]*(b-a)
                delta=samples-car['world']
                dims=np.column_stack([delta[:,:2]@f,delta[:,:2]@side,delta[:,2]])
                if np.any(np.all(abs(dims)<car['extent'],axis=1)):
                    first=j;break
            if first is not None:break
        return first is None,minimum,first

    def evaluate(self,s,position,velocity,xy,center,route,points,tracks,stamp,speed,gates=(),departure_floor_offset=None):
        started=time.monotonic();deadline=started+self.budget
        candidate_deadline=started+.45*self.budget
        self.speed=max(0.,float(speed))
        points=np.asarray(points,dtype=float)
        points=points[np.all(np.isfinite(points),axis=1)]
        # Use every local return for certification. A voxel representative can
        # hide the closest surface by its diagonal; no margin was compensating
        # for that loss in the previous implementation.
        points=points[np.linalg.norm(points-position,axis=1)<34.]
        self.index=PointIndex(points,origin=position,faces=measured_car_faces(tracks,route,stamp),road_height=5.)
        horizon=min(65.,max(30.,route.total_s-s))
        stations=s+np.arange(0.,horizon+.5,1.)
        cars=[]
        for t in tracks:
            cs=route.project_gate(*t['world'][:2])
            if not s-8.<cs<s+horizon+8.:continue
            f=np.asarray(route.tangent(cs)[:2]);side=np.array([-f[1],f[0]])
            age=max(0.,stamp-t['stamp'])
            # Stereo locates the visible face. Extrude the body away from the
            # aircraft rather than inventing several metres of body in front
            # of the observed surface. In lidar range, the thin face augments
            # actual whole-body points; outside it retain the full extrusion.
            far=cs-s>28.
            body_center=t['world']+np.array([*f,0.])*(2.5 if far else 0.)
            cars.append(dict(t,s=cs,world=body_center,forward=f,side=side,far=far,
                             extent=np.array([(3.5 if far else .6)+t['uncertainty']+age,
                                 t['half_width']+self.margin+.25+age*.3,
                                 t['half_height']+self.margin+.25+age*.3])))
        observed_cars=len(cars)
        # Fresh, accurately localized near faces are checked by PointIndex.
        # Coarse distant boxes still initiate predictive detours separately.
        cars=[c for c in cars if c['far']]
        baseline=self.positions(stations,xy,center,None);baseline[0]=position
        nominal=self.positions(stations,xy,center,None)
        def sides_at(ss):
            tangent=np.array([np.asarray(xy(t+.3)[:2])-np.asarray(xy(t-.3)[:2]) for t in ss])
            tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-6)
            return np.column_stack([-tangent[:,1],tangent[:,0],np.zeros(len(ss))])
        sides=sides_at(stations)
        dense_stations=s+np.arange(0.,horizon+.1,.2)
        dense_nominal=self.positions(dense_stations,xy,center,None)
        dense_sides=sides_at(dense_stations)
        def path_for(plan):
            offsets=plan.offsets(stations)
            path=nominal+offsets[:,0,None]*sides
            path[:,2]+=offsets[:,1];path[0]=position
            return path,offsets
        def dense_path_for(plan):
            offsets=plan.offsets(dense_stations)
            path=dense_nominal+offsets[:,0,None]*dense_sides
            path[:,2]+=offsets[:,1];path[0]=position
            return path
        def certify_plan(plan):
            path=dense_path_for(plan)
            limits=departure_floor_limits(dense_stations,path,center,departure_floor_offset,
                                           self.index,self.margin+.1)
            if np.any(path[:,2]>limits):return False,0.,None
            return self.certify(path,points,cars,position)
        clear,minclear,first=self.certify(baseline,points,cars,position)
        floor_limits=departure_floor_limits(stations,baseline,center,departure_floor_offset,
                                           self.index,self.margin+.1)
        violations=np.flatnonzero(baseline[:,2]>floor_limits)
        if len(violations):clear=False;first=int(violations[0]);minclear=0.
        # Keep the full spatial plan until the aircraft has returned to the
        # nominal corridor. Rebuilding its ramp at every cloud never completes.
        if self.plan is not None and (s<self.plan.end or not clear):
            path,_=path_for(self.plan)
            ok,minclear,_=certify_plan(self.plan)
            if ok:return self._result(stations,path,minclear,observed_cars)
        previous=self.plan
        if clear and (previous is None or
                      (np.linalg.norm(previous.offset(s))<.02 and np.linalg.norm(previous.slope(s))<.01)):
            self.plan=None
            return dict(active=False,feasible=True,cap=None,source='PREDICTIVE',cars=observed_cars,selected_clearance=minclear if math.isfinite(minclear) else None)
        obstacle=0. if clear else max(0.,float(stations[first]-s))
        a,b=np.asarray(xy(s-.3)[:2]),np.asarray(xy(s+.3)[:2]);f=(b-a)/max(1e-6,np.linalg.norm(b-a))
        side=np.array([-f[1],f[0]])
        start=np.array([(position[:2]-baseline[0,:2])@side,position[2]-center(s)])
        # baseline[0] was replaced by the aircraft; use the actual reference.
        start[0]=(position[:2]-np.asarray(xy(s)[:2]))@side
        if self.plan is not None:
            start=self.plan.offset(s);slope=self.plan.slope(s)
        else:
            progress=max(3.,float(velocity[:2]@f))
            slope=np.array([velocity[:2]@side/progress,
                            velocity[2]/progress-(center(s+.5)-center(s-.5))])
        hold=s+min(horizon-10.,max(obstacle+12.,max((c['s']-s+8. for c in cars if c['s']>s),default=0.)))
        best=None;score=float('inf')
        # A clear nominal corridor requires a certified return from the old
        # offset, rather than dropping the entire reference in one frame.
        if clear:
            for length in (12.,22.,32.):
                plan=Detour(s,start,(0.,0.),length,s+length,start_slope=slope)
                path,_=path_for(plan)
                ok,mc,_=certify_plan(plan)
                if ok:
                    self.plan=plan
                    return self._result(stations,path,mc,observed_cars)
        lengths=sorted(set([max(4.,min(28.,obstacle-2.)),max(4.,min(16.,obstacle-2.))]),reverse=True)
        targets=[(y,z) for y in (-3.,-2.5,-2.,-1.5,-1.,0.,1.,1.5,2.,2.5,3.)
                 for z in np.arange(-self.vertical_limit,self.vertical_limit+.01,.5)]
        if departure_floor_offset is not None and s<20.:
            # Include a smooth ramp just above the floor instead of rounding
            # it down to the coarse 0.5 m vertical search grid.
            bound=max(-self.vertical_limit,departure_floor_offset-.02)
            targets=list(set((y,min(z,bound)) for y,z in targets))
        targets.sort(key=lambda yz:(yz[0]**2+1.5*yz[1]**2))
        for length in lengths:
            for target in targets:
                if time.monotonic()>candidate_deadline:break
                if target[0]**2+1.5*target[1]**2>=score:continue
                plan=Detour(s,start,target,length,hold,start_slope=slope)
                path,offsets=path_for(plan)
                ok,mc,_=certify_plan(plan)
                if not ok:continue
                if np.max(abs(offsets[:,0]))>3.15 or np.max(abs(offsets[:,1]))>1.65:continue
                # Gate windows are sampled at their exact station. Penalize
                # missed scoring gates while lidar certifies their actual frame.
                missed=0
                for g in gates:
                    if not s<g['s']<stations[-1]:continue
                    gp=self.positions([g['s']],xy,center,plan)[0]
                    gf=np.asarray(route.tangent(g['s'])[:2]);gs=np.array([-gf[1],gf[0]])
                    lat=abs((gp[:2]-np.array([g['x'],g['y']]))@gs)
                    vert=abs(gp[2]-g['z'])
                    missed+=int(lat>1. or vert>1.)
                dd=np.diff(offsets,n=2,axis=0)
                cost=target[0]**2+1.5*target[1]**2+50.*float(np.max(np.linalg.norm(dd,axis=1)))+10.*missed
                if self.plan is not None:cost+=4.*float(np.sum((np.array(target)-self.plan.target)**2))
                if cost<score:best=(plan,path,mc);score=cost
        if best is None:
            options=lattice_search(stations,nominal,sides,start,self.index,self.margin,cars,self.plan,
                                   start_slope=slope,deadline=deadline,vertical_limit=self.vertical_limit)
            if options:
                for plan in options:
                    path,_=path_for(plan)
                    ok,mc,_=certify_plan(plan)
                    if ok:
                        best=(plan,path,mc);break
            if best is None and cars:
                local=baseline[:min(29,len(baseline))]
                clear,mc,_=self.certify(local,points,[],position)
                if clear:
                    # A far, uncertain car volume does not justify parking on
                    # measured clear road. Approach its lidar refinement zone
                    # with a braking envelope, then plan from actual surfaces.
                    distance=min(c['s']-s for c in cars)
                    cap=math.sqrt(2.*min(4.,self.braking)*max(0.,distance-24.))
                    # Retain any nonzero reference. Its local swept path must
                    # pass the same radar check before approaching refinement.
                    if previous is not None and np.linalg.norm(previous.offset(s))>.02:
                        local,_=path_for(previous)
                        ok,mc,_=self.certify(local[:min(29,len(local))],points,[],position)
                        if not ok:return dict(active=True,feasible=False,cap=0.,source='PREDICTIVE',cars=observed_cars)
                        self.plan=previous
                        info=self._result(stations,local,mc,observed_cars)
                        return dict(info,cap=min(cap,info['cap']),refinement=True)
                    self.plan=None
                    return dict(active=False,feasible=True,cap=cap,source='PREDICTIVE',cars=observed_cars,
                                refinement=True,obstacle_distance=distance,selected_clearance=mc)
            if best is None:return dict(active=True,feasible=False,cap=0.,source='PREDICTIVE',cars=observed_cars,obstacle_distance=obstacle,
                                       search_timeout=time.monotonic()>deadline)
        self.plan,path,mc=best
        return self._result(stations,path,mc,observed_cars,obstacle)

    def _result(self,stations,path,clearance,cars,obstacle=None):
        # The measured position-to-reference connector is a tracking error,
        # not a one-metre bend in the planned trajectory.
        dd=np.diff(path[1:],n=2,axis=0)
        local=np.sqrt(5./np.maximum(1e-5,np.linalg.norm(dd,axis=1)))
        distance=np.maximum(0.,stations[2:-1]-stations[0]-.35*self.speed)
        cap=float(min(30.,np.min(np.sqrt(local*local+2.*self.braking*distance))))
        y,z=self.plan.offset(stations[0])
        return dict(active=True,feasible=True,cap=cap,source='PREDICTIVE',cars=cars,
                    lateral=float(y),vertical=float(z),target=self.plan.target.tolist(),
                    start=self.plan.s,end=self.plan.end,entry_length=self.plan.length,
                    selected_clearance=clearance if math.isfinite(clearance) else None,
                    obstacle_distance=obstacle)
