#!/usr/bin/env python3
"""Continuous gate-corridor path; crossing a gate never switches the target."""
import math
from spatial_curve import SpatialCurve
from altitude_profile import ProfileBlender

class XYTracker:
    def __init__(self, lookahead_base=8., lookahead_kv=.7, k_pursuit=1.2,
                 xy_converge=.5, gate_blend_start=25., gate_blend_full=8.,
                 exit_blend_distance=4., half_width=1.5, margin=.8, acceleration=4.):
        self.lookahead_base = float(lookahead_base)
        self.lookahead_kv = float(lookahead_kv)
        self.k_pursuit = float(k_pursuit)
        self.clearance = max(.1, half_width-margin)
        self.acceleration = acceleration
        self.reset()

    def reset(self):
        self.route = self.offset = None
        self.previous_velocity = (0., 0.)
        self.last_s = 0.
        self.offset_blender=ProfileBlender(.4)
        self.stamp=None

    def lookahead(self, v): return self.lookahead_base+self.lookahead_kv*v

    def basis(self, s):
        a,b = self.route.point_at(max(0.,s-.5)), self.route.point_at(s+.5)
        dx,dy = b[0]-a[0], b[1]-a[1]
        length = max(1e-6, math.hypot(dx,dy))
        return dx/length,dy/length

    def configure(self, route, gates, s_now=0.,stamp=None):
        old = self.offset
        previous_center=lambda s:self.offset_blender.center(s,stamp)
        self.route = route
        rows = []
        for g in gates:
            if not g.get('valid',True) or g['s'] <= s_now+3.: continue
            s = g['s']
            x,y,_ = route.point_at(s)
            dx,dy = self.basis(s)
            lateral = -(g['x']-x)*dy+(g['y']-y)*dx
            lo,hi = lateral-self.clearance,lateral+self.clearance
            rows.append([s,lo,hi,max(lo,min(hi,0.))])
        rows.sort()
        # Prefer a straight line through overlapping openings, rather than gate centers.
        for _ in range(16):
            for i,row in enumerate(rows):
                numerator,denominator = 0.,.02
                for j in (i-1,i+1):
                    if 0 <= j < len(rows):
                        weight = 1./max(1.,abs(rows[j][0]-row[0]))
                        numerator += weight*rows[j][3]
                        denominator += weight
                row[3] = max(row[1],min(row[2],numerator/denominator))
        anchors = [(0.,0.)] if old is None else [(max(0.,s_now-2.),previous_center(max(0.,s_now-2.))),
                                                 (s_now,previous_center(s_now))]
        anchors += [(r[0],r[3]) for r in rows]
        anchors += [(max(s_now+30.,anchors[-1][0]+30.),anchors[-1][1])]
        self.offset = SpatialCurve(anchors)
        if old is None:self.offset_blender.set_initial(self.offset,stamp)
        else:self.offset_blender.switch(self.offset,stamp)
        self.stamp=stamp

    def point_at(self,s):
        x,y,z = self.route.point_at(s)
        dx,dy = self.basis(s)
        lateral = self.offset_blender.center(s,self.stamp)
        return x-dy*lateral,y+dx*lateral,z

    def tangent(self,s):
        a,b = self.point_at(max(0.,s-.5)),self.point_at(s+.5)
        dx,dy = b[0]-a[0],b[1]-a[1]
        length = max(1e-6,math.hypot(dx,dy))
        return dx/length,dy/length

    def curvature(self,s):
        a,b = self.tangent(max(0.,s-2.)),self.tangent(s+2.)
        return math.hypot(b[0]-a[0],b[1]-a[1])/4.

    def target(self,p_xy,s_now,v,next_gate,d_gate,last_gate,point_at):
        self.last_s = s_now
        fn = self.point_at if self.route is not None else point_at
        return fn(s_now+self.lookahead(v))[:2]

    def velocity(self,p_xy,target_xy,v_max,arbiter=None,dt=.05,lateral_offset=0.,
                 terminal_position=None,lateral_speed_limit=None):
        command_limit=max(v_max,float(lateral_speed_limit or 0.))
        if terminal_position is not None:
            dx,dy=terminal_position[0]-p_xy[0],terminal_position[1]-p_xy[1]
            distance=math.hypot(dx,dy)
            speed=min(v_max,self.k_pursuit*distance,
                      math.sqrt(2.*self.acceleration*distance))
            vx,vy=dx*speed/max(distance,1e-6),dy*speed/max(distance,1e-6)
        elif self.route is None:
            dx,dy = target_xy[0]-p_xy[0],target_xy[1]-p_xy[1]
            length = max(1e-6,math.hypot(dx,dy))
            vx,vy = dx*v_max/length,dy*v_max/length
        else:
            dx,dy = self.tangent(self.last_s+max(.5,.25*v_max))
            x,y,_ = self.point_at(self.last_s)
            # The obstacle displacement is a reference position, not an extra
            # velocity that the route feedback immediately cancels.
            error = -(x-p_xy[0])*dy+(y-p_xy[1])*dx+lateral_offset
            lateral_limit=max(.65*v_max,float(lateral_speed_limit or 0.))
            lateral = max(-lateral_limit,min(lateral_limit,self.k_pursuit*error))
            vx,vy = v_max*dx-lateral*dy,v_max*dy+lateral*dx
            length = max(command_limit,math.hypot(vx,vy),1e-6)
            vx,vy = vx*command_limit/length,vy*command_limit/length
        px,py = self.previous_velocity
        delta = math.hypot(vx-px,vy-py)
        blend = min(1.,self.acceleration*dt/max(1e-6,delta))
        vx,vy = px+blend*(vx-px),py+blend*(vy-py)
        length = math.hypot(vx,vy)
        if length > command_limit and length > 1e-6:
            vx,vy = vx*command_limit/length,vy*command_limit/length
        self.previous_velocity = vx,vy
        return arbiter.arbitrate((vx,vy)) if arbiter is not None else (vx,vy)
