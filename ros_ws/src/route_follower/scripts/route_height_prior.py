#!/usr/bin/env python3
"""Locally validated NED height prior from the supplied course spline.

Never use raw spline Z as a flight command. Estimate its vertical datum/tilt
from independent gate measurements and reject the model if residuals are large.
"""
import numpy as np
from spatial_curve import SpatialCurve

class RouteHeightPrior:
    def __init__(self, route):
        self.route=route
        self.height=SpatialCurve(zip(route.seg_s,[v[2] for v in route.points]))
        self.valid=False
        self.error=None
        self.coefficients=None

    def fit(self,gates):
        points=[]
        for g in gates:
            if not g.get('trusted',True): continue
            s=float(g['s']);x,y,_=self.route.point_at(s)
            if points and s-points[-1][0]<8.: continue
            points.append((s,x,y,self.height.center(s),float(g['z'])))
        a=np.asarray(points)
        self.valid=False
        if len(points)<8 or np.ptp(a[:,0])<150 or np.ptp(a[:,3])<8: return False
        self.origin=np.mean(a[:,1:3],axis=0)
        xy=a[:,1:3]-self.origin
        if np.linalg.svd(xy,compute_uv=False)[-1]<5.: return False
        X=np.column_stack((xy,np.ones(len(a))))
        residual=a[:,4]+a[:,3]
        weights=np.ones(len(a))
        for _ in range(8):
            # Ridge guards weakly observed tilt on nearly straight stretches.
            regularizer=np.diag([10.,10.,0.])
            coef=np.linalg.solve(X.T@(weights[:,None]*X)+regularizer,X.T@(weights*residual))
            errors=residual-X@coef
            weights=np.minimum(1.,.45/np.maximum(abs(errors),1e-6))
        self.error=float(np.percentile(abs(errors),80))
        if np.linalg.norm(coef[:2])>.12 or self.error>.7: return False
        if np.count_nonzero(abs(errors)<1.)<.8*len(errors): return False
        self.coefficients=coef
        self.information_inverse=np.linalg.pinv(X.T@(weights[:,None]*X))
        inliers=errors[abs(errors)<1.]
        self.residual_sigma=max(.2,float(np.std(inliers,ddof=1)))
        self.valid=True
        return True

    def center(self,s):
        if not self.valid: raise ValueError('Uncalibrated course height')
        x,y,_=self.route.point_at(s)
        return float(-self.height.center(s)+np.dot(np.array([x,y])-self.origin,self.coefficients[:2])+self.coefficients[2])

    def consistent(self,s,z,tolerance=2.5):
        return not self.valid or abs(z-self.center(s))<=tolerance

    def uncertainty(self,s):
        if not self.valid: return float('inf')
        x,y,_=self.route.point_at(s)
        feature=np.array([x-self.origin[0],y-self.origin[1],1.])
        return self.residual_sigma*float(np.sqrt(1.+feature@self.information_inverse@feature))

    def horizon(self,last_evidence,limit=600.,uncertainty_limit=.8):
        horizon=last_evidence
        s=last_evidence+10.
        while s<=min(self.route.total_s,last_evidence+limit):
            if self.uncertainty(s)>uncertainty_limit:break
            horizon=s
            s+=10.
        return horizon
