"""Check observed gates before either hard or soft height evidence is stored.

The supplied road's raw Z has a different datum. Only a calibrated, bounded
RouteHeightPrior may reject a gate on height; XY alone cannot identify a layer.
"""
import numpy as np


def gate_route_membership(gate, route, height_prior, s_now, lateral_limit=3.):
    world=np.array([gate.get(k,np.nan) for k in ('x','y','z')],dtype=float)
    if not np.all(np.isfinite(world)):return False,'NONFINITE'
    index,t,distance,_=route.project(world)
    s=route.seg_s[index]+t*route.seg_len[index]
    if distance>lateral_limit:return False,'OTHER_ROAD'
    if s<s_now-5.:return False,'BEHIND'
    calibrated=(height_prior.valid and height_prior.uncertainty(s)<=.8)
    if calibrated:
        sigma=float(gate.get('sigma_z',.6))
        if not np.isfinite(sigma) or sigma<0.:return False,'NONFINITE'
        tolerance=2.5+height_prior.uncertainty(s)+min(sigma,.6)
        if abs(world[2]-height_prior.center(s))>tolerance:
            return False,'OTHER_LAYER'
    # A route-derived bbox normal is not measured plane evidence.
    if gate.get('geometry_valid',False):
        normal=np.array([gate.get(k,np.nan) for k in ('nx','ny','nz')],dtype=float)
        norm=np.linalg.norm(normal)
        if np.all(np.isfinite(normal)) and norm>1e-6:
            tangent=np.array(route.tangent(s),dtype=float)
            if calibrated:
                tangent[2]=(height_prior.center(s+.5)-height_prior.center(s-.5))
            tangent/=max(np.linalg.norm(tangent),1e-9)
            if abs(normal@tangent)/norm<.5:return False,'OTHER_DIRECTION'
    return True,'ACCEPTED'
