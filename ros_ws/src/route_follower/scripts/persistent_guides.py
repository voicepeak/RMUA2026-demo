#!/usr/bin/env python3
"""Persist stable height evidence independently of detector visibility and hard status."""
import math
import numpy as np

class PersistentGuides:
    def __init__(self):
        self.observations = {}
        self.guides = []

    def update(self, observations, now, s_now, project, min_support=6):
        for g in observations:
            if not g.get('valid',True): continue
            if not all(math.isfinite(float(g.get(k,float('nan')))) for k in ('x','y','z','last_seen')):
                continue
            if not 0. <= now-g['last_seen'] <= 1.5: continue
            if g.get('support',0)<min_support or g.get('sigma_z',9)>1.2: continue
            if (g.get('confidence') or 0.) < .65: continue
            s = project(g)
            old = self.observations.get(g['id'])
            if old is not None and old[0] <= s_now+3.: continue
            if s > s_now+3.:
                self.observations[g['id']] = (s,float(g['z']),g['last_seen'])
        # Passed anchors remain part of the spatial curve. They do not expire
        # with the detector track: erasing them re-flattens a previously mapped hill.
        points = sorted((s,z) for s,z,stamp in self.observations.values())
        merged,cluster = [],[]
        for s,z in points:
            if cluster and s-cluster[0][0]>7.:
                merged.append(tuple(float(v) for v in np.median(cluster,axis=0)))
                cluster=[]
            cluster.append((s,z))
        if cluster:
            merged.append(tuple(float(v) for v in np.median(cluster,axis=0)))
        changed = (len(merged)!=len(self.guides) or
                   any(abs(a[0]-b[0])>1. or abs(a[1]-b[1])>.3 for a,b in zip(merged,self.guides)))
        if changed: self.guides = merged
        return changed

    @property
    def horizon(self):
        return self.guides[-1][0] if self.guides else None
