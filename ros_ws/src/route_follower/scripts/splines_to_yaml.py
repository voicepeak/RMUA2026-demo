#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把官方 Splines.txt 转成 route_follower 使用的 route YAML。

路线 1->3 = spline 0 (起点->枢纽) + reverse(spline 2) (枢纽->终点)。
生成:
  route_1_3     : 完整 起点->枢纽->终点
  route_road1   : 仅 spline 0 (起点->枢纽), 用于 Test 1
"""

import os
import math

HERE = os.path.dirname(os.path.abspath(__file__))
SPLINES = os.path.join(HERE, "..", "config", "splines.txt")
OUT = os.path.join(HERE, "..", "config", "route_1_3.yaml")


def load_splines(path):
    splines = []
    with open(path) as f:
        for line in f:
            vals = [float(v) for v in line.split()]
            pts = [tuple(round(v, 4) for v in vals[i:i + 3])
                   for i in range(0, len(vals) - 2, 3)]
            if len(pts) >= 2:
                splines.append(pts)
    return splines


def length(pts):
    total = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        total += sum((b[k] - a[k]) ** 2 for k in range(3)) ** 0.5
    return total


def connect_routes(incoming, outgoing, clearance=25., spacing=3.):
    """Continue through each hub doorway before turning toward the next road.

    A straight chord between road endpoints cuts the entrance wall. Preserve
    the approach/departure tangents for clearance metres, then join them C1.
    """
    a,b=incoming[-1],outgoing[0]
    def unit(p,q):
        dx,dy=q[0]-p[0],q[1]-p[1]
        length=math.hypot(dx,dy)
        return dx/length,dy/length
    d0=unit(incoming[-2],a);d1=unit(b,outgoing[1])
    gap=math.hypot(b[0]-a[0],b[1]-a[1])
    if gap<2.*spacing:return list(incoming)+list(outgoing)
    extension=min(clearance,gap*.2)
    c=(a[0]+extension*d0[0],a[1]+extension*d0[1])
    d=(b[0]-extension*d1[0],b[1]-extension*d1[1])
    handle=math.hypot(d[0]-c[0],d[1]-c[1])*.4
    p1=(c[0]+handle*d0[0],c[1]+handle*d0[1])
    p2=(d[0]-handle*d1[0],d[1]-handle*d1[1])
    xy=[a[:2]]
    n=max(2,int(math.ceil(extension/spacing)))
    xy += [(a[0]+extension*i/n*d0[0],a[1]+extension*i/n*d0[1]) for i in range(1,n+1)]
    n=max(8,int(math.ceil(gap/spacing)))
    for i in range(1,n+1):
        t=i/n;u=1.-t
        xy.append(tuple(u**3*c[j]+3*u*u*t*p1[j]+3*u*t*t*p2[j]+t**3*d[j] for j in (0,1)))
    n=max(2,int(math.ceil(extension/spacing)))
    xy += [(d[0]+extension*i/n*d1[0],d[1]+extension*i/n*d1[1]) for i in range(1,n+1)]
    arc=[0.]
    for before,after in zip(xy,xy[1:]):arc.append(arc[-1]+math.hypot(after[0]-before[0],after[1]-before[1]))
    bridge=[(x,y,a[2]+(b[2]-a[2])*(3*(s/arc[-1])**2-2*(s/arc[-1])**3))
            for (x,y),s in zip(xy[1:-1],arc[1:-1])]
    return list(incoming)+bridge+list(outgoing)


def write_yaml(path, route_1_3, route_road1):
    with open(path, "w") as f:
        f.write("# 由官方 Splines.txt 自动生成 (spline0 + reverse(spline2))\n")
        f.write("routes:\n")
        for name, pts in (("route_1_3", route_1_3), ("route_road1", route_road1)):
            f.write("  %s:\n" % name)
            for p in pts:
                f.write("    - [%.4f, %.4f, %.4f]\n" % p)


def main():
    splines = load_splines(SPLINES)
    road1 = splines[0]
    road3_rev = list(reversed(splines[2]))
    route = connect_routes(road1, road3_rev)
    write_yaml(OUT, route, road1)
    print("spline0 n=%d len=%.1f m" % (len(road1), length(road1)))
    print("spline2 reversed n=%d len=%.1f m" % (len(road3_rev), length(road3_rev)))
    print("route_1_3 n=%d total len=%.1f m" % (len(route), length(route)))
    print("wrote", os.path.normpath(OUT))


if __name__ == "__main__":
    main()
