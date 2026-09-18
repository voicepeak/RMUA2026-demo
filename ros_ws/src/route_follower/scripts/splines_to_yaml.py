#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把官方 Splines.txt 转成 route_follower 使用的 route YAML。

路线 1->3 = spline 0 (起点->枢纽) + reverse(spline 2) (枢纽->终点)。
生成:
  route_1_3     : 完整 起点->枢纽->终点
  route_road1   : 仅 spline 0 (起点->枢纽), 用于 Test 1
"""

import os

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
    route = road1 + road3_rev
    write_yaml(OUT, route, road1)
    print("spline0 n=%d len=%.1f m" % (len(road1), length(road1)))
    print("spline2 reversed n=%d len=%.1f m" % (len(road3_rev), length(road3_rev)))
    print("route_1_3 n=%d total len=%.1f m" % (len(route), length(route)))
    print("wrote", os.path.normpath(OUT))


if __name__ == "__main__":
    main()
