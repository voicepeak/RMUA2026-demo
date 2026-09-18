#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在参考路径上按累计弧长生成占位 Gate (真实门坐标需用 route_recorder 采集替换)。

输出 gates YAML: 每个 gate 有 center(x,y,z)、法向(nx,ny,nz)=路径切向、
approach_distance、speed。z 使用 route 坐标系; 节点运行时会加 z_offset。
"""

import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTE = os.path.join(HERE, "..", "config", "route_1_3.yaml")
OUT = os.path.join(HERE, "..", "config", "gates_1_3.yaml")

# 占位门沿路径的累计距离 (m)
GATE_DISTANCES = [120.0, 300.0, 480.0, 700.0, 950.0, 1200.0]
APPROACH = 4.0
SPEED = 1.5


def point_at(pts, dist):
    acc = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        seg = math.dist(a, b)
        if acc + seg >= dist:
            r = (dist - acc) / seg if seg > 1e-9 else 0.0
            x = a[0] + r * (b[0] - a[0])
            y = a[1] + r * (b[1] - a[1])
            z = a[2] + r * (b[2] - a[2])
            # 切向
            tx, ty = b[0] - a[0], b[1] - a[1]
            L = math.hypot(tx, ty)
            return (x, y, z), (tx / L, ty / L, 0.0) if L > 1e-9 else (1.0, 0.0, 0.0)
        acc += seg
    a, b = pts[-2], pts[-1]
    tx, ty = b[0] - a[0], b[1] - a[1]
    L = math.hypot(tx, ty) or 1.0
    return tuple(b), (tx / L, ty / L, 0.0)


def main():
    with open(ROUTE) as f:
        routes = yaml.safe_load(f)["routes"]
    route = routes["route_1_3"]

    gates = []
    for i, d in enumerate(GATE_DISTANCES):
        (x, y, z), (nx, ny, nz) = point_at(route, d)
        gates.append({
            "id": i,
            "x": round(x, 3), "y": round(y, 3), "z": round(z, 3),
            "nx": round(nx, 4), "ny": round(ny, 4), "nz": nz,
            "approach_distance": APPROACH,
            "speed": SPEED,
            "valid": False,
        })

    with open(OUT, "w") as f:
        f.write("# PLACEHOLDER gates generated on route_1_3 (replace with recorded gates)\n")
        yaml.safe_dump({"gates": gates}, f, default_flow_style=False, sort_keys=False)
    print("wrote %d gates to %s" % (len(gates), os.path.normpath(OUT)))
    for g in gates:
        print("  gate %d  (%.1f, %.1f, %.1f) n=(%.2f, %.2f)" %
              (g["id"], g["x"], g["y"], g["z"], g["nx"], g["ny"]))


if __name__ == "__main__":
    main()
