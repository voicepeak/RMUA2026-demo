#!/usr/bin/env python3
"""Approach a stage trigger with enough height to depart on the next leg."""
import math


def approach_target(position, goal, hover_height=1.5, standoff=9.):
    height=float(goal[2])-float(position[2])
    ready=hover_height-.15<=height<=3.5
    if ready:return tuple(goal[:2]),True
    dx,dy=float(position[0])-goal[0],float(position[1])-goal[1]
    distance=math.hypot(dx,dy)
    if distance<1e-6:return tuple(position[:2]),False
    return (goal[0]+standoff*dx/distance,goal[1]+standoff*dy/distance),False
