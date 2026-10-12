#!/usr/bin/env python3
"""Approach a stage trigger with enough height to depart on the next leg."""
import math


def terminal_reference(route_xy,goal_xy,end_s,blend_distance=8.):
    """Keep the final road/gates before joining the published marker.

    Pointing directly at a slightly offset marker from 25 m away changes
    the corridor through the last frame. Join only over the final metres.
    """
    endpoint=route_xy(end_s)[:2]
    offset=(float(goal_xy[0])-endpoint[0],float(goal_xy[1])-endpoint[1])
    def point(station):
        weight=max(0.,min(1.,(station-end_s+blend_distance)/blend_distance))
        x,y=route_xy(station)[:2]
        return x+weight*offset[0],y+weight*offset[1]
    return point


def approach_target(position, goal, hover_height=1.5, standoff=9.):
    height=float(goal[2])-float(position[2])
    ready=hover_height-.15<=height<=3.5
    if ready:return tuple(goal[:2]),True
    dx,dy=float(position[0])-goal[0],float(position[1])-goal[1]
    distance=math.hypot(dx,dy)
    if distance<1e-6:return tuple(position[:2]),False
    return (goal[0]+standoff*dx/distance,goal[1]+standoff*dy/distance),False
