#!/usr/bin/env python3
"""Longitudinal scheduling + one-shot lateral/vertical + commitment.

The dynamic problem is planned in route coordinates (s, v, t). Static and
dynamic occupancy enter as a follow/wait constraint; a one-shot lateral or
vertical decision is only produced when the corridor itself is blocked. The
planner never certifies a full stop envelope; emergency braking stays with
the execution layer (`emergency_stop_distance`).

Pure functions, no ROS, no live controller state.
"""
import math
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class LongitudinalConfig:
    horizon: float = 4.0
    dt: float = .25
    # 1 m/s resolution so a 4 m/s^2 / 6 m/s^2 limit can change speed
    # every dt=0.25s step (a 2 m/s grid cannot brake 1.5 m/s per step).
    speeds: tuple = (0.,1.,2.,3.,4.,5.,6.,7.,8.,9.,10.,11.,12.)
    a_accel: float = 4.0
    a_brake: float = 6.0
    follow_gap: float = 1.5
    follow_time: float = .3
    plan_margin: float = .6
    absolute_margin: float = .45
    braking_response: float = .25
    braking_decel: float = 6.0
    commitment: float = .6
    switch_improvement: float = .15
    lateral_offsets: tuple = (-1., 1., -1.5, 1.5, -2., 2.)
    lateral_gain: float = 3.0
    lateral_min_free: float = 6.0
    lookahead: float = 16.0
    sample_step: float = .25
    car_half_default: float = .8
    lateral_gate: float = .5


def emergency_stop_distance(speed, cfg, response=None, decel=None):
    """Reaction travel + braking + absolute margin (the only hard predicate)."""
    speed = max(0., float(speed))
    response = cfg.braking_response if response is None else float(response)
    decel = cfg.braking_decel if decel is None else max(.1, float(decel))
    return speed*response + speed*speed/(2.*decel) + cfg.absolute_margin


def side_at(tangent):
    return np.array([-tangent[1], tangent[0]], dtype=float)


def project_cars(tracks, project_xy, center_at, tangent_at, now, cfg,
                 state_store=None):
    """Project observed car boxes into route coordinates."""
    cars = []
    for track in tracks or []:
        world = np.asarray(track.get('world', track.get('center', [])), dtype=float)
        if world.shape != (3,) or not np.all(np.isfinite(world)):
            continue
        stamp = float(track.get('stamp', now))
        age = max(0., now-stamp)
        if age > 2.:
            continue
        s = float(project_xy(world[0], world[1]))
        center = np.asarray(center_at(s), dtype=float)
        tangent = np.asarray(tangent_at(s), dtype=float)
        y = float((world-center)[:2]@side_at(tangent))
        half_y = float(track.get('half_width', track.get('half', cfg.car_half_default)))
        half_s = float(track.get('half_length', max(half_y, cfg.car_half_default)))
        velocity = np.asarray(track.get('velocity', [0., 0., 0.]), dtype=float)
        if state_store is not None:
            measured = state_store.velocity(track, now)
            if measured is not None:
                velocity = measured
        v_s = float(velocity[:2]@np.asarray(tangent)[:2]) if np.all(np.isfinite(velocity)) else 0.
        uncertainty = min(5., max(.2, float(track.get('uncertainty', .35))))
        half_z = float(track.get('half_height', half_y))
        cars.append(dict(s=s, y=y, z=float(world[2]), v_s=v_s, half_s=half_s,
                         half_y=half_y, half_z=half_z,
                         uncertainty=uncertainty, age=age, id=track.get('id'),
                         world=world))
    return cars


def _lane_overlap(car, drone_y, cfg):
    return abs(drone_y-car['y']) < car['half_y']+cfg.lateral_gate


def _edge_blocked(s0, s1, t0, t1, obstacles, drone_y, cfg):
    """Return the blocking obstacle over the edge, or None."""
    if s1 <= s0:
        return None
    span = max(.05, t1-t0)
    samples = max(2, int(math.ceil(span/max(.05, cfg.dt*.5))))
    for car in obstacles:
        if not _lane_overlap(car, drone_y, cfg):
            continue
        gap = car['half_s']+cfg.follow_gap+car['uncertainty']
        for i in range(samples+1):
            t = t0+span*i/samples
            uav = s0+(s1-s0)*i/samples
            if abs(uav-(car['s']+car['v_s']*t)) < gap:
                return car
    return None


def plan_longitudinal(s_now, v_now, cars, cfg, cruise, free_soft, drone_y=0.,
                      static_stop=True):
    """DP over (time, speed) with follow/wait and a static soft stop.

    Returns dict(target_speed, wait, blocked, profile, blocked_car_id,
    static_stop_s).
    """
    speeds = sorted(set([0.]+[float(v) for v in cfg.speeds if v <= cruise+1e-9]+[float(cruise)]))
    steps = max(1, int(round(cfg.horizon/cfg.dt)))
    obstacles = [car for car in cars if _lane_overlap(car, drone_y, cfg)]
    static_s = None
    if static_stop and free_soft is not None and math.isfinite(free_soft):
        static_s = s_now+max(0., float(free_soft))
        obstacles.append(dict(s=static_s, y=drone_y, v_s=0., half_s=0.,
                              half_y=1e3, uncertainty=0., id='static'))
    data = {}
    # First step starts from the measured speed, not a quantized level.
    for k2, v2 in enumerate(speeds):
        if v2 > v_now+cfg.a_accel*cfg.dt+1e-9 or v2 < v_now-cfg.a_brake*cfg.dt-1e-9:
            continue
        s2 = s_now+.5*(v_now+v2)*cfg.dt
        hit = _edge_blocked(s_now, s2, 0., cfg.dt, obstacles, drone_y, cfg)
        if hit is not None:
            continue
        data[(1, k2)] = (s2, None, v2)
    for step in range(1, steps):
        t0 = step*cfg.dt
        nxt = {}
        for (st, k), (s, pred, act) in data.items():
            if st != step:
                continue
            v = speeds[k]
            for k2, v2 in enumerate(speeds):
                if v2 > v+cfg.a_accel*cfg.dt+1e-9 or v2 < v-cfg.a_brake*cfg.dt-1e-9:
                    continue
                s2 = s+.5*(v+v2)*cfg.dt
                hit = _edge_blocked(s, s2, t0, t0+cfg.dt, obstacles, drone_y, cfg)
                if hit is not None:
                    continue
                key = (step+1, k2)
                candidate = (s2, (st, k), v2)
                if key not in nxt or candidate[0] > nxt[key][0]+1e-9:
                    nxt[key] = candidate
        if not nxt:
            break
        data.update(nxt)
    def stoppable(node_s, v):
        response = cfg.braking_response
        t_stop = response+v/(2.*cfg.braking_decel) if v > 1e-6 else 0.
        d_stop = emergency_stop_distance(v, cfg)
        for car in obstacles:
            car_s = car['s']+car['v_s']*t_stop
            if node_s+d_stop > car_s-car['half_s']-car['uncertainty']:
                return False
        return True

    final = {key: node for key, node in data.items()
             if key[0] >= 1 and stoppable(node[0], speeds[key[1]])}
    if not final:
        # No discrete profile proves a stop; command the legal maximum brake
        # and let the emergency layer handle the residual. Never keep driving.
        target = 0.
        nearest = min((car for car in obstacles if car['id'] != 'static'),
                      key=lambda car: abs(car['s']-s_now), default=None)
        return dict(target_speed=float(target), wait=bool(target < .01),
                    blocked=(nearest or {}).get('id', 'static_or_no_stop'),
                    profile=[float(target)], blocked_car_id=(nearest or {}).get('id'),
                    static_stop_s=static_s)
    key, (s, pred, act) = max(final.items(), key=lambda item: item[1][0])
    chain = []
    while key[0] >= 1:
        node = data[key]
        chain.append(node[2])
        if node[1] is None:
            break
        key = node[1]
    chain.reverse()
    target = float(chain[0])
    blocked = None if target > .01 else 'static_or_car'
    return dict(target_speed=target, wait=bool(target < .01), blocked=blocked,
                profile=chain, blocked_car_id=None, static_stop_s=static_s)


class CarVelocityStore:
    """Measured world velocity from consecutive detections of the same car."""

    def __init__(self, smoothing=.5):
        self.items = []
        self.smoothing = float(smoothing)

    def velocity(self, track, now):
        world = np.asarray(track.get('world', []), dtype=float)
        if world.shape != (3,):
            return None
        best = None
        for item in self.items:
            distance = float(np.linalg.norm(world-item['world']))
            if best is None or distance < best[0]:
                best = (distance, item)
        if best is not None and best[0] < 3.:
            item = best[1]
            dt = now-item['stamp']
            if .02 < dt < 1.:
                measured = (world-item['world'])/dt
                item['velocity'] = (1.-self.smoothing)*item['velocity']+self.smoothing*measured
                item['world'] = world
                item['stamp'] = now
                return item['velocity'].copy()
        self.items.append(dict(world=world, stamp=now, velocity=np.zeros(3)))
        self.items = self.items[-12:]
        return None


def car_box_conflict(cars, s_now, y_off, z, stop_distance, cfg):
    """A tracked car box overlaps the flown corridor within the hard margin.

    Independent of whether the raw lidar returns include the car roof.
    """
    for car in cars:
        ds = car['s']-s_now
        if ds < -car['half_s']-2. or ds > stop_distance+car['half_s']+2.:
            continue
        if abs(y_off-car['y']) > car['half_y']+cfg.lateral_gate:
            continue
        if abs(z-car['z']) > car['half_z']+cfg.absolute_margin:
            continue
        if ds-car['half_s'] < stop_distance:
            return car
    return None


def choose_one_shot(current_free, offsets, cfg, vertical_ok=False):
    """Lateral offset when the current corridor is blocked.

    `offsets` maps y_off to (soft_free, hard_free, min_clearance) of the
    shifted world path. Prefer the largest free distance with the smallest
    |y|; vertical only when no lateral qualifies.
    """
    best = None
    for y in cfg.lateral_offsets:
        if y not in offsets:
            continue
        soft, hard, _ = offsets[y]
        if soft < cfg.lateral_min_free or soft < current_free+cfg.lateral_gain:
            continue
        if best is None or soft > best[1]+1e-9 or (
                abs(soft-best[1]) <= 1e-9 and abs(y) < abs(best[0])):
            best = (y, soft)
    if best is not None:
        return dict(kind='lateral', y=float(best[0]), z=0., reason='corridor_blocked')
    if vertical_ok:
        return dict(kind='vertical', y=0., z=1., reason='vertical_override')
    return dict(kind=None, y=0., z=0., reason='no_certified_alternative')


def commitment_should_switch(old, new, now, until, cfg):
    """Keep the committed decision unless it is unsafe or clearly worse."""
    if old is None or until is None or now >= until:
        return True
    if new.get('unsafe'):
        return True
    old_cost = float(old.get('cost', 0.))
    new_cost = float(new.get('cost', 0.))
    return new_cost < old_cost*(1.-cfg.switch_improvement)
