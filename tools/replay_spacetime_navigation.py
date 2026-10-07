#!/usr/bin/env python3
"""Replay one complete frozen planning snapshot, including response history.

Old hit-point archives alone cannot recover free space or actuator history.
This tool never guesses either, and never publishes flight commands.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml
from spacetime_snapshot import read_snapshot
from st_lattice import STLattice,LatticeConfig
from trajectory_collision import CollisionConfig,TrajectoryCollision
from validate_spacetime import trace_data,write_preview


def replay(path,out,config_path=None,budget=None):
    snapshot,parameters,provenance=read_snapshot(path)
    config_path=Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/config/spacetime_planner.yaml' if config_path is None else config_path
    options=yaml.safe_load(config_path.read_text());planning=options['planner'].copy()
    if budget is not None:planning['budget']=budget
    config=LatticeConfig(**planning);collision=CollisionConfig(**options['collision'])
    if out.exists():raise ValueError('Use a new output directory')
    planner=STLattice(parameters,config,collision);result=planner.plan(snapshot);out.mkdir(parents=True)
    row=dict(case='Frozen snapshot',step=0,stamp=snapshot.pose_stamp,position=snapshot.response_state.position[0].tolist(),
        result=result.summary(),obstacles=[dict(track_id=o.track_id,position=o.position.tolist(),velocity=o.velocity.tolist(),size=o.bbox_size.tolist()) for o in snapshot.obstacles],
        trace=None,backup=None,prefix=None)
    summary=dict(result=result.summary(),provenance=provenance,scope='Frozen offline plan replay; no flight or scheduling evidence',
        planner=asdict(config),collision=asdict(collision),
        input_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (path,path.with_suffix('.npz'),config_path)})
    if result.trajectory is not None:
        trajectory=result.trajectory;checker=TrajectoryCollision(snapshot,collision)
        if not checker.check(trajectory.trace).safe or not checker.check(trajectory.backup).safe:raise AssertionError('Returned plan is not certified')
        trace=trajectory.trace;backup=trajectory.backup
        np.savez_compressed(out/'trajectory.npz',times=trace.times,positions=trace.positions,velocities=trace.velocities,
            nominal_commands=trace.nominal_commands,applied_commands=trace.applied_commands,
            backup_times=backup.times,backup_positions=backup.positions,backup_velocities=backup.velocities,
            backup_nominal_commands=backup.nominal_commands,backup_applied_commands=backup.applied_commands)
        summary.update(start_stamp=trajectory.start_stamp,valid_until=trajectory.valid_until,
            primitives=[dict(asdict(p),target=p.target.tolist()) for p in trajectory.primitives],
            profiles=[{key:getattr(p,key) for key in ('lateral','height','height_gain','lateral_gain','lateral_speed_limit','lateral_damping')} for p in trajectory.profiles])
        row.update(trace=trace_data(trace),backup=trace_data(backup),prefix=trace_data(trace))
        # Only highlight the actual certified prefix, not the entire search.
        ids=trace.times<=trajectory.valid_until+1e-8
        row['prefix']=dict(times=trace.times[ids].tolist(),positions=trace.positions[ids].tolist())
    (out/'plan.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n');write_preview([row],out/'preview.html')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--snapshot',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True);parser.add_argument('--config',type=Path);parser.add_argument('--budget',type=float)
    args=parser.parse_args();summary=replay(args.snapshot,args.out,args.config,args.budget);print(json.dumps(summary['result']))


if __name__=='__main__':main()
