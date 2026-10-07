#!/usr/bin/env python3
"""Measure a real Python scheduler under isolated planner-process faults.

Oracle stationary observations and an explicitly ASSUMED expiring bridge.
Records numerical decisions only; never creates a ROS command publisher.
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time
import numpy as np
from spacetime_scenarios import known_snapshot
from spacetime_navigation import PlannerProcess
from st_lattice import LatticeConfig
from trajectory_executor import TrajectoryExecutor


def validate(out,duration=1.):
    out=Path(out);out.mkdir(parents=True,exist_ok=False);summary=[]
    for fault in ('delay_0.2','delay_0.6','delay_1.2','delay_2.2','hang','exit','pose_loss','cloud_loss','reset'):
        snapshot,parameters=known_snapshot()
        parameters=replace(parameters,scenarios=tuple(s for s in parameters.scenarios for _ in range(3)),
                           periods=(.05,.16,.4)*3,integration_step=.05)
        old=snapshot.response_state
        values={key:np.repeat(getattr(old,key),3,axis=0) for key in
                ('position','velocity','previous','applied','last_control','next_control','effective')}
        snapshot=replace(snapshot,response_state=replace(old,model_key=parameters.model_key,**values))
        executor=TrajectoryExecutor(parameters);executor.reset(snapshot.epoch)
        worker=PlannerProcess(parameters,LatticeConfig(budget=.1,forward_distance=2.,lateral_rate=0.,height_step=0.))
        rows=[]
        try:
            warm_deadline=time.monotonic()+10.
            while not worker._ready():
                if time.monotonic()>warm_deadline:raise RuntimeError('Planner worker did not warm up')
                time.sleep(.01)
            delay=float(fault.split('_')[1]) if fault.startswith('delay_') else 3600. if fault in ('hang','exit') else 0.
            worker.submit(snapshot,delay)
            if fault=='exit':worker.process.terminate();worker.process.join(timeout=.1)
            started=time.monotonic();due=started;stale_arrival=started
            while time.monotonic()-started<duration:
                now=time.monotonic()
                if now<due:time.sleep(min(.01,due-now));continue
                due=max(due+.05,now+.05);elapsed=now-started;stamp=100.+elapsed
                state=replace(snapshot.response_state,stamp=stamp,next_control=np.full(9,stamp),next_physics=stamp)
                mapping=replace(snapshot.occupancy,evaluated_at=stamp,stamp=stamp,
                                valid=not (fault=='cloud_loss' and elapsed>=.2))
                current=replace(snapshot,pose_stamp=stamp,response_state=state,occupancy=mapping,
                                epoch=1 if fault=='reset' and elapsed>=.2 else 0)
                reply=worker.poll()
                if reply is not None and reply.result.trajectory is not None:
                    executor.accept(reply.result.trajectory,current,reply.collision_key,reply.submitted_monotonic)
                arrival=stale_arrival if fault=='pose_loss' and elapsed>=.2 else now
                decision=executor.tick(current,now,arrival,bridge_verified=True)
                published=time.monotonic()
                rows.append(dict(elapsed=elapsed,published_monotonic=published,compute_ms=1000.*(published-now),
                    mode=decision.mode,reason=decision.reason,certified=decision.certified,
                    command=decision.command.tolist(),planner_reason=worker.last_reason))
        finally:worker.close()
        intervals=np.diff([row['published_monotonic'] for row in rows])
        summary.append(dict(fault=fault,decisions=len(rows),interval_p95_ms=float(np.percentile(intervals,95)*1000.),
                            interval_max_ms=float(intervals.max()*1000.),
                            compute_p95_ms=float(np.percentile([row['compute_ms'] for row in rows],95)),
                            reasons=sorted({row['reason'] for row in rows}),planner_reason=worker.last_reason,
                            passed=all(row['mode']!='NORMAL' for row in rows if row['reason']!='PASS') and
                                   len(rows)>=duration*15 and float(intervals.max())<.15))
        (out/(fault+'.jsonl')).write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in rows))
    report=dict(scope='Oracle numerical scheduler; assumed bridge expiry, NO ROS publication/flight',
                period=.05,scenarios=9,cases=summary,passed=all(row['passed'] for row in summary))
    (out/'summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2));return report


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=1.);args=parser.parse_args()
    if args.duration<.6:parser.error('Duration must be at least .6s')
    if not validate(args.out,args.duration)['passed']:raise SystemExit(1)
