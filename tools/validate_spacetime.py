#!/usr/bin/env python3
"""Receding-horizon oracle tests of the real offline planner/response/checker.

Known FREE space is supplied explicitly, not inferred from sparse point clouds.
Every step executes only the certified first primitive in the middle empirical
scenario; perfect observations then refresh the planning snapshot. Wall-clock
planner cost is measured separately. This is not a bridge/executor/flight test.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from spacetime_scenarios import cases,advance_oracle
from spacetime_snapshot import write_snapshot
from st_lattice import STLattice,LatticeConfig
from trajectory_collision import TrajectoryCollision


def trace_data(trace):
    # Display is sampled only; all collision checks use the complete trace.
    ids=np.unique(np.linspace(0,len(trace.times)-1,min(100,len(trace.times)),dtype=int))
    return dict(times=trace.times[ids].tolist(),positions=trace.positions[ids].tolist())


def write_preview(rows,path):
    payload=json.dumps(rows,allow_nan=False).replace('<','\\u003c')
    page='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>时空规划离线验证</title>
<style>body{font:16px sans-serif;background:#121722;color:#eef3ff;margin:24px}canvas{background:#202838;width:100%;max-width:1100px}input{width:65%}pre{white-space:pre-wrap}</style>
<h1>时空规划 / 响应 / 完整制动</h1><p>离线理想观测场景：紫色为全部响应情景的规划轨迹，白色为唯一可执行首段，灰色虚线为已认证制动；蓝色目标当前框、橙色框随时间预测。XY / XZ 使用世界 NED。此回放不证明实飞或执行时序。</p>
<input id="step" type="range" min="0" step="1"><span id="label"></span><canvas id="view" width="1100" height="640"></canvas><pre id="info"></pre>
<script>const rows=PAYLOAD,s=document.getElementById('step'),c=document.getElementById('view'),ctx=c.getContext('2d');s.max=rows.length-1;s.value=0;
function draw(){const r=rows[+s.value];ctx.clearRect(0,0,1100,640);document.getElementById('label').textContent=`${r.case} / step ${r.step} / ${r.result.reason}`;
for(const [axis,offset,label] of [[1,0,'XY / NED'],[2,320,'XZ / NED']]){const p=q=>[100+(q[0]-r.position[0])*40,offset+160+(q[axis]-r.position[axis])*40];ctx.fillStyle='white';ctx.fillText(label,10,offset+18);
for(const [key,color,dashed] of [['trace','#bb80ff',false],['backup','#9ba8bc',true],['prefix','#ffffff',false]]){const trace=r[key];if(!trace)continue;ctx.strokeStyle=color;ctx.setLineDash(dashed?[5,4]:[]);for(let k=0;k<trace.positions[0].length;k++){ctx.beginPath();trace.positions.forEach((row,i)=>{const [x,y]=p(row[k]);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);});ctx.stroke();}ctx.setLineDash([]);}
for(const obstacle of r.obstacles){for(const dt of [0,1,2,3,4]){const center=obstacle.position.map((v,i)=>v+dt*obstacle.velocity[i]);const [x,y]=p(center);ctx.strokeStyle=dt?'#ffae42':'#5e9eff';ctx.setLineDash(dt?[3,3]:[]);ctx.strokeRect(x-obstacle.size[0]*20,y-obstacle.size[axis]*20,obstacle.size[0]*40,obstacle.size[axis]*40);ctx.fillStyle=ctx.strokeStyle;ctx.fillText(`${obstacle.track_id} +${dt}s`,x+2,y-3);}ctx.setLineDash([]);}}
document.getElementById('info').textContent=JSON.stringify({case:r.case,step:r.step,position:r.position,result:r.result},null,2);}
s.oninput=draw;draw();</script></html>'''
    path.write_text(page.replace('PAYLOAD',payload))


def validate(out,budget=.35,max_steps=80,selected=None):
    if out.exists():raise ValueError('Use a new output directory')
    if not np.isfinite(budget) or budget<=0 or not isinstance(max_steps,int) or max_steps<1:raise ValueError('Invalid validation budget')
    all_cases=cases();names=list(all_cases) if selected is None else list(selected)
    if any(name not in all_cases for name in names):raise ValueError('Unknown validation case')
    out.mkdir(parents=True);rows=[];summaries={}
    for name in names:
        snapshot,parameters=all_cases[name]
        # B/C/D isolate temporal avoidance without lateral alternatives. A/F
        # retain side actions. All cases retain full body/dynamics constraints.
        config=LatticeConfig(budget=budget,forward_distance=8.,height_step=0.,max_expansions=1000,
                             lateral_rate=0. if name[0] in 'BCD' else 1.5)
        planner=STLattice(parameters,config);start=snapshot.pose_stamp;durations=[];waits=0;failures=[];goal=8.1
        case_out=out/name;case_out.mkdir();write_snapshot(snapshot,parameters,case_out/'initial.json',
            'Oracle-known FREE space and controlled CV tracks; not LiDAR or flight evidence')
        counts={};max_lateral=0.
        with (case_out/'steps.jsonl').open('w') as log:
            for step in range(max_steps):
                result=planner.plan(snapshot);durations.append(result.elapsed_ms)
                counts[result.reason]=counts.get(result.reason,0)+1
                row=dict(case=name,step=step,stamp=snapshot.pose_stamp,position=snapshot.response_state.position[0].tolist(),
                         result=result.summary(),obstacles=[dict(track_id=o.track_id,position=o.position.tolist(),
                             velocity=o.velocity.tolist(),size=o.bbox_size.tolist()) for o in snapshot.obstacles],
                         trace=None,backup=None,prefix=None)
                if result.trajectory is None:failures.append(result.reason);log.write(json.dumps(row)+'\n');rows.append(row);break
                trajectory=result.trajectory;checker=TrajectoryCollision(snapshot,planner.collision_config)
                if not checker.check(trajectory.trace).safe or not checker.check(trajectory.backup).safe:raise AssertionError('Uncertified plan returned')
                previous=snapshot;snapshot,prefix=advance_oracle(snapshot,parameters,trajectory)
                if not checker.check(prefix).safe:raise AssertionError('Uncertified executed prefix')
                np.testing.assert_allclose(prefix.positions[-1],trajectory.backup.positions[0],atol=1e-9)
                waits+=int(trajectory.primitives[0].name=='WAIT');max_lateral=max(max_lateral,float(abs(prefix.positions[:,:,1]).max()))
                row.update(trace=trace_data(trajectory.trace),backup=trace_data(trajectory.backup),prefix=trace_data(prefix))
                log.write(json.dumps(row,allow_nan=False)+'\n');rows.append(row)
                if snapshot.response_state.position[0,0]>=goal:break
        passed=bool(snapshot.response_state.position[0,0]>=goal and not failures)
        summary=dict(passed=passed,steps=len(durations),simulation_seconds=snapshot.pose_stamp-start,
            end_position=snapshot.response_state.position[0].tolist(),wait_prefixes=waits,max_lateral=max_lateral,
            plan_reasons=counts,failures=failures,planning_ms=dict(p50=float(np.median(durations)),
                p95=float(np.percentile(durations,95)),maximum=float(np.max(durations))),planner=asdict(config))
        (case_out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');summaries[name]=summary
        print(name,json.dumps(summary),flush=True)
    source=Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/scripts'
    paths=[source/p for p in ('trajectory_types.py','route_coordinates.py','response_rollout.py','velocity_response.py',
                             'trajectory_collision.py','st_lattice.py','dynamic_tracker.py','local_occupancy.py')]
    paths.extend([Path(__file__),Path(__file__).with_name('spacetime_scenarios.py'),Path(__file__).with_name('spacetime_snapshot.py')])
    result=dict(scope='Oracle receding-horizon response simulation, not flight, real occupancy, bridge or scheduling validation',
        cases=summaries,all_passed=all(s['passed'] for s in summaries.values()),wall_time_budget_seconds=budget,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');write_preview(rows,out/'preview.html')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--budget',type=float,default=.35);parser.add_argument('--max-steps',type=int,default=80)
    parser.add_argument('--cases',nargs='+');args=parser.parse_args()
    result=validate(args.out,args.budget,args.max_steps,args.cases)
    if not result['all_passed']:raise SystemExit('At least one oracle case did not reach the target')


if __name__=='__main__':main()
