#!/usr/bin/env python3
"""Summarize independently recorded commands and bounded probe phases."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();run=args.run
    phase_file=run/'phases.jsonl';trace_file=run/'trace/control.jsonl'
    phases=[json.loads(line) for line in phase_file.open()]
    records=[json.loads(line) for line in trace_file.open()]
    poses=[r for r in records if r['topic']=='pose' and abs(sum(x*x for x in r['data']['quaternion'])-1.)<.01]
    commands=[r for r in records if r['topic']=='command']
    rows=[]
    for first,following in zip(phases,phases[1:]):
        if first['phase'] not in ('drive','brake','curve_brake','vertical_up','vertical_down'):continue
        selected=[r for r in poses if first['pose_stamp']<=r['sensor_stamp']<=following['pose_stamp']]
        actual=[r for r in commands if first['wall']<=r['received_wall']<following['wall']]
        if len(selected)<2 or not actual:continue
        positions=np.array([r['data']['position'] for r in selected])
        body=np.array([r['data']['body_velocity'] for r in actual])
        rows.append(dict(phase=first['phase'],level=first.get('level'),
            start_stamp=selected[0]['sensor_stamp'],end_stamp=selected[-1]['sensor_stamp'],
            displacement=(positions[-1]-positions[0]).tolist(),
            minimum_z=float(positions[:,2].min()),maximum_z=float(positions[:,2].max()),
            published_body_velocity_minimum=body.min(axis=0).tolist(),
            published_body_velocity_maximum=body.max(axis=0).tolist(),
            pose_count=len(selected),command_count=len(actual),
            ended_by=following['phase']))
    summary=dict(scope='Actual subscribed commands and pose-clock displacement in bounded calibration; not a race result or global model guarantee.',
        completed=(run/'complete.json').exists(),failed=(run/'failed.json').exists(),phases=rows,
        sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (phase_file,trace_file)})
    (run/'response_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(run=str(run),completed=summary['completed'],phases=rows),indent=2))


if __name__=='__main__':main()
