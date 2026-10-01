#!/usr/bin/env python3
"""Summarize diagnostic clocks and telemetry bounded by observed HUD validity."""
import argparse
import csv
import json
from pathlib import Path
import re
from collections import Counter

def percentiles(values):
    if not values:return None
    ordered=sorted(values);result={}
    for label,fraction in (('p50',.5),('p95',.95),('max',1.)):
        index=(len(ordered)-1)*fraction;low=int(index);high=min(low+1,len(ordered)-1)
        result[label]=ordered[low]+(ordered[high]-ordered[low])*(index-low)
    return result

def summarize(run):
    settings=json.loads((run/'settings.json').read_text())
    result=dict(seed=123,requested_clock_speed=settings.get('ClockSpeed',1.),accepted=False,
                scope='Diagnostic summary; local progress does not prove official stage completion')
    stop=run/'stop.json'
    if stop.exists():result['stop']=json.loads(stop.read_text())
    captures=run/'hud/captures.jsonl';cutoff=None;finished=False
    if captures.exists():
        for line in captures.open():
            row=json.loads(line);ocr=row.get('ocr','')
            if re.search(r'\bFinished\b',ocr,re.I):
                if not finished:result['first_finished_hud']=row
                finished=True
            elif not finished and re.search(r'State\s*:',ocr,re.I):
                cutoff=row['wall_time'];result['last_valid_hud']=row
    result['telemetry_validity_basis']=('last non-Finished HUD wall timestamp'
        if cutoff is not None else 'HUD unavailable; official validity unknown')
    poses=run/'flight/poses.csv'
    if poses.exists():
        rows=list(csv.DictReader(poses.open()))
        rows=[row for row in rows if row.get('received_wall')]
        if len(rows)>2:
            first,last=rows[0],rows[-1]
            wall=float(last['received_wall'])-float(first['received_wall'])
            result['pose_clock_ratio']=(float(last['stamp'])-float(first['stamp']))/max(.001,wall)
    mission=run/'mission/mission.jsonl';boundaries=[]
    if mission.exists():
        boundaries=[(event['stamp'],event['stage']) for event in
                    (json.loads(line) for line in mission.open())
                    if event.get('kind')=='CONTROLLER_STARTED']
    count=0;start=end=None;maximum={};stages={};stream=run/'flight/streams.jsonl'
    if stream.exists():
        for line in stream.open():
            row=json.loads(line)
            if row.get('topic')!='telemetry' or (cutoff is not None and row['received']>cutoff):continue
            count+=1
            if start is None:start=row['received']
            end=row['received'];stage=0
            for stamp,index in boundaries:
                if row['received']>=stamp:stage=index
            key=str(stage);maximum[key]=max(maximum.get(key,0.),row['data']['s'])
            metrics=stages.setdefault(key,dict(received=[],speed=[],navigation_ms=[],grid_ms=[],reasons=Counter()))
            metrics['received'].append(row['received']);metrics['speed'].append(row['data']['speed'])
            clearance=row['data'].get('clearance',{})
            for metric in ('navigation_ms','grid_ms'):
                if metric in clearance:metrics[metric].append(clearance[metric])
            metrics['reasons'][clearance.get('command_reason','UNKNOWN')]+=1
    stage_result={}
    for stage,metrics in stages.items():
        times=metrics['received']
        stage_result[stage]=dict(samples=len(times),telemetry_wall_rate_hz=len(times)/max(.001,times[-1]-times[0]),
            command_speed=percentiles(metrics['speed']),navigation_ms=percentiles(metrics['navigation_ms']),
            grid_ms=percentiles(metrics['grid_ms']),command_reasons=dict(metrics['reasons']))
    result.update(valid_telemetry_samples=count,max_local_s_by_stage=maximum,
                  performance_by_stage=stage_result,
                  telemetry_wall_rate_hz=count/max(.001,(end-start)) if count>1 else None)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True)
    args=parser.parse_args();result=summarize(args.run)
    (args.run/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False))
