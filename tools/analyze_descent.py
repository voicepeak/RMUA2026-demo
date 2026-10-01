#!/usr/bin/env python3
"""Compare actual first-leg descent, excluding post-finish/local-only motion."""
import argparse
import json
from pathlib import Path
import numpy as np


def samples(directory,lo,hi):
    mission=[json.loads(l) for l in (directory/'mission/mission.jsonl').open()]
    finish=next(x['stamp'] for x in mission if x['kind']=='OFFICIAL_ENDPOINT_CHANGED')
    rows=[]
    for line in (directory/'flight/streams.jsonl').open():
        x=json.loads(line)
        if x['topic']=='telemetry' and x['received']<finish and lo<x['data']['s']<hi:
            if not rows or x['data']['pose_stamp']>rows[-1]['pose_stamp']:rows.append(x['data'])
    t=np.array([x['pose_stamp'] for x in rows]);tt=np.arange(t[0],t[-1],.1)
    def series(key):return np.interp(tt,t,[key(x) for x in rows])
    pitch=series(lambda x:np.degrees(x['attitude_rad'][1]));speed=series(lambda x:x['measured_progress_speed'])
    lifted=pitch>3.
    runs=np.split(np.flatnonzero(lifted),np.flatnonzero(np.diff(np.flatnonzero(lifted))>1)+1)
    data=dict(progress=series(lambda x:x['s']),pitch=pitch,speed=speed,
              commanded=series(lambda x:x['speed']),slope=series(lambda x:x['dzds']),
              error=series(lambda x:x['z']-x['z_ref']),time=tt-tt[0])
    metrics=dict(elapsed_seconds=float(t[-1]-t[0]),actual_speed_median=float(np.median(speed)),
                 actual_speed_p10=float(np.percentile(speed,10)),
                 pitch_rate_abs_p95_deg_s=float(np.percentile(abs(np.diff(pitch)/.1),95)),
                 pitch_peak_degrees=float(np.max(pitch)),
                 nose_up_episodes_over_3deg_200ms=sum(len(x)>=2 for x in runs),
                 reference_reverse_samples=int(sum(x['dzds']<-.02 for x in rows)),
                 obstacle_stop_samples=sum(x['speed_limits']['reason']=='OBSTACLE' and x['speed']<.1 for x in rows),
                 height_error_abs_p95=float(np.percentile(abs(data['error']),95)),
                 official_first_leg_end_wall_stamp=finish)
    return data,metrics


def main():
    p=argparse.ArgumentParser();p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--start',type=float,default=1040.);p.add_argument('--end',type=float,default=1250.)
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(5,1,figsize=(11,12),sharex=True)
    report=dict(window=[a.start,a.end],scope='First leg before official end_goal update')
    for directory,label in [(a.baseline,'baseline'),(a.candidate,'candidate')]:
        data,metrics=samples(directory,a.start,a.end);report[label]=dict(run=str(directory),**metrics)
        for ax,key,title in zip(axes,['speed','commanded','pitch','slope','error'],
                                  ['Measured progress speed (m/s)','Commanded speed (m/s)','Pitch (degrees)',
                                   'Height path slope (NED m/m)','Height tracking error (m)']):
            ax.plot(data['progress'],data[key],label=label);ax.set_ylabel(title);ax.grid(alpha=.25)
    axes[0].legend();axes[-1].set_xlabel('Route progress (m)');fig.tight_layout()
    fig.savefig(a.out/'descent_comparison.png',dpi=160)
    (a.out/'descent_comparison.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
