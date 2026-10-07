#!/usr/bin/env python3
"""Replay chronological recorded world clouds without issuing flight commands.

Cloud archive format matches RouteFollower's paired *.json/*.npz snapshots.
Reports associate prediction residuals with future observed box centers; those
are LiDAR measurements, not independent vehicle ground truth. Optional ROS
output publishes diagnostics and preview markers only.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import yaml

SCRIPTS=Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/scripts'
sys.path.insert(0,str(SCRIPTS))
from dynamic_tracker import DynamicTracker,TrackerConfig
from lidar_scene import extract_clusters
from planning_visualization import track_marker_specs,build_track_markers


def observation_stamp(meta):
    if meta.get('cloud_stamp') is not None:return float(meta['cloud_stamp']),'cloud_stamp'
    geometry=meta.get('clearance',{}).get('geometry_cloud_stamp')
    if geometry is not None:return float(geometry),'geometry_cloud_stamp'
    return float(meta['pose_stamp']),'pose_stamp_proxy'


def replay(cloud_dir,out,config_path,max_frames=None,publish_ros=False,rate=10.):
    if not np.isfinite(rate) or rate<=0:raise ValueError('Replay rate must be positive')
    config=yaml.safe_load(config_path.read_text())
    tracker=DynamicTracker(TrackerConfig(**config['tracker']))
    clustering=config.get('clustering',{})
    # Fail unknown options even when the archive is empty.
    import inspect
    allowed=set(inspect.signature(extract_clusters).parameters)-{'points','position','forward','center','s','with_count'}
    if not set(clustering)<=allowed:raise ValueError('Unknown clustering configuration key')
    items=[]
    for path in cloud_dir.glob('*.json'):
        meta=json.loads(path.read_text());stamp,source=observation_stamp(meta)
        if not np.isfinite(stamp):raise ValueError('Nonfinite timestamp: '+str(path))
        if not path.with_suffix('.npz').is_file():raise ValueError('Missing cloud: '+str(path))
        items.append((stamp,path,meta,source))
    items.sort(key=lambda item:(int(item[2].get('epoch',0)),item[0],str(item[1])))
    if max_frames is not None:
        if max_frames<1:raise ValueError('max_frames must be positive')
        items=items[:max_frames]
    if not items:raise ValueError('No paired cloud snapshots in '+str(cloud_dir))
    if out.exists():raise ValueError('Use a new output directory: '+str(out))
    out.mkdir(parents=True)
    diagnostics=markers=None
    if publish_ros:
        import rospy
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray
        rospy.init_node('dynamic_tracker_replay',anonymous=True)
        diagnostics=rospy.Publisher('/rmua/perception/dynamic_tracks',String,queue_size=1,latch=True)
        markers=rospy.Publisher('/rmua/perception/dynamic_tracks_markers',MarkerArray,queue_size=1,latch=True)
        rospy.sleep(.5)
    rows=[];durations=[];previous=None;source_epoch=None;intervals=[];proxies=0;duplicates=0
    hashes={};next_predictions=[];residuals={1.:[],2.:[]}
    with (out/'tracks.jsonl').open('w') as stream:
        for stamp,path,meta,source in items:
            frame_epoch=int(meta.get('epoch',0))
            if source_epoch is not None and frame_epoch!=source_epoch:
                tracker.reset();previous=None;next_predictions=[]
            source_epoch=frame_epoch
            if previous is not None and stamp==previous:duplicates+=1;continue
            if previous is not None:intervals.append(stamp-previous)
            previous=stamp;proxies+=source=='pose_stamp_proxy'
            hashes[path.name]=hashlib.sha256(path.read_bytes()).hexdigest()
            hashes[path.with_suffix('.npz').name]=hashlib.sha256(path.with_suffix('.npz').read_bytes()).hexdigest()
            with np.load(path.with_suffix('.npz'),allow_pickle=False) as data:
                points=data['points'];position=data['position'];reference=data['reference']
            s=float(meta['s'])
            if reference.ndim!=2 or reference.shape[1]!=3 or len(reference)<2:
                raise ValueError('Invalid route reference: '+str(path))
            stations=s+np.arange(len(reference),dtype=float)
            tangent=reference[1,:2]-reference[0,:2]
            center=lambda q:float(np.interp(q,stations,reference[:,2]))
            started=time.perf_counter()
            tracks=tracker.update(points,position,stamp,tangent,center,s,**clustering)
            duration=1000.*(time.perf_counter()-started);durations.append(duration)
            summary=tracker.summary()
            observed={(o.epoch,o.track_id):o for o in tracks if o.age==0.}
            retained=[]
            for prediction in next_predictions:
                if stamp<prediction['target']-.2:
                    retained.append(prediction);continue
                key=prediction['epoch'],prediction['track_id']
                if abs(stamp-prediction['target'])<=.2 and key in observed:
                    o=observed[key]
                    # Re-evaluate at the exact observation time; interpolation
                    # of ground truth or a near future timestamp is not assumed.
                    expected=prediction['obstacle'].predict_at(stamp).position
                    residuals[prediction['dt']].append(float(np.linalg.norm(o.observed_position-expected)))
            next_predictions=retained
            for o in tracks:
                if o.status!='CONFIRMED':continue
                for dt in (1.,2.):
                    next_predictions.append(dict(epoch=o.epoch,track_id=o.track_id,dt=dt,
                                                 target=stamp+dt,obstacle=o))
            row=dict(sample=path.name,stamp=stamp,timestamp_source=source,source_epoch=frame_epoch,position=position.tolist(),
                     tracking_ms=duration,**summary)
            stream.write(json.dumps(row,allow_nan=False)+'\n');rows.append(row)
            if publish_ros:
                if rospy.is_shutdown():break
                diagnostics.publish(String(data=json.dumps(row,allow_nan=False)))
                markers.publish(build_track_markers(track_marker_specs(tracks),rospy.Time.now(),
                                                    lifetime=max(.5,2./rate)))
                rospy.sleep(1./rate)
    quantiles=lambda values:None if not values else dict(p50=float(np.percentile(values,50)),
                                                        p95=float(np.percentile(values,95)),max=float(np.max(values)))
    summary=dict(scope='Open-loop recorded LiDAR replay; no flight commands or vehicle ground truth.',
        cloud_dir=str(cloud_dir.resolve()),config=asdict(tracker.config),clustering=clustering,
        config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        scripts_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                       (SCRIPTS/'dynamic_tracker.py',SCRIPTS/'lidar_scene.py',SCRIPTS/'planning_visualization.py')},
        input_sha256=hashes,frames=len(rows),duplicate_frames_skipped=duplicates,proxy_timestamp_frames=proxies,
        sensor_interval_seconds=quantiles(intervals),tracking_ms=quantiles(durations),
        confirmed_frames=sum(row['confirmed_count']>0 for row in rows),
        moving_frames=sum(row['moving_count']>0 for row in rows),
        tracks_created=sum(row['created'] for row in rows),tracks_expired=sum(row['expired'] for row in rows),
        prediction_observation_residual_m={str(dt):dict(samples=len(values),statistics=quantiles(values))
                                           for dt,values in residuals.items()},
        limitations=['IDs inferred from association; ambiguous merged clusters may lose identity.',
                     'Residuals use associated visible bbox centers; no independent ground truth.',
                     'Sparse archives may exceed configured track lifetimes; do not reinterpret expiry as ID switching.'])
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    write_preview(rows,out/'preview.html')
    return summary


def write_preview(rows,path):
    """Self-contained interactive time slider with top/side NED projections."""
    # Embed only known numeric/string diagnostics, never archive filenames or
    # arbitrary JSON text into executable HTML.
    frames=[dict(stamp=r['stamp'],position=r['position'],tracks=r['tracks']) for r in rows]
    payload=json.dumps(frames,allow_nan=False).replace('<','\\u003c')
    page='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>LiDAR动态跟踪重放</title>
<style>body{font:16px sans-serif;margin:24px;background:#121722;color:#edf2ff}canvas{background:#202838;border-radius:8px;width:100%;max-width:1100px}input{width:80%}button{padding:8px}pre{white-space:pre-wrap}label{display:block;margin:12px 0}</style>
<h1>LiDAR 动态跟踪重放</h1><p>蓝色：最近一次观测包围盒；橙色：未来 1 秒 / 2 秒含不确定度的包围盒。白点：无人机。上图 XY， 下图 XZ，均为世界 NED 坐标。非车辆真值，也非实飞结果。</p>
<button id="play">播放</button><label>帧 <input id="time" type="range" min="0" step="1"><span id="label"></span></label>
<canvas id="view" width="1100" height="680"></canvas><pre id="info"></pre>
<script>const frames=PAYLOAD;const slider=document.getElementById('time'),canvas=document.getElementById('view'),ctx=canvas.getContext('2d');slider.max=frames.length-1;slider.value=0;
function draw(){const f=frames[+slider.value];ctx.clearRect(0,0,1100,680);document.getElementById('label').textContent=`${+slider.value+1}/${frames.length}, t=${f.stamp.toFixed(3)}`;
for(const [axis,offset,title] of [[1,0,'XY: y grows downward (NED)'],[2,340,'XZ: z grows downward (NED)']]){const origin=f.position;const world=(p)=>[550+(p[0]-origin[0])*18,offset+170+(p[axis]-origin[axis])*18];ctx.strokeStyle='#39465c';ctx.beginPath();ctx.moveTo(0,offset+170);ctx.lineTo(1100,offset+170);ctx.stroke();ctx.fillStyle='white';ctx.fillText(title+' / 18 px per metre',12,offset+20);ctx.beginPath();ctx.arc(550,offset+170,4,0,7);ctx.fill();
for(const t of f.tracks){const boxes=[{position:t.observed_position,size:t.bbox_size,dt:0},...t.predictions.map(p=>({position:p.position,size:p.high.map((v,i)=>v-p.low[i]),dt:p.dt}))];for(const box of boxes){const [x,y]=world(box.position),w=box.size[0]*18,h=box.size[axis]*18;ctx.strokeStyle=box.dt?'#ff9f30':'#4b9fff';ctx.setLineDash(box.dt?[5,4]:[]);ctx.strokeRect(x-w/2,y-h/2,w,h);ctx.fillStyle=ctx.strokeStyle;ctx.fillText(`${t.epoch}:${t.track_id} +${box.dt}s`,x+3,y-3);}ctx.setLineDash([]);}}
document.getElementById('info').textContent=f.tracks.map(t=>`ID ${t.epoch}:${t.track_id} ${t.status}  v=[${t.velocity.map(v=>v.toFixed(2))}] m/s  age=${t.age.toFixed(2)}s  confidence=${t.confidence.toFixed(2)}  motion_confirmed=${t.motion_confirmed}`).join('\\n')||'当前没有有效 track';}
slider.oninput=draw;let timer=null;document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;return;}timer=setInterval(()=>{if(+slider.value>=frames.length-1){clearInterval(timer);timer=null;return;}slider.value=+slider.value+1;draw();},100);};draw();</script></html>'''
    path.write_text(page.replace('PAYLOAD',payload))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cloud-dir',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=SCRIPTS.parent/'config/dynamic_tracker.yaml')
    parser.add_argument('--max-frames',type=int)
    parser.add_argument('--publish-ros',action='store_true',help='Replay diagnostics/markers; never publish VelCmd')
    parser.add_argument('--rate',type=float,default=10.)
    args=parser.parse_args()
    summary=replay(args.cloud_dir,args.out,args.config,args.max_frames,args.publish_ros,args.rate)
    print(json.dumps({k:summary[k] for k in ('frames','confirmed_frames','moving_frames','tracking_ms',
                                            'sensor_interval_seconds','prediction_observation_residual_m')}))


if __name__=='__main__':main()
