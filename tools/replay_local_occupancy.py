#!/usr/bin/env python3
"""Replay paired LiDAR frames with verified exposure-time origins.

Archives without sensor_origin are rejected. An aircraft position at a later
planning timestamp is not a valid substitute for ray casting.
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
from lidar_points import LidarFrame
from local_occupancy import LocalOccupancy,OccupancyConfig,owned_dynamic_returns,FREE,OCCUPIED,UNKNOWN
from planning_visualization import (occupancy_marker_specs,build_occupancy_markers,
                                    track_marker_specs,build_track_markers)


def read_frame(path):
    meta=json.loads(path.read_text())
    with np.load(path.with_suffix('.npz'),allow_pickle=False) as data:
        origin=data['sensor_origin'] if 'sensor_origin' in data else meta.get('sensor_origin')
        if origin is None:raise ValueError('Missing exposure-time sensor_origin: '+str(path))
        stamp=float(data['cloud_stamp']) if 'cloud_stamp' in data else meta.get('cloud_stamp')
        if stamp is None:raise ValueError('Missing exposure-time cloud_stamp: '+str(path))
        epoch=data['lidar_epoch'].item() if 'lidar_epoch' in data else meta.get('lidar_epoch',meta.get('epoch',0))
        if isinstance(epoch,bool) or not isinstance(epoch,int) or epoch<0:
            raise ValueError('Invalid LiDAR epoch: '+str(path))
        if 'lidar_epoch' in data and meta.get('lidar_epoch') is not None and epoch!=meta['lidar_epoch']:
            raise ValueError('Conflicting LiDAR epochs: '+str(path))
        if ('sensor_origin' in data and meta.get('sensor_origin') is not None and
                not np.allclose(data['sensor_origin'],meta['sensor_origin'],rtol=0.,atol=1e-8)):
            raise ValueError('Conflicting sensor origins: '+str(path))
        if 'cloud_stamp' in data and meta.get('cloud_stamp') is not None and abs(float(data['cloud_stamp'])-float(meta['cloud_stamp']))>1e-9:
            raise ValueError('Conflicting cloud timestamps: '+str(path))
        frame=LidarFrame(float(stamp),data['frame_points'] if 'frame_points' in data else data['points'],origin,epoch)
        position=data['position'].copy();reference=data['reference'].copy()
    if (position.shape!=(3,) or not np.all(np.isfinite(position)) or reference.ndim!=2 or
            reference.shape[1]!=3 or len(reference)<2 or not np.all(np.isfinite(reference))):
        raise ValueError('Invalid reference geometry: '+str(path))
    return meta,frame,position,reference


def replay(cloud_dir,out,config_path=None,tracker_config_path=None,max_frames=None,publish_ros=False,rate=10.):
    config_path=SCRIPTS.parent/'config/local_occupancy.yaml' if config_path is None else config_path
    tracker_config_path=SCRIPTS.parent/'config/dynamic_tracker.yaml' if tracker_config_path is None else tracker_config_path
    configuration=yaml.safe_load(config_path.read_text())
    tracking=yaml.safe_load(tracker_config_path.read_text())
    mapping=LocalOccupancy(OccupancyConfig(**configuration['occupancy']))
    tracker=DynamicTracker(TrackerConfig(**tracking['tracker']))
    if not np.isfinite(rate) or rate<=0:raise ValueError('Invalid replay rate')
    paths=[]
    # Validate frame contracts before creating outputs or any occupancy evidence.
    for path in cloud_dir.glob('*.json'):
        meta,frame,position,reference=read_frame(path)
        paths.append((frame.epoch,frame.stamp,path))
    paths.sort()
    if max_frames is not None:
        if max_frames<1:raise ValueError('max_frames must be positive')
        paths=paths[:max_frames]
    if not paths:raise ValueError('No paired exposure frames in '+str(cloud_dir))
    if out.exists():raise ValueError('Use a new output directory: '+str(out))
    out.mkdir(parents=True)
    if publish_ros:
        import rospy
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray
        rospy.init_node('local_occupancy_replay',anonymous=True)
        diagnostic_pub=rospy.Publisher('/rmua/perception/occupancy_debug',String,queue_size=1,latch=True)
        map_pub=rospy.Publisher('/rmua/perception/occupancy_markers',MarkerArray,queue_size=1,latch=True)
        track_pub=rospy.Publisher('/rmua/perception/dynamic_tracks_markers',MarkerArray,queue_size=1,latch=True)
        rospy.sleep(.5)
    rows=[];times=[];previous=None;duplicates=0;hashes={}
    with (out/'occupancy.jsonl').open('w') as log:
        for epoch,stamp,path in paths:
            if previous==(epoch,stamp):duplicates+=1;continue
            if previous is not None and epoch!=previous[0]:tracker.reset()
            previous=epoch,stamp
            meta,frame,position,reference=read_frame(path)
            stations=float(meta['s'])+np.arange(len(reference))
            forward=reference[1,:2]-reference[0,:2]
            center=lambda q:float(np.interp(q,stations,reference[:,2]))
            started=time.perf_counter()
            tracks=tracker.update(frame.points,position,frame.stamp,forward,center,float(meta['s']),**tracking.get('clustering',{}))
            owners=owned_dynamic_returns(len(frame.points),tracks,frame.stamp,mapping.config)
            track_ms=1000.*(time.perf_counter()-started)
            started=time.perf_counter()
            mapping.update(frame.points,frame.sensor_origin,frame.stamp,position,forward,owners,epoch)
            snapshot=mapping.snapshot();duration=1000.*(time.perf_counter()-started);times.append(duration)
            # Bounded sampling affects display only, never mapping/query results.
            free=snapshot.voxel_centers(FREE,max_count=2048)
            static=snapshot.voxel_centers(OCCUPIED,layer='static',max_count=1024)
            unknown=snapshot.voxel_centers(UNKNOWN,max_count=512)
            row=dict(sample=path.name,stamp=frame.stamp,source_epoch=epoch,position=position.tolist(),
                     sensor_origin=frame.sensor_origin.tolist(),mapping_ms=duration,tracking_ms=track_ms,
                     map=mapping.summary(),tracks=[track.summary() for track in tracks],
                     display=dict(free=free.tolist(),static=static.tolist(),unknown=unknown.tolist()))
            log.write(json.dumps(row,allow_nan=False)+'\n');rows.append(row)
            for p in (path,path.with_suffix('.npz')):hashes[p.name]=hashlib.sha256(p.read_bytes()).hexdigest()
            if publish_ros:
                if rospy.is_shutdown():break
                preview_stamp=rospy.Time.now()
                diagnostic_pub.publish(String(data=json.dumps({k:v for k,v in row.items() if k!='display'},allow_nan=False)))
                map_pub.publish(build_occupancy_markers(occupancy_marker_specs(snapshot),preview_stamp,lifetime=max(.5,2./rate)))
                track_pub.publish(build_track_markers(track_marker_specs(tracks),preview_stamp,lifetime=max(.5,2./rate)))
                rospy.sleep(1./rate)
    summary=dict(scope='Recorded exposure-time rays; open-loop perception replay, not flight evidence.',
        frames=len(rows),duplicate_frames_skipped=duplicates,config=asdict(mapping.config),
        tracker_config=asdict(tracker.config),input_sha256=hashes,
        source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
                       (SCRIPTS/'local_occupancy.py',SCRIPTS/'lidar_points.py',SCRIPTS/'dynamic_tracker.py',
                        SCRIPTS/'lidar_scene.py',SCRIPTS/'planning_visualization.py',Path(__file__))},
        mapping_ms=dict(p50=float(np.median(times)),p95=float(np.percentile(times,95)),maximum=float(np.max(times))),
        dynamic_frames=sum(row['map']['dynamic_hit_voxels']>0 for row in rows),
        max_voxels=max(row['map']['voxel_count'] for row in rows),
        rays_skipped=sum(row['map']['rays_skipped'] for row in rows),
        latest_map=rows[-1]['map'])
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    write_preview(rows,out/'preview.html')
    return summary


def write_preview(rows,path):
    payload=json.dumps(rows,allow_nan=False).replace('<','\\u003c')
    page='''<!doctype html><html lang="zh"><meta charset="utf-8"><title>滚动 Occupancy 重放</title>
<style>body{font:16px sans-serif;background:#121722;color:#eef3ff;margin:24px}canvas{background:#202838;width:100%;max-width:1100px}input{width:65%}label{margin:12px}pre{white-space:pre-wrap}</style>
<h1>滚动 Occupancy / 动态预测</h1><p>绿色 FREE，红色静态 OCCUPIED，灰色 UNKNOWN；蓝框最近目标估计，橙框未来 1s/2s 含不确定度预测。XY/XZ 显示世界 NED；未绘制的区域也不能视为 FREE。显示点有采样上限。</p>
<button id="play">播放</button><input id="time" type="range" min="0" step="1"><span id="label"></span><label><input id="unknown" type="checkbox" style="width:auto">显示 UNKNOWN</label>
<canvas id="view" width="1100" height="680"></canvas><pre id="info"></pre>
<script>const frames=PAYLOAD,s=document.getElementById('time'),c=document.getElementById('view'),ctx=c.getContext('2d');s.max=frames.length-1;s.value=0;
function draw(){const f=frames[+s.value];ctx.clearRect(0,0,1100,680);document.getElementById('label').textContent=`${+s.value+1}/${frames.length} t=${f.stamp.toFixed(3)}`;
for(const [axis,offset,label] of [[1,0,'XY / NED'],[2,340,'XZ / NED']]){const world=p=>[120+(p[0]-f.position[0])*32,offset+170+(p[axis]-f.position[axis])*32];ctx.fillStyle='white';ctx.fillText(label+' / 32 pixels per metre',10,offset+20);
for(const [name,color] of [['unknown','#667085'],['free','#36d875'],['static','#f35c52']]){if(name==='unknown'&&!document.getElementById('unknown').checked)continue;ctx.fillStyle=color;ctx.globalAlpha=name==='unknown'?.15:name==='free'?.4:.7;for(const p of f.display[name]){const [x,y]=world(p);ctx.fillRect(x-2,y-2,4,4);}ctx.globalAlpha=1;}
const [x,y]=world(f.position);ctx.fillStyle='white';ctx.beginPath();ctx.arc(x,y,4,0,7);ctx.fill();const [ox,oy]=world(f.sensor_origin);ctx.strokeStyle='white';ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(ox,oy);ctx.stroke();
for(const t of f.tracks){const boxes=[{position:t.position,size:t.bbox_size,dt:0},...t.predictions.map(p=>({position:p.position,size:p.high.map((v,i)=>v-p.low[i]),dt:p.dt}))];for(const b of boxes){const [bx,by]=world(b.position);ctx.strokeStyle=b.dt?'#ff9c35':'#5e9eff';ctx.setLineDash(b.dt?[5,4]:[]);ctx.strokeRect(bx-b.size[0]*16,by-b.size[axis]*16,Math.max(1,b.size[0]*32),Math.max(1,b.size[axis]*32));ctx.fillStyle=ctx.strokeStyle;ctx.fillText(`${t.epoch}:${t.track_id} +${b.dt}s`,bx+3,by-3);}ctx.setLineDash([]);}}
document.getElementById('info').textContent=JSON.stringify({map:f.map,mapping_ms:f.mapping_ms,tracking_ms:f.tracking_ms,sensor_origin:f.sensor_origin},null,2);}
s.oninput=draw;document.getElementById('unknown').onchange=draw;let timer=null;document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;return;}timer=setInterval(()=>{if(+s.value>=frames.length-1){clearInterval(timer);timer=null;return;}s.value=+s.value+1;draw();},100);};draw();</script></html>'''
    path.write_text(page.replace('PAYLOAD',payload))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cloud-dir',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=SCRIPTS.parent/'config/local_occupancy.yaml')
    parser.add_argument('--tracker-config',type=Path,default=SCRIPTS.parent/'config/dynamic_tracker.yaml')
    parser.add_argument('--max-frames',type=int)
    parser.add_argument('--publish-ros',action='store_true')
    parser.add_argument('--rate',type=float,default=10.)
    args=parser.parse_args()
    summary=replay(args.cloud_dir,args.out,args.config,args.tracker_config,args.max_frames,args.publish_ros,args.rate)
    print(json.dumps({k:summary[k] for k in ('frames','mapping_ms','dynamic_frames','max_voxels','rays_skipped')}))


if __name__=='__main__':main()
