#!/usr/bin/env python3
"""Bounded read-only ROS capture for continuous tracker validation.

Requires one archived *.json/*.npz route-reference pair from the same leg.
Only subscribes to pose and LiDAR, never sends VelCmd/reset/start/stop requests.
Transforms returns with their interpolated sensor-time pose and -0.05m mount.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import sys
import threading
import time
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros_ws/src/route_follower/scripts'))
from pose_history import PoseHistory
from lidar_points import valid_points


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,required=True,help='Archived reference JSON for the current leg')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=8.,help='Wall-clock recording seconds')
    parser.add_argument('--rate',type=float,default=10.,help='Maximum captured frames/sec of sensor time')
    args=parser.parse_args()
    if any(not np.isfinite(v) or v<=0 for v in (args.duration,args.rate)):parser.error('duration/rate must be positive')
    if args.out.exists():parser.error('Use a new output directory')
    meta=json.loads(args.reference.read_text())
    with np.load(args.reference.with_suffix('.npz'),allow_pickle=False) as data:reference=data['reference'].copy()
    if reference.ndim!=2 or reference.shape[1]!=3 or len(reference)<2:parser.error('Invalid route reference')
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from tf.transformations import quaternion_matrix
    rospy.init_node('record_tracker_clouds',anonymous=True)
    args.out.mkdir(parents=True)
    lock=threading.Lock();poses=PoseHistory();clouds=deque(maxlen=16)
    counts=dict(received=0,recorded=0,unpaired=0,invalid=0,clock_resets=0)
    last_pose=None;last_recorded=None;epoch=0

    def pose_cb(msg):
        nonlocal last_pose,epoch,last_recorded
        p=msg.pose.position;q=msg.pose.orientation;stamp=msg.header.stamp.to_sec()
        position=np.array([p.x,p.y,p.z]);quaternion=np.array([q.x,q.y,q.z,q.w])
        if not np.isfinite(stamp) or not np.all(np.isfinite(np.r_[position,quaternion])) or np.linalg.norm(quaternion)<1e-9:
            return
        with lock:
            if last_pose is not None and stamp<last_pose:
                poses.rows.clear();clouds.clear();last_recorded=None;epoch+=1;counts['clock_resets']+=1
            last_pose=stamp;poses.add(stamp,position,quaternion/np.linalg.norm(quaternion))

    def lidar_cb(msg):
        stamp=msg.header.stamp.to_sec();fields={f.name:f for f in msg.fields}
        if (not np.isfinite(stamp) or msg.point_step<=0 or
                any(k not in fields or fields[k].datatype!=7 for k in ('x','y','z'))):
            with lock:counts['invalid']+=1
            return
        endian='>' if msg.is_bigendian else '<'
        try:
            dtype=np.dtype(dict(names=['x','y','z'],formats=[endian+'f4']*3,
                                offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
            # Row stride matters for organized clouds with padding.
            array=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,
                             strides=(msg.row_step,msg.point_step))
            points=valid_points(np.column_stack([array[k].reshape(-1) for k in ('x','y','z')]))
        except (ValueError,TypeError):
            with lock:counts['invalid']+=1
            return
        with lock:
            if len(clouds)==clouds.maxlen:counts['unpaired']+=1
            clouds.append((stamp,points));counts['received']+=1

    subscribers=[rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt',PoseStamped,pose_cb,queue_size=20),
                 rospy.Subscriber('/airsim_node/drone_1/lidar',PointCloud2,lidar_cb,queue_size=2,buff_size=4*1024*1024)]
    started=time.monotonic();segments=np.diff(reference[:,:2],axis=0);length=np.sum(segments**2,axis=1)
    stamps=[]
    while not rospy.is_shutdown() and time.monotonic()-started<args.duration:
        ready=[]
        with lock:
            while clouds:
                stamp,points=clouds[0]
                pose=poses.sample(stamp)
                if pose is None:
                    if poses.rows and stamp<poses.rows[0][0]:
                        clouds.popleft();counts['unpaired']+=1;continue
                    break
                clouds.popleft()
                if last_recorded is not None and stamp-last_recorded<1./args.rate:continue
                last_recorded=stamp;ready.append((stamp,points,pose,epoch))
        for stamp,points,(position,quaternion),frame_epoch in ready:
            rotation=quaternion_matrix(quaternion)[:3,:3]
            sensor_origin=position+rotation@np.array([0.,0.,-.05])
            world=points@rotation.T+sensor_origin
            fraction=np.clip(np.sum((position[:2]-reference[:-1,:2])*segments,axis=1)/np.maximum(1e-9,length),0.,1.)
            nearest=np.sum((position[:2]-reference[:-1,:2]-fraction[:,None]*segments)**2,axis=1)
            nearest[length<1e-9]=np.inf
            index=int(np.argmin(nearest));station=float(meta['s'])+index+fraction[index]
            ref_stations=float(meta['s'])+np.arange(len(reference))
            local=np.column_stack([np.interp(station+np.arange(66.),ref_stations,reference[:,axis]) for axis in (0,1,2)])
            name=args.out/('e%d_%012.6f'%(frame_epoch,stamp))
            np.savez_compressed(str(name)+'.npz',points=world,position=position,reference=local,sensor_origin=sensor_origin)
            Path(str(name)+'.json').write_text(json.dumps(dict(s=station,cloud_stamp=stamp,pose_stamp=stamp,
                epoch=frame_epoch,sensor_origin=sensor_origin.tolist(),
                scope='Read-only capture using an archived fixed route/height reference'),allow_nan=False)+'\n')
            counts['recorded']+=1;stamps.append(stamp)
        time.sleep(.01)
    for subscriber in subscribers:subscriber.unregister()
    with lock:counts['unpaired']+=len(clouds)
    intervals=np.diff(stamps)
    counts.update(wall_seconds=time.monotonic()-started,reference=str(args.reference.resolve()),
                  sensor_interval_p50=None if not len(intervals) else float(np.median(intervals)),
                  sensor_interval_max=None if not len(intervals) else float(np.max(intervals)))
    (args.out/'capture_summary.txt').write_text(json.dumps(counts,indent=2)+'\n')
    print(json.dumps(counts))
    if not counts['recorded']:raise SystemExit('No time-paired frames captured')


if __name__=='__main__':main()
