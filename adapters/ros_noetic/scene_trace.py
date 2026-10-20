"""Bounded official-sensor scene recorder, isolated from the control process.

Callbacks retain only the latest message. Compression and disk writes run here,
never on the flight loop. No control publishers, task requests or debug inputs.
"""
import argparse
import json
from pathlib import Path
import threading
import time
import numpy as np


def main():
    import cv2
    import rospy
    import rosgraph
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, PointCloud2
    from sensor_msgs import point_cloud2
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args()
    output=args.run_dir/'scene_trace'
    output.mkdir(exist_ok=False)
    rospy.init_node('rmua_scene_trace',anonymous=True,disable_signals=True)
    graph_types=rosgraph.Master(rospy.get_name()).getTopicTypes()
    bridge=CvBridge()
    lock=threading.Lock()
    pending={}
    last={}
    rows=[]
    counts={}
    def receive(name,msg):
        stamp=msg.header.stamp.to_sec()
        with lock:
            if stamp>last.get(name,float('-inf')):
                pending[name]=(msg,time.monotonic())
    cameras=('front_left','front_right','back_left','back_right')
    listeners=[rospy.Subscriber('/airsim_node/drone_1/'+name+'/Scene',Image,
               lambda msg,n=name:receive(n,msg),queue_size=1,buff_size=4000000) for name in cameras]
    listeners.append(rospy.Subscriber('/airsim_node/drone_1/lidar',PointCloud2,
                     lambda msg:receive('lidar',msg),queue_size=1,buff_size=4000000))
    deadline=time.monotonic()+40
    fault=None
    try:
        while time.monotonic()<deadline and not rospy.is_shutdown() and not (args.run_dir/'scene_stop').exists():
            with lock:
                batch=pending.copy()
                pending.clear()
            for name,(msg,arrival) in batch.items():
                stamp=msg.header.stamp.to_sec()
                period=.09 if name=='lidar' else .45
                if stamp-last.get(name,float('-inf'))<period or counts.get(name,0)>=400:
                    continue
                index=counts.get(name,0)
                relative=name+'_'+str(index).zfill(4)+('.npz' if name=='lidar' else '.png')
                if name=='lidar':
                    points=np.asarray(list(point_cloud2.read_points(msg,field_names=('x','y','z'),
                                      skip_nans=True)),dtype=np.float32).reshape(-1,3)
                    np.savez_compressed(str(output/relative),points=points)
                else:
                    frame=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                    if not cv2.imwrite(str(output/relative),frame):
                        raise RuntimeError('scene image write failed')
                rows.append({'sensor':name,'stamp':stamp,'frame_id':msg.header.frame_id,
                             'arrival_wall_monotonic':arrival,'file':relative})
                last[name]=stamp
                counts[name]=index+1
            if all(counts.get(n,0)>0 for n in ('lidar',)+cameras):
                (args.run_dir/'scene_ready').touch(exist_ok=True)
            time.sleep(.005)
    except Exception as error:
        fault=str(error)
    finally:
        for listener in listeners:listener.unregister()
        (output/'index.json').write_text(json.dumps({'input_policy':'official_sensors_only',
            'advertised_topic_types_for_interface_audit':graph_types,
            'recording_is_lossy':True,'latest_message_only':True,'fault':fault,
            'counts':counts,'frames':rows},indent=2)+'\n')
    return 0 if fault is None and all(counts.get(n,0)>0 for n in ('lidar',)+cameras) else 2


if __name__=='__main__':
    raise SystemExit(main())
