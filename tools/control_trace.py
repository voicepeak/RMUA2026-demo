#!/usr/bin/env python3
"""Independent recording of the actual command topic and sensor timing."""
import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from airsim_ros.msg import VelCmd
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--save-clouds', action='store_true')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if args.save_clouds:
        (args.out/'clouds').mkdir(exist_ok=True)
    file = (args.out/'control.jsonl').open('w', buffering=1)
    lock = threading.Lock()
    def write(topic, msg, data):
        row = dict(topic=topic, sensor_stamp=msg.header.stamp.to_sec(),
                   received_wall=time.time(), received_monotonic=time.monotonic(),
                   received_ros=rospy.Time.now().to_sec(), data=data)
        with lock:
            if not file.closed:
                file.write(json.dumps(row, allow_nan=False)+'\n')
    def pose(msg):
        p, q = msg.pose.position, msg.pose.orientation
        write('pose', msg, dict(position=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w]))
    def command(msg):
        write('command', msg, dict(body_velocity=[msg.vx,msg.vy,msg.vz],
              yaw_rate_deg=msg.yawRate, acceleration=msg.va, stop=msg.stop))
    def imu(msg):
        q, a, w = msg.orientation, msg.linear_acceleration, msg.angular_velocity
        write('imu', msg, dict(quaternion=[q.x,q.y,q.z,q.w],
              linear_acceleration=[a.x,a.y,a.z], angular_velocity=[w.x,w.y,w.z]))
    def cloud(msg):
        data = dict(frame_id=msg.header.frame_id, width=msg.width, height=msg.height,
                    row_step=msg.row_step, point_step=msg.point_step)
        if args.save_clouds:
            name = '%.9f.npz' % msg.header.stamp.to_sec()
            fields = {f.name:f for f in msg.fields}
            if all(k in fields and fields[k].datatype == 7 for k in ('x','y','z')):
                dtype = np.dtype(dict(names=['x','y','z'],
                    formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                    offsets=[fields[k].offset for k in ('x','y','z')],itemsize=msg.point_step))
                raw = np.frombuffer(msg.data,dtype=dtype)
                points = np.column_stack([raw[k] for k in ('x','y','z')])
                np.savez_compressed(args.out/'clouds'/name, points=points)
                data['file'] = 'clouds/'+name
        write('lidar', msg, data)
    rospy.init_node('control_trace', anonymous=True)
    base = '/airsim_node/drone_1/'
    rospy.Subscriber(base+'debug/pose_gt',PoseStamped,pose,queue_size=100)
    rospy.Subscriber(base+'vel_body_cmd',VelCmd,command,queue_size=100)
    rospy.Subscriber(base+'imu/imu',Imu,imu,queue_size=100)
    rospy.Subscriber(base+'lidar',PointCloud2,cloud,queue_size=5,buff_size=4*1024*1024)
    def diagnostic(msg):
        with lock:
            if not file.closed:
                file.write(json.dumps(dict(topic='command_trace',received_wall=time.time(),
                    received_monotonic=time.monotonic(),received_ros=rospy.Time.now().to_sec(),
                    data=json.loads(msg.data)))+'\n')
    rospy.Subscriber('/rmua/controller/command_trace',String,diagnostic,queue_size=100)
    try:
        rospy.spin()
    finally:
        with lock:
            file.close()


if __name__ == '__main__':
    main()
