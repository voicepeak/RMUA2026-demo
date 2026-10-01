#!/usr/bin/env python3
"""Short, bounded velocity/braking measurements near the first road start.

Uses the unchanged official simulator. This is calibration, not a race result.
Run alongside control_trace.py, without route_follower.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from airsim_ros.msg import VelCmd
from tf.transformations import euler_from_quaternion


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--levels',default='2,4,6,8')
    parser.add_argument('--slew',type=float,default=0.)
    parser.add_argument('--drive-seconds',type=float,default=2.)
    parser.add_argument('--compensation',type=float,default=0.)
    parser.add_argument('--brake-feedback',type=float,default=0.)
    parser.add_argument('--start-x',type=float,default=8.)
    parser.add_argument('--brake-slew',type=float,default=0.)
    parser.add_argument('--vertical-command-max',type=float,default=2.5)
    parser.add_argument('--vertical-step',type=float,default=0.)
    args = parser.parse_args()
    args.out.mkdir(parents=True,exist_ok=True)
    events = (args.out/'phases.jsonl').open('w',buffering=1)
    rospy.init_node('motion_probe')
    state = {}
    def pose(msg):
        p,q=msg.pose.position,msg.pose.orientation
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01:return
        position=np.array([p.x,p.y,p.z]);stamp=msg.header.stamp.to_sec()
        velocity=state.get('velocity',np.zeros(3)).copy()
        if state and .001<stamp-state['stamp']<.2:
            dt=stamp-state['stamp'];beta=1.-math.exp(-dt/.08)
            velocity+=beta*((position-state['position'])/dt-velocity)
        state.update(position=position,velocity=velocity,yaw=euler_from_quaternion([q.x,q.y,q.z,q.w])[2],
                     stamp=msg.header.stamp.to_sec(),arrived=time.monotonic())
    rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt',PoseStamped,pose,queue_size=1)
    publisher=rospy.Publisher('/airsim_node/drone_1/vel_body_cmd',VelCmd,queue_size=1)
    previous=np.zeros(3);previous_stamp=None;current_phase=None
    def publish(world):
        nonlocal previous,previous_stamp
        world=np.asarray(world,dtype=float).copy()
        if args.slew>0 and previous_stamp is not None:
            delta=world[:2]-previous[:2];length=np.linalg.norm(delta)
            rate=args.brake_slew if current_phase=='brake' and args.brake_slew>0 else args.slew
            limit=rate*max(0.,min(.1,state['stamp']-previous_stamp))
            if length>limit:world[:2]=previous[:2]+delta*limit/length
        previous=world.copy();previous_stamp=state.get('stamp')
        if args.compensation:
            world[2]+=args.compensation*np.sum((world[:2]-state.get('velocity',np.zeros(3))[:2])**2)
            world[2]=np.clip(world[2],-2.,args.vertical_command_max)
        yaw=state.get('yaw',0.);cy,sy=math.cos(yaw),math.sin(yaw)
        msg=VelCmd();msg.header.stamp=rospy.Time.now();msg.header.frame_id='drone_1'
        msg.vx=cy*world[0]+sy*world[1];msg.vy=-sy*world[0]+cy*world[1]
        msg.vz=-world[2];msg.va=8;msg.stop=0;msg.yawRate=0.
        publisher.publish(msg)
    def event(phase,**fields):
        row=dict(phase=phase,wall=time.time(),pose_stamp=state.get('stamp'),
                 position=state['position'].tolist(),**fields)
        events.write(json.dumps(row)+'\n');print(json.dumps(row),flush=True)
    def phase(name,duration,command,**fields):
        nonlocal current_phase
        current_phase=name
        event(name,**fields);start=state['stamp'];wall=time.monotonic()
        while not rospy.is_shutdown() and state['stamp']-start<duration:
            if time.monotonic()-state['arrived']>.3:raise RuntimeError('pose stale')
            p=state['position']
            if name not in ('takeoff','depart','return','settle') and (p[2]<-4.5 or p[2]>-.5 or abs(p[1])>3.5 or p[0]>38.):
                raise RuntimeError('probe boundary reached: '+str(p))
            if time.monotonic()-wall>duration*3+5:raise RuntimeError('pose clock not advancing')
            publish(command(p,state['stamp']-start));time.sleep(.02)
    deadline=time.monotonic()+25
    while not state and time.monotonic()<deadline:time.sleep(.02)
    if not state:raise RuntimeError('no valid pose')
    if np.linalg.norm(state['position'][:2])>3.:raise RuntimeError('fresh start required')
    try:
        phase('takeoff',4.,lambda p,t:np.array([0.,0.,np.clip(1.3*(-2.2-p[2]),-1.5,1.5)]))
        phase('depart',6.,lambda p,t:np.array([min(2.,max(0.,args.start_x-p[0])),np.clip(-p[1],-1.,1.),np.clip(-2.2-p[2],-1.,1.)]))
        if args.vertical_step:
            for cycle in range(4):
                phase('vertical_up',1.5,lambda p,t:np.array([0.,0.,-args.vertical_step]),cycle=cycle)
                phase('vertical_down',1.5,lambda p,t:np.array([0.,0.,args.vertical_step]),cycle=cycle)
        for level in ([] if args.vertical_step else map(float,args.levels.split(','))):
            phase('settle',3.,lambda p,t:np.array([0.,np.clip(-p[1],-1.,1.),np.clip(-2.2-p[2],-1.,1.)]),level=level)
            phase('drive',args.drive_seconds,lambda p,t:np.array([level,0.,0.]),level=level)
            phase('brake',max(3.5,level/max(.1,args.slew)+2.) if args.slew else 3.5,
                  lambda p,t:np.r_[-args.brake_feedback*state['velocity'][:2]*min(1.,3./max(1e-8,args.brake_feedback*np.linalg.norm(state['velocity'][:2]))),0.],level=level,brake_feedback=args.brake_feedback)
            phase('return',min(15.,max(4.,(state['position'][0]-args.start_x)/1.5+3.)),
                  lambda p,t:np.array([np.clip(1.2*(args.start_x-p[0]),-2.,2.),np.clip(-p[1],-1.,1.),np.clip(-2.2-p[2],-1.,1.)]),level=level)
        event('complete')
        (args.out/'complete.json').write_text(json.dumps(dict(status='MEASUREMENTS_COMPLETE',wall=time.time()),indent=2)+'\n')
    except Exception as error:
        event('failed',error=str(error))
        (args.out/'failed.json').write_text(json.dumps(dict(error=str(error)),indent=2)+'\n')
        raise
    finally:
        for _ in range(20):publish(np.zeros(3));time.sleep(.02)
        events.close()


if __name__=='__main__':main()
