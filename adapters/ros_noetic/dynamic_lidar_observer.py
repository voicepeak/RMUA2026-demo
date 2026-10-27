"""Read-only online LiDAR obstacle candidate and veto probe."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import threading
import time

from adapters.ros_noetic.flight_sensors import FlightSensors
from adapters.ros_noetic.lidar_cloud import decode_xyz
from estimation.history import PoseHistory
from perception.object_motion.online import OnlineDynamicLidar
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.transport import LatestSlot


def wait_for_imu(cloud, arrival, snapshot, wall_clock, sleep, stopped, max_wait_s=.03):
    """Wait briefly for causal IMU coverage without extending source lifetime."""
    while not stopped():
        now, epoch, fresh, _ = snapshot()
        elapsed = wall_clock() - arrival
        if (not fresh or epoch != cloud.metadata.epoch or now is None
                or now >= cloud.metadata.stamp or elapsed < 0 or elapsed >= max_wait_s):
            return
        sleep(min(.001, max_wait_s - elapsed))


def observe(run,duration,settings):
    import rospy
    import rosgraph
    from sensor_msgs.msg import PointCloud2
    rospy.init_node('rmua_dynamic_lidar_observer',anonymous=True,disable_signals=True)
    history=PoseHistory();sensors=FlightSensors(pose_history=history)
    rig=load_nominal_rig(settings);pipeline=OnlineDynamicLidar(history,rig.lidar_pose)
    slot=LatestSlot();stop=threading.Event();faults=[];rows=[];received=0;lock=threading.Lock()
    def callback(msg):
        nonlocal received
        with lock:received+=1
        slot.put((msg,time.monotonic()))
    listener=rospy.Subscriber('/airsim_node/drone_1/lidar',PointCloud2,callback,queue_size=1,buff_size=4000000)
    def snapshot():
        with sensors.lock:
            fresh=sensors.clock.fresh(time.monotonic()) and not sensors.fault
            state=sensors.estimator.snapshot() if sensors.estimator.initialized and sensors.estimator.stamp is not None else None
            return sensors.clock.stamp,sensors.clock.epoch,fresh,state
    def worker():
        try:
            with (run/'dynamic_lidar_results.jsonl').open('x') as output:
                while not stop.is_set():
                    item=slot.take()
                    if item is None:time.sleep(.002);continue
                    message,arrival=item;started=time.monotonic();now,epoch,fresh,_=snapshot()
                    if not fresh:
                        row={'status':'REJECTED','reason':'STALE_IMU_CLOCK','control_eligible':False}
                    else:
                        try:
                            cloud=decode_xyz(message,epoch)
                            wait_started=time.monotonic()
                            wait_for_imu(cloud,arrival,snapshot,time.monotonic,time.sleep,stop.is_set)
                            waited=time.monotonic()-wait_started
                            row=pipeline.process(cloud,arrival,snapshot,time.monotonic)
                            row['imu_wait_s']=waited
                        except (ValueError,ArithmeticError) as error:
                            row={'status':'REJECTED','reason':'DECODE_'+str(error),'control_eligible':False}
                    row.update(processing_s=time.monotonic()-started)
                    output.write(json.dumps(row,allow_nan=False)+'\n');output.flush();rows.append(row)
        except Exception as error:
            with lock:faults.append('WORKER:'+str(error))
    thread=threading.Thread(target=worker,daemon=True);thread.start()
    graph=None;started=time.monotonic()
    try:
        while time.monotonic()-started<duration and not rospy.is_shutdown():
            with sensors.lock:
                if sensors.fault:raise RuntimeError(sensors.fault)
            with lock:
                if faults:raise RuntimeError(faults[0])
            time.sleep(.02)
    except Exception as error:
        with lock:faults.append(str(error))
    finally:
        stop.set();slot.close();thread.join(10.)
        if thread.is_alive():faults.append('WORKER_DID_NOT_STOP')
        try:
            pubs,subs,_=rosgraph.Master(rospy.get_name()).getSystemState();node=rospy.get_name()
            graph={'subscriptions':sorted(t for t,nodes in subs if node in nodes),
                   'publishers':sorted(t for t,nodes in pubs if node in nodes)}
            expected=sorted(['/airsim_node/drone_1/lidar','/airsim_node/drone_1/gps',
                             '/airsim_node/drone_1/imu/imu','/airsim_node/initial_pose'])
            if graph['subscriptions']!=expected or set(graph['publishers'])-{'/rosout'}:faults.append('GRAPH_POLICY')
        except Exception as error:faults.append('GRAPH_AUDIT:'+str(error))
        listener.unregister();sensors.close()
    summary={'faults':faults,'received':received,'slot':slot.stats(),'results':len(rows),
             'statuses':dict(Counter(row['status'] for row in rows)),
             'veto_reasons':dict(Counter(row['dynamic_check']['reason'] for row in rows if row['status']=='PROCESSED')),
             'node_graph':graph,'settings_sha256':rig.settings_sha256,'control_published':False,
             'debug_inputs_used':False,'control_eligible':False,'acceptance_eligible':False}
    (run/'dynamic_lidar_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    return 0 if rows and not faults else 2


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--duration',type=float,default=8.);p.add_argument('--settings',default='/workspace/main/configs/simulator_settings.json')
    a=p.parse_args()
    if not math.isfinite(a.duration) or not 0<a.duration<=30:p.error('duration must be in (0,30]')
    return observe(a.run_dir,a.duration,a.settings)


if __name__=='__main__':raise SystemExit(main())
