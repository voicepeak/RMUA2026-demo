"""Read-only asynchronous stereo pipeline; late outputs remain diagnostic only."""
import argparse
from collections import Counter,OrderedDict
from dataclasses import asdict
import json
import math
from pathlib import Path
import resource
import socket
import threading
import time

import numpy as np

from adapters.ros_noetic.capture_io import save_message
from adapters.ros_noetic.stereo_images import decode_image
from adapters.ros_noetic.stereo_observer import TOPICS
from estimation.clock import SensorClock
from perception.stereo_depth.learned import rgb_pixels
from perception.stereo_depth.pairing import StereoPairBuffer
from perception.stereo_depth.transport import (LatestSlot,send_packet,receive_packet,encode_images,
    decode_disparity,digest,freshness_reason)


def observe(run,duration,image_ttl):
    import rosgraph
    import rospy
    from sensor_msgs.msg import Image,Imu

    rospy.init_node('rmua_learned_stereo_observer',anonymous=True,disable_signals=True)
    lock=threading.Lock();clock=SensorClock();pending={};received=Counter();overwritten=Counter()
    fault=[];slot=LatestSlot();stop=threading.Event();results=[];admission=Counter();clock_rows=[]
    def receive(side,message):
        arrival=time.monotonic()
        with lock:
            received[side]+=1
            if side=='imu':
                try:
                    if clock.observe(message.header.stamp.to_sec(),arrival):fault.append('IMU_CLOCK_RESET')
                    clock_rows.append((clock.stamp,arrival,clock.epoch))
                except ValueError as error:fault.append(str(error))
            else:
                if side in pending:overwritten[side]+=1
                pending[side]=(message,arrival)
    def clock_snapshot():
        with lock:
            return clock.stamp,clock.epoch,clock.fresh(time.monotonic()) and not fault,clock.arrival
    listeners=[rospy.Subscriber(topic,Imu if side=='imu' else Image,
        lambda msg,s=side:receive(s,msg),queue_size=1,buff_size=4000000) for side,topic in TOPICS.items()]
    connection=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);connection.settimeout(10.)
    connection.connect(str(run/'learned.sock'))
    def consume():
        try:
            with (run/'online_results.jsonl').open('x') as output:
                while not stop.is_set():
                    item=slot.take()
                    if item is None:time.sleep(.002);continue
                    request_id,pair,sources=item
                    now,epoch,fresh,_=clock_snapshot();wall=time.monotonic()
                    source_arrival=min(x[1] for x in sources.values())
                    reason=freshness_reason(pair.metadata,now,epoch,fresh,source_arrival,wall)
                    if reason:admission[reason]+=1;continue
                    source_meta={'pair':asdict(pair.metadata),'left':asdict(pair.left.metadata),'right':asdict(pair.right.metadata)}
                    build_begin=time.monotonic()
                    payload=encode_images(rgb_pixels(pair.left),rgb_pixels(pair.right))
                    header={'command':'predict','request_id':request_id,'image_shape':[720,960,3],
                        'source_metadata':source_meta,'input_sha256':digest(payload)}
                    first=len(results)==0
                    # Conversion and serialization consume the original lease too.
                    now,epoch,fresh,_=clock_snapshot()
                    reason=freshness_reason(pair.metadata,now,epoch,fresh,source_arrival,time.monotonic())
                    if reason:
                        admission['AFTER_BUILD_'+reason]+=1
                        continue
                    header['imu_clock_at_dispatch']=now
                    sent=time.monotonic();send_packet(connection,header,payload)
                    response,body=receive_packet(connection);returned=time.monotonic()
                    if (response['request_id']!=request_id or response['source_metadata']!=json.loads(json.dumps(source_meta))
                            or response['input_sha256']!=header['input_sha256'] or digest(body)!=response['output_sha256']
                            or response['current_control_eligible'] or response['acceptance_eligible']):
                        raise RuntimeError('GPU response differs from original observation identity')
                    disparity,valid=decode_disparity(body)
                    now,epoch,fresh,imu_arrival=clock_snapshot();checked=time.monotonic()
                    reason=freshness_reason(pair.metadata,now,epoch,fresh,source_arrival,checked)
                    row=dict(response,source_metadata=source_meta,build_begin_wall=build_begin,send_begin_wall=sent,
                        result_received_wall=returned,result_checked_wall=checked,
                        left_arrival_wall=sources['left'][1],right_arrival_wall=sources['right'][1],
                        imu_clock_at_dispatch=header.get('imu_clock_at_dispatch',None),
                        imu_clock_at_result=now,imu_arrival_at_result=imu_arrival,epoch_at_result=epoch,
                        clock_fresh_at_result=fresh,timing_current=reason is None,timing_rejection=reason,
                        sensor_age_at_result_s=now-min(pair.left.metadata.stamp,pair.right.metadata.stamp),
                        wall_age_at_result_s=checked-source_arrival,
                        valid_pixels=int(valid.sum()),callback_to_result_s=returned-source_arrival,
                        rpc_includes_returned_disparity=True,control_published=False,
                        exposure_sync_verified=False,metric_calibration_verified=False)
                    if first:
                        saved=run/'first_online';saved.mkdir()
                        for side,(message,_) in sources.items():save_message(saved,side,message,'image')
                        np.savez(saved/'rgb_inputs.npz',left=rgb_pixels(pair.left),right=rgb_pixels(pair.right))
                        np.savez_compressed(run/'first_online'/'prediction.npz',disparity=disparity,valid=valid)
                        (run/'first_online'/'request.json').write_text(json.dumps(header,indent=2)+'\n')
                    output.write(json.dumps(row,allow_nan=False)+'\n');output.flush();results.append(row)
                send_packet(connection,{'command':'stop'},b'')
        except Exception as error:
            with lock:fault.append('INFERENCE_TRANSPORT:'+str(error))
    thread=threading.Thread(target=consume,daemon=True);thread.start()
    buffer=StereoPairBuffer();originals={side:OrderedDict() for side in ('left','right')}
    started=time.monotonic();pairs=0;graph=None
    try:
        while time.monotonic()-started<duration and not rospy.is_shutdown():
            with lock:
                batch=dict(pending);pending.clear();errors=list(fault)
            if errors:raise RuntimeError(errors[0])
            now,epoch,fresh,_=clock_snapshot()
            if now is None:
                if time.monotonic()-started>5:raise RuntimeError('NO_IMU_CLOCK')
                time.sleep(.002);continue
            if not fresh:raise RuntimeError('STALE_IMU_CLOCK')
            for side,(message,arrival) in batch.items():
                frame=decode_image(message,side,epoch,image_ttl)
                if buffer.add(side,frame,arrival):
                    originals[side][frame.metadata.stamp]=(message,arrival)
                while len(originals[side])>8:originals[side].popitem(last=False)
            pair=buffer.pop(now,time.monotonic())
            if pair is not None:
                sources={side:originals[side][getattr(pair,side).metadata.stamp] for side in originals}
                slot.put((pairs,pair,sources));pairs+=1
            time.sleep(.002)
    except Exception as error:
        with lock:fault.append(str(error))
    finally:
        selection_ended=time.monotonic()
        stop.set();slot.close();thread.join(12.)
        if thread.is_alive():fault.append('TRANSPORT_THREAD_DID_NOT_STOP')
        connection.close()
        try:
            pubs,subs,_=rosgraph.Master(rospy.get_name()).getSystemState();node=rospy.get_name()
            graph={'subscriptions':sorted(t for t,nodes in subs if node in nodes),
                'publishers':sorted(t for t,nodes in pubs if node in nodes)}
            if graph['subscriptions']!=sorted(TOPICS.values()) or set(graph['publishers'])-{'/rosout'}:
                fault.append('LEARNED_OBSERVER_GRAPH_POLICY')
        except Exception as error:fault.append('GRAPH_AUDIT:'+str(error))
        for listener in listeners:listener.unregister()
        with (run/'online_imu_clock.jsonl').open('x') as output:
            for stamp,arrival,epoch in clock_rows:
                output.write(json.dumps({'stamp':stamp,'arrival_wall':arrival,'epoch':epoch})+'\n')
        summary={'faults':fault,'pairs':pairs,'results':len(results),'received':dict(received),
            'selection_started_wall':started,'selection_ended_wall':selection_ended,
            'result_drain_ended_wall':time.monotonic(),
            'raw_pending_overwritten':dict(overwritten),'pairing_events':dict(buffer.events),
            'slot':slot.stats(),'admission_rejections':dict(admission),'node_graph':graph,
            'image_age_budget_s':image_ttl,'clock_records':len(clock_rows),
            'max_rss_kib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            'timing_current_results':sum(row['timing_current'] for row in results),
            'control_published':False,'current_control_eligible':False,'acceptance_eligible':False,
            'exposure_sync_verified':False,'metric_calibration_verified':False,
            'raw_recording_is_lossy':True,'processing_queue_capacity':1,'in_flight_limit':1}
        (run/'online_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    return 0 if pairs and not fault else 2


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=10.)
    parser.add_argument('--image-age-budget-ms',type=int,choices=(150,200),default=150)
    args=parser.parse_args()
    if not math.isfinite(args.duration) or not 0<args.duration<=30:parser.error('duration must be in (0,30]')
    return observe(args.run_dir,args.duration,args.image_age_budget_ms/1000.)


if __name__=='__main__':raise SystemExit(main())
