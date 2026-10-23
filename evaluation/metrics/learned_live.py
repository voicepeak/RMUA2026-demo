"""Independently audit live diagnostic source leases, payloads and pipeline timing."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def rows(path):return [json.loads(line) for line in path.read_text().splitlines()]


def describe(values):
    return {'samples':len(values),'min':float(min(values)),'p50':float(np.median(values)),
        'sample_p95':float(np.percentile(values,95)),'sample_p99':float(np.percentile(values,99)),
        'max':float(max(values))} if values else {'samples':0}


def audit(run):
    from cv_bridge import CvBridge
    from adapters.ros_noetic.capture_io import load_message

    summary=json.loads((run/'online_summary.json').read_text())
    results=rows(run/'online_results.jsonl');jobs=rows(run/'gpu_jobs.jsonl')
    clocks=rows(run/'online_imu_clock.jsonl')
    clock_set={(r['stamp'],r['arrival_wall'],r['epoch']) for r in clocks}
    valid=not summary['faults'] and len(results)==summary['results']==len(jobs)
    ids=[r['request_id'] for r in results]
    valid &= ids==sorted(set(ids)) and ids==[j['request_id'] for j in jobs]
    slot=summary['slot']
    valid &= (slot['capacity']==1 and slot['pending']==0 and slot['submitted']==summary['pairs']
        and slot['submitted']==slot['taken']+slot['superseded']+slot['discarded_on_close']
        and slot['taken']==len(results)+sum(summary['admission_rejections'].values()))
    per_result=[]
    for row,job in zip(results,jobs):
        meta=row['source_metadata'];a,b,p=meta['left'],meta['right'],meta['pair']
        same_job=all(row[key]==value for key,value in job.items())
        source_valid=(p['stamp']==max(a['stamp'],b['stamp']) and p['valid_until']==min(a['valid_until'],b['valid_until'])
            and a['epoch']==b['epoch']==p['epoch'] and abs(a['stamp']-b['stamp'])<=.005
            and all(abs(x['valid_until']-x['stamp']-summary['image_age_budget_s'])<1e-6 for x in (a,b)))
        clock_valid=(row['imu_clock_at_result'],row['imu_arrival_at_result'],row['epoch_at_result']) in clock_set
        checked=row['result_checked_wall'];now=row['imu_clock_at_result']
        fresh=(0<=checked-row['imu_arrival_at_result']<=.1)
        source_arrival=min(row['left_arrival_wall'],row['right_arrival_wall'])
        if not row['clock_fresh_at_result']:reason='STALE_IMU_CLOCK'
        elif row['epoch_at_result']!=p['epoch']:reason='OTHER_EPOCH'
        elif not p['stamp']<=now<p['valid_until']:reason='SOURCE_LEASE_NOT_CURRENT'
        elif not 0<=checked-source_arrival<.2:reason='WALL_EXPIRED'
        else:reason=None
        timing_valid=(source_arrival<=row['send_begin_wall']<=row['received_wall']<=row['processing_end_wall']
            <=row['result_received_wall']<=checked and reason==row['timing_rejection']
            and (reason is None)==row['timing_current'] and row['clock_fresh_at_result']==fresh
            and abs(row['sensor_age_at_result_s']-(now-min(a['stamp'],b['stamp'])))<1e-9
            and abs(row['wall_age_at_result_s']-(checked-source_arrival))<1e-9
            and p['stamp']<=row['imu_clock_at_dispatch']<p['valid_until'])
        row_valid=bool(same_job and source_valid and clock_valid and timing_valid
            and not row['current_control_eligible'] and not row['control_published'] and not row['acceptance_eligible'])
        valid &= row_valid;per_result.append({'request_id':row['request_id'],'audit_valid':row_valid})
    first_payload_valid=None
    if results:
        directory=run/'first_online';request=json.loads((directory/'request.json').read_text())
        first_payload_valid=True
        with np.load(directory/'rgb_inputs.npz',allow_pickle=False) as images:
            payload=images['left'].tobytes()+images['right'].tobytes()
            first_payload_valid &= hashlib.sha256(payload).hexdigest()==request['input_sha256']==results[0]['input_sha256']
            for side in ('left','right'):
                message=load_message(directory/(side+'.json'))
                first_payload_valid &= np.array_equal(CvBridge().imgmsg_to_cv2(message,desired_encoding='rgb8'),images[side])
                first_payload_valid &= message.header.stamp.to_sec()==request['source_metadata'][side]['stamp']
        with np.load(directory/'prediction.npz',allow_pickle=False) as saved:
            disp,mask=saved['disparity'],saved['valid']
            first_payload_valid &= bool(np.isnan(disp[~mask]).all() and np.isfinite(disp[mask]).all() and (disp[mask]>0).all())
            payload=disp.astype('<f4').tobytes()+mask.astype(np.uint8).tobytes()
            first_payload_valid &= hashlib.sha256(payload).hexdigest()==results[0]['output_sha256']
        valid &= first_payload_valid
    graph=summary['node_graph'] or {}
    expected=sorted(('/airsim_node/drone_1/front_left/Scene','/airsim_node/drone_1/front_right/Scene','/airsim_node/drone_1/imu/imu'))
    valid &= graph.get('subscriptions')==expected and not set(graph.get('publishers',[]))-{'/rosout'}
    valid &= not summary['control_published'] and len(clocks)==summary['clock_records']
    gpu=rows(run/'gpu_system_usage.jsonl')
    utilization=[];memory=[];cpu=[]
    for sample in gpu:
        if sample['gpu_exit_code']==0:
            fields=[part.strip() for part in sample['gpu_csv'].split(',')]
            if len(fields)==4:
                try:utilization.append(float(fields[1]));memory.append(float(fields[2]))
                except ValueError:pass
    for a,b in zip(gpu,gpu[1:]):
        delta=np.array(b['system_cpu_ticks'])-a['system_cpu_ticks'];total=delta.sum()
        if total>0 and (delta>=0).all():cpu.append(float(100*(total-delta[3]-delta[4])/total))
    intervals=np.diff([r['arrival_wall'] for r in clocks])
    result={'audit_valid':bool(valid),'result_count':len(results),'per_result':per_result,
        'first_payload_matches_independent_ros_decoder_and_output_digest':None if first_payload_valid is None else bool(first_payload_valid),
        'timing_current_results':sum(r['timing_current'] for r in results),
        'result_timing_rejections':{reason:sum(r['timing_rejection']==reason for r in results) for reason in sorted({r['timing_rejection'] for r in results if r['timing_rejection']})},
        'sensor_header_age_at_result_s':describe([r['sensor_age_at_result_s'] for r in results]),
        'callback_to_checked_result_s':describe([r['wall_age_at_result_s'] for r in results]),
        'rpc_round_trip_s':describe([r['result_received_wall']-r['send_begin_wall'] for r in results]),
        'gpu_receive_to_encoded_result_s':describe([r['processing_end_wall']-r['received_wall'] for r in results]),
        'bidirectional_model_s':describe([r['bidirectional_resize_transfer_s'] for r in results]),
        'imu_callback_interval_s':describe(intervals.tolist()),
        'gpu_utilization_percent':describe(utilization),'gpu_memory_used_mib':describe(memory),
        'system_cpu_busy_percent_all_cores':describe(cpu),'usage_sampling_period_s':.5,
        'graph':graph,'slot':slot,'image_age_budget_s':summary['image_age_budget_s'],
        'all_outputs_diagnostic_only':True,'true_exposure_latency_measured':False,
        'actuation_latency_measured':False,'cold_start_departure_verified':False,
        'geometry_accuracy_verified':False,'current_control_eligible':False,'acceptance_eligible':False,
        'interpretation':'Image-header/IMU and callback/monotonic ages measure this diagnostic pipeline, not exposure-to-actuation. No source lease is refreshed. Late results remain rejected; timely ones are still uncalibrated. Samples are development observations and not statistical real-time guarantees.'}
    (run/'online_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();return 0 if audit(args.run_dir)['audit_valid'] else 2


if __name__=='__main__':raise SystemExit(main())
