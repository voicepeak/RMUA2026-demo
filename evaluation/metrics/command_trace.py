"""Compare the independent ROS subscriber record with the sender's command log.

Wire agreement proves serialization/observation, not consumption by a motor.
Debug PWM is checked for observability; it is not assumed to be actual force.
"""
import argparse
import json
import hashlib
from pathlib import Path
import struct
from collections import Counter
import numpy as np

MD5='4967432b608c06d70b17be20124fa258'


def decode(row):
    if row.get('md5sum')!=MD5:return None
    try:
        data=row['data'];header=data['header'];stamp=header['stamp']
        return {'sequence':header['seq'],'stamp':stamp['secs']+stamp['nsecs']*1e-9,
                'values':tuple(data['rotorPWM'+str(i)] for i in range(4))}
    except (KeyError,TypeError):return None


def packed(values):return struct.pack('<4d',*values)


def evaluate(telemetry,wire,debug):
    decoded=[decode(x) for x in wire]
    invalid=sum(x is None for x in decoded)
    expected={r['command']['sequence']:r['command'] for r in telemetry}
    # rospy assigns Header.seq during serialization; it is distinct from the
    # sender's logical guard counter. Infer a single offset only from matching
    # measurement stamps AND bit-exact official float64 payloads, then audit every row.
    offsets=Counter(remote['sequence']-seq for remote in decoded if remote is not None
        for seq,command in expected.items() if abs(remote['stamp']-command['stamp'])<=1e-6
        and packed(remote['values'])==packed(command['values']))
    best=offsets.most_common(2)
    offset=best[0][0] if best and (len(best)==1 or best[0][1]>best[1][1]) else None
    observed={}
    duplicates=0
    for row in decoded:
        if row is None:continue
        seq=row['sequence']-(offset if offset is not None else 0)
        if seq in observed:duplicates+=1
        else:observed[seq]=row
    matched=0;missing=[];mismatch=[]
    for seq,command in expected.items():
        remote=observed.get(seq)
        if remote is None:missing.append(seq)
        elif (abs(remote['stamp']-command['stamp'])>1e-6
                or packed(remote['values'])!=packed(command['values'])):mismatch.append(seq)
        else:matched+=1
    extra=sorted(set(observed)-set(expected))
    result={'sender_commands':len(telemetry),'wire_records':len(wire),'matched_commands':matched,
            'wire_match_fraction':matched/len(telemetry) if telemetry else 0.,
            'missing_sequences':missing,'mismatched_sequences':mismatch,'unexpected_sequences':extra,
            'invalid_records':invalid,'duplicate_sequences':duplicates,'actuator_consumption_verified':False,
            'debug_pwm_is_verified_actual_motor_output':False,'official_result':'UNKNOWN'}
    result['ros_header_sequence_offset']=offset
    result['wire_trace_valid']=(bool(telemetry) and matched==len(telemetry)
                                and offset is not None and not (invalid or duplicates or extra))
    debug_rows=sorted((r for r in (decode(x) for x in debug) if r is not None),key=lambda r:r['stamp'])
    stamps=np.array(sorted(set(x['stamp'] for x in debug_rows)))
    if len(stamps)>1:result['debug_message_stamp_interval_median_s']=float(np.median(np.diff(stamps)))
    changes=[];previous=None
    for row in debug_rows:
        if previous is None or np.max(np.abs(np.array(row['values'])-previous))>1e-7:
            changes.append(row['stamp']);previous=np.array(row['values'])
    if len(changes)>1:result['observed_debug_value_change_interval_p10_p50_p90_s']=np.percentile(np.diff(changes),[10,50,90]).tolist()
    pulses=[]
    for i,row in enumerate(telemetry):
        if not (row.get('collective_pulse_applied') or row.get('angular_pulse_applied')):continue
        command=row['command'];end=telemetry[i+1]['command']['stamp'] if i+1<len(telemetry) else command['stamp']+.02
        candidates=[r for r in debug_rows if command['stamp']-1e-6<=r['stamp']<=end+.08]
        pulses.append({'sequence':command['sequence'],'stamp':command['stamp'],'values':command['values'],
            'next_command_after_sensor_s':end-command['stamp'],'debug_samples_in_window':len(candidates),
            # Debug values may be float32 internally before their float64 wire
            # serialization. Do not confuse quantization with a missed pulse.
            'debug_samples_matching_pulse':int(sum(np.max(np.abs(np.array(r['values'])-command['values']))<=1e-6
                                              for r in candidates))})
    result['experiment_pulses']=pulses
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    args=parser.parse_args()
    def rows(name):return [json.loads(x) for x in (args.run/name).read_text().splitlines()]
    report=evaluate(rows('pwm_telemetry.jsonl'),rows('evaluation_pwm_commands.jsonl'),rows('evaluation_actuation.jsonl'))
    analysis_hash=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report['analysis_code_sha256']=analysis_hash
    manifest=args.run/'manifest.json'
    if manifest.exists():
        original=json.loads(manifest.read_text()).get('source_sha256_before',{}).get('evaluation/metrics/command_trace.py')
        report['analysis_matches_run_source_snapshot']=original==analysis_hash
    (args.run/'command_trace_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if report['wire_trace_valid'] else 2


if __name__=='__main__':raise SystemExit(main())
