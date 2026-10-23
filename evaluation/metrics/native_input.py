"""Offline native PWM entry snapshot matching, distinct from actuator consumption."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import struct


def evaluate(records, telemetry, control_end=None):
    sources=defaultdict(list)
    for row in telemetry:
        begin=row['publish_begin_wall_monotonic']
        end=row['publish_end_wall_monotonic']
        if not math.isfinite(begin) or not begin<=end:raise ValueError('invalid sender timing')
        sources[struct.pack('<4f',*row['command']['values']).hex()].append(
            {'sequence':row['command']['sequence'],'begin':begin,'end':end})
    entries=[r for r in records if r['event']=='native_pwm_entry']
    exits={r['bridge_span']:r for r in records if r['event']=='native_pwm_exit'}
    bindings=sum(r['event']=='native_callback_bound' for r in records)
    frees=defaultdict(list)
    for r in records:
        if r['event']=='parameter_free_end':frees[r['parameter_address']].append(r)
    posts={r['bridge_span']:r for r in records if r['event']=='native_pwm_post'}
    control_begin=min((r['publish_begin_wall_monotonic'] for r in telemetry),default=0.)
    end_basis='controller_end' if control_end is not None else 'last_publisher_call_end'
    if control_end is None:
        control_end=max((r['publish_end_wall_monotonic'] for r in telemetry),default=0.)
    matched=[];unmatched=[];unreadable=[];incomplete=0
    latency=[];active_entries=0;freed_during_call=[]
    for entry in entries:
        start=entry['start_monotonic_ns']*1e-9
        end=entry['end_monotonic_ns']*1e-9
        if not 0<=start<=end:raise ValueError('invalid native snapshot timing')
        close=exits.get(entry['bridge_span'])
        if close is None or close['start_monotonic_ns']<entry['end_monotonic_ns']:
            incomplete+=1
        active=control_begin<=start<=end<=control_end
        active_entries+=active
        if close and close.get('native_call_interval_measured'):
            for free in frees[entry['parameter_address']]:
                if (close['start_monotonic_ns']<=free['start_monotonic_ns']
                        and free['end_monotonic_ns']<=close['end_monotonic_ns']):
                    freed_during_call.append({'entry':entry,'native_call':close,'free':free,
                        'post_snapshot':posts.get(entry['bridge_span']),'in_control_window':active})
        if not entry['parameter_read_ok']:
            unreadable.append(entry)
            continue
        payload=entry['parameter_float32_hex']
        raw=bytes.fromhex(payload)
        if len(raw)!=16:raise ValueError('native snapshot must have four float32 fields')
        candidates=[r for r in sources[payload] if r['begin']<=end]
        if not candidates:
            values=struct.unpack('<4f',raw)
            unmatched.append({'entry':entry,'snapshot_values':[v if math.isfinite(v) else None for v in values],
                              'finite_unit_range':all(math.isfinite(v) and 0<=v<=1 for v in values)})
            continue
        source=max(candidates,key=lambda r:r['begin'])
        bounds=(start-source['end'],end-source['begin'])
        if active:latency.append(bounds)
        matched.append({'bridge_span':entry['bridge_span'],'source_sequence':source['sequence'],
                        'entry_monotonic_s':start,'parameter_address':entry['parameter_address'],
                        'publish_to_snapshot_bounds_s':list(bounds),'in_control_window':active})
    result={'native_entries':len(entries),'matched_entries':len(matched),'unmatched_entries':len(unmatched),
            'unreadable_entries':len(unreadable),'incomplete_native_calls':incomplete,'callback_bindings':bindings,
            'unmatched':unmatched,'unreadable':unreadable,'matched':matched,
            'snapshot_coverage_valid':bool(entries) and bindings==1 and not unreadable and not incomplete,
            'record_write_errors':max((r['write_errors'] for r in records),default=0),
            'control_window_monotonic_s':[control_begin,control_end],'control_window_end_basis':end_basis,
            'native_entries_in_control_window':active_entries,
            'native_calls_with_exact_intervals':sum(bool(r.get('native_call_interval_measured')) for r in exits.values()),
            'native_entries_after_control':sum(r['start_monotonic_ns']*1e-9>control_end for r in entries),
            'parameters_freed_during_original_callback':len(freed_during_call),
            'freed_during_original_callback':freed_during_call,
            'actual_actuator_consumption_verified':False,'physical_impulse_cause_identified':False,
            'acceptance_eligible':False,
            'interpretation':'Snapshot of four float32 fields at the original native callback entry, not the downstream payload read or motor application. Payloads are compared to sender float32 conversions published before the snapshot ended. Bounds use the publisher-call and snapshot sampling intervals on the same Linux monotonic clock; asynchronous send can overlap the publisher call.'}
    if latency:
        result['publish_to_snapshot_lower_min_s']=min(x[0] for x in latency)
        result['publish_to_snapshot_upper_max_s']=max(x[1] for x in latency)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args=parser.parse_args()
    def rows(name):return [json.loads(x) for x in (args.run/name).read_text().splitlines()]
    summary=json.loads((args.run/'pwm_summary.json').read_text())
    result=evaluate(rows('bridge_probe.jsonl'),rows('pwm_telemetry.jsonl'),summary.get('control_end_wall_monotonic'))
    (args.run/'native_input_audit.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('matched','unmatched','unreadable')},indent=2))
    return 0 if result['snapshot_coverage_valid'] and not result['record_write_errors'] else 2


if __name__=='__main__':raise SystemExit(main())
