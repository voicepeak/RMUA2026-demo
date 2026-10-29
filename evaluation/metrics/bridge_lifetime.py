"""Offline address/timing audit; no dereferences and no actuator-causality claim."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path


REQUIRED = ('bridge_begin', 'bridge_end', 'parameter_borrow', 'command_constructed',
            'command_handoff', 'parameter_free_begin', 'parameter_free_end')


def evaluate(records):
    counts = Counter(row['event'] for row in records)
    spans = defaultdict(list)
    frees = defaultdict(list)
    for row in records:
        if (row.get('mode') not in ('lifetime','native-input','native-serialize')
                or not 0 <= row['start_monotonic_ns'] <= row['end_monotonic_ns']):
            raise ValueError('invalid lifetime record')
        if row['bridge_span']:
            spans[row['bridge_span']].append(row)
        if row['event'] == 'parameter_free_end':
            frees[row['parameter_address']].append(row)
    complete, handoffs, incomplete = 0, 0, 0
    violations = []
    for span, rows in spans.items():
        begin = [r for r in rows if r['event'] == 'bridge_begin']
        end = [r for r in rows if r['event'] == 'bridge_end']
        borrowed = [r for r in rows if r['event'] == 'parameter_borrow' and r['parameter_address'] != '0x0']
        if len(begin) != 1 or len(end) != 1:
            incomplete += 1
            continue
        complete += 1
        for borrow in borrowed:
            commands = [r for r in rows if r['event'] == 'command_constructed'
                        and r['parameter_address'] == borrow['parameter_address']
                        and borrow['end_monotonic_ns'] <= r['start_monotonic_ns']]
            for command in commands:
                candidates = [r for r in rows if r['event'] == 'command_handoff'
                              and r['parameter_address'] == borrow['parameter_address']
                              and r['command_address'] == command['command_address'] != '0x0'
                              and r['thread_id'] == command['thread_id'] == borrow['thread_id']
                              and command['end_monotonic_ns'] <= r['start_monotonic_ns']
                              and begin[0]['start_monotonic_ns'] <= borrow['start_monotonic_ns']
                              and r['end_monotonic_ns'] <= end[0]['end_monotonic_ns']]
                for handoff in candidates:
                    handoffs += 1
                    for free in frees[borrow['parameter_address']]:
                        # Strict ordering excludes a free from an earlier reuse of
                        # this address and calls overlapping the get() interval.
                        if (borrow['end_monotonic_ns'] <= free['start_monotonic_ns']
                                and free['end_monotonic_ns'] <= handoff['start_monotonic_ns']):
                            violations.append({'bridge_span': span,
                                'parameter_address': borrow['parameter_address'],
                                'command_address': command['command_address'],
                                'borrow': borrow, 'free': free, 'command_handoff': handoff,
                                'free_to_handoff_ns': handoff['start_monotonic_ns'] - free['end_monotonic_ns']})
    return {'event_counts': {name: counts[name] for name in REQUIRED},
            'complete_bridge_spans': complete, 'incomplete_bridge_spans': incomplete,
            'linked_parameter_handoffs': handoffs,
            'freed_parameter_handoffs': len(violations), 'violations': violations,
            'hook_coverage_valid': all(counts[name] > 0 for name in REQUIRED) and handoffs > 0,
            'record_write_errors': max((r['write_errors'] for r in records), default=0),
            'actual_native_consumer_read_observed': False,
            'physical_impulse_cause_identified': False, 'acceptance_eligible': False,
            'interpretation': 'A violation means a parameter address was freed after get() returned and before its Command shared_ptr get() returned to the original native-call sequence. The probe does not observe the downstream payload read, allocator reuse, or applied actuator input. Instrumentation changes timing.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    records = [json.loads(line) for line in (args.run / 'bridge_probe.jsonl').read_text().splitlines()]
    result = evaluate(records)
    (args.run / 'bridge_lifetime_audit.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'violations'}, indent=2))
    return 0 if result['hook_coverage_valid'] and not result['record_write_errors'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
