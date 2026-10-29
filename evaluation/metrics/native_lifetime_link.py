"""Link unmatched native snapshots to same-span borrows and same-address frees.

Address equality and event order cannot establish allocation identity, a downstream
read, or motor application; overlapping calls remain explicitly ambiguous.
"""
import argparse
from collections import defaultdict
import json
from pathlib import Path


def evaluate(records, native_audit):
    spans = defaultdict(list)
    frees = defaultdict(list)
    for record in records:
        if record.get('bridge_span'):
            spans[record['bridge_span']].append(record)
        if record.get('event') == 'parameter_free_end':
            frees[record['parameter_address']].append(record)
    findings = []
    for item in native_audit['unmatched']:
        entry = item['entry']
        rows = spans[entry['bridge_span']]
        borrows = [r for r in rows if r['event'] == 'parameter_borrow'
                   and r['parameter_address'] == entry['parameter_address'] != '0x0'
                   and r['end_monotonic_ns'] <= entry['start_monotonic_ns']]
        if len(borrows) != 1:
            findings.append({'bridge_span': entry['bridge_span'], 'classification': 'borrow_ambiguous'})
            continue
        borrow = borrows[0]
        same_address = [r for r in frees[entry['parameter_address']]
                        if r['start_monotonic_ns'] >= borrow['end_monotonic_ns']
                        and r['start_monotonic_ns'] <= entry['end_monotonic_ns']]
        completed = [r for r in same_address if r['end_monotonic_ns'] <= entry['start_monotonic_ns']]
        overlapping = [r for r in same_address if r['start_monotonic_ns'] <= entry['end_monotonic_ns']
                       and r['end_monotonic_ns'] > entry['start_monotonic_ns']]
        findings.append({'bridge_span': entry['bridge_span'], 'parameter_address': entry['parameter_address'],
                         'borrow': borrow, 'entry': entry, 'snapshot_values': item['snapshot_values'],
                         'finite_unit_range': item['finite_unit_range'],
                         'frees_completed_before_entry': completed,
                         'frees_overlapping_entry': overlapping,
                         'classification': ('free_completed_before_entry' if completed else
                                            'free_overlaps_entry' if overlapping else 'no_linked_free')})
    return {'unmatched_entries': len(findings), 'findings': findings,
            'allocation_identity_verified': False, 'downstream_consumption_verified': False,
            'physical_cause_identified': False,
            'interpretation': 'Same address and timing do not prove allocation identity; overlapping intervals do not establish free completion before the snapshot.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    with (args.run / 'bridge_probe.jsonl').open() as stream:
        records = [json.loads(line) for line in stream]
    audit = json.loads((args.run / 'native_input_audit.json').read_text())
    print(json.dumps(evaluate(records, audit), indent=2))


if __name__ == '__main__':
    main()
