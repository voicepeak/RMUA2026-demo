"""Describe timing around independently audited missing PWM records, without inferring cause."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics


def analyze(telemetry, wires, audits):
    send = {row['command']['sequence']: row for row in telemetry}
    if len(send) != len(telemetry):
        raise ValueError('duplicate sender sequence')
    result = {'observers': [], 'missing_windows': []}
    maps = []
    for wire, audit in zip(wires, audits):
        offset = audit['ros_header_sequence_offset']
        if offset is None or audit['invalid_records'] or audit['duplicate_sequences'] or audit['mismatched_sequences'] or audit['unexpected_sequences'] or audit['sender_commands'] != len(telemetry) or audit['wire_records'] != len(wire):
            raise ValueError('wire audit is not suitable for timing analysis')
        observed = {row['data']['header']['seq'] - offset: row['arrival_wall_monotonic'] for row in wire}
        if set(send) - set(observed) != set(audit['missing_sequences']):
            raise ValueError('wire records disagree with audit')
        delays = sorted((arrival - send[seq]['publish_begin_wall_monotonic']) * 1000 for seq, arrival in observed.items())
        if not delays or delays[0] < 0:
            raise ValueError('empty or non-monotonic wire timing')
        maps.append(observed)
        result['observers'].append({'records': len(delays), 'median_ms': statistics.median(delays),
                                    'p95_nearest_lower_ms': delays[int(.95 * (len(delays) - 1))],
                                    'at_least_6ms': sum(x >= 6 for x in delays), 'max_ms': delays[-1]})
    for idx, audit in enumerate(audits):
        for seq in audit['missing_sequences']:
            if seq - 1 not in send or seq + 1 not in send:
                continue
            start = send[seq]['publish_begin_wall_monotonic']
            result['missing_windows'].append({'observer_index': idx, 'logical_sequence': seq,
                'previous_publish_begin_ms': (send[seq-1]['publish_begin_wall_monotonic'] - start)*1000,
                'next_publish_begin_ms': (send[seq+1]['publish_begin_wall_monotonic'] - start)*1000,
                'publish_call_ms': (send[seq]['publish_end_wall_monotonic'] - start)*1000,
                'other_observer_arrival_ms': (maps[1-idx][seq] - start)*1000 if seq in maps[1-idx] else None})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    files = ['pwm_telemetry.jsonl', 'evaluation_pwm_commands.jsonl', 'second_wire_commands.jsonl',
             'command_trace_audit.json', 'second_wire_audit.json']
    raw = [(args.run/name).read_bytes() for name in files]
    rows = [[json.loads(line) for line in data.splitlines()] for data in raw[:3]]
    audits = [json.loads(data) for data in raw[3:]]
    report = analyze(rows[0], rows[1:], audits)
    report['run_directory'] = str(args.run)
    report['input_sha256'] = {name: hashlib.sha256(data).hexdigest() for name, data in zip(files, raw)}
    report['analysis_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    report['limitations'] = 'Same-host callback arrival timing only; ROS queuing, transport and backend consumption are not observed. Existing strict wire audit remains authoritative.'
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
