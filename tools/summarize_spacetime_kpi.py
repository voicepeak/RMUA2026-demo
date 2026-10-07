#!/usr/bin/env python3
"""Summarize recorded actual command timing separately from planner diagnostics."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np


def summarize(rows):
    commands=[r for r in rows if r.get('topic')=='command']
    diagnostics=[r['data'] for r in rows if r.get('topic')=='planning_debug']
    times=[r['received_monotonic'] for r in commands]
    intervals=np.diff(times)
    valid=intervals[intervals>=0.]
    costs=[r['compute_ms'] for r in diagnostics if 'compute_ms' in r]
    stats=lambda values:None if not len(values) else dict(mean=float(np.mean(values)),p95=float(np.percentile(values,95)),maximum=float(np.max(values)))
    return dict(actual_commands=len(commands),publication_interval_seconds=stats(valid),
        clock_resets=int(np.sum(intervals<0.)),executor_compute_ms=stats(costs),
        guard_reasons=dict(Counter(r.get('reason','UNKNOWN') for r in diagnostics)),
        execution_modes=dict(Counter(r.get('mode','UNKNOWN') for r in diagnostics)),
        unverified_bridge=any(r.get('reason')=='BRIDGE_CONTRACT_UNVERIFIED' for r in diagnostics),
        official_race_result='Not inferred from controller logs')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('trace',type=Path);parser.add_argument('--out',type=Path)
    args=parser.parse_args();result=summarize([json.loads(line) for line in args.trace.read_text().splitlines() if line.strip()])
    rendered=json.dumps(result,indent=2,allow_nan=False)+'\n'
    if args.out:args.out.write_text(rendered)
    print(rendered,end='')
