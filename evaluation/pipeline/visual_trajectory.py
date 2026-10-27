"""Form local VO segments first; evaluate them against isolated truth afterward."""
import argparse
import bisect
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from estimation.geometry import rotation
from estimation.history import slerp
from perception.visual_odometry.trajectory import VisualTrajectory


def build(estimates):
    segments = []; breaks = []; chain = None
    for row in estimates['results']:
        if row['status'] != 'ESTIMATED':
            breaks.append({'previous_stamp':row['previous_stamp'], 'current_stamp':row['current_stamp'],
                           'reason':row.get('reason', 'MOTION_REJECTED')})
            if chain is not None: chain.invalidate()
            chain = None
            continue
        motion = SimpleNamespace(**row['motion'])
        if chain is None:
            chain = VisualTrajectory(motion.previous_stamp, motion.epoch)
            segments.append({'anchor_stamp': motion.previous_stamp, 'epoch': motion.epoch, 'poses': []})
        try:
            pose = chain.append(motion)
        except ValueError as error:
            breaks.append({'previous_stamp':motion.previous_stamp, 'current_stamp':motion.current_stamp,
                           'reason':str(error)})
            chain = None
            continue
        segments[-1]['poses'].append(asdict(pose))
    return {'source': estimates['source'], 'segments': segments, 'breaks': breaks, 'truth_accessed': False,
            'control_eligible': False, 'metric_calibration_verified': False}


def evaluate(estimate, truth_path):
    path = Path(truth_path)
    records = [json.loads(line) for line in path.read_text().splitlines()]
    truth = sorted({r['stamp']: r for r in records if r['stamp'] > 0}.values(), key=lambda r: r['stamp'])
    stamps = [r['stamp'] for r in truth]
    def interpolate(stamp):
        i = bisect.bisect_left(stamps, stamp)
        if i == 0 or i == len(stamps): return None
        a, b = truth[i-1], truth[i]; span = b['stamp'] - a['stamp']
        if span <= 0 or span > .05: return None
        f = (stamp-a['stamp'])/span
        return (1-f)*np.asarray(a['position'])+f*np.asarray(b['position']), rotation(slerp(a['orientation'],b['orientation'],f))
    results = []
    for segment in estimate['segments']:
        anchor = interpolate(segment['anchor_stamp']); comparisons = []
        for pose in segment['poses']:
            current = interpolate(pose['stamp'])
            if anchor is None or current is None: continue
            relative = anchor[1].T @ (current[0]-anchor[0])
            error = float(np.linalg.norm(np.asarray(pose['position_initial_body_m'])-relative))
            comparisons.append({'stamp':pose['stamp'],'elapsed_s':pose['stamp']-segment['anchor_stamp'],
                                'translation_error_m':error})
        results.append({'anchor_stamp':segment['anchor_stamp'],'steps':len(segment['poses']), 'comparisons':comparisons})
    return {'source':estimate['source'], 'truth_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'role':'isolated_post_estimate_evaluation', 'segments':results, 'control_eligible':False}


def main():
    p = argparse.ArgumentParser(); sub = p.add_subparsers(dest='command',required=True)
    b = sub.add_parser('build'); b.add_argument('estimate'); b.add_argument('output')
    e = sub.add_parser('evaluate'); e.add_argument('estimate'); e.add_argument('truth'); e.add_argument('output')
    a = p.parse_args(); source = json.loads(Path(a.estimate).read_text())
    result = build(source) if a.command == 'build' else evaluate(source,a.truth)
    with Path(a.output).open('x') as out: json.dump(result,out,indent=2,allow_nan=False);out.write('\n')
    print(json.dumps({'output':a.output,'segments':len(result['segments'])}))


if __name__ == '__main__': main()
