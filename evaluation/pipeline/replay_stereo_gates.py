"""Offline gate proposals from retained official flight stereo images."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import cv2

from interfaces.contracts import Metadata
from perception.gate_geometry.stereo_gate import stereo_gate_candidates, prepare_gate_edges
from perception.gate_geometry.roi_disparity import gate_region_disparity
from perception.stereo_depth.geometry import load_nominal_rig
from perception.stereo_depth.pairing import ImageFrame, StereoPair
from perception.stereo_depth.sgbm import compute_diagnostic_disparity


def replay(source, settings, stride=5, depth_method="full", reuse_edges=False):
    source = Path(source); directory = source / 'scene_trace'
    index = json.loads((directory / 'index.json').read_text())
    rig = load_nominal_rig(settings)
    manifest = json.loads((source / 'manifest.json').read_text())
    if manifest['simulator_preflight']['settings_sha256'] != rig.settings_sha256:
        raise ValueError('settings do not match recorded capture')
    if index['fault'] is not None: raise ValueError('scene recorder failed')
    sides = {side: sorted((r for r in index['frames'] if r['sensor'] == 'front_' + side),
                          key=lambda row: row['stamp']) for side in ('left', 'right')}
    used = set(); rows = []
    for left in sides['left'][::stride]:
        choices = [(abs(left['stamp'] - r['stamp']), i, r) for i, r in enumerate(sides['right']) if i not in used]
        if not choices: continue
        skew, index_right, right = min(choices, key=lambda v:v[0])
        if skew > .005:
            rows.append({'left': left['file'], 'status': 'UNPAIRED', 'skew_s': skew}); continue
        used.add(index_right); frames=[]; hashes={}
        for side, row in (('left', left), ('right', right)):
            path = directory / row['file']
            if path.parent != directory or path.suffix != '.png': raise ValueError('invalid image path')
            image=cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None: raise ValueError('image read failed')
            hashes[side]=hashlib.sha256(path.read_bytes()).hexdigest()
            meta=Metadata(row['stamp'],row['stamp']+.2,0,'retained_'+side,'front_'+side+'_image')
            frames.append(ImageFrame(meta,'bgr8',image))
        stamp=max(x.metadata.stamp for x in frames)
        pair=StereoPair(Metadata(stamp,min(x.metadata.valid_until for x in frames),0,'offline_pair','front_stereo'),
                        frames[0],frames[1],skew)
        started=time.monotonic(); prepared=prepare_gate_edges(pair) if reuse_edges else None
        depth=(gate_region_disparity(pair,prepared=prepared) if depth_method=="roi" else compute_diagnostic_disparity(pair))
        # Query at capture time solely to test geometry; do not claim real-time output.
        gates=stereo_gate_candidates(pair,depth,rig,stamp,0,prepared=prepared)
        rows.append({'left':left['file'],'right':right['file'],'input_sha256':hashes,'skew_s':skew,
                     'status':'PROCESSED','gate_count':len(gates),'candidates':[asdict(g) for g in gates],
                     'valid_disparity_fraction':float(depth.valid.mean()),'elapsed_s':time.monotonic()-started,
                     'query_at_capture_time_not_output_time':True})
    return {'source':str(source),'depth_method':depth_method,'reuse_edges':reuse_edges,'role':'offline_nominal_gate_proposal','settings_sha256':rig.settings_sha256,
            'index_sha256':hashlib.sha256((directory/'index.json').read_bytes()).hexdigest(),
            'control_published':False,'acceptance_eligible':False,'results':rows}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('source');parser.add_argument('output')
    parser.add_argument('--settings',default='configs/simulator_settings.json');parser.add_argument('--stride',type=int,default=5)
    parser.add_argument('--depth-method',choices=('full','roi'),default='full')
    parser.add_argument('--reuse-edges',action='store_true')
    args=parser.parse_args()
    if not 1 <= args.stride <= 20: parser.error('stride must be 1..20')
    cv2.setNumThreads(1)
    result=replay(args.source,args.settings,args.stride,args.depth_method,args.reuse_edges)
    with Path(args.output).open('x') as out:json.dump(result,out,indent=2,allow_nan=False);out.write('\n')
    print(json.dumps({'output':args.output,'frames':len(result['results']),
                      'candidate_counts':[r.get('gate_count') for r in result['results']]}))


if __name__ == '__main__':main()
