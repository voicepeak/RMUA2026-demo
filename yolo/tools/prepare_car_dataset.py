#!/usr/bin/env python3
"""Snapshot car rectangles into YOLO detection, holding out time blocks."""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
from PIL import Image


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--src',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--runtime-path',required=True)
    args=parser.parse_args()
    rows=list(csv.DictReader((args.src/'manifest.csv').open()))
    # Consecutive validation blocks, rather than randomly interleaved views.
    val_ids=set(range(13,19))|set(range(46,52))
    val_rows=[r for r in rows if int(Path(r['file']).stem.split('_')[-1]) in val_ids]
    stats=collections.Counter();index=[]
    for row in rows:
        image=args.src/row['file'];annotation=image.with_suffix('.json')
        raw=annotation.read_bytes();data=json.loads(raw)
        w,h=Image.open(image).size
        if data['imageWidth']!=w or data['imageHeight']!=h:
            raise ValueError(f'Annotation dimensions differ: {annotation}')
        file_id=int(image.stem.split('_')[-1])
        split='val' if file_id in val_ids else 'train'
        group=row['source'].split('/')[1]
        if split=='train' and any(
            v['source'].split('/')[1]==group and
            abs(float(v['source_elapsed_seconds'])-float(row['source_elapsed_seconds']))<3.
            for v in val_rows):
            split='excluded_boundary'
        lines=[]
        for shape in data.get('shapes',[]):
            if shape['label']!='car' or shape['shape_type']!='rectangle':
                raise ValueError(f'Unexpected label/shape in {annotation}')
            points=shape['points']
            if len(points)<2 or not all(math.isfinite(v) for p in points for v in p):
                raise ValueError(f'Invalid rectangle in {annotation}')
            x0,x1=min(p[0] for p in points),max(p[0] for p in points)
            y0,y1=min(p[1] for p in points),max(p[1] for p in points)
            if not (0<=x0<x1<=w and 0<=y0<y1<=h):
                raise ValueError(f'Out of bounds rectangle in {annotation}')
            lines.append(f'0 {(x0+x1)/(2*w):.8f} {(y0+y1)/(2*h):.8f} {(x1-x0)/w:.8f} {(y1-y0)/h:.8f}')
        entry=dict(row,split=split,boxes=len(lines),annotation_sha256=hashlib.sha256(raw).hexdigest())
        index.append(entry);stats[split+'_images']+=1;stats[split+'_boxes']+=len(lines)
        snapshots=args.out/'annotations';snapshots.mkdir(parents=True,exist_ok=True)
        (snapshots/annotation.name).write_bytes(raw)
        if split=='excluded_boundary':continue
        for sub in ('images','labels'):(args.out/sub/split).mkdir(parents=True,exist_ok=True)
        shutil.copy2(image,args.out/'images'/split/image.name)
        (args.out/'labels'/split/(image.stem+'.txt')).write_text('\n'.join(lines)+'\n')
    (args.out/'data.yaml').write_text(f'path: {args.runtime_path}\ntrain: images/train\nval: images/val\nnames:\n  0: car\n')
    report=dict(stats=stats,index=index,split_policy='Two held-out time blocks; 3s guard; same-course pilot validation only')
    (args.out/'conversion_report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    print(json.dumps(dict(stats),ensure_ascii=False))


if __name__=='__main__':main()
