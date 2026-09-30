#!/usr/bin/env python3
"""Train a separate car detector without replacing the gate detector."""
import argparse
import json
from pathlib import Path
import shutil
import torch
from ultralytics import YOLO


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--data',type=Path,required=True)
    parser.add_argument('--project',type=Path,required=True)
    parser.add_argument('--name',default='car_score91_yolo11n')
    parser.add_argument('--epochs',type=int,default=180)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    torch.set_num_threads(4)
    model=YOLO(str(root/'weights/yolo11n.pt'))
    model.train(data=str(args.data),project=str(args.project),name=args.name,
        epochs=args.epochs,patience=45,imgsz=960,batch=6,device=0,workers=2,
        optimizer='AdamW',lr0=.001,lrf=.01,weight_decay=.0005,
        seed=123,deterministic=True,amp=False,cache=False,
        mosaic=.5,close_mosaic=20,mixup=0.,degrees=5.,translate=.08,
        scale=.25,fliplr=.5,hsv_h=.01,hsv_s=.3,hsv_v=.3,
        plots=True,save=True)
    best=Path(model.trainer.save_dir)/'weights/best.pt'
    validation=YOLO(str(best)).val(data=str(args.data),imgsz=960,batch=6,device=0,
        project=str(args.project),name=args.name+'_validation',plots=True)
    destination=root/'weights/car_score91_best.pt'
    shutil.copy2(best,destination)
    summary=dict(weights=str(destination),training_directory=str(model.trainer.save_dir),
        metrics={k:float(v) for k,v in validation.results_dict.items()},
        names=validation.names,warning='Small same-course dataset; validation is not cross-seed generalization.')
    (Path(model.trainer.save_dir)/'deployment_summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':main()
