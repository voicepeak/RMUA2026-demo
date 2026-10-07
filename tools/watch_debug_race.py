#!/usr/bin/env python3
"""Capture hidden simulator HUD and stop finished/stalled diagnostic flights."""
import argparse
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import time
from PIL import Image
from race_monitor_policy import should_stop_controller_event

parser=argparse.ArgumentParser()
parser.add_argument('--out',type=Path,required=True)
parser.add_argument('--display',default=':2')
parser.add_argument('--stall-seconds',type=float,default=20.)
args=parser.parse_args()
out=args.out.resolve();hud=out/'hud';hud.mkdir(exist_ok=True)
env=dict(os.environ,DISPLAY=args.display)
stream=out/'flight/streams.jsonl'
offset=0;pending='';counter=0;finished_frames=0
last_s=None;best_s=None;progress_at=time.monotonic();last_telemetry=None
def stop(reason,**fields):
    if (out/'stop.json').exists():return
    payload=dict(reason=reason,wall_time=time.time(),official_result='UNKNOWN')
    payload.update(fields)
    (out/'stop.json').write_text(json.dumps(payload,indent=2)+'\n')
    print(json.dumps(payload),flush=True)

while True:
    if (out/'stop.json').exists():break
    now=time.monotonic()
    if stream.exists():
        with stream.open() as file:
            file.seek(offset);pending+=file.read();offset=file.tell()
        rows=pending.split('\n');pending=rows.pop()
        for line in rows:
            try:row=json.loads(line)
            except ValueError:continue
            data=row.get('data',{})
            if row.get('topic')=='events' and should_stop_controller_event(data):
                stop('CONTROLLER_TERMINATION',controller=data);raise SystemExit
            if row.get('topic')!='telemetry':continue
            last_telemetry=now;s=float(data['s'])
            if last_s is None or s<last_s-30.:
                best_s=s;progress_at=now
            elif s>best_s+.5:
                best_s=s;progress_at=now
            last_s=s
    try:
        listing=subprocess.run(['xwininfo','-root','-tree'],env=env,capture_output=True,
                               text=True,timeout=3).stdout
        match=re.search(r'(0x[0-9a-f]+) "RMUA\s*"',listing)
        if match:
            raw=hud/'latest.xwd'
            subprocess.run(['xwd','-silent','-id',match.group(1),'-out',str(raw)],
                           env=env,check=True,timeout=3)
            content=raw.read_bytes();header=struct.unpack('>25I',content[:100])
            image=Image.frombytes('RGB',(header[4],header[5]),content[header[0]+header[19]*12:],
                                  'raw','BGRX',header[12],1)
            filename='hud_%04d.png'%counter
            image.crop((max(0,image.width-255),0,image.width,320)).resize((510,640)).save(hud/filename)
            ocr_image=hud/'latest_ocr.png'
            Image.open(hud/filename).crop((110,20,510,550)).convert('L').point(
                lambda x:0 if x>230 else 255).save(ocr_image)
            container_image=Path('/workspace')/ocr_image.relative_to(Path(__file__).resolve().parents[2])
            ocr=subprocess.run(['docker','exec','rmua_noetic','tesseract',str(container_image),
                                'stdout','--psm','6'],capture_output=True,text=True,timeout=3).stdout
            with (hud/'captures.jsonl').open('a') as file:
                file.write(json.dumps(dict(file=filename,wall_time=time.time(),ocr=ocr))+'\n')
            counter+=1
            finished_frames=finished_frames+1 if re.search(r'\bFinished\b',ocr,re.I) else 0
            if finished_frames>=2:
                stop('HUD_FINISHED',hud=filename,ocr=ocr,official_result='FINISHED_OBSERVED');break
    except (subprocess.SubprocessError,OSError,ValueError) as error:
        print(str(error),flush=True)
    if last_s is not None and now-progress_at>args.stall_seconds:
        stop('DEBUG_PROGRESS_STALLED',local_s=last_s,best_s=best_s,
             seconds_without_progress=now-progress_at);break
    if last_telemetry is not None and now-last_telemetry>15.:
        stop('CONTROL_TELEMETRY_LOST',local_s=last_s);break
    time.sleep(1.)
