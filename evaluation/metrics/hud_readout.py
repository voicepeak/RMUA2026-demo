"""Extract provisional HUD text after flight; retain image and raw OCR for review."""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path
from PIL import Image


def parse_text(text):
    def one(pattern, convert):
        values=re.findall(pattern,text,re.MULTILINE)
        return convert(values[0]) if len(values)==1 else None
    return {'score':one(r'^\s*Score\s*:\s*(\d+)\s*$',int),
            'time_s':one(r'^\s*Time\s*:\s*(\d+(?:\.\d+)?)\s*$',float),
            'state':one(r'^\s*State\s*:\s*([A-Za-z][A-Za-z0-9]*)\s*$',str)}


def readout(run, container):
    root=Path(__file__).resolve().parents[2];run=Path(run).resolve()
    relative=run.relative_to(root/'runs')
    if not relative.parts or not re.fullmatch(r'[a-zA-Z0-9_.-]+',container):
        raise ValueError('invalid run or container')
    hud=run/'hud'; index=json.loads((hud/'index.json').read_text())
    if index.get('fault') is not None or not index.get('frames'):
        raise ValueError('HUD capture incomplete')
    frame=index['frames'][-1]; name=frame['file']
    if not re.fullmatch(r'hud_\d{4}\.png',name): raise ValueError('invalid frame name')
    image=hud/name; output=hud/'readout_input.png'
    if output.exists() or (hud/'readout.json').exists():raise FileExistsError('readout already exists')
    Image.open(image).resize((510,640)).crop((110,20,510,550)).convert('L').point(
        lambda x:0 if x>230 else 255).save(output)
    container_path='/workspace/main/'+output.relative_to(root).as_posix()
    command=['docker','exec',container,'tesseract',container_path,'stdout','--psm','6']
    result=subprocess.run(command,capture_output=True,text=True,timeout=10,check=True)
    version=subprocess.run(['docker','exec',container,'tesseract','--version'],capture_output=True,text=True,timeout=10,check=True).stdout.splitlines()[0]
    payload={'source':name,'source_sha256':hashlib.sha256(image.read_bytes()).hexdigest(),
             'capture_wall_time':frame['wall_time'],'ocr_engine':version,'raw_ocr':result.stdout,
             'provisional':parse_text(result.stdout),'official_result':'UNKNOWN',
             'acceptance_eligible':False,'note':'OCR may be wrong; inspect retained image. No passage inferred.'}
    (hud/'readout.json').write_text(json.dumps(payload,indent=2)+'\n')
    return payload


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--container',required=True)
    args=p.parse_args();print(json.dumps(readout(args.run,args.container),indent=2))


if __name__ == '__main__':main()
