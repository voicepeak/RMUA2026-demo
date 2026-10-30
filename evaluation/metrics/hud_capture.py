"""Record only the local simulator HUD window; no controller or score inference."""
import argparse
import json
import re
import struct
import subprocess
import time
from pathlib import Path
from PIL import Image


def decode_xwd(data):
    if len(data) < 100:
        raise ValueError('short XWD header')
    h = struct.unpack('>25I', data[:100])
    header, version, fmt, depth, width, height, _, byte_order, _, _, _, bits, stride, visual, red, green, blue, _, _, colors = h[:20]
    if (version != 7 or fmt != 2 or depth != 24 or byte_order != 0 or bits != 32
            or visual != 5 or (red, green, blue) != (0xff0000, 0xff00, 0xff)
            or not 255 <= width <= 4096 or not 320 <= height <= 4096 or stride < width*4):
        raise ValueError('unsupported XWD format')
    offset = header + colors*12
    if header < 100 or len(data) < offset + stride*height:
        raise ValueError('truncated XWD pixels')
    return Image.frombytes('RGB', (width, height), data[offset:offset+stride*height], 'raw', 'BGRX', stride, 1)


def capture(run):
    out = run/'hud'; out.mkdir(exist_ok=False)
    rows = []; fault = None
    try:
        while not (run/'hud_stop').exists():
            listing = subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True,check=True,timeout=3).stdout
            windows = re.findall(r'^\s*(0x[0-9a-f]+) "RMUA\s*"', listing, re.M)
            if len(windows) != 1:
                raise RuntimeError('expected exactly one RMUA simulator window')
            raw = subprocess.run(['xwd','-silent','-id',windows[0]],capture_output=True,check=True,timeout=3).stdout
            image = decode_xwd(raw)
            crop = image.crop((image.width-255, 0, image.width, 320))
            name = 'hud_%04d.png' % len(rows); crop.save(out/name)
            rows.append({'file':name, 'wall_time':time.time(), 'monotonic':time.monotonic(), 'window_id':windows[0]})
            (run/'hud_ready').touch(exist_ok=True)
            time.sleep(1.)
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        fault = str(error)
    (out/'index.json').write_text(json.dumps({'frames':rows,'fault':fault,'source':'simulator_window_crop',
        'official_result':'UNKNOWN','control_eligible':False},indent=2)+'\n')
    return 0 if rows and fault is None else 2


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('--run-dir',type=Path,required=True)
    return capture(parser.parse_args().run_dir)


if __name__ == '__main__': raise SystemExit(main())
