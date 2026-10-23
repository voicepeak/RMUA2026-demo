"""Bounded diagnostic GPU/system CPU sampling, separate from the sensor node."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import threading
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args();stop=threading.Event()
    signal.signal(signal.SIGTERM,lambda *_:stop.set())
    started=time.monotonic()
    with (args.run_dir/'gpu_system_usage.jsonl').open('x') as output:
        while not stop.is_set() and time.monotonic()-started<150:
            sample=subprocess.run(['nvidia-smi','--query-gpu=timestamp,utilization.gpu,memory.used,power.draw',
                '--format=csv,noheader,nounits'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=3)
            cpu=[int(x) for x in Path('/proc/stat').read_text().splitlines()[0].split()[1:9]]
            output.write(json.dumps({'wall':time.monotonic(),'gpu_csv':sample.stdout.strip(),
                'gpu_exit_code':sample.returncode,'gpu_error':sample.stderr.strip(),'system_cpu_ticks':cpu})+'\n')
            output.flush();stop.wait(.5)


if __name__=='__main__':main()
