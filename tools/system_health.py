#!/usr/bin/env python3
"""Read-only host power/frequency evidence for simulator timing comparisons."""
import os
import time
from pathlib import Path


def snapshot():
    result=dict(wall_time=time.time(),monotonic=time.monotonic(),load_average=list(os.getloadavg()))
    supplies={}
    for path in sorted(Path('/sys/class/power_supply').glob('*')):
        fields={}
        for name in ('type','status','capacity','online','energy_now','power_now'):
            try:fields[name]=(path/name).read_text().strip()
            except OSError:pass
        if fields:supplies[path.name]=fields
    result['power_supplies']=supplies
    frequencies=[];governors=set()
    for path in Path('/sys/devices/system/cpu/cpufreq').glob('policy*'):
        try:frequencies.append(int((path/'scaling_cur_freq').read_text()))
        except (OSError,ValueError):pass
        try:governors.add((path/'scaling_governor').read_text().strip())
        except OSError:pass
    if frequencies:
        frequencies.sort()
        result['cpu_frequency_khz']=dict(min=min(frequencies),median=frequencies[len(frequencies)//2],max=max(frequencies))
    result['cpu_governors']=sorted(governors)
    return result
