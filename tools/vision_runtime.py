#!/usr/bin/env python3
"""Probe the vision interpreter before starting the official departure clock."""
import json
import subprocess


def probe_cuda(container='rmua_noetic'):
    code='''import json
try:
 import torch
 available=torch.cuda.is_available()
 if available:
  value=torch.ones(1,device='cuda').sum().item()
  torch.cuda.synchronize()
 print(json.dumps(dict(available=available,torch=torch.__version__,cuda=torch.version.cuda,
                      device=torch.cuda.get_device_name(0) if available else None)))
except Exception as error:
 print(json.dumps(dict(available=False,error=repr(error))))
'''
    try:
        result=subprocess.run(['docker','exec',container,'/opt/conda/envs/xal/bin/python','-c',code],
                              capture_output=True,text=True,timeout=20.)
        report=json.loads(result.stdout.strip().splitlines()[-1])
        report['available']=bool(report.get('available') and result.returncode==0)
        if result.stderr:report['stderr']=result.stderr.strip()[-1500:]
        return report
    except (subprocess.TimeoutExpired,ValueError,IndexError,OSError) as error:
        return dict(available=False,error=repr(error))
