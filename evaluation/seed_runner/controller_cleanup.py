"""Terminate only a legacy exec controller carrying this run's exact UUID path."""
import json
import os
import subprocess


def stop_bound_controller(container,module,run_directory):
    script='''
import json,os,signal,sys
module,run_directory=sys.argv[1:]
stopped=[]
for name in os.listdir('/proc'):
    if not name.isdigit():continue
    try:
        with open('/proc/'+name+'/cmdline','rb') as handle:
            argv=[p.decode() for p in handle.read().split(b'\\0') if p]
        if module not in argv or '--run-dir' not in argv:continue
        index=argv.index('--run-dir')
        if index+1>=len(argv) or argv[index+1]!=run_directory:continue
        os.kill(int(name),signal.SIGTERM)
        stopped.append(int(name))
    except (OSError,UnicodeError):pass
print(json.dumps(stopped))
'''
    output=subprocess.check_output(['docker','exec','-u',str(os.getuid())+':'+str(os.getgid()),
                    container,'python3','-c',script,module,run_directory],text=True,timeout=8)
    return json.loads(output)
