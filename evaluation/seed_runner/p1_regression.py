"""Frozen-source development reproduction; never labels a batch as stage acceptance."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import uuid
from evaluation.seed_runner.manifest import source_hashes


def load_protocol(path):
    protocol=json.loads(path.read_text())
    seeds=protocol.get('seeds')
    if (protocol.get('schema_version')!=1 or not isinstance(seeds,list) or not 1<=len(seeds)<=50
            or any(type(seed) is not int or seed<0 for seed in seeds)
            or protocol.get('mode') not in ('PWM','VEL')
            or not 10<=protocol.get('hover_seconds',0)<=20):
        raise ValueError('invalid bounded reproduction protocol')
    for name in ('braking','yaw_motion','snapshot'):
        if type(protocol.get(name)) is not bool:
            raise ValueError('protocol flags must be booleans')
    return protocol


def main():
    root=Path(__file__).resolve().parents[2]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,default=root/'configs/p1_development_reproduction_v1.json')
    parser.add_argument('--limit',type=int)
    args=parser.parse_args()
    protocol=load_protocol(args.protocol)
    seeds=protocol['seeds']
    if args.limit is not None:
        if not 1<=args.limit<=len(seeds):parser.error('invalid case limit')
        seeds=seeds[:args.limit]
    batch=root/'runs/batches'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+uuid.uuid4().hex)
    batch.mkdir(parents=True,exist_ok=False)
    frozen=source_hashes(root)
    report={'protocol':protocol,'source_sha256_frozen':frozen,'cases':[],
            'stage_acceptance':False,'official_result':'UNKNOWN','source_unchanged':True}
    def save():
        (batch/'batch_report.json').write_text(json.dumps(report,indent=2)+'\n')
    save()
    print('Batch:',batch,flush=True)
    for index,seed in enumerate(seeds):
        command=['python3','-m','evaluation.seed_runner.pwm_identification','--seed',str(seed),
                 '--mode',protocol['mode'],'--hover-seconds',str(protocol['hover_seconds']),'--evaluate-truth']
        for key,flag in (('braking','--braking'),('yaw_motion','--yaw-motion'),('snapshot','--snapshot')):
            if protocol[key]:command.append(flag)
        output=batch/('case_%02d.console.log'%index)
        with output.open('x') as log:
            code=subprocess.call(command,cwd=root,stdout=log,stderr=subprocess.STDOUT)
        candidates=[Path(line.strip()) for line in output.read_text().splitlines()
                    if line.startswith(str(root/'runs')+'/')]
        candidates=[p for p in candidates if p.parent==root/'runs' and (p/'manifest.json').exists()]
        case={'index':index,'seed':seed,'exit_code':code,'console':output.name,'command':command}
        if len(candidates)==1:
            run=candidates[0]
            prefix=protocol['mode'].lower()
            case['run_directory']=str(run)
            summary=run/(prefix+'_summary.json')
            if summary.exists():case['summary']=json.loads(summary.read_text())
            if (run/'fault_observation.json').exists():case['fault_observation']='fault_observation.json'
            case['run_source_unchanged']=json.loads((run/'manifest.json').read_text()).get('source_unchanged',False)
        report['cases'].append(case)
        report['source_unchanged']=source_hashes(root)==frozen
        save()
        print('Case',index+1,'seed',seed,'exit',code,'fault',case.get('summary',{}).get('fault'),
              'run',case.get('run_directory'),flush=True)
        if not report['source_unchanged'] or code not in (0,2):break
    report['completed_cases']=len(report['cases'])
    report['all_cases_completed']=len(report['cases'])==len(seeds)
    report['control_fault_cases']=sum(c.get('summary',{}).get('fault') is not None for c in report['cases'])
    save()
    return 0 if report['source_unchanged'] and report['all_cases_completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
