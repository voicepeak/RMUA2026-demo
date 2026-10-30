"""Frozen-source development reproduction; never labels a batch as stage acceptance."""
import argparse
from datetime import datetime,timezone
import json
from pathlib import Path
import subprocess
import uuid
from evaluation.seed_runner.manifest import source_hashes, archive_sources


def case_succeeded(case):
    """Completion requires the expected run evidence, not just a process exit."""
    summary = case.get('summary')
    return (case.get('exit_code') == 0 and case.get('run_source_unchanged') is True
            and isinstance(summary, dict) and 'fault' in summary and summary['fault'] is None
            and case.get('run_state') == 'EVALUATED'
            and case.get('evaluation_valid') is True)


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
    if type(protocol.get('max_retries', 0)) is not int or not 0 <= protocol.get('max_retries', 0) <= 2:
        raise ValueError('max retries must be an integer between 0 and 2')
    if type(protocol.get('round_trip', False)) is not bool:
        raise ValueError('round trip flag must be boolean')
    if protocol.get('round_trip') and (protocol['mode'] != 'PWM' or protocol['hover_seconds'] < 16
            or protocol['braking'] or protocol['yaw_motion'] or protocol.get('angular_recovery')
            or protocol.get('stationary_bootstrap') or protocol.get('stationary_bootstrap_cases') is not None
            or 'angular_pulse_moments' in protocol):
        raise ValueError('round trip requires an isolated bounded PWM flight')
    if type(protocol.get('angular_recovery', False)) is not bool:
        raise ValueError('angular recovery flag must be boolean')
    if type(protocol.get('stationary_bootstrap', False)) is not bool:
        raise ValueError('stationary bootstrap flag must be boolean')
    by_case = protocol.get('stationary_bootstrap_cases')
    if by_case is not None and (not isinstance(by_case, list) or len(by_case) != len(seeds)
            or any(type(value) is not bool for value in by_case) or protocol['mode'] != 'PWM'
            or protocol.get('stationary_bootstrap', False) or protocol.get('angular_recovery', False)
            or 'angular_pulse_moments' in protocol or protocol['braking'] or protocol['yaw_motion']
            or protocol['snapshot']):
        raise ValueError('per-case bootstrap requires an isolated PWM hover with one boolean per seed')
    if protocol.get('angular_recovery') and protocol['mode']!='PWM':
        raise ValueError('angular recovery requires PWM')
    profile=protocol.get('angular_recovery_profile','standard')
    if profile not in ('standard','fast') or (profile!='standard' and not protocol.get('angular_recovery')):
        raise ValueError('invalid angular recovery profile')
    if 'angular_pulse_moments' in protocol:
        moments=protocol['angular_pulse_moments']
        if (not protocol.get('angular_recovery') or protocol['mode']!='PWM'
                or protocol['braking'] or protocol['yaw_motion']
                or not isinstance(moments,list) or not 1<=len(moments)<=4
                or any(type(v) not in (int,float) or not 0<v<=.8 for v in moments)
                or moments!=sorted(moments)):
            raise ValueError('invalid staged angular pulse protocol')
    return protocol


def main():
    root=Path(__file__).resolve().parents[2]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,default=root/'configs/p1_development_reproduction_v1.json')
    parser.add_argument('--limit',type=int)
    args=parser.parse_args()
    protocol=load_protocol(args.protocol)
    bootstrap_cases=protocol.get('stationary_bootstrap_cases',
                                 [protocol.get('stationary_bootstrap', False)] * len(protocol['seeds']))
    plans=[{'seed':seed,'stationary_bootstrap':bootstrap_cases[index], 'angular_pulse_moment':moment}
           for moment in protocol.get('angular_pulse_moments',[None])
           for index,seed in enumerate(protocol['seeds'])]
    if args.limit is not None:
        if not 1<=args.limit<=len(plans):parser.error('invalid case limit')
        plans=plans[:args.limit]
    batch=root/'runs/batches'/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+uuid.uuid4().hex)
    batch.mkdir(parents=True,exist_ok=False)
    frozen=source_hashes(root)
    report={'protocol':protocol,'source_sha256_frozen':frozen,'cases':[],
            'planned_cases':plans,'source_archive':archive_sources(root,batch,frozen),
            'stage_acceptance':False,'official_result':'UNKNOWN','source_unchanged':True}
    def save():
        (batch/'batch_report.json').write_text(json.dumps(report,indent=2)+'\n')
    save()
    print('Batch:',batch,flush=True)
    for index,plan in enumerate(plans):
        seed=plan['seed']
        command=['python3','-m','evaluation.seed_runner.pwm_identification','--seed',str(seed),
                 '--mode',protocol['mode'],'--hover-seconds',str(protocol['hover_seconds']),'--evaluate-truth']
        for key,flag in (('braking','--braking'),('yaw_motion','--yaw-motion'),('snapshot','--snapshot')):
            if protocol[key]:command.append(flag)
        if protocol.get('round_trip'):command.append('--round-trip')
        if protocol.get('angular_recovery'):command.append('--angular-recovery')
        if protocol.get('angular_recovery_profile','standard')!='standard':
            command.extend(['--angular-recovery-profile',protocol['angular_recovery_profile']])
        if plan['stationary_bootstrap']:command.append('--stationary-bootstrap')
        if plan['angular_pulse_moment'] is not None:
            command.extend(['--angular-pulse','--angular-pulse-moment',str(plan['angular_pulse_moment'])])
        attempts=[]
        for attempt in range(protocol.get('max_retries', 0)+1):
            output=batch/('case_%02d%s.console.log' % (index, '' if attempt == 0 else '_retry_%d' % attempt))
            with output.open('x') as log:
                code=subprocess.call(command,cwd=root,stdout=log,stderr=subprocess.STDOUT)
            candidates=[Path(line.strip()) for line in output.read_text().splitlines()
                        if line.startswith(str(root/'runs')+'/')]
            candidates=[p for p in candidates if p.parent==root/'runs' and (p/'manifest.json').exists()]
            case={'index':index,'seed':seed,'exit_code':code,'console':output.name,'command':command}
            case['angular_pulse_moment']=plan['angular_pulse_moment']
            case['stationary_bootstrap']=plan['stationary_bootstrap']
            if len(candidates)==1:
                run=candidates[0]
                prefix=protocol['mode'].lower()
                case['run_directory']=str(run)
                summary=run/(prefix+'_summary.json')
                if summary.exists():case['summary']=json.loads(summary.read_text())
                if (run/'fault_observation.json').exists():case['fault_observation']='fault_observation.json'
                manifest=json.loads((run/'manifest.json').read_text())
                case['run_source_unchanged']=(manifest.get('source_unchanged',False)
                                             and manifest['source_sha256_before']==frozen)
                case['run_state']=manifest.get('run_state')
                metrics=run/'closed_loop_metrics.json'
                if metrics.exists():
                    case['evaluation_valid']=json.loads(metrics.read_text()).get('evaluation_valid')
            case['succeeded']=case_succeeded(case)
            case['attempt'] = attempt
            attempts.append(case)
            report['source_unchanged']=source_hashes(root)==frozen
            save()
            print('Case',index+1,'seed',seed,'exit',code,'fault',case.get('summary',{}).get('fault'),
                  'run',case.get('run_directory'),flush=True)
            if case['succeeded'] or not report['source_unchanged'] or code not in (0,2):break
        case=attempts[0]
        case['retries']=attempts[1:]
        case['succeeded_after_retries']=any(a['succeeded'] for a in attempts)
        report['cases'].append(case)
        save()
        if not report['source_unchanged'] or code not in (0,2):break
        if 'angular_pulse_moments' in protocol and not case['succeeded']:
            report['staged_pulse_stopped_on_failed_case']=True
            break
    report['completed_cases']=len(report['cases'])
    report['all_cases_completed']=len(report['cases'])==len(plans)
    report['control_fault_cases']=sum(c.get('summary',{}).get('fault') is not None for c in report['cases'])
    report['successful_cases']=sum(c['succeeded'] for c in report['cases'])
    report['batch_succeeded']=(report['source_unchanged'] and report['all_cases_completed']
                               and report['successful_cases']==len(plans))
    save()
    return 0 if report['batch_succeeded'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
