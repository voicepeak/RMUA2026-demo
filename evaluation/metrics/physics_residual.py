"""Offline consistency diagnostics from IMU and evaluation-only rotor feedback.

Motor coefficients are provisional. Residuals locate discrepancies; they do not
identify collision, wind, numerical faults, or prove an actuator latency bound.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from control.motor_allocator.allocator import MotorAllocator


def feedback_samples(rows,model):
    samples={}
    for row in rows:
        data=row.get('data',{})
        header=data.get('header',{}).get('stamp',{})
        if 'secs' not in header:continue
        stamp=header['secs']+header.get('nsecs',0)*1e-9
        try:
            pwm=np.array([data['rotorPWM'+str(i)] for i in range(4)],dtype=float)
        except KeyError:continue
        if np.all(np.isfinite(pwm)) and np.all((pwm>=0)&(pwm<=1)):
            samples[stamp]=model.B @ (model.max_thrust*pwm)
    stamps=np.array(sorted(samples))
    return stamps,np.array([samples[t] for t in stamps])


def interval_wrench(stamps,wrenches,start,end,max_gap=.04):
    """Integrate zero-order-held feedback; reject gaps or absent coverage."""
    index=int(np.searchsorted(stamps,start,side='right'))-1
    if index<0 or not start<end:return None
    if end>stamps[-1]+max_gap:return None
    total=np.zeros(4)
    cursor=start
    while cursor<end:
        if cursor-stamps[index]>max_gap:return None
        right=min(end,stamps[index+1] if index+1<len(stamps) else end)
        if right-stamps[index]>max_gap:return None
        total+=(right-cursor)*wrenches[index]
        cursor=right
        index+=1
    return total/(end-start)


def evaluate(raw_rows,actuation,config):
    model=MotorAllocator(config)
    inertia=np.array(config['inertia_kg_m2'])
    stamps,wrenches=feedback_samples(actuation,model)
    raw=sorted({r['stamp']:r for r in raw_rows}.values(),key=lambda r:r['stamp'])
    residuals=[]
    for a,b in zip(raw,raw[1:]):
        dt=b['stamp']-a['stamp']
        if not .002<=dt<=.05 or not len(stamps):continue
        mean=interval_wrench(stamps,wrenches,a['stamp'],b['stamp'])
        if mean is None:continue
        wa,wb=np.array(a['omega_body']),np.array(b['omega_body'])
        omega=(wa+wb)*.5
        angular=inertia*(wb-wa)/dt+np.cross(omega,inertia*omega)-mean[1:]
        # Trapezoidal endpoint approximation, not a collocated force sensor.
        specific=(np.array(a['acceleration_body'])+b['acceleration_body'])*.5
        linear=specific-np.array((0.,0.,-mean[0]/config['mass_kg']))
        residuals.append({'stamp':b['stamp'],'interval_s':dt,
            'angular_residual_Nm':angular.tolist(),'angular_norm_Nm':float(np.linalg.norm(angular)),
            'specific_force_residual_m_s2':linear.tolist(),'mean_feedback_wrench':mean.tolist(),
            'omega_body_rad_s':wb.tolist()})
    result={'matched_intervals':len(residuals),'candidate_intervals':max(0,len(raw)-1),
        'evaluation_only':True,'cause_identified':False,'motor_model_status':config['parameter_status'],
        'actuation_latency_verified':False,'official_result':'UNKNOWN'}
    if residuals:
        magnitudes=[r['angular_norm_Nm'] for r in residuals]
        result['angular_residual_p50_p95_p99_max_Nm']=np.percentile(magnitudes,[50,95,99,100]).tolist()
        result['largest_angular_discrepancies']=sorted(residuals,key=lambda r:r['angular_norm_Nm'],reverse=True)[:5]
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    args=parser.parse_args()
    def rows(name):return [json.loads(line) for line in (args.run/name).read_text().splitlines()]
    imu_file=args.run/'evaluation_imu.jsonl'
    raw=rows('evaluation_imu.jsonl') if imu_file.exists() else [r['raw'] for r in rows('pwm_telemetry.jsonl')]
    telemetry=rows('pwm_telemetry.jsonl')
    start=telemetry[0]['command']['stamp'] if telemetry else float('inf')
    end=telemetry[-1]['command']['stamp'] if telemetry else float('-inf')
    fault=args.run/'fault_observation.json'
    if fault.exists():
        value=json.loads(fault.read_text())
        if value.get('fresh_read_in_fault_iteration') and value.get('raw'):
            raw.append(value['raw']);end=max(end,value['raw']['stamp'])
    raw=[r for r in raw if start<=r['stamp']<=end]
    root=Path(__file__).resolve().parents[2]
    model_path=args.run/'vehicle_model.json'
    if not model_path.exists():
        # Historical runs did not emit their effective controller model. Refuse
        # silent analysis with subsequently changed workspace coefficients.
        from evaluation.seed_runner.manifest import file_hash
        manifest=json.loads((args.run/'manifest.json').read_text())
        model_path=root/'configs/vehicle_identification.json'
        expected=manifest['source_sha256_before']['configs/vehicle_identification.json']
        if file_hash(model_path)!=expected:raise ValueError('historical motor model changed')
    result=evaluate(raw,rows('evaluation_actuation.jsonl'),json.loads(model_path.read_text()))
    result['model_provenance']='controller_emitted_snapshot' if model_path.parent==args.run else 'historical_source_hash_match'
    result['imu_source']='independent_full_rate_recorder' if imu_file.exists() else 'controller_sampled_imu'
    (args.run/'physics_residual.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    return 0 if result['matched_intervals'] else 2


if __name__=='__main__':raise SystemExit(main())
