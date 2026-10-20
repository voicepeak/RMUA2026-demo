"""Exploratory fixed-target gain study, not a recovery-envelope certification.

Assumes recovery has already selected the fixed target. Actual supervisor
transitions, initialization, ESKF, collision, actuator consumption and GPS errors
are absent. Production state-machine tests cover the retained 2.5rad/s boundary.
"""
import argparse,json,itertools,math
from pathlib import Path
import numpy as np
from control.se3_controller.controller import Se3Controller
from control.motor_allocator.allocator import MotorAllocator
from estimation.geometry import rotation,delta_quaternion,multiply,unit
from interfaces.contracts import StateEstimate,Metadata,Health
from evaluation.seed_runner.manifest import file_hash

def main():
    root=Path(__file__).resolve().parents[2]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if root/'runs' not in args.output.resolve().parents:parser.error('output must be inside ignored runs/')
    if args.output.exists():parser.error('refusing to overwrite an existing result')
    cfg=json.loads((root/'configs/vehicle_identification.json').read_text())
    inputs=['control/se3_controller/controller.py','control/motor_allocator/allocator.py','estimation/geometry.py','interfaces/contracts.py','configs/vehicle_identification.json']
    source_before={name:file_hash(root/name) for name in inputs}
    results=[]
    for (attitude_gain,gain),axis,lag,delay,iscale in itertools.product(((1.,2.),(4.,2.),(4.,3.),(6.,3.)),range(4),(.02,.04),(0.,.02),(.8,1.2)):
     c=Se3Controller(cfg);c.kw*=gain/2;c.kr*=attitude_gain;alloc=MotorAllocator(cfg)
     direction=np.eye(3)[axis] if axis<3 else np.array([1.,1.,0.])/2**.5
     p=np.array([0.,0.,-1.]);v=np.zeros(3);q=np.array([1.,0.,0.,0.]);w=direction*4.2
     actual=np.array([cfg['mass_kg']*9.81,0.,0.,0.]);request=actual.copy()
     inertia=np.array(cfg['inertia_kg_m2'])*iscale;dt=.0025;history=[(p.copy(),v.copy(),q.copy(),np.zeros(3)) for _ in range(round(delay/dt))]
     peak_tilt=peak_drift=0.;saturated=0;fault=None
     for i in range(1600):
      history.append((p.copy(),v.copy(),q.copy(),w.copy()))
      if i%5==0:
       sensed=history[max(0,len(history)-1-round(delay/dt))]
       state=StateEstimate(Metadata(1+i*dt,1.05+i*dt,0,'model'),tuple(sensed[0]),tuple(sensed[1]),tuple(sensed[2]),tuple(sensed[3]),Health.OK)
       wrench=c.step(state,(0.,0.,-1.),dt=.0125,allow_integral=False,rate_gain_scale=2.)
       a=alloc.allocate(wrench.thrust,wrench.moment);request=np.array(a.achieved_wrench);saturated+=a.saturated
      actual+=(1-np.exp(-dt/lag))*(request-actual)
      R=rotation(q);acceleration=np.array([0.,0.,9.81])-R[:,2]*actual[0]/cfg['mass_kg']
      alpha=(actual[1:]-np.cross(w,inertia*w))/inertia
      p+=v*dt+.5*acceleration*dt*dt;v+=acceleration*dt
      q=unit(multiply(q,delta_quaternion(w*dt+.5*alpha*dt*dt)));w+=alpha*dt
      tilt=math.acos(float(np.clip(R[2,2],-1,1)));drift=float(np.linalg.norm(p-[0,0,-1]))
      peak_tilt=max(peak_tilt,tilt);peak_drift=max(peak_drift,drift)
      if tilt>.35 or drift>.8:
       fault='TILT' if tilt>.35 else 'DRIFT';break
     result={'gain':gain,'attitude_gain':attitude_gain,'axis':axis,'lag_s':lag,'delay_s':delay,'inertia_scale':iscale,
     'impulse_norm_rad_s':4.2,'peak_tilt_rad':peak_tilt,'peak_drift_m':peak_drift,
     'final_rate_rad_s':float(np.linalg.norm(w)),'final_speed_m_s':float(np.linalg.norm(v)),
     'saturated_control_ticks':int(saturated),'fault':fault,'end_s':i*dt,
     'settled':bool(fault is None and np.linalg.norm(w)<.2 and np.linalg.norm(v)<.25 and tilt<.12)}
     results.append(result)
    report={'exploratory_model_only':True,'safety_supervisor_exercised':False,'source_sha256_before':source_before,'source_sha256_after':{name:file_hash(root/name) for name in inputs},'controller_limits_unchanged':True,'version':'v2_prefault_history','simulation_source_sha256':file_hash(Path(__file__)),
     'assumptions':'Instant 4.2rad/s angular impulse, healthy exact position/orientation sensors delayed 0/20ms, including pre-impulse zero-rate history; provisional mixer, no wind/contact/GPS bias, inertia +-20%, actuator first-order lag20/40ms. Not a certified uncertainty envelope.',
     'cases':results}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as output:output.write(json.dumps(report,indent=2)+'\n')
    for attitude_gain,gain in ((1.,2.),(4.,2.),(4.,3.),(6.,3.)):
     rows=[r for r in results if r['gain']==gain and r['attitude_gain']==attitude_gain]
     print('attitude/rate scales',attitude_gain,gain,'settled',sum(r['settled'] for r in rows),'/',len(rows),'faults',[(r['axis'],r['lag_s'],r['delay_s'],r['inertia_scale'],r['fault']) for r in rows if r['fault']])

if __name__ == "__main__":
    main()
