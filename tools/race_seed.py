"""Choose measured inputs without importing another seed's scene geometry."""
import json
import math
import yaml


def live_gate_vision_arguments(vehicle=False):
    args=['_cold_start_gate_map:=true']
    # The outbound gates have a wider opening than the vehicle-leg frames.
    args+=['_max_width:='+('8.0' if vehicle else '18.0'),
           '_max_height:='+('8.0' if vehicle else '12.0')]
    return args


def cold_start_vision_arguments(seed,vehicle=False):
    """Compatibility for older callers; seed never selects a control policy."""
    return live_gate_vision_arguments(vehicle)


def initial_course_inputs(config,out,seed):
    folder=out/'course_inputs'
    folder.mkdir(parents=True,exist_ok=True)
    gates=folder/'gates.yaml';guides=folder/'guides.yaml'
    common=dict(seed=seed,provenance='live_run',run_id=str(out.resolve()),
                bootstrap='Empty; observations must come from this run')
    gates.write_text(json.dumps(dict(common,gates=[]),indent=2)+'\n')
    guides.write_text(json.dumps(dict(common,altitude_guides=[]),indent=2)+'\n')
    return gates,guides


def current_run_measured_map(path,out,seed):
    if path is None:return []
    data=yaml.safe_load(path.read_text())
    if (not isinstance(data,dict) or data.get('seed')!=seed or
            data.get('provenance')!='live_run' or data.get('run_id')!=str(out.resolve())):
        raise ValueError('Measured-map input must belong to this live run and seed')
    return data.get('gates',[])


def initial_gate_map_ready(gates,now):
    """Require fresh, stable local evidence before issuing the first command."""
    if not isinstance(gates,list):return False
    for gate in gates:
        try:
            values=[float(gate[k]) for k in ('x','y','z','sigma_x','sigma_y','sigma_z','last_seen')]
            x,y,z,sx,sy,sz,stamp=values
            if not all(math.isfinite(v) for v in values):continue
            if (gate.get('valid',True) and gate.get('hard_anchor',False) and
                    gate.get('support',0)>=8 and float(gate.get('confidence') or 0)>=.65 and
                    max(sx,sy)<=1.2 and 0.<=sz<=.6 and min(sx,sy)>=0. and
                    0.<=now-stamp<=1. and math.hypot(x,y)<60.):return True
        except (KeyError,TypeError,ValueError):continue
    return False
