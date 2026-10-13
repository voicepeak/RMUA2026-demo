"""Choose measured inputs without importing another seed's scene geometry."""
import json


def cold_start_vision_arguments(seed,vehicle=False):
    if seed==123:return []
    args=['_cold_start_gate_map:=true']
    # The outbound gates have a wider opening than the vehicle-leg frames.
    if not vehicle:args+=['_max_width:=18.0','_max_height:=12.0']
    return args


def initial_course_inputs(config,out,seed):
    if seed==123:
        return config/'gates_seed123_recorded.yaml',config/'guides_seed123_recorded.yaml'
    folder=out/'course_inputs'
    folder.mkdir(parents=True,exist_ok=True)
    gates=folder/'gates.yaml';guides=folder/'guides.yaml'
    common=dict(seed=seed,provenance='Empty bootstrap; observations must come from this run')
    gates.write_text(json.dumps(dict(common,gates=[]),indent=2)+'\n')
    guides.write_text(json.dumps(dict(common,altitude_guides=[]),indent=2)+'\n')
    return gates,guides
