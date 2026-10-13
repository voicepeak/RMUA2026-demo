"""Choose measured inputs without importing another seed's scene geometry."""
import json


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
