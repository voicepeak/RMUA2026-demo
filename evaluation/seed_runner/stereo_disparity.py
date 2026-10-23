"""Archive exact sources for historical stereo disparity or geometry in Noetic."""
import argparse
import json
import os
from pathlib import Path
import subprocess

from evaluation.seed_runner.controller_cleanup import stop_bound_controller
from evaluation.seed_runner.manifest import file_hash, prepare_run, source_hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-run', type=Path, required=True)
    parser.add_argument('--analysis', choices=('disparity', 'geometry'), default='disparity')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    source = args.source_run.resolve()
    if root / 'runs' not in source.parents:
        parser.error('source must be an owned historical run under main/runs')
    names = ('stereo_first_pair.npz', 'stereo_pairs.jsonl', 'stereo_summary.json', 'manifest.json')
    inputs = {name: file_hash(source / name) for name in names}
    original = json.loads((source / 'manifest.json').read_text())
    key = 'geometry_experiment' if args.analysis == 'geometry' else 'disparity_experiment'
    config = {key: {'source_run': str(source.relative_to(root)),
                                     'input_sha256': inputs, 'historical_only': True}}
    if args.analysis == 'geometry':
        config[key].update(settings_file='configs/simulator_settings.json',
                           settings_sha256=file_hash(root / 'configs/simulator_settings.json'))
    container = os.environ.get('RMUA_CONTAINER', 'rmua_noetic')
    digest = subprocess.check_output(['docker', 'inspect', container, '--format', '{{.Image}}'], text=True).strip()
    run, manifest = prepare_run(root, root / 'runs', original['seed'], digest, config)
    manifest.update(stage='P2_OFFLINE_STEREO_GEOMETRY' if args.analysis == 'geometry' else 'P2_OFFLINE_DISPARITY_DIAGNOSIS', acceptance_eligible=False,
                    control_published=False, run_state='PREPARED')
    (run / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(run, flush=True)
    module = 'evaluation.metrics.stereo_geometry' if args.analysis == 'geometry' else 'evaluation.metrics.stereo_disparity'
    container_run = '/workspace/main/' + str(run.relative_to(root))
    code = 1
    try:
        with (run / 'worker.log').open('x') as output:
            code = subprocess.call([str(root / 'tools/enter_env.sh'), 'env', 'OPENBLAS_NUM_THREADS=1',
                    'python3', '-m', module, '--run-dir', container_run],
                    stdout=output, stderr=subprocess.STDOUT, timeout=60)
        manifest['run_state'] = 'COMPUTED' if code == 0 else 'FAILED'
    except Exception as error:
        manifest.update(run_state='FAILED', error=str(error))
    finally:
        manifest['terminated_owned_worker_pids'] = stop_bound_controller(container, module, container_run)
        manifest['source_sha256_after'] = source_hashes(root)
        manifest['source_unchanged'] = manifest['source_sha256_before'] == manifest['source_sha256_after']
        manifest['input_unchanged'] = inputs == {name: file_hash(source / name) for name in names}
        manifest['worker_exit_code'] = code
        manifest['telemetry'] = ['stereo_disparity_report.json', 'diagnostic_disparity.npz']
        if args.analysis == 'geometry':
            manifest['telemetry'] = ['stereo_geometry_report.json', 'stereo_geometry_points.npz']
        (run / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return code if manifest['source_unchanged'] and manifest['input_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
