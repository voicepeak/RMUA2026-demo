"""Frozen three-way development diagnosis; no competition acceptance claims."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

from evaluation.seed_runner.manifest import archive_sources, file_hash, source_hashes
from evaluation.seed_runner.simulator_preflight import verify_local_simulator


MODES = ('none', 'observe', 'serialize')


def case_order(protocol):
    if (protocol.get('schema_version') != 1
            or type(protocol.get('repeats')) is not int or not 1 <= protocol['repeats'] <= 5
            or type(protocol.get('hover_seconds')) not in (int, float)
            or not 10 <= protocol['hover_seconds'] <= 20
            or type(protocol.get('stationary_bootstrap')) is not bool):
        raise ValueError('invalid bounded bridge protocol')
    seeds = protocol.get('seeds')
    if (not isinstance(seeds, list) or not 1 <= len(seeds) <= 4
            or any(type(seed) is not int or seed < 0 for seed in seeds)):
        raise ValueError('invalid development seeds')
    modes = tuple(protocol.get('modes', MODES))
    if modes not in (MODES, ('none', 'lifetime'), ('none', 'native-input')):
        raise ValueError('unsupported fixed diagnostic comparison')
    cases = []
    for repeat in range(protocol['repeats']):
        for index, seed in enumerate(seeds):
            offset = (repeat + index) % len(modes)
            for mode in modes[offset:] + modes[:offset]:
                cases.append({'seed': seed, 'repeat': repeat, 'mode': mode})
    return cases


def collect_case(run):
    manifest = json.loads((run / 'manifest.json').read_text())
    summary_path = run / 'pwm_summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else None
    metrics_path = run / 'closed_loop_metrics.json'
    metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else None
    trace_path = run / 'command_trace_audit.json'
    trace = json.loads(trace_path.read_text()) if trace_path.exists() else None
    lifetime_path = run / 'bridge_lifetime_audit.json'
    lifetime = json.loads(lifetime_path.read_text()) if lifetime_path.exists() else None
    native_path = run / 'native_input_audit.json'
    native = json.loads(native_path.read_text()) if native_path.exists() else None
    return {'run_directory': str(run), 'run_state': manifest.get('run_state'),
            'controller_fault': summary.get('fault') if summary else 'MISSING_SUMMARY',
            'controller_completed': bool(summary and summary.get('fault') is None
                                         and summary.get('commands', 0) > 0
                                         and manifest.get('observer_exit_code') == 0),
            'command_trace_audit_exit_code': manifest.get('command_trace_audit_exit_code'),
            'evaluation_valid': metrics.get('evaluation_valid', False) if metrics else False,
            'summary': summary, 'metrics': metrics, 'command_trace': trace,
            'lifetime_audit': lifetime,
            'native_input_audit': ({k: v for k, v in native.items()
                                    if k not in ('matched', 'unmatched', 'unreadable',
                                                 'freed_during_original_callback')}
                                   if native else None),
            'bridge_probe_summary': manifest.get('bridge_probe_summary'),
            'source_archive': manifest.get('source_archive'),
            'instrumentation': manifest.get('simulator_runtime_instrumentation')}


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol', type=Path, default=root / 'configs/bridge_comparison_v1.json')
    args = parser.parse_args()
    protocol = json.loads(args.protocol.read_text())
    planned = case_order(protocol)
    frozen = source_hashes(root)
    simulator = verify_local_simulator(root)
    container_digest = subprocess.check_output(['docker', 'inspect', os.environ.get('RMUA_CONTAINER', 'rmua_noetic'),
                                                '--format', '{{.Image}}'], text=True).strip()
    variant = ('native-input' if any(c['mode'] == 'native-input' for c in planned)
               else 'lifetime' if any(c['mode'] == 'lifetime' for c in planned) else 'concurrency')
    probe = Path(subprocess.check_output(['bash', str(root / 'tools/build_bridge_probe.sh'), variant],
                                        cwd=root, text=True).strip())
    probe_hash = file_hash(probe)
    batch = root / 'runs/batches' / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')
                                    + uuid.uuid4().hex)
    batch.mkdir(parents=True, exist_ok=False)
    report = {'protocol': protocol, 'planned_cases': planned, 'source_sha256_frozen': frozen,
              'source_archive': archive_sources(root, batch, frozen), 'simulator': simulator,
              'simulator_container_image_digest': container_digest,
              'probe_binary_sha256': probe_hash, 'cases': [], 'status': 'RUNNING',
              'probe_build_variant': variant,
              'stage_acceptance': False, 'physical_impulse_root_cause_proven': False,
              'official_result': 'UNKNOWN'}

    def save():
        (batch / 'batch_report.json').write_text(json.dumps(report, indent=2) + '\n')

    save()
    print('Batch:', batch, flush=True)
    try:
        for index, plan in enumerate(planned):
            if source_hashes(root) != frozen or file_hash(probe) != probe_hash:
                raise RuntimeError('source or diagnostic binary changed before case')
            command = [sys.executable, '-m', 'evaluation.seed_runner.pwm_identification',
                       '--seed', str(plan['seed']), '--hover-seconds', str(protocol['hover_seconds']),
                       '--evaluate-truth']
            if protocol['stationary_bootstrap']:
                command.append('--stationary-bootstrap')
            if plan['mode'] != 'none':
                command.extend(['--bridge-probe', plan['mode']])
            console = batch / ('case_%02d.console.log' % index)
            with console.open('x') as output:
                code = subprocess.call(command, cwd=root, stdout=output, stderr=subprocess.STDOUT)
            candidates = [Path(line.strip()) for line in console.read_text().splitlines()
                          if line.startswith(str(root / 'runs') + '/')]
            candidates = [path for path in candidates if path.parent == root / 'runs'
                          and (path / 'manifest.json').is_file()]
            case = dict(plan, index=index, exit_code=code, console=console.name, command=command)
            report['cases'].append(case)
            if len(candidates) != 1:
                raise RuntimeError('case has no unique run manifest')
            run = candidates[0]
            case.update(collect_case(run))
            manifest = json.loads((run / 'manifest.json').read_text())
            case['provenance_verified'] = (
                manifest.get('source_sha256_before') == frozen and manifest.get('source_unchanged')
                and manifest['simulator_preflight'] == simulator
                and manifest.get('simulator_container_image_digest') == container_digest
                and verify_local_simulator(root) == simulator and source_hashes(root) == frozen
                and file_hash(run / case['source_archive']['path']) == case['source_archive']['sha256'])
            if case['instrumentation']:
                case['provenance_verified'] = (case['provenance_verified']
                    and case['instrumentation']['binary_sha256'] == probe_hash
                    and case['instrumentation']['mode'] == plan['mode']
                    and manifest.get('original_simulator_files_unchanged') is True)
            elif plan['mode'] != 'none':
                case['provenance_verified'] = False
            save()
            print('Case', index + 1, plan['seed'], plan['mode'], 'exit', code,
                  'fault', case['controller_fault'], 'trace', case['command_trace_audit_exit_code'], flush=True)
            if not case['provenance_verified'] or code not in (0, 2):
                raise RuntimeError('case provenance or infrastructure failed; retain evidence and stop batch')
        report['status'] = 'COMPLETE'
    except Exception as error:
        report['status'] = 'ABORTED'
        report['error'] = str(error)
        raise
    finally:
        report['completed_cases'] = len(report['cases'])
        report['all_cases_completed'] = (report['status'] == 'COMPLETE'
                                         and len(report['cases']) == len(planned))
        report['source_unchanged'] = source_hashes(root) == frozen
        save()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
