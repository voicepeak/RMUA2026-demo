#!/usr/bin/env bash
set -euo pipefail
python3 - <<'PY'
from pathlib import Path
import json
root = Path('/home/tianbot/RUMA-hf-public-20261019')
service = json.loads((root / 'upload_service_status.local.json').read_text())
state = json.loads((root / 'upload_state.local.json').read_text())
planned = json.loads((root / 'public/shard_plan.json').read_text())
source_bytes = sum(s['source_bytes'] for s in planned)
done_bytes = sum(s['source_bytes'] for s in planned if s['name'] in state['verified'])
command = Path('/proc') / str(service['pid']) / 'cmdline'
alive = command.exists() and b'evaluation.dataset_capture.upload_service' in command.read_bytes()
cleanup_path = root / 'local_cleanup_status.local.json'
cleanup = json.loads(cleanup_path.read_text()) if cleanup_path.exists() else None
if cleanup:
    cleanup_command = Path('/proc') / str(cleanup['pid']) / 'cmdline'
    cleanup['process_alive'] = (cleanup_command.exists()
        and b'evaluation.dataset_capture.cleanup_archive' in cleanup_command.read_bytes())
print(json.dumps({
    'repo': 'https://huggingface.co/datasets/' + service['repo_id'],
    'state': service['state'], 'process_alive': alive,
    'verified_shards': len(state['verified']), 'expected_shards': len(planned),
    'verified_source_gib': round(done_bytes / 1024 ** 3, 2),
    'total_source_gib': round(source_bytes / 1024 ** 3, 2),
    'percent': round(100 * done_bytes / source_bytes, 2),
    'error': service.get('error'),
    'local_cleanup': cleanup,
}, ensure_ascii=False, indent=2))
PY
