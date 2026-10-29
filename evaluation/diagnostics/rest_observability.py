"""Synthetic indistinguishability of rest and constant motion; never publishes control."""
import hashlib
import json
from pathlib import Path

import numpy as np
from estimation.stationary_bootstrap import resting_window


def experiment():
    start, duration, speed = 100., 1.1, .18
    imu = [{'stamp': start+i*.01, 'acceleration_body': [0., 0., -9.81],
            'omega_body': [0., 0., 0.], 'imu_orientation_wxyz': [1., 0., 0., 0.]}
           for i in range(111)]
    # The exact same observations are consistent with a fixed position plus a
    # bounded time-varying GPS error, and with constant velocity and no error.
    gps = [(start+i*.1, (1.+speed*(i*.1-duration/2), 2., .31), (1., 0., 0., 0.))
           for i in range(12)]
    result = resting_window(imu, gps, (1., 0., 0., 0.), start+duration)
    if result is None:
        raise AssertionError('constructed ambiguous window no longer accepted')
    static_truth = np.array([1., 2., .31])
    max_static_error = max(float(np.linalg.norm(np.asarray(p)-static_truth)) for _, p, _ in gps)
    return {'assumed_static_truth_m': static_truth.tolist(), 'assumed_moving_speed_m_s': speed,
            'duration_s': duration, 'max_static_gps_error_m': max_static_error,
            'same_imu_and_gps_for_both_worlds': True, 'current_window_accepted': True,
            'reset_velocity_m_s': [0., 0., 0.], 'moving_world_reset_error_m_s': speed,
            'bootstrap_result': result,
            'limitations': 'Synthetic inputs only. A ground-start guarantee could exclude the moving world; no simulator truth, actual noise distribution or in-flight behavior is established.'}


def main():
    result = experiment()
    files = [Path(__file__), Path(__file__).resolve().parents[2]/'estimation/stationary_bootstrap.py']
    result['source_sha256'] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
