"""Compare saved official-image VO with recorded official-IMU gyro only."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys

from perception.visual_odometry.gyro_consistency import gyro_rotation, rotation_residual


def compare(estimate_path, telemetry_path):
    estimate_path, telemetry_path = Path(estimate_path), Path(telemetry_path)
    estimates=json.loads(estimate_path.read_text()); groups={}
    for line in telemetry_path.read_text().splitlines():
        row=json.loads(line);raw=row['raw'];epoch=row['command']['epoch']
        group=groups.setdefault(epoch,{})
        # Duplicates do not introduce a new sample or refresh a timestamp.
        group.setdefault(raw['stamp'],tuple(raw['omega_body']))
    groups={epoch:sorted(samples.items()) for epoch,samples in groups.items()}
    rows=[]
    for row in estimates['results']:
        if row['status']!='ESTIMATED':continue
        motion=row['motion'];result={'previous_stamp':motion['previous_stamp'],
                                    'current_stamp':motion['current_stamp'],'epoch':motion['epoch']}
        try:
            gyro=gyro_rotation(groups.get(motion['epoch'],[]),motion['previous_stamp'],motion['current_stamp'])
            result.update(status='COMPARED',rotation_residual_rad=rotation_residual(motion['rotation_previous_from_current_body'],gyro))
        except ValueError as error:result.update(status='REJECTED',reason=str(error))
        rows.append(result)
    return {'source':estimates['source'],'estimate_sha256':hashlib.sha256(estimate_path.read_bytes()).hexdigest(),
            'imu_telemetry_sha256':hashlib.sha256(telemetry_path.read_bytes()).hexdigest(),
            'input_role':'official image VO and recorded official IMU angular velocity only',
            'gyro_bias_correction':'none','truth_accessed':False,'control_eligible':False,
            'status_counts':dict(Counter(r['status'] for r in rows)), 'results':rows}


def main():
    result=compare(sys.argv[1],sys.argv[2]);output=Path(sys.argv[3])
    with output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'output':str(output),'status_counts':result['status_counts']}))


if __name__=='__main__':main()
