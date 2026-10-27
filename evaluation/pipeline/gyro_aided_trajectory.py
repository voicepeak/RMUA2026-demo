"""Build a local gyro-aided stereo trajectory without truth or GPS inputs."""
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

from perception.visual_odometry.gyro_consistency import gyro_rotation
from perception.visual_odometry.trajectory import VisualTrajectory
from dataclasses import asdict


def build(estimate_path, telemetry_path):
    estimate_path,telemetry_path=Path(estimate_path),Path(telemetry_path)
    estimates=json.loads(estimate_path.read_text());groups={}
    for line in telemetry_path.read_text().splitlines():
        row=json.loads(line);raw=row['raw'];epoch=row['command']['epoch']
        groups.setdefault(epoch,{}).setdefault(raw['stamp'],tuple(raw['omega_body']))
    groups={e:sorted(group.items()) for e,group in groups.items()}
    segments=[];breaks=[];chain=None
    for row in estimates['results']:
        if row['status']!='ESTIMATED':
            breaks.append({'previous_stamp':row['previous_stamp'],'current_stamp':row['current_stamp'],
                           'reason':row.get('reason','MOTION_REJECTED')})
            chain=None;continue
        motion=row['motion'];previous,current,epoch=motion['previous_stamp'],motion['current_stamp'],motion['epoch']
        try:
            r=gyro_rotation(groups.get(epoch,[]),previous,current)
            if chain is None:
                chain=VisualTrajectory(previous,epoch)
                segments.append({'anchor_stamp':previous,'epoch':epoch,'poses':[]})
            step=SimpleNamespace(previous_stamp=previous,current_stamp=current,epoch=epoch,
                translation_previous_body_m=motion['translation_previous_body_m'],
                rotation_previous_from_current_body=r)
            pose=chain.append(step);segments[-1]['poses'].append(asdict(pose))
        except ValueError as error:
            breaks.append({'previous_stamp':previous,'current_stamp':current,'reason':str(error)})
            chain=None
    return {'source':estimates['source'],'segments':segments,'breaks':breaks,'truth_accessed':False,
            'control_eligible':False,'metric_calibration_verified':False,'gyro_bias_correction':'none',
            'estimate_sha256':hashlib.sha256(estimate_path.read_bytes()).hexdigest(),
            'imu_telemetry_sha256':hashlib.sha256(telemetry_path.read_bytes()).hexdigest()}


def main():
    result=build(sys.argv[1],sys.argv[2]);output=Path(sys.argv[3])
    with output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'output':str(output),'segments':len(result['segments']),'breaks':len(result['breaks'])}))


if __name__=='__main__':main()
