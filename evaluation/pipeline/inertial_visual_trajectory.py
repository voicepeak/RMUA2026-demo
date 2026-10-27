"""Offline nominal visual-inertial diagnostic from official recorded sensors."""
import bisect
import hashlib
import json
from pathlib import Path
import sys
from estimation.history import slerp
from perception.visual_odometry.inertial_fusion import InertialVisualTrajectory


def build(estimate_path,telemetry_path):
    estimate_path,telemetry_path=Path(estimate_path),Path(telemetry_path)
    estimates=json.loads(estimate_path.read_text());groups={}
    for line in telemetry_path.read_text().splitlines():
        row=json.loads(line);raw=row['raw'];epoch=row['command']['epoch']
        groups.setdefault(epoch,{}).setdefault(raw['stamp'],(tuple(raw['acceleration_body']),tuple(raw['imu_orientation_wxyz'])))
    groups={epoch:sorted((stamp,a,q) for stamp,(a,q) in group.items()) for epoch,group in groups.items()}
    def anchor(samples,stamp):
        stamps=[s[0] for s in samples];i=bisect.bisect_right(stamps,stamp)-1
        if i<0 or i+1>=len(samples) or samples[i+1][0]-samples[i][0]>.04:
            raise ValueError('initial imu orientation not bracketed')
        a,b=samples[i],samples[i+1];f=(stamp-a[0])/(b[0]-a[0]);return slerp(a[2],b[2],f)
    segments=[];breaks=[];fusion=None
    for row in estimates['results']:
        if row['status']!='ESTIMATED':
            breaks.append({'previous_stamp':row['previous_stamp'],'current_stamp':row['current_stamp'],
                           'reason':row.get('reason','MOTION_REJECTED')});fusion=None;continue
        m=row['motion'];start,end,epoch=m['previous_stamp'],m['current_stamp'],m['epoch'];samples=groups.get(epoch,[])
        try:
            if fusion is None:
                fusion=InertialVisualTrajectory(start,epoch,anchor(samples,start))
                segments.append({'anchor_stamp':start,'epoch':epoch,'poses':[]})
            if epoch!=fusion.epoch or abs(start-fusion.stamp)>1e-6:raise ValueError('visual interval discontinuity')
            pose=fusion.append(end,m['translation_previous_body_m'],samples)
            segments[-1]['poses'].append(pose)
        except (ValueError,ArithmeticError) as error:
            breaks.append({'previous_stamp':start,'current_stamp':end,'reason':str(error)});fusion=None
    return {'source':estimates['source'],'segments':segments,'breaks':breaks,'truth_accessed':False,
            'control_eligible':False,'metric_calibration_verified':False,'initial_velocity_assumption':'zero',
            'position_gain':.5,'velocity_gain':.3,'imu_bias_correction':'none',
            'estimate_sha256':hashlib.sha256(estimate_path.read_bytes()).hexdigest(),
            'imu_telemetry_sha256':hashlib.sha256(telemetry_path.read_bytes()).hexdigest()}


def main():
    result=build(sys.argv[1],sys.argv[2]);output=Path(sys.argv[3])
    with output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({'output':str(output),'segments':len(result['segments']),'breaks':len(result['breaks'])}))


if __name__=='__main__':main()
