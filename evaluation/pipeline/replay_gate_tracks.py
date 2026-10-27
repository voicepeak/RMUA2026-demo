"""Replay gate candidates with recorded estimator states, without truth input."""
import argparse
import bisect
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from estimation.geometry import multiply, unit
from estimation.history import slerp
from interfaces.contracts import Metadata
from perception.gate_geometry.stereo_gate import GateCandidate
from world_model.gate_landmarks.tracker import GateTracker


@dataclass(frozen=True)
class DiagnosticPose:
    stamp: float
    epoch: int
    position_ned: tuple
    orientation_wxyz: tuple
    bracket_stamps: tuple
    position_correction_residual_m: float
    rotation_correction_residual_rad: float
    frame_id: str = 'world_ned'


def interpolate_pose(rows, stamp, epoch=0):
    """Interpolate recorded controller state, not independently calibrated truth."""
    timestamps=[row['raw']['stamp'] for row in rows]
    i=bisect.bisect_left(timestamps,stamp)
    if i==0 or i==len(rows):return None,'NO_STATE_BRACKET'
    a,b=rows[i-1],rows[i];t0,t1=timestamps[i-1],timestamps[i]
    if (t1<=t0 or t1-t0>.05 or a['state']['health']!='OK' or b['state']['health']!='OK'
            or a['command']['epoch']!=epoch or b['command']['epoch']!=epoch):
        return None,'BAD_STATE_BRACKET'
    first,second=a['state'],b['state'];dt=t1-t0;fraction=(stamp-t0)/dt
    p0,p1=np.asarray(first['position_ned']),np.asarray(second['position_ned'])
    predicted=.5*(np.asarray(first['velocity_ned'])+second['velocity_ned'])*dt
    position_residual=float(np.linalg.norm(p1-p0-predicted))
    q0,q1=unit(first['orientation_wxyz']),unit(second['orientation_wxyz'])
    delta=unit(multiply(q0*(1.,-1.,-1.,-1.),q1))
    if delta[0]<0:delta=-delta
    length=float(np.linalg.norm(delta[1:]))
    angle=delta[1:]*(2*math.atan2(length,float(delta[0]))/length) if length>1e-12 else np.zeros(3)
    expected=.5*(np.asarray(first['angular_velocity_body'])+second['angular_velocity_body'])*dt
    angular_residual=float(np.linalg.norm(angle-expected))
    if position_residual>.1 or angular_residual>.05:return None,'STATE_DISCONTINUITY'
    return DiagnosticPose(stamp,epoch,tuple((1-fraction)*p0+fraction*p1),tuple(slerp(q0,q1,fraction)),
                          (t0,t1),position_residual,angular_residual),None


def replay(gate_file):
    gate_file=Path(gate_file); data=json.loads(gate_file.read_text()); source=Path(data['source'])
    log=source/('pwm_telemetry.jsonl' if (source/'pwm_telemetry.jsonl').exists() else 'vel_telemetry.jsonl')
    states=[json.loads(line) for line in log.read_text().splitlines()]
    if any(a['raw']['stamp']>=b['raw']['stamp'] for a,b in zip(states,states[1:])):
        raise ValueError('recorded estimator clock not strictly increasing')
    tracker=GateTracker();output=[]
    for frame in data['results']:
        if frame['status']!='PROCESSED':
            output.append({'left':frame['left'],'status':frame['status']});continue
        if not frame['candidates']:
            # Without candidate metadata the saved 51 data does not retain image stamp here.
            output.append({'left':frame['left'],'status':'NO_CANDIDATE'});continue
        values=frame['candidates'];stamp=values[0]['metadata']['stamp'];epoch=values[0]['metadata']['epoch']
        pos,reason=interpolate_pose(states,stamp,epoch)
        if pos is None:
            output.append({'left':frame['left'],'status':reason});continue
        candidates=[]
        for value in values:
            m=Metadata(**{**value['metadata'],'covariance_or_bounds':tuple(value['metadata']['covariance_or_bounds'])})
            candidates.append(GateCandidate(m,tuple(tuple(p) for p in value['image_edges_px']),
                tuple(value['center_body_frd_m']),value['nominal_width_m'],value['nominal_height_m'],
                tuple(value['edge_disparities_px']),tuple(value['edge_support'])))
        # This is the earliest time both estimator samples exist, not the actual
        # completion time of the expensive disparity/gate computation.
        now=pos.bracket_stamps[1]
        try:
            associations=tracker.ingest(tuple(candidates),pos,now,epoch)
            tracks=tracker.snapshot(now,epoch)
            output.append({'left':frame['left'],'status':'TRACKED','stamp':stamp,'now':now,
                           'associations':associations,'tracks':[asdict(t) for t in tracks],
                           'pose_bracket_stamps':pos.bracket_stamps})
        except ValueError as error:
            output.append({'left':frame['left'],'status':'REJECTED','reason':str(error)})
    return {'gate_input':str(gate_file),'gate_input_sha256':hashlib.sha256(gate_file.read_bytes()).hexdigest(),
            'state_input':str(log),'state_input_sha256':hashlib.sha256(log.read_bytes()).hexdigest(),
            'uses_truth':False,'now_is_earliest_bracket_not_actual_processing_end':True,
            'control_eligible':False,'results':output}


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('gate_file');parser.add_argument('output')
    args=parser.parse_args();data=replay(args.gate_file)
    with Path(args.output).open('x') as out:json.dump(data,out,indent=2,allow_nan=False);out.write('\n')
    print(json.dumps({'output':args.output,'statuses':[r['status'] for r in data['results']]}))


if __name__ == '__main__':main()
