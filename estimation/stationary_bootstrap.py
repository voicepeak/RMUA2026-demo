"""Conservative pre-control resting-window check for local takeoff experiments.

Stationary IMU alone cannot distinguish constant velocity. This helper also
requires GPS position consistency and must only run before control is acquired
in a fresh ground-start scene. It is not an in-flight reset or outage recovery.
"""
import numpy as np
from estimation.geometry import unit


def resting_window(imu,gps,initial_attitude,stamp):
    imu=[r for r in imu if stamp-1.1<=r['stamp']<=stamp]
    gps=[r for r in gps if stamp-1.1<=r[0]<=stamp]
    if len(imu)<65 or len(gps)<7:return None
    times=np.array([r['stamp'] for r in imu])
    if (not np.all(np.isfinite(times)) or times[-1]-times[0]<1.
            or np.any(np.diff(times)<=0) or np.max(np.diff(times))>.04):return None
    gt=np.array([r[0] for r in gps])
    if (not np.all(np.isfinite(gt)) or np.any(np.diff(gt)<=0)
            or gt[-1]-gt[0]<.6 or stamp-gt[-1]>.15):return None
    positions=np.array([r[1] for r in gps])
    if not np.all(np.isfinite(positions)) or np.any(np.all(positions==0,axis=1)):return None
    if np.max(np.linalg.norm(positions-positions.mean(0),axis=1))>.2:return None
    acceleration=np.array([r['acceleration_body'] for r in imu])
    gyro=np.array([r['omega_body'] for r in imu])
    if (acceleration.shape != (len(imu),3) or gyro.shape != (len(imu),3)
            or not np.all(np.isfinite(acceleration)) or not np.all(np.isfinite(gyro))
            or np.max(np.linalg.norm(gyro,axis=1))>.025
            or np.max(np.std(acceleration,axis=0))>.08
            or abs(np.linalg.norm(acceleration.mean(0))-9.81)>.15):return None
    q0=unit(initial_attitude)
    for r in imu:
        q=r.get('imu_orientation_wxyz')
        if q is None:return None
        q=np.array(q,dtype=float)
        if not np.all(np.isfinite(q)) or abs(np.linalg.norm(q)-1)>.01:return None
        if 2*np.arccos(np.clip(abs(unit(q)@q0),0.,1.))>.02:return None
    return {'position':positions.mean(0).tolist(),'gyro_bias':gyro.mean(0).tolist(),
            'imu_samples':len(imu),'gps_samples':len(gps),'span_s':float(times[-1]-times[0]),
            'imu_window_start_stamp':float(times[0]),'imu_window_end_stamp':float(times[-1]),
            'gps_window_start_stamp':float(gt[0]),'gps_window_end_stamp':float(gt[-1]),
            'gps_position_std_m':positions.std(0).tolist(),
            'acceleration_std_m_s2':acceleration.std(0).tolist(),
            'assumption':'fresh ground-start scene, before any control publisher',
            'imu_orientation_use':'resting alignment check only; initial_pose supplies attitude'}
