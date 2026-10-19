"""Truth-only response measurement, never exported as a certified braking bound."""
import numpy as np


def measure_braking(telemetry,truth,brake_elapsed=6.,stop_speed=.08,hold_seconds=.35):
    result={'valid':False,'certified_deceleration_bound':False,
            'stop_speed_threshold_m_s':stop_speed,'hold_seconds':hold_seconds}
    commands=[r for r in telemetry if r.get('sim_elapsed',-1) >= brake_elapsed]
    ordered=sorted({r['stamp']:r for r in truth if r['stamp']>0}.values(),key=lambda r:r['stamp'])
    if not commands or len(ordered)<20:
        return {**result,'reason':'MISSING_DATA'}
    stamp=commands[0]['command']['stamp']
    times=np.array([r['stamp'] for r in ordered])
    positions=np.array([r['position'] for r in ordered])
    roi=(times >= stamp-.1) & (times <= stamp+4.5)
    times,positions=times[roi],positions[roi]
    if (len(times)<20 or times[0]>stamp or times[-1]<stamp+hold_seconds
            or np.max(np.diff(times))>.05):
        return {**result,'reason':'TRUTH_COVERAGE_OR_GAP'}
    vx=np.gradient(positions[:,0],times)
    v0=float(np.interp(stamp,times,vx))
    x0=float(np.interp(stamp,times,positions[:,0]))
    if v0 < .15:
        return {**result,'reason':'INSUFFICIENT_INITIAL_SPEED','initial_speed_m_s':v0}
    stop=None
    for i,t in enumerate(times):
        if t<stamp or t+hold_seconds>times[-1]:
            continue
        window=(times>=t)&(times<=t+hold_seconds)
        if np.all(np.abs(vx[window])<=stop_speed):
            stop=i
            break
    if stop is None:
        return {**result,'reason':'DID_NOT_SETTLE_WITHIN_OBSERVATION','initial_speed_m_s':v0}
    window=(times>=stamp)&(times<=times[stop]+hold_seconds)
    distance=float(max(0.,np.max(positions[window,0])-x0))
    if distance<=1e-5:
        return {**result,'reason':'INVALID_FORWARD_DISTANCE'}
    effective=max(0.,v0*v0-vx[stop]*vx[stop])/(2*distance)
    result.update(valid=True,reference_sensor_stamp=stamp,initial_speed_m_s=v0,
                  stop_time_s=float(times[stop]-stamp),forward_excursion_m=distance,
                  effective_average_deceleration_m_s2=float(effective),
                  peak_speed_after_reference_m_s=float(np.max(vx[window])),
                  lateral_excursion_m=float(np.ptp(positions[window,1])),
                  note='Includes controller/sensing lag; effective average is not an instantaneous lower bound.')
    return result
