"""LOCAL EVALUATION ONLY. Separate process, no publishers or controller imports."""
import argparse
import json
from pathlib import Path
import threading
import time


def main():
    import rospy
    import roslib.message
    from geometry_msgs.msg import PoseStamped,Vector3Stamped
    from sensor_msgs.msg import Imu
    from rosgraph_msgs.msg import Log
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--record-pwm-commands',action='store_true',help='requires observer-compatible PWM publisher')
    args = parser.parse_args()
    run = args.run_dir
    rospy.init_node('rmua_hidden_truth_evaluator', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    rows = []
    actuation=[]
    commands=[]
    wind=[]
    imu=[]
    logs=[]
    def receive(msg):
        p, q = msg.pose.position, msg.pose.orientation
        row = {'stamp': msg.header.stamp.to_sec(), 'position': (p.x, p.y, p.z),
               'orientation': (q.w, q.x, q.y, q.z),'arrival_wall_monotonic':time.monotonic()}
        with lock:
            if len(rows) < 30000:
                rows.append(row)
    listener = rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt', PoseStamped,
                                receive, queue_size=1)
    def plain(value):
        if hasattr(value,'__slots__'):
            return {slot:plain(getattr(value,slot)) for slot in value.__slots__}
        if isinstance(value,(list,tuple)):
            return [plain(v) for v in value]
        return value
    def receive_actuation(msg,destination=None):
        header=msg._connection_header
        cls=roslib.message.get_message_class(header['type'])
        row={'arrival_wall_monotonic':time.monotonic(),'type':header['type'],
             'md5sum':header['md5sum']}
        if cls is not None and cls._md5sum==header['md5sum']:
            decoded=cls();decoded.deserialize(msg._buff)
            row['data']=plain(decoded)
        else:
            row['raw_hex']=msg._buff.hex()
        with lock:
            target=actuation if destination is None else destination
            if len(target)<30000:target.append(row)
    actuator_listener=rospy.Subscriber('/airsim_node/drone_1/debug/rotor_pwm',rospy.AnyMsg,
                                       receive_actuation,queue_size=1)
    command_listener=(rospy.Subscriber('/airsim_node/drone_1/rotor_pwm_cmd',rospy.AnyMsg,
                        lambda msg:receive_actuation(msg,commands),queue_size=1000)
                        if args.record_pwm_commands else None)
    def receive_wind(msg):
        v=msg.vector
        with lock:
            if len(wind)<10000:wind.append({'stamp':msg.header.stamp.to_sec(),
                'frame_id':msg.header.frame_id,'wind_world_m_s':(v.x,v.y,v.z)})
    def receive_imu(msg):
        arrival=time.monotonic()
        a,w,q=msg.linear_acceleration,msg.angular_velocity,msg.orientation
        with lock:
            if len(imu)<30000:imu.append({'stamp':msg.header.stamp.to_sec(),
                'acceleration_body':(a.x,a.y,a.z),'omega_body':(w.x,w.y,w.z),
                'imu_orientation_wxyz':(q.w,q.x,q.y,q.z),'arrival_wall_monotonic':arrival})
    def receive_log(msg):
        with lock:
            if len(logs)<10000:logs.append({'stamp':msg.header.stamp.to_sec(),
                'node':msg.name,'level':msg.level,'message':msg.msg})
    wind_listener=rospy.Subscriber('/airsim_node/drone_1/debug/wind',Vector3Stamped,receive_wind,queue_size=1)
    imu_listener=rospy.Subscriber('/airsim_node/drone_1/imu/imu',Imu,receive_imu,queue_size=1)
    log_listener=rospy.Subscriber('/rosout_agg',Log,receive_log,queue_size=10)
    deadline = time.monotonic()+40
    try:
        while time.monotonic() < deadline and not rospy.is_shutdown() and not (run/'truth_stop').exists():
            with lock:
                if rows and not (run/'truth_ready').exists():
                    (run/'truth_ready').touch()
            time.sleep(.02)
    finally:
        listener.unregister()
        actuator_listener.unregister()
        if command_listener:command_listener.unregister()
        wind_listener.unregister()
        imu_listener.unregister()
        log_listener.unregister()
        with lock:
            (run/'evaluation_truth.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows))
            (run/'evaluation_actuation.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in actuation))
            if command_listener:
                (run/'evaluation_pwm_commands.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in commands))
            for name,data in (('wind',wind),('imu',imu),('rosout',logs)):
                (run/('evaluation_'+name+'.jsonl')).write_text(''.join(json.dumps(row)+'\n' for row in data))
    return 0 if rows else 2


if __name__ == '__main__':
    raise SystemExit(main())
