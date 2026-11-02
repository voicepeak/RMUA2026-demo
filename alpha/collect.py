"""Scout-only VEL/pose_gt scene capture, separate from PWM competition control."""
import argparse
from datetime import datetime
import hashlib
import json
import math
import time
import threading
import os
from pathlib import Path
import signal
import subprocess
import sys

HERE = Path(__file__).resolve().parent
SIM = Path.home()/'RUMA-by-helinjun/simulator/simulator_12.0.0.5'
TOPICS = ['/airsim_node/drone_1/front_left/Scene',
          '/airsim_node/drone_1/front_left/Scene/camera_info',
          '/airsim_node/drone_1/lidar', '/airsim_node/drone_1/debug/pose_gt']


def compare_stationary():
    """Compare one local-frame snapshot; cannot decide global gate layout."""
    import cv2
    import numpy as np
    import rosbag
    from cv_bridge import CvBridge
    from sensor_msgs import point_cloud2
    data = []
    for seed in (123, 456):
        directory = HERE/'runs'/('scout_'+str(seed))
        picked = {}
        with rosbag.Bag(str(directory/'scene.bag')) as bag:
            for topic, msg, stamp in bag.read_messages(topics=TOPICS):
                if topic not in picked:
                    picked[topic] = (msg, stamp.to_sec())
                if all(topic in picked for topic in TOPICS):
                    break
        if len(picked) != len(TOPICS):
            raise RuntimeError('missing topics for seed '+str(seed))
        image, image_t = picked[TOPICS[0]]
        cloud, cloud_t = picked[TOPICS[2]]
        pose, pose_t = picked[TOPICS[3]]
        pixels = CvBridge().imgmsg_to_cv2(image, 'bgr8')
        cv2.imwrite(str(directory/'initial_frame.png'), pixels)
        points = np.asarray(list(point_cloud2.read_points(cloud, field_names=('x','y','z'), skip_nans=True)), dtype=float).reshape(-1,3)
        voxels = {tuple(v) for v in np.floor(points/.25).astype(np.int32)}
        p, q = pose.pose.position, pose.pose.orientation
        data.append({'seed':seed, 'image': pixels, 'voxels':voxels, 'points':len(points),
                     'position':[p.x,p.y,p.z], 'orientation':[q.x,q.y,q.z,q.w],
                     'pose_offset_s':pose_t-image_t, 'lidar_offset_s':cloud_t-image_t,
                     'lidar_frame':cloud.header.frame_id})
    a,b=data
    union = len(a['voxels'] | b['voxels'])
    result = {'scope':'single initial stationary view; no full route or gate-layout conclusion',
              'resolution_m':.25, 'lidar_voxel_iou':len(a['voxels'] & b['voxels'])/union if union else None,
              'image_mean_absolute_bgr_difference':float(np.mean(np.abs(a['image'].astype(float)-b['image'].astype(float)))) if a['image'].shape == b['image'].shape else None,
              'samples':[{k:v for k,v in x.items() if k not in ('image','voxels')} for x in data],
              'caveat':'first available messages, no motion compensation or pose alignment; visual difference can include rendering/noise'}
    (HERE/'runs'/'scout_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


def compare_coverage(a, b, out):
    import json
    import sys
    from pathlib import Path
    import numpy as np
    import rosbag
    from scipy.spatial import cKDTree
    from tf.transformations import quaternion_matrix
    sys.path.insert(0, '/workspace/alpha')
    from adapters.ros_noetic.lidar_cloud import decode_xyz

    POSE='/airsim_node/drone_1/debug/pose_gt'
    LIDAR='/airsim_node/drone_1/lidar'

    def poses(path):
        data=[]
        with rosbag.Bag(str(path)) as bag:
            for _, m, _ in bag.read_messages(topics=[POSE]):
                p,q=m.pose.position,m.pose.orientation
                data.append((m.header.stamp.to_sec(),p.x,p.y,p.z,q.x,q.y,q.z,q.w))
        a=np.asarray(data,dtype=np.float64)
        if len(a)<10 or np.any(np.diff(a[:,0])<0): raise ValueError('bad pose series')
        return a

    def cloud_map(path, own, other):
        other_tree=cKDTree(other[::20,1:4])
        own_t=own[:,0]
        chunks=[]
        used=0
        excluded=0
        last=-1e30
        max_skew=0.
        with rosbag.Bag(str(path)) as bag:
            for _, msg, _ in bag.read_messages(topics=[LIDAR]):
                t=msg.header.stamp.to_sec()
                if t-last < 0.5: continue
                last=t
                i=np.searchsorted(own_t,t)
                candidates=[j for j in (i-1,i) if 0<=j<len(own)]
                if not candidates: continue
                i=min(candidates,key=lambda j:abs(own_t[j]-t))
                skew=abs(own_t[i]-t)
                if skew>.1 or other_tree.query(own[i,1:4])[0]>3.0:
                    excluded+=1
                    continue
                cloud=decode_xyz(msg,0).points
                R=quaternion_matrix(own[i,4:8])[:3,:3]
                # SensorLocalFrame, zero rotation and offset from simulator settings.json.
                world=(cloud+np.array([0.,0.,-.05]))@R.T+own[i,1:4]
                vox=np.floor(world/.25).astype(np.int32)
                chunks.append(np.unique(vox,axis=0))
                used+=1
                max_skew=max(max_skew,skew)
        if not chunks: raise ValueError('no overlapping clouds')
        vox=np.unique(np.concatenate(chunks),axis=0)
        return vox,{'lidar_frames_used':used,'lidar_frames_excluded':excluded,'max_pose_skew_s':max_skew,
                    'pose_start':own[0,1:4].tolist(),'pose_end':own[-1,1:4].tolist(),
                    'pose_count':len(own),'voxel_count':len(vox)}

    def run(a,b,out):
        a,b=Path(a),Path(b)
        pa,pb=poses(a/'scene.bag'),poses(b/'scene.bag')
        va,sa=cloud_map(a/'scene.bag',pa,pb)
        vb,sb=cloud_map(b/'scene.bag',pb,pa)
        dtype=np.dtype([('x','<i4'),('y','<i4'),('z','<i4')])
        def flat(v): return np.ascontiguousarray(v).view(dtype).ravel()
        aa,bb=flat(va),flat(vb)
        common=np.intersect1d(aa,bb,assume_unique=True)
        only_a=np.setdiff1d(aa,bb,assume_unique=True)
        only_b=np.setdiff1d(bb,aa,assume_unique=True)
        def bins(vals):
            xyz=vals.view('<i4').reshape(-1,3)
            blocks=np.floor_divide(xyz,40)
            keys,counts=np.unique(blocks,axis=0,return_counts=True)
            order=np.argsort(counts)[::-1][:12]
            return [{'origin_ned_m':(keys[i]*10).tolist(),'count':int(counts[i])} for i in order]
        result={'scope':'only scans whose pose has counterpart within 3m; not full course',
                'resolution_m':.25,'frame_assumption':'NED pose_gt + quaternion; lidar SensorLocalFrame, offset (0,0,-0.05)m',
                'pose_match_max_age_s':.1,'scan_period_s':.5,'voxel_iou':len(common)/(len(aa)+len(bb)-len(common)),
                'voxels_common':len(common),'voxels_only_123':len(only_a),'voxels_only_456':len(only_b),
                'largest_10m_bins_only_123':bins(only_a),'largest_10m_bins_only_456':bins(only_b),
                'sample_123':sa,'sample_456':sb,
                'caveat':'occupied endpoints only; unseen does not equal free; scene dynamics, occlusion, sensor sampling, registration, noise and timestamp/frame uncertainty can produce differences; not gate identity or full layout proof'}
        Path(out).write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2))

    run(a, b, out)


def scout_worker(out):
    import rosgraph
    import rospy
    import yaml
    from airsim_ros.msg import VelCmd
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Imu
    sys.path.insert(0, '/workspace/alpha')
    from control.control_authority.lease import ControlLease, check_graph
    route_file = Path('/workspace/beta/rmua_ws/src/route_follower/config/gates_seed123_recorded.yaml')
    gates = yaml.safe_load(route_file.read_text())['gates']
    points = [(float(g['x']), float(g['y']), float(g['z'])) for g in gates[:17]]
    route_path = route_file.with_name('route_1_3.yaml')
    guides_path = route_file.with_name('guides_seed123_recorded.yaml')
    route = yaml.safe_load(route_path.read_text())['routes']['route_1_3']
    guides = yaml.safe_load(guides_path.read_text())['altitude_guides']
    stations = [0.]
    for a,b in zip(route,route[1:]):
        stations.append(stations[-1]+math.hypot(b[0]-a[0],b[1]-a[1]))
    from bisect import bisect_left
    def xy_at(station):
        i = max(0, min(len(route)-2, bisect_left(stations,station)-1))
        fraction = (station-stations[i])/(stations[i+1]-stations[i])
        return tuple(route[i][j]+fraction*(route[i+1][j]-route[i][j]) for j in (0,1))
    for guide in guides:
        x,y = xy_at(float(guide['s']))
        points.append((float(x),float(y),float(guide['z'])))
    rospy.init_node('alpha_scout_vel', anonymous=True, disable_signals=True)
    latest = {'pose': None, 'pose_wall': 0., 'imu': None, 'imu_wall': 0.}
    def pose_cb(msg):
        latest['pose'], latest['pose_wall'] = msg, time.monotonic()
    def imu_cb(msg):
        latest['imu'], latest['imu_wall'] = msg, time.monotonic()
    rospy.Subscriber('/airsim_node/drone_1/debug/pose_gt', PoseStamped, pose_cb, queue_size=1)
    rospy.Subscriber('/airsim_node/drone_1/imu/imu', Imu, imu_cb, queue_size=1)
    master = rosgraph.Master(rospy.get_name())
    def graph(require_self):
        pubs, _, _ = master.getSystemState()
        check_graph(dict(pubs), rospy.get_name(), 'VEL', require_self)
    result = {'mode':'scout-only', 'control':'VEL', 'pose_gt':'development guidance',
              'route_source':str(route_file), 'route_sha256':hashlib.sha256(route_file.read_bytes()).hexdigest(),
              'coverage_scope':'beta route_1_3 only; not complete official six-stage course', 'route_count':len(points), 'route_xy_file':str(route_path), 'route_xy_sha256':hashlib.sha256(route_path.read_bytes()).hexdigest(), 'height_file':str(guides_path), 'height_sha256':hashlib.sha256(guides_path.read_bytes()).hexdigest(), 'waypoints_reached':0, 'fault':None, 'coverage_complete':False,
              'limits':{'xy_m_s':2.0,'z_m_s':0.6,'rate_rad_s':1.0,'tilt_rad':0.35,
                        'stale_s':0.5,'max_leg_m':45.0}, 'debug':True}
    out = Path(out)
    publisher = None
    try:
        with ControlLease(rospy.get_param('/run_id'), 'VEL'):
            graph(False)
            deadline = time.monotonic()+8
            while time.monotonic() < deadline and (latest['pose'] is None or latest['imu'] is None):
                time.sleep(.05)
            if latest['pose'] is None or latest['imu'] is None:
                raise RuntimeError('POSE_OR_IMU_UNAVAILABLE')
            publisher = rospy.Publisher('/airsim_node/drone_1/vel_body_cmd', VelCmd, queue_size=1)
            deadline = time.monotonic()+3
            while time.monotonic()<deadline and publisher.get_num_connections()!=1:
                time.sleep(.05)
            if publisher.get_num_connections()!=1:
                raise RuntimeError('VEL_CONNECTION_COUNT')
            graph(True)
            pos = latest['pose'].pose.position
            targets = [(pos.x,pos.y,pos.z-1.5)] + points
            start = time.monotonic()
            last_progress = start
            best = float('inf')
            index = 0
            last_graph = 0.
            with (out/'scout_trajectory.jsonl').open('w') as trace:
                while index < len(targets):
                    now = time.monotonic()
                    if now-start > 1800:
                        raise RuntimeError('MISSION_TIMEOUT')
                    if now-latest['pose_wall'] > .5 or now-latest['imu_wall'] > .5:
                        raise RuntimeError('STALE_POSE_OR_IMU')
                    if now-last_graph > 1:
                        graph(True)
                        last_graph = now
                    p = latest['pose'].pose.position
                    q = latest['pose'].pose.orientation
                    imu = latest['imu']
                    omega = math.sqrt(imu.angular_velocity.x**2+imu.angular_velocity.y**2+imu.angular_velocity.z**2)
                    qi = imu.orientation
                    cos_tilt = 1-2*(qi.x*qi.x+qi.y*qi.y)
                    if not math.isfinite(omega) or omega > 1.:
                        raise RuntimeError('ANGULAR_RATE_LIMIT')
                    if not math.isfinite(cos_tilt) or cos_tilt < math.cos(.35):
                        raise RuntimeError('TILT_LIMIT')
                    target = targets[index]
                    if not all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w,*target)):
                        raise RuntimeError('NONFINITE_POSE_OR_TARGET')
                    if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .05:
                        raise RuntimeError('INVALID_POSE_QUATERNION')
                    dx,dy,dz = target[0]-p.x,target[1]-p.y,target[2]-p.z
                    distance = math.sqrt(dx*dx+dy*dy+dz*dz)
                    if distance > 45:
                        raise RuntimeError('UNVERIFIED_ROUTE_GAP_%d_%.1fm'%(index,distance))
                    tolerance = .35 if index == 0 else 1.5
                    if distance < tolerance:
                        if index:
                            result['waypoints_reached'] = index
                        index += 1
                        best, last_progress = float('inf'), now
                        continue
                    if distance < best-.3:
                        best, last_progress = distance, now
                    if now-last_progress > 25:
                        raise RuntimeError('NO_PROGRESS_WAYPOINT_%d'%index)
                    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                    vx,vy = .35*dx,.35*dy
                    horizontal = math.hypot(vx,vy)
                    if horizontal>2:
                        vx,vy = vx*2/horizontal,vy*2/horizontal
                    cmd = VelCmd()
                    cmd.header.stamp = rospy.Time.now()
                    cmd.header.frame_id = 'drone_1'
                    cmd.vx = math.cos(yaw)*vx+math.sin(yaw)*vy
                    cmd.vy = -math.sin(yaw)*vx+math.cos(yaw)*vy
                    cmd.vz = max(-.6,min(.6,-.35*dz))
                    cmd.yawRate = 0.
                    cmd.va, cmd.stop = 4,0
                    publisher.publish(cmd)
                    trace.write(json.dumps({'wall':now,'pose_stamp':latest['pose'].header.stamp.to_sec(),
                        'position':[p.x,p.y,p.z], 'target':target,'waypoint':index,
                        'velocity_body':[cmd.vx,cmd.vy,cmd.vz], 'angular_rate':omega})+'\n')
                    trace.flush()
                    time.sleep(.1)
            result['coverage_complete'] = True
    except Exception as exc:
        result['fault'] = str(exc)
    finally:
        if publisher is not None:
            cmd = VelCmd()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = 'drone_1'
            cmd.va,cmd.stop = 4,0
            for _ in range(3):
                publisher.publish(cmd)
                time.sleep(.1)
            publisher.unregister()
        (out/'scout_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    return 0 if result['coverage_complete'] else 1


def record_worker(out):
    import rosbag
    import rospy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
    rospy.init_node('alpha_scout_recorder',anonymous=True)
    lock = threading.Lock()
    rates = {TOPICS[0]:2., TOPICS[1]:1., TOPICS[2]:4., TOPICS[3]:20., '/airsim_node/drone_1/imu/imu':20.}
    types = {TOPICS[0]:Image, TOPICS[1]:CameraInfo, TOPICS[2]:PointCloud2, TOPICS[3]:PoseStamped, '/airsim_node/drone_1/imu/imu':Imu}
    last = {topic:0. for topic in rates}
    counts = {topic:0 for topic in rates}
    bag = rosbag.Bag(str(Path(out)/'scene.bag'),'w')
    def callback(message, topic):
        now = time.monotonic()
        with lock:
            if now-last[topic] < 1./rates[topic]:
                return
            last[topic] = now
            bag.write(topic,message,rospy.Time.now())
            counts[topic] += 1
    subscribers = [rospy.Subscriber(topic,types[topic],callback,topic,queue_size=1,buff_size=2**24) for topic in rates]
    try:
        rospy.spin()
    finally:
        for subscriber in subscribers: subscriber.unregister()
        with lock: bag.close()
        (Path(out)/'record_stats.json').write_text(json.dumps({'counts':counts,'max_rates_hz':rates},indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('seed', type=int)
    args = parser.parse_args()
    if args.seed < 0:
        parser.error('seed must be nonnegative')
    base = HERE/'runs'/('scout_'+str(args.seed))
    out = base/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    out.mkdir(parents=True, exist_ok=False)
    container_out = '/workspace/alpha/alpha/runs/'+str(out.relative_to(HERE/'runs'))
    config = SIM/'Build/LinuxNoEditor/RMUA/Content/Configs/GameConfig.json'
    config_bytes = config.read_bytes()
    settings = json.loads(config_bytes)
    metadata = {'seed': args.seed, 'timestamp': datetime.now().astimezone().isoformat(),
                'commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=HERE, text=True).strip(),
                'simulator_realpath': str(SIM.resolve()), 'mode': 'offscreen', 'control': 'VEL',
                'purpose': 'scout-only; pose_gt-guided development flight; not competition',
                'coverage_scope':'beta route_1_3 only, not complete official six-stage course',
                'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'pose_gt': 'development guidance only; not competition control', 'topics': TOPICS,
                'game_config_sha256': hashlib.sha256(config_bytes).hexdigest(),
                'game_config': settings, 'config_modified': False,
                'coverage_complete': False, 'result': 'running'}
    path = out/'meta.json'
    def save():
        path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
    save()
    def interrupted(sig, frame):
        raise RuntimeError('interrupted: '+str(sig))
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupted)
    container = os.environ.get('RMUA_CONTAINER', 'rmua_next_gpu')
    try:
        subprocess.run([str(HERE/'sim_up.sh'), str(args.seed)], check=True,
                       env={**os.environ, 'ALPHA_SIM_LOG': container_out+'/simulator.log'})
        cmd = ('source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash; '
               'exec python3 /workspace/alpha/alpha/collect.py --record-worker "$1"')
        with (out/'capture.log').open('w') as log:
            recorder = subprocess.Popen(['docker', 'exec', '-u', '1000:1000', container, 'bash', '-c', cmd,
                            'alpha-scout', container_out], stdout=log, stderr=subprocess.STDOUT)
            time.sleep(2)
            if recorder.poll() is not None:
                raise RuntimeError('rosbag failed to start')
            cmd = ('source /opt/ros/noetic/setup.bash; source /workspace/rmua_ws/devel/setup.bash; '
                   'source /workspace/main/.runtime_ws/devel/setup.bash; '
                   'exec python3 /workspace/alpha/alpha/collect.py --worker "$1"')
            with (out/'scout.log').open('w') as worker_log:
                subprocess.run(['docker','exec','-u','1000:1000',container,'bash','-c',cmd,
                                'alpha-worker',container_out],stdout=worker_log,stderr=subprocess.STDOUT,
                               timeout=1820,check=False)
            subprocess.run(['docker','exec',container,'pkill','-INT','-f','[c]ollect.py --record-worker'],check=False)
            try:
                recorder.wait(timeout=20)
            except subprocess.TimeoutExpired:
                recorder.terminate()
                raise RuntimeError('rosbag failed to finalize')
        cmd = 'source /opt/ros/noetic/setup.bash; rosbag info --yaml "$1/scene.bag"'
        with (out/'bag_info.yaml').open('w') as info:
            subprocess.run(['docker', 'exec', '-u', '1000:1000', container, 'bash', '-c', cmd,
                            'alpha-info', container_out], stdout=info, check=True, timeout=20)
        result = json.loads((out/'scout_result.json').read_text())
        metadata['coverage_complete'] = result['coverage_complete']
        metadata['result'] = 'complete' if result['coverage_complete'] else 'incomplete'
        metadata['scout_fault'] = result['fault']
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        metadata['result'] = 'failed'
        metadata['error'] = str(exc)
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, signal.SIG_IGN)
        if (HERE/'.sim-owner').exists():
            cleanup = subprocess.run([str(HERE/'sim_down.sh')], check=False).returncode
            metadata['cleanup_exit'] = cleanup
            if cleanup:
                metadata['result'] = 'failed'
        save()
    print(json.dumps({'seed': args.seed, 'path': str(out), 'result': metadata['result'],
                      'coverage_complete': metadata['coverage_complete']}, ensure_ascii=False))
    return 0 if metadata['result'] == 'complete' else 1


if __name__ == '__main__':
    if len(sys.argv) == 5 and sys.argv[1] == '--compare-coverage':
        compare_coverage(*sys.argv[2:])
        sys.exit(0)
    if len(sys.argv) == 3 and sys.argv[1] == '--record-worker':
        record_worker(sys.argv[2])
        sys.exit(0)
    if len(sys.argv) == 3 and sys.argv[1] == '--worker':
        sys.exit(scout_worker(sys.argv[2]))
    if sys.argv[1:] == ['--compare-internal']:
        compare_stationary()
    else:
        sys.exit(main())
