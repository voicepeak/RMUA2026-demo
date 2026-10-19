"""Read-only local diagnostics from official cameras and LiDAR, no debug inputs."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    import cv2
    import rospy
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image,PointCloud2
    from sensor_msgs import point_cloud2
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args()
    rospy.init_node('rmua_scene_snapshot',anonymous=True,disable_signals=True)
    bridge=CvBridge()
    result={'input_policy':'official_sensors_only','cameras':{}}
    for camera in ('front_left','front_right','back_left','back_right'):
        msg=rospy.wait_for_message('/airsim_node/drone_1/'+camera+'/Scene',Image,timeout=5)
        frame=bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        path=args.run_dir/(camera+'.png')
        if path.exists():raise RuntimeError('snapshot already exists')
        if not cv2.imwrite(str(path),frame):raise RuntimeError('image write failed')
        result['cameras'][camera]={'stamp':msg.header.stamp.to_sec(),'frame_id':msg.header.frame_id}
    msg=rospy.wait_for_message('/airsim_node/drone_1/lidar',PointCloud2,timeout=5)
    points=np.asarray(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)),dtype=float).reshape(-1,3)
    np.savez_compressed(str(args.run_dir/'scene_lidar.npz'),points=points)
    result['lidar']={'stamp':msg.header.stamp.to_sec(),'frame_id':msg.header.frame_id,'points':len(points)}
    (args.run_dir/'scene_snapshot.json').write_text(json.dumps(result,indent=2)+'\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
