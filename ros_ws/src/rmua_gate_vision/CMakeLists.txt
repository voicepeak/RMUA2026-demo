cmake_minimum_required(VERSION 3.0.2)
project(rmua_gate_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gate_vision_node.py
  scripts/gate_detect_viz.py
  scripts/gate_yolo_node.py
  scripts/gate_detector_opencv.py
  scripts/gate_stereo.py
  scripts/gate_tracker.py
  scripts/gate_map.py
  scripts/gate_dataset_capture.py
  scripts/timestamp_sync.py
  scripts/gate_geometry.py
  scripts/stereo_keypoint_matcher.py
  scripts/gate_reprojection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
