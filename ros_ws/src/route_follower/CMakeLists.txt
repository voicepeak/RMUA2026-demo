cmake_minimum_required(VERSION 3.0.2)
project(route_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  airsim_ros
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/route_follower.py
  scripts/route_recorder.py
  scripts/splines_to_yaml.py
  scripts/make_gates.py
  scripts/z_probe.py
  scripts/altitude_profile.py
  scripts/gate_chain.py
  scripts/command_arbiter.py
  scripts/avoidance_interface.py
  scripts/yaw_controller.py
  scripts/speed_scheduler.py
  scripts/z_capability.py
  scripts/z_capability_probe.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
