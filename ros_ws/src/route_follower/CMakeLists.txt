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
  scripts/splines_to_yaml.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
