cmake_minimum_required(VERSION 3.0.2)
project(start_to_goal)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  airsim_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/start_to_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
