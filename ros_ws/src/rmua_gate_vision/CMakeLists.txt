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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
