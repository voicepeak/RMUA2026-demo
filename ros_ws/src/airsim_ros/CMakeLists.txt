cmake_minimum_required(VERSION 3.0.2)
project(airsim_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  VelCmd.msg
)

add_service_files(
  FILES
  Takeoff.srv
  Land.srv
  Reset.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
