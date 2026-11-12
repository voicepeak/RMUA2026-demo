cmake_minimum_required(VERSION 3.0.2)
project(rl_policy_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  airsim_ros
)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}" CACHE PATH "ONNX Runtime root")
if(NOT ONNXRUNTIME_ROOT)
  set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/onnxruntime")
endif()
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
set(ONNXRUNTIME_LIB "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(rl_policy_bridge_node src/rl_policy_bridge.cpp)
target_link_libraries(rl_policy_bridge_node
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${ONNXRUNTIME_LIB}
  pthread
)
set_target_properties(rl_policy_bridge_node PROPERTIES
  INSTALL_RPATH "${ONNXRUNTIME_ROOT}/lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)
