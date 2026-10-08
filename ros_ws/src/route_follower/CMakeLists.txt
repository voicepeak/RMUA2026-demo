cmake_minimum_required(VERSION 3.0.2)
project(route_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  airsim_ros
  std_srvs
)

catkin_package()

add_library(velocity_response_native SHARED src/velocity_response_native.cpp)
target_compile_options(velocity_response_native PRIVATE -O3 -std=c++11)
install(TARGETS velocity_response_native LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

catkin_install_python(PROGRAMS
  scripts/route_follower.py
  scripts/online_gate_cache.py
  scripts/route_recorder.py
  scripts/splines_to_yaml.py
  scripts/make_gates.py
  scripts/z_probe.py
  scripts/altitude_profile.py
  scripts/gate_chain.py
  scripts/command_arbiter.py
  scripts/command_deadline.py
  scripts/avoidance_interface.py
  scripts/yaw_controller.py
  scripts/speed_scheduler.py
  scripts/motion_estimator.py
  scripts/ceiling_guidance.py
  scripts/terrain_speed.py
  scripts/predictive_avoidance.py
  scripts/pose_history.py
  scripts/lidar_points.py
  scripts/point_index.py
  scripts/lattice_detour.py
  scripts/async_planner.py
  scripts/execution_guard.py
  scripts/lidar_navigation.py
  scripts/longitudinal_planner.py
  scripts/velocity_response.py
  scripts/response_native.py
  scripts/lidar_scene.py
  scripts/dynamic_tracker.py
  scripts/local_occupancy.py
  scripts/trajectory_types.py
  scripts/route_coordinates.py
  scripts/response_rollout.py
  scripts/trajectory_collision.py
  scripts/st_lattice.py
  scripts/trajectory_tracker.py
  scripts/trajectory_executor.py
  scripts/spacetime_navigation.py
  scripts/spacetime_runtime.py
  scripts/planning_snapshot_io.py
  scripts/planning_visualization.py
  scripts/path_sampling.py
  scripts/z_capability.py
  scripts/z_capability_probe.py
  scripts/climb_feasibility.py
  scripts/curve_preview.py
  scripts/gate_task_state.py
  scripts/mission_state.py
  scripts/reference_planner.py
  scripts/safety_supervisor.py
  scripts/lidar_clearance.py
  scripts/route_height_prior.py
  scripts/persistent_guides.py
  scripts/spatial_curve.py
  scripts/xy_tracker.py
  scripts/terminal_handoff.py
  scripts/z_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
