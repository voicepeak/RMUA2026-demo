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
  scripts/avoidance_interface.py
  scripts/yaw_controller.py
  scripts/speed_scheduler.py
  scripts/motion_estimator.py
  scripts/terrain_speed.py
  scripts/predictive_avoidance.py
  scripts/pose_history.py
  scripts/lidar_points.py
  scripts/point_index.py
  scripts/lattice_detour.py
  scripts/async_planner.py
  scripts/execution_guard.py
  scripts/lidar_navigation.py
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
  scripts/z_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
