cmake_minimum_required(VERSION 3.5)
project(test_trajectory_following)

# dependencies
find_package(ament_cmake_auto REQUIRED)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install python scripts
install(PROGRAMS
  scripts/simple_trajectory.py
  scripts/joy_ctrl_record_replay_traj.py
  scripts/vehicle_kinematics_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE config launch param)
