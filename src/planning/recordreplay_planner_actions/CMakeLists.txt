cmake_minimum_required(VERSION 3.6)

project(recordreplay_planner_actions)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RecordTrajectory.action"
  "action/ReplayTrajectory.action"
)

ament_auto_package()
