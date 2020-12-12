cmake_minimum_required(VERSION 3.6)

project(recordreplay_planner_actions)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Build
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RecordTrajectory.action"
  "action/ReplayTrajectory.action"
)

ament_auto_package()
