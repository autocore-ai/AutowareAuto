cmake_minimum_required(VERSION 3.6)

project(recordreplay_planner_node)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_action REQUIRED)

### Compile options
set(CMAKE_CXX_STANDARD 14)

### Build
rosidl_generate_interfaces(${PROJECT_NAME}
    "action/RecordTrajectory.action"
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/recordreplay_planner_node/recordreplay_planner_node.cpp)

ament_auto_add_executable(${PROJECT_NAME}_exe
  src/recordreplay_planner_node/recordreplay_planner_main.cpp)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  # TODO(s.me) do we do those for nodes?
endif()

ament_auto_package(INSTALL_TO_SHARE param/)
