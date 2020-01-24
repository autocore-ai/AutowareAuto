cmake_minimum_required(VERSION 3.6)

project(recordreplay_planner_actions)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(rclcpp_action REQUIRED)
find_package(rcl_action REQUIRED)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

### Build
rosidl_generate_interfaces(${PROJECT_NAME}_action
    "action/RecordTrajectory.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()
