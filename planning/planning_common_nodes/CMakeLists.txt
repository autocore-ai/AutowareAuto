cmake_minimum_required(VERSION 3.6)

project(planning_common_nodes)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
### Compile options
set(CMAKE_CXX_STANDARD 14)

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/planning_common_nodes/planner_base_node.cpp)
mpc_warnings(${PROJECT_NAME} RCLCPP MSGS)
mpc_clang_tidy(${PROJECT_NAME} RCLCPP MSGS
  ADDITIONAL_CHECKS "-fuchsia-default-arguments" # I'm honestly just being lazy
)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(planning_common_nodes_unit_tests
    test/error.cpp
    test/gtest_main.cpp
    test/source.cpp)
  ament_target_dependencies(planning_common_nodes_unit_tests "motion_testing_nodes")
  target_link_libraries(planning_common_nodes_unit_tests ${PROJECT_NAME})
  mpc_warnings(planning_common_nodes_unit_tests GTEST RCLCPP MSGS TF2)
  add_dependencies(planning_common_nodes_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
