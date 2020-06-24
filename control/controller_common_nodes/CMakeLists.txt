cmake_minimum_required(VERSION 3.6)

project(controller_common_nodes)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
### Compile options
set(CMAKE_CXX_STANDARD 14)

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/controller_common_nodes/controller_node_base.cpp)
mpc_warnings(${PROJECT_NAME} RCLCPP MSGS TF2)
mpc_clang_tidy(${PROJECT_NAME} RCLCPP MSGS
  ADDITIONAL_CHECKS "-fuchsia-default-arguments" # I'm honestly just being lazy
)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(controller_common_nodes_nodes_unit_tests
    test/bad_cases.cpp
    test/error_handling.cpp
    test/gtest_main.cpp
    test/sanity_check.cpp
    test/transform.cpp)
  target_link_libraries(controller_common_nodes_nodes_unit_tests ${PROJECT_NAME})
  mpc_warnings(controller_common_nodes_nodes_unit_tests GTEST RCLCPP MSGS TF2)
  add_dependencies(controller_common_nodes_nodes_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
