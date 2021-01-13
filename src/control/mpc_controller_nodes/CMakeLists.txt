cmake_minimum_required(VERSION 3.6)

project(mpc_controller_nodes)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
### Compile options
set(CMAKE_CXX_STANDARD 14)

### Build
ament_auto_add_library(mpc_controller_node SHARED
  src/mpc_controller_nodes/mpc_controller_node.cpp)

ament_auto_add_executable(mpc_controller_node_exe
  src/mpc_controller_nodes/mpc_controller_main.cpp)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(mpc_controller_nodes_unit_tests
    test/gtest_main.cpp
    test/sanity_check.cpp)
  target_link_libraries(mpc_controller_nodes_unit_tests mpc_controller_node)
  add_dependencies(mpc_controller_nodes_unit_tests mpc_controller_node)
endif()

ament_auto_package(INSTALL_TO_SHARE param/)
