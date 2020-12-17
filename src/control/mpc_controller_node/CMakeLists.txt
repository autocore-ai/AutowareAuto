cmake_minimum_required(VERSION 3.6)

project(mpc_controller_node)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
### Compile options
set(CMAKE_CXX_STANDARD 14)

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mpc_controller_node/mpc_controller_node.cpp)

ament_auto_add_executable(${PROJECT_NAME}_exe
  src/mpc_controller_node/mpc_controller_main.cpp)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(mpc_controller_node_unit_tests
    test/gtest_main.cpp
    test/sanity_check.cpp)
  target_link_libraries(mpc_controller_node_unit_tests ${PROJECT_NAME})
  add_dependencies(mpc_controller_node_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE param/)
