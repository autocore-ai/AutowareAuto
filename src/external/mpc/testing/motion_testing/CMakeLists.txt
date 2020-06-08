cmake_minimum_required(VERSION 3.6)

project(motion_testing)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
### Compile options
set(CMAKE_CXX_STANDARD 14)

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED src/motion_testing/motion_testing.cpp)
mpc_warnings(${PROJECT_NAME} MSGS)
mpc_clang_tidy(${PROJECT_NAME})

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(motion_testing_unit_tests
    test/gtest_main.cpp
    test/constant_trajectory.cpp
    test/trajectory_checks.cpp)
  target_link_libraries(motion_testing_unit_tests ${PROJECT_NAME})
  mpc_warnings(motion_testing_unit_tests GTEST MSGS)
  add_dependencies(motion_testing_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
