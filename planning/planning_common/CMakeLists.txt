cmake_minimum_required(VERSION 3.6)

project(planning_common)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
### Compile options
set(CMAKE_CXX_STANDARD 14)

# Build library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/planning_common/planner_base.cpp
  src/planning_common/planning_context.cpp
)
mpc_warnings(${PROJECT_NAME} MSGS)
mpc_clang_tidy(${PROJECT_NAME} ADDITIONAL_CHECKS -fuchsia-default-arguments) # I'm lazy

### Test
if(BUILD_TESTING)
  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(planning_common_unit_tests
    test/gtest_main.cpp
    test/context.cpp
    test/planner_base.cpp)
  target_link_libraries(planning_common_unit_tests ${PROJECT_NAME})
  mpc_warnings(planning_common_unit_tests GTEST MSGS ADDITIONAL_WARNINGS -Wno-missing-prototypes)
  add_dependencies(planning_common_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
