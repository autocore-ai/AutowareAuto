cmake_minimum_required(VERSION 3.6)

project(controller_common)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
### Compile options
set(CMAKE_CXX_STANDARD 14)

# TODO(c.ho) this needs to be fixed
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
# Build library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/controller_common/controller_base.cpp
)
mpc_warnings(${PROJECT_NAME} MSGS)
mpc_clang_tidy(${PROJECT_NAME})

### Test
if(BUILD_TESTING)
  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # Unit test
  apex_test_tools_add_gtest(controller_common_unit_tests
    test/gtest_main.cpp
    test/behavior.cpp
    test/misc.cpp
    test/state_tracking.cpp)
  target_link_libraries(controller_common_unit_tests ${PROJECT_NAME})
  mpc_warnings(controller_common_unit_tests GTEST MSGS ADDITIONAL_WARNINGS -Wno-missing-prototypes)
  add_dependencies(controller_common_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
