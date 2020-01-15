cmake_minimum_required(VERSION 3.6)

project(recordreplay_planner)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
### Compile options
set(CMAKE_CXX_STANDARD 14)

# Build library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/recordreplay_planner/recordreplay_planner.cpp
)
string(CONCAT checks
  "-misc-macro-parentheses," # Generally good, but doesn't work for default method stuff
  "-clang-diagnostic-padded," # Good to know, but not helpful
  "-fuchsia-default-arguments," # Lazy
  "-modernize-use-using," # Acado from here on down
  "-readability-avoid-const-params-in-decls,"
  "-modernize-deprecated-headers,"
  "-hicpp-deprecated-headers,"
  "-cppcoreguidelines-pro-bounds-constant-array-index") # Will bring back #8

### Test
if(BUILD_TESTING)
  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ## Unit test
  ament_add_gtest(recordreplay_planner_unit_tests
    test/gtest_main.cpp
    test/sanity_checks.cpp)
  target_link_libraries(recordreplay_planner_unit_tests ${PROJECT_NAME})
  target_include_directories(recordreplay_planner_unit_tests PRIVATE "include")
endif()

ament_auto_package()
