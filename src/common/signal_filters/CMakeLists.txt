cmake_minimum_required(VERSION 3.5)
project(signal_filters)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(signal_filter src/signal_filter.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(signal_filter_test test/sanity_check.cpp test/gtest_main.cpp)
  target_include_directories(signal_filter_test PRIVATE include)
endif()

ament_auto_package()
