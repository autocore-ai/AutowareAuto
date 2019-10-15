# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)

### Export headers
project(pure_pursuit)

## dependencies
find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/pure_pursuit/config.hpp
  src/config.cpp
  include/pure_pursuit/pure_pursuit.hpp
  src/pure_pursuit.cpp
)
autoware_set_compile_options(${PROJECT_NAME})
if(BUILD_TESTING)
  # run common linters
  # find_package(ament_lint_auto)
  # ament_lint_auto_find_test_dependencies()
  autoware_static_code_analysis()

  # gtest
  find_package(ament_cmake_gtest REQUIRED)
  set(PURE_PURSUIT_GTEST pure_pursuit_gtest)
  ament_add_gtest(${PURE_PURSUIT_GTEST}
    test/gtest_main.cpp
    test/sanity_checks.cpp
    test/test_pure_pursuit.cpp)
  target_include_directories(${PURE_PURSUIT_GTEST} PRIVATE "include")
  target_link_libraries(${PURE_PURSUIT_GTEST} ${PROJECT_NAME} osrf_testing_tools_cpp::memory_tools)
  ament_target_dependencies(${PURE_PURSUIT_GTEST} "osrf_testing_tools_cpp")
endif()

# Ament Exporting
ament_auto_package()
