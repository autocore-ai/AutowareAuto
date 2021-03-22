# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)
project(kalman_filter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# includes
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# build library
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/kalman_filter.cpp
  src/linear_motion_model.cpp
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  set(KALMAN_FILTER_GTEST kalman_filter_gtest)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  ament_add_gtest(${KALMAN_FILTER_GTEST}
                  test/src/test_kalman_filter.cpp
                  test/test_variables.cpp
                  test/test_generic_state.cpp
                  test/test_linear_motion_model.cpp)
  autoware_set_compile_options(${KALMAN_FILTER_GTEST})
  target_include_directories(${KALMAN_FILTER_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${KALMAN_FILTER_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${KALMAN_FILTER_GTEST}
    "motion_model" "autoware_auto_common")
endif()

# Ament Exporting
ament_auto_package()
