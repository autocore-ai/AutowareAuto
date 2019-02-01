# Copyright 2017-2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)
project(kalman_filter)

#dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(motion_model REQUIRED)
find_package(Eigen3 REQUIRED)

# includes
include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# just for pclint + coverity + compiler warnings
add_library(
  ${PROJECT_NAME} SHARED
  include/kalman_filter/srcf_core.hpp
  include/kalman_filter/esrcf.hpp
  src/kalman_filter.cpp
)
autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} "motion_model" "eigen3")

if(BUILD_TESTING)
  set(KALMAN_FILTER_GTEST kalman_filter_gtest)
  # run linters
  autoware_static_code_analysis()
  # gtest
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${KALMAN_FILTER_GTEST} test/src/test_kalman_filter.cpp)
  target_include_directories(${KALMAN_FILTER_GTEST} PRIVATE "test/include")
  ament_target_dependencies(${KALMAN_FILTER_GTEST}
    "motion_model")
endif()

autoware_install(HAS_INCLUDE)
# Ament Exporting
ament_export_dependencies("motion_model")
ament_package()
