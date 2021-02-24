# Copyright 2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)
project(motion_model)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# includes
include_directories(
  SYSTEM ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/motion_model/motion_model.hpp
  include/motion_model/constant_velocity.hpp
  include/motion_model/constant_acceleration.hpp
  include/motion_model/catr_core.hpp
  include/motion_model/catr_model.hpp
  src/constant_velocity.cpp
  src/constant_acceleration.cpp
  src/catr_core.cpp
  src/catr_model.cpp
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  set(MOTION_MODEL_GTEST motion_model_gtest)
  ament_add_gtest(${MOTION_MODEL_GTEST}
    test/test_catr.cpp
    test/test_constant_acceleration.cpp
    test/test_constant_velocity.cpp
    test/test_parameter_estimator.cpp
  )
  target_include_directories(${MOTION_MODEL_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${MOTION_MODEL_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${MOTION_MODEL_GTEST})
endif()

# Ament Exporting
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS "motion_model-extras.cmake")
ament_auto_package()
