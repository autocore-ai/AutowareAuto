# Copyright 2018 Apex.AI, Inc.
# All rights reserved.
cmake_minimum_required(VERSION 3.5)
project(motion_model)

## dependencies
find_package(autoware_auto_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# includes
include_directories(include)
include_directories(
  SYSTEM ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME} SHARED
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
ament_target_dependencies(${PROJECT_NAME})
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # run linters
  autoware_static_code_analysis()

  # gtest
  set(MOTION_MODEL_GTEST motion_model_gtest)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${MOTION_MODEL_GTEST} test/src/test_motion_models.cpp)
  target_include_directories(${MOTION_MODEL_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${MOTION_MODEL_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${MOTION_MODEL_GTEST})
endif()

autoware_install(LIBRARIES ${PROJECT_NAME} HAS_INCLUDE)
# Ament Exporting
ament_package()
