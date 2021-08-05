# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.5)
project(motion_model)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# build library
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/linear_motion_model.cpp
  src/differential_drive_motion_model.cpp)
autoware_set_compile_options(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  set(MOTION_MODEL_GTEST motion_model_gtest)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  ament_add_gtest(${MOTION_MODEL_GTEST}
                  test/test_differential_motion_model.cpp
                  test/test_linear_motion_model.cpp
                  test/test_stationary_motion_model.cpp)
  autoware_set_compile_options(${MOTION_MODEL_GTEST})
  target_include_directories(${MOTION_MODEL_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${MOTION_MODEL_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${MOTION_MODEL_GTEST} "autoware_auto_common")
endif()

# Ament Exporting
ament_export_include_directories(${EIGEN3_INCLUDE_DIR})
ament_auto_package()
