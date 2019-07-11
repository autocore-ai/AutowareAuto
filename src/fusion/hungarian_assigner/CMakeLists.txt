# Copyright 2018 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
cmake_minimum_required(VERSION 3.5)

project(hungarian_assigner)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
# Disable warnings due to external dependencies (Eigen)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
# build library
add_library(${PROJECT_NAME} SHARED
  include/hungarian_assigner/visibility_control.hpp
  include/hungarian_assigner/hungarian_assigner.hpp
  src/hungarian_assigner.cpp
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  # Linting
  autoware_static_code_analysis()

  # gtest
  set(HUNGARIAN_ASSIGN_GTEST hungarian_assigner_gtest)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${HUNGARIAN_ASSIGN_GTEST} test/src/test_hungarian_assigner.cpp)
  target_include_directories(${HUNGARIAN_ASSIGN_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${HUNGARIAN_ASSIGN_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${HUNGARIAN_ASSIGN_GTEST})
endif()

# Ament Exporting
autoware_install(LIBRARIES ${PROJECT_NAME} HAS_INCLUDE)
ament_package()
