# Copyright 2018 the Autoware Foundation
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

project(hungarian_assigner)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()
# Disable warnings due to external dependencies (Eigen)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# build library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/hungarian_assigner.cpp
)
autoware_set_compile_options(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(HUNGARIAN_ASSIGN_GTEST hungarian_assigner_gtest)
  ament_add_gtest(${HUNGARIAN_ASSIGN_GTEST} test/src/test_hungarian_assigner.cpp)
  autoware_set_compile_options(${HUNGARIAN_ASSIGN_GTEST})
  target_include_directories(${HUNGARIAN_ASSIGN_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${HUNGARIAN_ASSIGN_GTEST} ${PROJECT_NAME})
endif()

# Ament Exporting
ament_auto_package()
