# Copyright 2020 Arm Limited
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

project(object_collision_estimator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(OBJECT_COLLISION_ESTIMATOR_LIB_SRC
  src/object_collision_estimator.cpp
)

set(OBJECT_COLLISION_ESTIMATOR_LIB_HEADERS
  include/object_collision_estimator/object_collision_estimator.hpp
  include/object_collision_estimator/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${OBJECT_COLLISION_ESTIMATOR_LIB_SRC}
  ${OBJECT_COLLISION_ESTIMATOR_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_NAME "test_object_collision_estimator")
  set(TEST_SOURCES test/${TEST_NAME}.cpp)
  set(TEST_OBJECT_COLLISION_ESTIMATOR_EXE ${TEST_NAME})
  ament_add_gtest(${TEST_OBJECT_COLLISION_ESTIMATOR_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_OBJECT_COLLISION_ESTIMATOR_EXE} ${PROJECT_NAME})
endif()
set( CMAKE_VERBOSE_MAKEFILE on )
# ament package generation and installing
ament_auto_package()
