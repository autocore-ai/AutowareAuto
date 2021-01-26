# Copyright 2020 The Autoware Foundation
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

project(trajectory_smoother)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(TRAJECTORY_SMOOTHER_LIB_SRC
  src/trajectory_smoother.cpp
)

set(TRAJECTORY_SMOOTHER_LIB_HEADERS
  include/trajectory_smoother/trajectory_smoother.hpp
  include/trajectory_smoother/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${TRAJECTORY_SMOOTHER_LIB_SRC}
  ${TRAJECTORY_SMOOTHER_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/gtest_main.cpp test/test_trajectory_smoother.cpp)
  set(TEST_TRAJECTORY_SMOOTHER_EXE test_trajectory_smoother)
  ament_add_gtest(${TEST_TRAJECTORY_SMOOTHER_EXE} ${TEST_SOURCES})
  target_link_libraries(${TEST_TRAJECTORY_SMOOTHER_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
