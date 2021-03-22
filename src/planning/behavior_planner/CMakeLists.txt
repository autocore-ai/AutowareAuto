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

project(behavior_planner)

# dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(BEHAVIOR_PLANNER_LIB_SRC
  src/behavior_planner.cpp
  src/trajectory_manager.cpp
)

set(BEHAVIOR_PLANNER_LIB_HEADERS
  include/behavior_planner/behavior_planner.hpp
  include/behavior_planner/trajectory_manager.hpp
  include/behavior_planner/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${BEHAVIOR_PLANNER_LIB_SRC}
  ${BEHAVIOR_PLANNER_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_behavior_planner.cpp)
  set(TEST_BEHAVIOR_PLANNER_EXE test_behavior_planner)
  ament_add_gtest(${TEST_BEHAVIOR_PLANNER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_BEHAVIOR_PLANNER_EXE})
  target_link_libraries(${TEST_BEHAVIOR_PLANNER_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
