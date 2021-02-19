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

cmake_minimum_required(VERSION 3.5)

project(off_map_obstacles_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(OFF_MAP_OBSTACLES_FILTER_LIB_SRC
  src/off_map_obstacles_filter.cpp
)

set(OFF_MAP_OBSTACLES_FILTER_LIB_HEADERS
  include/off_map_obstacles_filter/off_map_obstacles_filter.hpp
  include/off_map_obstacles_filter/visibility_control.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${OFF_MAP_OBSTACLES_FILTER_LIB_SRC}
  ${OFF_MAP_OBSTACLES_FILTER_LIB_HEADERS}
)
autoware_set_compile_options(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES test/test_off_map_obstacles_filter.cpp)
  set(TEST_OFF_MAP_OBSTACLES_FILTER_EXE test_off_map_obstacles_filter)
  ament_add_gtest(${TEST_OFF_MAP_OBSTACLES_FILTER_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_OFF_MAP_OBSTACLES_FILTER_EXE})
  target_link_libraries(${TEST_OFF_MAP_OBSTACLES_FILTER_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
