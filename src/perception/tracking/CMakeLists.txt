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

project(tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# require that dependencies from package.xml be available
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}
)

set(TRACKING_LIB_SRC
  src/data_association.cpp
  src/multi_object_tracker.cpp
  src/tracked_object.cpp
  src/projection.cpp
)

set(TRACKING_LIB_HEADERS
  include/tracking/data_association.hpp
  include/tracking/multi_object_tracker.hpp
  include/tracking/tracked_object.hpp
  include/tracking/tracker_types.hpp
  include/tracking/visibility_control.hpp
  include/tracking/roi_association.hpp
)

# generate library
ament_auto_add_library(${PROJECT_NAME} SHARED
  ${TRACKING_LIB_SRC}
  ${TRACKING_LIB_HEADERS}
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
autoware_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME} Eigen3)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  set(TEST_SOURCES
    test/test_data_association.cpp
    test/test_projection.cpp
    test/test_multi_object_tracker.cpp
    test/test_tracked_object.cpp
  )
  set(TEST_TRACKING_EXE test_multi_object_tracker)
  ament_add_gtest(${TEST_TRACKING_EXE} ${TEST_SOURCES})
  autoware_set_compile_options(${TEST_TRACKING_EXE})
  target_link_libraries(${TEST_TRACKING_EXE} ${PROJECT_NAME})
endif()

# ament package generation and installing
ament_auto_package()
